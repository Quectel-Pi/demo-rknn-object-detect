// Copyright (c) 2023 by Rockchip Electronics Co., Ltd. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*-------------------------------------------
                Includes
-------------------------------------------*/
#include <dlfcn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>

#include "im2d.h"
#include "rga.h"
#include "RgaUtils.h"

#include "rknn_api.h"
#include "postprocess.h"

/* GStreamer real-time display */
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>

static GstElement *g_display_pipeline = NULL;
static GstElement *g_display_appsrc = NULL;
static int g_display_enabled = 1;    /* 1: show on screen, 0: off */
static int g_save_enabled = 0;       /* 1: also save to file, 0: no */
static int g_display_w = 0, g_display_h = 0;

#include "utils/mpp_decoder.h"
#include "utils/mpp_encoder.h"
#include "utils/drawing.h"

/* V4L2 USB camera + stb_image MJPG decode → RGB → RKNN */
#include "v4l2_capture.h"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include "tracker.h"
#include "event_logger.h"
#include <unordered_set>

#if defined(BUILD_VIDEO_RTSP)
#include "mk_mediakit.h"
#endif

#define OUT_VIDEO_PATH "out.h264"

typedef struct
{
  rknn_context rknn_ctx;
  rknn_input_output_num io_num;
  rknn_tensor_attr *input_attrs;
  rknn_tensor_attr *output_attrs;
  int model_channel;
  int model_width;
  int model_height;
  FILE *out_fp;
  MppDecoder *decoder;
  MppEncoder *encoder;
} rknn_app_context_t;

typedef struct
{
  int width;
  int height;
  int width_stride;
  int height_stride;
  int format;
  char *virt_addr;
  int fd;
} image_frame_t;

/*-------------------------------------------
                  Functions
-------------------------------------------*/

static void dump_tensor_attr(rknn_tensor_attr *attr)
{
  printf("  index=%d, name=%s, n_dims=%d, dims=[%d, %d, %d, %d], n_elems=%d, size=%d, fmt=%s, type=%s, qnt_type=%s, "
         "zp=%d, scale=%f\n",
         attr->index, attr->name, attr->n_dims, attr->dims[0], attr->dims[1], attr->dims[2], attr->dims[3],
         attr->n_elems, attr->size, get_format_string(attr->fmt), get_type_string(attr->type),
         get_qnt_type_string(attr->qnt_type), attr->zp, attr->scale);
}

double __get_us(struct timeval t) { return (t.tv_sec * 1000000 + t.tv_usec); }

/* ---------------- GStreamer real-time display ---------------- */
/* BGR display: 用于 USB 摄像头模式下实时显示检测后的 BGR 帧。
 * NV12 display（gst_display_init / gst_display_push_frame）用于 MppDecoder
 * 原版路径；BGR display（gst_display_bgr_init / gst_display_bgr_push）是新增的，
 * 适合 cv::Mat BGR → waylandsink 直推。 */
static GstElement *g_disp_bgr_pipeline = NULL;
static GstElement *g_disp_bgr_appsrc = NULL;
/* 显示窗口的输出尺寸（waylandsink surface 大小），0 表示用 appsrc 输入尺寸 */
static int g_disp_out_w = 0, g_disp_out_h = 0;

static int gst_display_bgr_init(int width, int height)
{
  if (g_disp_bgr_pipeline) return 0;
  gst_init(NULL, NULL);
  int out_w = g_disp_out_w > 0 ? g_disp_out_w : width;
  int out_h = g_disp_out_h > 0 ? g_disp_out_h : height;
  char pipeline[512];
  /* appsrc 输入 BGR width×height → videoscale 放大到 out_w×out_h → waylandsink
   * 这样 waylandsink 会按 out_w×out_h 创建 surface（窗口大）。
   * 若 out==in 不加 videoscale（省 CPU）。 */
  if (out_w == width && out_h == height) {
    snprintf(pipeline, sizeof(pipeline),
             "appsrc name=src is-live=true format=time "
             "caps=video/x-raw,format=BGR,width=%d,height=%d,framerate=30/1 "
             "! videoconvert ! waylandsink sync=false",
             width, height);
  } else {
    /* 注意：videoscale 后不能加 format=BGR caps（会 not-negotiated），
     * waylandsink 在这个环境只认 YUV。用 videoconvert 在 videoscale 之后
     * 做格式转换，caps filter 只指定尺寸不指定 format。 */
    snprintf(pipeline, sizeof(pipeline),
             "appsrc name=src is-live=true format=time "
             "caps=video/x-raw,format=BGR,width=%d,height=%d,framerate=30/1 "
             "! videoscale ! videoconvert ! "
             "video/x-raw,width=%d,height=%d "
             "! waylandsink sync=false",
             width, height, out_w, out_h);
  }
  g_disp_bgr_pipeline = gst_parse_launch(pipeline, NULL);
  if (!g_disp_bgr_pipeline) {
    printf("gst_bgr: pipeline create failed\n");
    return -1;
  }
  g_disp_bgr_appsrc = gst_bin_get_by_name(GST_BIN(g_disp_bgr_pipeline), "src");
  gst_element_set_state(g_disp_bgr_pipeline, GST_STATE_PLAYING);
  printf("gst_bgr: pipeline created %dx%d\n", width, height);
  return 0;
}

static int gst_display_bgr_push(void *bgr_data, int width, int height)
{
  if (!g_disp_bgr_pipeline || !g_disp_bgr_appsrc) return 0;
  int buf_size = width * height * 3;
  GstBuffer *buf = gst_buffer_new_allocate(NULL, buf_size, NULL);
  GstMapInfo map;
  gst_buffer_map(buf, &map, GST_MAP_WRITE);
  memcpy(map.data, bgr_data, buf_size);
  gst_buffer_unmap(buf, &map);

  static guint64 fc = 0;
  const guint64 FN = 30, FD = 1;
  GST_BUFFER_PTS(buf) = gst_util_uint64_scale(fc, GST_SECOND * FD, FN);
  GST_BUFFER_DTS(buf) = GST_BUFFER_PTS(buf);
  GST_BUFFER_DURATION(buf) = gst_util_uint64_scale(GST_SECOND * FD, 1, FN);
  fc++;

  GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(g_disp_bgr_appsrc), buf);
  if (ret != GST_FLOW_OK) {
    return -1;
  }
  return 0;
}

static void gst_display_bgr_deinit(void)
{
  if (g_disp_bgr_pipeline) {
    gst_element_set_state(g_disp_bgr_pipeline, GST_STATE_NULL);
    gst_object_unref(g_disp_bgr_pipeline);
    g_disp_bgr_pipeline = NULL;
    g_disp_bgr_appsrc = NULL;
  }
}

/* 第二个 waylandsink 窗口：统计面板（右框）。
 * 独立 pipeline，appsrc 输入 BGR，尺寸由调用方决定。 */
static GstElement *g_panel_pipeline = NULL;
static GstElement *g_panel_appsrc = NULL;

static int gst_panel_init(int width, int height)
{
  if (g_panel_pipeline) return 0;
  gst_init(NULL, NULL);
  char pipeline[512];
  snprintf(pipeline, sizeof(pipeline),
           "appsrc name=psrc is-live=true format=time "
           "caps=video/x-raw,format=BGR,width=%d,height=%d,framerate=30/1 "
           "! videoconvert ! waylandsink sync=false",
           width, height);
  g_panel_pipeline = gst_parse_launch(pipeline, NULL);
  if (!g_panel_pipeline) {
    printf("gst_panel: pipeline create failed\n");
    return -1;
  }
  g_panel_appsrc = gst_bin_get_by_name(GST_BIN(g_panel_pipeline), "psrc");
  gst_element_set_state(g_panel_pipeline, GST_STATE_PLAYING);
  printf("gst_panel: pipeline created %dx%d\n", width, height);
  return 0;
}

static int gst_panel_push(void *bgr_data, int width, int height)
{
  if (!g_panel_pipeline || !g_panel_appsrc) return 0;
  int buf_size = width * height * 3;
  GstBuffer *buf = gst_buffer_new_allocate(NULL, buf_size, NULL);
  GstMapInfo map;
  gst_buffer_map(buf, &map, GST_MAP_WRITE);
  memcpy(map.data, bgr_data, buf_size);
  gst_buffer_unmap(buf, &map);
  static guint64 fc = 0;
  GST_BUFFER_PTS(buf) = gst_util_uint64_scale(fc, GST_SECOND, 30);
  GST_BUFFER_DURATION(buf) = gst_util_uint64_scale(GST_SECOND, 1, 30);
  fc++;
  gst_app_src_push_buffer(GST_APP_SRC(g_panel_appsrc), buf);
  return 0;
}

static void gst_panel_deinit(void)
{
  if (g_panel_pipeline) {
    gst_element_set_state(g_panel_pipeline, GST_STATE_NULL);
    gst_object_unref(g_panel_pipeline);
    g_panel_pipeline = NULL;
    g_panel_appsrc = NULL;
  }
}

static int gst_display_init(int width, int height, int stride)
{
  if (!g_display_enabled) return 0;
  if (g_display_pipeline) return 0;

  gst_init(NULL, NULL);
  g_display_w = width;
  g_display_h = height;

  char pipeline[512];
  snprintf(pipeline, sizeof(pipeline),
           "appsrc name=src is-live=true format=time "
           "caps=video/x-raw,format=NV12,width=%d,height=%d,framerate=25/1 "
           "! videoconvert ! waylandsink",
           width, height);

  g_display_pipeline = gst_parse_launch(pipeline, NULL);
  if (!g_display_pipeline)
  {
    printf("gst_display: pipeline create failed\n");
    g_display_enabled = 0;
    return -1;
  }
  g_display_appsrc = gst_bin_get_by_name(GST_BIN(g_display_pipeline), "src");
  gst_element_set_state(g_display_pipeline, GST_STATE_PLAYING);
  printf("gst_display: pipeline created %dx%d (stride %d)\n", width, height, stride);
  return 0;
}

static int gst_display_push_frame(void *yuv_data, int stride, int height, int width)
{
  if (!g_display_enabled || !g_display_pipeline || !g_display_appsrc) return 0;

  /* The source frame is NV12 with horizontal stride (e.g. 1472 for 1280-wide
   * video). Pack it into a tight width x height buffer so GStreamer decodes
   * it correctly: copy Y plane row-by-row, then interleaved UV plane. */
  int buf_size = width * height * 3 / 2;
  GstBuffer *buf = gst_buffer_new_allocate(NULL, buf_size, NULL);
  GstMapInfo map;
  gst_buffer_map(buf, &map, GST_MAP_WRITE);
  unsigned char *dst = map.data;
  unsigned char *src = (unsigned char *)yuv_data;

  for (int y = 0; y < height; y++)
    memcpy(dst + y * width, src + y * stride, width);
  /* UV plane: (height/2) rows, each row has (width) bytes interleaved U,V */
  unsigned char *src_uv = src + stride * height;
  unsigned char *dst_uv = dst + width * height;
  for (int y = 0; y < height / 2; y++)
    memcpy(dst_uv + y * width, src_uv + y * stride, width);

  gst_buffer_unmap(buf, &map);

  /* Timestamp at 25fps so waylandsink renders at real video pace.
   * Without PTS the sink drains frames as fast as pushed -> late frames
   * drop / the tail of the video appears cut. */
  static guint64 frame_count = 0;
  const guint64 FPS_NUM = 25, FPS_DEN = 1;
  GST_BUFFER_PTS(buf) = gst_util_uint64_scale(frame_count, GST_SECOND * FPS_DEN, FPS_NUM);
  GST_BUFFER_DTS(buf) = GST_BUFFER_PTS(buf);
  GST_BUFFER_DURATION(buf) = gst_util_uint64_scale(GST_SECOND * FPS_DEN, 1, FPS_NUM);
  frame_count++;

  GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(g_display_appsrc), buf);
  if (ret != GST_FLOW_OK)
  {
    /* pipeline stopped, disable display */
    g_display_enabled = 0;
    return -1;
  }
  return 0;
}

static void gst_display_deinit(void)
{
  if (g_display_pipeline)
  {
    gst_element_set_state(g_display_pipeline, GST_STATE_NULL);
    gst_object_unref(g_display_pipeline);
    g_display_pipeline = NULL;
    g_display_appsrc = NULL;
  }
}
/* ---------------- end GStreamer display ---------------- */

static unsigned char *load_data(FILE *fp, size_t ofst, size_t sz)
{
  unsigned char *data;
  int ret;

  data = NULL;

  if (NULL == fp)
  {
    return NULL;
  }

  ret = fseek(fp, ofst, SEEK_SET);
  if (ret != 0)
  {
    printf("blob seek failure.\n");
    return NULL;
  }

  data = (unsigned char *)malloc(sz);
  if (data == NULL)
  {
    printf("buffer malloc failure.\n");
    return NULL;
  }
  ret = fread(data, 1, sz, fp);
  return data;
}

static unsigned char *read_file_data(const char *filename, int *model_size)
{
  FILE *fp;
  unsigned char *data;

  fp = fopen(filename, "rb");
  if (NULL == fp)
  {
    printf("Open file %s failed.\n", filename);
    return NULL;
  }

  fseek(fp, 0, SEEK_END);
  int size = ftell(fp);

  data = load_data(fp, 0, size);

  fclose(fp);

  *model_size = size;
  return data;
}

static int write_data_to_file(const char *path, char *data, unsigned int size)
{
  FILE *fp;

  fp = fopen(path, "w");
  if (fp == NULL)
  {
    printf("open error: %s", path);
    return -1;
  }

  fwrite(data, 1, size, fp);
  fflush(fp);

  fclose(fp);
  return 0;
}

static int init_model(const char *model_path, rknn_app_context_t *app_ctx)
{
  int ret;
  rknn_context ctx;

  /* Create the neural network */
  printf("Loading mode...\n");
  int model_data_size = 0;
  unsigned char *model_data = read_file_data(model_path, &model_data_size);
  if (model_data == NULL)
  {
    return -1;
  }

  ret = rknn_init(&ctx, model_data, model_data_size, 0, NULL);
  if (ret < 0)
  {
    printf("rknn_init error ret=%d\n", ret);
    return -1;
  }

  if (model_data)
  {
    free(model_data);
  }

  rknn_sdk_version version;
  ret = rknn_query(ctx, RKNN_QUERY_SDK_VERSION, &version, sizeof(rknn_sdk_version));
  if (ret < 0)
  {
    printf("rknn_query RKNN_QUERY_SDK_VERSION error ret=%d\n", ret);
    return -1;
  }
  printf("sdk version: %s driver version: %s\n", version.api_version, version.drv_version);

  ret = rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &app_ctx->io_num, sizeof(rknn_input_output_num));
  if (ret < 0)
  {
    printf("rknn_query RKNN_QUERY_IN_OUT_NUM error ret=%d\n", ret);
    return -1;
  }
  printf("model input num: %d, output num: %d\n", app_ctx->io_num.n_input, app_ctx->io_num.n_output);

  rknn_tensor_attr *input_attrs = (rknn_tensor_attr *)malloc(app_ctx->io_num.n_input * sizeof(rknn_tensor_attr));
  memset(input_attrs, 0, app_ctx->io_num.n_input * sizeof(rknn_tensor_attr));
  for (int i = 0; i < app_ctx->io_num.n_input; i++)
  {
    input_attrs[i].index = i;
    ret = rknn_query(ctx, RKNN_QUERY_INPUT_ATTR, &(input_attrs[i]), sizeof(rknn_tensor_attr));
    if (ret < 0)
    {
      printf("rknn_query RKNN_QUERY_INPUT_ATTR error ret=%d\n", ret);
      return -1;
    }
    dump_tensor_attr(&(input_attrs[i]));
  }

  rknn_tensor_attr *output_attrs = (rknn_tensor_attr *)malloc(app_ctx->io_num.n_output * sizeof(rknn_tensor_attr));
  memset(output_attrs, 0, app_ctx->io_num.n_output * sizeof(rknn_tensor_attr));
  for (int i = 0; i < app_ctx->io_num.n_output; i++)
  {
    output_attrs[i].index = i;
    ret = rknn_query(ctx, RKNN_QUERY_OUTPUT_ATTR, &(output_attrs[i]), sizeof(rknn_tensor_attr));
    if (ret < 0)
    {
      printf("rknn_query RKNN_QUERY_OUTPUT_ATTR error ret=%d\n", ret);
      return -1;
    }
    dump_tensor_attr(&(output_attrs[i]));
  }

  app_ctx->input_attrs = input_attrs;
  app_ctx->output_attrs = output_attrs;
  app_ctx->rknn_ctx = ctx;

  if (input_attrs[0].fmt == RKNN_TENSOR_NCHW)
  {
    printf("model is NCHW input fmt\n");
    app_ctx->model_channel = input_attrs[0].dims[1];
    app_ctx->model_height = input_attrs[0].dims[2];
    app_ctx->model_width = input_attrs[0].dims[3];
  }
  else
  {
    printf("model is NHWC input fmt\n");
    app_ctx->model_height = input_attrs[0].dims[1];
    app_ctx->model_width = input_attrs[0].dims[2];
    app_ctx->model_channel = input_attrs[0].dims[3];
  }
  printf("model input height=%d, width=%d, channel=%d\n", app_ctx->model_height, app_ctx->model_width, app_ctx->model_channel);

  return 0;
}

static int release_model(rknn_app_context_t *app_ctx)
{
  if (app_ctx->rknn_ctx != 0)
  {
    rknn_destroy(app_ctx->rknn_ctx);
  }
  free(app_ctx->input_attrs);
  free(app_ctx->output_attrs);
  deinitPostProcess();
  return 0;
}

static int inference_model(rknn_app_context_t *app_ctx, image_frame_t *img, detect_result_group_t *detect_result)
{
  int ret;
  rknn_context ctx = app_ctx->rknn_ctx;
  int model_width = app_ctx->model_width;
  int model_height = app_ctx->model_height;
  int model_channel = app_ctx->model_channel;

  struct timeval start_time, stop_time;
  const float nms_threshold = NMS_THRESH;
  const float box_conf_threshold = BOX_THRESH;
  // You may not need resize when src resulotion equals to dst resulotion
  /* resize_buf is static — RGA imresize is async and may still be
   * DMA-reading the buffer when we return. Freeing it per-frame causes
   * use-after-free / double-free / heap corruption. */
  static void *resize_buf = NULL;
  static int resize_buf_size = 0;
  int need = model_width * model_height * model_channel;
  if (resize_buf_size < need) {
    free(resize_buf);
    resize_buf = malloc(need);
    resize_buf_size = need;
  }
  // init rga context
  rga_buffer_t src;
  rga_buffer_t dst;
  im_rect src_rect;
  im_rect dst_rect;
  memset(&src_rect, 0, sizeof(src_rect));
  memset(&dst_rect, 0, sizeof(dst_rect));
  memset(&src, 0, sizeof(src));
  memset(&dst, 0, sizeof(dst));

  printf("input image %dx%d stride %dx%d format=%d\n", img->width, img->height, img->width_stride, img->height_stride, img->format);

  float scale_w = (float)model_width / img->width;
  float scale_h = (float)model_height / img->height;

  rknn_input inputs[1];
  memset(inputs, 0, sizeof(inputs));
  inputs[0].index = 0;
  inputs[0].type = RKNN_TENSOR_UINT8;
  inputs[0].size = model_width * model_height * model_channel;
  inputs[0].fmt = RKNN_TENSOR_NHWC;
  inputs[0].pass_through = 0;

  printf("resize with RGA!\n");
  memset(resize_buf, 0, need);

  src = wrapbuffer_virtualaddr((void *)img->virt_addr, img->width, img->height, img->format, img->width_stride, img->height_stride);
  dst = wrapbuffer_virtualaddr((void *)resize_buf, model_width, model_height, RK_FORMAT_RGB_888);
  ret = imcheck(src, dst, src_rect, dst_rect);
  if (IM_STATUS_NOERROR != ret)
  {
    printf("%d, check error! %s", __LINE__, imStrError((IM_STATUS)ret));
    return -1;
  }
  IM_STATUS STATUS = imresize(src, dst);

  inputs[0].buf = resize_buf;

  gettimeofday(&start_time, NULL);
  rknn_inputs_set(ctx, app_ctx->io_num.n_input, inputs);

  rknn_output outputs[app_ctx->io_num.n_output];
  memset(outputs, 0, sizeof(outputs));
  for (int i = 0; i < app_ctx->io_num.n_output; i++)
  {
    outputs[i].index = i;
    outputs[i].want_float = 0;
  }

  ret = rknn_run(ctx, NULL);
  ret = rknn_outputs_get(ctx, app_ctx->io_num.n_output, outputs, NULL);
  gettimeofday(&stop_time, NULL);
  printf("once run use %f ms\n", (__get_us(stop_time) - __get_us(start_time)) / 1000);

  printf("post process config: box_conf_threshold = %.2f, nms_threshold = %.2f\n", box_conf_threshold, nms_threshold);

  std::vector<float> out_scales;
  std::vector<int32_t> out_zps;
  for (int i = 0; i < app_ctx->io_num.n_output; ++i)
  {
    out_scales.push_back(app_ctx->output_attrs[i].scale);
    out_zps.push_back(app_ctx->output_attrs[i].zp);
  }
  BOX_RECT pads;
  memset(&pads, 0, sizeof(BOX_RECT));

  post_process((int8_t *)outputs[0].buf, (int8_t *)outputs[1].buf, (int8_t *)outputs[2].buf, model_height, model_width,
               box_conf_threshold, nms_threshold, pads, scale_w, scale_h, out_zps, out_scales, detect_result);
  ret = rknn_outputs_release(ctx, app_ctx->io_num.n_output, outputs);

  /* Note: resize_buf is static — do NOT free. RGA imresize is async and may
   * still be reading it. Buffer persists to next frame (same size). */
  return 0;
}

void mpp_decoder_frame_callback(void *userdata, int width_stride, int height_stride, int width, int height, int format, int fd, void *data)
{

  rknn_app_context_t *ctx = (rknn_app_context_t *)userdata;

  int ret = 0;
  static int frame_index = 0;
  frame_index++;

  void *mpp_frame = NULL;
  int mpp_frame_fd = 0;
  void *mpp_frame_addr = NULL;
  int enc_data_size;

  rga_buffer_t origin;
  rga_buffer_t src;

  if (ctx->encoder == NULL)
  {
    MppEncoder *mpp_encoder = new MppEncoder();
    MppEncoderParams enc_params;
    memset(&enc_params, 0, sizeof(MppEncoderParams));
    enc_params.width = width;
    enc_params.height = height;
    enc_params.hor_stride = width_stride;
    enc_params.ver_stride = height_stride;
    enc_params.fmt = MPP_FMT_YUV420SP;
    // enc_params.type = MPP_VIDEO_CodingHEVC;
    // Note: rk3562只能支持h264格式的视频流
    enc_params.type = MPP_VIDEO_CodingAVC;
    mpp_encoder->Init(enc_params, NULL);

    ctx->encoder = mpp_encoder;
  }

  int enc_buf_size = ctx->encoder->GetFrameSize();
  char *enc_data = (char *)malloc(enc_buf_size);

  image_frame_t img;
  img.width = width;
  img.height = height;
  img.width_stride = width_stride;
  img.height_stride = height_stride;
  img.fd = fd;
  img.virt_addr = (char *)data;
  img.format = RK_FORMAT_YCbCr_420_SP;
  detect_result_group_t detect_result;
  memset(&detect_result, 0, sizeof(detect_result_group_t));

  ret = inference_model(ctx, &img, &detect_result);
  if (ret != 0)
  {
    printf("inference model fail\n");
    goto RET;
  }

  mpp_frame = ctx->encoder->GetInputFrameBuffer();
  mpp_frame_fd = ctx->encoder->GetInputFrameBufferFd(mpp_frame);
  mpp_frame_addr = ctx->encoder->GetInputFrameBufferAddr(mpp_frame);

  // Copy To another buffer avoid to modify mpp decoder buffer
  origin = wrapbuffer_fd(fd, width, height, RK_FORMAT_YCbCr_420_SP, width_stride, height_stride);
  src = wrapbuffer_fd(mpp_frame_fd, width, height, RK_FORMAT_YCbCr_420_SP, width_stride, height_stride);
  imcopy(origin, src);

  // Draw objects
  for (int i = 0; i < detect_result.count; i++)
  {
      detect_result_t *det_result = &(detect_result.results[i]);
      printf("%s @ (%d %d %d %d) %f\n", det_result->name, det_result->box.left, det_result->box.top,
             det_result->box.right, det_result->box.bottom, det_result->prop);
      int x1 = det_result->box.left;
      int y1 = det_result->box.top;
      int x2 = det_result->box.right;
      int y2 = det_result->box.bottom;
      draw_rectangle_yuv420sp((unsigned char *)mpp_frame_addr, width_stride, height_stride, x1, y1, x2 - x1 + 1, y2 - y1 + 1, 0x00FF0000, 4);
      /* Draw English label above the box (white text on black box) */
      if (det_result->name[0] != '\0')
      {
          char label[OBJ_NAME_MAX_SIZE + 16];
          snprintf(label, sizeof(label), "%s", det_result->name);
          draw_text_yuv420sp((unsigned char *)mpp_frame_addr, width_stride, height_stride,
                             x1, (y1 > 14) ? y1 - 12 : 0, label, 2);
      }
  }

  // Real-time display: push the drawn frame to waylandsink
  if (g_display_enabled)
  {
    if (!g_display_pipeline)
    {
      gst_display_init(width, height, width_stride);
    }
    gst_display_push_frame(mpp_frame_addr, width_stride, height_stride, width);
  }

  // Encode to file (only if --save)
  if (g_save_enabled)
  {
    // Write header on first frame
    if (frame_index == 1)
    {
      enc_data_size = ctx->encoder->GetHeader(enc_data, enc_buf_size);
      fwrite(enc_data, 1, enc_data_size, ctx->out_fp);
    }
    memset(enc_data, 0, enc_buf_size);
    enc_data_size = ctx->encoder->Encode(mpp_frame, enc_data, enc_buf_size);
    fwrite(enc_data, 1, enc_data_size, ctx->out_fp);
  }

RET:
  if (enc_data != nullptr)
  {
    free(enc_data);
  }
}

int process_video_file(rknn_app_context_t *ctx, const char *path)
{
  int video_size;
  char *video_data = (char *)read_file_data(path, &video_size);
  char *video_data_end = video_data + video_size;
  printf("read video size=%d\n", video_size);

  const int SIZE = 8192;
  char *video_data_ptr = video_data;

  do
  {
    int pkt_eos = 0;
    int size = SIZE;
    if (video_data_ptr + size >= video_data_end)
    {
      pkt_eos = 1;
      size = video_data_end - video_data_ptr;
    }

    ctx->decoder->Decode((uint8_t *)video_data_ptr, size, pkt_eos);

    video_data_ptr += size;

    if (video_data_ptr >= video_data_end)
    {
      printf("reset decoder\n");
      break;
    }

    // LOGD("video_data_ptr=%p video_data_end=%p", video_data_ptr, video_data_end);
    // usleep(10*1000);
  } while (1);

  return 0;
}

/* ===================== V4L2 USB camera (MJPG → stb_image → RKNN) ===================== *
 * 链路（你确认的数据流）：
 *   V4L2 采集 MJPG (/dev/videoN) → stb_image 解码为 RGB uint8
 *   → cv::resize letterbox 640×640
 *   → inference_model (RGA → rknn_inputs_set → rknn_run → post_process)
 *   → 画检测框 → imwrite 保存关键帧
 *
 * 完全不经过 MppDecoder。stb_image（单头文件）负责 MJPG → RGB 解码。
 */

/* 解码 MJPG 帧为 RGB888。返回 cv::Mat。失败返回空 Mat。
 * stb_image 的 stbi_load_from_memory 解码 JPEG → RGB，返回 malloc'd buffer。 */
static cv::Mat decode_mjpg_to_rgb(uint8_t *jpeg_data, int jpeg_size)
{
  int w, h, channels;
  unsigned char *img = stbi_load_from_memory(jpeg_data, jpeg_size,
                                              &w, &h, &channels, 3); /* req=3 → RGB */
  if (img == NULL) {
    fprintf(stderr, "[cam] stb_image decode fail: %s\n", stbi_failure_reason());
    return cv::Mat();
  }
  /* stb_image 返回的 buffer 是 row-major RGB，stride=w*3。
   * cv::Mat 构造时不 copy（默认 share），用 wrap + 后续 release 释放 stb buffer。 */
  cv::Mat rgb(h, w, CV_8UC3, img);
  cv::Mat rgb_copy = rgb.clone();   /* deep copy，stb buffer 可立即释放 */
  stbi_image_free(img);
  return rgb_copy;
}

int process_v4l2_camera(rknn_app_context_t *ctx, const char *device)
{
  v4l2_ctx_t cam;
  if (v4l2_open(&cam, device, 1280, 720) < 0) {
    fprintf(stderr, "[cam] v4l2_open failed for %s\n", device);
    return -1;
  }
  if (cam.n_buffers == 0) {
    fprintf(stderr, "[cam] no mmap buffers\n");
    v4l2_close(&cam);
    return -1;
  }
  fprintf(stderr, "[cam] %s %dx%d stride=%d bufsize=%d buffers=%d\n",
          device, cam.width, cam.height, cam.stride, cam.frame_size, cam.n_buffers);

  int model_w = ctx->model_width;   /* 640 */
  int model_h = ctx->model_height;  /* 640 */

  /* tracker + event logger 初始化 */
  IoUTracker tracker(0.3f, 30);
  EventLogger events(30);   /* 每 30 帧输出一次 counts */

  /* 活物集合：person + COCO 动物类（索引 0=person, 14-23 为动物类）。
   * 对应 coco_80_labels_list.txt: person / bird / cat / dog / horse /
   * sheep / cow / elephant / bear / zebra / giraffe */
  auto is_alive = [](const std::string &n) -> bool {
    static const std::unordered_set<std::string> alive = {
      "person","bird","cat","dog","horse","sheep","cow",
      "elephant","bear","zebra","giraffe"
    };
    return alive.count(n) > 0;
  };

  /* 统计面板 BGR 画布。PANEL_W=360 → combined=640+360=1000×480 单窗口。
   * 面板贴在摄像头右侧，不用全屏，不缩放摄像头。 */
  const int PANEL_W = 360;
  const int PANEL_H = cam.height;  /* 与摄像头采集高一致，480 */
  cv::Scalar PANEL_BG(30, 30, 30);    /* 深灰 BGR */

  int frame_index = 0;
  struct timeval last_t = {0, 0};
  float fps_avg = 0.0f;

  while (1) {
    int jpeg_size = 0;
    void *mjpeg_buf = v4l2_grab_frame(&cam, &frame_index, &jpeg_size);
    if (mjpeg_buf == NULL || jpeg_size <= 0) {
      usleep(10000);
      continue;
    }

    /* MJPG → RGB (libjpeg-turbo CPU 软解) */
    cv::Mat rgb = decode_mjpg_to_rgb((uint8_t *)mjpeg_buf, jpeg_size);
    v4l2_release_frame(&cam);   /* 早释放 V4L2 buffer */
    if (rgb.empty()) {
      fprintf(stderr, "[cam] jpeg decode fail frame %d\n", frame_index);
      continue;
    }
    int src_w = rgb.cols;
    int src_h = rgb.rows;

    /* letterbox 640×640, 填充 114 灰。s = 640 / max(w,h)。 */
    float s = (float)model_w / std::max(src_w, src_h);
    int new_w = (int)(src_w * s + 0.5f);
    int new_h = (int)(src_h * s + 0.5f);
    int pad_x = (model_w - new_w) / 2;
    int pad_y = (model_h - new_h) / 2;
    cv::Mat resized;
    cv::resize(rgb, resized, cv::Size(new_w, new_h), 0, 0, cv::INTER_LINEAR);
    cv::Mat letterbox(model_h, model_w, CV_8UC3, cv::Scalar(114, 114, 114));
    resized.copyTo(letterbox(cv::Rect(pad_x, pad_y, new_w, new_h)));

    /* 构造 image_frame_t，RGB 888 直接送入 RGA → rknn。
     * src/dst 同尺寸时 RGA imresize ≈ copy。 */
    image_frame_t img;
    img.width = model_w;
    img.height = model_h;
    img.width_stride = model_w;
    img.height_stride = model_h;
    img.fd = -1;
    img.virt_addr = (char *)letterbox.data;
    img.format = RK_FORMAT_RGB_888;

    detect_result_group_t detect_result;
    memset(&detect_result, 0, sizeof(detect_result));

    struct timeval t0;
    gettimeofday(&t0, NULL);
    int ret = inference_model(ctx, &img, &detect_result);
    if (ret != 0) {
      fprintf(stderr, "[cam] inference fail frame %d\n", frame_index);
      continue;
    }

    /* FPS */
    struct timeval now;
    gettimeofday(&now, NULL);
    if (last_t.tv_sec != 0) {
      float dt = (__get_us(now) - __get_us(last_t)) / 1000000.0f;
      if (dt > 0) {
        float inst = 1.0f / dt;
        fps_avg = (fps_avg == 0) ? inst : (fps_avg * 0.9f + inst * 0.1f);
      }
    }
    last_t = now;

    /* 转 Detection 喂给 tracker 做跨帧 IoU 跟踪 + 分配 track_id。
     * post_process 返回的 box 坐标基于 640×640 letterbox 空间（含 pad），
     * 需要逆变换回原始帧坐标后再画框。 */
    std::vector<Detection> dets;
    for (int i = 0; i < detect_result.count; i++) {
      detect_result_t *dr = &detect_result.results[i];
      int ox1 = (int)((dr->box.left - pad_x) / s);
      int oy1 = (int)((dr->box.top - pad_y) / s);
      int ox2 = (int)((dr->box.right - pad_x) / s);
      int oy2 = (int)((dr->box.bottom - pad_y) / s);
      ox1 = std::max(0, std::min(ox1, src_w - 1));
      oy1 = std::max(0, std::min(oy1, src_h - 1));
      ox2 = std::max(0, std::min(ox2, src_w - 1));
      oy2 = std::max(0, std::min(oy2, src_h - 1));
      Detection d;
      d.name = dr->name;
      d.left = ox1; d.top = oy1; d.right = ox2; d.bottom = oy2;
      d.score = dr->prop;
      dets.push_back(d);
    }
    std::vector<TrackBox> tracks = tracker.update(dets);

    /* 事件日志（JSON 行，stdout）— frame/appear/counts 三类事件 */
    events.log_frame(frame_index, src_w, src_h, fps_avg, tracks);

    /* 画带 track_id 的检测框到原始 rgb 帧。
     * 活物（person / 动物）→ 红框；死物 → 蓝框。
     * BGR 下：红=(0,0,255)，蓝=(255,0,0)。注意 rgb 是 RGB 顺序，
     * RGB 下 红=(255,0,0)，蓝=(0,0,255)。 */
    for (auto &t : tracks) {
      int x1 = (int)t.left, y1 = (int)t.top;
      int x2 = (int)t.right, y2 = (int)t.bottom;
      x1 = std::max(0, std::min(x1, src_w - 1));
      y1 = std::max(0, std::min(y1, src_h - 1));
      x2 = std::max(0, std::min(x2, src_w - 1));
      y2 = std::max(0, std::min(y2, src_h - 1));
      cv::Scalar color = is_alive(t.name)
                          ? cv::Scalar(255, 0, 0)    /* RGB 红 = 活物 */
                          : cv::Scalar(0, 0, 255);   /* RGB 蓝 = 死物 */
      cv::rectangle(rgb, cv::Point(x1, y1), cv::Point(x2, y2), color, 2);
      /* 框上方显示类别名（带 track_id） */
      char label[64];
      snprintf(label, sizeof(label), "%s#%d", t.name.c_str(), t.track_id);
      cv::putText(rgb, label,
                  cv::Point(x1, std::max(12, y1 - 5)),
                  cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
    }

    fprintf(stderr, "frame %d  fps %.1f  det %d  tracks %zu  src %dx%d\n",
            frame_index, fps_avg, detect_result.count, tracks.size(), src_w, src_h);

    /* 主+面板一起在下方统一显示为单窗口（左主帧+右面板）。此处不再单独推主帧。 */

    /* 统计面板（右框）：BGR 画布，绘制 FPS / 当前在线 / track list。
     * 与主帧拼成单张 combined = [主帧 640 | 面板 360] × 480，单窗口显示，
     * 面板永远在主画面右侧而不是居中。 */
    if (g_display_enabled) {
      /* 全屏 cover 模式：摄像头按等比放大到高=target_h（cover 填满高度），
       * 宽超出部分居中裁掉。面板半透明叠加在右上角，不占独立区域。
       * 这样摄像头无缩放变形，画质最好。
       * src 1280×720 → 放大到高 800 宽=1422，裁掉左右各 71 → 1280×800 */
      int target_w = (g_disp_out_w > 0) ? g_disp_out_w : 1280;
      int target_h = (g_disp_out_h > 0) ? g_disp_out_h : 800;
      int cover_w = src_w * target_h / src_h;   /* 1280*800/720=1422 */
      int crop_x = (cover_w - target_w) / 2;    /* (1422-1280)/2=71 */

      cv::Mat bgr;
      cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
      /* 摄像头放大到 cover_w × target_h，然后裁中心 target_w × target_h */
      cv::Mat cover;
      cv::resize(bgr, cover, cv::Size(cover_w, target_h), 0, 0, cv::INTER_LINEAR);
      cv::Mat combined = cover(cv::Rect(crop_x, 0, target_w, target_h)).clone();

      /* 面板：在右侧画一个半透明深色矩形区域 + 文字，叠加在摄像头上 */
      int panel_w_final = 340;
      int panel_h_final = target_h;
      cv::Mat overlay = combined.clone();
      cv::rectangle(overlay,
                    cv::Point(target_w - panel_w_final, 0),
                    cv::Point(target_w, panel_h_final),
                    PANEL_BG, -1);   /* 实心深灰矩形 */
      cv::addWeighted(overlay, 0.75, combined, 0.25, 0, combined);
      /* 在叠加区域上画文字 */
      cv::Mat panel_roi = combined(cv::Rect(target_w - panel_w_final + 10, 0,
                                           panel_w_final - 20, panel_h_final));
      int y = 40;
      char buf[128];
      snprintf(buf, sizeof(buf), "TRACKS: %zu", tracks.size());
      cv::putText(combined, buf, cv::Point(target_w - panel_w_final + 15, y),
                  cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 255), 2); y += 45;
      cv::putText(combined, "ONLINE COUNTS",
                  cv::Point(target_w - panel_w_final + 15, y),
                  cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(200, 200, 200), 2); y += 35;
      std::map<std::string,int> online;
      for (auto &t : tracks) online[t.name]++;
      for (auto &kv : online) {
        snprintf(buf, sizeof(buf), "%-14s %d", kv.first.c_str(), kv.second);
        cv::Scalar c = is_alive(kv.first)
                         ? cv::Scalar(0, 0, 255)
                         : cv::Scalar(255, 0, 0);
        cv::putText(combined, buf,
                    cv::Point(target_w - panel_w_final + 15, y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, c, 2); y += 32;
      }
      y += 15;
      cv::putText(combined, "TRACK LIST",
                  cv::Point(target_w - panel_w_final + 15, y),
                  cv::FONT_HERSHEY_SIMPLEX, 0.8,
                  cv::Scalar(200, 200, 200), 2); y += 32;
      int shown = 0;
      for (auto &t : tracks) {
        if (shown++ >= 20) break;
        snprintf(buf, sizeof(buf), "#%-3d %-12s %.2f",
                 t.track_id, t.name.c_str(), t.score);
        cv::Scalar c = is_alive(t.name) ? cv::Scalar(0, 0, 255)
                                        : cv::Scalar(255, 0, 0);
        cv::putText(combined, buf,
                    cv::Point(target_w - panel_w_final + 15, y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, c, 2); y += 28;
      }

      if (!g_disp_bgr_pipeline)
        gst_display_bgr_init(target_w, target_h);
      gst_display_bgr_push(combined.data, target_w, target_h);
    }
  }

  gst_display_bgr_deinit();
  gst_panel_deinit();
  v4l2_close(&cam);
  return 0;
}

#if defined(BUILD_VIDEO_RTSP)
void API_CALL on_track_frame_out(void *user_data, mk_frame frame)
{
  rknn_app_context_t *ctx = (rknn_app_context_t *)user_data;
  printf("on_track_frame_out ctx=%p\n", ctx);
  const char *data = mk_frame_get_data(frame);
  size_t size = mk_frame_get_data_size(frame);
  printf("decoder=%p\n", ctx->decoder);
  ctx->decoder->Decode((uint8_t *)data, size, 0);
}

void API_CALL on_mk_play_event_func(void *user_data, int err_code, const char *err_msg, mk_track tracks[],
                                    int track_count)
{
  rknn_app_context_t *ctx = (rknn_app_context_t *)user_data;
  if (err_code == 0)
  {
    // success
    printf("play success!");
    int i;
    for (i = 0; i < track_count; ++i)
    {
      if (mk_track_is_video(tracks[i]))
      {
        log_info("got video track: %s", mk_track_codec_name(tracks[i]));
        // 监听track数据回调
        mk_track_add_delegate(tracks[i], on_track_frame_out, user_data);
      }
    }
  }
  else
  {
    printf("play failed: %d %s", err_code, err_msg);
  }
}

void API_CALL on_mk_shutdown_func(void *user_data, int err_code, const char *err_msg, mk_track tracks[], int track_count)
{
  printf("play interrupted: %d %s", err_code, err_msg);
}

int process_video_rtsp(rknn_app_context_t *ctx, const char *url)
{
  mk_config config;
  memset(&config, 0, sizeof(mk_config));
  config.log_mask = LOG_CONSOLE;
  mk_env_init(&config);
  mk_player player = mk_player_create();
  mk_player_set_on_result(player, on_mk_play_event_func, ctx);
  mk_player_set_on_shutdown(player, on_mk_shutdown_func, ctx);
  mk_player_play(player, url);

  printf("enter any key to exit\n");
  getchar();

  if (player)
  {
    mk_player_release(player);
  }
  return 0;
}
#endif

/*-------------------------------------------
                  Main Functions
-------------------------------------------*/
int main(int argc, char **argv)
{
  int status = 0;
  int ret;

  /* V4L2 live camera mode (USB MJPG): usage:
   *   rknn_yolov5_video_demo <model.rknn> --v4l2 /dev/video73 [--noshow]
   * Feeds USB camera MJPG frames to MppDecoder(JPEG) → inference + display. */
  if (argc >= 3 && strcmp(argv[2], "--v4l2") == 0)
  {
    const char *model_name = argv[1];
    const char *device = "/dev/video73";
    for (int i = 3; i < argc; i++) {
      if (strncmp(argv[i], "/dev/", 5) == 0) device = argv[i];
      else if (strcmp(argv[i], "--noshow") == 0) g_display_enabled = 0;
      else if (strcmp(argv[i], "--display") == 0) g_display_enabled = 1;
      else if (strcmp(argv[i], "--disp-size") == 0 && i + 1 < argc) {
        /* --disp-size 960x540 — 设 waylandsink 窗口输出尺寸 */
        i++;
        if (sscanf(argv[i], "%dx%d", &g_disp_out_w, &g_disp_out_h) == 2) {
          printf("display size: %dx%d\n", g_disp_out_w, g_disp_out_h);
        } else {
          printf("invalid --disp-size format (use WxH, e.g. 960x540)\n");
          g_disp_out_w = g_disp_out_h = 0;
        }
      }
    }
    printf("v4l2 mode: model=%s device=%s display=%d disp=%dx%d\n",
           model_name, device, g_display_enabled, g_disp_out_w, g_disp_out_h);

    rknn_app_context_t app_ctx;
    memset(&app_ctx, 0, sizeof(rknn_app_context_t));
    ret = init_model(model_name, &app_ctx);
    if (ret != 0) { printf("init model fail\n"); return -1; }

    ret = process_v4l2_camera(&app_ctx, device);

    gst_display_deinit();
    if (app_ctx.decoder != nullptr) { delete app_ctx.decoder; app_ctx.decoder = nullptr; }
    release_model(&app_ctx);
    return ret;
  }

  if (argc < 4)
  {
    printf("Usage: %s <rknn_model> <video_path> <video_type 264/265> [--display] [--save] [--noshow]\n", argv[0]);
    printf("  %s <rknn_model> --v4l2 /dev/videoNN [--display] [--noshow] [--disp-size WxH]\n", argv[0]);
    printf("  --display  show real-time detection on screen (default)\n");
    printf("  --noshow   disable screen display\n");
    printf("  --save     also save detection video to %s\n", OUT_VIDEO_PATH);
    return -1;
  }

  char *model_name = (char *)argv[1];
  char *video_name = argv[2];
  int video_type = atoi(argv[3]);

  /* parse optional flags */
  for (int i = 4; i < argc; i++)
  {
    if (strcmp(argv[i], "--noshow") == 0) g_display_enabled = 0;
    else if (strcmp(argv[i], "--display") == 0) g_display_enabled = 1;
    else if (strcmp(argv[i], "--save") == 0) g_save_enabled = 1;
  }

  rknn_app_context_t app_ctx;
  memset(&app_ctx, 0, sizeof(rknn_app_context_t));

  ret = init_model(model_name, &app_ctx);
  if (ret != 0)
  {
    printf("init model fail\n");
    return -1;
  }

  if (app_ctx.decoder == NULL)
  {
    MppDecoder *decoder = new MppDecoder();
    decoder->Init(video_type, 30, &app_ctx);
    decoder->SetCallback(mpp_decoder_frame_callback);
    app_ctx.decoder = decoder;
  }

  if (app_ctx.out_fp == NULL && g_save_enabled)
  {
    FILE *fp = fopen(OUT_VIDEO_PATH, "w");
    if (fp == NULL)
    {
      printf("open %s error\n", OUT_VIDEO_PATH);
      return -1;
    }
    app_ctx.out_fp = fp;
  }

  printf("app_ctx=%p decoder=%p\n", &app_ctx, app_ctx.decoder);

  /* Real-time display pipeline is initialized lazily in the frame callback
   * with the actual decoder stride (width_stride), so no init here. */

  if (strncmp(video_name, "rtsp", 4) == 0)
  {
#if defined(BUILD_VIDEO_RTSP)
    process_video_rtsp(&app_ctx, video_name);
#else
    printf("rtsp no support\n");
#endif
  }
  else
  {
    process_video_file(&app_ctx, video_name);
  }

  printf("waiting finish\n");
  usleep(3 * 1000 * 1000);

  // release
  gst_display_deinit();
  if (app_ctx.out_fp != NULL)
  {
    fflush(app_ctx.out_fp);
    fclose(app_ctx.out_fp);
  }

  if (app_ctx.decoder != nullptr)
  {
    delete (app_ctx.decoder);
    app_ctx.decoder = nullptr;
  }
  if (app_ctx.encoder != nullptr)
  {
    delete (app_ctx.encoder);
    app_ctx.encoder = nullptr;
  }

  release_model(&app_ctx);

  return 0;
}
