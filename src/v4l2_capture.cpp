// V4L2 MJPG capture for USB camera (UVC, /dev/video73 etc).
//
// USB cameras (UVC) output MJPG or YUYV. MJPG is the preferred format for
// real-time detection: 1280x720@30fps full-frame, ~87KB/frame (USB-friendly).
// UVC devices are single-plane (V4L2_BUF_TYPE_VIDEO_CAPTURE, not MPLANE).
//
// This file handles both single-plane (USB) and MPLANE (MIPI rkisp) devices.
// Output: compressed MJPG frame (SOI...EOI). The actual byte count per frame
// is in bytesused (returned via frame_size), NOT stride*height.
// The caller feeds the MJPG frame to MppDecoder for hardware JPEG decoding.

#include "v4l2_capture.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>

#define N_BUFFERS 4

static int xioctl(int fh, int request, void *arg)
{
  int r;
  do {
    r = ioctl(fh, request, arg);
  } while (-1 == r && EINTR == errno);
  return r;
}

int v4l2_open(v4l2_ctx_t *ctx, const char *device, int width, int height)
{
  memset(ctx, 0, sizeof(*ctx));
  ctx->fd = -1;

  ctx->fd = open(device, O_RDWR, 0);
  if (ctx->fd < 0) {
    fprintf(stderr, "v4l2: cannot open %s: %s\n", device, strerror(errno));
    return -1;
  }

  struct v4l2_capability cap;
  if (xioctl(ctx->fd, VIDIOC_QUERYCAP, &cap) < 0) {
    fprintf(stderr, "v4l2: VIDIOC_QUERYCAP failed: %s\n", strerror(errno));
    return -1;
  }

  int is_mplane = 0;
  if (cap.capabilities & V4L2_CAP_DEVICE_CAPS) {
    is_mplane = (cap.device_caps & V4L2_CAP_VIDEO_CAPTURE_MPLANE) != 0;
  } else {
    is_mplane = (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE) != 0;
  }
  if (!is_mplane && !(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
    fprintf(stderr, "v4l2: %s is not a capture device (caps=0x%08x)\n",
            device, cap.capabilities);
    return -1;
  }
  v4l2_set_mplane(is_mplane);

  enum v4l2_buf_type type;
  struct v4l2_format fmt;
  memset(&fmt, 0, sizeof(fmt));

  if (is_mplane) {
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    fmt.type = type;
    fmt.fmt.pix_mp.width = width;
    fmt.fmt.pix_mp.height = height;
    fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_MJPEG;
    fmt.fmt.pix_mp.num_planes = 1;
    fmt.fmt.pix_mp.field = V4L2_FIELD_NONE;
    if (xioctl(ctx->fd, VIDIOC_S_FMT, &fmt) < 0) {
      fprintf(stderr, "v4l2: S_FMT MPLANE MJPEG failed: %s\n", strerror(errno));
      return -1;
    }
    ctx->stride = fmt.fmt.pix_mp.plane_fmt[0].bytesperline;
    ctx->frame_size = fmt.fmt.pix_mp.plane_fmt[0].sizeimage;
    ctx->width = fmt.fmt.pix_mp.width;
    ctx->height = fmt.fmt.pix_mp.height;
  } else {
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.type = type;
    fmt.fmt.pix.width = width;
    fmt.fmt.pix.height = height;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    if (xioctl(ctx->fd, VIDIOC_S_FMT, &fmt) < 0) {
      fprintf(stderr, "v4l2: S_FMT single-plane MJPEG failed: %s\n", strerror(errno));
      return -1;
    }
    ctx->stride = fmt.fmt.pix.bytesperline;
    ctx->frame_size = fmt.fmt.pix.sizeimage;
    ctx->width = fmt.fmt.pix.width;
    ctx->height = fmt.fmt.pix.height;
  }

  /* For MJPG, stride/sizeimage may be 0 or a rough max; the real per-frame
   * size comes from bytesused at DQBUF time. Just make sure frame_size is
   * large enough for the mmap. */
  if (ctx->frame_size <= 0) ctx->frame_size = width * height * 2;
  if (ctx->stride < width) ctx->stride = width;

  uint32_t actual_fmt = 0;
  if (is_mplane) {
    actual_fmt = fmt.fmt.pix_mp.pixelformat;
  } else {
    actual_fmt = fmt.fmt.pix.pixelformat;
  }
  fprintf(stderr, "v4l2: %s requested MJPG, got 0x%08x (%c%c%c%c), %dx%d "
          "stride=%d size=%d mplane=%d\n",
          device, actual_fmt,
          (actual_fmt >> 0) & 0xff, (actual_fmt >> 8) & 0xff,
          (actual_fmt >> 16) & 0xff, (actual_fmt >> 24) & 0xff,
          ctx->width, ctx->height, ctx->stride, ctx->frame_size, is_mplane);
  if (actual_fmt != V4L2_PIX_FMT_MJPEG) {
    fprintf(stderr, "v4l2: WARNING — device did not accept MJPG.\n");
  }

  /* REQUESTBUFS + mmap N buffers */
  struct v4l2_requestbuffers req;
  memset(&req, 0, sizeof(req));
  req.count = N_BUFFERS;
  req.type = type;
  req.memory = V4L2_MEMORY_MMAP;
  if (xioctl(ctx->fd, VIDIOC_REQBUFS, &req) < 0) {
    fprintf(stderr, "v4l2: REQBUFS failed: %s\n", strerror(errno));
    return -1;
  }
  if ((int)req.count < N_BUFFERS) {
    fprintf(stderr, "v4l2: only %d buffers granted\n", req.count);
  }

  ctx->n_buffers = req.count;
  for (int i = 0; i < (int)req.count; i++) {
    struct v4l2_buffer buf;
    struct v4l2_plane planes[1];
    memset(&buf, 0, sizeof(buf));
    memset(planes, 0, sizeof(planes));
    buf.type = type;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    if (is_mplane) {
      buf.m.planes = planes;
      buf.length = 1;
    }
    if (xioctl(ctx->fd, VIDIOC_QUERYBUF, &buf) < 0) {
      fprintf(stderr, "v4l2: QUERYBUF %d failed: %s\n", i, strerror(errno));
      return -1;
    }
    int map_len;
    off_t map_off;
    if (is_mplane) {
      map_len = planes[0].length;
      map_off = planes[0].m.mem_offset;
    } else {
      map_len = buf.length;
      map_off = buf.m.offset;
    }
    ctx->buffer[i] = mmap(NULL, map_len, PROT_READ | PROT_WRITE, MAP_SHARED,
                         ctx->fd, map_off);
    if (ctx->buffer[i] == MAP_FAILED) {
      fprintf(stderr, "v4l2: mmap %d failed: %s\n", i, strerror(errno));
      return -1;
    }
    ctx->buffer_size[i] = map_len;
    if (xioctl(ctx->fd, VIDIOC_QBUF, &buf) < 0) {
      fprintf(stderr, "v4l2: initial QBUF %d failed: %s\n", i, strerror(errno));
      return -1;
    }
  }

  if (xioctl(ctx->fd, VIDIOC_STREAMON, &type) < 0) {
    fprintf(stderr, "v4l2: STREAMON failed: %s\n", strerror(errno));
    return -1;
  }
  fprintf(stderr, "v4l2: streaming started, %d buffers, type=%s\n",
          ctx->n_buffers, is_mplane ? "MPLANE" : "single-plane");
  return 0;
}

static struct v4l2_buffer g_cur_buf;
static struct v4l2_plane   g_cur_planes[1];
static int g_cur_buf_active = 0;
static int g_is_mplane = 0;

void v4l2_set_mplane(int m) { g_is_mplane = m; }

void *v4l2_grab_frame(v4l2_ctx_t *ctx, int *frame_index, int *frame_size)
{
  if (ctx->n_buffers == 0) return NULL;
  memset(&g_cur_buf, 0, sizeof(g_cur_buf));
  memset(g_cur_planes, 0, sizeof(g_cur_planes));
  g_cur_buf.type = g_is_mplane ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE
                                : V4L2_BUF_TYPE_VIDEO_CAPTURE;
  g_cur_buf.memory = V4L2_MEMORY_MMAP;
  if (g_is_mplane) {
    g_cur_buf.m.planes = g_cur_planes;
    g_cur_buf.length = 1;
  }
  if (xioctl(ctx->fd, VIDIOC_DQBUF, &g_cur_buf) < 0) {
    if (errno == EAGAIN) return NULL;
    fprintf(stderr, "v4l2: DQBUF failed: %s\n", strerror(errno));
    return NULL;
  }
  g_cur_buf_active = 1;
  if (frame_index) *frame_index = g_cur_buf.sequence;
  /* For MJPG, actual frame size is bytesused, not stride*height */
  if (frame_size) {
    if (g_is_mplane)
      *frame_size = g_cur_planes[0].bytesused;
    else
      *frame_size = g_cur_buf.bytesused;
  }
  if (g_cur_buf.index >= (uint32_t)ctx->n_buffers) {
    fprintf(stderr, "v4l2: DQBUF index out of range %d\n", g_cur_buf.index);
    return NULL;
  }
  return ctx->buffer[g_cur_buf.index];
}

void v4l2_release_frame(v4l2_ctx_t *ctx)
{
  if (!g_cur_buf_active) return;
  if (xioctl(ctx->fd, VIDIOC_QBUF, &g_cur_buf) < 0) {
    fprintf(stderr, "v4l2: re-QBUF failed: %s\n", strerror(errno));
  }
  g_cur_buf_active = 0;
}

void v4l2_close(v4l2_ctx_t *ctx)
{
  if (ctx->n_buffers > 0) {
    int type = g_is_mplane ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE
                           : V4L2_BUF_TYPE_VIDEO_CAPTURE;
    xioctl(ctx->fd, VIDIOC_STREAMOFF, &type);
    for (int i = 0; i < ctx->n_buffers; i++) {
      if (ctx->buffer[i] && ctx->buffer[i] != MAP_FAILED) {
        munmap(ctx->buffer[i], ctx->buffer_size[i]);
      }
    }
  }
  if (ctx->fd >= 0) close(ctx->fd);
  memset(ctx, 0, sizeof(*ctx));
}
