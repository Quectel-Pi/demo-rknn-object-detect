#include <stdio.h>
#include <sys/time.h>

#include "mpp_decoder.h"
#include <unistd.h>
#include <pthread.h>
#include <sys/syscall.h>

#define LOGD printf
// #define LOGD

static unsigned long GetCurrentTimeMS() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec*1000+tv.tv_usec/1000;
}

MppDecoder::MppDecoder()
{

}

MppDecoder::~MppDecoder() {
    if (loop_data.packet) {
        mpp_packet_deinit(&loop_data.packet);
        loop_data.packet = NULL;
    }
    if (frame) {
        mpp_frame_deinit(&frame);
        frame = NULL;
    }
    if (mpp_ctx) {
        mpp_destroy(mpp_ctx);
        mpp_ctx = NULL;
    }

    if (loop_data.frm_grp) {
        mpp_buffer_group_put(loop_data.frm_grp);
        loop_data.frm_grp = NULL;
    }
}

int MppDecoder::Init(int video_type, int fps, void* userdata)
{
    MPP_RET ret         = MPP_OK;
    this->userdata = userdata;
    this->fps = fps;
    this->last_frame_time_ms = 0;
    if(video_type == 264) {
        mpp_type  = MPP_VIDEO_CodingAVC;
    } else if (video_type == 265) {
        mpp_type  =MPP_VIDEO_CodingHEVC;
    } else if (video_type == (int)MPP_VIDEO_CodingMJPEG) {
        mpp_type  = MPP_VIDEO_CodingMJPEG;
        /* JPEG 也需要 split_parse=1（和 mpi_dec_test 默认一致，line 426）。
         * 之前误设为 0 导致 MPP 无法解析 JPEG 帧。 */
        need_split = 1;
    } else {
        LOGD("unsupport video_type %d", video_type);
        return -1;
    }
    LOGD("mpi_dec_test start ");
    memset(&loop_data, 0, sizeof(loop_data));
    LOGD("mpi_dec_test decoder test start mpp_type %d ", mpp_type);

    MppDecCfg cfg       = NULL;

    MppCtx mpp_ctx          = NULL;
    ret = mpp_create(&mpp_ctx, &mpp_mpi);
    if (MPP_OK != ret) {
        LOGD("mpp_create failed ");
        return 0;
    }

    ret = mpp_init(mpp_ctx, MPP_CTX_DEC, mpp_type);
    if (ret) {
        LOGD("%p mpp_init failed \n", mpp_ctx);
        return -1;
    }

    /* For JPEG decoding, force output format to NV12 (MPP_FMT_YUV420SP).
     * MPP JPEG decoder defaults to YUV422 YUYV output; without this the
     * downstream callback (which assumes NV12) gets the wrong format.
     * Reference: external/mpp/test/mpi_dec_test.c:382-388 */
    if (mpp_type == MPP_VIDEO_CodingMJPEG) {
        MppFrameFormat out_fmt = MPP_FMT_YUV420SP;
        ret = mpp_mpi->control(mpp_ctx, MPP_DEC_SET_OUTPUT_FORMAT, &out_fmt);
        if (ret) {
            LOGD("%p failed to set JPEG output format NV12 ret %d \n", mpp_ctx, ret);
            return -1;
        }
        LOGD("%p JPEG decoder output format set to NV12 \n", mpp_ctx);
    }

    mpp_dec_cfg_init(&cfg);

    /* get default config from decoder context */
    ret = mpp_mpi->control(mpp_ctx, MPP_DEC_GET_CFG, cfg);
    if (ret) {
        LOGD("%p failed to get decoder cfg ret %d ", mpp_ctx, ret);
        return -1;
    }

    /*
     * split_parse is to enable mpp internal frame spliter when the input
     * packet is not aplited into frames.
     */
    ret = mpp_dec_cfg_set_u32(cfg, "base:split_parse", need_split);
    if (ret) {
        LOGD("%p failed to set split_parse ret %d ", mpp_ctx, ret);
        return -1;
    }

    ret = mpp_mpi->control(mpp_ctx, MPP_DEC_SET_CFG, cfg);
    if (ret) {
        LOGD("%p failed to set cfg %p ret %d ", mpp_ctx, cfg, ret);
        return -1;
    }

    mpp_dec_cfg_deinit(cfg);

    /* For JPEG decoding, pre-allocate an output MppFrame AND its backing
     * buffer, passed via KEY_OUTPUT_FRAME metadata on each packet. Without
     * this, MPP JPEG decoder never produces output frames.
     * The buffer is allocated generously (1920x1088 NV12 ~= 3MB); MPP will
     * adjust after parsing the JPEG header. Reference: mpi_dec_test.c
     * dec_advanced() lines 292-295, 467-498. */
    if (mpp_type == MPP_VIDEO_CodingMJPEG) {
        MppBufferGroup frm_grp = NULL;
        ret = mpp_buffer_group_get_internal(&frm_grp, MPP_BUFFER_TYPE_DRM);
        if (ret) {
            LOGD("%p JPEG: get mpp buffer group failed ret %d \n", mpp_ctx, ret);
            return -1;
        }
        /* 精确按 1280x720 NV12 aligned(16) 分配:
         * hor_stride=1280(已对齐), ver_stride=720→736(对齐16)
         * NV12 = 1280*736*3/2 = 1406976 */
        size_t jpeg_buf_size = 1280 * 736 * 3 / 2;
        ret = mpp_buffer_group_limit_config(frm_grp, jpeg_buf_size, 4);
        if (ret) {
            LOGD("%p JPEG: limit buffer group failed ret %d \n", mpp_ctx, ret);
            return -1;
        }
        MppBuffer frm_buf = NULL;
        ret = mpp_buffer_get(frm_grp, &frm_buf, jpeg_buf_size);
        if (ret) {
            LOGD("%p JPEG: mpp_buffer_get failed ret %d \n", mpp_ctx, ret);
            return -1;
        }
        MppFrame jpeg_frame = NULL;
        ret = mpp_frame_init(&jpeg_frame);
        if (ret) {
            LOGD("%p JPEG: mpp_frame_init failed ret %d \n", mpp_ctx, ret);
            return -1;
        }
        /* 设 frame 的格式为 NV12 output + 分辨率，让 MPP 知道输出规格 */
        mpp_frame_set_fmt(jpeg_frame, MPP_FMT_YUV420SP);
        mpp_frame_set_buffer(jpeg_frame, frm_buf);
        jpeg_out_frame = jpeg_frame;
        LOGD("%p JPEG: pre-allocated frame + buffer (buf_size=%zu) \n",
             mpp_ctx, jpeg_buf_size);
    }

    loop_data.ctx            = mpp_ctx;
    loop_data.mpi            = mpp_mpi;
    loop_data.eos            = 0;
    loop_data.packet_size    = packet_size;
    loop_data.frame          = 0;
    loop_data.frame_count    = 0;
    return 1;
}

int MppDecoder::Reset() {
    if (mpp_mpi != NULL) {
        mpp_mpi->reset(mpp_ctx);
    }
    return 0;
}

int MppDecoder::Decode(uint8_t* pkt_data, int pkt_size, int pkt_eos)
{
    MpiDecLoopData *data=&loop_data;
    RK_U32 pkt_done = 0;
    RK_U32 err_info = 0;
    MPP_RET ret = MPP_OK;
    MppCtx ctx  = data->ctx;
    MppApi *mpi = data->mpi;

    size_t read_size = 0;
    size_t packet_size = data->packet_size;

    LOGD("receive packet size=%d ", pkt_size);

    if (packet == NULL) {
        ret = mpp_packet_init(&packet, NULL, 0);
    }

    /* Reset eos flag from previous call. The packet object is reused across
     * Decode() calls; if a previous frame set eos, it persists and MPP treats
     * all subsequent packets as end-of-stream, refusing to decode them.
     * For JPEG per-frame mode (pkt_eos=1 every call), this caused frame=NULL
     * forever after the first frame. */
    mpp_packet_clr_eos(packet);

    ///////////////////////////////////////////////
    // ret = mpp_packet_init(&packet, frame_data, frame_size);
    mpp_packet_set_data(packet, pkt_data);
    mpp_packet_set_size(packet, pkt_size);
    mpp_packet_set_pos(packet, pkt_data);
    mpp_packet_set_length(packet, pkt_size);
    // setup eos flag
    if (pkt_eos)
        mpp_packet_set_eos(packet);
    do {

        RK_S32 times = 5;
        // send the packet first if packet is not done
        if (!pkt_done) {
            /* For JPEG advanced mode: set the pre-allocated output frame on
             * the packet metadata so MPP decodes into our buffer. Without
             * this, decode_get_frame returns frame=NULL forever.
             * Reference: mpi_dec_test.c dec_advanced() lines 292-295. */
            if (jpeg_out_frame) {
                MppMeta meta = mpp_packet_get_meta(packet);
                if (meta)
                    mpp_meta_set_frame(meta, KEY_OUTPUT_FRAME, jpeg_out_frame);
            }
            ret = mpi->decode_put_packet(ctx, packet);
            if (MPP_OK == ret) {
                pkt_done = 1;
            }
        }
        // then get all available frame and release
        do {
            RK_S32 get_frm = 0;
            RK_U32 frm_eos = 0;

            try_again:
            ret = mpi->decode_get_frame(ctx, &frame);
            if (MPP_ERR_TIMEOUT == ret) {
                if (times > 0) {
                    times--;
                    usleep(2000);
                    goto try_again;
                }
                LOGD("decode_get_frame failed too much time \n");
            }

            if (MPP_OK != ret) {
                LOGD("decode_get_frame failed ret %d \n", ret);
                break;
            }

            /* Debug: log when get_frame returns OK but frame is NULL.
             * This helps diagnose JPEG decoder not producing output. */
            if (frame == NULL) {
                LOGD("decode_get_frame: MPP_OK but frame=NULL (times=%d) \n", times);
            }

            if (frame) {
                RK_U32 hor_stride = mpp_frame_get_hor_stride(frame);
                RK_U32 ver_stride = mpp_frame_get_ver_stride(frame);
                RK_U32 hor_width = mpp_frame_get_width(frame);
                RK_U32 ver_height = mpp_frame_get_height(frame);
                RK_U32 buf_size = mpp_frame_get_buf_size(frame);
                RK_S64 pts = mpp_frame_get_pts(frame);
                RK_S64 dts = mpp_frame_get_dts(frame);

                LOGD("decoder require buffer w:h [%d:%d] stride [%d:%d] buf_size %d pts=%lld dts=%lld ",
                        hor_width, ver_height, hor_stride, ver_stride, buf_size, pts, dts);

                if (mpp_frame_get_info_change(frame)) {

                    LOGD("decode_get_frame get info changed found \n");
                    if (NULL == data->frm_grp) {
                        /* If buffer group is not set create one and limit it */
                        ret = mpp_buffer_group_get_internal(&data->frm_grp, MPP_BUFFER_TYPE_DRM);
                        if (ret) {
                            LOGD("%p get mpp buffer group failed ret %d ", ctx, ret);
                            break;
                        }

                        /* Set buffer to mpp decoder */
                        ret = mpi->control(ctx, MPP_DEC_SET_EXT_BUF_GROUP, data->frm_grp);
                        if (ret) {
                            LOGD("%p set buffer group failed ret %d ", ctx, ret);
                            break;
                        }
                    } else {
                        /* If old buffer group exist clear it */
                        ret = mpp_buffer_group_clear(data->frm_grp);
                        if (ret) {
                            LOGD("%p clear buffer group failed ret %d ", ctx, ret);
                            break;
                        }
                    }

                    /* Use limit config to limit buffer count to 24 with buf_size */
                    ret = mpp_buffer_group_limit_config(data->frm_grp, buf_size, 24);
                    if (ret) {
                        LOGD("%p limit buffer group failed ret %d ", ctx, ret);
                        break;
                    }

                    /*
                     * All buffer group config done. Set info change ready to let
                     * decoder continue decoding
                     */
                    ret = mpi->control(ctx, MPP_DEC_SET_INFO_CHANGE_READY, NULL);
                    if (ret) {
                        LOGD("%p info change ready failed ret %d \n", ctx, ret);
                        break;
                    }

                    this->last_frame_time_ms = GetCurrentTimeMS();

                    /* After info-change is handled, do NOT break — continue the
                     * inner loop to drain the decoded frame from the same packet.
                     * The old code broke here, leaving the frame stuck in the
                     * decoder and never producing output. (path A fix) */
                    mpp_frame_deinit(&frame);
                    frame = NULL;
                    continue;
                } else {
                    err_info = mpp_frame_get_errinfo(frame) | mpp_frame_get_discard(frame);
                    if (err_info) {
                        LOGD("decoder_get_frame get err info:%d discard:%d. info_change=%d w:h=%d:%d stride=%d:%d \n",
                                mpp_frame_get_errinfo(frame), mpp_frame_get_discard(frame),
                                mpp_frame_get_info_change(frame), hor_width, ver_height, hor_stride, ver_stride);
                    }
                    data->frame_count++;
                    struct timeval tv;
                    gettimeofday(&tv, NULL);
                    LOGD("get one frame %ld ",  (tv.tv_sec * 1000 + tv.tv_usec/1000));
                    // mpp_frame_get_width(frame);
                    // char *input_data =(char *) mpp_buffer_get_ptr(mpp_frame_get_buffer(frame));
                    if (callback != nullptr) {
                        MppFrameFormat format = mpp_frame_get_fmt(frame);
                        char *data_vir =(char *) mpp_buffer_get_ptr(mpp_frame_get_buffer(frame));
                        int fd = mpp_buffer_get_fd(mpp_frame_get_buffer(frame));
                        LOGD("data_vir=%p fd=%d ", data_vir, fd);
                        callback(this->userdata, hor_stride, ver_stride, hor_width, ver_height, format, fd, data_vir);
                    }
                    unsigned long cur_time_ms = GetCurrentTimeMS();
                    long time_gap = 1000/this->fps - (cur_time_ms - this->last_frame_time_ms);
                    LOGD("time_gap=%ld", time_gap);
                    if (time_gap > 0) {
                        usleep(time_gap * 1000);
                    }
                    this->last_frame_time_ms = GetCurrentTimeMS();
                }
                frm_eos = mpp_frame_get_eos(frame);

                /* JPEG 模式下 frame 可能就是 jpeg_out_frame（预分配的），
                 * 不能 deinit 它——下一帧还要用。只有非预分配帧才 deinit。 */
                if (frame != jpeg_out_frame) {
                    ret = mpp_frame_deinit(&frame);
                }
                frame = NULL;

                // if(frame_pre!=NULL)
                // {
                //     mpp_frame_deinit(&frame_pre);
                // }
                // &frame_pre=&frame;

                get_frm = 1;
            }

            // try get runtime frame memory usage
            if (data->frm_grp) {
                size_t usage = mpp_buffer_group_usage(data->frm_grp);
                if (usage > data->max_usage)
                    data->max_usage = usage;
            }

            // if last packet is send but last frame is not found continue
            if (pkt_eos && pkt_done && !frm_eos) {
                usleep(1*1000);
                continue;
            }

            if (frm_eos) {
                LOGD("found last frame ");
                break;
            }

            if (data->frame_num > 0 && data->frame_count >= data->frame_num) {
                data->eos = 1;
                break;
            }

            if (get_frm)
                continue;

            /* No frame obtained this iteration. If the packet was already
             * submitted (pkt_done) but the frame isn't ready yet, poll
             * decode_get_frame with a short sleep instead of breaking out.
             * For JPEG per-frame decoding the hardware needs a few ms after
             * decode_put_packet before the frame is available; breaking out
             * here would lose the frame entirely. Limit to 50 retries
             * (~100ms) to avoid hanging on a truly broken stream. */
            if (pkt_done && !pkt_eos) {
                static RK_S32 no_frm_poll = 0;
                no_frm_poll++;
                if (no_frm_poll < 50) {
                    usleep(2000);
                    continue;
                }
                no_frm_poll = 0;
                LOGD("decode_get_frame: gave up after 50 polls, no frame \n");
            }
            break;
        } while (1);

        if (data->frame_num > 0 && data->frame_count >= data->frame_num) {
            data->eos = 1;
            LOGD("reach max frame number %d ", data->frame_count);
            break;
        }

        if (pkt_done)
            break;

        /*
         * why sleep here:
         * mpi->decode_put_packet will failed when packet in internal queue is
         * full,waiting the package is consumed .Usually hardware decode one
         * frame which resolution is 1080p needs 2 ms,so here we sleep 3ms
         * * is enough.
         */
        usleep(3*1000);
    } while (1);
    mpp_packet_deinit(&packet);

    return ret;
}

int MppDecoder::SetCallback(MppDecoderFrameCallback callback) {
    this->callback = callback;
    return 0;
}