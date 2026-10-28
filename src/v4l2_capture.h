#ifndef _V4L2_CAPTURE_H_
#define _V4L2_CAPTURE_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  int fd;
  void *buffer[4];       /* mmap'd buffers */
  int buffer_size[4];
  int n_buffers;
  int width;
  int height;
  int stride;            /* bytes per line (for MJPG this is ~width or 0) */
  int frame_size;        /* max frame buffer size for mmap */
} v4l2_ctx_t;

/* Open /dev/videoXX and mmap 4 buffers. Returns 0 on success. */
int v4l2_open(v4l2_ctx_t *ctx, const char *device, int width, int height);

/* Mark whether the open device is V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE.
 * Called by v4l2_open based on QUERYCAP results. */
void v4l2_set_mplane(int m);

/* Dequeue one frame. Returns pointer to mmap'd buffer, or NULL on error.
 * frame_index (optional) gets the absolute frame counter.
 * frame_size (optional) gets the actual bytes used (important for MJPG). */
void *v4l2_grab_frame(v4l2_ctx_t *ctx, int *frame_index, int *frame_size);

/* Requeue a buffer after you're done with its data (after drawing/detecting). */
void v4l2_release_frame(v4l2_ctx_t *ctx);

/* Streaming off + unmap + close. */
void v4l2_close(v4l2_ctx_t *ctx);

#ifdef __cplusplus
}
#endif

#endif /* _V4L2_CAPTURE_H_ */
