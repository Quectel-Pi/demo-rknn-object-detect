#ifndef DRAWING_MODULE_
#define DRAWING_MODULE_

void draw_rectangle_yuv420sp(unsigned char* yuv420sp, int w, int h, int rx, int ry, int rw, int rh, unsigned int color, int thickness);

void draw_image_yuv420sp(unsigned char* yuv420sp, int w, int h, unsigned char* draw_img, int rx, int ry, int rw, int rh);

/* Draw an ASCII text label on the Y plane of a YUV420SP frame using OpenCV
 * putText (Hershey vector font). White text on a black box, scaled by
 * `scale` (font_scale passed to cv::putText). (tx,ty) is top-left of box. */
void draw_text_yuv420sp(unsigned char* yuv420sp, int w, int h, int tx, int ty, const char* text, int scale);

#endif 
