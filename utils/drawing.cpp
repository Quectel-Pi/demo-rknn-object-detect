#include "drawing.h"
#include "string.h"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"

#define max(a, b) (((a) > (b)) ? (a) : (b))
#define min(a, b) (((a) < (b)) ? (a) : (b))

void draw_rectangle_c1(unsigned char* pixels, int w, int h, int stride, int rx, int ry, int rw, int rh, unsigned int color, int thickness)
{
    const unsigned char* pen_color = (const unsigned char*)&color;

    if (thickness == -1)
    {
        // filled
        for (int y = ry; y < ry + rh; y++)
        {
            if (y < 0)
                continue;

            if (y >= h)
                break;

            unsigned char* p = pixels + stride * y;

            for (int x = rx; x < rx + rw; x++)
            {
                if (x < 0)
                    continue;

                if (x >= w)
                    break;

                p[x] = pen_color[0];
            }
        }

        return;
    }

    const int t0 = thickness / 2;
    const int t1 = thickness - t0;

    // draw top
    {
        for (int y = ry - t0; y < ry + t1; y++)
        {
            if (y < 0)
                continue;

            if (y >= h)
                break;

            unsigned char* p = pixels + stride * y;

            for (int x = rx - t0; x < rx + rw + t1; x++)
            {
                if (x < 0)
                    continue;

                if (x >= w)
                    break;

                p[x] = pen_color[0];
            }
        }
    }

    // draw bottom
    {
        for (int y = ry + rh - t0; y < ry + rh + t1; y++)
        {
            if (y < 0)
                continue;

            if (y >= h)
                break;

            unsigned char* p = pixels + stride * y;

            for (int x = rx - t0; x < rx + rw + t1; x++)
            {
                if (x < 0)
                    continue;

                if (x >= w)
                    break;

                p[x] = pen_color[0];
            }
        }
    }

    // draw left
    for (int x = rx - t0; x < rx + t1; x++)
    {
        if (x < 0)
            continue;

        if (x >= w)
            break;

        for (int y = ry + t1; y < ry + rh - t0; y++)
        {
            if (y < 0)
                continue;

            if (y >= h)
                break;

            unsigned char* p = pixels + stride * y;

            p[x] = pen_color[0];
        }
    }

    // draw right
    for (int x = rx + rw - t0; x < rx + rw + t1; x++)
    {
        if (x < 0)
            continue;

        if (x >= w)
            break;

        for (int y = ry + t1; y < ry + rh - t0; y++)
        {
            if (y < 0)
                continue;

            if (y >= h)
                break;

            unsigned char* p = pixels + stride * y;

            p[x] = pen_color[0];
        }
    }
}

void draw_rectangle_c2(unsigned char* pixels, int w, int h, int stride, int rx, int ry, int rw, int rh, unsigned int color, int thickness)
{
    const unsigned char* pen_color = (const unsigned char*)&color;

    if (thickness == -1)
    {
        // filled
        for (int y = ry; y < ry + rh; y++)
        {
            if (y < 0)
                continue;

            if (y >= h)
                break;

            unsigned char* p = pixels + stride * y;

            for (int x = rx; x < rx + rw; x++)
            {
                if (x < 0)
                    continue;

                if (x >= w)
                    break;

                p[x * 2 + 0] = pen_color[0];
                p[x * 2 + 1] = pen_color[1];
            }
        }

        return;
    }

    const int t0 = thickness / 2;
    const int t1 = thickness - t0;

    // draw top
    {
        for (int y = ry - t0; y < ry + t1; y++)
        {
            if (y < 0)
                continue;

            if (y >= h)
                break;

            unsigned char* p = pixels + stride * y;

            for (int x = rx - t0; x < rx + rw + t1; x++)
            {
                if (x < 0)
                    continue;

                if (x >= w)
                    break;

                p[x * 2 + 0] = pen_color[0];
                p[x * 2 + 1] = pen_color[1];
            }
        }
    }

    // draw bottom
    {
        for (int y = ry + rh - t0; y < ry + rh + t1; y++)
        {
            if (y < 0)
                continue;

            if (y >= h)
                break;

            unsigned char* p = pixels + stride * y;

            for (int x = rx - t0; x < rx + rw + t1; x++)
            {
                if (x < 0)
                    continue;

                if (x >= w)
                    break;

                p[x * 2 + 0] = pen_color[0];
                p[x * 2 + 1] = pen_color[1];
            }
        }
    }

    // draw left
    for (int x = rx - t0; x < rx + t1; x++)
    {
        if (x < 0)
            continue;

        if (x >= w)
            break;

        for (int y = ry + t1; y < ry + rh - t0; y++)
        {
            if (y < 0)
                continue;

            if (y >= h)
                break;

            unsigned char* p = pixels + stride * y;

            p[x * 2 + 0] = pen_color[0];
            p[x * 2 + 1] = pen_color[1];
        }
    }

    // draw right
    for (int x = rx + rw - t0; x < rx + rw + t1; x++)
    {
        if (x < 0)
            continue;

        if (x >= w)
            break;

        for (int y = ry + t1; y < ry + rh - t0; y++)
        {
            if (y < 0)
                continue;

            if (y >= h)
                break;

            unsigned char* p = pixels + stride * y;

            p[x * 2 + 0] = pen_color[0];
            p[x * 2 + 1] = pen_color[1];
        }
    }
}

void draw_rectangle_c3(unsigned char* pixels, int w, int h, int stride, int rx, int ry, int rw, int rh, unsigned int color, int thickness)
{
    const unsigned char* pen_color = (const unsigned char*)&color;

    if (thickness == -1)
    {
        // filled
        for (int y = ry; y < ry + rh; y++)
        {
            if (y < 0)
                continue;

            if (y >= h)
                break;

            unsigned char* p = pixels + stride * y;

            for (int x = rx; x < rx + rw; x++)
            {
                if (x < 0)
                    continue;

                if (x >= w)
                    break;

                p[x * 3 + 0] = pen_color[0];
                p[x * 3 + 1] = pen_color[1];
                p[x * 3 + 2] = pen_color[2];
            }
        }

        return;
    }

    const int t0 = thickness / 2;
    const int t1 = thickness - t0;

    // draw top
    {
        for (int y = ry - t0; y < ry + t1; y++)
        {
            if (y < 0)
                continue;

            if (y >= h)
                break;

            unsigned char* p = pixels + stride * y;

            for (int x = rx - t0; x < rx + rw + t1; x++)
            {
                if (x < 0)
                    continue;

                if (x >= w)
                    break;

                p[x * 3 + 0] = pen_color[0];
                p[x * 3 + 1] = pen_color[1];
                p[x * 3 + 2] = pen_color[2];
            }
        }
    }

    // draw bottom
    {
        for (int y = ry + rh - t0; y < ry + rh + t1; y++)
        {
            if (y < 0)
                continue;

            if (y >= h)
                break;

            unsigned char* p = pixels + stride * y;

            for (int x = rx - t0; x < rx + rw + t1; x++)
            {
                if (x < 0)
                    continue;

                if (x >= w)
                    break;

                p[x * 3 + 0] = pen_color[0];
                p[x * 3 + 1] = pen_color[1];
                p[x * 3 + 2] = pen_color[2];
            }
        }
    }

    // draw left
    for (int x = rx - t0; x < rx + t1; x++)
    {
        if (x < 0)
            continue;

        if (x >= w)
            break;

        for (int y = ry + t1; y < ry + rh - t0; y++)
        {
            if (y < 0)
                continue;

            if (y >= h)
                break;

            unsigned char* p = pixels + stride * y;

            p[x * 3 + 0] = pen_color[0];
            p[x * 3 + 1] = pen_color[1];
            p[x * 3 + 2] = pen_color[2];
        }
    }

    // draw right
    for (int x = rx + rw - t0; x < rx + rw + t1; x++)
    {
        if (x < 0)
            continue;

        if (x >= w)
            break;

        for (int y = ry + t1; y < ry + rh - t0; y++)
        {
            if (y < 0)
                continue;

            if (y >= h)
                break;

            unsigned char* p = pixels + stride * y;

            p[x * 3 + 0] = pen_color[0];
            p[x * 3 + 1] = pen_color[1];
            p[x * 3 + 2] = pen_color[2];
        }
    }
}

void draw_rectangle_c4(unsigned char* pixels, int w, int h, int stride, int rx, int ry, int rw, int rh, unsigned int color, int thickness)
{
    const unsigned char* pen_color = (const unsigned char*)&color;

    if (thickness == -1)
    {
        // filled
        for (int y = ry; y < ry + rh; y++)
        {
            if (y < 0)
                continue;

            if (y >= h)
                break;

            unsigned char* p = pixels + stride * y;

            for (int x = rx; x < rx + rw; x++)
            {
                if (x < 0)
                    continue;

                if (x >= w)
                    break;

                p[x * 4 + 0] = pen_color[0];
                p[x * 4 + 1] = pen_color[1];
                p[x * 4 + 2] = pen_color[2];
                p[x * 4 + 3] = pen_color[3];
            }
        }

        return;
    }

    const int t0 = thickness / 2;
    const int t1 = thickness - t0;

    // draw top
    {
        for (int y = ry - t0; y < ry + t1; y++)
        {
            if (y < 0)
                continue;

            if (y >= h)
                break;

            unsigned char* p = pixels + stride * y;

            for (int x = rx - t0; x < rx + rw + t1; x++)
            {
                if (x < 0)
                    continue;

                if (x >= w)
                    break;

                p[x * 4 + 0] = pen_color[0];
                p[x * 4 + 1] = pen_color[1];
                p[x * 4 + 2] = pen_color[2];
                p[x * 4 + 3] = pen_color[3];
            }
        }
    }

    // draw bottom
    {
        for (int y = ry + rh - t0; y < ry + rh + t1; y++)
        {
            if (y < 0)
                continue;

            if (y >= h)
                break;

            unsigned char* p = pixels + stride * y;

            for (int x = rx - t0; x < rx + rw + t1; x++)
            {
                if (x < 0)
                    continue;

                if (x >= w)
                    break;

                p[x * 4 + 0] = pen_color[0];
                p[x * 4 + 1] = pen_color[1];
                p[x * 4 + 2] = pen_color[2];
                p[x * 4 + 3] = pen_color[3];
            }
        }
    }

    // draw left
    for (int x = rx - t0; x < rx + t1; x++)
    {
        if (x < 0)
            continue;

        if (x >= w)
            break;

        for (int y = ry + t1; y < ry + rh - t0; y++)
        {
            if (y < 0)
                continue;

            if (y >= h)
                break;

            unsigned char* p = pixels + stride * y;

            p[x * 4 + 0] = pen_color[0];
            p[x * 4 + 1] = pen_color[1];
            p[x * 4 + 2] = pen_color[2];
            p[x * 4 + 3] = pen_color[3];
        }
    }

    // draw right
    for (int x = rx + rw - t0; x < rx + rw + t1; x++)
    {
        if (x < 0)
            continue;

        if (x >= w)
            break;

        for (int y = ry + t1; y < ry + rh - t0; y++)
        {
            if (y < 0)
                continue;

            if (y >= h)
                break;

            unsigned char* p = pixels + stride * y;

            p[x * 4 + 0] = pen_color[0];
            p[x * 4 + 1] = pen_color[1];
            p[x * 4 + 2] = pen_color[2];
            p[x * 4 + 3] = pen_color[3];
        }
    }
}

void draw_rectangle_yuv420sp(unsigned char* yuv420sp, int w, int h, int rx, int ry, int rw, int rh, unsigned int color, int thickness)
{
    // assert w % 2 == 0
    // assert h % 2 == 0
    // assert rx % 2 == 0
    // assert ry % 2 == 0
    // assert rw % 2 == 0
    // assert rh % 2 == 0
    // assert thickness % 2 == 0

    const unsigned char* pen_color = (const unsigned char*)&color;

    unsigned int v_y;
    unsigned int v_uv;
    unsigned char* pen_color_y = (unsigned char*)&v_y;
    unsigned char* pen_color_uv = (unsigned char*)&v_uv;
    pen_color_y[0] = pen_color[0];
    pen_color_uv[0] = pen_color[1];
    pen_color_uv[1] = pen_color[2];

    unsigned char* Y = yuv420sp;
    draw_rectangle_c1(Y, w, h, w, rx, ry, rw, rh, v_y, thickness);

    unsigned char* UV = yuv420sp + w * h;
    int thickness_uv = thickness == -1 ? thickness : max(thickness / 2, 1);
    draw_rectangle_c2(UV, w / 2, h / 2, w, rx / 2, ry / 2, rw / 2, rh / 2, v_uv, thickness_uv);
}


void draw_image_yuv420sp(unsigned char* yuv420sp, int w, int h, unsigned char* draw_img, int rx, int ry, int rw, int rh) {
    for (int i = 0; i < rh; i++) {
        memcpy(yuv420sp + (ry+i) * w + rx,  draw_img + i * rw,  rw);
    }
    for (int i = 0; i < rh/2; i++) {
        memcpy(yuv420sp + w*h + (ry/2+i) * w+ rx, draw_img + rw*rh + i*rw, rw);
    }
}

/* Draw text on the Y plane of a YUV420SP frame using OpenCV putText
 * (Hershey vector font, anti-aliased). White text on a black box.
 * `font_scale` is passed to cv::putText (0.8-1.0 readable, 1.5+ large). */
void draw_text_yuv420sp(unsigned char* yuv420sp, int w, int h, int tx, int ty, const char* text, int font_scale)
{
    if (font_scale < 1) font_scale = 1;
    double fs = (double)font_scale;
    int thickness = 2 * font_scale;

    /* measure text to size the black box */
    int baseline = 0;
    cv::Size text_sz = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, fs, thickness, &baseline);
    const int pad = 4 * font_scale;
    const int box_w = text_sz.width + pad * 2;
    const int box_h = text_sz.height + baseline + pad * 2;

    if (tx < 0) tx = 0;
    if (ty < 0) ty = 0;
    if (tx + box_w >= w) tx = w - box_w - 1;
    if (ty + box_h >= h) ty = h - box_h - 1;

    /* black background box (Y plane) */
    for (int y = ty; y < ty + box_h && y < h; y++)
    {
        unsigned char* p = yuv420sp + y * w;
        for (int x = tx; x < tx + box_w && x < w; x++)
            p[x] = 0;
    }
    /* neutral chroma under the box */
    for (int y = ty / 2; y < (ty + box_h) / 2 && y < h / 2; y++)
    {
        unsigned char* pu = yuv420sp + w * h + y * w;
        unsigned char* pv = pu + w * h / 4;
        for (int x = tx / 2; x < (tx + box_w) / 2 && x < w / 2; x++)
        {
            pu[x] = 128;
            pv[x] = 128;
        }
    }

    /* wrap the whole Y plane as a cv::Mat and draw text */
    cv::Mat y_plane(h, w, CV_8UC1, yuv420sp, w);
    cv::putText(y_plane, text, cv::Point(tx + pad, ty + pad + text_sz.height),
                cv::FONT_HERSHEY_SIMPLEX, fs, cv::Scalar(255), thickness, cv::LINE_AA);
}
