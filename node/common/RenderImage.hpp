#ifndef SAMPLE_COMMON_RENDERIMAGE_HPP_
#define SAMPLE_COMMON_RENDERIMAGE_HPP_

#include "RenderWindow.hpp"

using namespace RenderImage;

static int CoverYuv422_T_RGB_Pixel(int y, int u, int v)
{
    unsigned int pixel32 = 0;
    unsigned char *pixel = (unsigned char *)&pixel32;
    int r, g, b;
    r = y + (1.370705 * (v - 128));
    g = y - (0.698001 * (v - 128)) - (0.337633 * (u - 128));
    b = y + (1.732446 * (u - 128));
    if (r > 255)
    {
        r = 255;
    }
    if (g > 255)
    {
        g = 255;
    }
    if (b > 255)
    {
        b = 255;
    }
    if (r < 0)
    {
        r = 0;
    }
    if (g < 0)
    {
        g = 0;
    }
    if (b < 0)
    {
        b = 0;
    }
    pixel[0] = r;
    pixel[1] = g;
    pixel[2] = b;
    return pixel32;
}

static void YUV422_T_RGB(unsigned int nWidth, unsigned int nHeight,const unsigned char *pYUVSrc, unsigned char *pRGBDst)
{
    unsigned int in, out = 0;
    unsigned int pixel_16;
    unsigned char pixel_24[3];
    unsigned int pixel32;
    int y0, u, y1, v;

    if ((pYUVSrc == NULL) || (pRGBDst == NULL))
    {    
        return;
    }

    for (in = 0; in < nWidth * nHeight * 2; in += 4)
    {
        pixel_16 =
            pYUVSrc[in + 3] << 24 |
            pYUVSrc[in + 2] << 16 |
            pYUVSrc[in + 1] << 8 |
            pYUVSrc[in + 0];
        y0 = (pixel_16 & 0x000000ff);
        u = (pixel_16 & 0x0000ff00) >> 8;
        y1 = (pixel_16 & 0x00ff0000) >> 16;
        v = (pixel_16 & 0xff000000) >> 24;
        pixel32 = CoverYuv422_T_RGB_Pixel(y0, u, v);
        pixel_24[0] = (pixel32 & 0x000000ff);
        pixel_24[1] = (pixel32 & 0x0000ff00) >> 8;
        pixel_24[2] = (pixel32 & 0x00ff0000) >> 16;
        pRGBDst[out++] = pixel_24[0];
        pRGBDst[out++] = pixel_24[1];
        pRGBDst[out++] = pixel_24[2];
        pixel32 = CoverYuv422_T_RGB_Pixel(y1, u, v);
        pixel_24[0] = (pixel32 & 0x000000ff);
        pixel_24[1] = (pixel32 & 0x0000ff00) >> 8;
        pixel_24[2] = (pixel32 & 0x00ff0000) >> 16;
        pRGBDst[out++] = pixel_24[0];
        pRGBDst[out++] = pixel_24[1];
        pRGBDst[out++] = pixel_24[2];
    }
    return;
}

static int parseFrame(MV3D_RGBD_FRAME_DATA* pstFrameData, RIFrameInfo* pDepth
                             ,RIFrameInfo* pRgb)
{
    for (unsigned int i = 0; i < pstFrameData->nImageCount; i++)
    {
        // LOGD("parseFrame :enImageType (%08x) framenum (%d) height(%d) width(%d)  len (%d)!", pstFrameData->stImageData[i].enImageType, pstFrameData->stImageData[i].nFrameNum,
            // pstFrameData->stImageData[i].nHeight, pstFrameData->stImageData[i].nWidth, pstFrameData->stImageData[i].nDataLen);

        if (ImageType_Depth == pstFrameData->stImageData[i].enImageType)
        {
            pDepth->enPixelType = RIPixelType_Coord3D_C16;
            pDepth->nFrameNum   = pstFrameData->stImageData[i].nFrameNum;
            pDepth->nHeight     = pstFrameData->stImageData[i].nHeight;
            pDepth->nWidth      = pstFrameData->stImageData[i].nWidth;
            pDepth->nFrameLength= pstFrameData->stImageData[i].nDataLen;
            pDepth->pData       = pstFrameData->stImageData[i].pData;

        }

        if (ImageType_RGB8_Planar == pstFrameData->stImageData[i].enImageType)
        {
            pRgb->enPixelType   = RIPixelType_RGB8_Planar;
            pRgb->nFrameNum     = pstFrameData->stImageData[i].nFrameNum;
            pRgb->nHeight       = pstFrameData->stImageData[i].nHeight;
            pRgb->nWidth        = pstFrameData->stImageData[i].nWidth;
            pRgb->nFrameLength  = pstFrameData->stImageData[i].nDataLen;
            pRgb->pData         = pstFrameData->stImageData[i].pData;
        }

        if (ImageType_YUV422 == pstFrameData->stImageData[i].enImageType)
        {
            int nDstImageLen = pstFrameData->stImageData[i].nWidth * pstFrameData->stImageData[i].nHeight * 3;
            static unsigned char* pRGBBuffer = NULL;
            static unsigned int nImageLen = nDstImageLen;
            if (pRGBBuffer == NULL || nImageLen != nDstImageLen)
            {
                nImageLen = nDstImageLen;
                pRGBBuffer = (unsigned char *)malloc(nImageLen * sizeof(unsigned char));
                if (pRGBBuffer == NULL)
                {
                    LOGD("YUV422 buffer malloc fail!");
                    return -1;
                }
                memset(pRGBBuffer, 0x00, nImageLen * sizeof(unsigned char));
                LOGD("YUV422 buffer malloc success!");
            }
            YUV422_T_RGB(pstFrameData->stImageData[i].nWidth, pstFrameData->stImageData[i].nHeight, pstFrameData->stImageData[i].pData, pRGBBuffer);

            pRgb->enPixelType = RIPixelType_RGB8_Packed;
            pRgb->nFrameNum = pstFrameData->stImageData[i].nFrameNum;
            pRgb->nHeight = pstFrameData->stImageData[i].nHeight;
            pRgb->nWidth = pstFrameData->stImageData[i].nWidth;
            pRgb->nFrameLength = pstFrameData->stImageData[i].nDataLen;
            pRgb->pData = pRGBBuffer;
        }
    }

    return 0;
}


#endif