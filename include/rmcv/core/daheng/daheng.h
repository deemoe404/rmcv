//
// Created by yaione on 3/7/22.
//

#ifndef RM_STANDARD2022_DAHENG_H
#define RM_STANDARD2022_DAHENG_H

#include "GxIAPI.h"
#include "DxImageProc.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cstdlib>

using namespace std;
using namespace cv;

class DahengCamera {
private:
    GX_STATUS status = GX_STATUS_SUCCESS;
    GX_DEV_HANDLE hDevice = nullptr;
    GX_FRAME_DATA frameData{};
    void *pRaw8Buffer = nullptr;
    void *pRGBframeData = nullptr;
    int64_t PixelFormat = GX_PIXEL_FORMAT_BAYER_GR8;
    int64_t ColorFilter = GX_COLOR_FILTER_NONE;

public:
    long fps = 0;

    bool dahengCameraInit(char *sn, int expoosureTime = 2000, int frameSpeed = 210) {
        GXInitLib();

        auto *openParam = new GX_OPEN_PARAM;
        openParam->openMode = GX_OPEN_SN;
        openParam->accessMode = GX_ACCESS_EXCLUSIVE;
        openParam->pszContent = sn;
        status = GXOpenDevice(openParam, &hDevice);
        if (status != GX_STATUS_SUCCESS) {
            return false;
        }

        int64_t nPayLoadSize = 0;

        status = GXGetInt(hDevice, GX_INT_PAYLOAD_SIZE, &nPayLoadSize);
        if (status != GX_STATUS_SUCCESS) {
            return false;
        }

        frameData.pImgBuf = malloc((size_t) nPayLoadSize);
        pRaw8Buffer = malloc(nPayLoadSize);
        pRGBframeData = malloc(nPayLoadSize * 3);
        GXGetEnum(hDevice, GX_ENUM_PIXEL_FORMAT, &PixelFormat);
        GXGetEnum(hDevice, GX_ENUM_PIXEL_COLOR_FILTER, &ColorFilter);
        GXSetEnum(hDevice, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
        GXSetEnum(hDevice, GX_ENUM_ACQUISITION_FRAME_RATE_MODE, GX_ACQUISITION_FRAME_RATE_MODE_ON);
        GXSetFloat(hDevice, GX_FLOAT_ACQUISITION_FRAME_RATE, frameSpeed);
        GXSetFloat(hDevice, GX_FLOAT_EXPOSURE_TIME, expoosureTime);
        GXSetEnum(hDevice, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_OFF);
        status = GXSendCommand(hDevice, GX_COMMAND_ACQUISITION_START);
        if (status != GX_STATUS_SUCCESS) {
            return false;
        }
        return true;
    }

    ~DahengCamera() {
        status = GXSendCommand(hDevice, GX_COMMAND_ACQUISITION_STOP);

        free(frameData.pImgBuf);
        frameData.pImgBuf = nullptr;
        free(pRaw8Buffer);
        pRaw8Buffer = nullptr;
        free(pRGBframeData);
        pRGBframeData = nullptr;

        GXCloseDevice(hDevice);
        GXCloseLib();
    }

    Mat getFrame() {
        if (GXGetImage(hDevice, &frameData, 100) == GX_STATUS_SUCCESS) {
            if (frameData.nStatus == 0) {
                ProcessData(frameData.pImgBuf, pRaw8Buffer, pRGBframeData, frameData.nWidth, frameData.nHeight,
                            (int) PixelFormat, 4);
                fps++;
                Mat src(Size(frameData.nWidth, frameData.nHeight), CV_8UC3, pRGBframeData);
                return src;
            }
        }

        return {};
    }

    static void ProcessData(void *pImageBuf, void *pImageRaw8Buf, void *pImageRGBBuf, int nImageWidth, int nImageHeight,
                            int nPixelFormat, int nPixelColorFilter) {
        switch (nPixelFormat) {
            case GX_PIXEL_FORMAT_BAYER_GR12:
            case GX_PIXEL_FORMAT_BAYER_RG12:
            case GX_PIXEL_FORMAT_BAYER_GB12:
            case GX_PIXEL_FORMAT_BAYER_BG12:
                DxRaw16toRaw8(pImageBuf, pImageRaw8Buf, nImageWidth, nImageHeight, DX_BIT_4_11);
                DxRaw8toRGB24(pImageRaw8Buf, pImageRGBBuf, nImageWidth, nImageHeight, RAW2RGB_NEIGHBOUR,
                              DX_PIXEL_COLOR_FILTER(nPixelColorFilter), false);
                break;

            case GX_PIXEL_FORMAT_BAYER_GR10:
            case GX_PIXEL_FORMAT_BAYER_RG10:
            case GX_PIXEL_FORMAT_BAYER_GB10:
            case GX_PIXEL_FORMAT_BAYER_BG10:
                DxRaw16toRaw8(pImageBuf, pImageRaw8Buf, nImageWidth, nImageHeight, DX_BIT_2_9);
                DxRaw8toRGB24(pImageRaw8Buf, pImageRGBBuf, nImageWidth, nImageHeight, RAW2RGB_NEIGHBOUR,
                              DX_PIXEL_COLOR_FILTER(nPixelColorFilter), false);
                break;

            case GX_PIXEL_FORMAT_BAYER_GR8:
            case GX_PIXEL_FORMAT_BAYER_RG8:
            case GX_PIXEL_FORMAT_BAYER_GB8:
            case GX_PIXEL_FORMAT_BAYER_BG8:
                DxRaw8toRGB24(pImageBuf, pImageRGBBuf, nImageWidth, nImageHeight, RAW2RGB_NEIGHBOUR,
                              DX_PIXEL_COLOR_FILTER(nPixelColorFilter), false); //RAW2RGB_ADAPTIVE
                break;

            case GX_PIXEL_FORMAT_MONO12:
            case GX_PIXEL_FORMAT_MONO10:
                DxRaw16toRaw8(pImageBuf, pImageRaw8Buf, nImageWidth, nImageHeight, DX_BIT_4_11);
                DxRaw8toRGB24(pImageRaw8Buf, pImageRGBBuf, nImageWidth, nImageHeight, RAW2RGB_NEIGHBOUR,
                              DX_PIXEL_COLOR_FILTER(NONE), false);
                break;

            case GX_PIXEL_FORMAT_MONO8:
                DxRaw8toRGB24(pImageBuf, pImageRGBBuf, nImageWidth, nImageHeight, RAW2RGB_NEIGHBOUR,
                              DX_PIXEL_COLOR_FILTER(NONE), false);
                break;

            default:
                break;
        }
    }
};

#endif //RM_STANDARD2022_DAHENG_H
