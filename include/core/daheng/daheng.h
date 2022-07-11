//
// Created by yaione on 3/7/22.
//

#ifndef RMCV_DAHENG_H
#define RMCV_DAHENG_H

#include "GxIAPI.h"
#include "DxImageProc.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cstdlib>

using namespace std;
using namespace cv;

namespace rm {
    class DahengCamera {
    private:
        GX_STATUS status = GX_STATUS_SUCCESS;
        GX_DEV_HANDLE hDevice = nullptr;
        GX_FRAME_DATA frameData{};
        void *pRaw8Buffer = nullptr;
        void *pMirrorBuffer = nullptr;
        void *pRGBframeData = nullptr;
        int64_t PixelFormat = GX_PIXEL_FORMAT_BAYER_GR8;
        int64_t ColorFilter = GX_COLOR_FILTER_NONE;


    public:
        long fps = 0;
        int64_t SensorWidth = -1, SensorHeight = -1;

        /// Initialize DaHeng camera with given parameters.
        /// \param sn SN number of target camera.
        /// \param autoWhiteBalance Auto adjust white balance.
        /// \param expoosureTime Exposure time.
        /// \param gainFactor Gain factor. Value should be inside [-1.0, 1.0].
        /// \return Initialization status.
        bool
        dahengCameraInit(char *sn, bool autoWhiteBalance = false, int expoosureTime = 2000, double gainFactor = 1.0) {
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
            GXGetInt(hDevice, GX_INT_SENSOR_WIDTH, &SensorWidth);
            GXGetInt(hDevice, GX_INT_SENSOR_HEIGHT, &SensorHeight);

            pRaw8Buffer = malloc(nPayLoadSize);
            pMirrorBuffer = malloc(nPayLoadSize * 3);
            pRGBframeData = malloc(nPayLoadSize * 3);
            GXGetEnum(hDevice, GX_ENUM_PIXEL_FORMAT, &PixelFormat);
            GXGetEnum(hDevice, GX_ENUM_PIXEL_COLOR_FILTER, &ColorFilter);
            GXSetEnum(hDevice, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);

            GXSetEnum(hDevice, GX_ENUM_BALANCE_WHITE_AUTO,
                      autoWhiteBalance ? GX_BALANCE_WHITE_AUTO_CONTINUOUS : GX_BALANCE_WHITE_AUTO_OFF);
            GXSetFloat(hDevice, GX_FLOAT_EXPOSURE_TIME, expoosureTime);

            GXSetEnum(hDevice, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_ALL);
            GX_FLOAT_RANGE gainRange;
            GXGetFloatRange(hDevice, GX_FLOAT_GAIN, &gainRange);
            GXSetFloat(hDevice, GX_FLOAT_GAIN, gainRange.dMax * gainFactor);

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

        Mat getFrame(bool flip = false, bool mirror = false) {
            if (GXGetImage(hDevice, &frameData, 100) == GX_STATUS_SUCCESS) {
                if (frameData.nStatus == 0) {
                    ProcessData(frameData.pImgBuf, pRaw8Buffer, pRGBframeData, frameData.nWidth, frameData.nHeight,
                                (int) PixelFormat, mirror ? 2 : 4, flip, mirror);
                    fps++;
                    Mat src(Size(frameData.nWidth, frameData.nHeight), CV_8UC3, pRGBframeData);
                    return src;
                }
            }

            return {};
        }

        void ProcessData(void *pImageBuf, void *pImageRaw8Buf, void *pImageRGBBuf, int nImageWidth, int nImageHeight,
                         int nPixelFormat, int nPixelColorFilter, bool flip = false, bool mirror = false) {
            switch (nPixelFormat) {
                case GX_PIXEL_FORMAT_BAYER_GR12:
                case GX_PIXEL_FORMAT_BAYER_RG12:
                case GX_PIXEL_FORMAT_BAYER_GB12:
                case GX_PIXEL_FORMAT_BAYER_BG12:
                    if (mirror) {
                        DxImageMirror(pImageBuf, pMirrorBuffer, nImageWidth, nImageHeight, HORIZONTAL_MIRROR);
                        DxRaw16toRaw8(pMirrorBuffer, pImageRaw8Buf, nImageWidth, nImageHeight, DX_BIT_4_11);
                        DxRaw8toRGB24(pImageRaw8Buf, pImageRGBBuf, nImageWidth, nImageHeight, RAW2RGB_NEIGHBOUR,
                                      DX_PIXEL_COLOR_FILTER(nPixelColorFilter), flip);
                    } else {
                        DxRaw16toRaw8(pImageBuf, pImageRaw8Buf, nImageWidth, nImageHeight, DX_BIT_4_11);
                        DxRaw8toRGB24(pImageRaw8Buf, pImageRGBBuf, nImageWidth, nImageHeight, RAW2RGB_NEIGHBOUR,
                                      DX_PIXEL_COLOR_FILTER(nPixelColorFilter), flip);
                    }
                    break;

                case GX_PIXEL_FORMAT_BAYER_GR10:
                case GX_PIXEL_FORMAT_BAYER_RG10:
                case GX_PIXEL_FORMAT_BAYER_GB10:
                case GX_PIXEL_FORMAT_BAYER_BG10:
                    if (mirror) {
                        DxImageMirror(pImageBuf, pMirrorBuffer, nImageWidth, nImageHeight, HORIZONTAL_MIRROR);
                        DxRaw16toRaw8(pMirrorBuffer, pImageRaw8Buf, nImageWidth, nImageHeight, DX_BIT_2_9);
                        DxRaw8toRGB24(pImageRaw8Buf, pImageRGBBuf, nImageWidth, nImageHeight, RAW2RGB_NEIGHBOUR,
                                      DX_PIXEL_COLOR_FILTER(nPixelColorFilter), flip);
                    } else {
                        DxRaw16toRaw8(pImageBuf, pImageRaw8Buf, nImageWidth, nImageHeight, DX_BIT_2_9);
                        DxRaw8toRGB24(pImageRaw8Buf, pImageRGBBuf, nImageWidth, nImageHeight, RAW2RGB_NEIGHBOUR,
                                      DX_PIXEL_COLOR_FILTER(nPixelColorFilter), flip);
                    }
                    break;

                case GX_PIXEL_FORMAT_BAYER_GR8:
                case GX_PIXEL_FORMAT_BAYER_RG8:
                case GX_PIXEL_FORMAT_BAYER_GB8:
                case GX_PIXEL_FORMAT_BAYER_BG8:
                    if (mirror) {
                        DxImageMirror(pImageBuf, pMirrorBuffer, nImageWidth, nImageHeight, HORIZONTAL_MIRROR);
                        DxRaw8toRGB24(pMirrorBuffer, pImageRGBBuf, nImageWidth, nImageHeight, RAW2RGB_NEIGHBOUR,
                                      DX_PIXEL_COLOR_FILTER(nPixelColorFilter), flip); //RAW2RGB_ADAPTIVE
                    } else {
                        DxRaw8toRGB24(pImageBuf, pImageRGBBuf, nImageWidth, nImageHeight, RAW2RGB_NEIGHBOUR,
                                      DX_PIXEL_COLOR_FILTER(nPixelColorFilter), flip); //RAW2RGB_ADAPTIVE
                    }
                    break;

                case GX_PIXEL_FORMAT_MONO12:
                case GX_PIXEL_FORMAT_MONO10:
                    if (mirror) {
                        DxImageMirror(pImageBuf, pMirrorBuffer, nImageWidth, nImageHeight, HORIZONTAL_MIRROR);
                        DxRaw16toRaw8(pMirrorBuffer, pImageRaw8Buf, nImageWidth, nImageHeight, DX_BIT_4_11);
                        DxRaw8toRGB24(pImageRaw8Buf, pImageRGBBuf, nImageWidth, nImageHeight, RAW2RGB_NEIGHBOUR,
                                      DX_PIXEL_COLOR_FILTER(NONE), flip);
                    } else {
                        DxRaw16toRaw8(pImageBuf, pImageRaw8Buf, nImageWidth, nImageHeight, DX_BIT_4_11);
                        DxRaw8toRGB24(pImageRaw8Buf, pImageRGBBuf, nImageWidth, nImageHeight, RAW2RGB_NEIGHBOUR,
                                      DX_PIXEL_COLOR_FILTER(NONE), flip);
                    }
                    break;

                case GX_PIXEL_FORMAT_MONO8:
                    if (mirror) {
                        DxImageMirror(pImageBuf, pMirrorBuffer, nImageWidth, nImageHeight, HORIZONTAL_MIRROR);
                        DxRaw8toRGB24(pMirrorBuffer, pImageRGBBuf, nImageWidth, nImageHeight, RAW2RGB_NEIGHBOUR,
                                      DX_PIXEL_COLOR_FILTER(NONE), flip);
                    } else {
                        DxRaw8toRGB24(pImageBuf, pImageRGBBuf, nImageWidth, nImageHeight, RAW2RGB_NEIGHBOUR,
                                      DX_PIXEL_COLOR_FILTER(NONE), flip);
                    }
                    break;

                default:
                    break;
            }
        }
    };
}


#endif //RMCV_DAHENG_H
