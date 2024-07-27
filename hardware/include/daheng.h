//
// Created by yaione on 3/7/22.
//

#ifndef RMCV_DAHENG_H
#define RMCV_DAHENG_H

#include "daheng/GxIAPI.h"
#include "daheng/DxImageProc.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cstdlib>


namespace rm::hardware
{
    class daheng
    {
        GX_STATUS status = GX_STATUS_SUCCESS;
        GX_DEV_HANDLE hDevice = nullptr;
        GX_FRAME_DATA frameData{};
        void* pRaw8Buffer = nullptr;
        void* pMirrorBuffer = nullptr;
        void* pRGBframeData = nullptr;
        int64_t PixelFormat = GX_PIXEL_FORMAT_BAYER_GR8;
        int64_t ColorFilter = GX_COLOR_FILTER_NONE;

        void ProcessData(void* pImageBuf, void* pImageRaw8Buf, void* pImageRGBBuf, int nImageWidth, int nImageHeight,
                         int nPixelFormat, int nPixelColorFilter, bool flip = false, bool mirror = false) const;

    public:
        long fps = 0;
        int64_t SensorWidth = -1, SensorHeight = -1;

        /// Initialize DaHeng camera with given parameters.
        /// \param sn SN number of target camera.
        /// \param autoWhiteBalance Auto adjust white balance.
        /// \param expoosureTime Exposure time.
        /// \param gainFactor Gain factor. Value should be inside [-1.0, 1.0].
        /// \return Initialization status.
        bool initialize(std::string index, bool autoWhiteBalance = false, int expoosureTime = 2000,
                              double gainFactor = 1.0);

        ~daheng();

        cv::Mat capture(bool flip = false, bool mirror = false);
    };
}

#endif //RMCV_DAHENG_H
