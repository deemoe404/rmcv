//
// Created by yaione on 3/7/22.
//

#ifndef RM_STANDARD2022_DAHENG_H
#define RM_STANDARD2022_DAHENG_H

#include "GxIAPI.h"
#include "DxImageProc.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <stdlib.h>

using namespace std;
using namespace cv;

class DahengCamera {
private:
    GX_STATUS status = GX_STATUS_SUCCESS;
    GX_DEV_HANDLE hDevice = NULL;
    GX_FRAME_DATA frameData;                         ///< 定义GXGetImage的传入参数
    void *pRaw8Buffer = NULL;                        ///< 将非8位raw数据转换成8位数据的时候的中转缓冲buffer
    void *pRGBframeData = NULL;                      ///< RAW数据转换成RGB数据后的存储空间，大小是相机输出数据大小的3倍
    int64_t PixelFormat = GX_PIXEL_FORMAT_BAYER_GR8; ///< 当前相机的pixelformat格式
    int64_t ColorFilter = GX_COLOR_FILTER_NONE;      ///< bayer插值的参数

public:
    int fps = 0;
    /**
     * 摄像头相机内参及畸变系数
     */
    Mat cameraMatrix = (Mat_<double>(3, 3)
            << 1.5024, 0.000000000000, 0.000000000000, 0.000000000000, 1.4541, 0.000000000000, 0.6305, 0.5163, 0.0010);
    Mat distCoeffes = (Mat_<double>(1, 5) << -0.3146, 0.2621, 0.4609, 0.000000000000, 0.000000000000);

    /**
     * 大恒相机(水星系列)初始化
     */
    bool dahengCameraInit(char *sn, int expoosureTime = 2000, int speedLevel = 5) {
        //初始化库
        GXInitLib();

        //通过序列号
        GX_OPEN_PARAM *openParam = new GX_OPEN_PARAM;
        openParam->openMode = GX_OPEN_SN;
        openParam->accessMode = GX_ACCESS_EXCLUSIVE;
        //"RA0025005014" "RA0031005014"
        openParam->pszContent = sn;
        status = GXOpenDevice(openParam, &hDevice);
        if (status != GX_STATUS_SUCCESS) {
            cerr << status << endl;
            printf("设备打开失败!\n");
            return 0;
        }

        int64_t nPayLoadSize = 0;
        //获取图像buffer大小，下面动态申请内存
        // int64_t nWidth = 1024;
        // int64_t nHeight = 720;
        // int64_t nOffsetX = 128;
        // int64_t nOffsetY = 152;
        // status = GXSetInt(hDevice, GX_INT_WIDTH, nWidth);
        // status = GXSetInt(hDevice, GX_INT_HEIGHT, nHeight);
        // status = GXSetInt(hDevice, GX_INT_OFFSET_X, nOffsetX);
        // status = GXSetInt(hDevice, GX_INT_OFFSET_Y, nOffsetY);

        status = GXGetInt(hDevice, GX_INT_PAYLOAD_SIZE, &nPayLoadSize);
        if (status != GX_STATUS_SUCCESS) {
            cerr << status << endl;
            cerr << "获取图像BUFFER失败" << endl;
            return false;
        }
        //根据获取的图像buffer大小m_nPayLoadSize申请buffer
        frameData.pImgBuf = malloc((size_t) nPayLoadSize);
        //将非8位raw数据转换成8位数据的时候的中转缓冲buffer
        pRaw8Buffer = malloc(nPayLoadSize);
        //RGB数据是RAW数据的3倍大小
        pRGBframeData = malloc(nPayLoadSize * 3);
        //获取相机输出数据的颜色格式
        GXGetEnum(hDevice, GX_ENUM_PIXEL_FORMAT, &PixelFormat);
        GXGetEnum(hDevice, GX_ENUM_PIXEL_COLOR_FILTER, &ColorFilter);
        //设置采集模式为连续采集
        GXSetEnum(hDevice, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
        //设置采集速度级别
        // GXSetInt(hDevice, GX_INT_ACQUISITION_SPEED_LEVEL, speedLevel);
        //使 能 采 集 帧 率 调 节 模 式
        GXSetEnum(hDevice, GX_ENUM_ACQUISITION_FRAME_RATE_MODE, GX_ACQUISITION_FRAME_RATE_MODE_ON);
        //设 置 采 集 帧 率,假 设 设 置 为 10.0, 用 户 按 照 实 际 需 求 设 置 此 值
        GXSetFloat(hDevice, GX_FLOAT_ACQUISITION_FRAME_RATE, 400);

        //设置曝光时间
        GXSetFloat(hDevice, GX_FLOAT_EXPOSURE_TIME, expoosureTime);

        //设置连续自动白平衡
        GXSetEnum(hDevice, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_CONTINUOUS);

        //发送开始采集命令
        status = GXSendCommand(hDevice, GX_COMMAND_ACQUISITION_START);
        if (status != GX_STATUS_SUCCESS) {
            cerr << "发送开采指令失败" << endl;
            return false;
        }
        return true;
    }

    ~DahengCamera() {
        //发送停止采集命令
        status = GXSendCommand(hDevice, GX_COMMAND_ACQUISITION_STOP);

        //释放buffer
        free(frameData.pImgBuf);
        frameData.pImgBuf = NULL;
        free(pRaw8Buffer);
        pRaw8Buffer = NULL;
        free(pRGBframeData);
        pRGBframeData = NULL;

        GXCloseDevice(hDevice);
        GXCloseLib();
    }

    Mat getFrame() {
        //调用GXGetImage取一帧图像
        if (GXGetImage(hDevice, &frameData, 100) == GX_STATUS_SUCCESS) {
            if (frameData.nStatus == 0) {
                //将Raw数据处理成RGB数据
                ProcessData(frameData.pImgBuf, pRaw8Buffer, pRGBframeData, frameData.nWidth, frameData.nHeight,
                            PixelFormat, 4);
                fps++;
                Mat src(Size(frameData.nWidth, frameData.nHeight), CV_8UC3, pRGBframeData);
                return src;
            }
        }

        return cv::Mat();
    }

    void ProcessData(void *pImageBuf, void *pImageRaw8Buf, void *pImageRGBBuf, int nImageWidth, int nImageHeight,
                     int nPixelFormat, int nPixelColorFilter) {
        switch (nPixelFormat) {
            //当数据格式为12位时，位数转换为4-11
            case GX_PIXEL_FORMAT_BAYER_GR12:
            case GX_PIXEL_FORMAT_BAYER_RG12:
            case GX_PIXEL_FORMAT_BAYER_GB12:
            case GX_PIXEL_FORMAT_BAYER_BG12:
                //将12位格式的图像转换为8位格式
                DxRaw16toRaw8(pImageBuf, pImageRaw8Buf, nImageWidth, nImageHeight, DX_BIT_4_11);
                //将Raw8图像转换为RGB图像以供显示
                DxRaw8toRGB24(pImageRaw8Buf, pImageRGBBuf, nImageWidth, nImageHeight, RAW2RGB_NEIGHBOUR,
                              DX_PIXEL_COLOR_FILTER(nPixelColorFilter), false);
                break;

                //当数据格式为10位时，位数转换为2-9
            case GX_PIXEL_FORMAT_BAYER_GR10:
            case GX_PIXEL_FORMAT_BAYER_RG10:
            case GX_PIXEL_FORMAT_BAYER_GB10:
            case GX_PIXEL_FORMAT_BAYER_BG10:
                ////将10位格式的图像转换为8位格式,有效位数2-9
                DxRaw16toRaw8(pImageBuf, pImageRaw8Buf, nImageWidth, nImageHeight, DX_BIT_2_9);
                //将Raw8图像转换为RGB图像以供显示
                DxRaw8toRGB24(pImageRaw8Buf, pImageRGBBuf, nImageWidth, nImageHeight, RAW2RGB_NEIGHBOUR,
                              DX_PIXEL_COLOR_FILTER(nPixelColorFilter), false);
                break;

            case GX_PIXEL_FORMAT_BAYER_GR8:
            case GX_PIXEL_FORMAT_BAYER_RG8:
            case GX_PIXEL_FORMAT_BAYER_GB8:
            case GX_PIXEL_FORMAT_BAYER_BG8:
                //将Raw8图像转换为RGB图像以供显示
                // g_objTimeCounter.Begin();
                DxRaw8toRGB24(pImageBuf, pImageRGBBuf, nImageWidth, nImageHeight, RAW2RGB_NEIGHBOUR,
                              DX_PIXEL_COLOR_FILTER(nPixelColorFilter), false); //RAW2RGB_ADAPTIVE
                // printf("DxRaw8toRGB24 耗时：%ld\n", g_objTimeCounter.End());
                break;

            case GX_PIXEL_FORMAT_MONO12:
                //将12位格式的图像转换为8位格式
                DxRaw16toRaw8(pImageBuf, pImageRaw8Buf, nImageWidth, nImageHeight, DX_BIT_4_11);
                //将Raw8图像转换为RGB图像以供显示
                DxRaw8toRGB24(pImageRaw8Buf, pImageRGBBuf, nImageWidth, nImageHeight, RAW2RGB_NEIGHBOUR,
                              DX_PIXEL_COLOR_FILTER(NONE), false);
                break;

            case GX_PIXEL_FORMAT_MONO10:
                //将10位格式的图像转换为8位格式
                DxRaw16toRaw8(pImageBuf, pImageRaw8Buf, nImageWidth, nImageHeight, DX_BIT_4_11);
                //将Raw8图像转换为RGB图像以供显示
                DxRaw8toRGB24(pImageRaw8Buf, pImageRGBBuf, nImageWidth, nImageHeight, RAW2RGB_NEIGHBOUR,
                              DX_PIXEL_COLOR_FILTER(NONE), false);
                break;

            case GX_PIXEL_FORMAT_MONO8:
                //将Raw8图像转换为RGB图像以供显示
                DxRaw8toRGB24(pImageBuf, pImageRGBBuf, nImageWidth, nImageHeight, RAW2RGB_NEIGHBOUR,
                              DX_PIXEL_COLOR_FILTER(NONE), false);
                break;

            default:
                break;
        }
    }
};

#endif //RM_STANDARD2022_DAHENG_H
