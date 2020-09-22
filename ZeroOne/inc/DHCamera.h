#ifndef DHCAMERA_DHCAMERA_H
#define DHCAMERA_DHCAMERA_H

#include "GxIAPI.h"
#include "DxImageProc.h"
#include "ModulesDetect.h"

#include <thread>
#include <mutex>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

class DHCamera {

public:

    ModulesDetect             Modules_Detect;

    uint32_t                  ui32DeviceNum = 0;
    uint32_t                  ui32FPS = 0;
    //unsigned char             udpAddress = NULL;

    GX_STATUS                 emStatus;
    GX_DEV_HANDLE             g_hDevice = NULL;                           ///< Device handle
    GX_DEV_HANDLE             g2_hDevice = NULL;                           ///< Device handle

    //camera first
    float                     DHcamera1dx = 960;
    float                     realdistance[5];
    bool                      g_bColorFilter = false;                        ///< Color filter support flag
    int64_t                   g_i64ColorFilter = GX_COLOR_FILTER_NONE;    ///< Color filter of device

    bool                      g_bAcquisitionFlag = false;                    ///< Thread running flag
    pthread_t                 g_nAcquisitonThreadID = 0;                ///< Thread ID of Acquisition thread

    bool                      g_ImageProcessFlag = false;
    pthread_t                 g_ImageProcessThreadID = 1;

    bool                      g_ImageShowFlag = false;

    //camera second
    bool                      g2_bColorFilter = false;                        ///< Color filter support flag
    int64_t                   g2_i64ColorFilter = GX_COLOR_FILTER_NONE;    ///< Color filter of device

    bool                      g2_bAcquisitionFlag = false;                    ///< Thread running flag
    pthread_t                 g2_nAcquisitonThreadID = 2;                ///< Thread ID of Acquisition thread

    bool                      g2_ImageProcessFlag = false;
    pthread_t                 g2_ImageProcessThreadID = 3;

    bool                      g2_ImageShowFlag = false;


    unsigned char*            g_pRGBImageBuf = NULL;               ///< Memory for RAW8toRGB24
    unsigned char*            g_pRaw8Image = NULL;                 ///< Memory for RAW16toRAW8

    unsigned char*            g2_pRGBImageBuf = NULL;               ///< Memory for RAW8toRGB24
    unsigned char*            g2_pRaw8Image = NULL;                 ///< Memory for RAW16toRAW8


    int64_t                   g_nPayloadSize = 0;                         ///< Payload size
    int64_t                   g2_nPayloadSize = 0;                         ///< Payload size


    unsigned char           * pbyBuffer=NULL;
    unsigned char           * g_pRgbBuffer[2];     //处理后数据缓存区

    bool                    updated;
    bool                    started=0;
    bool                    g_bSaveVedioFlag=false;


    Mat                      src_image; // camera 1
    Mat                      src_image2; //Camera 2

    Point2f                 DH_StartPoint;
    Point2f                 DH_EndPoint;

    Point3d                 BoxPosition;

    // Mat_<double>            cameraMatrix;
    // Mat_<double>            distCoeffs;

    int    Init();
    int    Play();
    int    Read();
    int    Stop();
    int    Uninit();
    int    SetExposureTime(bool auto_exp, double exp_time = 10000);
    double GetExposureTime();
    int    SetLargeResolution(bool if_large_resolution);
    Size   GetResolution();
    int    SetGain(double gain);
    double GetGain();
    int    SetWBMode(bool auto_wb = true);
    int    GetWBMode(bool & auto_wb);
    int    SetOnceWB();
    int    ProcessFrame();
    int    Set_fps(int fps_mode);
    int    PreForAcquisition();
    int    UnPreForAcquisition();
    int    Gammaprocess();


    void   GetErrorString(GX_STATUS emErrorStatus);
    void   DH_Camera();



    GX_STATUS GX_VERIFY(GX_STATUS emStatus);
    GX_STATUS GX_VERIFY_EXIT(GX_STATUS emStatus);



};

void *ProcGetImage(void* pParam);           //图像获取线程函数
void *ProcGetImage2(void* pParam);           //图像获取线程函数
void *ImageProcess(void* image);            //图像处理线程函数



//void bgr2binary(Mat &srcImage, Mat &dstImage, int method);//lingai
//void Otsu(Mat &srcImage , int &threshold) ;//lingai
#endif
