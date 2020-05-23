#ifndef DHCAMERA_DHCAMERA_H
#define DHCAMERA_DHCAMERA_H

#include "GxIAPI.h"
#include "DxImageProc.h"
#include <TemplateMatch.h>
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

    uint32_t                  ui32DeviceNum = 0;
    GX_STATUS                 emStatus;

    GX_DEV_HANDLE             g_hDevice = NULL;                     ///< Device handle
    bool                      g_bColorFilter = false;                        ///< Color filter support flag
    int64_t                   g_i64ColorFilter = GX_COLOR_FILTER_NONE;    ///< Color filter of device

    bool                      g_bAcquisitionFlag = false;                    ///< Thread running flag
    pthread_t                 g_nAcquisitonThreadID = 0;                ///< Thread ID of Acquisition thread

    bool                      g_ImageProcessFlag = false;
    pthread_t                 g_ImageProcessThreadID = 1;

    unsigned char*            g_pRGBImageBuf = NULL;               ///< Memory for RAW8toRGB24
    unsigned char*            g_pRaw8Image = NULL;                 ///< Memory for RAW16toRAW8


    int64_t                   g_nPayloadSize = 0;                         ///< Payload size

    unsigned char           * pbyBuffer=NULL;
    unsigned char           * g_pRgbBuffer[2];     //处理后数据缓存区

    bool                    updated;
    bool                    started=0;
    bool                    g_bSaveVedioFlag=false;
    mutex                   mutex1;
    IplImage                * iplImage = NULL;
    Mat                      src_image;

   // Mat_<double>            cameraMatrix;
   // Mat_<double>            distCoeffs;

  int    Init();
  int    Play();
  int    Read();
  int    GetFrame(Mat& frame,bool is_color);
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
  int    GetFrame();
  int    Set_fps(int fps_mode);
  int    PreForAcquisition();
  int    UnPreForAcquisition();
  void   GetErrorString(GX_STATUS emErrorStatus);

  GX_STATUS GX_VERIFY(GX_STATUS emStatus);
  GX_STATUS GX_VERIFY_EXIT(GX_STATUS emStatus);

  DHCamera();

};

void *ProcGetImage(void* pParam);
void *ImageProcess(void* image);

#endif
