#ifndef BOXPROCESS_H
#define BOXPROCESS_H


#include "GxIAPI.h"
#include "DxImageProc.h"

#include <pthread.h>
#include <assert.h>
#include <string.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "ModulesDetect.h"
#include <DHCamera.h>
#include <SolveSRPnP.h>

using namespace std;
using namespace cv;

class BoxProcess
{

public:

  int BoxStage;

  int VisualRecognition_Init(DHCamera &DH_Camera);
  void GetimagePoints(DHCamera &DH_Camera ,int16_t VisionMessage[]);
  void GetboxPosition(DHCamera &DH_Camera ,Point3d  boxPosition);
  int  GetCamareStop(DHCamera &DH_Camera);
  void GetImageshow(DHCamera &DH_Camera);
  void GetImage0ff(DHCamera &DH_Camera);
  void SetudpAddress(DHCamera &DH_Camera,  uint32_t address);
  int  GetRealdistance(DHCamera & DH_Camera , Point2f EndPoint, float Realdistance[],float UavHeight);

};




#endif // BOXPROCESS_H

