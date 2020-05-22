#ifndef MODULESDETECT_H
#define MODULESDETECT_H

#include <iostream>

#include <stdio.h>
#include "stdlib.h"

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace std;
using namespace cv;

class RotRect {
public:
    cv::Point2f center;
    cv::Point2f dir;
    float width;
    float height;

    RotRect() : width(0), height(0) {};
    RotRect(const cv::Rect & rect) :
        width(rect.width), height(rect.height)
    {
        center.x = rect.x + rect.width*0.5f;
        center.y = rect.y + rect.height*0.5f;
        dir.x = 1;
        dir.y = 0;
  }
};

class ModulesDetect
{
  public:

    int bgr2binary(Mat &srcImage, Mat &dstImage, int method);
    int Otsu(Mat &srcImage , int &threshold );
    int RecognitionFailure(Mat &srcImage);
    float GetPixelLength(Point PixelPointA, Point PixelPointB);
    Point find_connected(Mat &binary_img);

    struct LeafInfo
        {
          RotatedRect ellipseRect;
          Point2f leaf_center;
          Point2f vertices[4];
          Rect externel_rect;
          Point2f vec_chang;
          float chang,kuan;
          bool istarget=false;
          Point target_pix;
        };

    Mat detect_frame;

};

class CamParams
{
  public:
    int rows, cols,fps;
    float cx, cy, fx, fy,distcoef1,distcoef2;
    CamParams(int rows_, int cols_,int fps_,
                 float cx_,float cy_,
                 float fx_, float fy_,
                 float distcoef1_,float distcoef2_ ):
        rows(rows_),cols(cols_),
        cx(cx_),cy(cy_),
        fx(fx_),fy(fy_),
        fps(fps_),distcoef1(distcoef1_),distcoef2(distcoef2_)
    {}
    CamParams(int cam_idx,bool is_large);
};

#endif // MODULESDETECT_H
