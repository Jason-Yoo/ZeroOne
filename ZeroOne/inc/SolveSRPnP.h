#ifndef SOLVESRPNP_H
#define SOLVESRPNP_H


#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <SRPnP.hpp>


using namespace cv;
using namespace Eigen;
using namespace std;
using namespace gv;


void SolveSRPnP(const vector<Point3d> & objectPoints,
        const vector<Point2d> & imagePoints,
        const Mat & cameraMatrix,
        const Mat & distCoeffs,
        Mat & rMatrix,
        Mat & tVector);

void Calculate_RT(vector<Point2f> &Image_Points, vector<Point3d> &BoxPosition);

#endif // SOLVESRPNP_H

