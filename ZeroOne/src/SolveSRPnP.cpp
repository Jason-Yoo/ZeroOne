#include <SolveSRPnP.h>

void SolveSRPnP(const vector<Point3d> & objectPoints,
        const vector<Point2d> & imagePoints,
        const Mat & cameraMatrix,
        const Mat & distCoeffs,
        Mat & rMatrix,
        Mat & tVector)
{
        //定义查找表
//        float lut[24][4] = { { 1, 2, 3, 4 },{ 1, 2, 4, 3 },{ 1, 3, 2, 4 },{ 1, 3, 4, 2 },{ 1, 4, 2, 3 },{ 1, 4, 3, 2 },
//        { 2, 1, 3, 4 },{ 2, 1, 4, 3 },{ 2, 3, 1, 4 },{ 2, 3, 4, 1 },{ 2, 4, 1, 3 },{ 2, 4, 3, 1 },
//        { 3, 1, 2, 4 },{ 3, 1, 4, 2 },{ 3, 2, 1, 4 },{ 3, 2, 4, 1 },{ 3, 4, 1, 2 },{ 3, 4, 2, 1 },
//        { 4, 1, 2, 3 },{ 4, 1, 3, 2 },{ 4, 2, 1, 3 },{ 4, 2, 3, 1 },{ 4, 3, 1, 2 },{ 4, 3, 2, 1 } };
        float lut[4][4] = { { 1, 2, 3, 4 },{ 2, 3, 4, 1 },{ 3, 4, 1, 2 },{ 4, 1, 2, 3 } };

        //矫正畸变,注意矫正后的点就是归一化的图像点;
        vector<Point2d> dstPoints;
        undistortPoints(imagePoints, dstPoints, cameraMatrix, distCoeffs);

        //将OpenCV数据类型转换为工具箱可使用类型,即Eigen类型;
        points_t world_points;
        Image_points image_points;
        for (unsigned int j = 0; j < imagePoints.size(); j++)
        {
                point_t tempWorld;
                tempWorld[0] = objectPoints[j].x;
                tempWorld[1] = objectPoints[j].y;
                tempWorld[2] = objectPoints[j].z;

                Image_point tempImage;
                tempImage[0] = dstPoints[j].x;
                tempImage[1] = dstPoints[j].y;

                world_points.push_back(tempWorld);
                image_points.push_back(tempImage);
        }

        std::vector<double> error;
        transformations_t solution;

        for (int i = 0; i < 1; i++)
        {
                Image_points Image_input;
                Image_input.push_back(image_points[lut[i][0] - 1]);
                Image_input.push_back(image_points[lut[i][1] - 1]);
                Image_input.push_back(image_points[lut[i][2] - 1]);
                Image_input.push_back(image_points[lut[i][3] - 1]);

                //使用SRPnP算法计算位姿
                transformation_t pose = SRPnP::srpnp(Image_input, world_points);

                rotation_t R_estimation = pose.block<3, 3>(0, 0);
                translation_t t_estimation = pose.col(3);

                double er = 0;
                for (int k = 0; k < Image_input.size(); k++)
                {
                        point_t Xc = R_estimation*world_points[k] + t_estimation;
                        Image_point xc(Xc[0] / Xc[2], Xc[1] / Xc[2]);
                        er = er + (xc - Image_input[k]).norm();
                }
                error.push_back(er);
                solution.push_back(pose);
        }

        double min_err = error[0];
        int index = 0;
        for (int i = 1; i < error.size(); i++)
        {
                if (error[i]<min_err)
                {
                        min_err = error[i];
                        index = i;
                }
        }

        rotation_t R = solution[index].block<3, 3>(0, 0).transpose();
        translation_t t = -R*solution[index].col(3);

        double rot[9] = { R.row(0)[0], R.row(0)[1], R.row(0)[2],
                R.row(1)[0], R.row(1)[1], R.row(1)[2],
                R.row(2)[0], R.row(2)[1], R.row(2)[2] };
        double tran[3] = { t.col(0)[0], t.col(0)[1], t.col(0)[2] };

        Mat camera_Matrix = Mat(3, 3, CV_64FC1, rot);
        Mat camera_Vector(3, 1, CV_64FC1, tran);

        rMatrix = camera_Matrix.clone();
        tVector = camera_Vector.clone();
}
void getProjection(Mat & cameraMatrix,Mat & rMatrix,Mat & tVector)
{
    //calculate point for stacking
   float threeDim_wish[4][3] = {{0, 0, 25}, {0, 50, 25}, {100, 50, 25}, {100, 0, 25}};
    //转换为可以函数输入的形式;
    vector<Point3d> WishPoints;
    for (int i = 0; i < 4; i++)
    {
        WishPoints.push_back(Point3d(threeDim_wish[i][0], threeDim_wish[i][1], threeDim_wish[i][2]));
    }
    Eigen::MatrixXd A = Eigen::MatrixXd::Ones(1, 4);
  //  Eigen::MatrixXd B = rMatrix * WishPoints + tVector * A;
   // Eigen::MatrixXd p2d_i = cameraMatrix * B;
   // p2d_i = p2d_i / repmat(p2d_i(end,:),3,1);
   // p2d_i = p2d_i(1:2,:);
}
void Calculate_RT(vector<Point2f> &Image_Points, vector<Point3d> &BoxPosition)
{
    //定义输出旋转矩阵和平移矩阵
    Mat rMatrix;
    Mat tVector;

    //世界坐标 cm
  //  float threeDim[4][3] = {{0, 0, 0}, {0, 50, 0}, {100, 50, 0}, {100, 0, 0}};
    float threeDim[4][3] = {{0, 0, 0}, {0, 19.5, 0}, {28.5, 19.5, 0}, {28.5, 0, 0}};

    //转换为可以函数输入的形式;
    vector<Point3d> objectPoints;
    for (int i = 0; i < 4; i++)
    {
        objectPoints.push_back(Point3d(threeDim[i][0], threeDim[i][1], threeDim[i][2]));
    }

    //图像坐标
    vector<Point2d> imagePoints(4);
    for (int i = 0; i < 4; i++)
    {
        imagePoints[i] = Image_Points[i];
    }

    //相机内参数
    Mat_<double> cameraMatrix(3, 3);
    double fx = 736.1196;
    double fy = 739.3896;
    double Cx = 648.7056;
    double Cy = 512.9957;
    cameraMatrix << fx, 0, Cx, 0, fy, Cy, 0, 0, 1;
    //畸变系数;
    Mat_<double> distCoeffs(1, 5);
    distCoeffs << -0.2972, 0.0742, 0.00000, -0.00000, 0.00000;

//    double t = (double)getTickCount();
//    SolveSRPnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rMatrix, tVector);
//    t = ((double)getTickCount() - t) / getTickFrequency();

//    // 输出角度形式;
//    double pi = 3.1415926;
//    double A = atan(rMatrix.at<double>(1, 2) / rMatrix.at<double>(2, 2)) * 180 / pi;
//    double B = asin(-rMatrix.at<double>(0, 2)) * 180 / pi;
//    double C = atan(rMatrix.at<double>(0, 1) / rMatrix.at<double>(0, 0)) * 180 / pi;

//    cout << "*********Use SRPnP method to solve PNP problem************" << endl;
//    cout << "SRPnP method time=" << t * 1000 << "ms" << endl;
//    cout << "Pitch=" << A << endl;
//    cout << "Yaw  =" << B << endl;
//    cout << "Roll = " << C << endl;
//    cout << "X=" << tVector.at<double>(0, 0) << endl;
//    cout << "Y=" << tVector.at<double>(1, 0) << endl;
//    cout << "Z=" << tVector.at<double>(2, 0) << endl;
//    cout << "********************************************" << endl;

     double t = (double)getTickCount();
     solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rMatrix, tVector,false, CV_ITERATIVE);
     t = ((double)getTickCount() - t) / getTickFrequency();

     //update BoxPosition
     double pi = 3.1415926;
     BoxPosition[0].x = atan(rMatrix.at<double>(1, 2) / rMatrix.at<double>(2, 2)) * 180 / pi;
     BoxPosition[0].y = asin(-rMatrix.at<double>(0, 2)) * 180 / pi;
     BoxPosition[0].z = atan(rMatrix.at<double>(0, 1) / rMatrix.at<double>(0, 0)) * 180 / pi;
     BoxPosition[1].x = tVector.at<double>(0, 0);
     BoxPosition[1].y = tVector.at<double>(1, 0);
     BoxPosition[1].z = tVector.at<double>(2, 0);

//     // 输出角度形式;
//     double pi = 3.1415926;
//     double A = atan(rMatrix.at<double>(1, 2) / rMatrix.at<double>(2, 2)) * 180 / pi;
//     double B = asin(-rMatrix.at<double>(0, 2)) * 180 / pi;
//     double C = atan(rMatrix.at<double>(0, 1) / rMatrix.at<double>(0, 0)) * 180 / pi;

     cout << "***********solvePnP算法进行位姿态解算**********" << endl;
     cout << "Opencv中solvePnP算法解算时间=" << t * 1000 << "ms" << endl;
     cout << "Pitch=" << BoxPosition[0] << endl;
     cout << "Yaw  =" << BoxPosition[1] << endl;
     cout << "Roll =" << BoxPosition[2] << endl;
     cout << "X=" << BoxPosition[3] << endl;
     cout << "Y=" << BoxPosition[4] << endl;
     cout << "Z=" << BoxPosition[5] << endl;
     cout << "******************************************" << endl;

}

