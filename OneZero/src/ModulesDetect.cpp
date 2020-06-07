#include "ModulesDetect.h"

/// \brief ModulesDetect::Otsu
/// \param srcImage   input
/// \param threshold    output
/// \return
///
int ModulesDetect::Otsu(Mat &srcImage , int &threshold)
{

    int height = srcImage.rows;
    int width = srcImage.cols;

    //histogram
    float histogram[256] = {0};
    for (int i = 0; i < height; i++)
    {
        unsigned char *p = (unsigned char *)srcImage.data + srcImage.step * i;
        for (int j = 0; j < width; j++)
        {
            histogram[*p++]++;
        }
    }
    //normalize histogram
    int size = height * width;
    for (int i = 0; i < 256; i++)
    {
        histogram[i] = histogram[i] / size;
    }

    //average pixel value
    float avgValue = 0;
    for (int i = 0; i < 256; i++)
    {
        avgValue += i * histogram[i]; //整幅图像的平均灰度
    }

    float maxVariance = 0;
    float w = 0, u = 0;
    for (int i = 0; i < 256; i++)
    {
        w += histogram[i];     //假设当前灰度i为阈值, 0~i 灰度的像素(假设像素值在此范围的像素叫做前景像素) 所占整幅图像的比例
        u += i * histogram[i]; // 灰度i 之前的像素(0~i)的平均灰度值： 前景像素的平均灰度值

        float t = avgValue * w - u;
        float variance = t * t / (w * (1 - w));
        if (variance > maxVariance)
        {
            maxVariance = variance;
            threshold = i;
        }
    }
   // cout<<"gray threthold is "<< threshold << endl;
    return 0;
}

/// \brief ModulesDetect::bgr2binary
/// input color image and output binary image. It can use 2 methods to acheive that.
/// \param srcImage   input
/// \param dstImage    output
/// \param method --1: split channels --2: use canny
/// \return
///
int ModulesDetect::bgr2binary(Mat &srcImage, Mat &dstImage, int method)
{
 // cout<<"ModulesDetect->bgr2binary process is begin"<< endl;

  if (srcImage.empty())
    return -1;
  if(method==1)
  {
    //method 1: split channels and substract
    vector<Mat> imgChannels;
    split(srcImage, imgChannels);
    Mat red_channel = imgChannels.at(2);
    Mat green_channel = imgChannels.at(1);
    Mat blue_channel = imgChannels.at(0);
    Mat mid_channel=blue_channel-red_channel;

    int g_Otsu=0;
    if(Otsu(mid_channel,g_Otsu))
    cout<<"ModulesDetect->Otsu process failed"<< endl;

    threshold(mid_channel, dstImage, 50, 255, CV_THRESH_BINARY);

    //imshow("mid_channel",mid_channel);

  }
 else
  {
     // cvtColor(srcImage, dstImage, COLOR_BGR2GHSV);

      GaussianBlur(dstImage, dstImage, Size(3, 3), 0, 0);

      //int g_Otsu=0;
      // if(Otsu(dstImage,g_Otsu))
      //cout<<"ModulesDetect->Otsu process failed"<< endl;
      // threshold(dstImage, dstImage, g_Otsu, 255, CV_THRESH_BINARY);

      //Canny
      int edgeThresh =80;
      Canny(dstImage, dstImage, edgeThresh, edgeThresh * 3, 3);
  }
 // cout<<"ModulesDetect->bgr2binary process successful"<< endl;
  return 0;
}
///
/// \brief ModulesDetect::GetPixelLength      计算像素坐标距离
/// \param PixelPointA                       第一个点的像素坐标
/// \param PixelPointB                       第二个点的像素坐标
/// \return                                  像素坐标距离
///
float ModulesDetect::GetPixelLength(Point PixelPointA, Point PixelPointB)
{
  float PixelLength;
  PixelLength = powf((PixelPointA.x - PixelPointB.x), 2) + powf((PixelPointA.y - PixelPointB.y), 2);
  PixelLength = sqrtf(PixelLength);
  return PixelLength;
}
//希尔排序
void ShellSort(int* h, size_t len)
{
    if(h==NULL) return;
    if(len<=1) return;

    for(int div=len/2;div>=1;div/=2)
        for(int k=0;k<div;++k)
            for(int i=div+k;i<len;i+=div)
                for(int j=i;j>k;j-=div)
                    if(h[j]<h[j-div]) swap(h[j],h[j-div]);
                    else break;

    return;
}

/// \brief ModulesDetect::find_connected 连通域分析
/// \return   像素坐标
Point ModulesDetect::find_connected(Mat &binary_img)
{
    Mat labels, img_color, stats,centroids;
    Mat binary_inv = ~binary_img;
    int nccomps = cv::connectedComponentsWithStats(binary_inv, labels, stats, centroids);

    //去除过小区域，初始化颜色表
    vector<uchar> colors(nccomps);
    vector<int> Area_line;
    colors[0] = 0; // background pixels remain black.

    vector<int> tgt_lables;
    int area_max = 400, area_min = 300;
    for (int j = 0; j < stats.rows; j++)  //x0,y0,width,height,area
    {
        int unit_x = stats.at<int>(j,0) ,unit_y= stats.at<int>(j,1);
        int area = stats.at<int>(j,4);
        int width = stats.at<int>(j,2), height = stats.at<int>(j,3);
        float wh_ratio = float(width) / float(height);

        if (unit_x==0 && unit_y == 0) //background
        {

            continue;
        }
        if (area < area_min)
        {
            colors[j]=0;
            continue;
        }
        if (wh_ratio < 0.3 || wh_ratio > 3)
        {
            colors[j]=0;
            continue;
        }
        colors[j] = 255;
        tgt_lables.push_back(j);
    }

    //对连通域进行面积排序，保留前3
//    if(tgt_lables.size() < 1)   return Point(0, 0);
//    for(int i=1;i<tgt_lables.size();++i){
//        for(int j=i;j>0;--j){
//            if( stats.at<uchar>(4,tgt_lables[j]) < stats.at<uchar>(4,tgt_lables[j-1]) )
//                swap(stats.at<uchar>(4,tgt_lables[j]),stats.at<uchar>(4,tgt_lables[j-1]));
//        }
//    }
//    for (int i=1; i<tgt_lables.size(); i++)
//    {
//        if(i>3){
//            int j =tgt_lables[i];
//            colors[j]=0;
//            printf("see whats going on");
//        }

//    }

//    if (tgt_lables.size() == 1)
//    {

//        int unit_x = stats.at<uchar>(0,tgt_lables[0]), unit_y = stats.at<uchar>(1,tgt_lables[0]);
//        int area = stats.at<uchar>(4,tgt_lables[0]);
//        int width = stats.at<uchar>(2,tgt_lables[0]), height = stats.at<uchar>(3,tgt_lables[0]);
//        float wh_ratio = float(width) / float(height);
//        Point tgt_point(unit_x+0.5*width,unit_y+0.5*height);


//        return tgt_point;
//    }
    img_color = Mat::zeros(binary_inv.size(), CV_8UC1);
    for( int y = 0; y < img_color.rows; y++ ){
        for( int x = 0; x < img_color.cols; x++ )
        {
            int label = labels.at<int>(y, x);
            CV_Assert(0 <= label && label <= nccomps);
            img_color.at<uchar>(y, x) = colors[label];
        }
    }

    binary_img=img_color;
    return Point(0, 0);
}
/**
* @brief 排序
* @param inputContours    输入轮廓
* @param outputContours   输出轮廓
* @param sortType   排序类型：0-按x排序，1-按y排序，2-按面积排序
* @param sortOrder   排序方式：true-升序，false-降序
*
* @return 返回说明
*     -<em>false</em> fail
*     -<em>true</em> succeed
*/
void  SortContourPoint(vector<vector<Point>> inputContours, vector<vector<Point>> &outputContours, int sortType, bool sortOrder)
{
    vector<Point> tempContoursPoint;

    //计算轮廓矩
    vector<Moments> mu(inputContours.size());
    //计算轮廓的质心
    vector<Point2f> mc(inputContours.size());
    //计算轮廓的面积
    double area, area1;
    //2.2 定义Rect类型的vector容器roRect存放最小外接矩形，初始化大小为contours.size()即轮廓个数
    vector<RotatedRect> roRect(inputContours.size());
    //计算轮廓最小外接矩形中心
    Point center, center1;

    for (int i = 0; i < inputContours.size(); i++)
    {
        tempContoursPoint.clear();    //每次循环注意清空

        mu[i] = moments(inputContours[i], false); //轮廓矩

        mc[i] = Point2d(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00); //轮廓质心

        area = inputContours[i].size(); //轮廓点数
        area = contourArea(inputContours[i], false); //轮廓的面积

        //旋转矩形主要成员有center、size、 angle、points()
        roRect[i] = minAreaRect(Mat(inputContours[i]));
        center = roRect[i].center; //轮廓最小外接矩形中心
        //outputContours[0] = inputContours[1];
    }


    ///冒泡法
    for (int i = 0; i < inputContours.size() - 1; i++)          //n个数要进行n-1趟比较
    {
        for (int j = 0; j < (inputContours.size() - 1) - i; j++)       //每趟比较n-i次
        {
            mu[j] = moments(inputContours[j], false); //轮廓矩
            mu[j+1] = moments(inputContours[j+1], false); //轮廓矩

            mc[j] = Point2d(mu[j].m10 / mu[j].m00, mu[j].m01 / mu[j].m00); //轮廓质心
            mc[j+1] = Point2d(mu[j+1].m10 / mu[j+1].m00, mu[j+1].m01 / mu[j+1].m00); //轮廓质心

            area = inputContours[j].size(); //轮廓点数
            area = contourArea(inputContours[j], false); //轮廓的面积
            area1 = inputContours[j+1].size(); //轮廓点数
            area1 = contourArea(inputContours[j+1], false); //轮廓的面积

            //旋转矩形主要成员有center、size、 angle、points()
            roRect[j] = minAreaRect(Mat(inputContours[j]));
            center = roRect[j].center; //轮廓最小外接矩形中心
            roRect[j+1] = minAreaRect(Mat(inputContours[j+1]));
            center1 = roRect[j+1].center; //轮廓最小外接矩形中心

            if (mc[j].x > mc[j + 1].x)      //依次比较两个相邻的数，将小数放在前面，大数放在后面:升序
            {
                tempContoursPoint = inputContours[j];   //temp是局部变量
                inputContours[j] = inputContours[j + 1];
                inputContours[j + 1] = tempContoursPoint;
            }
        }
    }

    outputContours = inputContours;

}

int ModulesDetect::Get_TargrtRoi(Mat &srcImage ,RotatedRect &TargetRoi )
{
    //find Ins_ROI
    vector<vector<Point>> contours;   //每一组Point点集就是一个轮廓
    vector<Vec4i> hierarcy;           //矩形集合
  //  vector<int> modules_center_candidates;

    findContours(srcImage, contours, hierarcy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE); //找出所有轮廓 包括轮廓关系
    if(contours.size()<=0)
          return -1;
     vector<Rect> box(contours.size()); //定义最小外接矩形集合
 //    vector <Point2f> modulesCenter(contours.size());     //modules中心的点
     int MaxArea = 0;
     int MaxArea_num = 0;

      //绘制轮廓图
      for (int i = 0; i < contours.size(); i++)
      {
        LeafInfo leafInfo;
        leafInfo.ellipseRect = fitEllipse(contours[i]);

        int area = leafInfo.ellipseRect.size.area();
        if (area < 500 || area > (srcImage.cols*srcImage.rows)/2)
        {

            continue;
        }

        if (leafInfo.ellipseRect.size.height > leafInfo.ellipseRect.size.width)
        {
            leafInfo.chang = leafInfo.ellipseRect.size.height;
            leafInfo.kuan = leafInfo.ellipseRect.size.width;

        }
        else
        {
            leafInfo.kuan = leafInfo.ellipseRect.size.height;
            leafInfo.chang = leafInfo.ellipseRect.size.width;

        }

        float w_div_h = leafInfo.chang / leafInfo.kuan;
        if (w_div_h > 2)
        {
            continue;
        }
        if (area > MaxArea){
            MaxArea=area;
            MaxArea_num =i;

        }

        Point2f lf_c_sum(0,0);
        for (int j = 0; j < 4; j++)
        {
             lf_c_sum += leafInfo.vertices[j];
        }
        leafInfo.leaf_center = lf_c_sum / 4;


        box[i] = boundingRect(contours[i]);


//        leafInfo.ellipseRect.points(leafInfo.vertices);
//        float dist_threth = (leafInfo.chang + leafInfo.kuan)/2;
//        for (int i = 0; i < 4; i++)
//        {

//            line(srcImage, leafInfo.vertices[i], leafInfo.vertices[(i + 1) % 4], Scalar(0, 255, 0));
//            float edge_length=GetPixelLength(leafInfo.vertices[i], leafInfo.vertices[(i + 1) % 4]);
//            if (edge_length>1.5*leafInfo.kuan)
//            {
//                Point2f temp_vec = Point2f(leafInfo.vertices[i] - leafInfo.vertices[(i + 1) % 4]);
//                leafInfo.vec_chang = temp_vec / norm(temp_vec);
//            }

        }
      Mat ImageRoi;
      int numOfblue = 0;            //记录颜色的像素点
      float blue_rate = 0;                     //要计算的百分率
      float Max_bluerate = 0;
      int Max_bluenum = 0;
      for(int i=0;i < box.size();i++)
      {
          if(box[i].x == 0 && box[i].y == 0)
              continue;
       //   ImageRoi = srcImage(Rect(box[i].x, box[i].y, box[i].width, box[i].height));
         // Mat imgHSV;

         // cvtColor(ImageRoi, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
          //蓝色的HSV区间
//          int iLowH = 100;
//          int iHighH = 140;

//          int iLowS = 90;
//          int iHighS = 255;

//          int iLowV = 90;
//          int iHighV = 255;

//          for (int rows = 0; rows < imgHSV.rows;rows++)
//          {
//              for (int cols = 0; cols<imgHSV.cols;cols++)   //遍历图片的每一个像素点
//              {
//                  vector<int> colorVec;
//                  colorVec.push_back(imgHSV.at<Vec3b>(rows,cols)[0]);
//                  colorVec.push_back(imgHSV.at<Vec3b>(rows,cols)[1]);
//                  colorVec.push_back(imgHSV.at<Vec3b>(rows,cols)[2]);
//                  if((colorVec[0]>=iLowH&&colorVec[0]<=iHighH)&&(colorVec[1]>=iLowS&&colorVec[1]<=iHighS)&&(colorVec[2]>=iLowV&&colorVec[2]<=iHighV)){

//                         numOfblue++;
//                  }
//              }

//          }
//          blue_rate = (float)numOfblue / (float)(imgHSV.rows * imgHSV.cols);
//          if(Max_bluerate < blue_rate)
//          {
//              Max_bluerate = blue_rate;
//              Max_bluenum = i;
//              printf("The rate:%.2f%%\n", Max_bluerate * 100);

//          }
      }
        //求最小外接矩形
//        Point2f rect[4];
        TargetRoi = minAreaRect(Mat(contours[MaxArea_num]));
//        TargetRoi.points(rect);
//        for (int j = 0; j < 4; j++)
//        {
//          line(srcImage, rect[j], rect[(j + 1) % 4], Scalar(0, 0, 255), 3, 8);  //绘制最小外接矩形每条边
//        }
        return 1;
}

///
/// \brief ModulesDetect::RecognitionFailur
/// \param srcImage
/// \return
///
int ModulesDetect::RecognitionFailure(Mat &srcImage)
{
   // cout<<"ModulesDetect->RecognitionFailure process is begin"<< endl;

    Mat RF_image;
    RotatedRect TargetRoi;
    int Targer_Flag = 0;


    if(bgr2binary(srcImage,RF_image,1))
     cout<<"ModulesDetect->bgr2binary process failed"<< endl;

    //连接连通域
    static Mat kernel_close = getStructuringElement(MORPH_RECT, Size(3,3), Point(-1, -1));
    morphologyEx(RF_image, RF_image, MORPH_DILATE, kernel_close);

    find_connected(RF_image);
    Targer_Flag = Get_TargrtRoi(RF_image,TargetRoi);

    if(Targer_Flag)
    {
        //solve pnp problem
        Point2f Image_Point[4];
      //  Get_ConerPoint(srcImage, TargetRoi, Image_Point[0] );
        TargetRoi.points(Image_Point);
        for (int j = 0; j < 4; j++)
        {
            line(srcImage, Image_Point[j], Image_Point[(j + 1) % 4], Scalar(0, 0, 255), 2, 8);  //绘制最小外接矩形每条边
        }
      //  Calculate_RT(Image_Point);
    }

    namedWindow("find_connected",WINDOW_AUTOSIZE);
    imshow("find_connected",RF_image);
    waitKey(3);
  //  cout<<"ModulesDetect->RecognitionFailure process successful"<< endl;
    return 0;
}
//围绕矩形中心缩放
Rect rectCenterScale(Rect rect, Size size)
{
    rect = rect + size;
    Point pt;
    pt.x = cvRound(size.width / 2.0);
    pt.y = cvRound(size.height / 2.0);
    return (rect - pt);
}
//*函数功能：求两条直线交点*/
Point2f getCrossPoint(Vec4i LineA, Vec4i LineB)
{
    double ka, kb;
    ka = (double)(LineA[3] - LineA[1]) / (double)(LineA[2] - LineA[0]); //求出LineA斜率
    kb = (double)(LineB[3] - LineB[1]) / (double)(LineB[2] - LineB[0]); //求出LineB斜率

    Point2f crossPoint;
    crossPoint.x = (ka*LineA[0] - LineA[1] - kb*LineB[0] + LineB[1]) / (ka - kb);
    crossPoint.y = (ka*kb*(LineA[0] - LineB[0]) + ka*LineB[1] - kb*LineA[1]) / (ka - kb);
    return crossPoint;
}


int  ModulesDetect::Get_ConerPoint(Mat &srcImage, RotatedRect Target_Roi, Point2f &Image_Point)
{
    Point2f RotateRect_point[4];
    Point2f RotateRect_center;
    Target_Roi.points(RotateRect_point);
    RotateRect_center = Point(Target_Roi.center.x, Target_Roi.center.y);


    Rect Target_Rect = Target_Roi.boundingRect();
    Size rate(Target_Rect.width / 5, Target_Rect.height / 5);
    Target_Rect = rectCenterScale(Target_Rect, rate);
    Mat rect_check = srcImage(Target_Rect);
    Mat side_image;

    cvtColor(rect_check, rect_check, COLOR_BGR2GRAY);
    GaussianBlur(rect_check, rect_check, Size(3, 3), 0, 0);
    //bgr2binary(rect_check,side_image,1);
    //定义核
    //Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
    //进行形态学开运算操作
   // morphologyEx(side_image, side_image, MORPH_OPEN, element);
    //边缘检测
    Canny(rect_check, side_image, 60, 150, 3);
    /*霍夫直线检测*/
    vector<Vec4i> Lines;
    HoughLinesP(side_image, Lines, 1, CV_PI / 360, 200, 100, 10);
    Vec4i LineStand = Lines[0];
    Vec4i LineAnother;
    double ka = (double)(LineStand[1] - LineStand[3]) / (double)(LineStand[0] - LineStand[2]);
    double kb;
    for (int i = 1; i < Lines.size(); i++)
    {
        double ki = (double)(Lines[i][1] - Lines[i][3]) / (double)(Lines[i][0] - Lines[i][2]);
        if (ki*ka < 0)
        {
            LineAnother = Lines[i];
            kb = ki;
        }
    }


    imshow("Get_ConerPoint",side_image);
    imshow("Target_Rect",rect_check);

}

