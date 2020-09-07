/******************************************************************************
* Function: Blue box detection and feature point extraction                  *
* Author :  Jiduocai Yang                                                    *
* Contact:  jiduocaiyang@gmail.com                                           *
* Address:  NUAA                                                             *
******************************************************************************/
#include "ModulesDetect.h"


// gamma correction
cv::Mat gamma_correction(cv::Mat img, double gamma_c, double gamma_g){
  // get height and width
  int width = img.cols;
  int height = img.rows;
  int channel = img.channels();

  // output image
  cv::Mat out = cv::Mat::zeros(height, width, CV_8UC3);

  double val;

  // gamma correction
  for (int y = 0; y< height; y++){
    for (int x = 0; x < width; x++){
      for (int c = 0; c < channel; c++){
          val = (double)img.at<cv::Vec3b>(y, x)[c] / 255;

          out.at<cv::Vec3b>(y, x)[c] = (uchar)(pow(val / gamma_c, 1 / gamma_g) * 255);
      }
    }
  }

  return out;
}


void MyGammaCorrection(Mat& src, Mat& dst11, float fGamma)
{

    // build look up table
    unsigned char lut[256];
    for( int i = 0; i < 256; i++ )
    {
        lut[i] = saturate_cast<uchar>(pow((float)(i/255.0), fGamma) * 255.0f);
    }

    dst11 = src.clone();
    const int channels = dst11.channels();
    switch(channels)
    {
        case 3:  //彩色图的情况
            {

                MatIterator_<Vec3b> it, end;
                for( it = dst11.begin<Vec3b>(), end = dst11.end<Vec3b>(); it != end; it++ )
                {
                    //(*it)[0] = pow((float)(((*it)[0])/255.0), fGamma) * 255.0;
                    //(*it)[1] = pow((float)(((*it)[1])/255.0), fGamma) * 255.0;
                    //(*it)[2] = pow((float)(((*it)[2])/255.0), fGamma) * 255.0;
                    (*it)[0] = lut[((*it)[0])];
                    (*it)[1] = lut[((*it)[1])];
                    (*it)[2] = lut[((*it)[2])];
                }

                break;

            }
    }
}

static inline bool ContoursSortFun(vector<cv::Point> contour1, vector<cv::Point> contour2)
{

    return (cv::contourArea(contour1) > cv::contourArea(contour2));

}



int ModulesDetect::Otsu(Mat &srcImage , int &threshold)  // 输入为原图像，输出为阈值
{

    int height = srcImage.rows;    //图像的高
    int width = srcImage.cols;     //图像的宽

    //histogram                   //直方图
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

/// \brief ModulesDetect::bgr2binary   二值化
/// input color image and output binary image. It can use 2 methods to acheive that.
/// \param srcImage   input
/// \param dstImage    output
/// \param method --1: split channels --2: use HSV
/// \return
///
int ModulesDetect::bgr2binary(Mat &srcImage, Mat &dstImage, int method)
{
 // cout<<"ModulesDetect->bgr2binary process is begin"<< endl;



  if (srcImage.empty())
    return 0;
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

    threshold(mid_channel, dstImage, g_Otsu, 255, CV_THRESH_BINARY);
  }

 else
  {
      //蓝色的HSV范围
      int iLowH = 100;
      int iHighH = 124;

      int iLowS = 43;
      int iHighS = 255;

      int iLowV = 46;
      int iHighV = 255;
      Mat HSVImage;

        //MyGammaCorrection(srcImage , fGamma);
      //circle(srcImage, Point(srcImage.cols/ 2, srcImage.rows / 2), 150, Scalar(255, 0, 0),200);
      cvtColor(srcImage, HSVImage, COLOR_BGR2HSV);    //将RGB图像转化为HSV

      inRange(HSVImage,Scalar(iLowH,iLowS,iLowV),Scalar(iHighH,iHighS,iHighV),dstImage);     //找寻在要求区间内的颜色,Binarization
      // inRange：二值化，主要是将在两个阈值内的像素值设置为白色（255），而不在阈值区间内的像素值设置为黑色（0），该功能类似于之间所讲的双阈值化操作。

      int g_Otsu=0;
      if(Otsu(dstImage,g_Otsu))
         cout<<"ModulesDetect->Otsu process failed"<< endl;

      threshold(dstImage, dstImage, g_Otsu*0.5, 255, CV_THRESH_BINARY);   //利用Otsu求得的阈值g_Otsu进行二值化

  }
  //imshow("Otsu process ",dstImage);
 // cout<<"ModulesDetect->bgr2binary process successful"<< endl;

  return 0;
}

///
/// \brief ModulesDetect::GetPixelLength     计算像素坐标距离
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

/// \brief ModulesDetect::find_connected 连通域分析
/// \return   像素坐标
Point ModulesDetect::find_connected(Mat &binary_img)    //无损滤波
{
    Mat labels, img_color, stats,centroids;
    int nccomps = cv::connectedComponentsWithStats(binary_img, labels, stats, centroids); //用于过滤原始图像中轮廓分析后较小的区域，留下较大区域。

    //去除过小区域，初始化颜色表
    vector<uchar> colors(nccomps);
    vector<int> Area_line;
    colors[0] = 0; // background pixels remain black.

    vector<int> tgt_lables;
    int  area_min = 300;
    for (int j = 0; j < stats.rows; j++)  //x0,y0,width,height,area
    {
        int unit_x = stats.at<int>(j,0) ,unit_y= stats.at<int>(j,1);
        int area = stats.at<int>(j,4);
//        int width = stats.at<int>(j,2), height = stats.at<int>(j,3);
//        float wh_ratio = float(width) / float(height);

        if (unit_x==0 && unit_y == 0) //background
        {

            continue;
        }
        if (area < area_min)
        {
            colors[j]=0;
            continue;
        }
//        if (wh_ratio > 4)
//        {
//            colors[j]=0;
//            continue;
//        }
        colors[j] = 255;
//        tgt_lables.push_back(j);
    }

    img_color = Mat::zeros(binary_img.size(), CV_8UC1);
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

    for (uint i = 0; i < inputContours.size(); i++)
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
    for (uint i = 0; i < inputContours.size() - 1; i++)          //n个数要进行n-1趟比较
    {
        for (uint j = 0; j < (inputContours.size() - 1) - i; j++)       //每趟比较n-i次
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

int ModulesDetect::Get_TargrtRoi(Mat &srcImage ,Mat &grayImage ,RotatedRect &TargetRoi )  //返回蓝色占比最高的矩形框
{
    //find Ins_ROI
    vector<vector<Point>> contours;   //每一组Point点集就是一个轮廓
    vector<Vec4i> hierarcy;           //矩形集合


    findContours(grayImage, contours, hierarcy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE); //找出所有轮廓 包括轮廓关系
    //参数1：grayImage二值图像，参数2：contours定义为“vector<vector<Point>> contours”，是一个双重向量
    //向量内每个元素保存了一组由连续的Point构成的点的集合的向量，每一组点集就是一个轮廓，有多少轮廓，contours就有多少元素
    //参数3：hierarchy定义为“vector<Vec4i> hierarchy，向量hierarchy内的元素和轮廓向量contours内的元素是一一对应的，向量的容量相同
    //参数4：定义轮廓的检索模式，CV_RETR_EXTERNAL：只检测最外围轮廓，包含在外围轮廓内的内围轮廓被忽略；
    //参数5：定义轮廓的近似方法，CV_CHAIN_APPROX_NONE：保存物体边界上所有连续的轮廓点到contours向量内；

    if(contours.size()<=0)
          return 0;

    for (int i = 0; i < contours.size(); i++)
    {
        std::sort(contours.begin(), contours.end(), ContoursSortFun);
        break;
    }
    uint Maxboxnum = 0;
    if(contours.size()<=3)
        Maxboxnum = contours.size();
    else
         Maxboxnum = 3;

    vector<Rect> box(contours.size()); //定义最小外接矩形集合
    for (uint i = 0; i < Maxboxnum; i++)
    {
        LeafInfo leafInfo;
        leafInfo.ellipseRect = fitEllipse(contours[i]);  //椭圆拟合

        int area = leafInfo.ellipseRect.size.area();
        if (area < 300 || area > (grayImage.cols*grayImage.rows)/2)
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
        Point2f lf_c_sum(0,0);
        for (int j = 0; j < 4; j++)
        {
            lf_c_sum += leafInfo.vertices[j];
        }
        leafInfo.leaf_center = lf_c_sum / 4;


        box[i] = boundingRect(contours[i]);

    }
    Mat ImageRoi;
    int numOfblue = 0;            //记录颜色的像素点
    float blue_rate = 0;          //要计算的百分率
    float Max_bluerate = 0;
    int Max_bluenum = 0;

    //蓝色的HSV范围
    int iLowH = 100;
    int iHighH = 124;

    int iLowS = 43;
    int iHighS = 255;

    int iLowV = 46;
    int iHighV = 255;

    for(uint i=0;i < box.size();i++)
    {
        if(box[i].x == 0 && box[i].y == 0)
            continue;
        ImageRoi = srcImage(Rect(box[i].x, box[i].y, box[i].width, box[i].height));
        Mat HsvImage ;
        ImageRoi.copyTo(HsvImage);
        cvtColor(HsvImage, HsvImage, CV_BGR2HSV);//CV_BGR2HSV
        numOfblue = 0;

        for(int m = 0; m < HsvImage.rows; m++)
        {
            for(int n = 0; n < HsvImage.cols; n++)
            {

                if((HsvImage.at<Vec3b>(m,n)[0] >= iLowH && HsvImage.at<Vec3b>(m,n)[0] <= iHighH) &&
                        (HsvImage.at<Vec3b>(m,n)[1] >= iLowS && HsvImage.at<Vec3b>(m,n)[1] <= iHighS ) &&
                        (HsvImage.at<Vec3b>(m,n)[2] >= iLowV && HsvImage.at<Vec3b>(m,n)[2] <= iHighV ) )

                    numOfblue++;
            }
        }

        blue_rate = (float)numOfblue / (float)(HsvImage.rows * HsvImage.cols);
        if(Max_bluerate < blue_rate)
        {
            Max_bluerate = blue_rate;
            Max_bluenum = i;
        }
    }
    if(Max_bluerate >= 0.4)
    {
        //  printf("The rate:%.2f%%\n", Max_bluerate * 100);
        TargetRoi = minAreaRect(Mat(contours[Max_bluenum]));
        //   ImageRoi = srcImage(Rect(box[Max_bluenum].x, box[Max_bluenum].y, box[Max_bluenum].width, box[Max_bluenum].height));
        //   imshow("ImageRoi",ImageRoi);
        return 1;
    }
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
int ModulesDetect::RotatePoint(Point2f &ptSrc, Point2f &ptRotation, double &angle)
{
    //其中圆心（a,b),  圆上一点坐标(x0,y0), 旋转角度α
    //那么旋转后的坐标 x=a+(x0-a)cosα-(y0-b)sinα ,    y=b+(x0-a)sinα+(y0-b)cosα
    float a = 0;
    float b = 0;
    float x0 = ptSrc.x;
    float y0 = ptSrc.y;
    ptRotation.x = a + (x0-a) * cos(angle * M_PI / 180) - (y0-b) * sin(angle * M_PI / 180);
    ptRotation.y = b + (x0-a) * sin(angle * M_PI / 180) + (y0-b) * cos(angle * M_PI / 180);
    return 1;

}
//寻找箱子角点
int  ModulesDetect::Get_ConerPoint(Mat &srcImage, RotatedRect &Target_Roi, vector<Point2f> &Image_Point)
{
    Point2f RotateRect_point[4];
    Point2f RotateRect_center;
    Target_Roi.points(RotateRect_point);
    RotateRect_center = Point(Target_Roi.center.x, Target_Roi.center.y);
    double RotateRect_degree = Target_Roi.angle;
   // cout << "RotateRect_degree: "  << RotateRect_degree << endl;
    //Mat RotateRect_rotm = getRotationMatrix2D(RotateRect_center, RotateRect_degree, 1.0);

    //use the RotateRect_point as image point
    float distance_long = GetPixelLength(RotateRect_point[0],RotateRect_point[3]);
    float distance_short = GetPixelLength(RotateRect_point[0],RotateRect_point[1]);
    if(distance_short > distance_long)
    {
        Point2f point =  RotateRect_point[0];
        RotateRect_point[0] = RotateRect_point[1];
        RotateRect_point[1] = RotateRect_point[2];
        RotateRect_point[2] = RotateRect_point[3];
        RotateRect_point[3] = point;

        float distance_mid = distance_long;
        distance_long  = distance_short;
        distance_short = distance_mid;
    }
    for(int i = 0;i < 4; i++ )
    {
        Image_Point.push_back(RotateRect_point[i]);
    }
        Image_Point.push_back(RotateRect_center);


    Point2f Rotation_p[4];
    Point2f Rotation_pr[4];// for a new line
    for(int i = 0;i < 4; i++ )
    {
        Rotation_p[i].x = RotateRect_point[i].x - RotateRect_center.x;
        Rotation_p[i].y = -(RotateRect_point[i].y - RotateRect_center.y);
        RotatePoint(Rotation_p[i],Rotation_p[i],RotateRect_degree);
        Point2f corners_R =Rotation_p[i];
        if(corners_R.x < 0 && corners_R.y > 0)
        {
            Rotation_pr[0] =corners_R;   //LU
        }
        if(corners_R.x < 0 && corners_R.y < 0)
        {
            Rotation_pr[1] =corners_R;   //LD
        }
        if(corners_R.x > 0 && corners_R.y < 0)
        {
            Rotation_pr[2] =corners_R;   //RU
        }
        if(corners_R.x > 0 && corners_R.y > 0)
        {
            Rotation_pr[3] =corners_R;   //RD
        }

    }
 

    //For ROI Image
    Rect Target_Rect = Target_Roi.boundingRect();
    //Size rate(Target_Rect.width / 7, Target_Rect.height / 7);
    //Target_Rect = rectCenterScale(Target_Rect, rate);
    if(Target_Rect.x < 0) Target_Rect.x = 0;
    if(Target_Rect.y < 0) Target_Rect.y = 0;
    if(Target_Rect.x+Target_Rect.width >= srcImage.cols)
    {
        Target_Rect.width = srcImage.cols-Target_Rect.x-1;
    }
    if(Target_Rect.y+Target_Rect.height >= srcImage.rows)
    {
        Target_Rect.height = srcImage.rows-Target_Rect.y-1;
    }
    //blue rect box in srcImage
  
    Mat rect_check = srcImage(Target_Rect);
    Mat ROI_image;
    Mat V2Image;
    rect_check.copyTo(ROI_image);

    //hsv method
    bgr2binary(ROI_image,V2Image,2);
    static Mat kernel_close = getStructuringElement(MORPH_RECT, Size(3,3), Point(-1, -1));
    morphologyEx(V2Image, V2Image, MORPH_DILATE, kernel_close);

    //Shi-Tomasi算法
    vector<Point2f> corners;//提供初始角点的坐标位置和精确的坐标的位置
    int maxcorners = 100;
    double qualityLevel = 0.01;  //角点检测可接受的最小特征值
    double minDistance = distance_short*0.3;	//角点之间最小距离
    int blockSize = 3;//计算导数自相关矩阵时指定的领域范围
    double  k = 0.04; //权重系数

    goodFeaturesToTrack(V2Image, corners, maxcorners, qualityLevel, minDistance, Mat(), blockSize, false, k);
    //Mat():表示感兴趣区域；false:表示不用Harris角点检测
     Point2f ROI_RotateRectCenter;
     ROI_RotateRectCenter.x = RotateRect_center.x-Target_Rect.x;
     ROI_RotateRectCenter.y = RotateRect_center.y-Target_Rect.y;
     int MinDistanceLU = 1000;
     int MinDistanceLD = 1000;
     int MinDistanceRU = 1000;
     int MinDistanceRD = 1000;
     Point2f Imagepoint_corners[4];
    //find best four points in corners
    for (unsigned i = 0; i < corners.size(); i++)
    {
        Point2f corners_R;
        corners_R.x = corners[i].x - ROI_RotateRectCenter.x;
        corners_R.y = -(corners[i].y - ROI_RotateRectCenter.y);

        RotatePoint(corners_R,corners_R,RotateRect_degree);
        float distance;
        if(corners_R.x < 0 && corners_R.y > 0)
        {
            distance = GetPixelLength(corners_R,Rotation_pr[0]);
            if (distance < MinDistanceLU)
            {
                MinDistanceLU = distance;
                Imagepoint_corners[0] = corners[i];
                continue;
            }
        }
        if(corners_R.x < 0 && corners_R.y < 0)
        {
            distance = GetPixelLength(corners_R,Rotation_pr[1]);
            if (distance < MinDistanceLD)
            {
                MinDistanceLD =distance;
                Imagepoint_corners[1] = corners[i];
                continue;
            }
        }
        if(corners_R.x > 0 && corners_R.y < 0)
        {
            distance = GetPixelLength(corners_R,Rotation_pr[2]);
            if (distance < MinDistanceRD)
            {
                MinDistanceRD =distance;
                Imagepoint_corners[2] = corners[i];
                continue;
            }

        }
        if(corners_R.x > 0 && corners_R.y > 0)
        {
            distance = GetPixelLength(corners_R,Rotation_pr[3]);
            if (distance < MinDistanceRU)
            {
                MinDistanceRU =distance;
                Imagepoint_corners[3] = corners[i];
                continue;
            }
        }
    }
    int distance_error[4];
    distance_error[0] = MinDistanceLU;
    distance_error[1] = MinDistanceLD;
    distance_error[2] = MinDistanceRU;
    distance_error[3] = MinDistanceRD;
    //find the best two points in four points
    Point2f bestPoints [2];
    for (unsigned n = 0; n < 4; n++)
    {
        for(unsigned i = 0; i < 4; i++)
        {
            if(distance_error[i] > distance_error[i+1])
                swap(distance_error[i],distance_error[i+1]);
        }
    }

    //绘制角点
    for (unsigned i = 0; i <4; i++)
    {
        //cout<<"distance "<< i << " =" << distance_error[i] <<endl;
       // cout<<"distance "<< i << " =" << Imagepoint_corners[i] <<endl;
        if(distance_error[i] <= 5)
        //points not change after the distance change
        circle(rect_check, Imagepoint_corners[i], 5, cv::Scalar(0, 0, 255), 2, 8, 0);

    }


}


void contrast(Mat &srcImage1,Mat &srcImage)
{
    int height = srcImage1.rows;//求出src1的高
    int width = srcImage1.cols;//求出src1的宽
    srcImage = Mat::zeros(srcImage1.size(),srcImage1.type());  //这句很重要，创建一个与原图一样大小的空白图片
    float alpha = 1.5;//调整对比度为1.5
    //循环操作，遍历每一列，每一行的元素
    for(int row = 0;row < height; row++)
    {
        for(int col = 0;col < width;col++)
        {
            if(srcImage1.channels() == 3)//判断是否为3通道图片
            {
                //将遍历得到的原图像素值，返回给变量b,g,r
                float b = srcImage1.at<Vec3b>(row,col)[0];//nlue
                float g = srcImage1.at<Vec3b>(row,col)[1];//green
                float r = srcImage1.at<Vec3b>(row,col)[2];//red
                //开始操作像素，对变量b,g,r做改变后再返回到新的图片。
                srcImage.at<Vec3b>(row,col)[0] = saturate_cast<uchar>(b*alpha);
                srcImage.at<Vec3b>(row,col)[1] = saturate_cast<uchar>(g*alpha);
                srcImage.at<Vec3b>(row,col)[2] = saturate_cast<uchar>(r*alpha);
            }
        }
    }

}
/// \brief ModulesDetect::RecognitionFailur
/// \param srcImage
/// \return
///
int ModulesDetect::RecognitionFailure(Mat &srcImage,RotatedRect &TargetRoi,vector<Point2f> &Image_Point)
{
   // cout<<"ModulesDetect->RecognitionFailure process is begin"<< endl;

    Mat grayImage;
  //  RotatedRect TargetRoi;
    int Targer_Flag = 0;


    if(bgr2binary(srcImage,grayImage,2))
     cout<<"ModulesDetect->bgr2binary process failed"<< endl;
    //连接连通域
    static Mat kernel_close = getStructuringElement(MORPH_RECT, Size(3,3), Point(-1, -1));
    morphologyEx(grayImage, grayImage, MORPH_DILATE, kernel_close);

    find_connected(grayImage);
    detect_frame = grayImage;

    Targer_Flag = Get_TargrtRoi(srcImage,grayImage,TargetRoi);
    if(Targer_Flag)
    {
        //find image points
        Get_ConerPoint(srcImage, TargetRoi, Image_Point);
        for(uint i = 0; i < Image_Point.size(); i++)
        {
            circle(srcImage, Image_Point[i], 3, Scalar(255,0,0),-1);

        }

        //line the TargetRoi with red box
        Point2f TargetRoi_Points[4];
        TargetRoi.points(TargetRoi_Points);
        for (int j = 0; j < 4; j++)
        {
            line(srcImage, TargetRoi_Points[j], TargetRoi_Points[(j + 1) % 4], Scalar(0, 0, 255), 1, 8);  //绘制最小外接矩形每条边
        }
        return 1;
    }
    else
    {
          // cout<<"Hard to find the TargetRoi,please check Get_TargrtRoi process"<< endl;
    }
    return 0;
}


int ModulesDetect::Bluebox_Detection(Mat &srcImage)
{

    RotatedRect TargetRoi;
    vector<Point2f> Image_Point;
    int d_x;
    int d_y;
    String sd_x,sd_y;


    circle(srcImage, Point(640,512), 3, Scalar(0, 255, 0), 8);
    if(RecognitionFailure(srcImage,TargetRoi,Image_Point))
    {
        line(srcImage,  Point(640,512),  Point(Image_Point[4].x, Image_Point[4].y), Scalar(0, 0, 255), 2, 8);
        d_x = int(Image_Point[4].x)-640;
        d_y = int(Image_Point[4].y)-512;
      
        // solve pnp problem
       // Calculate_RT(Image_Point, BoxPosition);
    }
    else
    {
        d_x = 0;
        d_y = 0;

    }

    sd_x = std::to_string(d_x);
    sd_y = std::to_string(d_y);

    putText(srcImage, "d_x",  Point2f(200, 50 ), CV_FONT_HERSHEY_COMPLEX_SMALL, 1.8, Scalar(0,0,255),2,8);
    putText(srcImage, sd_x,   Point2f(340, 50 ), CV_FONT_HERSHEY_COMPLEX_SMALL, 1.8, Scalar(0,0,255),2,8);
    putText(srcImage, "d_y",  Point2f(200, 100), CV_FONT_HERSHEY_COMPLEX_SMALL, 1.8, Scalar(0,0,255),2,8);
    putText(srcImage, sd_y,   Point2f(340, 100), CV_FONT_HERSHEY_COMPLEX_SMALL, 1.8, Scalar(0,0,255),2,8);



    return 0;
}

int Bluebox_Detection(Mat &srcImage)
{


    cout << "in_ModulesDetect" << endl;

    //*******************input test code**************************

    //Mat src = srcImage.clone();
    Mat dstImage;
    float fGamma = 1.8;


    Mat srcImage1;
    srcImage.copyTo(srcImage1);
    MyGammaCorrection(srcImage1, srcImage,fGamma);

    //bgr2binary(srcImage,dstImage,2);

    Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
    erode(dstImage, dstImage, element);
    //Mat dilatedImage;
    dilate(dstImage, dstImage, element);
    vector<vector<Point>> contours;

    vector<Vec4i> hierarcy;
    findContours(dstImage, contours, hierarcy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    vector<Rect> boundRect(contours.size());
    vector<RotatedRect> box(contours.size());
    Point2f rect[4];
    //cout << "strat_for_soon" << endl;
    //char width[20], height[20];
    float angle_rotation;
    float angle_rotation1;
    int d_x;
    int d_y;
    String sd_x,sd_y;

    if(contours.size()>0)
    {
        for (int i = 0; i < contours.size(); i++)
        {
            std::sort(contours.begin(), contours.end(), ContoursSortFun);

            break;
        }

        box[0] = minAreaRect(Mat(contours[0]));
        boundRect[0] = boundingRect(Mat(contours[0]));
        circle(srcImage, Point(box[0].center.x, box[0].center.y), 3, Scalar(0, 255, 0), 8);
        circle(srcImage, Point(640,512), 3, Scalar(0, 255, 0), 8);
        box[0].points(rect);
        circle(srcImage, Point(rect[0].x, rect[0].y), 3, Scalar(255, 0, 0), 8);
        circle(srcImage, Point(rect[1].x, rect[1].y), 3, Scalar(0, 255, 0), 8);
        circle(srcImage, Point(rect[2].x, rect[2].y), 3, Scalar(139, 0, 139), 8);
        circle(srcImage, Point(rect[3].x, rect[3].y), 3, Scalar(79, 79, 79), 8);
        line(srcImage,  Point(640,512),  Point(box[0].center.x, box[0].center.y), Scalar(0, 0, 255), 2, 8);
        //rectangle(dstImg, Point(boundRect[i].x, boundRect[i].y), Point(boundRect[i].x + boundRect[i].width, boundRect[i].y + boundRect[i].height), Scalar(0, 255, 0), 2, 8);
        for (int j = 0; j < 4; j++)
        {
            line(srcImage, rect[j], rect[(j + 1) % 4], Scalar(0, 0, 255), 2, 8);
        }

        //z_s是左上，z_x是左下，y_s是右上，y_x是右下，zx_x是中心x值，zx_y是中心y值
        String z_s_x,z_s_y,z_x_x,z_x_y,y_s_x,y_s_y,y_x_x,y_x_y,zx_x,zx_y;

        z_s_x = std::to_string(int(rect[2].x));
        z_s_y = std::to_string(int(rect[2].y));
        z_x_x = std::to_string(int(rect[1].x));
        z_x_y = std::to_string(int(rect[1].y));

        y_s_x = std::to_string(int(rect[3].x));
        y_s_y = std::to_string(int(rect[3].y));
        y_x_x = std::to_string(int(rect[0].x));
        y_x_y = std::to_string(int(rect[0].y));

        zx_x = std::to_string(int(box[0].center.x));
        zx_y = std::to_string(int(box[0].center.y));


        d_x = int(box[0].center.x)-640;
        d_y = int(box[0].center.y)-512;

        sd_x = std::to_string(d_x);
        sd_y = std::to_string(d_y);

        if ((box[0].size.width / box[0].size.height) < 1)
        {
            String angle;

            angle_rotation = 90 + box[0].angle;//正数，逆时针旋转
            angle=std::to_string(float(angle_rotation));

            putText(srcImage, "angle", Point2f(100, 100), CV_FONT_HERSHEY_COMPLEX_SMALL, 2, Scalar(0,0,255),2,8);
            putText(srcImage, angle,  Point2f(250, 100), CV_FONT_HERSHEY_COMPLEX_SMALL, 2, Scalar(0,0,255),2,8);

            putText(srcImage, "z_s",  Point2f(100, 150), CV_FONT_HERSHEY_COMPLEX_SMALL, 2, Scalar(0,0,255),2,8);
            putText(srcImage, z_s_x,  Point2f(200, 150), CV_FONT_HERSHEY_COMPLEX_SMALL, 2, Scalar(0,0,255),2,8);
            putText(srcImage, z_s_y,  Point2f(340, 150), CV_FONT_HERSHEY_COMPLEX_SMALL, 2, Scalar(0,0,255),2,8);

            putText(srcImage, "z_x",  Point2f(100, 200), CV_FONT_HERSHEY_COMPLEX_SMALL, 1.8, Scalar(0,0,255),2,8);
            putText(srcImage, z_x_x,  Point2f(200, 200), CV_FONT_HERSHEY_COMPLEX_SMALL, 1.8, Scalar(0,0,255),2,8);
            putText(srcImage, z_x_y,  Point2f(340, 200), CV_FONT_HERSHEY_COMPLEX_SMALL, 1.8, Scalar(0,0,255),2,8);

            putText(srcImage, "y_s",  Point2f(100, 250), CV_FONT_HERSHEY_COMPLEX_SMALL, 1.8, Scalar(0,0,255),2,8);
            putText(srcImage, y_s_x,  Point2f(200, 250), CV_FONT_HERSHEY_COMPLEX_SMALL, 1.8, Scalar(0,0,255),2,8);
            putText(srcImage, y_s_y,  Point2f(340, 250), CV_FONT_HERSHEY_COMPLEX_SMALL, 1.8, Scalar(0,0,255),2,8);

            putText(srcImage, "y_x",  Point2f(100, 300), CV_FONT_HERSHEY_COMPLEX_SMALL, 1.8, Scalar(0,0,255),2,8);
            putText(srcImage, y_x_x,  Point2f(200, 300), CV_FONT_HERSHEY_COMPLEX_SMALL, 1.8, Scalar(0,0,255),2,8);
            putText(srcImage, y_x_y,  Point2f(340, 300), CV_FONT_HERSHEY_COMPLEX_SMALL, 1.8, Scalar(0,0,255),2,8);

            putText(srcImage, "zx",  Point2f(100, 350), CV_FONT_HERSHEY_COMPLEX_SMALL, 1.8, Scalar(0,0,255),2,8);
            putText(srcImage, zx_x,  Point2f(200, 350), CV_FONT_HERSHEY_COMPLEX_SMALL, 1.8, Scalar(0,0,255),2,8);
            putText(srcImage, zx_y,  Point2f(340, 350), CV_FONT_HERSHEY_COMPLEX_SMALL, 1.8, Scalar(0,0,255),2,8);

            putText(srcImage, "d_x",  Point2f(200, 400), CV_FONT_HERSHEY_COMPLEX_SMALL, 1.8, Scalar(0,0,255),2,8);
            putText(srcImage, sd_x,  Point2f(340, 400), CV_FONT_HERSHEY_COMPLEX_SMALL, 1.8, Scalar(0,0,255),2,8);

            putText(srcImage, "d_y",  Point2f(200, 450), CV_FONT_HERSHEY_COMPLEX_SMALL, 1.8, Scalar(0,0,255),2,8);
            putText(srcImage, sd_y,  Point2f(340, 450), CV_FONT_HERSHEY_COMPLEX_SMALL, 1.8, Scalar(0,0,255),2,8);

           }
        else
        {
            String angle1;

            angle_rotation1 = box[0].angle; //负数，顺时针旋
            angle1=std::to_string(float(angle_rotation1));
            putText(srcImage, "angle1", Point2f(100, 100), CV_FONT_HERSHEY_COMPLEX_SMALL, 2, Scalar(0,0,255),2,8);
            putText(srcImage, angle1,  Point2f(250, 100), CV_FONT_HERSHEY_COMPLEX_SMALL, 2, Scalar(0,0,255),2,8);

            putText(srcImage, "z_s",  Point2f(100, 150), CV_FONT_HERSHEY_COMPLEX_SMALL, 2, Scalar(0,0,255),2,8);
            putText(srcImage, z_s_x,  Point2f(200, 150), CV_FONT_HERSHEY_COMPLEX_SMALL, 2, Scalar(0,0,255),2,8);
            putText(srcImage, z_s_y,  Point2f(340, 150), CV_FONT_HERSHEY_COMPLEX_SMALL, 2, Scalar(0,0,255),2,8);

            putText(srcImage, "z_x",  Point2f(100, 200), CV_FONT_HERSHEY_COMPLEX_SMALL, 1.8, Scalar(0,0,255),2,8);
            putText(srcImage, z_x_x,  Point2f(200, 200), CV_FONT_HERSHEY_COMPLEX_SMALL, 1.8, Scalar(0,0,255),2,8);
            putText(srcImage, z_x_y,  Point2f(340, 200), CV_FONT_HERSHEY_COMPLEX_SMALL, 1.8, Scalar(0,0,255),2,8);

            putText(srcImage, "y_s",  Point2f(100, 250), CV_FONT_HERSHEY_COMPLEX_SMALL, 1.8, Scalar(0,0,255),2,8);
            putText(srcImage, y_s_x,  Point2f(200, 250), CV_FONT_HERSHEY_COMPLEX_SMALL, 1.8, Scalar(0,0,255),2,8);
            putText(srcImage, y_s_y,  Point2f(340, 250), CV_FONT_HERSHEY_COMPLEX_SMALL, 1.8, Scalar(0,0,255),2,8);

            putText(srcImage, "y_x",  Point2f(100, 300), CV_FONT_HERSHEY_COMPLEX_SMALL, 1.8, Scalar(0,0,255),2,8);
            putText(srcImage, y_x_x,  Point2f(200, 300), CV_FONT_HERSHEY_COMPLEX_SMALL, 1.8, Scalar(0,0,255),2,8);
            putText(srcImage, y_x_y,  Point2f(340, 300), CV_FONT_HERSHEY_COMPLEX_SMALL, 1.8, Scalar(0,0,255),2,8);

            putText(srcImage, "zx",  Point2f(100, 350), CV_FONT_HERSHEY_COMPLEX_SMALL, 1.8, Scalar(0,0,255),2,8);
            putText(srcImage, zx_x,  Point2f(200, 350), CV_FONT_HERSHEY_COMPLEX_SMALL, 1.8, Scalar(0,0,255),2,8);
            putText(srcImage, zx_y,  Point2f(340, 350), CV_FONT_HERSHEY_COMPLEX_SMALL, 1.8, Scalar(0,0,255),2,8);

            putText(srcImage, "d_x",  Point2f(200, 400), CV_FONT_HERSHEY_COMPLEX_SMALL, 1.8, Scalar(0,0,255),2,8);
            putText(srcImage, sd_x,  Point2f(340, 400), CV_FONT_HERSHEY_COMPLEX_SMALL, 1.8, Scalar(0,0,255),2,8);

            putText(srcImage, "d_y",  Point2f(200, 450), CV_FONT_HERSHEY_COMPLEX_SMALL, 1.8, Scalar(0,0,255),2,8);
            putText(srcImage, sd_y,  Point2f(340, 450), CV_FONT_HERSHEY_COMPLEX_SMALL, 1.8, Scalar(0,0,255),2,8);

      }

    }
    return 0;

}
