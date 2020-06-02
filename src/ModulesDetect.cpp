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

    threshold(mid_channel, dstImage, g_Otsu, 255, CV_THRESH_BINARY);

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
   // Mat kernel = (Mat_<float>(2, 2) << 2, 7, 10, 0) ;
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
        if (wh_ratio < 0.5 || wh_ratio > 2)
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
int ModulesDetect::Get_TargrtRoi(Mat &srcImage ,RotatedRect &TargetRoi )
{
    //find Ins_ROI
    vector<vector<Point>> contours;   //每一组Point点集就是一个轮廓
    vector<Vec4i> hierarcy;           //矩形集合
  //  vector<int> modules_center_candidates;

    findContours(srcImage, contours, hierarcy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE); //找出所有轮廓 包括轮廓关系
    if(contours.size()<=0)
          return -1;
     vector<RotatedRect> box(contours.size()); //定义最小外接矩形集合
 //    vector <Point2f> modulesCenter(contours.size());     //modules中心的点
     int MaxArea = 0;
     int MaxArea_num = 0;
     vector<RotRect> Target_ROI ;
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
//        if (area > MaxArea){
//            MaxArea=area;
//            MaxArea_num =i;

//        }

        Point2f lf_c_sum(0,0);
        for (int j = 0; j < 4; j++)
        {
             lf_c_sum += leafInfo.vertices[j];
        }
        leafInfo.leaf_center = lf_c_sum / 4;

        RotRect Rot_Rect;
        Rot_Rect.center = leafInfo.leaf_center;
        Rot_Rect.width  = leafInfo.kuan;
        Rot_Rect.height  = leafInfo.chang;
        Target_ROI.push_back(Rot_Rect);

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
      for(int i=0;i < Target_ROI.size();i++)
      {
          ImageRoi = srcimage(Rect(Target_ROI[i].c));

      }


       // leafInfo.externel_rect = leafInfo.ellipseRect.boundingRect();

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
        Point2f Image_point[4];
        TargetRoi.points(Image_point);
        for (int j = 0; j < 4; j++)
        {
            line(srcImage, Image_point[j], Image_point[(j + 1) % 4], Scalar(0, 0, 255), 5, 8);  //绘制最小外接矩形每条边
        }
        Calculate_RT(Image_point);
    }

    namedWindow("find_connected",WINDOW_AUTOSIZE);
    imshow("find_connected",RF_image);
    waitKey(3);
  //  cout<<"ModulesDetect->RecognitionFailure process successful"<< endl;
    return 0;
}

