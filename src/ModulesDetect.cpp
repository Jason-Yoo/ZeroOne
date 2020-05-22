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

/// \brief Dafu_Detector::bgr2binary
/// input color image and output binary image. It can use 2 methods to acheive that.
/// \param srcImage   input
/// \param dstImage    output
/// \param method --1: split channels --2: use canny
/// \return
///
int ModulesDetect::bgr2binary(Mat &srcImage, Mat &dstImage, int method)
{
  cout<<"ModulesDetect->bgr2binary process is begin"<< endl;

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
      cvtColor(srcImage, dstImage, COLOR_BGR2GRAY);
      GaussianBlur(dstImage, dstImage, Size(3, 3), 0, 0);

      //int g_Otsu=0;
      // if(Otsu(dstImage,g_Otsu))
      //cout<<"ModulesDetect->Otsu process failed"<< endl;
      // threshold(dstImage, dstImage, g_Otsu, 255, CV_THRESH_BINARY);

      //Canny
      int edgeThresh =80;
      Canny(dstImage, dstImage, edgeThresh, edgeThresh * 3, 3);
  }
  cout<<"ModulesDetect->bgr2binary process successful"<< endl;
  return 0;
}
///
/// \brief Dafu_Detector::GetPixelLength      计算像素坐标距离
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

Point ModulesDetect::find_connected(Mat &binary_img)
{
    Mat labels, img_color, stats,centroids;
    Mat binary_inv = ~binary_img;
   // Mat kernel = (Mat_<float>(2, 2) << 2, 7, 10, 0) ;
    int nccomps = cv::connectedComponentsWithStats(binary_inv, labels, stats, centroids);
    //去除过小区域，初始化颜色表
    vector<uchar> colors(nccomps);
    colors[0] = 0; // background pixels remain black.
    for( int i = 1; i < nccomps; i++ ) {
        colors[i] = 255;
        if( stats.at<int>(i, cv::CC_STAT_AREA) < 400 )
            colors[i] = 0; // small regions are painted with black too.
    }

    img_color = Mat::zeros(binary_inv.size(), CV_8UC1);
    for( int y = 0; y < img_color.rows; y++ )
        for( int x = 0; x < img_color.cols; x++ )
        {
            int label = labels.at<int>(y, x);
            CV_Assert(0 <= label && label <= nccomps);
            img_color.at<uchar>(y, x) = colors[label];
        }
    binary_img=img_color;


//    vector<int> tgt_lables;
//    int area_max = 400, area_min = 100;
//    for (int j = 0; j < stats.rows; j++)  //x0,y0,width,height,area
//    {
//        int unit_x = stats.at<int>(j,0) ,unit_y= stats.at<int>(j,1);
//        int area = stats.at<int>(j,4);
//        int width = stats.at<int>(j,2), height = stats.at<int>(j,3);
//        float wh_ratio = float(width) / float(height);

//        if (unit_x==0&& unit_y == 0) //background
//        {
//            continue;
//        }
//        if (area < area_min)
//        {
//            continue;
//        }
//        if (wh_ratio > 3 || wh_ratio < 0.33)
//        {
//            continue;
//        }
//        tgt_lables.push_back(j);
//    }
//    if (tgt_lables.size() == 1)
//    {
//        printf("got target");
//        //int unit_x = stats.at<uchar>(0,tgt_lables[0]), unit_y = stats.at<uchar>(1,tgt_lables[0]);
//        //int area = stats.at<uchar>(4,tgt_lables[0]);
//        //int width = stats.at<uchar>(2,tgt_lables[0]), height = stats.at<uchar>(3,tgt_lables[0]);
//        //float wh_ratio = float(width) / float(height);
//        //	Point tgt_point(unit_x+0.5*width,unit_y+0.5*height);
//     //   float center_x = kernel.at<double>(tgt_lables[0], 0);
//     //   float center_y = kernel.at<double>(tgt_lables[0], 1);
//     //   Point tgt_point(center_x,center_y );
//     //   return tgt_point;
//    }
//    else
//    {
//        printf("see whats going on");
//        return Point(0, 0);
//    }

    return Point(0, 0);
}

///
/// \brief ModulesDetect::RecognitionFailur
/// \param srcImage
/// \return
///
int ModulesDetect::RecognitionFailure(Mat &srcImage)
{
    cout<<"ModulesDetect->RecognitionFailure process is begin"<< endl;

    Mat RF_image;

    if(bgr2binary(srcImage,RF_image,1))
     cout<<"ModulesDetect->bgr2binary process failed"<< endl;

    //连接连通域
    static Mat kernel_close = getStructuringElement(MORPH_RECT, Size(3,3), Point(-1, -1));
    morphologyEx(RF_image, RF_image, MORPH_DILATE, kernel_close);

    find_connected(RF_image);

    detect_frame=RF_image;
    //find Ins_ROI
    vector<vector<Point>> contours;   //每一组Point点集就是一个轮廓
    vector<Vec4i> hierarcy;           //矩形集合
    vector<int> modules_center_candidates;

    findContours(RF_image, contours, hierarcy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE); //找出所有轮廓 包括轮廓关系
    if(contours.size()<=0)
          return -1;
     vector<RotatedRect> box(contours.size()); //定义最小外接矩形集合
     vector <Point2f> modulesCenter(contours.size());     //modules中心的点

      //绘制轮廓图
      for (int i = 0; i < contours.size(); i++)
      {
        LeafInfo leafInfo;
        leafInfo.ellipseRect = fitEllipse(contours[i]);

        int area = leafInfo.ellipseRect.size.area();
        if (area < 500)
        {
            modules_center_candidates.push_back(i);   // likely to be center
           // continue;
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
        if (w_div_h < 2)
        {
            continue;
        }

        Point2f lf_c_sum(0,0);
        leafInfo.ellipseRect.points(leafInfo.vertices);
        float dist_threth = (leafInfo.chang + leafInfo.kuan)/2;
        for (int i = 0; i < 4; i++)
        {
            lf_c_sum += leafInfo.vertices[i];
            line(srcImage, leafInfo.vertices[i], leafInfo.vertices[(i + 1) % 4], Scalar(0, 255, 0));
            float edge_length=GetPixelLength(leafInfo.vertices[i], leafInfo.vertices[(i + 1) % 4]);
            if (edge_length>1.5*leafInfo.kuan)
            {
                Point2f temp_vec = Point2f(leafInfo.vertices[i] - leafInfo.vertices[(i + 1) % 4]);
                leafInfo.vec_chang = temp_vec / norm(temp_vec);
            }

        }
        leafInfo.leaf_center = lf_c_sum / 4;
        leafInfo.externel_rect = leafInfo.ellipseRect.boundingRect();


        //求最小外接矩形
        Point2f rect[4];
        box[i] = minAreaRect(Mat(contours[i]));
        box[i].points(rect);
        for (int j = 0; j < 4; j++)
        {
          line(srcImage, rect[j], rect[(j + 1) % 4], Scalar(0, 0, 255), 1, 8);  //绘制最小外接矩形每条边
        }

      }
    cout<<"ModulesDetect->RecognitionFailure process successful"<< endl;
    return 0;
}

