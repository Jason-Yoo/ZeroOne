#include "inc/ModulesDetect.h"

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
      cvtColor(srcImage, HSVImage, COLOR_BGR2HSV);
      inRange(HSVImage,Scalar(iLowH,iLowS,iLowV),Scalar(iHighH,iHighS,iHighV),dstImage);          //找寻在要求区间内的颜色

      int g_Otsu=0;
       if(Otsu(dstImage,g_Otsu))
      cout<<"ModulesDetect->Otsu process failed"<< endl;
       threshold(dstImage, dstImage, g_Otsu*0.5, 255, CV_THRESH_BINARY);

  }
  //imshow("Otsu process ",dstImage);
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
        if (wh_ratio > 4)
        {
            colors[j]=0;
            continue;
        }
        colors[j] = 255;
        tgt_lables.push_back(j);
    }

    //对连通域进行面积排序，保留前3
//    if(tgt_lables.size() < 1)
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

int ModulesDetect::Get_TargrtRoi(Mat &srcImage ,Mat &grayImage ,RotatedRect &TargetRoi )
{
    //find Ins_ROI
    vector<vector<Point>> contours;   //每一组Point点集就是一个轮廓
    vector<Vec4i> hierarcy;           //矩形集合


    findContours(grayImage, contours, hierarcy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE); //找出所有轮廓 包括轮廓关系
    if(contours.size()<=0)
          return 0;
     vector<Rect> box(contours.size()); //定义最小外接矩形集合
//   vector <Point2f> modulesCenter(contours.size());     //modules中心的点
      //绘制轮廓图
      for (uint i = 0; i < contours.size(); i++)
      {
        LeafInfo leafInfo;
        leafInfo.ellipseRect = fitEllipse(contours[i]);

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
        //rectangle(srcImage, Point(box[i].x, box[i].y), Point(box[i].x + box[i].width, box[i].y + box[i].height), Scalar(255, 0, 0), 2, 8);

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
            printf("The rate:%.2f%%\n", Max_bluerate * 100);
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


int  ModulesDetect::Get_ConerPoint(Mat &srcImage, RotatedRect Target_Roi, vector<Point2f> &Image_Point)
{
    Point2f RotateRect_point[4];
    Point2f RotateRect_center;
    Target_Roi.points(RotateRect_point);
    RotateRect_center = Point(Target_Roi.center.x, Target_Roi.center.y);

    for(int i = 0;i < 4; i++ )
    {
        RotateRect_point[i] = RotateRect_point[i] - RotateRect_center;

    }
    double  RotateRect_k[4];
    for(int i = 0;i < 4; i++ )
    {
        RotateRect_k[i] = (double)(RotateRect_point[i].y-RotateRect_point[(i + 1) % 4].y)/(double)(RotateRect_point[i].x-RotateRect_point[(i + 1) % 4].x);
        //   cout << "RotateRect_c"   << i <<":"<<RotateRect_point[i]<< endl;
        cout << "RotateRect_k"  << i <<":"<< RotateRect_k[i]   << endl;
    }


    Rect Target_Rect = Target_Roi.boundingRect();
    // Size rate(Target_Rect.width / 5, Target_Rect.height / 5);
    // Target_Rect = rectCenterScale(Target_Rect, rate);
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
   // rectangle(srcImage, Point(Target_Rect.x, Target_Rect.y), Point(Target_Rect.x + Target_Rect.width, Target_Rect.y + Target_Rect.height), Scalar(255, 0, 0), 2, 8);


    Mat rect_check = srcImage(Target_Rect);
    Mat ROI_image;
    rect_check.copyTo(ROI_image);
    cvtColor(ROI_image, ROI_image, COLOR_BGR2GRAY);
    GaussianBlur(ROI_image, ROI_image, Size(3, 3), 0, 0);

    //边缘检测
    Mat canny_image;
    Canny(ROI_image, canny_image, 60, 150, 3);
    //霍夫直线检测
    vector<Vec4i> Lines;
    HoughLinesP(canny_image, Lines, 1, CV_PI / 360, 50, 50, 10);

    // draw line
    for (size_t i = 0; i < Lines.size(); i++)
    {
        cv::Vec4i& linex = Lines[i];
        int dx=linex[2]-linex[0];
        int dy=linex[2]-linex[1];
        line(rect_check, cv::Point(linex[0], linex[1]), cv::Point(linex[2], linex[3]), cv::Scalar(0, 255, 0), 1);
    }

    // k value filter
   // vector<Vec4i> line1_buff(Lines.size());
   // vector<Vec4i> line2_buff(Lines.size());
    vector<Point> line1_points;
    vector<Point> line2_points;
    double line1_k = (RotateRect_k[0]+RotateRect_k[2])/2;
    double line2_k = (RotateRect_k[1]+RotateRect_k[3])/2;

    for (uint i = 1; i < Lines.size(); i++)
    {
        double ki = (double)(Lines[i][1] - Lines[i][3]) / (double)(Lines[i][0] - Lines[i][2]);
        if(ki > line1_k*0.7 && ki < line1_k*1.3)
        {
          //  line1_buff[i] = Lines[i];
            line1_points.push_back(Point(Lines[i][0],Lines[i][1]));
            line1_points.push_back(Point(Lines[i][2],Lines[i][3]));

        }

        if(ki > line2_k*0.7 && ki < line2_k*1.3)
        {
           //  line2_buff[i] = Lines[i];
             line2_points.push_back(Point(Lines[i][0],Lines[i][1]));
             line2_points.push_back(Point(Lines[i][2],Lines[i][3]));

        }

    }
    for (uint i = 0; i < line1_points.size(); i++)
    {
        circle(rect_check, line1_points[i], 5, cv::Scalar(0, 0, 255), 2, 8, 0);
    }
    for (uint i = 0; i < line2_points.size(); i++)
    {
        circle(rect_check, line2_points[i], 5, cv::Scalar(0, 0, 255), 2, 8, 0);
    }


//    Vec4f line1_para;
//    fitLine(line1_points, line1_para, DIST_L2, 0, 1e-2, 1e-2);

//    Vec4f line2_para;
//    fitLine(line2_points, line2_para, DIST_L2, 0, 1e-2, 1e-2);

//    std::cout << "line_para = " << line1_para << std::endl;

//    //求出直线上的两个点
//    double k_line = line1_para[1]/line1_para[0];
 //   Point p1(0,k_line*(0 - line1_para[2]) + line1_para[3]);
 //   Point p2(ROI_image.cols - 1,k_line*(ROI_image.cols - 1 - line1_para[2]) + line1_para[3]);

     //显示拟合出的直线
 //   line(ROI_image,p1,p2,Scalar(0,0,255),2);



    return 1;

}
int ModulesDetect::Bluebox_Detection(Mat &srcImage,Mat &DstImage,int method)
{

    RotatedRect TargetRoi;

    if(method == 1)
    {
        if(ROI_TrackFlag)
        {
            Size rate(ROI_TrackRect.width / 4, ROI_TrackRect.height / 4);
            ROI_TrackRect = rectCenterScale(ROI_TrackRect, rate);
           if(ROI_TrackRect.x < 0) ROI_TrackRect.x = 0;
           if(ROI_TrackRect.y < 0) ROI_TrackRect.y = 0;
           if(ROI_TrackRect.x+ROI_TrackRect.width >= srcImage.cols)
           {
               ROI_TrackRect.width = srcImage.cols-ROI_TrackRect.x-1;
           }
           if(ROI_TrackRect.y+ROI_TrackRect.height >= srcImage.rows)
           {
               ROI_TrackRect.height = srcImage.rows-ROI_TrackRect.y-1;
           }

           rectangle(srcImage, Point(ROI_TrackRect.x, ROI_TrackRect.y), Point(ROI_TrackRect.x + ROI_TrackRect.width, ROI_TrackRect.y + ROI_TrackRect.height), Scalar(0, 255, 0), 2, 8);
           Mat ROI_check = srcImage(ROI_TrackRect);
           if(RecognitionFailure(ROI_check,TargetRoi))
           {
               ROI_TrackRect = TargetRoi.boundingRect();

           }
           else
               ROI_TrackFlag = false;

        }
        else
        {
            if(RecognitionFailure(srcImage,TargetRoi))
            {
                ROI_TrackFlag = true;
                ROI_TrackRect = TargetRoi.boundingRect();
            }


        }
    }
    if(method == 2)
    {
        RecognitionFailure(srcImage,TargetRoi);
    }
    DstImage = detect_frame;

    return 0;
}
///
/// \brief ModulesDetect::RecognitionFailur
/// \param srcImage
/// \return
///
int ModulesDetect::RecognitionFailure(Mat &srcImage,RotatedRect &TargetRoi)
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
    detect_frame = grayImage;
    find_connected(grayImage);

    Targer_Flag = Get_TargrtRoi(srcImage,grayImage,TargetRoi);
    if(Targer_Flag)
    {


        //find image points
        vector<Point2f> Image_Point;
        Get_ConerPoint(srcImage, TargetRoi, Image_Point);
        for(uint i = 0; i < Image_Point.size(); i++)
        {
          //  circle(srcImage, Image_Point[i], 3, Scalar(255,0,0),-1); //第五个参数我设为-1，表明这是个实点。

        }

        //solve pnp problem
      //  Calculate_RT(Image_Point);

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
           cout<<"Hard to find the TargetRoi,please check Get_TargrtRoi process"<< endl;
    }
    return 0;
}

