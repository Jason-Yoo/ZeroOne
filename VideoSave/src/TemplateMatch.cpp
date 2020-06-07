#include <inc/TemplateMatch.h>

void TemplateMatch::ImageTracking(Mat image,Mat rectImage)
{

       int choice = 6;
       while (choice)
       {

           if (!image.data )  //图像为空或Esc键按下退出播放
           {
               break;
           }
           if (rectImage.data)
           {
               ImageResult = Mat::zeros(resultRows, resultcols, CV_32FC1);//建立结果矩阵，注意是单通道的32位浮点型
               switch (choice) { //根据一开始的选择使用对应的匹配模式
               case 1:
                   matchTemplate(image, rectImage, ImageResult, TM_SQDIFF);
                   break;
               case 2:
                   matchTemplate(image, rectImage, ImageResult, TM_SQDIFF_NORMED);
                   break;
               case 3:
                   matchTemplate(image, rectImage, ImageResult, TM_CCORR);
                   break;
               case 4:
                   matchTemplate(image, rectImage, ImageResult, TM_CCORR_NORMED);
                   break;
               case 5:
                   matchTemplate(image, rectImage, ImageResult, TM_CCOEFF);
                   break;
               case 6:
                   matchTemplate(image, rectImage, ImageResult, TM_CCOEFF_NORMED);
                   break;
               }
               minMaxLoc(ImageResult, &minValue, &maxValude, &minPoint, &maxPoint, Mat());  //最小值最大值获取
               Point point;
               switch (choice) {//为了统一处理，所以先把Point取出来
               case 1:
                   point = minPoint;
                   break;
               case 2:
                   point = minPoint;
                   break;
               case 3:
                   point = maxPoint;
                   break;
               case 4:
                   point = maxPoint;
                   break;
               case 5:
                   point = maxPoint;
                   break;
               case 6:
                   point = maxPoint;
                   break;
               }
               rectangle(image, point, Point(point.x + rectImage.cols, point.y + rectImage.rows), Scalar(0, 0, 255), 2); //绘制
               //更新当前模板匹配的模板
               Mat resultImage = image(Rect(point, Point(point.x + rectImage.cols, point.y + rectImage.rows)));
               rectImage = resultImage.clone();
               //当前帧数输出到视频流
         //      writer << image;
           }
           imshow("Video", image);
       }

}

//鼠标回调函数
void TemplateMatch::onMouse(int event, int x, int y, int flags, void *ustc)
{
    if (event == CV_EVENT_LBUTTONDOWN)   //检测到左键按下时
    {
        leftButtonDownFlag = true; //标志位为true，也就是停止读取下一帧图像
        beginPoint = Point(x, y);  //设置左键按下点的矩形起点
        endPoint = beginPoint;
    }
    if (event == CV_EVENT_MOUSEMOVE && leftButtonDownFlag)
    {                               //当鼠标移动且之前左键有按下的话
        imageCopy = image.clone();
        endPoint = Point(x, y);
        if (beginPoint != endPoint)
        {
            //在复制的图像上绘制矩形
            rectangle(imageCopy, beginPoint, endPoint, Scalar(0, 0, 255), 2);
        }
        //imshow("Video", imageCopy);
    }
    if (event == CV_EVENT_LBUTTONUP) //左键放开时，开始匹配
    {
        leftButtonDownFlag = false;
        Mat subImage = image(Rect(beginPoint, endPoint)); //截取图像
        rectImage = subImage.clone();              //给全局的待匹配图像
        resultRows = image.rows - rectImage.rows + 1;   //输出结果图像的行数及列数
        resultcols = image.cols - rectImage.rows + 1;

    }
}
