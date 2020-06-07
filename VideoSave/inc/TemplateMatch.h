#ifndef TEMPLATEMATCH_H
#define TEMPLATEMATCH_H

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class TemplateMatch {

public:
    Mat image;         //当前帧图像
    Mat imageCopy;     //用于拷贝的当前帧图像
    Mat rectImage;     //子图像
    bool leftButtonDownFlag = false; //左键单击后视频暂停标志位
    Point beginPoint;  //矩形框起点
    Point endPoint;    //矩形框终点
    int resultRows;    //模板匹配result的行
    int resultcols;    //模板匹配result的列
    Mat ImageResult;   //模板匹配result
    double minValue;   //模板匹配result最小值
    double maxValude;  //模板匹配result最大值
    Point minPoint;    //模板匹配result最小值位置
    Point maxPoint;    //模板匹配result最大值位置
    int frameCount = 0; //帧数统计

    TemplateMatch();
    void onMouse(int event, int x, int y, int flags, void *ustc);
    void ImageTracking(Mat image,Mat rectImage);


};

#endif // TEMPLATEMATCH_H
