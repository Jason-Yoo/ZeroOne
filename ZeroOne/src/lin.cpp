#include <opencv2/opencv.hpp>
#include <iostream>
#include<algorithm>
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>    
#include <opencv2/imgproc/types_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
using namespace cv;
using namespace std;


void MyGammaCorrection(Mat& src, Mat& frame, float fGamma)

{
	// build look up table  
	unsigned char lut[256];
	for (int i = 0; i < 256; i++)
	{
		lut[i] = saturate_cast<uchar>(pow((float)(i / 255.0), fGamma) * 255.0f);
	}
	frame = src.clone();
	const int channels = frame.channels();
	switch (channels)
	{

	case 1:  
	{
		MatIterator_<uchar> it, end;
		for (it = frame.begin<uchar>(), end = frame.end<uchar>(); it != end; it++)
			//*it = pow((float)(((*it))/255.0), fGamma) * 255.0;  
			*it = lut[(*it)];
		break;
	}
	case 3:  
	{
		MatIterator_<Vec3b> it, end;

		for (it = frame.begin<Vec3b>(), end = frame.end<Vec3b>(); it != end; it++)

		{
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


int main(int argc, char** argv)
{
	double low_H = 100;
	double low_S = 80;
	double low_V = 80;
	double high_H = 124;
	double high_S = 255;
	double high_V = 255;
	//videocapture结构创建一个catture视频对象
	VideoCapture capture;
	//连接视频
	capture.open("F:/ceshi/01.mp4");
	
	if (!capture.isOpened()) {
		printf("could not load video data...\n");
		return -1;
	}
	//int frames = capture.get(CAP_PROP_FRAME_COUNT);//获取视频针数目(一帧就是一张图片)
	
	// 获取帧的视频宽度，视频高度
	//Size size = Size(capture.get(CAP_PROP_FRAME_WIDTH), capture.get(CAP_PROP_FRAME_HEIGHT));
	//cout << frames << endl;

	//cout << size << endl;
	// 创建视频中每张图片对象
	Mat frame;
	Mat HSVImage;
	Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
	//namedWindow("video-demo", WINDOW_AUTOSIZE);
	// 循环显示视频中的每张图片
	while (true)
	
	{
		capture >> frame;
		//省略对图片的处理
		//视频播放完退出
		Mat src = frame.clone();
		Mat dst;
		//Mat frame_threshold = frame.clone();
		float fGamma =3;
		MyGammaCorrection(src, frame, fGamma);
		cvtColor(frame, HSVImage, COLOR_BGR2HSV);
	    inRange(HSVImage, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), HSVImage);
		//double fps = capture.get(CAP_PROP_FPS);//获取每针视频的频率
		//cout << fps << endl;
		//Mat erodedImage;
		erode(HSVImage, HSVImage, element);
		//Mat dilatedImage;
		dilate(HSVImage, HSVImage, element);
		//Mat dstImg = frame.clone();
		//Mat srcImg = frame_threshold.clone();
		vector<vector<Point>> contours;
		vector<Vec4i> hierarcy;
		findContours(HSVImage, contours, hierarcy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
		vector<Rect> boundRect(contours.size());  //定义外接矩形集合
		vector<RotatedRect> box(contours.size()); //定义最小外接矩形集合
		Point2f rect[4];
		for (int i = 0; i < contours.size(); i++)
		{         
			       std::sort(contours.begin(), contours.end(), ContoursSortFun);
			       box[0] = minAreaRect(Mat(contours[0]));  //计算每个轮廓最小外接矩形
			       boundRect[0] = boundingRect(Mat(contours[0]));
			       circle(frame, Point(box[0].center.x, box[0].center.y), 3, Scalar(0, 255, 0),  8);  //绘制最小外接矩形的中心点
			       box[0].points(rect);  //把最小外接矩形四个端点复制给rect数组
				   circle(frame, Point(rect[0].x, rect[0].y), 3, Scalar(255, 0, 0),  8);  //绘制最小外接矩形的中心点
				   circle(frame, Point(rect[1].x, rect[1].y), 3, Scalar(0, 255, 0), 8);  //绘制最小外接矩形的中心点
				   circle(frame, Point(rect[2].x, rect[2].y), 3, Scalar(139, 0, 139), 8);  //绘制最小外接矩形的中心点
				   circle(frame, Point(rect[3].x, rect[3].y), 3, Scalar(79, 79, 79), 8);  //绘制最小外接矩形的中心点

			       //rectangle(dstImg, Point(boundRect[i].x, boundRect[i].y), Point(boundRect[i].x + boundRect[i].width, boundRect[i].y + boundRect[i].height), Scalar(0, 255, 0), 2, 8);
			       for (int j = 0; j < 4; j++)
			        {
				            line(frame, rect[j], rect[(j + 1) % 4], Scalar(0, 0, 255), 2, 8);  //绘制最小外接矩形每条边
			        }
				   
				   char width[20], height[20];
				   //sprintf_s(width, "width=%0.2f", box[i].size.width);//
				   //sprintf_s(height, "height=%0.2f", box[i].size.height);//
				   //putText(frame, width, box[i].center, CV_FONT_HERSHEY_COMPLEX_SMALL, 0.85, Scalar(0, 0, 255));
				   //putText(frame, height, box[i].center + Point2f(0, 20), CV_FONT_HERSHEY_COMPLEX_SMALL, 0.85, Scalar(0, 0, 255));
		

		float angle_rotation;
		float angle_rotation1;
		if ((box[0].size.width / box[0].size.height) < 1)
		{
			angle_rotation = 90 + box[0].angle;//正数，逆时针旋转
			cout << " angle_rotation " << " :" << angle_rotation << endl;
			cout << " 左下点 " << " :" << rect[1].x<<','<< rect[0].y << endl;
			cout << " 左上点 " << " :" << rect[2].x << ',' << rect[1].y << endl;
			cout << " 右上点 " << " :" << rect[3].x << ',' << rect[2].y << endl;
			cout << " 右下点 " << " :" << rect[0].x << ',' << rect[3].y << endl;
			cout << " 中心点 " << " :" << box[0].center.x << ',' << box[i].center.y << endl;
			cout << " 面积 " << " :" << contourArea(contours[i], false)  << endl;
		}
		else
		{
			angle_rotation1 = box[0].angle; //负数，顺时针旋
			cout << "angle_rotation" << " :" << angle_rotation1 << endl;
			cout << " 左下点 " << " :" << rect[0].x << ',' << rect[0].y << endl;
			cout << " 左上点 " << " :" << rect[1].x << ',' << rect[1].y << endl;
			cout << " 右上点 " << " :" << rect[2].x << ',' << rect[2].y << endl;
			cout << " 右下点 " << " :" << rect[3].x << ',' << rect[3].y << endl;
			cout << " 中心点 " << " :" << box[0].center.x << ',' << box[i].center.y << endl;
			cout << " 面积 " << " :" << contourArea(contours[0], false) << endl;

		}
		
		cout << "width " << " :" << box[0].size.width << endl;
		cout << "height "  << " :" << box[0].size.height << endl << endl;
		//Mat matRotation = getRotationMatrix2D(box[i].center, angle_rotation, 1);//获得旋转矩阵正逆时针 负顺时针
		//Mat reverseMatRotation;
		//invert(matRotation, reverseMatRotation, DECOMP_SVD);
}
		namedWindow("HSV", WINDOW_AUTOSIZE);
		imshow("HSV", frame);
	   if (frame.empty())break;
		//imshow("video-demo", frame);
		//在视频播放期间按键退出
		if (waitKey(1) >= 0) break;
	}
	//释放
	capture.release();
	return 0;
}
