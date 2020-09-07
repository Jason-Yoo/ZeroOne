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
	//videocapture�ṹ����һ��catture��Ƶ����
	VideoCapture capture;
	//������Ƶ
	capture.open("F:/ceshi/01.mp4");
	
	if (!capture.isOpened()) {
		printf("could not load video data...\n");
		return -1;
	}
	//int frames = capture.get(CAP_PROP_FRAME_COUNT);//��ȡ��Ƶ����Ŀ(һ֡����һ��ͼƬ)
	
	// ��ȡ֡����Ƶ��ȣ���Ƶ�߶�
	//Size size = Size(capture.get(CAP_PROP_FRAME_WIDTH), capture.get(CAP_PROP_FRAME_HEIGHT));
	//cout << frames << endl;

	//cout << size << endl;
	// ������Ƶ��ÿ��ͼƬ����
	Mat frame;
	Mat HSVImage;
	Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
	//namedWindow("video-demo", WINDOW_AUTOSIZE);
	// ѭ����ʾ��Ƶ�е�ÿ��ͼƬ
	while (true)
	
	{
		capture >> frame;
		//ʡ�Զ�ͼƬ�Ĵ���
		//��Ƶ�������˳�
		Mat src = frame.clone();
		Mat dst;
		//Mat frame_threshold = frame.clone();
		float fGamma =3;
		MyGammaCorrection(src, frame, fGamma);
		cvtColor(frame, HSVImage, COLOR_BGR2HSV);
	    inRange(HSVImage, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), HSVImage);
		//double fps = capture.get(CAP_PROP_FPS);//��ȡÿ����Ƶ��Ƶ��
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
		vector<Rect> boundRect(contours.size());  //������Ӿ��μ���
		vector<RotatedRect> box(contours.size()); //������С��Ӿ��μ���
		Point2f rect[4];
		for (int i = 0; i < contours.size(); i++)
		{         
			       std::sort(contours.begin(), contours.end(), ContoursSortFun);
			       box[0] = minAreaRect(Mat(contours[0]));  //����ÿ��������С��Ӿ���
			       boundRect[0] = boundingRect(Mat(contours[0]));
			       circle(frame, Point(box[0].center.x, box[0].center.y), 3, Scalar(0, 255, 0),  8);  //������С��Ӿ��ε����ĵ�
			       box[0].points(rect);  //����С��Ӿ����ĸ��˵㸴�Ƹ�rect����
				   circle(frame, Point(rect[0].x, rect[0].y), 3, Scalar(255, 0, 0),  8);  //������С��Ӿ��ε����ĵ�
				   circle(frame, Point(rect[1].x, rect[1].y), 3, Scalar(0, 255, 0), 8);  //������С��Ӿ��ε����ĵ�
				   circle(frame, Point(rect[2].x, rect[2].y), 3, Scalar(139, 0, 139), 8);  //������С��Ӿ��ε����ĵ�
				   circle(frame, Point(rect[3].x, rect[3].y), 3, Scalar(79, 79, 79), 8);  //������С��Ӿ��ε����ĵ�

			       //rectangle(dstImg, Point(boundRect[i].x, boundRect[i].y), Point(boundRect[i].x + boundRect[i].width, boundRect[i].y + boundRect[i].height), Scalar(0, 255, 0), 2, 8);
			       for (int j = 0; j < 4; j++)
			        {
				            line(frame, rect[j], rect[(j + 1) % 4], Scalar(0, 0, 255), 2, 8);  //������С��Ӿ���ÿ����
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
			angle_rotation = 90 + box[0].angle;//��������ʱ����ת
			cout << " angle_rotation " << " :" << angle_rotation << endl;
			cout << " ���µ� " << " :" << rect[1].x<<','<< rect[0].y << endl;
			cout << " ���ϵ� " << " :" << rect[2].x << ',' << rect[1].y << endl;
			cout << " ���ϵ� " << " :" << rect[3].x << ',' << rect[2].y << endl;
			cout << " ���µ� " << " :" << rect[0].x << ',' << rect[3].y << endl;
			cout << " ���ĵ� " << " :" << box[0].center.x << ',' << box[i].center.y << endl;
			cout << " ��� " << " :" << contourArea(contours[i], false)  << endl;
		}
		else
		{
			angle_rotation1 = box[0].angle; //������˳ʱ����
			cout << "angle_rotation" << " :" << angle_rotation1 << endl;
			cout << " ���µ� " << " :" << rect[0].x << ',' << rect[0].y << endl;
			cout << " ���ϵ� " << " :" << rect[1].x << ',' << rect[1].y << endl;
			cout << " ���ϵ� " << " :" << rect[2].x << ',' << rect[2].y << endl;
			cout << " ���µ� " << " :" << rect[3].x << ',' << rect[3].y << endl;
			cout << " ���ĵ� " << " :" << box[0].center.x << ',' << box[i].center.y << endl;
			cout << " ��� " << " :" << contourArea(contours[0], false) << endl;

		}
		
		cout << "width " << " :" << box[0].size.width << endl;
		cout << "height "  << " :" << box[0].size.height << endl << endl;
		//Mat matRotation = getRotationMatrix2D(box[i].center, angle_rotation, 1);//�����ת��������ʱ�� ��˳ʱ��
		//Mat reverseMatRotation;
		//invert(matRotation, reverseMatRotation, DECOMP_SVD);
}
		namedWindow("HSV", WINDOW_AUTOSIZE);
		imshow("HSV", frame);
	   if (frame.empty())break;
		//imshow("video-demo", frame);
		//����Ƶ�����ڼ䰴���˳�
		if (waitKey(1) >= 0) break;
	}
	//�ͷ�
	capture.release();
	return 0;
}
