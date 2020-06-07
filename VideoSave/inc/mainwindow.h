#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "GxIAPI.h"
#include "DxImageProc.h"
#include <iostream>

#include <eigen3/Eigen/Dense>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <inc/TemplateMatch.h>

using namespace cv;
using namespace std;


namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

public:
    GX_STATUS status;
    GX_DEV_HANDLE m_hDevice;
    bool m_Is_implemented;
    bool Only_once;
    bool g_bSaveVedioFlag;
    int64_t m_pixel_color;
    char *m_rgb_image;
    Mat m_image;
    Mat UndistortImage;
    Mat rectImage;
    void ShowErrorString(GX_STATUS emErrorStatus);
    static  void  OnFrameCallbackFun(GX_FRAME_CALLBACK_PARAM* pFrame);
    static  void  on_mouse(int EVENT, int x, int y, int flags, void* userdata);
    void onMouse(int events, int x, int y,void* userdata);
    void ImageProcess();
    void ImageTracking(Mat image);

    int ImageWidth;
    int ImageHeight;
    int resultRows;
    int resultcols;
    Mat ImageResult;   //模板匹配result
    double minValue;   //模板匹配result最小值
    double maxValude;  //模板匹配result最大值
    Point minPoint;    //模板匹配result最小值位置
    Point maxPoint;    //模板匹配result最大值位置
     RotatedRect check_box;
     VideoWriter writer;



signals:
void draw_SrcImageSignal();
void draw_DstImageSignal();



private slots:
    void on_OpenButton_clicked();
    void on_CLoseButton_clicked();
    void on_TempleteButton_clicked();
    void on_AviSaveButton_clicked();
    void draw_SrcImageSlots();
    void draw_DstImageSlots();

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
