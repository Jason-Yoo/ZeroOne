#include "inc/mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    connect(this,SIGNAL(draw_SrcImageSignal()),this,SLOT(draw_SrcImageSlots()));
    connect(this,SIGNAL(draw_DstImageSignal()),this,SLOT(draw_DstImageSlots()));
    status = GXInitLib();
}

MainWindow::~MainWindow()
{
    status = GXCloseLib();
    delete ui;
}

void MainWindow::draw_SrcImageSlots()

{
    imshow("SrcImage", m_image);
}
void MainWindow::draw_DstImageSlots()

{
    imshow("rectImage", rectImage);
}
void  MainWindow::ImageProcess()
{

    Mat_<double> cameraMatrix(3, 3);
    Eigen::Matrix<double,3,3> M;
    double fx = 736.1196;
    double fy = 739.3896;
    double Cx = 648.7056;
    double Cy = 512.9957;
    cameraMatrix << fx, 0, Cx, 0, fy, Cy, 0, 0, 1;
    //畸变系数
    Mat_<double> distCoeffs(1, 5);
    distCoeffs << -0.2972, 0.0742, 0.00000, -0.00000, 0.00000;

    if(!rectImage.data)
    {
    Size imageSize(ImageWidth, ImageHeight);

    check_box.center.x=ImageWidth/2;
    check_box.center.y=ImageHeight/2;
    check_box.size=Size2f(ImageWidth/8,ImageHeight/8);
    //画矩形

    Point2f rect[4];
    check_box.points(rect);
    for (int j = 0; j < 4; j++)
    {
      line(m_image, rect[j], rect[(j + 1) % 4], Scalar(0,0,255), 2, 8);  //绘制最小外接矩形每条边
    }

    //设置绘制文本的相关参数
    std::string text = "Set_Template";
    int font_face = cv::FONT_HERSHEY_COMPLEX;
    double font_scale = 1;
    int thickness = 1;
    int baseline;
    //获取文本框的长宽
    cv::Size text_size = cv::getTextSize(text, font_face, font_scale, thickness, &baseline);
    putText(m_image, text, cvPoint(rect[1].x, rect[1].y-10), font_face, font_scale, cv::Scalar(255, 0, 0), thickness, 8, 0);
    }

    ImageTracking(m_image);



}
void MainWindow::ImageTracking(Mat image)
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

               char string[10];
               sprintf(string, "x=%dy=%d", point.x,point.y); // 帧率保留两位小数
               std::string fpsString("target:");
               fpsString += string;
                  putText(image,               // 图像矩阵
                          fpsString,                // string型文字内容
                          cv::Point(5, 20),         // 文字坐标，以左下角为原点
                          cv::FONT_HERSHEY_SIMPLEX, // 字体类型
                          0.6,                      // 字体大小
                          cv::Scalar(0, 255, 0));   // 字体颜色（B,G,R）

           }
           break;
       }

}
void MainWindow::on_AviSaveButton_clicked()
{
     g_bSaveVedioFlag = !g_bSaveVedioFlag;

}

void MainWindow::on_TempleteButton_clicked()
{
     Size imageSize(ImageWidth, ImageHeight);
     Rect Rect_box;
     Rect_box.size()=check_box.size;

     Point2f rect[4];
     check_box.points(rect);
     Rect_box.x =rect[0].x;
     Rect_box.y =rect[0].y;


     // 获取ROI
     Mat subImage = m_image(Rect(rect[0], rect[2]));
     rectImage = subImage.clone();              //给全局的待匹配图像
     resultRows = m_image.rows - rectImage.rows + 1;   //输出结果图像的行数及列数
     resultcols = m_image.cols - rectImage.rows + 1;
     rectangle(m_image,Rect_box,Scalar(0,255,0),3,8,0);//用矩形画矩形窗

     // setMouseCallback("PNP_Problem", MainWindow::on_mouse,this);   //设置鼠标回调函数

     // namedWindow("rectImage");
     // imshow("rectImage",rectImage);

     //draw image
     draw_DstImageSignal();

}

void  MainWindow::OnFrameCallbackFun(GX_FRAME_CALLBACK_PARAM* pFrame)
{
    if(pFrame->status)
    return;

    MainWindow * main = (MainWindow *) pFrame->pUserParam;
    if(main->m_Is_implemented)
    {
        DxRaw8toRGB24((void*)pFrame->pImgBuf,main->m_rgb_image,pFrame->nWidth, pFrame->nHeight,RAW2RGB_NEIGHBOUR,DX_PIXEL_COLOR_FILTER(BAYERBG),false);
        memcpy(main->m_image.data,main->m_rgb_image,pFrame->nHeight*pFrame->nWidth*3);
    }
    else
    {
        memcpy(main->m_image.data,pFrame->pImgBuf,pFrame->nHeight*pFrame->nWidth);
    }
    //size
    main->ImageWidth = pFrame->nWidth;
    main->ImageHeight= pFrame->nHeight;

    if(main->g_bSaveVedioFlag)
     main->writer.write(main->m_image);

    //Image process
  //  main->ImageProcess();
    //draw image
    main->draw_SrcImageSignal();

}

void MainWindow::on_OpenButton_clicked()
{
    //枚举设备个数

    uint32_t nDeviceNum = 0;
    status =GXUpdateDeviceList(&nDeviceNum, 1000);
    if(nDeviceNum <= 0)
    return;
    GX_OPEN_PARAM openParam;
    openParam.accessMode = GX_ACCESS_EXCLUSIVE;//访问方式
    openParam.openMode = GX_OPEN_INDEX; //通过序列号
    openParam.pszContent = "1";
    //open camera

    status = GXOpenDevice(&openParam, &m_hDevice);
    //ShowErrorString(status);
    if(status)
    return;

    ////////////////////////init opencv////////////////////

    int64_t width,height;
    status = GXGetInt(m_hDevice,GX_INT_WIDTH,&width);
    status = GXGetInt(m_hDevice,GX_INT_HEIGHT,&height);

    // 查询当前相机是否支持GX_ENUM_PIXEL_COLOR_FILTER
    status=GXIsImplemented(m_hDevice,GX_ENUM_PIXEL_COLOR_FILTER, &m_Is_implemented);
    //支持彩色图像
    if(m_Is_implemented)
    {
        status= GXGetEnum(m_hDevice, GX_ENUM_PIXEL_COLOR_FILTER, &m_pixel_color);
        m_image.create(height,width,CV_8UC3);
        m_rgb_image = new char[width*height*3];

    }
    else
    {
        m_image.create(height,width,CV_8UC1);

    }

    //Set Balance White Mode : Continuous
    status = GXSetEnum(m_hDevice, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_CONTINUOUS);

    //Set acquisition mode
    status = GXSetEnum(m_hDevice, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);

    //Set trigger mode
    status = GXSetEnum(m_hDevice, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_OFF);

    //注册图像处理回调函数
    status = GXRegisterCaptureCallback(m_hDevice, this, (GXCaptureCallBack)OnFrameCallbackFun);

    //发送开采命令
    status = GXSendCommand(m_hDevice, GX_COMMAND_ACQUISITION_START);

    g_bSaveVedioFlag = false;
    int frameRate = 40;
    writer.open("./uavgp.avi",CV_FOURCC('H', '2', '6', '4'),frameRate, Size(m_image.cols,m_image.rows),1);

}

void MainWindow::on_CLoseButton_clicked()
{
    //发送停采命令
    status = GXSendCommand(m_hDevice, GX_COMMAND_ACQUISITION_STOP);

    //注销采集回调
    status = GXUnregisterCaptureCallback(m_hDevice);
    status = GXCloseDevice(m_hDevice); //close device

    if(m_rgb_image)
    {
        delete m_rgb_image;
        m_rgb_image = NULL;
    }
}
void MainWindow::on_mouse(int EVENT, int x, int y, int flags, void* userdata)
{

    MainWindow* temp = reinterpret_cast<MainWindow*>(userdata);
    temp->onMouse(EVENT, x, y, userdata);

}
void MainWindow::onMouse(int events, int x, int y,void* userdata)
{
    Mat srcImage;
    srcImage = *(Mat*)userdata;

    Point recent_Point;
    switch (events)
    {
        case CV_EVENT_LBUTTONDOWN:

              recent_Point=Point(x, y);
               cout << recent_Point.x << " " << recent_Point.y<<" ";
               //pts_src.push_back(recent_Point);

              break;
        case CV_EVENT_MOUSEMOVE:

             break;
        case CV_EVENT_LBUTTONUP:

             break;
    }
    imshow("PNP_Problem",srcImage);


}


