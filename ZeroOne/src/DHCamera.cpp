/******************************************************************************
* Function: DaHeng Camera Driver                                             *
* Author :  Jiduocai Yang                                                    *
* Contact:  jiduocaiyang@gmail.com                                           *
* Address:  NUAA                                                             *
******************************************************************************/
#include "DHCamera.h"
#include <stdlib.h>
#include<stdio.h>

typedef unsigned char byte;

// *************UDP头文件×××××××××××× //
#include<sys/select.h>
#include<unistd.h>
#include<sys/types.h>
#include<sys/socket.h>
#include<arpa/inet.h>
#include<netinet/in.h>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <iostream>
#include<opencv2/opencv.hpp>
//  ××××××××××××××××××××××××××××  //



#define ACQ_BUFFER_NUM          5               ///< Acquisition Buffer Qty.
#define ACQ_TRANSFER_SIZE       (64 * 1024)     ///< Size of data transfer block
#define ACQ_TRANSFER_NUMBER_URB 64              ///< Qty. of data transfer block
#define FILE_NAME_LEN           50              ///< Save image file name length

#define PIXFMT_CVT_FAIL             -1             ///< PixelFormatConvert fail
#define PIXFMT_CVT_SUCCESS          0              ///< PixelFormatConvert success
#define _IP_MARK "."



void DHCamera::DH_Camera()
{

    g_pRgbBuffer[0] = NULL;
    g_pRgbBuffer[1] = NULL;
    updated = false;

    for(uint i = 0;i < 5;i++)
    {
        realdistance [i] = 0;
    }
    DH_StartPoint = Point(0,0);
    DH_EndPoint   = Point(0,0);

    //相机内参数
    //    Mat_<double> cameraMatrix(3, 3);
    //    double fx = 736.1196;
    //    double fy = 739.3896;
    //    double Cx = 648.7056;
    //    double Cy = 512.9957;
    //    cameraMatrix << fx, 0, Cx, 0, fy, Cy, 0, 0, 1;
    //    //畸变系数;
    //    Mat_<double> distCoeffs(1, 5);
    //    distCoeffs << -0.2972, 0.0742, 0.00000, -0.00000, 0.00000;
}

//   ××××××××××××相机初始化××××××××××××××××××××××   //
int DHCamera::Init()
{
    printf("CAMERA SDK INIT...\n");
    GX_STATUS emStatus = GX_STATUS_SUCCESS;
    //Initialize libary
    // 在 起 始 位 置 调 用 GXInitLib()进 行 初 始 化 , 申 请 资 源
    emStatus = GXInitLib();

    if(emStatus != GX_STATUS_SUCCESS)
    {
        GetErrorString(emStatus);
        return emStatus;
    }
    printf("DONE\n");

    //枚举设备，并建立设备列表
    printf("ENUM CAMERA DEVICES...\n");
    emStatus = GXUpdateDeviceList(&ui32DeviceNum, 1000);
    if(emStatus != GX_STATUS_SUCCESS)
    {
        GetErrorString(emStatus);
        GXCloseLib();    //调 用 GXCLoseLib()释 放 资 源
        return emStatus;
    }

    //If no device found, app exit
    //如果没有找到设备，则退出
    if(ui32DeviceNum <= 0)
    {
        printf("<No device found>\n");
        GXCloseLib();
        return emStatus;
    }

    //Open first device enumerated
    // 打开第一个设备
    emStatus = GXOpenDeviceByIndex(1, &g_hDevice);
    if(emStatus != GX_STATUS_SUCCESS)
    {
        GetErrorString(emStatus);
        GXCloseLib();
        return emStatus;
    }
    printf("Open first device\n");


    //Get the type of Bayer conversion. whether is a color camera.
    // 查询当前相机是否支持某功能
    emStatus = GXIsImplemented(g_hDevice, GX_ENUM_PIXEL_COLOR_FILTER, &g_bColorFilter);
    GX_VERIFY_EXIT(emStatus);

    //This app only support color cameras
    if (!g_bColorFilter)
    {
        printf("<This app only support color cameras! App Exit!>\n");
        GXCloseDevice(g_hDevice);
        g_hDevice = NULL;
        GXCloseLib();
        return 0;
    }
    else
    {     //若当前设备为彩色相机
        emStatus = GXGetEnum(g_hDevice, GX_ENUM_PIXEL_COLOR_FILTER, &g_i64ColorFilter);   //获取当前枚举值
        GX_VERIFY_EXIT(emStatus);
    }

    emStatus = GXGetInt(g_hDevice, GX_INT_PAYLOAD_SIZE, &g_nPayloadSize);
    GX_VERIFY(emStatus);

    //Set acquisition mode
    emStatus = GXSetEnum(g_hDevice, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);   // 设置枚举值
    GX_VERIFY_EXIT(emStatus);

    //Set trigger mode
    emStatus = GXSetEnum(g_hDevice, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_OFF);
    GX_VERIFY_EXIT(emStatus);

    //Set buffer quantity of acquisition queue
    uint64_t nBufferNum = ACQ_BUFFER_NUM;
    emStatus = GXSetAcqusitionBufferNumber(g_hDevice, nBufferNum);   // 设置采集 buffer 个数
    GX_VERIFY_EXIT(emStatus);

    //查询某功能码当前是否可写
    //GX_DS_INT_STREAM_TRANSFER_SIZE  USB3 Vision相机传输时每个数据块的
    bool bStreamTransferSize = false;
    emStatus = GXIsImplemented(g_hDevice, GX_DS_INT_STREAM_TRANSFER_SIZE, &bStreamTransferSize);
    GX_VERIFY_EXIT(emStatus);

    if(bStreamTransferSize)
    {
        //Set size of data transfer block
        emStatus = GXSetInt(g_hDevice, GX_DS_INT_STREAM_TRANSFER_SIZE, ACQ_TRANSFER_SIZE);
        GX_VERIFY_EXIT(emStatus);
    }

    bool bStreamTransferNumberUrb = false;
    emStatus = GXIsImplemented(g_hDevice, GX_DS_INT_STREAM_TRANSFER_NUMBER_URB, &bStreamTransferNumberUrb);
    GX_VERIFY_EXIT(emStatus);

    if(bStreamTransferNumberUrb)
    {
        //Set qty. of data transfer block
        emStatus = GXSetInt(g_hDevice, GX_DS_INT_STREAM_TRANSFER_NUMBER_URB, ACQ_TRANSFER_NUMBER_URB);
        GX_VERIFY_EXIT(emStatus);
    }

    GX_VERIFY_EXIT(emStatus);
    //Set  Exposure
    // emStatus = GXSetFloat(g_hDevice, GX_FLOAT_EXPOSURE_TIME, 12000.0000);
    emStatus = GXSetEnum(g_hDevice,GX_ENUM_BALANCE_WHITE_AUTO,GX_BALANCE_WHITE_AUTO_CONTINUOUS);   //设 置 连 续 自 动 白 平 衡
    emStatus = GXSetEnum(g_hDevice, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_CONTINUOUS);
    GX_VERIFY_EXIT(emStatus);

    //Set  GAMMA
    emStatus = GXSetBool(g_hDevice, GX_BOOL_GAMMA_ENABLE, true);
    GX_GAMMA_MODE_ENTRY nValue;
    nValue = GX_GAMMA_SELECTOR_USER;
    emStatus = GXSetEnum(g_hDevice, GX_ENUM_GAMMA_MODE, nValue);
    double dColorParam = 0.5;
    emStatus = GXSetFloat(g_hDevice, GX_FLOAT_GAMMA_PARAM, dColorParam);

    //Allocate the memory for pixel format transform
    PreForAcquisition();

    if(ui32DeviceNum >= 2)
    {
        // 打开第2个设备
        emStatus = GXOpenDeviceByIndex(2, &g2_hDevice);
        if(emStatus != GX_STATUS_SUCCESS)
        {
            GetErrorString(emStatus);
            GXCloseLib();
            return emStatus;
        }
        printf("Open second device\n");

        // 查询当前相机是否支持某功能
        emStatus = GXIsImplemented(g2_hDevice, GX_ENUM_PIXEL_COLOR_FILTER, &g2_bColorFilter);
        GX_VERIFY_EXIT(emStatus);

        //This app only support color cameras
        if (!g2_bColorFilter)
        {
            printf("<This app only support color cameras! App Exit!>\n");
            GXCloseDevice(g2_hDevice);
            g2_hDevice = NULL;
            GXCloseLib();
            return 0;
        }
        else
        {     //若当前设备为彩色相机
            emStatus = GXGetEnum(g2_hDevice, GX_ENUM_PIXEL_COLOR_FILTER, &g2_i64ColorFilter);   //获取当前枚举值
            GX_VERIFY_EXIT(emStatus);
        }

        emStatus = GXGetInt(g2_hDevice, GX_INT_PAYLOAD_SIZE, &g2_nPayloadSize);
        GX_VERIFY(emStatus);

        //Set acquisition mode
        emStatus = GXSetEnum(g2_hDevice, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);   // 设置枚举值
        GX_VERIFY_EXIT(emStatus);

        //Set trigger mode
        emStatus = GXSetEnum(g2_hDevice, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_OFF);
        GX_VERIFY_EXIT(emStatus);

        //Set buffer quantity of acquisition queue
        uint64_t nBufferNum = ACQ_BUFFER_NUM;
        emStatus = GXSetAcqusitionBufferNumber(g2_hDevice, nBufferNum);   // 设置采集 buffer 个数
        GX_VERIFY_EXIT(emStatus);

        //查询某功能码当前是否可写
        //GX_DS_INT_STREAM_TRANSFER_SIZE  USB3 Vision相机传输时每个数据块的
        bool bStreamTransferSize = false;
        emStatus = GXIsImplemented(g2_hDevice, GX_DS_INT_STREAM_TRANSFER_SIZE, &bStreamTransferSize);
        GX_VERIFY_EXIT(emStatus);

        if(bStreamTransferSize)
        {
            //Set size of data transfer block
            emStatus = GXSetInt(g2_hDevice, GX_DS_INT_STREAM_TRANSFER_SIZE, ACQ_TRANSFER_SIZE);
            GX_VERIFY_EXIT(emStatus);
        }

        bool bStreamTransferNumberUrb = false;
        emStatus = GXIsImplemented(g2_hDevice, GX_DS_INT_STREAM_TRANSFER_NUMBER_URB, &bStreamTransferNumberUrb);
        GX_VERIFY_EXIT(emStatus);

        if(bStreamTransferNumberUrb)
        {
            //Set qty. of data transfer block
            emStatus = GXSetInt(g2_hDevice, GX_DS_INT_STREAM_TRANSFER_NUMBER_URB, ACQ_TRANSFER_NUMBER_URB);
            GX_VERIFY_EXIT(emStatus);
        }
        GX_VERIFY_EXIT(emStatus);
        //Set  Exposure
        // emStatus = GXSetFloat(g_hDevice, GX_FLOAT_EXPOSURE_TIME, 12000.0000);
        emStatus = GXSetEnum(g2_hDevice,GX_ENUM_BALANCE_WHITE_AUTO,GX_BALANCE_WHITE_AUTO_CONTINUOUS);   //设 置 连 续 自 动 白 平 衡
        emStatus = GXSetEnum(g2_hDevice, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_CONTINUOUS);
        GX_VERIFY_EXIT(emStatus);


        //分配内存
        g2_pRGBImageBuf = new unsigned char[g_nPayloadSize * 3];
        g2_pRaw8Image = new unsigned char[g_nPayloadSize];

    }

}

int DHCamera::Uninit()
{

}

void imageUDP(Mat srcimage, int sockfd, struct sockaddr_in addr)
{
    socklen_t          addr_len=sizeof(addr);

    Mat tupian ;
    char sendData[65535];
    std::vector<uchar> imgData;
    vector<int> quality;
    int imgSize;

    //resize(srcimage, tupian ,cv::Size(480, 320));
    resize(srcimage, tupian ,cv::Size(320, 256));
    quality.push_back(CV_IMWRITE_JPEG_CHROMA_QUALITY);
    quality.push_back(10);
    imencode(".jpg",tupian, imgData,quality);
    imgSize = imgData.size();
    if(imgSize <= 65535)
    {
        for (int i = 0; i != imgSize; ++i)
        {
            sendData[i] = imgData[i];
        }
        //发送编码后的图像
        sendto(sockfd, sendData, imgSize, 0, (sockaddr*)&addr, addr_len);
    }
    else
    {
        cout<<"imgSize = "<<  imgSize  << endl;
    }
    memset(&sendData,0,sizeof(sendData));
}




//  *****************开始获取图像************************   //
int DHCamera::Read()
{

    //Device start acquisition
    emStatus = GXStreamOn(g_hDevice);   // 开采,包括流开采和设备开采。  [in] g_hDevice 设备句柄
    if(emStatus != GX_STATUS_SUCCESS)
    {
        //Release the memory allocated
        // 释放已经分配的内存
        UnPreForAcquisition();
        GX_VERIFY_EXIT(emStatus);
    }

    int64_t width,height;
    emStatus = GXGetInt(g_hDevice,GX_INT_WIDTH,&width);
    emStatus = GXGetInt(g_hDevice,GX_INT_HEIGHT,&height);
    src_image.create(height,width,CV_8UC3);

    int nRet = pthread_create(&g_nAcquisitonThreadID, NULL, ProcGetImage, this);
    if(nRet != 0)
    {
        //Release the memory allocated
        // 释放已经分配的内存
        UnPreForAcquisition();

        GXCloseDevice(g_hDevice);
        g_hDevice = NULL;
        GXCloseLib();     //关闭设备库,释放资源。当停止 GxIAPI 对设备进行的所有控制之后,必须调用此接口来释放资源,与GXInitLib 对应。

        printf("<Failed to create the acquisition g_hDevice thread, App Exit!>\n");
        exit(nRet);
    }
    else
    {
        printf("<Create the acquisition g_hDevice thread is successful, App Exit!>\n");

    }

    if(g2_hDevice != NULL)
    {
        //Device start acquisition
        emStatus = GXStreamOn(g2_hDevice);   // 开采,包括流开采和设备开采。  [in] g_hDevice 设备句柄
        if(emStatus != GX_STATUS_SUCCESS)
        {
            //Release the memory allocated
            // 释放已经分配的内存
            if (g2_pRGBImageBuf != NULL)
            {
                delete[] g2_pRGBImageBuf;
                g2_pRGBImageBuf = NULL;
            }
            GX_VERIFY_EXIT(emStatus);
        }

        int64_t width,height;
        emStatus = GXGetInt(g2_hDevice,GX_INT_WIDTH,&width);
        emStatus = GXGetInt(g2_hDevice,GX_INT_HEIGHT,&height);
        src_image2.create(height,width,CV_8UC3);

        int nRet = pthread_create(&g2_nAcquisitonThreadID, NULL, ProcGetImage2, this);
        if(nRet != 0)
        {
            //Release the memory allocated
            // 释放已经分配的内存
            if (g2_pRGBImageBuf != NULL)
            {
                delete[] g2_pRGBImageBuf;
                g2_pRGBImageBuf = NULL;
            }

            GXCloseDevice(g2_hDevice);
            g2_hDevice = NULL;
            GXCloseLib();     //关闭设备库,释放资源。当停止 GxIAPI 对设备进行的所有控制之后,必须调用此接口来释放资源,与GXInitLib 对应。

            printf("<Failed to create the acquisition g2_hDevice thread, App Exit!>\n");
            exit(nRet);
        }
        else
        {
            printf("<Create the acquisition g2_hDevice thread is successful, App Exit!>\n");

        }


    }

}


//****************开始图像处理****************************//
int DHCamera::ProcessFrame()
{
    //   ×××××××××××××××××××××××××××图像处理线程×××××××××××××××××××××××× //
    int nRet = pthread_create(&g_ImageProcessThreadID, NULL, ImageProcess, this);
    if(nRet != 0)
    {

        printf("<Failed to create the ImageProcess thread, App Exit!>\n");
        exit(nRet);
    }
    else
    {
        printf("<create the ImageProcess thread success!>\n");
    }

    return 0;

}
int DHCamera::Gammaprocess()
{


    //获 取 Gamma 调 节 参 数
    double dGammaParam;
    int nLutLength;
    unsigned char*      pGammaLut;                ///< Gamma look up table

    VxInt64 nColorCorrectionParam  = 0;

    //获 取 对 比 度 调 节 参 数
    int64_t nContrastParam;
    unsigned char* pContrastLut;
    GX_STATUS GxStatus = GXGetInt (g_hDevice, GX_INT_CONTRAST_PARAM, &nContrastParam);
    if (GxStatus != GX_STATUS_SUCCESS)
    {
        return 0;
    }

    emStatus = GXGetFloat (g_hDevice,GX_FLOAT_GAMMA_PARAM, &dGammaParam);
    if (emStatus != GX_STATUS_SUCCESS)
    {
        return 0;
    }
    do
    {
        //获 取 Gamma 查 找 表 的 长 度
        VxInt32 DxStatus= DxGetGammatLut(dGammaParam, NULL, &nLutLength);
        if (DxStatus != DX_OK)
        {
            break;
        }
        //为 Gamma 查 找 表 申 请 空 间
        pGammaLut = new unsigned char[nLutLength];
        if (pGammaLut== NULL)
        {
            DxStatus= DX_NOT_ENOUGH_SYSTEM_MEMORY;
            break;
        }
        //计 算 Gamma 查 找 表
        dGammaParam = 1.5;   //想图像亮一点改大这个值
        DxStatus = DxGetGammatLut(dGammaParam, pGammaLut, &nLutLength);
        if (DxStatus != DX_OK)
        {
            break;
        }
        //获 取 对 比 度 查 找 表 的 长 度
        DxStatus= DxGetContrastLut(nContrastParam, NULL, &nLutLength);
        if (DxStatus != DX_OK)
        {
            break;
        }
        //为 对 比 度 查 找 表 申 请 空 间
        pContrastLut = new unsigned char[nLutLength];
        if (pContrastLut == NULL)
        {
            DxStatus= DX_NOT_ENOUGH_SYSTEM_MEMORY;
            break;
        }
        //计 算 对 比 度 查 找 表
        nContrastParam = 50;  //
        DxStatus = DxGetContrastLut(nContrastParam, pContrastLut, &nLutLength);
        if (DxStatus != DX_OK)
        {
            break;
        }

        //设 置 查 找 表 失 败 , 释 放 资 源
        if (DxStatus != DX_OK)
        {
            if (pGammaLut != NULL)
            {
                delete[] pGammaLut;
                pGammaLut = NULL;
            }
            if (pContrastLut!= NULL)
            {
                delete []pContrastLut;
                pContrastLut = NULL;
            }

            return 0;

        }
    }while(0);

    int64_t nWidth,nHeight;
    emStatus = GXGetInt(g_hDevice,GX_INT_WIDTH,&nWidth);
    emStatus = GXGetInt(g_hDevice,GX_INT_HEIGHT,&nHeight);

    //提 升 图 像 的 质 量
    emStatus = DxImageImprovment(g_pRGBImageBuf,g_pRGBImageBuf,nWidth,nHeight, nColorCorrectionParam, pContrastLut, pGammaLut);
    if (pGammaLut!= NULL)
    {
        delete []pGammaLut;
        pGammaLut= NULL;
    }
    if (pContrastLut!= NULL)
    {
        delete []pContrastLut;
        pContrastLut = NULL;
    }

}
//int转换成IP
string INTtoIP(uint32_t num)
{

    string strRet = "";
    for (int i=0;i<4;i++)
    {
        uint32_t tmp=(num>>((3-i)*8))&0xFF;

        char chBuf[8] = "";
        // _itoa_s(tmp, chBuf, 10);
        //chBuf = std::to_string(tmp);
        sprintf(chBuf, "%d", tmp);
        strRet += chBuf;
        if (i < 3)
        {
            strRet += _IP_MARK;
        }
    }

    return strRet;
}

void *ImageProcess(void* image)      // 图像处理线程函数
{
    DHCamera *DH_camera = (DHCamera *)image;
    // ModulesDetect        Modules_Detect;
    //Thread running flag setup
    DH_camera->g_ImageProcessFlag = true;      //线程运行标志启动
    DH_camera->Modules_Detect.ROI_TrackFlag = false;
    //  Modules_Detect.ROI_TrackFlag = false;
    //*******************
    int sockfd;
    int port_out = 12322;
    // 创建socket
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if(-1==sockfd){
        // return false;
        puts("Failed to create socket");
    }
    // 设置地址与端口
    struct sockaddr_in addr;
    socklen_t          addr_len=sizeof(addr);
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(port_out);
    addr.sin_addr.s_addr = inet_addr("192.168.101.119");

    int imageFlag = 1;  //0代表传图1,1代表传图2,2代表什么都不做
    Mat srcimage;



    //pthread_mutex_t mutex=PTHREAD_MUTEX_INITIALIZER;//创建互斥锁并初始化
    //pthread_mutex_lock(&g_ImageProcessThreadID);//对线程上锁，此时其他线程阻塞等待该线程释放锁
    while(DH_camera->g_ImageProcessFlag)
    {

        if(imageFlag == 0)
        {
            if(DH_camera->src_image.data)
            {
                DH_camera->src_image.copyTo(srcimage);
            }
            else
            {
                printf("<Image data1 is NULL,Please check the Image get process!>\n");
            }

        }

        if(imageFlag == 1)
        {
            if(DH_camera->src_image2.data)
            {
                DH_camera->src_image2.copyTo(srcimage);
            }
            else
            {
                printf("<Image data2 is NULL,Please check the Image get process!>\n");
            }
        }

        while(imageFlag == 2)
        {
            cout << "waitting imageFlag equal 0 or 1" << endl;
        }
        if(srcimage.data)
        {

             int boxdetecFlag = 0;
             double t = (double)getTickCount();
             boxdetecFlag = DH_camera->Modules_Detect.Bluebox_Detection(srcimage);   //输入为原始图像，输出为位姿
             t = ((double)getTickCount() - t) / getTickFrequency();
             //   cout<<"Bluebox_Detection time = "<< t*1000 << "ms" << endl;
             //  time (&lEnd);
             double fps = 1.0/t ;
             char string[10];      // 用于存放帧率的字符串

             //sprintf(string, "%.2f", DH_camera->ui32FPS); // 帧率保留两位小数
             sprintf(string, "%.2f", fps); // 帧率保留两位小数
             std::string fpsString("FPS:");
             fpsString += string; // 在"FPS:"后加入帧率数值字符串
             putText(srcimage,               // 图像矩阵
                     fpsString,                // string型文字内容
                     Point(20, 50),         // 文字坐标，以左下角为原点
                     CV_FONT_HERSHEY_COMPLEX_SMALL, // 字体类型
                     1.8,                      // 字体大小
                     cv::Scalar(0, 0, 255));   // 字体颜色（B,G,R）

             if(boxdetecFlag)
             {

                 if(DH_camera->DH_EndPoint.x != 0)
                 {
                     DH_camera->DH_StartPoint = DH_camera->Modules_Detect.ImagePoint[0];
                     circle(srcimage, DH_camera->DH_EndPoint, 3, Scalar(79, 79, 79), 8);
                     line(srcimage, DH_camera->DH_StartPoint,  DH_camera->DH_EndPoint , Scalar(0, 0, 255), 2, 8);
                     // line(srcimage, Point(1219,542) , Point(1176,1020), Scalar(0, 0, 255), 2, 8);
                 }
             }

             putText(srcimage, "RealDx",  Point2f(100, 300), CV_FONT_HERSHEY_COMPLEX_SMALL, 2.2, Scalar(0,0,255),2,8);
             String srealdistancex = std::to_string(DH_camera->realdistance[1]);
             putText(srcimage, srealdistancex,  Point2f(350, 300), CV_FONT_HERSHEY_COMPLEX_SMALL, 2.5, Scalar(0,0,255),2,8);

             putText(srcimage, "RealDy",  Point2f(100, 350), CV_FONT_HERSHEY_COMPLEX_SMALL, 2.2, Scalar(0,0,255),2,8);
             String srealdistancey = std::to_string(DH_camera->realdistance[2]);
             putText(srcimage, srealdistancey,  Point2f(350, 350), CV_FONT_HERSHEY_COMPLEX_SMALL, 2.5, Scalar(0,0,255),2,8);

             imageUDP(srcimage, sockfd, addr);

        }

    }
    close(sockfd);
    printf("<ImageProcess thread Exit!>\n");
}





// ××××××××××××××××××××××××××××××××××× //
//image get thread
// *****************图片获取线程******************** //
void *ProcGetImage(void* pParam)
{

    DHCamera *DH_camera = (DHCamera *)pParam;
    GX_STATUS emStatus = GX_STATUS_SUCCESS;

    //Thread running flag setup
    DH_camera->g_bAcquisitionFlag = true;
    PGX_FRAME_BUFFER pFrameBuffer = NULL;

    time_t lInit;
    time_t lEnd;
    uint32_t ui32FrameCount = 0;

    while(DH_camera->g_bAcquisitionFlag)
    {
        if(!ui32FrameCount)
        {
            time(&lInit);
        }
        // Get a frame from Queue
        // 从队列中读取一帧
        emStatus = GXDQBuf(DH_camera->g_hDevice, &pFrameBuffer, 1000);   //在开始采集之后,通过此接口可以获取一副图像(零拷贝)
        if(emStatus != GX_STATUS_SUCCESS)
        {
            if (emStatus == GX_STATUS_TIMEOUT)
            {
                continue;
            }
            else
            {
                DH_camera->GetErrorString(emStatus);
                break;
            }
        }

        if(pFrameBuffer->nStatus != GX_FRAME_STATUS_SUCCESS)
        {
            printf("<Abnormal Acquisition: Exception code: %d>\n", pFrameBuffer->nStatus);
        }

        else
        {
            // 图像获取成功
            ui32FrameCount++;
            time (&lEnd);

            // Image process        DxRaw8toRGB24:该函数用于将 Bayer 图像转换为 RGB 图像。
            DxRaw8toRGB24((void*)pFrameBuffer->pImgBuf, DH_camera->g_pRGBImageBuf,pFrameBuffer->nWidth,pFrameBuffer->nHeight,RAW2RGB_NEIGHBOUR,DX_PIXEL_COLOR_FILTER(BAYERBG),false);
            DH_camera->Gammaprocess();
            memcpy( DH_camera->src_image.data, DH_camera->g_pRGBImageBuf,pFrameBuffer->nHeight*pFrameBuffer->nWidth*3);

            // Print acquisition info each second.
            if (lEnd - lInit >= 2)
            {

                printf("<Successful  src_image1  acquisition: FrameCount: %u Width: %d Height: %d FrameID: %lu>\n",
                       ui32FrameCount, pFrameBuffer->nWidth, pFrameBuffer->nHeight, pFrameBuffer->nFrameID);
                ui32FrameCount = 0;

            }

        }

        emStatus = GXQBuf(DH_camera->g_hDevice, pFrameBuffer);     //调 用 GXQBuf 将 图 像 buf 放 回 库 中 继 续 采 图
        if(emStatus != GX_STATUS_SUCCESS)
        {

            DH_camera->GetErrorString(emStatus);
            break;
        }

    }

    //   close(sockfd);
    printf("<Acquisition thread Exit!>\n");
}
//  ×××××××××××××××××××××××××××××××××××××××××××××××××××  //

// *****************图片获取线程******************** //
void *ProcGetImage2(void* pParam)
{

    DHCamera *DH_camera = (DHCamera *)pParam;
    GX_STATUS emStatus = GX_STATUS_SUCCESS;

    //Thread running flag setup
    DH_camera->g2_bAcquisitionFlag = true;
    PGX_FRAME_BUFFER pFrameBuffer = NULL;

    time_t lInit;
    time_t lEnd;
    uint32_t ui32FrameCount = 0;

    while(DH_camera->g2_bAcquisitionFlag)
    {
        if(!ui32FrameCount)
        {
            time(&lInit);
        }
        // Get a frame from Queue
        // 从队列中读取一帧
        emStatus = GXDQBuf(DH_camera->g2_hDevice, &pFrameBuffer, 1000);   //在开始采集之后,通过此接口可以获取一副图像(零拷贝)
        if(emStatus != GX_STATUS_SUCCESS)
        {
            if (emStatus == GX_STATUS_TIMEOUT)
            {
                continue;
            }
            else
            {
                DH_camera->GetErrorString(emStatus);
                break;
            }
        }

        if(pFrameBuffer->nStatus != GX_FRAME_STATUS_SUCCESS)
        {
            printf("<Abnormal Acquisition: Exception code: %d>\n", pFrameBuffer->nStatus);
        }

        else
        {
            // 图像获取成功
            ui32FrameCount++;
            time (&lEnd);

            // Image process        DxRaw8toRGB24:该函数用于将 Bayer 图像转换为 RGB 图像。
            DxRaw8toRGB24((void*)pFrameBuffer->pImgBuf, DH_camera->g2_pRGBImageBuf,pFrameBuffer->nWidth,pFrameBuffer->nHeight,RAW2RGB_NEIGHBOUR,DX_PIXEL_COLOR_FILTER(BAYERBG),false);
            memcpy( DH_camera->src_image2.data, DH_camera->g2_pRGBImageBuf,pFrameBuffer->nHeight*pFrameBuffer->nWidth*3);
            // Print acquisition info each second.
            if (lEnd - lInit >= 2)
            {

                printf("<Successful src_image2 acquisition: FrameCount: %u Width: %d Height: %d FrameID: %lu>\n",
                       ui32FrameCount, pFrameBuffer->nWidth, pFrameBuffer->nHeight, pFrameBuffer->nFrameID);
                ui32FrameCount = 0;

            }

        }

        emStatus = GXQBuf(DH_camera->g2_hDevice, pFrameBuffer);     //调 用 GXQBuf 将 图 像 buf 放 回 库 中 继 续 采 图
        if(emStatus != GX_STATUS_SUCCESS)
        {

            DH_camera->GetErrorString(emStatus);
            break;
        }

    }
    printf("<Acquisition thread2 Exit!>\n");
}


//-------------------------------------------------
/**
\brief Allocate the memory for pixel format transform
\return void
*/
//-------------------------------------------------
int DHCamera::PreForAcquisition()
{
    g_pRGBImageBuf = new unsigned char[g_nPayloadSize * 3];
    g_pRaw8Image = new unsigned char[g_nPayloadSize];

    return 0;
}
//-------------------------------------------------
/**
\brief Release the memory allocated
\return void
*/
//-------------------------------------------------
int DHCamera::UnPreForAcquisition()
{
    //Release resources
    if (g_pRaw8Image != NULL)
    {
        delete[] g_pRaw8Image;
        g_pRaw8Image = NULL;
    }
    if (g_pRGBImageBuf != NULL)
    {
        delete[] g_pRGBImageBuf;
        g_pRGBImageBuf = NULL;
    }

    return 0;
}


int DHCamera::Stop()
{
    printf("<DHCamera  Stop  process!>\n");

    //Stop Acquisition thread
    g_bAcquisitionFlag = false;
    pthread_join(g_nAcquisitonThreadID, NULL);

    //Stop ImageProcess thread
    g_ImageProcessFlag = false;
    pthread_join(g_ImageProcessThreadID, NULL);


    //Stop Acquisition thread2
    if(g2_bAcquisitionFlag = true)
    {
        g2_bAcquisitionFlag = false;
        pthread_join(g2_nAcquisitonThreadID, NULL);

        //Device stop acquisition
        emStatus = GXStreamOff(g2_hDevice);
        if(emStatus != GX_STATUS_SUCCESS)
        {
            //Release the memory allocated
            UnPreForAcquisition();
            GX_VERIFY_EXIT(emStatus);
        }

        emStatus = GXCloseDevice(g2_hDevice);
        if(emStatus != GX_STATUS_SUCCESS)
        {
            GetErrorString(emStatus);
            g_hDevice = NULL;
            GXCloseLib();
            return emStatus;
        }

    }

    //
    destroyAllWindows();

    //Device stop acquisition
    emStatus = GXStreamOff(g_hDevice);
    if(emStatus != GX_STATUS_SUCCESS)
    {
        //Release the memory allocated
        UnPreForAcquisition();
        GX_VERIFY_EXIT(emStatus);
    }

    //Release the resources and stop acquisition thread
    UnPreForAcquisition();

    //Close device
    emStatus = GXCloseDevice(g_hDevice);
    if(emStatus != GX_STATUS_SUCCESS)
    {
        GetErrorString(emStatus);
        g_hDevice = NULL;
        GXCloseLib();
        return emStatus;
    }

    //Release libary
    emStatus = GXCloseLib();
    if(emStatus != GX_STATUS_SUCCESS)
    {
        GetErrorString(emStatus);
        return emStatus;
    }


    printf("<App exit!>\n");
    return 0;
}

//----------------------------------------------------------------------------------
/**
\brief  Get description of input error code
\param  emErrorStatus  error code

\return void
*/
//----------------------------------------------------------------------------------
void DHCamera::GetErrorString(GX_STATUS emErrorStatus)
{
    char *error_info = NULL;
    size_t size = 0;
    GX_STATUS emStatus = GX_STATUS_SUCCESS;

    // Get length of error description
    emStatus = GXGetLastError(&emErrorStatus, NULL, &size);
    if(emStatus != GX_STATUS_SUCCESS)
    {
        printf("<Error when calling GXGetLastError>\n");
        return;
    }

    // Alloc error resources
    error_info = new char[size];
    if (error_info == NULL)
    {
        printf("<Failed to allocate memory>\n");
        return ;
    }

    // Get error description
    emStatus = GXGetLastError(&emErrorStatus, error_info, &size);
    if (emStatus != GX_STATUS_SUCCESS)
    {
        printf("<Error when calling GXGetLastError>\n");
    }
    else
    {
        printf("%s\n", (char*)error_info);
    }

    // Realease error resources
    if (error_info != NULL)
    {
        delete []error_info;
        error_info = NULL;
    }
}
//Show error message
GX_STATUS DHCamera::GX_VERIFY(GX_STATUS emStatus)
{
    if (emStatus != GX_STATUS_SUCCESS)
    {
        GetErrorString(emStatus);
        return emStatus;
    }
}

//Show error message, close device and lib
GX_STATUS DHCamera::GX_VERIFY_EXIT(GX_STATUS emStatus)
{
    if (emStatus != GX_STATUS_SUCCESS)
    {
        GetErrorString(emStatus);
        GXCloseDevice(g_hDevice);
        g_hDevice = NULL;
        GXCloseLib();
        printf("<App Exit!>\n");
        return emStatus;
    }
}
