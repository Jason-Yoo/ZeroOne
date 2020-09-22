#include "BoxProcess.h"

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
//  ××××××××××××××××××××××××××××  //


int BoxProcess::VisualRecognition_Init(DHCamera &DH_Camera)
{
    //Device Init
    DH_Camera.Init();     //相机初始化

    DH_Camera.DH_Camera(); //

    //Device start acquisition
    DH_Camera.Read();   // 内有图片获取的线程

    //start image process    开始图像处理线程
    DH_Camera.ProcessFrame();

    return 1;
}
void BoxProcess::GetimagePoints(DHCamera & DH_Camera , int16_t VisionMessage[])
{
    // VisionMessage[0] = 0xeb;
    // VisionMessage[1] = 0x90;
    // VisionMessage[2] = 0x90;
    // VisionMessage[3] = 0xA1;
    // VisionMessage[4] = 0xA3;
    // VisionMessage[5] = 0x01; //ID
    uint num = 0;
    //  int16_t sum=0;
    for(uint i = 0 ; i < 7 ; i++)
    {
        VisionMessage[num]   = int16_t(DH_Camera.Modules_Detect.ImagePoint[i].x);
        VisionMessage[num+1] = int16_t(DH_Camera.Modules_Detect.ImagePoint[i].y);

        //     sum =  sum+ VisionMessage[num]+ VisionMessage[num+1];
        num = num+2;
        //   cout<<"imagePoints "<< i << " =" << DH_Camera.Modules_Detect.ImagePoint[i] <<endl;

    }
    VisionMessage[num-1] = BoxStage;  //01 pingfang
    VisionMessage[num] = 0xFF;  //01 pingfang
    // VisionMessage[20] = 0;
    // VisionMessage[21] = sum;
    // VisionMessage[22] = 0x0d;
    // for(uint i = 0 ; i < num+1; i++)
    // {
    //      cout<<"VisionMessage "<< i << " =" << int16_t(VisionMessage[i]) <<endl;
    // }
}
int  BoxProcess::GetRealdistance(DHCamera & DH_Camera , Point2f EndPoint, float Realdistance[],float UavHeight)
{
    Point2f startpoint = DH_Camera.Modules_Detect.ImagePoint[0];
    Point2f offset_X  = Point2f(EndPoint.x,startpoint.y);
    Point2f offset_Y  = Point2f(startpoint.x,EndPoint.y);
    DH_Camera.DH_EndPoint   = EndPoint;

    Realdistance[0] = DH_Camera.Modules_Detect.PointDistance(startpoint,EndPoint,DH_Camera.DHcamera1dx,UavHeight);
    Realdistance[1] = DH_Camera.Modules_Detect.PointDistance(startpoint,offset_X,DH_Camera.DHcamera1dx,UavHeight);
    Realdistance[2] = DH_Camera.Modules_Detect.PointDistance(startpoint,offset_Y,DH_Camera.DHcamera1dx,UavHeight);

    DH_Camera.realdistance[0] = Realdistance[0];
    DH_Camera.realdistance[1] = Realdistance[1];
    DH_Camera.realdistance[2] = Realdistance[2];

    DH_Camera.Modules_Detect.judgeBoxState(DH_Camera.Modules_Detect.ImagePoint,UavHeight);

    if( DH_Camera.Modules_Detect.is_parallel)
        BoxStage = 1;
    if( DH_Camera.Modules_Detect.is_stand)
        BoxStage = 0;   
}
void BoxProcess::GetboxPosition(DHCamera & DH_Camera , Point3d boxPosition)
{
    boxPosition.x = DH_Camera.BoxPosition.x;
    boxPosition.y = DH_Camera.BoxPosition.y;
    boxPosition.z = DH_Camera.BoxPosition.z;
}
int BoxProcess::GetCamareStop(DHCamera &DH_Camera)
{
    //Device Stop
    if(!DH_Camera.Stop())
        return 1;
    else
        return 0;
}
//void BoxProcess::SetudpAddress(DHCamera &DH_Camera, uint32_t address)
//{
//    //  DH_Camera.udpAddress = address;

//}
void BoxProcess::GetImageshow(DHCamera &DH_Camera)
{
    DH_Camera.g_ImageShowFlag = true;
}
void BoxProcess::GetImage0ff(DHCamera &DH_Camera)
{
    DH_Camera.g_ImageShowFlag = false;
}
int main()
{
    BoxProcess Box_Process;
    DHCamera  DH_Camera;
    char chStartKey = 0;
    bool bWaitStart = true;
    int16_t VisionMessage[23] = {0};
    float   distance[10] = {0};
    Point2f EndPoint  = Point2f(1141,784);

    Box_Process.VisualRecognition_Init(DH_Camera);    //相机初始化、图像获取、图像处理线程
    Box_Process.GetImageshow(DH_Camera);

    while(bWaitStart)
    {
        chStartKey = getchar();
        switch(chStartKey)
        {
        case 'x':
        case 'X':
            bWaitStart = false;
            break;
        case 's':
        case 'S':
            Box_Process.GetimagePoints(DH_Camera, VisionMessage);
            Box_Process.GetRealdistance(DH_Camera,EndPoint,distance,31);
            break;
        default:
            break;
        }


    }
    Box_Process.GetCamareStop(DH_Camera);

}
