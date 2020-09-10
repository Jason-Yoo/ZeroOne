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

    //Device start acquisition
    DH_Camera.Read();   // 内有图片获取的线程

    //start image process    开始图像处理线程
    DH_Camera.ProcessFrame();

    return 1;
}
void BoxProcess::GetimagePoints(DHCamera & DH_Camera , int16_t VisionMessage[])
{
    VisionMessage[0] = 0xeb;
    VisionMessage[1] = 0x90;
    VisionMessage[2] = 0x90;
    VisionMessage[3] = 0xA1;
    VisionMessage[4] = 0xA3;
    VisionMessage[5] = 0x01;
    uint num = 6;
    for(uint i = 0 ; i < 7 ; i++)
    {
        VisionMessage[num] = int16_t(DH_Camera.Modules_Detect.ImagePoint[i].x);
        VisionMessage[num+1] =  int16_t(DH_Camera.Modules_Detect.ImagePoint[i].y);
        num = num+2;
        cout<<"imagePoints "<< i << " =" << DH_Camera.Modules_Detect.ImagePoint[i] <<endl;

    }
    for(uint i = 0 ; i < 32; i++)
    {
         cout<<"VisionMessage "<< i << " =" << int16_t(VisionMessage[i]) <<endl;
    }


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
    int16_t VisionMessage[32] = {0};
    Box_Process.VisualRecognition_Init(DH_Camera);    //相机初始化、图像获取、图像处理线程

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
            Box_Process.GetImageshow(DH_Camera);
            Box_Process.GetimagePoints(DH_Camera, VisionMessage);
            break;
        default:
            break;
        }

    }
    Box_Process.GetCamareStop(DH_Camera);

}
