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


int VisualRecognition_Init(DHCamera &DH_Camera)
{
    //Device Init
    DH_Camera.Init();     //相机初始化

    //Device start acquisition
    DH_Camera.Read();   // 内有图片获取的线程

    //start image process    开始图像处理线程
   DH_Camera.ProcessFrame();

    return 1;

}

int main()
{
    DHCamera  DH_Camera;
    Point3d   BoxPosition;

    VisualRecognition_Init(DH_Camera);    //相机初始化、图像获取、图像处理线程

    while(1);

}
