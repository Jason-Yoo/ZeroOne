system required

ubuntu 16.04

opencv 3.3

daheng_camera_driver

Eigen 

ZeroOne for image process at daheng camera 

2020.09.07  Add UDP For Image  Transmission

2020.09.10  Fix UDP And Targrt ZeroOneVision LIBRARY 

User Init:

BoxProcess Box_Process; //图像处理类定义

DHCamera   DH_Camera;//相机类定义

int16_t VisionMessage[23] = {0};  //用于获取图像处理结果  具体查看协议

User API:

Box_Process.VisualRecognition_Init(DH_Camera);        //相机初始化、图像获取、图像处理线程

Box_Process.GetImageshow(DH_Camera);                  //显示图像处理结果

Box_Process.GetimagePoints(DH_Camera, VisionMessage); //获取处理后的数据

Box_Process.GetCamareStop(DH_Camera);     	      //关闭相机及图像处理线程
