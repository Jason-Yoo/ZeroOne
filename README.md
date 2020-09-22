System Required:

ubuntu 16.04

opencv 3.3

daheng_camera_driver

Eigen Library

Updatelist:

2020.03.25  ZeroOne for image process at daheng camera 

2020.09.07  Add UDP For Image  Transmission

2020.09.10  Fix UDP and Targrt ZeroOneVision LIBRARY 

2020.09.15  Add Second Camera for process

2020.09.20  Fix Bug and Add Caculate realdistance

2020.09.22  Delete useless files

Cmakelist Example:

target_link_libraries(${PROJECT_NAME}

    /lib/libZeroOneVision.so

    libpthread.so

    libgxiapi.so

    )

User Init:

BoxProcess Box_Process; //图像处理类定义

DHCamera   DH_Camera;//相机类定义

int16_t VisionMessage[20] = {0};  //用于获取图像处理结果  具体查看协议

float   Realdistance[5] = {0}; //获取世界坐标系里的距离

Point2f EndPoint  = Point2f(583,733);  

User API:

Box_Process.VisualRecognition_Init(DH_Camera);        //相机初始化、图像获取、图像处理线程

Box_Process.GetImageshow(DH_Camera);                  //显示图像处理结果

Box_Process.GetimagePoints(DH_Camera, VisionMessage); //获取处理后的数据

Box_Process.GetCamareStop(DH_Camera);     	      //关闭相机及图像处理线程

Box_Process.GetRealdistance(DH_Camera ,EndPoint,Realdistance,UavHeight); //获取实际距离

