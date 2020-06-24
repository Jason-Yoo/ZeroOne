
#include <BoxProcess.h>

void Camera_task()
{
    DHCamera      DH_Camera;

    //Device Init
    DH_Camera.Init();
    //Device start acquisition
    DH_Camera.Read();

    printf("\n");
    printf("Press [s] or [S] and then press [Enter] to show image from camera\n");
    printf("Press [x] or [X] and then press [Enter] to Exit the Program\n");
    printf("\n");

    //Main loop
    bool bRun = true;
    while(bRun == true)
    {
        char chKey = getchar();
        //press 's' and [Enter] to save image;
        //press 'x' and [Enter] to exit.
        switch(chKey)
        {
        //Show  Image
        case 'S':
        case 's':
            DH_Camera.ProcessFrame();
            break;
        //Save  Vedio
        case 'V':
        case 'v':
            DH_Camera.g_bSaveVedioFlag = !DH_Camera.g_bSaveVedioFlag;
            break;
        //Template_Match
        case 'T':
        case 't':
            //Template_Match.ImageTracking(DH_Camera.src_image);
            break;
        //save Image
        case 'U':
        case 'u':
           // imwrite("src_image.jpg",DH_Camera.src_image);;
            break;
        //Exit app
        case 'X':
        case 'x':
            bRun = false;
            break;
        default:
            break;
        }

    }
    //Stop DH_Camera thread
    DH_Camera.Stop();

}
void Vedio_task()
{
    printf("*********The video  task  is runing *************** \n");
    VideoCapture inputVideo;
    inputVideo.open("data/uavgp.avi");
    ModulesDetect Modules_Detect;
    Mat g_srcImage;        //原始图像
    vector<Point3d> BoxPosition;

       while (1)
       {
           if (inputVideo.isOpened() == 0)
               break;
           if (inputVideo.isOpened())
           {

               inputVideo.read(g_srcImage);
               Modules_Detect.Bluebox_Detection(g_srcImage,BoxPosition,2);

           }

           // 创建新窗口
           namedWindow("Vedio_task", WINDOW_NORMAL);
           imshow("Vedio_task", g_srcImage);
           waitKey(10);
       }
       waitKey(0);
} 
void Image_Task()
{
     printf("*********The Image task  is runing *************** \n");
     ModulesDetect Modules_Detect;
     Mat g_srcImage;        //原始图像
     g_srcImage = imread("data/bluebox4.jpg");

     if (g_srcImage.empty())
     printf("*********Not find a Image in film,Please check ********* ");
     else
     {

       //  Modules_Detect.Bluebox_Detection(g_srcImage,2);
         // 创建新窗口
         namedWindow("Image_Task", WINDOW_AUTOSIZE);
         imshow("Image_Task", g_srcImage);

     }

     waitKey(0);

}
int VisualRecognition_Init(DHCamera &DH_Camera)
{
    //Device Init
    DH_Camera.Init();
    //Device start acquisition
    DH_Camera.Read();
    //start image process
    DH_Camera.ProcessFrame();

    return 1;

}

int VisualRecognition_Box(DHCamera &DH_Camera,Point3d &BoxPosition,int mode)
{
    if(mode == 1)
    {
        if(DH_Camera.BoxPosition[1].z >=0 )
        {
            BoxPosition.x = DH_Camera.BoxPosition[1].x;
            BoxPosition.y = DH_Camera.BoxPosition[1].y;
            BoxPosition.z = DH_Camera.BoxPosition[1].z;
            cout << "X=" << BoxPosition.x << endl;
            cout << "Y=" << BoxPosition.y << endl;
            cout << "Z=" << BoxPosition.z << endl;
        }

    }
    if(mode == 2)
    {
        Vedio_task();
    }
    if(mode == 3)
    {
        Image_Task();
    }

    return 0;
}
/*int main()
{
    printf("\n");
    printf("Press [C] or [c] and then press [Enter] to start camera task\n");
    printf("Press [V] or [v] and then press [Enter] to start video  task\n");
    printf("Press [P] or [p] and then press [Enter] to start image  task\n");
    printf("\n\n");

    bool CRun = false;
    bool VRun = false;
    bool PRun = false;
    while(!CRun)
    {
        char StartKey = getchar();
        switch(StartKey)
        {
            case 'C':
            case 'c':
                CRun = true;
                break;
            case 'V':
            case 'v':
                VRun = true;
                break;
            case 'P':
            case 'p':
                PRun = true;
                break;
        }

        if(CRun) break;
        if(VRun) break;
        if(PRun) break;
    }
    if(CRun) Camera_task();
    if(VRun) Vedio_task();
    if(PRun) Image_Task();

    waitKey(0);
}
*/
int main()
{
     DHCamera  DH_Camera;
     Point3d   BoxPosition;

     printf("\n");
     printf("Press [a] or [A] and then press [Enter] to start acquisition\n");
     printf("Press [s] or [S] and then press [Enter] to show image from camera\n");
     printf("Press [x] or [X] and then press [Enter] to Exit the Program\n");
     printf("\n");

     char chStartKey = 0;
     bool bWaitStart = true;
     while (bWaitStart)
     {
         chStartKey = getchar();
         switch(chStartKey)
         {
         //press 'a' and [Enter] to start acquisition;
         //press 'x' and [Enter] to exit.
         case 'a':
         case 'A':
             //Start to acquisition
             VisualRecognition_Init(DH_Camera);
             break;
         case 'S':
         case 's':
             VisualRecognition_Box(DH_Camera,BoxPosition,1);
             break;
         case 'x':
         case 'X':
             printf("<App exit!>\n");
             return 0;
         default:
             break;
         }
     }
}
