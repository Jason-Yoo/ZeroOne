
#include <BoxProcess.h>

void Camera_task()
{
    DHCamera      DH_Camera;
    TemplateMatch Template_Match;

    //Device Init
    DH_Camera.Init();

//     Device start acquisition
    DH_Camera.Read();

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
            //pthread_create(&DHCamera.g_ImageProcessThreadID, NULL, ImageProcess, (void*)&DHCamera);
            DH_Camera.GetFrame();
            break;
        //Save  Vedio
        case 'V':
        case 'v':
            DH_Camera.g_bSaveVedioFlag = !DH_Camera.g_bSaveVedioFlag;
            break;
        //
        case 'T':
        case 't':
            //Template_Match.ImageTracking(DH_Camera.src_image);
            break;
        //save Image
        case 'U':
        case 'u':
            imwrite("src_image.jpg",DH_Camera.src_image);;
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
    //Stop TemplateMatch thread
    Template_Match.TemplateMatchFlag = false;
    pthread_join( Template_Match.TemplateMatchThreadID, NULL);
    DH_Camera.Stop();

}
void Vedio_task()
{
    printf("*********The video  task  is runing *************** \n");
    VideoCapture inputVideo;
    inputVideo.open("DJI_0022.avi");
    ModulesDetect Modules_Detect;
    Mat g_srcImage;        //原始图像

       while (1)
       {
           if (inputVideo.isOpened() == 0)
               break;
           if (inputVideo.isOpened())
           {

               inputVideo.read(g_srcImage);
               Modules_Detect.RecognitionFailure(g_srcImage);

           }

           // 创建新窗口
           namedWindow("Vedio_task", WINDOW_NORMAL);
           imshow("Vedio_task", g_srcImage);
           waitKey(5);
       }
       waitKey(0);
}


int main()
{
    printf("\n");
    printf("Press [C] or [c] and then press [Enter] to start camera task\n");
    printf("Press [V] or [v] and then press [Enter] to start video  task\n");
    printf("Initializing.........");
    printf("\n\n");

    bool CRun = false;
    bool VRun = false;
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
        }
        if(CRun) break;
        if(VRun) break;
    }
    if(CRun) Camera_task();
    if(VRun) Vedio_task();

    waitKey(0);

}
