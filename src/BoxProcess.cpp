
#include "GxIAPI.h"
#include "DxImageProc.h"

#include <pthread.h>
#include <assert.h>
#include <string.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


#include <DHCamera.h>
#include <TemplateMatch.h>
#include <SolveSRPnP.h>

using namespace std;
using namespace cv;

class BlueBox
{


};


//int main()
//{
//    printf("\n");
//    printf("Initializing......");
////    printf("\n\n");

//    DHCamera      DH_Camera;
//    TemplateMatch Template_Match;

//    //Device Init
//    DH_Camera.Init();


////     Device start acquisition
//    DH_Camera.Read();


//    //Main loop
//    bool bRun = true;
//    while(bRun == true)
//    {
//        char chKey = getchar();
//        //press 's' and [Enter] to save image;
//        //press 'x' and [Enter] to exit.
//        switch(chKey)
//        {
//        //Show  Image
//        case 'S':
//        case 's':
//            //pthread_create(&DHCamera.g_ImageProcessThreadID, NULL, ImageProcess, (void*)&DHCamera);
//            DH_Camera.GetFrame();
//            break;
//        //Save  Vedio
//        case 'V':
//        case 'v':
//            DH_Camera.g_bSaveVedioFlag = !DH_Camera.g_bSaveVedioFlag;
//            break;
//        //
//        case 'T':
//        case 't':
//            Template_Match.ImageTracking(DH_Camera.src_image);
//            break;
//        //save Image
//        case 'U':
//        case 'u':
//            imwrite("src_image.jpg",DH_Camera.src_image);;
//            break;
//        //Exit app
//        case 'X':
//        case 'x':
//            bRun = false;
//            break;
//        default:
//            break;
//        }

//    }

//    //Stop TemplateMatch thread
//    Template_Match.TemplateMatchFlag = false;
//    pthread_join( Template_Match.TemplateMatchThreadID, NULL);

//    DH_Camera.Stop();

//    waitKey(0);

//}
int main()
{
    Mat srcimage;
    srcimage=imread("src_image.jpg");
    Point2f image_point[4];
    image_point[0] = Point(430,43);
    image_point[1] = Point(870,46);
    image_point[2] = Point(881,915);
    image_point[3] = Point(433,918);
    Calculate_RT(image_point);
    int point_size = 1;
    Scalar point_color = (0,0,255);
    int thickness = 4;
    int n;
    for(n=0;n<4;n++)
    {
         circle(srcimage,image_point[n],point_size,point_color,thickness);
    }
    imshow("src_image",srcimage);
    waitKey(0);

}
