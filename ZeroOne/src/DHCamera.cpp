/// Please see camera development documentation.

#include "DHCamera.h"

#define ACQ_BUFFER_NUM          5               ///< Acquisition Buffer Qty.
#define ACQ_TRANSFER_SIZE       (64 * 1024)     ///< Size of data transfer block
#define ACQ_TRANSFER_NUMBER_URB 64              ///< Qty. of data transfer block
#define FILE_NAME_LEN           50              ///< Save image file name length

#define PIXFMT_CVT_FAIL             -1             ///< PixelFormatConvert fail
#define PIXFMT_CVT_SUCCESS          0              ///< PixelFormatConvert success


DHCamera::DHCamera()
{

    g_pRgbBuffer[0] = NULL;
    g_pRgbBuffer[1] = NULL;
    updated = false;

    //相机内参数
    Mat_<double> cameraMatrix(3, 3);
    double fx = 736.1196;
    double fy = 739.3896;
    double Cx = 648.7056;
    double Cy = 512.9957;
    cameraMatrix << fx, 0, Cx, 0, fy, Cy, 0, 0, 1;
    //畸变系数;
    Mat_<double> distCoeffs(1, 5);
    distCoeffs << -0.2972, 0.0742, 0.00000, -0.00000, 0.00000;
}

int DHCamera::Init()
{
    printf("CAMERA SDK INIT...\n");
    GX_STATUS emStatus = GX_STATUS_SUCCESS;
    //Initialize libary
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
        GXCloseLib();
        return emStatus;
    }

    //If no device found, app exit
    if(ui32DeviceNum <= 0)
    {
        printf("<No device found>\n");
        GXCloseLib();
        return emStatus;
    }

    //Open first device enumerated
    emStatus = GXOpenDeviceByIndex(1, &g_hDevice);
    if(emStatus != GX_STATUS_SUCCESS)
    {
        GetErrorString(emStatus);
        GXCloseLib();
        return emStatus;
    }
    printf("DONE\n");

    //Get the type of Bayer conversion. whether is a color camera.
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
    {
        emStatus = GXGetEnum(g_hDevice, GX_ENUM_PIXEL_COLOR_FILTER, &g_i64ColorFilter);
        GX_VERIFY_EXIT(emStatus);
    }

    //DC_design
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
                   bWaitStart = false;
                   break;
               case 'S':
               case 's':
                   printf("<Please start acquisiton before show image!>\n");
                   break;
               case 'x':
               case 'X':
                   //App exit
                   GXCloseDevice(g_hDevice);
                   g_hDevice = NULL;
                   GXCloseLib();
                   printf("<App exit!>\n");
                   return 0;
               default:
                   break;
           }
       }

    emStatus = GXGetInt(g_hDevice, GX_INT_PAYLOAD_SIZE, &g_nPayloadSize);
    GX_VERIFY(emStatus);

    //Set acquisition mode
    emStatus = GXSetEnum(g_hDevice, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
    GX_VERIFY_EXIT(emStatus);

    //Set trigger mode
    emStatus = GXSetEnum(g_hDevice, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_OFF);
    GX_VERIFY_EXIT(emStatus);

    //Set buffer quantity of acquisition queue
    uint64_t nBufferNum = ACQ_BUFFER_NUM;
    emStatus = GXSetAcqusitionBufferNumber(g_hDevice, nBufferNum);
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
    //Set Balance White Mode : Continuous
    emStatus = GXSetEnum(g_hDevice, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_ONCE);
    GX_VERIFY_EXIT(emStatus);

    //Set  Exposure
    emStatus = GXSetFloat(g_hDevice, GX_FLOAT_EXPOSURE_TIME, 13000.0000);
    GX_VERIFY_EXIT(emStatus);


    //Allocate the memory for pixel format transform
    PreForAcquisition();

}

int DHCamera::Uninit()
{

}

int DHCamera::Read()
{
        //Device start acquisition
        emStatus = GXStreamOn(g_hDevice);
        if(emStatus != GX_STATUS_SUCCESS)
        {
            //Release the memory allocated
            UnPreForAcquisition();
            GX_VERIFY_EXIT(emStatus);
        }

        int64_t width,height;
        emStatus = GXGetInt(g_hDevice,GX_INT_WIDTH,&width);
        emStatus = GXGetInt(g_hDevice,GX_INT_HEIGHT,&height);
        src_image.create(height,width,CV_8UC3);

       // mutex1.lock();
        //Start acquisition thread, if thread create failed, exit this app
        int nRet = pthread_create(&g_nAcquisitonThreadID, NULL, ProcGetImage, this);
        if(nRet != 0)
        {
            //Release the memory allocated
            UnPreForAcquisition();

            GXCloseDevice(g_hDevice);
            g_hDevice = NULL;
            GXCloseLib();

            printf("<Failed to create the acquisition thread, App Exit!>\n");
            exit(nRet);
        }
        else
        {
            printf("<Create the acquisition thread is successful, App Exit!>\n");

        }
       // mutex1.unlock();


}
int DHCamera::Set_fps(int fps_mode)
{

}
///
/// \brief DHCamera::GetFrame_B
/// \param frame
/// \param is_color
/// true if we want bgr img
/// \return
///
int DHCamera::GetFrame()
{

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

void *ImageProcess(void* image)
{
     DHCamera *DH_camera = (DHCamera *)image;
     //Thread running flag setup
     DH_camera->g_ImageProcessFlag = true;
     ModulesDetect Modules_Detect;
     Modules_Detect.ROI_TrackFlag = false;

 //    VideoWriter writer;
 //    int frameRate = 20;
 //    writer.open("./uavgp.avi",CV_FOURCC('H', '2', '6', '4'),frameRate, Size(DH_camera->src_image.cols,DH_camera->src_image.rows),1);

     while(DH_camera->g_ImageProcessFlag)
     {

        // if(DH_camera->g_bSaveVedioFlag)
       //  writer.write(DH_camera->src_image);

         if(DH_camera->src_image.data)
         {

             double t = (double)getTickCount();

             Modules_Detect.Bluebox_Detection(DH_camera->src_image,2);
             t = ((double)getTickCount() - t) / getTickFrequency();
             //double fps = 1.0/t;
             cout<<"Bluebox_Detection time = "<< t*1000 << "ms" << endl;
             namedWindow("DH_camera:",CV_WINDOW_AUTOSIZE);
             imshow("DH_camera:",DH_camera->src_image);
             waitKey(1);
         }
         else
         {
             printf("<Image data is NULL,Please check the Image get process!>\n");

         }

     }

     printf("<ImageProcess thread Exit!>\n");

}
//image get thread
void *ProcGetImage(void* pParam)
{
    DHCamera *DH_camera = (DHCamera *)pParam;
    ModulesDetect Modules_Detect;
    GX_STATUS emStatus = GX_STATUS_SUCCESS;

    //Thread running flag setup
    DH_camera->g_bAcquisitionFlag = true;
    PGX_FRAME_BUFFER pFrameBuffer = NULL;

    time_t lInit;
    time_t lEnd;
    uint32_t ui32FrameCount = 0;

    VideoWriter writer;
    int frameRate = 20;
    writer.open("./uavgp.avi",CV_FOURCC('H', '2', '6', '4'),frameRate, Size(DH_camera->src_image.cols,DH_camera->src_image.rows),1);

    while(DH_camera->g_bAcquisitionFlag)
    {
        if(!ui32FrameCount)
        {
            time(&lInit);
        }

        // Get a frame from Queue
        emStatus = GXDQBuf(DH_camera->g_hDevice, &pFrameBuffer, 1000);
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
            ui32FrameCount++;
            time (&lEnd);

            
            // Image process
            DxRaw8toRGB24((void*)pFrameBuffer->pImgBuf, DH_camera->g_pRGBImageBuf,pFrameBuffer->nWidth,pFrameBuffer->nHeight,RAW2RGB_NEIGHBOUR,DX_PIXEL_COLOR_FILTER(BAYERBG),false);
            memcpy( DH_camera->src_image.data, DH_camera->g_pRGBImageBuf,pFrameBuffer->nHeight*pFrameBuffer->nWidth*3);

           // Modules_Detect.RecognitionFailure(DH_camera->src_image);
            //namedWindow("DH_camera:",CV_WINDOW_AUTOSIZE);
            //imshow("DH_camera:",DH_camera->src_image);
            //waitKey(1);
             if(DH_camera->g_bSaveVedioFlag)
             writer.write(DH_camera->src_image);


            // Print acquisition info each second.
            if (lEnd - lInit >= 1)
            {
                printf("<Successful acquisition: FrameCount: %u Width: %d Height: %d FrameID: %llu>\n",
                ui32FrameCount, pFrameBuffer->nWidth, pFrameBuffer->nHeight, pFrameBuffer->nFrameID);
                ui32FrameCount = 0;

            }

        }

        emStatus = GXQBuf(DH_camera->g_hDevice, pFrameBuffer);
        if(emStatus != GX_STATUS_SUCCESS)
        {
            DH_camera->GetErrorString(emStatus);
            break;
        }
    }
    printf("<Acquisition thread Exit!>\n");
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

    //Stop Acquisition thread
    g_bAcquisitionFlag = false;
    pthread_join(g_nAcquisitonThreadID, NULL);

    //Stop ImageProcess thread
    g_ImageProcessFlag = false;
    pthread_join(g_ImageProcessThreadID, NULL);

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
