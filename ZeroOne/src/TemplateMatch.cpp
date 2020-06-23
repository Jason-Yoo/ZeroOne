#include <TemplateMatch.h>

void *TrackingPthread(void* image)
{
    char MatchKey = 0;
    TemplateMatch *Template_Match = (TemplateMatch *)image;
    //Thread running flag setup
    Template_Match->TemplateMatchFlag = true;
    Template_Match->rectImage=imread("rectImage.jpg");

    while(Template_Match->TemplateMatchFlag)
    {
        if(Template_Match->image.data);
        {

            int choice = 6;
            double t = (double)getTickCount();
            while (choice)
            {

                if (Template_Match->rectImage.data)
                {
                   Template_Match->ImageResult = Mat::zeros( Template_Match->resultRows, Template_Match->resultcols, CV_32FC1);//建立结果矩阵，注意是单通道的32位浮点型
                    switch (choice) { //根据一开始的选择使用对应的匹配模式
                    case 1:
                       matchTemplate(Template_Match->image, Template_Match->rectImage, Template_Match->ImageResult, TM_SQDIFF);
                        break;
                    case 2:
                       matchTemplate(Template_Match->image, Template_Match->rectImage, Template_Match->ImageResult, TM_SQDIFF_NORMED);
                        break;
                    case 3:
                        matchTemplate(Template_Match->image, Template_Match->rectImage, Template_Match->ImageResult, TM_CCORR);
                        break;
                    case 4:
                       matchTemplate(Template_Match->image,  Template_Match->rectImage,  Template_Match->ImageResult, TM_CCORR_NORMED);
                        break;
                    case 5:
                       matchTemplate(Template_Match->image,  Template_Match->rectImage,  Template_Match->ImageResult, TM_CCOEFF);
                        break;
                    case 6:
                      matchTemplate(Template_Match->image,  Template_Match->rectImage,  Template_Match->ImageResult, TM_CCOEFF_NORMED);
                        break;
                    }
                    minMaxLoc( Template_Match->ImageResult, &Template_Match->minValue, &Template_Match->maxValude, &Template_Match->minPoint, &Template_Match->maxPoint, Mat());  //最小值最大值获取
                    Point point;
                    switch (choice) {//为了统一处理，所以先把Point取出来
                    case 1:
                        point = Template_Match->minPoint;
                        break;
                    case 2:
                        point = Template_Match->minPoint;
                        break;
                    case 3:
                        point = Template_Match->maxPoint;
                        break;
                    case 4:
                        point = Template_Match->maxPoint;
                        break;
                    case 5:
                        point = Template_Match->maxPoint;
                        break;
                    case 6:
                        point = Template_Match->maxPoint;
                        break;
                    }
                    rectangle(Template_Match->image, point, Point(point.x +  Template_Match->rectImage.cols, point.y +  Template_Match->rectImage.rows), Scalar(0, 0, 255), 2); //绘制
                    //更新当前模板匹配的模板
                   // Mat resultImage = Template_Match->image(Rect(point, Point(point.x +  Template_Match->rectImage.cols, point.y +  Template_Match->rectImage.rows)));
                   // Template_Match->rectImage = resultImage.clone();

                    t = ((double)getTickCount() - t) / getTickFrequency();
                    double fps = 1.0/t;

                    char string[10];
                                   sprintf(string, "x=%d y=%d fps=%.2f", point.x,point.y,fps); // 帧率保留两位小数
                                   std::string targetString("target:");
                                   targetString += string;
                                      putText(Template_Match->image,               // 图像矩阵
                                              targetString,                // string型文字内容
                                              cv::Point(5, 20),         // 文字坐标，以左下角为原点
                                              cv::FONT_HERSHEY_SIMPLEX, // 字体类型
                                              0.6,                      // 字体大小
                                              cv::Scalar(0, 255, 0));   // 字体颜色（B,G,R）

                    imshow("Template_Match",Template_Match->image);
                    waitKey(1);
                    break;
             }

            break;
           }
        }

    }



}
void TemplateMatch::ImageTracking(Mat src_image)
{
       if(src_image.data)
       {

            image=src_image;
           // Ptr<Tracker> tracker = TrackerKCF::create();
            int nRet = pthread_create(&TemplateMatchThreadID, NULL, TrackingPthread, this);
            if(nRet != 0)
            {

                printf("<Failed to create the TemplateMatch thread, App Exit!>\n");
                exit(nRet);
            }
            else
            {
                printf("<create the TemplateMatch thread success!>\n");
            }

       }
       else
       {
           printf("<Failed to start TemplateMatch thread for geting src_image, App Exit!>\n");
       }



}
//Update Template
void TemplateMatch::UPTemplate(Mat src_image)
{

    printf("Press [R] or [r] and then press [Enter] to save Template_image for Tracking\n");
   // printf("Press [x] or [X] and then press [Enter] to Exit the UPTemplate Program\n");

    RotatedRect check_box;
    char chKey = 0;

    check_box.center.x=ImageWidth/2;
    check_box.center.y=ImageHeight/2;
    check_box.size=Size2f(ImageWidth/8,ImageHeight/8);
    bool  UPTemplate_Flag = true ;
    while(UPTemplate_Flag)
    {
        //画矩形
        Point2f rect[4];
        check_box.points(rect);
        for (int j = 0; j < 4; j++)
        {
            line(src_image, rect[j], rect[(j + 1) % 4], Scalar(0,0,255), 2, 8);  //绘制最小外接矩形每条边
        }

        //设置绘制文本的相关参数
        std::string text = "Set_Template with 'S' ";
        int font_face = cv::FONT_HERSHEY_COMPLEX;
        double font_scale = 1;
        int thickness = 1;
        int baseline;
        //获取文本框的长宽
        cv::Size text_size = cv::getTextSize(text, font_face, font_scale, thickness, &baseline);
        putText(src_image, text, cvPoint(rect[1].x, rect[1].y-10), font_face, font_scale, cv::Scalar(255, 0, 0), thickness, 8, 0);
        imshow("UPTemplate",src_image);

        chKey = getchar();
        switch(chKey)
        {
        //Save  Image
        case 'R':
        case 'r':
            // 获取ROI
            imageCopy = src_image(Rect(rect[0], rect[2]));
            rectImage = imageCopy.clone();              //给全局的待匹配图像
            resultRows = image.rows - rectImage.rows + 1;   //输出结果图像的行数及列数
            resultcols = image.cols - rectImage.rows + 1;
            imwrite("rectImage.jpg",rectImage);
            imshow("rectImage",rectImage);
            printf("save Template_image for Tracking is successful\n");
            UPTemplate_Flag = false;
            break;

        case 'X':
        case 'x':
            UPTemplate_Flag = false;
            break;
        default:
            break;
        }

    }




}

//鼠标回调函数
void TemplateMatch::onMouse(int event, int x, int y, int flags, void *ustc)
{
    if (event == CV_EVENT_LBUTTONDOWN)   //检测到左键按下时
    {
        leftButtonDownFlag = true; //标志位为true，也就是停止读取下一帧图像
        beginPoint = Point(x, y);  //设置左键按下点的矩形起点
        endPoint = beginPoint;
    }
    if (event == CV_EVENT_MOUSEMOVE && leftButtonDownFlag)
    {                               //当鼠标移动且之前左键有按下的话
        imageCopy = image.clone();
        endPoint = Point(x, y);
        if (beginPoint != endPoint)
        {
            //在复制的图像上绘制矩形
            rectangle(imageCopy, beginPoint, endPoint, Scalar(0, 0, 255), 2);
        }
        //imshow("Video", imageCopy);
    }
    if (event == CV_EVENT_LBUTTONUP) //左键放开时，开始匹配
    {
        leftButtonDownFlag = false;
        Mat subImage = image(Rect(beginPoint, endPoint)); //截取图像
        rectImage = subImage.clone();              //给全局的待匹配图像
         resultRows = image.rows - rectImage.rows + 1;   //输出结果图像的行数及列数
        resultcols = image.cols - rectImage.rows + 1;

    }
}
