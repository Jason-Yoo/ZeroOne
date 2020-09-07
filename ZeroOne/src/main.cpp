
#include <iostream>

#include "opencv2/core/opengl.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/cudaimgproc.hpp"
#include "opencv2/cudaarithm.hpp"

using namespace std;
using namespace cv;
using namespace cv::cuda;

//int main (int argc, char* argv[])
//{
//    try
//    {
//        ///　读取图片
//        cv::Mat src_host = cv::imread("test.bmp", cv::IMREAD_GRAYSCALE);
//        /// 定义GpuMat
//        cv::cuda::GpuMat dst, src;
//        /// 将主机内存的图像数据上传到GPU内存
//        src.upload(src_host);

//        /// 调用GPU的阈值函数(很多使用GPU加速的函数都和CPU版本的函数相同)
//        cv::cuda::threshold(src, dst, 20, 255, cv::THRESH_BINARY);

//        cv::Mat result_host;
//        /// 从GPU上下载阈值化完成的图片
//        dst.download(result_host);

//        /// 显示
//        cv::imshow("Result", result_host);
//        cv::waitKey();
//    }
//    catch(const cv::Exception& ex)
//    {
//        std::cout << "Error: " << ex.what() << std::endl;
//    }
//    return 0;
//}
