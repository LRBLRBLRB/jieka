/**@brief: Canny边缘检测
*/
#include <opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>

using namespace cv;

int main( )
{
    //载入原始图
    Mat src = imread("../../pictures/zhupi.png");
    if(!src.data ) { printf("读取srcImage错误！ \n"); return false; }

    Mat src1=src.clone();

    //显示原始图
    imshow("【原始图】Canny边缘检测", src);

    //----------------------------------------------------------------------------------
    //	一、最简单的canny用法，拿到原图后直接用。
    //----------------------------------------------------------------------------------
    //Canny( src, src, 150, 100,3 );
    //imshow("【效果图】Canny边缘检测", src);


    //----------------------------------------------------------------------------------
    //	二、高阶的canny用法，转成灰度图，降噪，用canny，最后将得到的边缘作为掩码，拷贝原图到效果图上，得到彩色的边缘图
    //----------------------------------------------------------------------------------
    Mat dst,edge,gray;

    // 【1】创建与src同类型和大小的矩阵(dst)
    dst.create( src1.size(), src1.type() );

    // 【2】将原图像转换为灰度图像
    cvtColor( src1, gray, CV_BGR2GRAY );

    // 【3】先用使用 3x3内核来降噪
    blur( gray, edge, Size(3,3) );

    // 【4】运行Canny算子
    Canny( edge, edge, 25, 40,3 );

    //【5】将g_dstImage内的所有元素设置为0
    dst = Scalar::all(0);

    //【6】使用Canny算子输出的边缘图g_cannyDetectedEdges作为掩码，来将原图g_srcImage拷到目标图g_dstImage中
    src1.copyTo( dst, edge);

    //【7】显示效果图
    imshow("【效果图】Canny边缘检测灰度", edge);
    imshow("【效果图】Canny边缘检测彩色", dst);


    waitKey(0);
    return 0;
}