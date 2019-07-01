#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

//-----------------------------------【命名空间声明部分】---------------------------------------
//     描述：包含程序所使用的命名空间
//-----------------------------------------------------------------------------------------------
using namespace std;
using namespace cv;


//-----------------------------------【全局函数声明部分】--------------------------------------
//     描述：全局函数声明
//-----------------------------------------------------------------------------------------------
static void ContrastAndBright(int, void *);

//-----------------------------------【全局变量声明部分】--------------------------------------
//     描述：全局变量声明
//-----------------------------------------------------------------------------------------------
int g_nContrastValue; //对比度值
int g_nBrightValue;  //亮度值
Mat g_srcImage,g_dstImage;
//-----------------------------------【main( )函数】--------------------------------------------
//     描述：控制台应用程序的入口函数，我们的程序从这里开始
//-----------------------------------------------------------------------------------------------
int main(  )
{
    //改变控制台前景色和背景色
    system("color5F");

    //读入用户提供的图像
    g_srcImage= imread( "../../pictures/pic.jpg");
    if(!g_srcImage.data ) { printf("Oh，no，读取g_srcImage图片错误~！\n"); return false; }
    g_dstImage= Mat::zeros( g_srcImage.size(), g_srcImage.type() );

    //设定对比度和亮度的初值
    g_nContrastValue=50;
    g_nBrightValue=50;

    namedWindow("result image", 1);
    //创建轨迹条
    createTrackbar("Contrast", "result image",&g_nContrastValue,100,ContrastAndBright );
    createTrackbar("Brightness","result image",&g_nBrightValue,100,ContrastAndBright );

    //显式先调用一次回调函数
    ContrastAndBright(g_nContrastValue,0);
    ContrastAndBright(g_nBrightValue,0);

    //输出一些帮助信息
    cout<<endl<<"\t请调整滚动条观察图像效果\n\n";

    waitKey();
    return 0;
}


//-----------------------------【ContrastAndBright( )函数】------------------------------------
//     描述：改变图像对比度和亮度值的回调函数
//-----------------------------------------------------------------------------------------------
static void ContrastAndBright(int, void *)
{

    //创建窗口
    namedWindow("source image", 1);

    //三个for循环，执行运算 g_dstImage(i,j) =a*g_srcImage(i,j) + b
//    for(int y = 0; y < g_srcImage.rows; y++ )
//    {
//        for(int x = 0; x < g_srcImage.cols; x++ )
//        {
//            for(int c = 0; c < 3; c++ )
//            {
//                g_dstImage.at<Vec3b>(y,x)[c]= saturate_cast<uchar>( (g_nContrastValue*0.02)*(g_srcImage.at<Vec3b>(y,x)[c] ) + g_nBrightValue-50 );
//            }
//        }
//    }
    g_dstImage = g_srcImage*(g_nContrastValue*0.02)+g_nBrightValue-50; //调整图像的亮度和对比度
    //显示图像
    imshow("source image", g_srcImage);
    imshow("result image", g_dstImage);
}
