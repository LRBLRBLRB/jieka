/**
 * @brief: 实验各种滤波函数
 */
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;

int main() {
    Mat src, dst;
    //读入图像
    src = imread("../../pictures/xiaoqi2.jpg");
    if(!src.data){cout<<"读入图像错误！"<<endl;return -1;}
    imshow("src image",src);
    //方框滤波&均值滤波blur
    boxFilter(src,dst,-1,Size(9,9));
    imshow("box filter",dst);

    //高斯滤波
    GaussianBlur(src,dst,Size(9,9),2,2);
    imshow("gaussian blur",dst);

    //中值滤波
    medianBlur(src, dst, 7);
    imshow("median blur",dst);
    //双边滤波
    bilateralFilter(src, dst, 25, 25*2, 25/2 );
    imshow("bilateral filter",dst);

    waitKey();
    return 0;
}