//roi+mask
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>


using namespace cv;
using namespace std;

//ROI叠加
bool ROI_AddImage() {

    //【1】读入图像
    Mat srcImage = imread("../../pictures/pic.jpg");
    Mat logoImage = imread("../../pictures/heibai.jpg");
    //检查是否读入成功
    if (!srcImage.data) {
        printf("你妹，读取srcImage1错误~！ \n");
        return false;
    }
    if (!logoImage.data) {
        printf("你妹，读取logoImage错误~！ \n");
        return false;
    }

    //【2】取出srcImage的一块ROI区域
    Mat imageROI = srcImage(Rect(200, 250, logoImage.cols, logoImage.rows));
    imshow("imageROI", imageROI);//展示ROI区域

    //【3】加载掩模（要和原图logoImage大小相同，通道数要么相同，要么是1通道，而灰度图是单通道的）
    Mat mask = imread("../../pictures/heibai.jpg", 0);
    imshow("mask", mask);//展示掩膜

    //【4】将logoImage在掩膜过滤下（对应掩膜为0的logoImage的像素被过滤），拷贝到ROI
    logoImage.copyTo(imageROI, mask);

    //【5】显示结果
    namedWindow("<1>利用ROI实现图像叠加示例窗口");
    imshow("<1>利用ROI实现图像叠加示例窗口", srcImage);

    return true;
}

//ROI线性混合
bool ROI_LinearBlend(double alpha) {
    //【1】读入图像
    Mat srcImage = imread("../../pictures/pic.jpg");
    Mat logoImage = imread("../../pictures/ubuntu.jpg");
    //检查是否读入成功
    if (!srcImage.data) {
        printf("你妹，读取srcImage1错误~！ \n");
        return false;
    }
    if (!logoImage.data) {
        printf("你妹，读取logoImage错误~！ \n");
        return false;
    }

    //取ROI
    Mat srcROI = srcImage(Rect(100, 100, logoImage.cols, logoImage.rows));
    imshow("srcROI", srcROI);
    imshow("logoImage", logoImage);
    //线性叠加
    addWeighted(srcROI, alpha, logoImage, 1 - alpha, 0., srcROI);
    //结果
    imshow("LinearBlend", srcImage);
}

int main() {
    //ROI_AddImage();
    ROI_LinearBlend(0.6);
    waitKey();
    return 0;
}

