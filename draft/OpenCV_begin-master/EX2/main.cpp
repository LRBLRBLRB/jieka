#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;

int main() {
    Mat src_image = imread("../../pictures/rgb.png");
    if(src_image.empty())return 1;
    imshow("src",src_image);
    //保存3通道的mat数组
    vector<Mat> channels(3);
    //分离图像
    split(src_image,channels);
    imshow("ch0",channels.at(0)); //b
    imshow("ch1",channels.at(1)); //g
    imshow("ch2",channels.at(2)); //r

    //merge,显示出红色通道
    Mat dst;
    Mat empty_ch(channels.at(0).size(),CV_8UC1,Scalar(0));
    vector<Mat> merge_mats(3);

    merge_mats.at(0) = empty_ch; //或者用merge_mats.push_back(empty_ch);
    merge_mats.at(1) = empty_ch;
    merge_mats.at(2) = channels.at(2);
    merge(merge_mats,dst);
    imshow("red",dst);

    waitKey();
    return 0;
}