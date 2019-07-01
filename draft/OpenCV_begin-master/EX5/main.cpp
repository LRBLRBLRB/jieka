/**
 * @brief:膨胀，腐蚀测试
 * @author：liuyvjin
 */
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;

Mat src ,dst;
void switchfun(int switchbar,void *sizeptr);
void sizefun(int sizebar, void *switchptr);
int main() {

    //读入原始图像
    src = imread("../../pictures/zhupi.png");
    if(!src.data){printf("读入src图片错误！\n");return -1;}
    imshow("src",src);

    //定义控制条
    namedWindow("dst: 0-dilate;1-erode");
    int switchbar = 0;  //切换腐蚀和膨胀 0/1
    int sizebar = 3; //改变视界大小
    createTrackbar("switch","dst: 0-dilate;1-erode",&switchbar,1,switchfun,&sizebar);
    createTrackbar("size","dst: 0-dilate;1-erode",&sizebar,10,sizefun,&switchbar);

    //初始化窗口
    Mat element = getStructuringElement(MORPH_RECT,Size(2*sizebar+1,2*sizebar+1),Point(sizebar, sizebar));
    dilate(src,dst, element);
    imshow("dst: 0-dilate;1-erode", dst);

    //等待键盘输入
    waitKey();
    return 0;
}

void switchfun(int switchbar,void *sizeptr)
{
    int sizebar = *static_cast<int*>(sizeptr);
    Mat element = getStructuringElement(MORPH_RECT,Size(2*sizebar+1,2*sizebar+1),Point(sizebar, sizebar));

    switch (switchbar){
        case 0:
            dilate(src,dst,element);
            break;
        case 1:
            erode(src,dst,element);
            break;
        default: ;
    }
    imshow("dst: 0-dilate;1-erode",dst);
}
void sizefun(int sizebar, void *switchptr)
{
    int switchbar = *static_cast<int*>(switchptr);
    Mat element = getStructuringElement(MORPH_RECT,Size(2*sizebar+1,2*sizebar+1),Point(sizebar, sizebar));

    switch (switchbar){
        case 0:
            dilate(src,dst,element);
            break;
        case 1:
            erode(src,dst,element);
            break;
        default: ;
    }
    imshow("dst: 0-dilate;1-erode",dst);
}