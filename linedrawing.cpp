#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <vector>
#include <opencv.hpp>

using namespace std;
using namespace cv;

void HoughTrans(Mat &srcImage, vector<Vec3f> &circles);
//函数声明！！！！！！！



int main (int argc, char **argv)
{
    //定义原图、灰度图、最终图的Mat对象，和白球、目标球的对象
    Mat srcImage, grayImage, houghImage, dstImage, whiteBall, dstBall;
    
    /// 读入源图片
    srcImage = imread("../pics/ball3.png");
    if (!srcImage.data) {
        printf("读入文件错误！\n");
        return 1;
    }
    printf("按下q或者esc退出程序\n");

    //转化边缘检测后的图为灰度图
    cvtColor(srcImage, grayImage, CV_BGR2GRAY);

    /// 保存找到的圆
    //Vec3f指的是,每一组有三个
    vector<Vec3f>circles;
    /// 霍夫变换，并绘图
    HoughTrans(srcImage,circles);

    /// 分类球
    int row1,row2,col1,col2;
    for (size_t i = 0; i < circles.size(); ++i) {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1])); //circles的三个参数：x,y,radius
        int radius = cvRound(circles[i][2]);

        row1 = max(center.y - radius,0);
        row2 = min(center.y + radius,360);
        col1 = max(center.x - radius,0);
        col2 = min(center.x + radius,640);
        //指定每个球的roi
        Mat roi = houghImage(Range(row1,row2),Range(col1,col2));
        identify(roi); //标出白球和目标球
    }
    imshow("分类", houghImage);

    //

    waitKey(0);
    return 0;
}

//输入彩色图，找出其中的圆和圆心并将其花在srcImage上
void HoughTrans(Mat &srcImage, Mat &grayImage, Mat &houghImage, vector<Vec3f> &circles) 
{
    vector<int> HoughParams = {20, 80, 12, 10, 20};//mindist,canny高阈值,累加器阈值,圆半径最小,最大
    //HoughCircles(InputArray image,OutputArray circles, int method, double dp,
    //             double minDist, double param1=100,double param2=100,
    //             int minRadius=0, int maxRadius=0 )
    //            输入      输出圆                累加器分辨率 mindist canny高阈值 累加器阈值  圆半径最小最大
    
    //霍夫圆变换HoughCircles——将灰度图中的每个圆的圆心和半径找到储存在circles中
    HoughCircles(grayImage, circles, CV_HOUGH_GRADIENT, 1, HoughParams.at(0), HoughParams.at(1), \
                 HoughParams.at(2), HoughParams.at(3), HoughParams.at(4));
    
    //依次在 houghImage 图中绘制出圆
    srcImage.copyTo(houghImage);
    for (size_t i = 0; i < circles.size(); i++) {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));   //circles的三个参数：x,y,radius
        int radius = cvRound(circles[i][2]);
        circle(houghImage, center, 3, Scalar(0, 255, 0), -1, 8, 0);    //绘制圆心
        circle(houghImage, center, radius, Scalar(155, 50, 255), 3, 8, 0);  //绘制圆轮廓
    }
    //显示效果图
    imshow("霍夫圆变换", houghImage);
}

//识别球的颜色，标出白球的质心位置
void identify(Mat &ball) {
    int i = 0, j = 0;
    float percent = 0;
    /// 白色的颜色范围
    vector<vector<int>> white_range = {{153, 255}, {138, 255}, {90, 255}};
    int n_white = 0, n_other = 0;  //计算白色像素个数，非白色个数
    /// 计算白色面积比例
    for (i = 0; i < ball.rows; ++i) {
        for (j = 0; j < ball.cols; ++j) {
            Vec3b pixel = ball.ptr<Vec3b>(i)[j];
            if (pixel[0] == 0 && pixel[1] == 0 && pixel[2] == 0)continue;
            if (pixel[0] >= white_range[0][0] && pixel[0] <= white_range[0][1]\
                && pixel[1] >= white_range[1][0] && pixel[0] <= white_range[1][1]\
                && pixel[2] >= white_range[2][0] && pixel[0] <= white_range[2][1])
                ++n_white;
            else ++n_other;
        }
    }
    percent = float(n_white) / (n_other + n_white);
    
    //将找到的白球的圆心标出
    if（percent < 0.5){
        cout << "This is a white ball." << endl;
        ball = Scalar(0,0,200);//四个参数是蓝、绿、红、透明度
    }
}

//将白球要运动到的目标击球点在图上标出
//初步思路——找出某直线上到该直线上某点距离为定值的点，然后将它在图像上标出
void hitingpoint (Mat &houghImage)
{
    //


}