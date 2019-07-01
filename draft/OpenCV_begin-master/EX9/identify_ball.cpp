/**
 * @brief: 区分花球和实心球
 * @author： liujin
 * @date: 2019/5/11
*/

/**头文件*/
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

using namespace cv;
using namespace std;

/**全局变量*/
Mat srcImage, grayImage, houghImage;
//各种参数
vector<int> HoughParams = {20, 80, 12, 10, 20};//mindist,canny高阈值,累加器阈值,圆半径最小,最大

/**函数声明*/
void identify(Mat &ball);

/**主程序*/
int main() {

    /// 读入源图片
    srcImage = imread("../pics/ball4.png");
    if (!srcImage.data) {
        printf("读入文件错误！\n");
        return 1;
    }
    printf("按下q或者esc退出程序,by 刘琎\n");

    /// 转为灰度图
    cvtColor(srcImage, grayImage, CV_BGR2GRAY);//转化边缘检测后的图为灰度图
    //gaussImage = grayImage.clone();

    /// 霍夫变换
    vector<Vec3f> circles;  //n*3维
    HoughCircles(grayImage, circles, CV_HOUGH_GRADIENT, 1, HoughParams.at(0), HoughParams.at(1), \
                 HoughParams.at(2), HoughParams.at(3), HoughParams.at(4));

    /// 绘制出找到的圆
    Mat ballMask;  //球的掩膜
    ballMask = Mat::zeros(srcImage.rows, srcImage.cols, CV_8UC1);
    for (size_t i = 0; i < circles.size(); ++i) {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1])); //circles的三个参数：x,y,radius
        int radius = cvRound(circles[i][2]);
        circle(ballMask, center, radius, Scalar(255), -1);
    }
    srcImage.copyTo(houghImage, ballMask);
    imshow("原图", houghImage);

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
        identify(roi); //分类球
    }
    imshow("分类", houghImage);
    /// 检测按键q或esc退出
    while (true) {
        int c = waitKey(0);
        //按下键盘按键Q或者ESC，程序退出
        if ((char) c == 'q' || (char) c == 27)
            break;
    }
    return 0;
}

void identify(Mat &ball) {
    int i = 0, j = 0;
    float percent = 0;
    /// 白色的颜色范围
    vector<vector<int>> white_range = {{153, 255},
                                       {138, 255},
                                       {90,  255}};
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
    /// 将找到的花球标红
    if (percent > 0.2 && percent<0.8){
        // cout << "这是花球" << endl;
        ball = Scalar(0,0,200);
    }
}

