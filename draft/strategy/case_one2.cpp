//这是用于选拔赛第一轮的台球击打程序，无障碍球，但是存在击打顺序与极大地可能性问题,不指定击打顺序

#include <iostream>
#include <iomanip>
#include <vector>
#include <stdio.h>
#include <math.h>
#include"algorithm"
#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/imgproc/imgproc_c.h"
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

//注意修改：define项、六个球洞的像素值、图片名称,霍夫圆变换中的阈值设定（使得circles中只有球）
#define length 229              //台球桌内长度，单位cm
#define width 111               //台球桌内宽度，单位cm
#define threshAngle 60          //可以击打球的最大角度，取值范围[0, 90)
#define terminalDistance 20     //末端到白球的距离，单位是像素？？

using namespace std;
using namespace cv;

//定义台球桌六个球洞的像素点位置为静态全局变量
Point hole[6];
void HoughTrans(Mat &srcImage, Mat &grayImage, Mat &houghImage, vector<Vec4f> &circles);
int identify(Mat &ball, vector<Vec4f> &circles, int &whiteBall, int &dstBall, int num);
int hittingpoint (Mat &houghImage, vector<Vec4f> &circles, int &whiteBall, int &dstBall, Point &dstAimingPoint);
double isAngle (Mat &houghImage, vector<Vec4f> &circles, int &whiteBall, Point &dstAim, Point hole);
//记得函数声明！！！！！！！



int main (int argc, char **argv)
{
    //ros::init(argc, argv, "case_one1");
    //ros::NodeHandle nh;
    
    //给台球桌球洞赋值，调试当天拍照然后手动标定？？
    hole[0].x = 0, hole[0].y = 0;
    hole[1].x = 0, hole[1].y = 0;
    hole[2].x = 0, hole[2].y = 0;
    hole[3].x = 0, hole[3].y = 0;
    hole[4].x = 0, hole[4].y = 0;
    hole[5].x = 0, hole[5].y = 0;

    //定义原图、灰度图、最终图的Mat对象
    Mat srcImage, grayImage, houghImage, dstImage;

    /// 读入源图片
    srcImage = imread("ball3.png");
    imshow("src",srcImage);
    waitKey(0);
    if (!srcImage.data) {
        printf("读入文件错误！\n");
        return 1;
    }
    resize(srcImage,srcImage,Size(640,360));
    printf("按下q或者esc退出程序\n");

    //转化边缘检测后的图为灰度图
    cvtColor(srcImage, grayImage, CV_BGR2GRAY);
    cout<<"1111"<<endl;
    /// 保存找到的圆
    //Vec4f每一组的三个参数是圆心坐标x,y，半径和性质
    //0表示白球，1表示目标球，2表示障碍球
    vector<Vec4f> circles;
    //分别储存白球和目标球在容器中的位置
    int whiteBall(-1), dstBall(-1);
    Point dstAimingPoint, terminal; //dstAimingPoint表示最佳瞄准点, terminal表示最佳瞄准点下的末端停留点

    /// 霍夫变换，并绘图
    HoughTrans(srcImage, grayImage, houghImage, circles);

    /// 分类球
    int row1, row2, col1, col2;
    for (size_t i = 0; i < circles.size(); ++i) {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1])); //circles的三个参数：x,y,radius
        int radius = cvRound(circles[i][2]);

        row1 = max(center.y - radius,0);
        row2 = min(center.y + radius,360);
        col1 = max(center.x - radius,0);
        col2 = min(center.x + radius,640);
        //指定每个球的roi
        Mat roi = houghImage(Range(row1,row2),Range(col1,col2));
        circles[i][3] = identify(roi, circles, whiteBall, dstBall, i); //标出白球和目标球
       
    }
    int time;
    cin>>time;

    //调用hittingPoint函数计算出六个球洞中哪一个球洞最好打，返回的是最好打的球洞编号
    //dstHoleNumber表示的是最佳进球洞。如果是【1-6】的整数，则说明将要将球打进该洞；如果取0，则说明需要还原白球到还原点
    int dstHoleNumber = hittingpoint(houghImage, circles, whiteBall, dstBall, dstAimingPoint);
    if(!dstHoleNumber){//这是要还原的情况
        cout << "We cannot hit the ball into any hole, please return the white ball to the original position." << endl;
        cout << "Press y to continue hitting." << endl;
        char isok = '\0';
        cin >> isok;
        while(isok != 'y') {
            cout << "Invalid input, please return the ball and press y again." << endl;
            cin >> isok;
        }
        cout << "Successfully return the white ball." << endl;
        return 0;//这里怎么才能重新调用？？或者用其他程序提示要还原白球？？？
    }
    else {//这是要打球的情况
        //根据dstAimingPoint和circles[whitBall]确定末端停留的位置，算法与求目标瞄准点位置的一样
        terminal.x = circles[whiteBall][0] + (terminalDistance / sqrtf(powf(circles[whiteBall][0] - dstAimingPoint.x, 2) 
                    + powf(circles[whiteBall][0] - dstAimingPoint.x, 2))) * (circles[whiteBall][0] - dstAimingPoint.x);
        terminal.y = circles[whiteBall][1] + (terminalDistance / sqrtf(powf(circles[whiteBall][0] - dstAimingPoint.x, 2) 
                    + powf(circles[whiteBall][0] - dstAimingPoint.x, 2))) * (circles[whiteBall][1] - dstAimingPoint.y);
        //waitKey(0);
    return 0;
    }
}

//输入彩色图，找出其中的圆和圆心并将其花在srcImage上
void HoughTrans(Mat &srcImage, Mat &grayImage, Mat &houghImage, vector<Vec4f> &circles) 
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
    imshow("Hough", houghImage);
    waitKey(0);
}

//识别球的颜色，标出白球的质心位置
int identify(Mat &ball, vector<Vec4f> &circles, int &whiteBall, int &dstBall, int num) 
{
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
    if(percent > 0.5){
        if(whiteBall == -1){
            whiteBall = num;
            cout << "检测到一个白球" << endl;
            //ball = Scalar(0,0,200);//四个参数是蓝、绿、红、透明度
            return 0;//返回值表明这是个白球
        }
        else {cout << "球分类错误：检测出两个白球" << endl; return -1;}
    }
    else {
        if (dstBall == -1){
            dstBall = num;
            cout << "This is a red ball." << endl;
            //ball = Scalar(0,0,0);
            return 1;//返回值表明这是一个红球（目标球）
        }
        else {cout << "球分类错误：检测出两个目标球" << endl; return -1;}
    }
}

//将白球要运动到的目标击球点在图上标出
//初步思路——半球法；找出某直线上到该直线上某点距离为定值的点，然后将它在图像上标出
int hittingpoint (Mat &houghImage, vector<Vec4f> &circles, int &whiteBall, int &dstBall, Point &dstAimingPoint)
{
    for(int i = 0; i < circles.size(); i++){
        if(circles[i][3] == 1){//说明是红球，只有一个红球，说明是目标球
            if(dstBall == -1) dstBall = i;
            else cout << "Classification warning: More cthan one red ball is classified." << endl;
        }
    }
    double distance = 2 * circles[dstBall][2]; //白球瞄准点到目标球心的距离
    
    //遍历六个球洞，求出最佳进球洞
    double angle[6];    //用来储存每个球洞对应的角度
    Point aimingPoint[6];   //储存要将球打进每个球洞，需要瞄准的点；dstAimingPoint储存最佳进球洞的瞄准点
    
    //将要打进每个球洞所需要的瞄准点和击球角度算出来
    for (int i = 0; i < 6; i++){
        //利用相似原理，求出目标瞄准点的位置
        aimingPoint[i].x = circles[dstBall][0] + (distance / sqrtf(powf(circles[dstBall][0] - hole[i].x, 2) 
                + powf(circles[dstBall][1] - hole[i].y, 2))) * (circles[dstBall][0] - hole[i].x);
        aimingPoint[i].y = circles[dstBall][1] + (distance / sqrtf(powf(circles[dstBall][0] - hole[i].x, 2) 
                + powf(circles[dstBall][1] - hole[i].y, 2))) * (circles[dstBall][1] - hole[i].y);
        //求出对应的aimingPoint[i]后，用isAngle判断该球洞是否有足够的击打角度
        angle[i] = isAngle(houghImage, circles, whiteBall, dstAimingPoint, hole[i]);
    }

    /*划线void line(Mat& img, Point pt1, Point pt2, const Scalar& color, int thickness=1, int lineType=8, 
                    int shift=0)——lineType: 线段的类型。可以取值8， 4， 和CV_AA， 分别代表8邻接连接线，4邻接连接线
                    和反锯齿连接线。默认值为8邻接。为了获得更好地效果可以选用CV_AA(采用了高斯滤波)。*/
    //Point dstWhite(circles[dstBall][0], circles[dstBall][1]);
    //line(houghImage, dstAimingPoint, dstWhite, Scalar(0, 255, 0));

    //选择最优击球球洞
    double m=180.0;
    int n = 0;
    for (int i = 0; i < 6; i++){
        if (fabs(angle[i]) < m) {m = fabs(angle[i]); n = i+1;}
    }
    if (m > threshAngle) {
        cout << "当前位置无法击打球，请将白球还原" << endl;
        return n;
    } 
    else{
        cout << "当前最佳进球洞为" << n << "号球洞" << endl;
        dstAimingPoint = aimingPoint[n];
        return n;
    }
}

//判断击球角度是否足够
//hole是函数hittingpoint发出的要检测的球洞
double isAngle (Mat &houghImage, vector<Vec4f> &circles, int &whiteBall, Point &aimingPoint, Point hole)
{
    //检测是否有足够的击球角度
    //只需要求出白球运动轨迹线与目标球运动轨迹线的夹角即可;利用向量夹角的定义计算
    double dx1 = aimingPoint.x - hole.x;
    double dy1 = aimingPoint.y - hole.y;
    double dx2 = circles[whiteBall][0] - aimingPoint.x;
    double dy2 = circles[whiteBall][1] - aimingPoint.y;

    double angle = acos((dx1 * dx2 + dy1 * dy2)/sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2)));
    //如果夹角在[-threshAngle,threshAngle]内，则可以击打该球，输出该球角度；否则不能击打该球，输出180
    if(angle <= (double)threshAngle && angle >= (double)(-1 * threshAngle)) return angle;
    else return 180;
}