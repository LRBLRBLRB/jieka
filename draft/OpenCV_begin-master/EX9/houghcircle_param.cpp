/**
 * @brief: 霍夫变换检测圆参数调整
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
Mat srcImage, grayImage, gaussImage, dstImage,houghImage;
//各种参数
vector<int> GaussParams = {0, 1, 3};  //依次是（是否使用高斯滤波，算子半径，xy标准差）
vector<int> CannyParams = {100, 50, 0}; //阈值1，阈值2，sobel算子大小(0代表3,1代表5,2代表7)
vector<int> HoughParams = {20, 80, 12, 10, 20};//mindist,canny高阈值,累加器阈值,圆半径最小,最大

/**函数声明*/
static void on_Canny(int, void *);

static void on_Gauss(int, void *);

static void on_Hough(int, void *);

void HoughTrans();

//颜色过滤函数640*360
void color_filter(Mat &src, Mat &dst, vector<vector<double>> minmax) {
    int i = 0, j = 0;

    dst.create(src.rows, src.cols, src.type());

    //拆分成三通道
    vector<Mat> channels(3);
    split(src, channels);

    //颜色掩码构建
    for (i = 0; i < 3; ++i)
        channels.at(i) = ((channels.at(i) < minmax[i].at(0)) | (channels.at(i) > minmax[i].at(1)));
    Mat mask = channels.at(0) | channels.at(1) | channels.at(2);

    src.copyTo(dst, mask);
}

/**主程序*/
int main() {
    //读入源图片
    srcImage = imread("../pics/ball1.png");
    if (!srcImage.data) {
        printf("读入文件错误！\n");
        return 1;
    }
    printf("按下q或者esc退出程序,by 刘琎\n");

    // 转为灰度图
    cvtColor(srcImage, grayImage, CV_BGR2GRAY);//转化边缘检测后的图为灰度图
    gaussImage = grayImage.clone();
    //imshow("灰度图",midImage);

    //高斯滤波参数调整
    //vector<int> GaussParams = {0, 3};//依次是（算子半径(零代表没用滤波)，xy标准差）
    namedWindow("高斯滤波参数调整");
    createTrackbar("是否启用", "高斯滤波参数调整", &GaussParams[0], 10, on_Gauss, (void *) &GaussParams);
    createTrackbar("算子半径", "高斯滤波参数调整", &GaussParams[1], 10, on_Gauss, (void *) &GaussParams); //因为要是奇数,所以×2+1
    createTrackbar("标 准 差", "高斯滤波参数调整", &GaussParams[2], 10, on_Gauss, (void *) &GaussParams);
    //GaussianBlur( midImage, midImage, Size(0, 0),3, 3 );
    // imshow("高斯滤波后",gaussImage);

    //边缘检测参数调整
    namedWindow("边缘检测参数调整");
    createTrackbar("阈值1", "边缘检测参数调整", &CannyParams[0], 255, on_Canny, (void *) &CannyParams);
    createTrackbar("阈值2", "边缘检测参数调整", &CannyParams[1], 255, on_Canny, (void *) &CannyParams);
    createTrackbar("Sobel算子半径", "边缘检测参数调整", &CannyParams[2], 2, on_Canny, (void *) &CannyParams);

    //霍夫变换参数调整
    namedWindow("霍夫变换参数调整");//mindist,canny高阈值,累加器阈值,圆半径最小,最大{10,80,25,10,25}
    createTrackbar("圆间最短距离", "霍夫变换参数调整", &HoughParams[0], 50, on_Hough, (void *) &HoughParams);
    createTrackbar("canny高阈值", "霍夫变换参数调整", &HoughParams[1], 255, on_Hough, (void *) &HoughParams);
    createTrackbar("累加器的阈值", "霍夫变换参数调整", &HoughParams[2], 100, on_Hough, (void *) &HoughParams);
    createTrackbar("圆半径最小值", "霍夫变换参数调整", &HoughParams[3], 50, on_Hough, (void *) &HoughParams);
    createTrackbar("圆半径最大值", "霍夫变换参数调整", &HoughParams[4], 50, on_Hough, (void *) &HoughParams);

    //初始化图像
    imshow("高斯滤波参数调整", gaussImage);//
    Canny(gaussImage, dstImage, 100, 50, 3);
    imshow("边缘检测参数调整", dstImage);
    HoughTrans();

    while(true)
    {
        int c = waitKey(0);
        //按下键盘按键Q或者ESC，程序退出
        if( (char)c == 'q'||(char)c == 27 )
            break;
    }
    //Mat kernel = (Mat_<char>(3, 3) << 0, -1, 0, -1, 5, -1, 0, -1, 0);
    return 0;
}

static void on_Gauss(int, void *params) {
    vector<int> *GaussParams = static_cast<vector<int> *>(params);

    if (GaussParams->at(0)) {  //如果启用高斯滤波则刷新图像
        int size = 2 * GaussParams->at(1) + 1;
        int sigma = GaussParams->at(2);
        GaussianBlur(grayImage, gaussImage, Size(size, size), sigma, sigma);
        imshow("高斯滤波参数调整", gaussImage);
        //刷新边缘检测图片
        on_Canny(CannyParams[0], (void *) &CannyParams);
        on_Hough(HoughParams[0], (void *) &HoughParams);
    } else {
        gaussImage = grayImage.clone();
        imshow("高斯滤波参数调整", gaussImage);
        //刷新边缘检测图片
        on_Canny(CannyParams[0], (void *) &CannyParams);
        on_Hough(HoughParams[0], (void *) &HoughParams);
    }


}

static void on_Canny(int, void *params) {
    vector<int> *CannyParams = static_cast<vector<int> *>(params);
    Canny(gaussImage, dstImage, CannyParams->at(0), CannyParams->at(1), CannyParams->at(2) * 2 + 3);
    imshow("边缘检测参数调整", dstImage);
}

static void on_Hough(int, void *params) {
    vector<int> *HoughParams = static_cast<vector<int> *>(params);

    //做必要的参数检查
    if(HoughParams->at(0)==0)HoughParams->at(0)=1;
    //进行霍夫圆变换
    vector<Vec3f> circles;
    HoughCircles(gaussImage, circles, CV_HOUGH_GRADIENT, 1, HoughParams->at(0), HoughParams->at(1), \
                 HoughParams->at(2), HoughParams->at(3), HoughParams->at(4));
    //依次在 houghImage 图中绘制出圆
    srcImage.copyTo(houghImage);
    for (size_t i = 0; i < circles.size(); i++) {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1])); //circles的三个参数：x,y,radius
        int radius = cvRound(circles[i][2]);
        //绘制圆心
        circle(houghImage, center, 3, Scalar(0, 255, 0), -1, 8, 0);  //
        //绘制圆轮廓
        circle(houghImage, center, radius, Scalar(155, 50, 255), 3, 8, 0);
    }
    //显示效果图
    imshow("霍夫变换参数调整", houghImage);
}

void HoughTrans() {
    //进行霍夫圆变换
    vector<Vec3f> circles;
    //HoughCircles(InputArray image,OutputArray circles, int method, double dp,
    //             double minDist, double param1=100,double param2=100,
    //             int minRadius=0, int maxRadius=0 )
    //            输入      输出圆                累加器分辨率 mindist canny高阈值 累加器阈值  圆半径最小最大
    HoughCircles(gaussImage, circles, CV_HOUGH_GRADIENT, 1, HoughParams.at(0), HoughParams.at(1), \
                 HoughParams.at(2), HoughParams.at(3), HoughParams.at(4));
    //依次在 houghImage 图中绘制出圆
    srcImage.copyTo(houghImage);
    for (size_t i = 0; i < circles.size(); i++) {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1])); //circles的三个参数：x,y,radius
        int radius = cvRound(circles[i][2]);
        //绘制圆心
        circle(houghImage, center, 3, Scalar(0, 255, 0), -1, 8, 0);  //
        //绘制圆轮廓
        circle(houghImage, center, radius, Scalar(155, 50, 255), 3, 8, 0);
    }
    //显示效果图
    imshow("霍夫变换参数调整", houghImage);
}

