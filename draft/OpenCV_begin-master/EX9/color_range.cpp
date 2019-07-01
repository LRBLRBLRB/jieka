/**
 * @brief: 颜色范围识别，给定一张图片，得到每个通道的最大最小值
 * @author： liujin
 * @date: 2019/5/11
*/
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/**头文件*/
using namespace cv;
using namespace std;

/**主函数*/
int main( )
{
    /// 载入原始图和Mat变量定义
    Mat srcImage = imread("../pics/白色采样.png");
    //Mat midImage,dstImage;//临时变量和目标图的定义

    /// 分离颜色通道
    vector<Mat> channels(3); //三颜色通道
    split(srcImage, channels);
    /// 3×2数组保存最大最小值
    vector<vector<double>> minmax(3,vector<double>(2));
    /// 找各个颜色通道最大最小范围
    for(int i=0;i<3;++i)
    {
        minMaxIdx(channels.at(i),&minmax[i][0],&minmax[i][1]);
        cout<<minmax[i][0]<<","<<minmax[i][1]<<endl;
    }

    waitKey(0);

    return 0;
}

