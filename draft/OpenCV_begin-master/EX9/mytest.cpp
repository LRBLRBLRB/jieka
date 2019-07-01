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
    srcImage = imread("../pics/ball3.png");
    if (!srcImage.data) {
        printf("读入文件错误！\n");
        return 1;
    }
    printf("按下q或者esc退出程序,by 刘琎\n");

    Mat roi = srcImage(Rect(100,100,3,3));
    cout<<format(roi,4)<<endl;

    cout<<int(roi.ptr<Vec3b>(1,1)[0][0])<<endl;/////////
    waitKey(0);
    return 0;
}

void identify(Mat &ball) {
    int i = 0, j = 0;

    /// 白色的颜色范围
    vector<vector<int>> white_range={{153,255},{138,255},{90,255}};
    int n_white=0,n_other=0;  //计算白色像素个数，非白色个数

    for(i=0;i<ball.rows;++i)
        for(j=0;j<ball.cols;++j)
        {
            Vec3b*  pixel  = ball.ptr<Vec3b>(i,j);
            //if(pixel[0] > white_range[0][1]);
        }

}

