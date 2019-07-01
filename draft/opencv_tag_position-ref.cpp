/*程序步骤： 
1.图片缩小 
2.灰度化，直方图均衡化，对比度增强，滤波 
3.otsu阈值分割 
4.五次膨胀 
5.轮廓查找，如果轮廓满足一下条件，认为可能为二维码区域，像素面积大于60，长短轴之比小于1.3 
6.对疑似区域做判断，因为本实验二维码贴在大概中心位置，找到疑似轮廓的质心，判断质心是不是在图片长宽的三分之一到三分之二之间，如果是认为是二维码区域 
7.对二维码区域画成蓝色 */

#include <stdio.h>  
#include <opencv/highgui.h>  
#include <time.h>  
#include <opencv2/opencv.hpp>  
#include <opencv/cv.h>  
#include <iostream> 
 
using namespace std;  
using namespace cv; 
 
int main(int argc,char *argv[])  
{
 
    Mat srcImage;   //自动管理内存？
    Mat resizeImage;
    Mat grayImage;
    Mat equalizeHistImage;
    Mat contrastandbrightImage;
    Mat sobelImage;
    Mat thresholdImage;
    Mat dilateImage;
    Mat areaImage;
    Mat ellipseImage;
    srcImage = imread("3.jpg");
    Size dsize = Size(srcImage.cols*0.3,srcImage.rows*0.3); //按比例放缩
    resize(srcImage, resizeImage,dsize);
    cvtColor(resizeImage,grayImage,CV_BGR2GRAY);
    equalizeHist(grayImage,equalizeHistImage);  //均衡直方图？
    ////亮度、对比度增强
    contrastandbrightImage= Mat::zeros( equalizeHistImage.size(), equalizeHistImage.type() ); 
    for(int y = 0; y < equalizeHistImage.rows; y++ )  
    {  
        for(int x = 0; x <equalizeHistImage.cols; x++ )  
        {  
            contrastandbrightImage.at<uchar>(y,x)= saturate_cast<uchar>(6*(equalizeHistImage.at<uchar>(y,x) ));     
        } 
    } 
    blur(contrastandbrightImage,contrastandbrightImage,Size(3,3));  
    //Sobel(contrastandbrightImage, sobelImage, CV_8U, 0,1 ,1, 1, 0, BORDER_DEFAULT);
    threshold(contrastandbrightImage,thresholdImage, 0, 255, CV_THRESH_OTSU+CV_THRESH_BINARY_INV);  
    Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));  
    //进行膨胀操作  
    dilate(thresholdImage, dilateImage, element);  
    dilate(dilateImage, dilateImage, element);
    dilate(dilateImage, dilateImage, element);  
    dilate(dilateImage, dilateImage, element);  
    dilate(dilateImage, dilateImage, element);  
 
    //dilateImage.copyTo(areaImage);
    //vector< vector< Point> > contours;  
    //findContours(areaImage,contours,CV_RETR_TREE,  CV_CHAIN_APPROX_NONE); 
    //vector<vector<Point> >::iterator itc= contours.begin();  
    //while (itc!=contours.end()) 
    //{  
    //  if( itc->size()<100)
    //  {  
    //      itc= contours.erase(itc);  
    //  }
    //   else
    //   {
    //     ++itc;  
    //  }
    //}  
    //drawContours(areaImage, contours, -1, Scalar(255), CV_FILLED);
 
    //找轮廓 两步判断  一个是大小  一个是长短轴比比  认为面积比60大，长短轴比比1.3小的区域是二维码区域  再做下一步判断
    dilateImage.copyTo(ellipseImage);
    vector<vector<Point> > twocontours;
    vector<Vec4i>twohierarchy;
    findContours(ellipseImage,twocontours,twohierarchy,CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE);  
    vector<Moments> mu(twocontours.size() );      
    vector<Point2f> mc( twocontours.size() );
    for(int k = 0; k < (int)twocontours.size(); k++)    //查找轮廓
    {
        if (int(twocontours.at(k).size()) <=60)//轮廓面积小于60的不处理
        {
            drawContours(ellipseImage, twocontours, k, Scalar(0), CV_FILLED);
        }
        else
        {
            RotatedRect rRect = fitEllipse(twocontours.at(k)); 
            double majorAxis = rRect.size.height > rRect.size.width ? rRect.size.height : rRect.size.width;
            double minorAxis = rRect.size.height > rRect.size.width ? rRect.size.width : rRect.size.height;
            float rate = majorAxis/minorAxis;
            if (rate<1.3)   //长短轴之比小于1.4的轮廓
            {
                //可能为二维码的区域，判断是否为二维码区域
                printf("%f\n",rate);
                //求区域的质心
                mu[k] = moments( twocontours[k], false );   
                mc[k] = Point2d( mu[k].m10/mu[k].m00 , mu[k].m01/mu[k].m00 ); 
                //打印质心
                printf("x=%f,y=%f\n",mc[k].x,mc[k].y);
                //打印图像的长宽
                printf("图像的长%d，图像的宽%d\n",resizeImage.cols,resizeImage.rows);
                //画出质心
                Point center = Point(mc[k].x,mc[k].y);  
                int r = 10;    
                circle(resizeImage,center,r,Scalar(255,0,0));  
                //判断质心是不是在图像中间   三分之一 < 质心 < 三分之二
                if ((int)mc[k].x<(int)2*(resizeImage.cols/3)&&(int)mc[k].x>(int)(resizeImage.cols/3))
                {
                    if ((int)mc[k].y<(int)2*(resizeImage.rows/3)&&(int)mc[k].y>(int)(resizeImage.rows/3))
                    {
                            drawContours(resizeImage, twocontours, k, Scalar(255,0,0), CV_FILLED);//把轮廓涂成蓝色
                            printf("蓝色是二维码区域\n");
                    }
                }
            }
            else  //长短轴之比大于1.4的轮廓  不是二维码区域
            {
                drawContours(resizeImage, twocontours, k, Scalar(0,0,255), CV_FILLED);//把轮廓涂成红色
            }
        }
    }       
    imshow("缩小图",resizeImage);
    imshow("灰度图",grayImage);
    imshow("对比度和亮度增强",contrastandbrightImage);
    //imshow("soble检测",sobelImage);
    imshow("二值化结果",thresholdImage);
    imshow("膨胀",dilateImage);
    //imshow("删除小面积",areaImage);
    imshow("长短轴",ellipseImage);
    waitKey(0);
}
 