#include<cv.h>  
#include<highgui.h>  
  
#define Photo_Name "Y800"

#pragma comment(lib, "cv.lib")  
#pragma comment(lib, "cxcore.lib")  
#pragma comment(lib, "highgui.lib")  
  
int main()  
{  
    CvPoint2D32f srcTri[4], dstTri[4];  //position of the four points of a rectangle
    CvMat*       warp_mat = cvCreateMat (3, 3, CV_32FC1);  //CV _ 32SC2表示元素是32位有符号且有2个通道
    IplImage*    src = NULL;  //是一个CvMat对象，但它还有一些其他成员变量将矩阵解释为图像
    IplImage*    dst = NULL;  //目标对象
  
    src = cvLoadImage (Photo_Name, 1);  //读取彩色图
    dst = cvCloneImage (src);   //复制src的图像到dst
    dst->origin = src->origin;      //图像原点位置： 0表示顶-左结构,1表示底-左结构
    cvZero (dst);   //将其每个图像点设为0
  
    srcTri[0].x = 0;  
    srcTri[0].y = 0;  
    srcTri[1].x = src->width - 1;  
    srcTri[1].y = 0;  
    srcTri[2].x = 0;  
    srcTri[2].y = src->height - 1;  
    srcTri[3].x = src->width - 1;  
    srcTri[3].y = src->height - 1;  
  
    dstTri[0].x = src->width * 0.05;  
    dstTri[0].y = src->height * 0.33;  
    dstTri[1].x = src->width * 0.9;  
    dstTri[1].y = src->height * 0.25;  
    dstTri[2].x = src->width * 0.2;  
    dstTri[2].y = src->height * 0.7;  
    dstTri[3].x = src->width * 0.8;  
    dstTri[3].y = src->height * 0.9;  
  
    cvGetPerspectiveTransform (srcTri, dstTri, warp_mat);  
    cvWarpPerspective (src, dst, warp_mat);  
  
    cvNamedWindow("src", 1);  
    cvShowImage("src", src);  
    cvNamedWindow ("Affine_Transform", 1);  
    cvShowImage ("Affine_Transform", dst);  
  
    cvWaitKey (0);  
  
    cvReleaseImage (&src);  
    cvReleaseImage (&dst);  
    cvReleaseMat (&warp_mat);  
  
    return 0;  
}  

//原文：https://blog.csdn.net/qq_18343569/article/details/47953843
