#include<cv.h>  
#include<highgui.h>  
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <zbar.h>
#include <iostream>
#include <iomanip>
  
#define Photo_Name "Y800"

#pragma comment(lib, "cv.lib")  
#pragma comment(lib, "cxcore.lib")  
#pragma comment(lib, "highgui.lib")  

using namespace std;
using namespace cv;
using namespace zbar;

class ImageCorner : public Mat
{
    private:
    Mat image;
    int width, height;//
    CvPoint2D32f imageTri[4];//表示图像的四个角点位置

    public:
    ImageCorner(): image(im), width(im.cols), height(im.rows) {}
    
    ImageCorner(Mat &im): image(im), width(im.cols), height(im.rows) {}

    ～ImageCorner(){}

    void convert_rec (ImageCorner &image_original)
    {
        ImageCorner &&imageSrc = image_original.clone();//将原图像拷贝进来
        ImageCorner imageGray;

        //调整原图像大小
        resize()

        //将原图转换为灰度图储存在gray中
        cvtColor(imageSrc, imageGray, COLOR_BGR2GRAY);
        imshow("灰度图", imageGray);

        resize
    }
}









class ImageConverter
{
    ros::NodeHandle nh;
    

    image_transport:: it;
    image_transport::Subscriber image_sub;
    image_transport::Publisher image_pub;

  public:
    ImageConverter():it(nh)
    {
        //使用image_transporImageTransportt订阅图像话题“in” 和 发布图像话题“out”
        image_sub=it.subscribe("/camera/rgb/image_raw",1,&ImageConverter::imageCb,this);
        image_pub=it.advertise("zbar_opencv",1);

    }

    ~ImageConverter(){}

    //订阅回调函数
    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            //将ROS图像消息转化为适合Opencv的CvImage
            cv_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
        }
        catch(cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s",e.what());
            return;
        }

        zbarscanner(cv_ptr);
        image_pub.publish(cv_ptr->toImageMsg());
    }
};






int main(int argc, char **argv) {

    ros::init(argc,argv,"convert-rectangle");
    ImageConverter ic;
    ros::spin();
    
    
    
    
    
    ros::shutdown;
    
    return 0;
}
