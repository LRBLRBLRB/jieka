/**
 * @brief:形态学运算
 * @author：liuyvjin
 */
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;


Mat g_srcImage, g_dstImage;//原始图和效果图
int g_ElementShape = MORPH_RECT;//元素结构的形状

//变量接收的TrackBar位置参数
int g_MaxIterationNum = 20;
int g_OpenCloseNum = 0;  //开闭
int g_ErodeDilateNum = 0;  //腐蚀、膨胀
int g_TopBlackHatNum = 0;  //顶帽、黑帽
int g_GradientNum = 0;  //形态学梯度


//-----------------------------------【全局函数声明部分】--------------------------------
//		描述：全局函数声明
//------------------------------------------------------------------------------------
static void on_OpenClose(int, void*);//回调函数
static void on_ErodeDilate(int, void*);//回调函数
static void on_TopBlackHat(int, void*);//回调函数
static void on_Gradient(int, void*);//回调函数
static void ShowHelpText();//帮助文字显示


//-----------------------------------【main( )函数】------------------------------------
//		描述：控制台应用程序的入口函数，我们的程序从这里开始
//-------------------------------------------------------------------------------------
int main( )
{
    ShowHelpText();

    //载入原图
    g_srcImage = imread("../../pictures/皮.png");
    if(!g_srcImage.data ) { printf("Oh，no，读取srcImage错误~！ \n"); return false; }

    //显示原始图
    namedWindow("【原始图】");
    imshow("【原始图】", g_srcImage);

    //创建三个窗口
    namedWindow("【开运算/闭运算】",1);
    namedWindow("【腐蚀/膨胀】",1);
    namedWindow("【顶帽/黑帽】",1);
    namedWindow("【形态学梯度】",1);
    //参数赋值
    g_OpenCloseNum=9;
    g_ErodeDilateNum=9;
    g_TopBlackHatNum=9;
    g_GradientNum=0;
    //分别为三个窗口创建滚动条
    createTrackbar("迭代值", "【开运算/闭运算】",&g_OpenCloseNum,g_MaxIterationNum*2+1,on_OpenClose);
    createTrackbar("迭代值", "【腐蚀/膨胀】",&g_ErodeDilateNum,g_MaxIterationNum*2+1,on_ErodeDilate);
    createTrackbar("迭代值", "【顶帽/黑帽】",&g_TopBlackHatNum,g_MaxIterationNum*2+1,on_TopBlackHat);
    createTrackbar("结构体大小","【形态学梯度】",&g_GradientNum,g_MaxIterationNum,on_Gradient);
    
    //轮询获取按键信息
    while(1)
    {
        int c;

        //执行回调函数
        on_OpenClose(g_OpenCloseNum, 0);
        on_ErodeDilate(g_ErodeDilateNum, 0);
        on_TopBlackHat(g_TopBlackHatNum,0);
        on_Gradient(g_GradientNum,0);
        //获取按键
        c = waitKey(0);

        //按下键盘按键Q或者ESC，程序退出
        if( (char)c == 'q'||(char)c == 27 )
            break;
        //按下键盘按键1，使用椭圆(Elliptic)结构元素结构元素MORPH_ELLIPSE
        if( (char)c == 49 )//键盘按键1的ASII码为49
            g_ElementShape = MORPH_ELLIPSE;
            //按下键盘按键2，使用矩形(Rectangle)结构元素MORPH_RECT
        else if( (char)c == 50 )//键盘按键2的ASII码为50
            g_ElementShape = MORPH_RECT;
            //按下键盘按键3，使用十字形(Cross-shaped)结构元素MORPH_CROSS
        else if( (char)c == 51 )//键盘按键3的ASII码为51
            g_ElementShape = MORPH_CROSS;
            //按下键盘按键space，在矩形、椭圆、十字形结构元素中循环
        else if( (char)c == ' ' )
            g_ElementShape = (g_ElementShape + 1) % 3;
    }

    return 0;
}


//-----------------------------------【on_OpenClose( )函数】----------------------------------
//		描述：【开运算/闭运算】窗口的回调函数
//-----------------------------------------------------------------------------------------------
static void on_OpenClose(int, void*)
{
    //偏移量的定义
    int offset = g_OpenCloseNum - g_MaxIterationNum;//偏移量
    int Absolute_offset = offset > 0 ? offset : -offset;//偏移量绝对值
    //自定义核
    Mat element = getStructuringElement(g_ElementShape, Size(Absolute_offset*2+1,\
    Absolute_offset*2+1), Point(Absolute_offset, Absolute_offset) );
    //进行操作
    if( offset < 0 )
        morphologyEx(g_srcImage, g_dstImage, CV_MOP_OPEN, element);
    else
        morphologyEx(g_srcImage, g_dstImage, CV_MOP_CLOSE, element);
    //显示图像
    imshow("【开运算/闭运算】",g_dstImage);
}


//-----------------------------------【on_ErodeDilate( )函数】----------------------------------
//		描述：【腐蚀/膨胀】窗口的回调函数
//-----------------------------------------------------------------------------------------------
static void on_ErodeDilate(int, void*)
{
    //偏移量的定义
    int offset = g_ErodeDilateNum - g_MaxIterationNum;	//偏移量
    int Absolute_offset = offset > 0 ? offset : -offset;//偏移量绝对值
    //自定义核
    Mat element = getStructuringElement(g_ElementShape, Size(Absolute_offset*2+1, Absolute_offset*2+1), Point(Absolute_offset, Absolute_offset) );
    //进行操作
    if( offset < 0 )
        erode(g_srcImage, g_dstImage, element);
    else
        dilate(g_srcImage, g_dstImage, element);
    //显示图像
    imshow("【腐蚀/膨胀】",g_dstImage);
}


//-----------------------------------【on_TopBlackHat( )函数】--------------------------------
//		描述：【顶帽运算/黑帽运算】窗口的回调函数
//----------------------------------------------------------------------------------------------
static void on_TopBlackHat(int, void*)
{
    //偏移量的定义
    int offset = g_TopBlackHatNum - g_MaxIterationNum;//偏移量
    int Absolute_offset = offset > 0 ? offset : -offset;//偏移量绝对值
    //自定义核
    Mat element = getStructuringElement(g_ElementShape, Size(Absolute_offset*2+1, Absolute_offset*2+1), Point(Absolute_offset, Absolute_offset) );
    //进行操作
    if( offset < 0 )
        morphologyEx(g_srcImage, g_dstImage, MORPH_TOPHAT , element);
    else
        morphologyEx(g_srcImage, g_dstImage, MORPH_BLACKHAT, element);
    //显示图像
    imshow("【顶帽/黑帽】",g_dstImage);
}
//-----------------------------------【on_TopBlackHat( )函数】--------------------------------
//		描述：【形态学梯度】窗口的回调函数
//----------------------------------------------------------------------------------------------
static void on_Gradient(int, void*)
{
    //自定义核
    Mat element = getStructuringElement(g_ElementShape, Size(g_GradientNum*2+1, g_GradientNum*2+1),\
    Point(g_GradientNum,g_GradientNum) );
    //进行操作
    morphologyEx(g_srcImage, g_dstImage, MORPH_GRADIENT , element);
    //显示图像
    imshow("【形态学梯度】",g_dstImage);
}
//-----------------------------------【ShowHelpText( )函数】----------------------------------
//		描述：输出一些帮助信息
//----------------------------------------------------------------------------------------------
static void ShowHelpText()
{
//输出一些帮助信息
    printf("\n\n\n\t请调整滚动条观察图像效果~\n\n");
    printf( "\n\n\t按键操作说明: \n\n"
            "\t\t键盘按键【ESC】或者【Q】- 退出程序\n"
            "\t\t键盘按键【1】- 使用椭圆(Elliptic)结构元素\n"
            "\t\t键盘按键【2】- 使用矩形(Rectangle )结构元素\n"
            "\t\t键盘按键【3】- 使用十字型(Cross-shaped)结构元素\n"
            "\t\t键盘按键【空格SPACE】- 在矩形、椭圆、十字形结构元素中循环\n"
    );
}
