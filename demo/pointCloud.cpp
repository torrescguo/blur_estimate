#include <stdio.h>
#include <iostream>
#include <math.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/highgui/highgui.hpp>
#include "../driver/include/uvc_cam_sdk.h"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include "../driver/include/bmp.h"
#include "../driver/src/hid_control/hid_control.hpp"
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <eigen3/Eigen/Core>
#include <pangolin/pangolin.h>
#include <unistd.h>

#include <opencv2/viz/types.hpp>
#include <opencv2/viz/viz3d.hpp>

#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <thread>
#include <semaphore.h>
// #include <pcl/visualization/pcl_visualizer.h>

// #include <pcl/visualization/cloud_viewer.h>  
// #include <iostream>  
// #include <pcl/io/io.h>  
// #include <pcl/io/pcd_io.h>  



using namespace std;
using namespace cv;
using namespace Eigen;

HidControl hid_control;
#ifdef MULTI_API_VERSION
bool g_is_new_fw = false;
#endif

#define CAPTURE_IMAGE_FPS (50)
#define WAIT_USLEEP_COFF  (1000*1000/50)

// #include <Eigen/Eigen>
// #include <pcl/point_cloud.h>
// #include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/point_types.h>

// #include <pcl/visualization/cloud_viewer.h>  
// #include <pcl/io/io.h>  
// #include <pcl/io/pcd_io.h>

static void start_uvc_camera()
{
    char path_f408_firstNode[50];
    udev_search_node(path_f408_firstNode,0);

    cout << "hhhhhhh" << path_f408_firstNode << endl;
    uvc_camera_sdk_init(path_f408_firstNode,640,800,1);
}

cv::Mat xyz;
cv::Mat stereo_depth;
static void convertGrayStereoToRGB(uint8_t * rgb_stereo,uint8_t * gray_l,uint8_t* gray_r,uint32_t w,uint32_t h)
{
    for(int i=0;i<h;i++)
    {
        uint8_t * gray_l_ptr = gray_l + i*w;
        uint8_t * gray_r_ptr = gray_r + i*w;
        uint8_t * rgb_l_ptr  = rgb_stereo + i * 2 *3 * w;
        uint8_t * rgb_r_ptr  = rgb_l_ptr  + 3 * w;
        for(int j=0;j<w;j++)
        {
            //left gray2rgb888
            rgb_l_ptr[j*3+0] = gray_l_ptr[j];
            rgb_l_ptr[j*3+1] = gray_l_ptr[j];
            rgb_l_ptr[j*3+2] = gray_l_ptr[j];
            //right gray2rgb888
            rgb_r_ptr[j*3+0] = gray_r_ptr[j];
            rgb_r_ptr[j*3+1] = gray_r_ptr[j];
            rgb_r_ptr[j*3+2] = gray_r_ptr[j];
        }
    }
}
static void onMouse(int event, int x, int y,int,void*)
{
    cv::Point origin;
    switch (event)
    {
        case cv::EVENT_LBUTTONDOWN:   //鼠标左按钮按下的事件
            origin = cv::Point(x, y);
            std::cout << "X : " << x << " Y : " << y << std::endl;
            // xyz.at<cv::Vec3f>(origin)[2] +=2;
            std::cout << origin << "Depth is: " << stereo_depth.at<uint16_t>(x,y)<< " MM "<<std::endl;
            break;
    }
}

// 在pangolin中画图，已写好，无需调整
// void showPointCloud(
//     const vector<Vector4d, Eigen::aligned_allocator<Vector4d>> &pointcloud);
// 生成点云
#define BUFF_SIZE 400
static vector<Vector4d, Eigen::aligned_allocator<Vector4d>> pointcloud[BUFF_SIZE];
static int read_idx = 0;
static int write_idx = 0;
pthread_t th_pointCloud;
sem_t sem_th_pointCloud;
// const vector<Vector4d, Eigen::aligned_allocator<Vector4d>>  *
/*
这段代码是一个C++函数，名为showpointCloud_th,它使用Pangolin库创建一个3D窗口并显示点云数据。以下是代码的逐行解释：

定义函数showpointCloud_th,参数为void*类型，表示没有实际用途的指针。

创建一个名为"Point Cloud Viewer"的Pangolin窗口，宽度为640像素，高度为400像素。

启用深度测试(GL_DEPTH_TEST)。

启用混合(GL_BLEND)。

设置混合函数为源颜色alpha与目标颜色alpha的差值。

创建一个OpenGl渲染状态(s_cam),包括投影矩阵和模型视图矩阵。

创建一个Pangolin显示对象(d_cam),并设置其边界、处理器和渲染状态。

进入一个无限循环，等待信号量(sem_th_pointCloud)通知可以绘制点云数据。

在每次循环中，首先清除颜色缓冲区和深度缓冲区。

激活渲染状态(s_cam)。

设置清除颜色为白色。

设置点的大小为3个单位。

开始绘制点云数据，遍历点云数组(pointcloud[read_idx])中的每个点，将其颜色设置为第三个通道的值，然后将其顶点坐标设置为对应的x、y、z坐标。

结束绘制点云数据。

调用pangolin::FinishFrame()完成当前帧的绘制。

更新读取索引(read_idx),使其在100以内循环。

由于没有实际用途的指针参数，函数返回时不会有任何操作
*/
void* showpointCloud_th(void*) 
{
    pangolin::CreateWindowAndBind("Point Cloud Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
        .SetHandler(new pangolin::Handler3D(s_cam));
    cout << "Init showpointCloud thread success!" << endl;
    while(1)
    {  
         sem_wait(&sem_th_pointCloud);
         if (pointcloud[read_idx].empty())
        {
            cerr << "Point cloud is empty!" << endl;
        
        }
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glPointSize(3);
        glBegin(GL_POINTS);
        for (auto &p: pointcloud[read_idx]) 
        {
            glColor3f(p[3], p[3], p[3]);
            glVertex3d(p[0], p[1], p[2]);
        }
        glEnd();
        pangolin::FinishFrame();
        
       
        read_idx = (++ read_idx) % BUFF_SIZE;

    }
}

static int main_blur_estimate()
{
#if 1
    std::string DISPLAY_HW_SW_VERSION = hid_control.get_software_version();
    float sw_v = hid_control.get_software_number();
    std::cout << "software version =" << sw_v  << std::endl;
    std::cout << " DISPLAY_HW_SW_VERSION = " << DISPLAY_HW_SW_VERSION << std::endl;
    #ifdef MULTI_API_VERSION
    hid_control.set_api_version(sw_v > 0.02);
    g_is_new_fw = sw_v > 0.02;
    #endif

    sleep(1);
    hid_control.set_gain_exposure(1,5);
#endif
    start_uvc_camera();
    uvc_camera_sdk_stream_start(1000*1000);
    uint64_t grab_times=0;

    int width = 640;
    int height = 400;
    double fx = 2.8238941925306216e+02, cx = 320, cy = 200; //相机内参
    double fy = fx;
	double b = 8.07146356e-01; //基线长度

    sem_init(&sem_th_pointCloud, 0, 0);
    pthread_create(&th_pointCloud,NULL,showpointCloud_th,NULL);
    while(1)
    {
        camera_t * uvc_frame=uvc_camera_sdk_stream_captured_once();
        printf("receive frame times %ld\n",grab_times++);
        
        uint8_t *start = uvc_frame->head.start; 
        // cv::Mat imageSource = cv::Mat(cv::Size(640,800), CV_8UC1, start);
        
        // uint8_t * rgb_stereo = (uint8_t*) malloc(width*height*2*3);
        // convertGrayStereoToRGB(rgb_stereo,start,start+width*height,width,height);

        // cv::Mat img_cv = cv::Mat(cv::Size(width*2,height), CV_8UC3,rgb_stereo );
        #if 1
        cv::Mat left  = cv::Mat(cv::Size(width,height), CV_8UC1,start );
        cv::Mat right  = cv::Mat(cv::Size(width,height), CV_8UC1,start+width*height );
        #else
        cv::Mat left = cv::imread("../data/pointcloud/left.png", 0);
        cv::Mat right = cv::imread("../data/pointcloud/right.png", 0);
        #endif
        
        cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(
            1, 96, 9, 8 * 9 * 9, 32 * 9 * 9, 1, 63, 10, 100, 32);    // 神奇的参数
        cv::Mat disparity_sgbm, disparity;
        sgbm->compute(left, right, disparity_sgbm);
        disparity_sgbm.convertTo(disparity, CV_32F, 1.0 / 16.0f);

        for (int v = 0; v < left.rows; v++)
        {
            for (int u = 0; u < left.cols; u++) {
                if (disparity.at<float>(v, u) <=1 || disparity.at<float>(v, u) >=96.0) continue;

                Vector4d point(0, 0, 0, left.at<uchar>(v, u) / 255.0); // 前三维为xyz,第四维为颜色

                // 根据双目模型计算 point 的位置
                double x = (u - cx) / fx;
                double y = (v - cy) / fy;
                double depth =fx*b / (disparity.at<float>(v, u));
                point[0] = x * depth;
                point[1] = y * depth;
                point[2] = depth;

                pointcloud[write_idx].push_back(point);
            }
        }
        sem_post(&sem_th_pointCloud);
        write_idx = (++write_idx) % BUFF_SIZE;
   
        cv::imshow("disparity", disparity/96);
            
        if (cv::waitKey(20) == 27 ) {
        break;
        }

        //  free(rgb_stereo);

    }
    
    uvc_camera_sdk_stream_stop();
}




int main()
{
    int ret = 0;

    ret = main_blur_estimate();

    return ret;
    
}