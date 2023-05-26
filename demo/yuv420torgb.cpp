#include <stdio.h>
#include <iostream>
#include <math.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/highgui/highgui.hpp>
#include "../driver/include/uvc_cam_sdk.h"
#include "../driver/include/bmp.h"
#include "../driver/src/hid_control/hid_control.hpp"

using namespace std;
using namespace cv;
HidControl hid_control;
#ifdef MULTI_API_VERSION
bool g_is_new_fw = false;
#endif

#define CAPTURE_IMAGE_FPS (50)
#define WAIT_USLEEP_COFF  (1000*1000/50)
void start_uvc_camera()
{
    char path_f408_firstNode[50];
    udev_search_node(path_f408_firstNode,0);

    cout << "hhhhhhh" << path_f408_firstNode << endl;
    uvc_camera_sdk_init(path_f408_firstNode,640,800,1);
}
void save_data_toBMP(const char * path,uint8_t * mem,uint32_t width,uint32_t height)
{
    uint8_t *rgb_data = (uint8_t*)malloc(width*height*3);
    for(uint32_t i = 0;i<height;i++)
    {
        for(uint32_t j = 0;j<width;j++)
        {
            rgb_data[i*width*3+j*3+0] = mem[i*width+j];
            rgb_data[i*width*3+j*3+1] = mem[i*width+j];
            rgb_data[i*width*3+j*3+2] = mem[i*width+j];
        }
    }
    save_bgr_bmp(path,rgb_data,width,height);
    free(rgb_data);
}

void save_data_toFile(const char * path,uint8_t * mem,uint32_t bytes)
{
    FILE * fp = fopen(path,"wb");
    fwrite(mem,sizeof(uint8_t),bytes,fp);
    fclose(fp);
}
void read_data_toFile(char * path,uint8_t * mem,uint32_t bytes)
{
    FILE * fp = fopen(path,"rb");
    fread(mem,sizeof(uint8_t),bytes,fp);
    fclose(fp);
}

void YUV420toRGB888(uint8_t *yuv420, uint8_t*rgb ,int width, int height) {
    int Ylen = width * height;
    uint8_t *p_Y420 = yuv420;
    uint8_t *p_U420 = p_Y420 + Ylen;
    uint8_t *p_V420 = p_U420 + Ylen / 4;

 
    int rgbIndex = 0;
    int R,G,B,Y,U,V;
    for(int y = 0; y < height; y++) {
        for(int x = 0; x < width; x++){
            Y = *(p_Y420 + y * width + x);
            U = *(p_U420 + (y / 2) * (width / 2) + x / 2);
            V = *(p_V420 + (y / 2) * (width / 2) + x / 2);

            R = Y + 1.402 * (V -128);
            G = Y - 0.34414 * (U - 128) - 0.71414 * (V - 128);
            B = Y + 1.772 * (U - 128);
 
            R = R < 255 ? R : 255;
            G = G < 255 ? G : 255;
            B = B < 255 ? B : 255;
 
            R = R < 0 ? 0 : R;
            G = G < 0 ? 0 : G;
            B = B < 0 ? 0 : B;
 
            *(rgb + rgbIndex++) = R;
            *(rgb + rgbIndex++) = G;
            *(rgb + rgbIndex++) = B;
        }
    }
}


int main()
{
    printf("main_blur_estimate\n");
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
    
    // start_uvc_camera();
    // uvc_camera_sdk_stream_start(1000*1000);

    uint8_t yuv420_buff[640*360+640*180];
    uint8_t rgb_buff[640*360*3];
    
   
    // while(1)
    {
    
        // read rgb image 640*360
        cv::Mat imageSource = imread("../data/rgb_test.png",1);//cv::Mat(cv::Size(640,800), CV_8UC1, start);
        cout << "Rgb    Image ------" << "width :" << imageSource.cols << "height :" << imageSource.rows << endl;
        
        // rgb to yuv420:Y + UV 640*360+640*180 
        Mat yuv420;

        cvtColor(imageSource, yuv420, CV_RGB2YUV_I420);
        
        uint32_t bytes = yuv420.cols * yuv420.rows;
        // generate yuv420 image
        save_data_toFile("../data/yuv420_test.raw",yuv420.data,bytes);
        cout << "Yuv420 Image ------" << "width :" << yuv420.cols << "height :" << yuv420.rows << endl;
        
        sleep(2);
        // yuv420 to RGB algo
        read_data_toFile("../data/yuv420_test.raw",yuv420_buff,640*540);
        YUV420toRGB888(yuv420_buff,rgb_buff,640,360);

        uint8_t rgb_r[640*360];
        uint8_t rgb_g[640*360];
        uint8_t rgb_b[640*360];

        for (int i = 0;i < 640*360;i++)
        {
            rgb_r[i] = rgb_buff[i*3];
            rgb_g[i] = rgb_buff[i*3] + 1;
            rgb_b[i] = rgb_buff[i*3] + 2;
        }
        Mat rgb_fram = Mat(Size(640,360),CV_8UC3,rgb_buff);

        Mat rgb_fram_r = Mat(Size(640,360),CV_8UC1,rgb_r);

        Mat rgb_fram_g = Mat(Size(640,360),CV_8UC1,rgb_g);

        Mat rgb_fram_b = Mat(Size(640,360),CV_8UC1,rgb_b);
        
        imshow("rgb", imageSource);
        imshow ("yuv420",yuv420);
        imshow("algo_rgb",rgb_fram);
        imshow("algo_r",rgb_fram_r);
        imshow("algo_g",rgb_fram_g);
        imshow("algo_b",rgb_fram_b);
        

        waitKey(0);
        // if (cv::waitKey(20) == 27 ) {
		// 	break;
		// }
        

        usleep(WAIT_USLEEP_COFF);
    }
    
    // uvc_camera_sdk_stream_stop();

    return 0;
}

