#include <stdio.h>
#include <iostream>
#include <math.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/highgui/highgui.hpp>
#include "./driver/include/uvc_cam_sdk.h"
#include "./driver/include/bmp.h"
#include "./driver/src/hid_control/hid_control.hpp"

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


#define GAMMA_VALUE 2.2
uint8_t gamma_lut[256];
void generate_gamma_lut(uint8_t *lut,float gamma_value)
{

    for (int i = 0;i < 256;i++)
    {
        float in = float(i) / 255.f;
        lut[i] = (uint8_t)(pow(in,1.f/gamma_value)*255.f);
    }
}


void gamma_correction(uint8_t *source,int width,int height,uint8_t *lutPtr)
{
    for (int i = 0;i < height;i++)
    {
        for (int j = 0;j < width;j++)
        {
            source[i*width+j] = lutPtr[source[i*width+j]];
        }
    }
}

uint8_t buff[400*640*2];
// uint8_t buff1[400*640];
int main_rotation()
{
    printf("hello dispUvc\n");
    start_uvc_camera();
    uvc_camera_sdk_stream_start(1000*1000);
    uint64_t grab_times=0;
    while(1)
    {
         camera_t * uvc_frame=uvc_camera_sdk_stream_captured_once();
         printf("receive frame times %ld\n",grab_times++);
         
         uint8_t *start = uvc_frame->head.start; 
         for (int i = 0;i < 400;i++)
         {
            for (int j = 0;j < 640;j++)
            {
                buff[j*400+i] = start[i*640+j];
            }
            
         }
         uint8_t *start1 = uvc_frame->head.start + 640*400;
         
         uint8_t *buff1 = buff + 640*400;
         for (int i = 0;i < 400;i++)
         {
            for (int j = 0;j < 640;j++)
            {
                buff1[j*400+i] = start1[i*640+j];
            }
            
         }
    #if 0
         save_data_toFile("recordu8.raw",uvc_frame->head.start,640*800);
         save_data_toBMP("record_left.bmp",uvc_frame->head.start,640,400);
         save_data_toBMP("record_right.bmp",uvc_frame->head.start+640*400,640,400);

         save_data_toFile("recordu8_rotation.raw",buff,640*800);
         save_data_toBMP("record_left_rotation.bmp",buff,400,640);
         save_data_toBMP("record_right_rotation.bmp",buff+640*400,400,640);
    #endif
         cv::Mat image_cv_mat = cv::Mat(cv::Size(400,640), CV_8UC1, buff+400*640);
		// cvui::update();
		cv::imshow("UVC FRAMES", image_cv_mat);
        		if (cv::waitKey(20) == 27 ) {
			break;
		}
        
        usleep(WAIT_USLEEP_COFF);
    }
    
    uvc_camera_sdk_stream_stop();
}

int main_blur_estimate()
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
    uint8_t temp_buff[640*800];
    start_uvc_camera();
    uvc_camera_sdk_stream_start(1000*1000);
    uint64_t grab_times=0;
    while(1)
    {
        camera_t * uvc_frame=uvc_camera_sdk_stream_captured_once();
        printf("receive frame times %ld\n",grab_times++);
        
        uint8_t *start = uvc_frame->head.start; 
        memcpy(temp_buff,start,640*800);

        cv::Mat imageSource = cv::Mat(cv::Size(640,800), CV_8UC1, start);
        Mat imageOri = cv::Mat(cv::Size(640,800), CV_8UC1, temp_buff);
        

        // cvtColor(imageSource, imageGrey, CV_RGB2GRAY);
        generate_gamma_lut(gamma_lut,GAMMA_VALUE);

        gamma_correction(imageSource.data,imageSource.cols,imageSource.rows,gamma_lut);
        

        Mat imageSobel,imageSobelOri;

        Sobel(imageSource, imageSobel, CV_16U, 1, 1);
        Sobel(imageOri, imageSobelOri, CV_16U, 1, 1);
    
        //图像的平均灰度
        double meanValue = 0.0;
        double meanValueOri = 0.0;
        meanValue = mean(imageSobel)[0];
        meanValueOri = mean(imageSobelOri)[0];
    
        //double to string
        stringstream meanValueStream,meanValueStreamOri;
        string meanValueString,meanValueStringOri;
        meanValueStream << meanValue;
        meanValueStreamOri << meanValueOri;
        meanValueStream >> meanValueString;
        meanValueStreamOri >> meanValueStringOri;
        meanValueString = "Articulation(Sobel Method): " + meanValueString;
        meanValueStringOri = "ArticulationOri(Sobel Method): " + meanValueStringOri;
        putText(imageSource, meanValueString, Point(20, 50), 3, 0.8, Scalar(255, 255, 25), 2);
        imshow("Articulation", imageSource);
        putText(imageOri, meanValueStringOri, Point(20, 50), 3, 0.8, Scalar(255, 255, 25), 2);
        imshow("ArticulationOri", imageOri);

        if (cv::waitKey(20) == 27 ) {
			break;
		}
        

        usleep(WAIT_USLEEP_COFF);
    }
    
    uvc_camera_sdk_stream_stop();
}


int main()
{
    int ret = 0;

    ret = main_blur_estimate();

    return ret;
    
}