#include "opencv2/opencv.hpp"
#include <opencv2/xfeatures2d.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>

#include "./driver/include/uvc_cam_sdk.h"
// #include "./driver/include/bmp.h"
// #include "./driver/src/hid_control/hid_control.hpp"

using namespace cv;
using namespace cv::xfeatures2d;
using namespace std;


#define CAPTURE_IMAGE_FPS (50)
#define WAIT_USLEEP_COFF  (1000*1000/50)
void start_uvc_camera()
{
    char path_f408_firstNode[50];
    udev_search_node(path_f408_firstNode,0);
    uvc_camera_sdk_init(path_f408_firstNode,640,800,1);
}

typedef struct point3d_t
{
    int x;
    int y;
    int z;
}point3d;


int thre = 10;

void trackBar(Mat src);

int main(int argc, char** argv)
{
    start_uvc_camera();
    uvc_camera_sdk_stream_start(1000*1000);
    namedWindow("output",WINDOW_AUTOSIZE);
    namedWindow("input",WINDOW_AUTOSIZE);

    createTrackbar("threshould", "output", &thre,255,NULL);

    static int flag = 0;

    std::vector<point3d> coordinates;
    point3d point1;
    point1.x = 1;
    point1.y = 2;
    point1.z = 3;
    point3d point2;
    point2.x = 2;
    point2.y = 2;
    point2.z = 3;
    point3d point3;
    point3.x = 3;
    point3.y = 2;
    point3.z = 3;
    coordinates.push_back(point1);
    coordinates.push_back(point2);
    coordinates.push_back(point3);
   

    cout << "**********" << coordinates.size() << endl;
    for(int i = 0;i < coordinates.size();i++)
    {
        cout << i <<  "------" << "(" << coordinates[i].x << "," << coordinates[i].y << "," << coordinates[i].z << ")" << endl;
    }

    // src = imread("../640x800.jpg"); 
    while(1)
    {
        camera_t * uvc_frame=uvc_camera_sdk_stream_captured_once();
        
        uint8_t *start = uvc_frame->head.start; 
        Mat src = cv::Mat(cv::Size(640,800), CV_8UC1, start);
        
        if (src.empty())
        {
            printf("can not load image \n");
            return -1;
        }
        
        imshow("input", src);
        
        trackBar(src);
         

        if (cv::waitKey(20) == 27 ) {
			break;
		}
        

        usleep(WAIT_USLEEP_COFF);
    }
    
    uvc_camera_sdk_stream_stop();

    return 0;
    
}

void trackBar(Mat src)
{
    std::vector<KeyPoint> keypoints;
    Ptr<FastFeatureDetector> detector = FastFeatureDetector::create(thre);
    detector->detect(src,keypoints);

    Mat dst = src.clone();
    drawKeypoints(dst, keypoints, dst, Scalar(255,255,0), DrawMatchesFlags::DRAW_OVER_OUTIMG);
    imshow("output", dst); 
}