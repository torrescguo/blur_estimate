

#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <math.h>
#include "../driver/include/hello.h"
using namespace std;
using namespace cv;

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


void read_data_toFile(char * path,uint8_t * mem,uint32_t bytes)
{
    FILE * fp = fopen(path,"rb");
    fread(mem,sizeof(uint8_t),bytes,fp);
    fclose(fp);
}
 
int main()
{
    
    log22();
    
    uint8_t rawbuff[640*400*4];
    uint8_t * second_start = rawbuff + 640*800;

	read_data_toFile("../data/640*800.raw",rawbuff,640*400*2);//imread("../640x800.jpg");
    memcpy(second_start,rawbuff,640*400*2);
	cout << "********************" << endl;
    Mat imageSource = cv::Mat(cv::Size(640,400*2), CV_8UC1, second_start);
    Mat imageGrey = cv::Mat(cv::Size(640,400*2), CV_8UC1, rawbuff);
    cout << "********************" << endl;
	// cvtColor(imageSource, imageGrey, CV_RGB2GRAY);
    cout << "********************" << endl;
    cout << imageGrey.cols << "\n" << imageGrey.rows << endl;
    generate_gamma_lut(gamma_lut,GAMMA_VALUE);

    gamma_correction(imageGrey.data,imageGrey.cols,imageGrey.rows,gamma_lut);
    

	Mat imageSobel;
	Sobel(imageGrey, imageSobel, CV_16U, 1, 1);
 
	//图像的平均灰度
	double meanValue = 0.0;
	meanValue = mean(imageSobel)[0];
 
	//double to string
	stringstream meanValueStream;
	string meanValueString;
	meanValueStream << meanValue;
	meanValueStream >> meanValueString;
	meanValueString = "Articulation(Sobel Method): " + meanValueString;
	putText(imageSource, meanValueString, Point(20, 50), 3, 0.8, Scalar(255, 255, 25), 2);
	imshow("Articulation", imageGrey);
    imshow("ArticulationOri", imageSource);
	waitKey();
}