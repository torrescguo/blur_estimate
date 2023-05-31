#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp> 
using namespace cv; 
int main()
{ 
    //读取图片 
    Mat img = imread("../data/rgb_test.png"); 
    //定义ORB特征检测器 
    Ptr<ORB> orb = ORB::create();
    orb->setNLevels(8); 
    orb->setFastThreshold(7);
    orb->setEdgeThreshold(15);
    orb->setMaxFeatures(1500);
    
    //存放关键点的容器 
    std::vector<KeyPoint> keyPoints; 
    //检测出特征点 
    orb->detect(img, keyPoints); 
    //绘制出特征点 
    Mat outImg; 
    drawKeypoints(img,keyPoints,outImg, Scalar(0,255,0,-1), DrawMatchesFlags::DEFAULT); 
    //显示结果 
    imshow("orbFeatures", outImg); 
    waitKey(0); 
    //保存结果图片 
    imwrite("../data/output.jpg",outImg); 
    return 0; 
}


