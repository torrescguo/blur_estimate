// #include <opencv2/opencv.hpp>
// #include <opencv2/features2d.hpp> 
// using namespace cv; 
// int main()
// { 
//     //读取图片 
//     Mat img = imread("../data/rgb_test.png"); 
//     //定义ORB特征检测器 
//     Ptr<ORB> orb = ORB::create();
//     orb->setNLevels(8); 
//     orb->setFastThreshold(7);
//     orb->setEdgeThreshold(15);
//     orb->setMaxFeatures(1500);
    
//     //存放关键点的容器 
//     std::vector<KeyPoint> keyPoints; 
//     //检测出特征点 
//     orb->detect(img, keyPoints); 
//     //绘制出特征点 
//     Mat outImg; 
//     drawKeypoints(img,keyPoints,outImg, Scalar(0,255,0,-1), DrawMatchesFlags::DEFAULT); 
//     //显示结果 
//     imshow("orbFeatures", outImg); 
//     waitKey(0); 
//     //保存结果图片 
//     imwrite("../data/output.jpg",outImg); 
//     return 0; 
// }




#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp> 
using namespace cv; 
int main() {

    // 加载人脸检测器模型
    Ptr<ORB> orb = ORB::create();

    // 读取图像并将其转换为灰度图像
    cv::Mat img = cv::imread("../data/face.jpg", cv::IMREAD_GRAYSCALE);

    // 创建ORB对象并设置其参数
    // orb->set("nfeatures", 3000);
    // orb->set("threshold", 1.3);
    orb->setNLevels(8); 
    orb->setFastThreshold(30);
    orb->setEdgeThreshold(35);
    orb->setMaxFeatures(3000);

    // 在图像中检测人脸
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    orb->detectAndCompute(img, Mat(), keypoints, descriptors);

    // 在图像中绘制人脸关键点
    cv::Mat outputImg;
    cv::drawKeypoints(img, keypoints, outputImg);

    // 将结果显示在窗口中
    cv::namedWindow("Face Detection");
    imshow("Face Detection", outputImg);
    waitKey(0);

    return 0;
}

