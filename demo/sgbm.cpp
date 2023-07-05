#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
using namespace std;
using namespace cv;
// 声明一些全局变量
cv::Ptr<cv::StereoSGBM> SGBM = cv::StereoSGBM::create();

//回调函数
void SGBMUpdate(int pos, void* data) {
	//cv::Mat disp;
	int SGBMNum = 2;
	int blockSize = cv::getTrackbarPos("blockSize", "SGBM_disparity");
	if (blockSize % 2 == 0){
		blockSize += 1;
	}
	if (blockSize < 5){
		blockSize = 5;
	}
	SGBM->setBlockSize(blockSize);
	SGBM->setNumDisparities(cv::getTrackbarPos("numDisparities", "SGBM_disparity"));
	SGBM->setSpeckleWindowSize(cv::getTrackbarPos("speckleWindowSize", "SGBM_disparity"));
	SGBM->setSpeckleRange(cv::getTrackbarPos("speckleRange", "SGBM_disparity"));
	SGBM->setUniquenessRatio(cv::getTrackbarPos("uniquenessRatio", "SGBM_disparity"));
	SGBM->setDisp12MaxDiff(cv::getTrackbarPos("disp12MaxDiff", "SGBM_disparity"));
	/*int P1 = 8 * left.channels() * SADWindowSize* SADWindowSize;
	int P2 = 32 * left.channels() * SADWindowSize* SADWindowSize;*/
	// 惩罚系数，一般：P1=8*通道数*SADWindowSize*SADWindowSize，P2=4*P1
	SGBM->setP1(600);
	// p1控制视差平滑度，p2值越大，差异越平滑
	SGBM->setP2(2400);
	SGBM->setMode(cv::StereoSGBM::MODE_SGBM);
	
}

/*
	SGBM函数初始化函数
*/
void SGBMStart() {
	// 最小视差值
	int minDisparity = 0;
	int SGBMNum = 2;
	// 视差范围，即最大视差值和最小视差值之差，必须是16的倍数。
	int numDisparities = SGBMNum * 16;
	// 匹配块大小，大于1的奇数
	int blockSize = 5; 
	// P1, P2控制视差图的光滑度
	// 惩罚系数，一般：P1 = 8 * 通道数*SADWindowSize*SADWindowSize，P2 = 4 * P1
	int P1 = 600;  
	// p1控制视差平滑度，p2值越大，差异越平滑
	int P2 = 2400; 
	// 左右视差图的最大容许差异（超过将被清零），默认为 - 1，即不执行左右视差检查。
	int disp12MaxDiff = 200; 
	int preFilterCap = 0;
	// 视差唯一性百分比， 视差窗口范围内最低代价是次低代价的(1 + uniquenessRatio / 100)倍时，最低代价对应的视差值才是该像素点的视差，否则该像素点的视差为 0，通常为5~15.
	int uniquenessRatio = 6; 
	// 平滑视差区域的最大尺寸，以考虑其噪声斑点和无效。将其设置为0可禁用斑点过滤。否则，将其设置在50 - 200的范围内。
	int speckleWindowSize = 60; 
	// 视差变化阈值，每个连接组件内的最大视差变化。如果你做斑点过滤，将参数设置为正值，它将被隐式乘以16.通常，1或2就足够好了
	int speckleRange = 2; 
	
	// cv::namedWindow("SGBM_disparity");
	cv::createTrackbar("blockSize", "SGBM_disparity", &blockSize, 21, SGBMUpdate);
	cv::createTrackbar("numDisparities", "SGBM_disparity", &numDisparities, 20, SGBMUpdate);
	cv::createTrackbar("speckleWindowSize", "SGBM_disparity", &speckleWindowSize, 200, SGBMUpdate);
	cv::createTrackbar("speckleRange", "SGBM_disparity", &speckleRange, 50, SGBMUpdate);
	cv::createTrackbar("uniquenessRatio", "SGBM_disparity", &uniquenessRatio, 50, SGBMUpdate);
	cv::createTrackbar("disp12MaxDiff", "SGBM_disparity", &disp12MaxDiff, 21, SGBMUpdate);
	
	// 创建SGBM算法对象
	
}

int main() {
	cv::Mat left = imread("../data/pointcloud/left.png", IMREAD_GRAYSCALE);
	cv::Mat right = imread("../data/pointcloud/right.png", IMREAD_GRAYSCALE);
	cout  << "step 1" << endl;
	cv::Mat disp;
	// cv::namedWindow("SGBM_disparity",cv::WINDOW_KEEPRATIO);
	SGBMStart();
	cout  << "step 2" << endl;
	while (true) {
		cout  << "step 3" << endl;
		SGBM->compute(left, right, disp);
		cv::Mat disp8U = Mat(disp.rows, disp.cols, CV_8UC1);       //显示
		normalize(disp, disp8U, 0, 255, NORM_MINMAX, CV_8UC1);
		cv::imshow("SGBM_disparity", disp8U);
		cv::waitKey(4);
		cout  << "step 4" << endl;
	}
	cout  << "step 5" << endl;
	return 0;

}

