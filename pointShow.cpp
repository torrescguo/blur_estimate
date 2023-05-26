#include<iostream>//取决于你的实际需要
#include<vector>
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>//这个头文件是与图像处理有关的必须的
/*以下是我对命名空间的习惯*/
using namespace std;
using namespace cv;

// circle(image2, points2, 3, Scalar(255, 0, 120), 30);//画圆，空心的
// circle(image1, points1, 3, Scalar(0, 255, 120), -1);//画点，其实就是实心圆

int main(){
	Mat image1 = imread("../640x800.jpg", 1);//路径根据自己的改
	vector<Point2f> points1;
	/*我的图是640*480的，所以以下生成点注意一下数据范围
	.......
	生成点懒得写了，我用的时候是keypoint转的，要自己造的话，参考这句
	points1.push_back(Point2f(2, 3));  
	*/
    for (int i = 160;i < 480;i+=20)
    {
        for (int j = 200;j < 600;j+=20)
        {
            points1.push_back(Point2f(i, j));
        }
    }
    
    cout << points1.size() << endl;
	for(int i = 0; i < points1.size(); i++){
		circle(image1, points1[i], 1, Scalar(0, 255, 0), -1);//画点，其实就是实心圆
	}
	imshow("PointsinImage", image1);
	waitKey(0);//敲键盘关图片，别直接×
	return 0;
}
