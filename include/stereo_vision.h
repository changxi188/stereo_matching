#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <Eigen/Core>
#include <pangolin/pangolin.h>
#include <unistd.h>


class StereoVision{
public:

	// 计算视差图
	void stereoMatchSgbm(const cv::Mat& left_image, const cv::Mat& right_image);

	// 将视差图转换成深度图
	void disparityToDepth();
	
	// 画出点云
	void showPointCloud(const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &pointCloud);


private:
	// 左右视图
	cv::Mat rected_left_, rected_right_;

	// 生成点云
	std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> pointcloud_;
	// 使用sgbm算法
	cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(
		0, 96, 9, 8 * 9 * 9, 32 * 9 * 9, 1, 63, 10, 100, 32);  //神奇的参数

	// 视差图
	cv::Mat disparity_;

	double fx_ = 773.3169472178228, fy_ = 778.0109058553899, cx_ = 165.0987372910953, cy_ = 49.78646900539321;
	double b_ = 0.041;
};
