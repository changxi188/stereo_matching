#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <opencv2/core/core.hpp>

#include "chessboard_images.h"
#include "stereo_calib_and_rect.h"
#include "stereo_vision.h"

using namespace std;
using namespace cv;

int main(){
	
	// chessboard image path
	string chessboard_image_path("/home/mr/Downloads/practice/stereo_matching");	

	// left and right image use for stereo match
	cv::Mat ori_left_image = cv::imread("/home/mr/Downloads/practice/stereo_matching/data/left_img/left_img_35.jpg");
	cv::Mat ori_right_image = cv::imread("/home/mr/Downloads/practice/stereo_matching/data/right_img/right_img_35.jpg");
	cv::Mat rected_left_image, rected_right_image;


	// use for load image
	auto chessboard_image = std::make_shared<ChessboardImages>(chessboard_image_path);

	// use for stereo calib and stereo rectation
	auto stereo_calib_and_rect = std::make_shared<StereoCalibAndRect>(chessboard_image);

	// use for stereo match 
	auto stereo_match = std::make_shared<StereoVision>();

	// 计算棋盘格角点3维世界坐标
	stereo_calib_and_rect->calcChessboardCorners();

	// 左相机标定
	bool is_left_cam_calib_ok = stereo_calib_and_rect->leftCameraCalibrate();
	if (!is_left_cam_calib_ok){
		cout << "calibrate false on left camera" << endl;
	}

	// 右相机标定
	bool is_right_cam_calib_ok = stereo_calib_and_rect->rightCameraCalibrate();
	if (!is_right_cam_calib_ok){
		cout << "calibrate false on right camera" << endl;
	}

	// 左右相机立体标定
	double rms = stereo_calib_and_rect->stereoCameraCalibrate();

	// 计算立体矫正需要的参数
	stereo_calib_and_rect->stereoRectify();

	// 显示矫正后的结果
	stereo_calib_and_rect->stereoRectifyVisualize(ori_left_image, ori_right_image, rected_left_image, rected_right_image);
	
	// 进行立体匹配
	stereo_match->stereoMatchSgbm(rected_left_image, rected_right_image);

	// 讲视差图转换成深度图并可视化
	stereo_match->disparityToDepth();

	return 0;
}

