#pragma once
#ifndef __STEREO_CALIB_AND_RECT_H__
#define __STEREO_CALIB_AND_RECT_H__
#include <iostream>
#include <string>
#include <vector>

#include <opencv/cv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "chessboard_images.h"

class ChessboardImages;

class StereoCalibAndRect{
public:
	typedef std::shared_ptr<StereoCalibAndRect> Ptr;
	StereoCalibAndRect(ChessboardImages::Ptr chessboard_images) : chessboard_images_(chessboard_images){
		//board_size_(w_, h_);
		board_size_.width = w_;
		board_size_.height = h_;
	}

	// record corners 3d coordinate in chessboard
	void calcChessboardCorners();

	// singel camera calibrate 
	bool singleCameraCalibrate(bool is_left,
							   cv::Mat& intr_mat, cv::Mat& dist_coeffs, 
							   std::vector<std::vector<cv::Point2f>>& image_points);

	// left camera calibrate
	bool leftCameraCalibrate();

	// right camera calibrate
	bool rightCameraCalibrate();

	// TODO: 对准左右相机照片

	// stereo camera calibrate
	double stereoCameraCalibrate();

	// stereo rectify
	void stereoRectify();


	// visual after stereo rectify
	void stereoRectifyVisualize(const cv::Mat& ori_left_image, const cv::Mat& ori_right_image,
								cv::Mat& rected_left_image, cv::Mat& rected_right_image);
	
	// stereo calibrate;
	virtual ~StereoCalibAndRect(){}


private:
	// -------------------------------- parameters -----------------------------------
	ChessboardImages::Ptr chessboard_images_;
	// the chessboard size
	const int w_ = 10;
	const int h_ = 8;
	cv::Size board_size_;
	//board_size_.width = w_;
	//board_size_.height = h_;
	// square size of chessboard
	const float chessboard_square_size_ = 10.0f;
	// reprojection error
	double rms_ = 0;



	// ---------------------------------- data ---------------------------------------
	// left/right camera intrinsics, distcoeffs
	cv::Mat intr_mat_left_, intr_mat_right_, dist_coeffs_left_, dist_coeffs_right_;
	// 2d corner pixel coordinate on all images
	std::vector<std::vector<cv::Point2f>> left_image_points_, right_image_points_;
	// 3d corner coordinate in world coordinates
	std::vector<std::vector<cv::Point3f>> object_points_ = 
		std::vector<std::vector<cv::Point3f>>(1);
	// image size
	cv::Size image_size_;


	// following data use for stereocalib
	// 两个相机之间的旋转矩阵
	cv::Mat R_;
	// 两个相机之间的平移矩阵
	cv::Mat t_;
	// 本质矩阵
	cv::Mat E_;
	// 基础矩阵
	cv::Mat F_;
	
	// 左目矫正矩阵(旋转矩阵)
	cv::Mat R_left_;
	// 右目矫正矩阵(旋转矩阵)
	cv::Mat R_right_;
	// 左目投影矩阵
	cv::Mat P_left_;
	// 右目投影矩阵
	cv::Mat P_right_;
	// disparity-to-depth 映射矩阵
	cv::Mat Q_;


	// stereo rectify
	cv::Mat rmap_left_[2], rmap_right_[2];

};

#endif
