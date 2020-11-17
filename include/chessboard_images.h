#pragma once
#ifndef __CHESSBOARD_IMAGES_H__
#define __CHESSBOARD_ImAGES_H__

#include <iostream>
#include <memory>
#include <opencv/cv.hpp>
#include <opencv2/core/core.hpp>


class ChessboardImages{
public:
	typedef std::shared_ptr<ChessboardImages> Ptr;
	ChessboardImages(const std::string& dataset_path) : dataset_path_(dataset_path) {}

	// use to load images
	bool nextImage(const std::string& left_or_right, int& img_index, cv::Mat& chessboard);

	bool nextLeftImage();

	bool nextRightImage();


public:
	cv::Mat left_chessboard_, right_chessboard_;

private:
	std::string dataset_path_;

	int current_left_img_index_ = 0;
	int current_right_img_index_ = 0;
};


#endif


