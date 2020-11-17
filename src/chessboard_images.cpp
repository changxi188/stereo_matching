#include <string>
#include <boost/format.hpp>
#include "chessboard_images.h"

bool ChessboardImages::nextImage(const std::string& left_or_right, int& img_index, cv::Mat& chessboard){
	boost::format fmt("%s/chessboard_images/%s/%s_%02d.jpg");
	chessboard = cv::imread((fmt % dataset_path_ % left_or_right % left_or_right % img_index).str());

	if (chessboard.data == nullptr){	
		return false;
	}
	//std::cout << img_index;
	img_index +=5;
	return true;
}

bool ChessboardImages::nextLeftImage(){
	return nextImage("left_img", current_left_img_index_, left_chessboard_);
}

bool ChessboardImages::nextRightImage(){
	return nextImage("right_img", current_right_img_index_, right_chessboard_);
}
