#include "stereo_calib_and_rect.h"


using namespace std;
using namespace cv;

void StereoCalibAndRect::calcChessboardCorners(){
	object_points_[0].resize(0);
	for (int i = 0; i < board_size_.height; i++){
		for (int j = 0; j < board_size_.width; ++j){
			object_points_[0].push_back(Point3f(j * chessboard_square_size_, i * chessboard_square_size_, 0));
		}
	}
}

bool StereoCalibAndRect::singleCameraCalibrate(bool is_left,
											   cv::Mat& intr_mat, cv::Mat& dist_coeffs,
											   std::vector<std::vector<cv::Point2f>>& image_points){
	// 判断是否还有多余的chessboard image
	bool is_more_image;
	// 有效棋盘格图片张数
	int nums = 0;

	// 2D corner coordinate on a image
	std::vector<cv::Point2f> point_buf;

	// 相机外参
	std::vector<cv::Mat> r_vecs, t_vecs;


	while(1){	
		cv::Mat view, view_gray;
		if (is_left){
			is_more_image = chessboard_images_->nextLeftImage();
			view = chessboard_images_->left_chessboard_;
		}
		else{
			is_more_image = chessboard_images_->nextRightImage();
			view = chessboard_images_->right_chessboard_;
		}

		if (!is_more_image)
			break;

		// 320 * 240
		image_size_ = view.size();
		//cout << view.size() << endl;

		// convert color image to gray image
		cv::cvtColor(view, view_gray, COLOR_BGR2GRAY); 

		// find corners on chessboard
		bool found = cv::findChessboardCorners(view, board_size_, point_buf,
				CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);

		if (found){
			nums++;
			cv::cornerSubPix(view_gray, point_buf, cv::Size(11, 11), cv::Size(-1, -1),
								TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
			cv::drawChessboardCorners(view, board_size_, cv::Mat(point_buf), found);
			cv::bitwise_not(view, view);
			image_points.push_back(point_buf);
			cout << ".";

		}
		cv::imshow("view", view);
		waitKey(30);
	}

	cout << "有效棋盘图张数" << nums << endl;
	object_points_.resize(image_points.size(), object_points_[0]);

	// calculate chessboardCornes
	//calcChessboardCorners();
	rms_ = cv::calibrateCamera(object_points_, image_points, image_size_, intr_mat, dist_coeffs, r_vecs, t_vecs);
	bool ok = cv::checkRange(intr_mat) && cv::checkRange(dist_coeffs);

	if (ok){
		cout << " RMS error = " << rms_ << endl;
		cout << "intrinsics matrix :" << endl;
		cout << intr_mat << endl;
		cout << "dist coeffs :" << endl;
		cout << dist_coeffs << endl;
		return true;
	}
	else{
		return false;
	}
}

bool StereoCalibAndRect::leftCameraCalibrate(){
	std::cout << "---------------------------------------------------------------------" << std::endl;
	std::cout << "calibrate left camera..." << std::endl;
	return singleCameraCalibrate(true, intr_mat_left_, dist_coeffs_left_, left_image_points_);
};

bool StereoCalibAndRect::rightCameraCalibrate(){
	std::cout << "---------------------------------------------------------------------" << std::endl;
	std::cout << "calibrate right camera..." << std::endl;
	return singleCameraCalibrate(false, intr_mat_right_, dist_coeffs_right_, right_image_points_);
};



double StereoCalibAndRect::stereoCameraCalibrate(){
	rms_ =  cv::stereoCalibrate(object_points_, left_image_points_, right_image_points_,
							   intr_mat_left_, dist_coeffs_left_, intr_mat_right_, dist_coeffs_right_,
							   image_size_, R_, t_, E_, F_, cv::CALIB_USE_INTRINSIC_GUESS,
							   cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 1e-6));
	std::cout << "---------------------------------------------------------------------" << std::endl;
	cout << " stereo calibrate RMS error = " << rms_ << endl;
	cout << "intrinsics left : " << endl;
	cout << intr_mat_left_ << endl;
	cout << "dist coeffs left : " << endl;
	cout << dist_coeffs_left_ << endl;
	cout << "intrinsics right : " << endl;
	cout << intr_mat_right_ << endl;
	cout << "dist coeffs right " << endl;
	cout << dist_coeffs_right_ << endl;
	return rms_;
}


void StereoCalibAndRect::stereoRectify(){	
	cv::Rect validRoi[2];
	std::cout << "---------------------------------------------------------------------" << std::endl;
	cv::stereoRectify(intr_mat_left_, dist_coeffs_left_, intr_mat_right_, dist_coeffs_right_,
				  image_size_, R_, t_, 
				  R_left_, R_right_, P_left_, P_right_, Q_, cv::CALIB_ZERO_DISPARITY, 
				  -1, image_size_, &validRoi[0], &validRoi[1]);
	cout << "Q : " << Q_ << endl;
	cout << "P_left : " << P_left_ << endl;
	cout << "P_right : " << P_right_ << endl;

	// stereoRectify
	cv::initUndistortRectifyMap(intr_mat_left_, dist_coeffs_left_, R_left_, P_left_, image_size_,
							   CV_16SC2, rmap_left_[0], rmap_left_[1]);

	cv::initUndistortRectifyMap(intr_mat_right_, dist_coeffs_right_, R_right_, P_right_, image_size_,
							   CV_16SC2, rmap_right_[0], rmap_right_[1]);


}

void StereoCalibAndRect::stereoRectifyVisualize(const cv::Mat& ori_left_image, const cv::Mat& ori_right_image,
												cv::Mat& rected_left_image, cv::Mat& rected_right_image){
	// 根据立体矫正的参数, 做重映射
	cv::remap(ori_left_image, rected_left_image, rmap_left_[0], rmap_left_[1], cv::INTER_LINEAR);
	cv::remap(ori_right_image, rected_right_image, rmap_right_[0], rmap_right_[1], cv::INTER_LINEAR);

	// 将左右视图拼接起来,用于可视化
	cv::Mat ori_double_image, rected_double_image;
	cv::hconcat(ori_left_image, ori_right_image, ori_double_image);
	cv::hconcat(rected_right_image, rected_right_image, rected_double_image);

	// 画直线
	for (int j = 0; j < ori_double_image.rows; j +=16){
		cv::line(ori_double_image, cv::Point(0, j), cv::Point(ori_double_image.cols, j), Scalar(0, 255, 0), 1, 8);
		cv::line(rected_double_image, cv::Point(0, j), cv::Point(rected_double_image.cols, j), Scalar(0, 255, 0), 1, 8);
	}
	cout << "stereo done" << endl;
	cv::imshow("ori_double_image", ori_double_image);
	cv::imshow("rected_double_image", rected_double_image);
	cv::waitKey(0);

}

