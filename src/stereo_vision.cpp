#include "stereo_vision.h"

using namespace std;
using namespace Eigen;


// 计算视差
void StereoVision::stereoMatchSgbm(const cv::Mat& rected_left, const cv::Mat& rected_right){
	
	rected_left.copyTo(rected_left_);
	rected_right.copyTo(rected_right_);

	cv::Mat disparity_sgbm;

	sgbm->compute(rected_left, rected_right, disparity_sgbm);

	disparity_sgbm.convertTo(disparity_, CV_8U, 255 / ((numDisparities * 16 + 16)*16.));

}

void StereoVision::disparityToDepth(){
	for (int v = 0; v < rected_left_.rows; v++){
		for (int u = 0; u < rected_left_.cols; u++){
			if (disparity_.at<float>(v, u) <= 0.0 || disparity_.at<float>(v, u) >= 96.0) continue;

			// 前三维为x, y, z, 第四维为颜色
			Eigen::Vector4d point(0, 0, 0, rected_left_.at<uchar>(v, u) / 255.0);

			double x = (u - cx_) / fx_;
			double y = (v - cy_) / fy_;
			double depth = fx_ * b_ / (disparity_.at<float> (v, u));
			point[0] = x * depth;
			point[1] = y * depth;
			point[2] = depth;

			pointcloud_.push_back(point);
		}
	}
	cv::imshow("disparity", disparity_ / 96.0);
	cv::waitKey(0);
	showPointCloud(pointcloud_);
} 


// 画出点云
void StereoVision::showPointCloud(const vector<Eigen::Vector4d, Eigen::aligned_allocator<Vector4d>> &pointcloud) {

    if (pointcloud.empty()) {
        cerr << "Point cloud is empty!" << endl;
        return;
    }

    pangolin::CreateWindowAndBind("Point Cloud Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
        .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glPointSize(2);
        glBegin(GL_POINTS);
        for (auto &p: pointcloud) {
            glColor3f(p[3], p[3], p[3]);
            glVertex3d(p[0], p[1], p[2]);
        }
        glEnd();
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
    return;
}
