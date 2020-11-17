#include <opencv2/opencv.hpp>
#include <iostream>
#include <boost/format.hpp>

using namespace std;

using namespace cv;


int main(){

	VideoCapture cap(2);
	if (!cap.isOpened()){

		cout << " open camera failed" << endl;
		return -1;
	}

	cvWaitKey(30);
	Mat frame;
	cap >> frame;
	//cout << 1111111111111111 << endl;
	cout << frame.rows << " " << frame.cols << endl;
	//cout << 2222222222222222 << endl;
	system("sh ~/Downloads/practice/stereo_matching/config/enable_camera.sh");
	imshow("original", frame);
	int num_img = 0;
	boost::format left_fmt("/home/mr/Downloads/practice/stereo_matching/data/left_img/left_img_%02d.jpg");
	boost::format right_fmt("/home/mr/Downloads/practice/stereo_matching/data/right_img/right_img_%02d.jpg");

	while(1){
		cap >> frame;
		if (frame.empty())
			break;
		imshow("original", frame);
		//waitKey(20);

		Mat DoubleImage;
		resize(frame, DoubleImage, Size(640, 240), (0, 0), (0, 0), INTER_AREA);
		//resize(frame, DoubleImage, Size(1280, 480), (0, 0), (0, 0), INTER_AREA);
		imshow("double image", DoubleImage);

		Mat LeftImage = DoubleImage(Rect(0, 0, 320, 240));
		Mat RightImage = DoubleImage(Rect(320, 0, 320, 240));	

		//Mat LeftImage = DoubleImage(Rect(0, 0, 640, 480));
		//Mat RightImage = DoubleImage(Rect(640, 0, 640, 480));
		if(num_img % 5 == 0){
			imshow("left image", LeftImage);
			imshow("Right image", RightImage);
			imwrite(boost::str(left_fmt % num_img), LeftImage);
			cout << (left_fmt % num_img).str() << endl;
			imwrite(boost::str(right_fmt % num_img), RightImage);
			cout << (right_fmt % num_img).str() << endl;
		}
		cout << num_img << endl;
		//imshow(boost::str(left_fmt % num_img), LeftImage);
		//imshow(boost::str(right_fmt % num_img), RightImage);


		/*
		char c = cvWaitKey(30);
		if (c == 27){
			break;
		}
		*/

		num_img++;
		waitKey(0);

	}

	cap.release();
}
