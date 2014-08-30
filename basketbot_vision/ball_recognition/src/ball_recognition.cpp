#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <list>

static const std::string OPENCV_WINDOW = "Image window";

		int a=15,b=30,c=50,d=190,e=164,f=256;
		int cl=1,op=1;


struct ball_data
{
	float x;
	float y;
	float r;
};

void on_trackbar( int, void* )
{
	
}


class ImageConverter
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;

public:
	ImageConverter()
		: nh_("~"),it_(nh_) {
		// Subscrive to input video feed and publish output video feed
		image_sub_ = it_.subscribe("/camera/image_raw", 1,
		                           &ImageConverter::imageCb, this);
		image_pub_ = it_.advertise("/image_converter/output_video", 1);

		nh_.getParam("a",a);
		nh_.getParam("b",b);
		nh_.getParam("c",c);
		nh_.getParam("d",d);
		std::cout << a << std::endl;
		cv::namedWindow(OPENCV_WINDOW);
		cv::namedWindow("control");
		cv::createTrackbar( "H min", "control", &a, 179, on_trackbar );
		cv::createTrackbar( "H max", "control", &b, 179, on_trackbar );
		cv::createTrackbar( "S min", "control", &c, 256, on_trackbar );
		cv::createTrackbar( "S max", "control", &d, 256, on_trackbar );
		cv::createTrackbar( "V min", "control", &e, 256, on_trackbar );
		cv::createTrackbar( "V max", "control", &f, 256, on_trackbar );
		cv::createTrackbar( "close", "control", &cl, 10, on_trackbar );
		cv::createTrackbar( "open", "control", &op, 10, on_trackbar );
		
	}

	~ImageConverter() {
		cv::destroyWindow(OPENCV_WINDOW);
		cv::destroyWindow("control");
	}

	void imageCb(const sensor_msgs::ImageConstPtr& msg) {
		cv_bridge::CvImagePtr cv_ptr;
		try {
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		cv::Mat img_ = cv_ptr->image.clone();

		//cv::Point point(img_.rows/2,img_.cols/2);


		//cv::Scalar val = cvGet2D(img_, 320, 240);
		cvtColor( cv_ptr->image, img_, CV_BGR2HSV );

		cv::Vec3b bgr = img_.at<cv::Vec3b>(img_.rows/2,img_.cols/2);
		std::cout << bgr <<std::endl;

		//cv::inRange(img_, cv::Scalar(0.11*179, 0.0*256, 0.0*256, 0),cv::Scalar(0.44*179, 1.00*256, 1.00*256, 0), img_);
		//cv::inRange(img_, cv::Scalar(15, a,00, 0),cv::Scalar(30, b, 256, 0), img_);
		cv::inRange(img_, cv::Scalar(a, c,e, 0),cv::Scalar(b, d, f, 0), img_);


		cv::Mat element = getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( op,op ));
		cv::Mat elementc = getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( cl,cl ));
		morphologyEx( img_, img_, cv::MORPH_OPEN, element );
		morphologyEx( img_, img_, cv::MORPH_CLOSE, elementc );
		//GaussianBlur( img_,img_, cv::Size(9, 9), 2, 2 );
		std::vector<cv::Vec3f> circles;
		//HoughCircles(img_, circles, CV_HOUGH_GRADIENT, 1,img_.rows/8, 200, 80, 0, 0 );
		std::cout << "Cerchi: "<< circles.size()<<std::endl;

		for( size_t i = 0; i < circles.size(); i++ ) {
			cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
			int radius = cvRound(circles[i][2]);
			// circle center
			cv::circle( img_, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
			// circle outline
			cv::circle( img_, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
		}


		// Draw an example circle on the video stream
		if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
			cv::circle(img_, cv::Point(cv_ptr->image.cols/2,cv_ptr->image.rows/2), 10, CV_RGB(255,0,0));

		// Update GUI Window
		cv::imshow(OPENCV_WINDOW, img_);
		cv::waitKey(3);

		// Output modified video stream
		image_pub_.publish(cv_ptr->toImageMsg());
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_converter");
	ImageConverter ic;
	ros::spin();
	return 0;
}
