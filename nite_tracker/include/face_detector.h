#include "opencv2/objdetect/objdetect.hpp"
#include <map>
#include <boost/thread/mutex.hpp> 
class FaceDetector
{
	boost::mutex faces_profile_mutex;
	
protected:
	virtual float calculateFaceSize(cv::Rect face, float distance);

	unsigned int iteration;
	bool working;
	cv::CascadeClassifier face_front_classifier;
	cv::CascadeClassifier face_profile_classifier;

	bool loadClassifier(std::string name, cv::CascadeClassifier &cascade);
	cv::Mat loadImage(cv::Mat);
	void    findFaces(bool front_only=false);
	void shrinkRectangles(std::vector<cv::Rect> &rectangles,float ratio);
	void filterFacesByDepth(cv::Mat depth);
	cv::Mat frame_gray;
	cv::Mat image;
	std::vector<cv::Rect> faces_front;
	std::vector<cv::Rect> faces_profile;
	void findFacesProfile(bool reverse);
	void drawDetectedFaces(cv::Scalar color= cv::Scalar(0,255,255));
	void filterFacesByDepthInternal(cv::Mat depth,std::vector<cv::Rect> &,float min,float max);
	void checkRectangle(cv::Mat image, cv::Rect &);
public:
	FaceDetector();
	void analyze(cv::Mat,cv::Mat);
	int analyze(cv::Mat image);
	std::vector<int> extractHumans(cv::Mat);
};
