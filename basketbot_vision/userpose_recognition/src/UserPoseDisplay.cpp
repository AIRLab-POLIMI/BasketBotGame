#include "UserPoseDisplay.h"
#include <boost/thread/lock_guard.hpp>
#include <boost/thread/thread.hpp>
#include <opencv2/imgproc/imgproc.hpp>

std::string WINDOW_NAME = "Display window";
using namespace cv;
boost::mutex m_mutex;
void UserPoseDisplay::setMouseCallback(CallbackFunc c,void *p)
{
	rightCallback = c;
	rightCallbackParam = p;
}

void UserPoseDisplay::setCurrent(float d,int ind)
{
	distance=d;
	index=ind;
}

void UserPoseDisplay::mouseCallback( int event, int x, int y, int flags, void* param )
{
	UserPoseDisplay *that = (UserPoseDisplay *)param;
	if(event == CV_EVENT_RBUTTONDOWN) {
		(*that->rightCallback)(that->rightCallbackParam,3,0);
		that->lastIndex++;
	}
	if(event == CV_EVENT_MBUTTONDOWN) {
		if(x < 20 || x > 84)
			return;
		int index = y/50;
		(*that->rightCallback)(that->rightCallbackParam,2,index);
	}
}

UserPoseDisplay::UserPoseDisplay():baseImage(480,640,CV_8UC1,0.0),trainingSetImage(1,1,CV_8UC1,0.0)
{
	lastIndex=0;
	ready=false;
}

void UserPoseDisplay::init()
{
	cv::namedWindow( WINDOW_NAME, WINDOW_AUTOSIZE );
	cv::setMouseCallback(WINDOW_NAME,&UserPoseDisplay::mouseCallback,this);
	ready=true;
}
cv::Mat UserPoseDisplay::generateImage(UserPoseData & upd,cv::Size size)
{
	cv::Mat skeletonImage(480,640,CV_8UC1,0.0);
	std::vector<tf::Transform> transforms = upd.readTransforms();
	for(int i=0; i<transforms.size(); ++i) {
		int px = transforms[i].getOrigin().x() * 200 + 320;
		int py = -transforms[i].getOrigin().y() * 200 + 240;
		float pz = transforms[i].getOrigin().z()*750.0;
		if(pz<-180)
			pz = -180.0;
		if(pz > 10)
			pz = 10.0;
		std::cerr <<"x: " <<px<<"  y:"<<py<<std::endl;
		circle(skeletonImage, Point(px,py), 5, 255 -10 + (int)pz,10);
	}
	if(size != cv::Size(640,480))
		cv::resize(skeletonImage,skeletonImage,Size(64,48));
	return skeletonImage;

}
void UserPoseDisplay::showTransforms(UserPoseData & upd)
{
	if(!ready)
		return;
	boost::lock_guard<boost::mutex> lock(m_mutex);

	baseImage.copyTo(image);
	cv::Mat section(image,cv::Rect(cv::Point(0,0),trainingSetImage.size()));
	trainingSetImage.copyTo(section);

	std::stringstream ss;
	ss<<std::fixed<<distance;
	if(index>= 0) {
		putText(image, ss.str() , cv::Point(0,index*50+50), cv::FONT_HERSHEY_COMPLEX_SMALL , 1, cv::Scalar(255),1);
		cv::rectangle(image,cv::Point(20,index*50),cv::Point(84,index*50+48), 255);
		putText(image, poseNames[index] , cv::Point(300,10), cv::FONT_HERSHEY_COMPLEX_SMALL , 1, cv::Scalar(255),1);
	}

	cv::Mat skIm = generateImage(upd,image.size());
	image+=skIm;
	imshow(WINDOW_NAME,image);
}
void UserPoseDisplay::spinOnce()
{
	if(!ready)
		return;
	waitKey(1);
}
void UserPoseDisplay::loadData(std::vector<UserPoseData> & updv)
{
	trainingSetImage=cv::Mat(50*updv.size() + 1,64,CV_8UC1,0.0);
	poseNames.resize(updv.size());
	for(int i = 0; i<updv.size(); ++i) {
		poseNames[i]=updv[i].readName();
		int baseX = i*50;//qui
		cv::Mat section = trainingSetImage.rowRange(baseX,baseX+48);
		cv::Mat immagine = generateImage(updv[i],cv::Size(64,48));
		immagine.copyTo(section);


	}

}
