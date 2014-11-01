#ifndef USERPOSE_DISPLAY_H
#define USERPOSE_DISPLAY_H
#include <vector>
#include <tf/transform_datatypes.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "userpose_data.h"


class UserPoseDisplay
{
	typedef void (*CallbackFunc)(void *,int,int);
	void * rightCallbackParam;

	CallbackFunc rightCallback;

	std::vector<std::string> poseNames;
	float distance;
	int index;
	cv::Mat baseImage;
	cv::Mat image;
	cv::Mat trainingSetImage;

	int lastIndex;
	cv::Mat generateImage(UserPoseData & upd,cv::Size size);
	bool ready;
public:
	UserPoseDisplay();
	void showTransforms(UserPoseData &);
	static void mouseCallback( int event, int x, int y, int flags, void* param );
	void setMouseCallback(CallbackFunc,void *param);
	void spinOnce();
	void setCurrent(float d,int ind);
	void loadData(std::vector<UserPoseData> & updv);
	void init();
};
#endif
