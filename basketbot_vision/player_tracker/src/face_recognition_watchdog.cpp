#include <ros/ros.h>
#include <stdlib.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <signal.h>
#include <sys/wait.h>
#include <image_transport/image_transport.h>
#include <stdio.h>
ros::Time imageTime;
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	imageTime = ros::Time::now();
}



int launch(char ** argv)
{
	int pid  = fork();
	if(pid < 0)
		exit(2);
	if(pid == 0) {
		char * const args[] = {};
		execlp("roslaunch","roslaunch",argv[0], argv[1],NULL);
		exit(1);
	}
	return pid;
}

int watchdog()
{
	int pid  = fork();
	if(pid < 0)
		exit(2);
	if(pid == 0) {
		freopen ("/dev/null","w",stdout);
		freopen ("/dev/null","w",stderr);
		char * const args[] = {};
		execlp("rosnode","rosnode", "ping", "/camera/driver",NULL);
		exit(1);
	}
	return pid;
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "face_recognition_watchdog");
	ros::NodeHandle nh;
	std::cerr << argc<<std::endl;
	if(argc < 3)
		return 0;
	ros::Time startTime;
	int launch_pid = 0,watchdog_pid = 0;

	image_transport::ImageTransport it(nh);

	image_transport::Subscriber sub = it.subscribe("/camera/depth/image_raw", 1, imageCallback);

	while(1) {
		if(!launch_pid)
			launch_pid = launch(argv +1 );
		if(!watchdog_pid) {
			watchdog_pid = watchdog();
			startTime = ros::Time::now();
		}
		
		int dead_pid = waitpid(-1,NULL,WNOHANG);
		ros::spinOnce();
		if(!ros::ok())
			break;
		
		if(dead_pid == launch_pid) {
			launch_pid = 0;
		} else if(dead_pid == watchdog_pid) {
			watchdog_pid = 0;
			ros::Duration elapsed = ros::Time::now() - startTime;
			std::cerr<<"passati: "<<elapsed.toSec()<<std::endl;
			if(elapsed.toSec() > 1.0 && launch_pid > 0)
				kill(launch_pid,SIGTERM);
		} else {
			ros::Duration elapsed = ros::Time::now() - startTime;
			ros::Duration lastImage = ros::Time::now() - imageTime;
			if(elapsed.toSec()>15.0 && lastImage.toSec() > 5.0)
				kill(launch_pid,SIGTERM);
			else
				ros::Duration(0.5).sleep();
		}


	}
	wait(NULL);
	return 0;
}
