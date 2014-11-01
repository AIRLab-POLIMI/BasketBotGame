#include <ros/ros.h>
#include <stdlib.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <signal.h>
#include <sys/wait.h>

#include <stdio.h>


int run_as_watchdog()
{
	while(1) {
		ros::Time startTime = ros::Time::now();

		int pid  = fork() ;
		if(pid < 0)
			exit(2);
		if(pid == 0) {

			execlp("rosnode","rosnode", "ping", "/camera/driver",NULL);
			exit(1);
		}
		wait(NULL);
		ros::Duration elapsed = ros::Time::now() - startTime;
		std::cerr<<"passati: "<<elapsed.toSec()<<std::endl;
		if(elapsed.toSec() > 1.0) {
			std::cerr<<"nodo crashato"<<std::endl;
			kill(getppid(),SIGINT);
			return 0;
		}
		if(!ros::ok())
			return 0;
	}
}

int run_as_launcher(char * launchname)
{
	while(1) {
		int pid  = fork() ;
		if(pid < 0)
			exit(2);
		if(pid == 0) {
			char * const args[] = {};
			execlp("roslaunch","roslaunch","basketbot_launch", launchname,"use_watchdog:=true",NULL);
			exit(1);
		}
		wait(NULL);
		if(!ros::ok())
			return 0;
	}

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
int main2(int argc, char **argv)
{
	ros::init(argc, argv, "face_recognition_watchdog");
	ros::NodeHandle nh;
	std::cerr << argc<<std::endl;
	if(argc > 1)
		return run_as_launcher(argv[1]);
	else
		return run_as_watchdog();
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
	while(1) {
		if(!launch_pid)
			launch_pid = launch(argv +1 );
		if(!watchdog_pid) {
			watchdog_pid = watchdog();
			startTime = ros::Time::now();
		}
		int dead_pid = wait(NULL);
		ros::spinOnce();
		if(!ros::ok())
			break;
		if(dead_pid == launch_pid) {
			launch_pid = 0;
		}
		if(dead_pid == watchdog_pid) {
			watchdog_pid = 0;
			ros::Duration elapsed = ros::Time::now() - startTime;
			std::cerr<<"passati: "<<elapsed.toSec()<<std::endl;
			if(elapsed.toSec() > 1.0 && launch_pid > 0) 
				kill(launch_pid,SIGTERM);
		}


	}
	wait(NULL);
	return 0;
}
