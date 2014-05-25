#include "kalman/ekfilter.hpp"

class playerEKF : public Kalman::EKFilter<float,1>
{
public:
	playerEKF();

protected:

	void makeA();
	void makeH();
	void makeV();
	void makeR();
	void makeW();
	void makeQ();
	void makeProcess();
	void makeMeasure();

	float periodT;
	double last_update;
	float last_odometry_v;
	float last_odometry_alpha;
	public:
	void updateOdometry(float v,float rot,double time);
	void updatePlayerPos(float r,float theta,double time);
	std::vector<float> getStatus();
	
};