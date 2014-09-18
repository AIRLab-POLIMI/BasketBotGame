#include "kalman/ekfilter.hpp"

class playerEKF : public Kalman::EKFilter<float,1>
{
public:
	playerEKF();
	~playerEKF();
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
	
	public:
	void updateOdometry(float v,float rot);
	void updatePlayerPos(float r,float theta);
	std::vector<float> getStatus();
	
};
