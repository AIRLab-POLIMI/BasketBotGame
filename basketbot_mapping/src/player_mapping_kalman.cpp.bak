#include "math.h"
#include "playerEKF.h"
#include <stdlib.h>
#include <stdio.h>
using namespace Kalman;

void  playerEKF::updatePlayerPos(float r,float theta,double time)
{
	Vector z(2);
	z(1) = r;
	z(2) = theta;
	updateOdometry(last_odometry_v,last_odometry_alpha,time);
	measureUpdateStep(z);
}

std::vector<float> playerEKF::getStatus()
{
	Vector x = getX();
	std::vector<float> result;
	result.push_back(x(1));
	result.push_back(x(2));
	result.push_back(x(3));
	result.push_back(x(4));
	
	Matrix P = calculateP();
	result.push_back(P(1,1));
	result.push_back(P(2,2));
	
	std::cout << "risultato = " << x<<std::endl;
	
	std::cout <<calculateP()<<std::endl;
	return result;
}

void playerEKF::updateOdometry(float v,float alpha,double time)
{
	last_odometry_v = v;
	last_odometry_alpha = alpha;
	if(last_update == 0)
	{
		last_update = time;
		return;
	}
	
	Vector u(2);
	u(1) = v;
	u(2) = alpha;
	
	while(time - last_update > periodT)
	{
		timeUpdateStep(u);
		last_update += periodT;
	}
}

void playerEKF::makeA()
{

	float COS = cos(periodT*u(2));
	float SIN = sin(periodT*u(2));
	A(1,1) = COS;
	A(1,2) = -SIN;
	A(1,3) = periodT;
	A(1,4) = 0;

	A(2,1) = SIN;
	A(2,2) = COS;
	A(2,3) = 0;
	A(2,4) = periodT;

	A(3,1) = 0;
	A(3,2) = 0;
	A(3,3) = COS;
	A(3,4) = -SIN;

	A(4,1) = 0;
	A(4,2) = 0;
	A(4,3) = SIN;
	A(4,4) = COS;

}

void playerEKF::makeH()
{
	H(1,1) = x(1)/sqrt(x(1)*x(1)+x(2)*x(2));
	H(1,2) = x(2)/sqrt(x(1)*x(1)+x(2)*x(2));
	H(1,3) = 0.0;
	H(1,4) = 0.0;

	H(2,1) = -x(2)/(x(1)*x(1)+x(2)*x(2));
	H(2,2) = x(1)/(x(1)*x(1)+x(2)*x(2));
	H(2,3) = 0.0;
	H(2,4) = 0.0;

}

void playerEKF::makeMeasure()
{
	z(1)=sqrt(x(1)*x(1)+x(2)*x(2));
	z(2)=atan2(x(2), x(1));
}

void playerEKF::makeProcess()
{
	float COS = cos(periodT*u(2));
	float SIN = sin(periodT*u(2));
	Vector x_(x.size());
	x_(1) = x(1)*COS - x(2) * SIN  + x(3)*periodT- periodT*u(1);
	x_(2) = x(1) * SIN + x(2) * COS + x(4) * periodT ;
	x_(3) = x(3) * COS - x(4) * SIN;
	x_(4) = x(4) * COS + x(3) * SIN;
	x.swap(x_);

}

void playerEKF::makeQ()
{
	Q(1,1) = 1;
	Q(1,2) = 0.01*0.01/10.0;
	Q(2,1) = 0.01*0.01/10.0;
	Q(2,2) = 1;
}

void playerEKF::makeR()
{
	R(1,1) = 0.1;
	R(1,2) = 0.0;
	R(2,1) = 0.0;
	R(2,2) = 0.1;
}

void playerEKF::makeV()
{
	V(1,1) = 1.0;
	V(1,2) = 0.0;
	V(2,1) = 0.0;
	V(2,2) = 1.0;
}

void playerEKF::makeW()
{
	W(1,1) = 0.0;
	W(1,2) = 0.0;
	W(2,1) = 0.0;
	W(2,2) = 0.0;
	W(3,1) = 1.0;
	W(3,2) = 0.0;
	W(4,1) = 0.0;
	W(4,2) = 1.0;
}

playerEKF::playerEKF()
{
	setDim(4, 2, 2, 2, 2);
	periodT = 0.15;
	last_update = 0;
	int n = 4;

	float _P0[] = {100.0*100.0, 0.0, 0.0, 0.0,
	               0.0, 10.0*10.0, 0.0, 0.0,
	               0.0, 0.0, 25.0*25.0, 0.0,
	               0.0, 0.0, 0.0, 10.0*10.0
	              };
	Matrix P0(n, n, _P0);
	Vector x(4);
	x(1) = 2;
	x(2) = 0;
	x(3) = 0;
	x(4) = 0;
	init(x, P0);

	selectKVectorContext(createKVectorContext());
}
