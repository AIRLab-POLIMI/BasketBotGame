#include "math.h"
#include "playerEKF.h"
#include <stdlib.h>
#include <stdio.h>
using namespace Kalman;

void  playerEKF::updatePlayerPos(float r,float theta)
{
	Vector z(2);
	z(1) = r;
	z(2) = theta;
	//std::cout << std::fixed<<"playerpos: " << r << "  " << theta << std::endl;
	measureUpdateStep(z);
}

std::vector<float> playerEKF::getStatus()
{
	Vector x = getX();
	std::vector<float> result;
	float COS = cos(x(2));
	float SIN = sin(x(2));
	result.push_back(x(1) * COS);
	result.push_back(x(1) * SIN) ;
	result.push_back(x(5) * COS - SIN * x(4));
	result.push_back(x(4) * COS + SIN * x(5) ) ;

	Matrix P = calculateP();
	result.push_back(P(1,1));
	result.push_back(P(2,2));

	//std::cout <<std::fixed<< "risultato = " << x<<std::endl;

	//std::cout <<P<<std::endl;
	return result;
}

void playerEKF::updateOdometry(float v,float alpha)
{
	Vector u(2);
	u(1) = v;
	u(2) = alpha;

	timeUpdateStep(u);

}

void playerEKF::makeA()
{

	float COS = cos(periodT*u(2));
	float SIN = sin(periodT*u(2));
	A(1,1) = 1;
	A(1,2) = u(1)*periodT*sin(x(2));
	A(1,3) = 0;
	A(1,4) = 0;
	A(1,5) = periodT;

	A(2,1) = 0;
	A(2,2) = 1;
	A(2,3) = periodT;
	A(2,4) = 0;
	A(2,5) = 0;

	A(3,1) = -x(4)* x(5)/(x(1)*x(1));
	A(3,2) = 0;
	A(3,3) = 0;
	A(3,4) = 1/x(1);
	A(3,5) = 0;

	A(4,1) = 0;
	A(4,2) = 0;
	A(4,3) = 0;
	A(4,4) = 1;
	A(4,5) = 0;

	A(5,1) = 0;
	A(5,2) = 0;
	A(5,3) = 0;
	A(5,4) = 0;
	A(5,5) = 1;

}

void playerEKF::makeH()
{	
	H(1,1) = 1;
	H(1,2) = 0;
	H(1,3) = 0.0;
	H(1,4) = 0.0;
	H(1,5) = 0.0;
	H(2,1) = 0;
	H(2,2) = 1;
	H(2,3) = 0.0;
	H(2,4) = 0.0;
	H(2,5) = 0.0;

}

void playerEKF::makeMeasure()
{
	z(1)=x(1);
	z(2)=x(2);
}

void playerEKF::makeProcess()
{

	Vector x_(x.size());
	x_(1) = x(1) + x(5)*periodT  -u(1)*cos(x(2))*periodT; //rho
	x_(2) = x(2) + x(3) * periodT - u(2) * periodT;      //alpha
	x_(3) = x(4) / x(1);        //omega
	x_(4) = x(4) ;				//vtang
	x_(5) = x(5);				//vradiale
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
	R(1,1) = 1;
	R(1,2) = 0.0;
	R(2,1) = 0.0;
	R(2,2) = 1;
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
	W(3,1) = 0.0;
	W(3,2) = 0.0;
	W(4,1) = 1.0;
	W(4,2) = 0.0;
	W(5,1) = 0.0;
	W(5,2) = 1.0;
}

playerEKF::~playerEKF()
{
	
}

playerEKF::playerEKF()
{
	setDim(4,1,1,1,1);
	setDim(5, 2, 2, 2, 2);
	periodT = 1.0/30.0;



	int n = 5;

	float _P0[] = {100.0*100.0,       0.0,       0.0,       0.0, 0.0,
	               0.0,         10.0*10.0,       0.0,       0.0, 0.0,
	               0.0,               0.0, 25.0*25.0,       0.0, 0.0,
	               0.0,               0.0,       0.0, 10.0*10.0, 0.0,
	               0.0,               0.0,       0.0,       0.0, 25*25.0,
	              };
	Matrix P0(n, n, _P0);
	Vector x(n);
	x(1) = 2;
	x(2) = 0;
	x(3) = 0;
	x(4) = 0;
	x(5) = 0;
	init(x, P0);

	selectKVectorContext(createKVectorContext());
	
	selectKMatrixContext(createKMatrixContext( "-",  "\n","<", 
                                       ">", 
                                      4));
	
}
