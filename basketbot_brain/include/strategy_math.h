#include <list>

class StrategyMath
{
	public:
	static unsigned int misuraInversioni(std::list<float> &vettore, float ampiezza_max);
	static float calcAverage(std::list<float> &);
	static float avgDistance(std::list<float> & lista,float origin = 0.0f);
	
};