#include <list>
#include <strategy_math.h>
class StrategyAnalyzer
{
	
	std::list<float> lastSpeeds;
std::list<float> lastDistances;
std::list<float> lastOrientations;

	
public:
	StrategyAnalyzer();
	void recordDistance(float);
	bool recordSpeed(float);
	void recordOrientation(float);
	float getAvgSpeed();
};