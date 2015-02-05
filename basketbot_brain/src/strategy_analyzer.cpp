#include <strategy_analyzer.h>

StrategyAnalyzer::StrategyAnalyzer()
{


}

void StrategyAnalyzer::recordDistance(float v)
{

}


float StrategyAnalyzer::getAvgSpeed()
{
	return StrategyMath::calcAverage(lastSpeeds);
}
bool StrategyAnalyzer::recordSpeed(float speed )
{
	lastSpeeds.push_back(speed);
	if(lastSpeeds.size() > 20) {
		lastSpeeds.pop_front();
		return true;
	}
	return false;
}
void StrategyAnalyzer::recordOrientation(float v)
{

}
