#include "strategy_math.h"
#include <math.h> 
unsigned int StrategyMath::misuraInversioni(std::list<float> &vettore, float ampiezza_max)
{
	int situazione = 0;

	unsigned int inversioni = 0;


	for(std::list<float>::iterator it = vettore.begin(); it!=vettore.end(); ++it) {
		if(situazione > -1 && *it < -ampiezza_max) {
			inversioni++;
			situazione = -1;
		} else if(situazione < 1 && *it > ampiezza_max) {
			inversioni++;
			situazione = 1;
		}

	}
	return inversioni;
}

float StrategyMath::calcAverage(std::list<float> & lista)
{
	if(lista.size() == 0)
		return 0;
	float media = 0;

	for(std::list<float>::iterator it = lista.begin(); it!=lista.end(); ++it) {
		float val = *it;
		media += val;

	}
	media /=lista.size();
	return media;
}

float StrategyMath::avgDistance(std::list<float> & lista,float origin)
{
	if(lista.size() == 0)
		return 0;
	float media = 0;

	for(std::list<float>::iterator it = lista.begin(); it!=lista.end(); ++it) {
		float val = fabs(*it - origin);
		media += val;

	}
	media /=lista.size();
	return media;


}
