#pragma once
#define _USE_MATH_DEFINES
#include <math.h>
#include <stdlib.h>
#include <time.h>
/*#include <cxcore.h>
#include <cxcore.hpp>
#include <cxoperations.hpp>
#include <cxtypes.h>*/
#include <limits>
#include <opencv.hpp>

#define __IPL_H__
// Includes maps sdk library header
#include "maps.hpp"

#include "xmlConfig.h"

using namespace std;
using namespace cv;
class utils
{
public:
	utils(void);
	~utils(void);

	static void initRandom()
	{
		// init générateur aléatoire
		srand(MAPS::CurrentTime());
	}
	static double gaussRnd(double dMean,double dSigma)
	{
		double u1=0,u2;
		while(u1==0.0)
			u1 = randDouble();

		u2 = randDouble();
		double test= -2*log(u1);
		return (sqrt(test)*cos(2*M_PI*u2)*dSigma + dMean);
		/*if(rnd!=rnd)	
			return 0;//gaussRnd(dSigma,dMean);
		else
			return((rnd));*/
	
	}

	static double randDouble()
	{
		return (double)rand() / (double)RAND_MAX ;
	}

	void utils::initScanLidar(xmlConfig * config);
	void utils::lidarSim(double pose[3], vector<double> &Distances);

private:
	// image source
	IplImage * _OriginalMap;
	xmlConfig * config;

	bool _bIsMapLoaded;
	double _dXmin;
	double _dXmax;
	double _dYmin;
	double _dYmax;

	
	
	double utils::rayTracing(int xPix, int yPix, double Alpha);
	void utils::positionToMap(double x, double y, double &xPix, double &yPix);
};

