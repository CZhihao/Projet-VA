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
class filtreParticulaire
{
public:
	filtreParticulaire(void);
	~filtreParticulaire(void);


	vector<vector<double>> particule;
	void filtreParticulaire::motionUpdate(double delta_x, double delta_theta);
	void filtreParticulaire::init(double pose[3], int nbParticule);
	void filtreParticulaire::measurementUpdate(vector<int> &z);
	void filtreParticulaire::computePose(double *pose[3]);
	void filtreParticulaire::resample();


private:
	IplImage * _OriginalMap;
	xmlConfig * config;
	double poseInit[3] = { 0,0,0 };

};
