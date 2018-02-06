#pragma once
////////////////////////////////
// RTMaps SDK Component header
////////////////////////////////

#ifndef _Maps_ComposantAvecConfigXML_H
#define _Maps_ComposantAvecConfigXML_H
#include <opencv.hpp>

#define __IPL_H__
// Includes maps sdk library header
#include "maps.hpp"

#include "xmlConfig.h"
#include "utils.h"
// Declares a new MAPSComponent child class
class filtreParticulaire
{
public:
	filtreParticulaire(void);
	~filtreParticulaire(void);


	
	void filtreParticulaire::motionUpdate(double delta_x, double delta_theta, double poseUpdated[3]);
	void filtreParticulaire::init(double pose[3], int nbParticule, vector< vector<double> > particule);
	void filtreParticulaire::measurementUpdate(vector<double> &z, vector< vector<double> > particule, int ScanLidar[55]);
	void filtreParticulaire::computePose(double pose[3], vector< vector<double> > particule, vector<double> poid);
	//void filtreParticulaire::resample();
	



private:
	IplImage * _OriginalMap;
	xmlConfig * config;
	
	
	
	
	
	utils utilsLidar;





	


};


class MAPSfiltreParticulaire : public MAPSComponent
{
	// Use standard header definition macro
	MAPS_COMPONENT_STANDARD_HEADER_CODE(MAPSfiltreParticulaire)
private:
	xmlConfig config;
	// Place here your specific methods and attributes
	utils utilsLidar;
	MAPSInput*	m_inputs[3];
	MAPSOutput*	m_outputs[2];
	vector<double> poid;
	double poseInit[3] = { 0,0,0 };
	double poseUpdated[3];
	double deltaX;
	double deltaTheta;
	filtreParticulaire aParticule;
	int ScanLidar[55];
	vector< vector<double> > particule;
	utils utils;
};

#endif
