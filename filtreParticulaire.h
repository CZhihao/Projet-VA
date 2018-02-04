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

	
	vector< vector<double> > particule;
	void filtreParticulaire::motionUpdate(double delta_x, double delta_theta);
	void filtreParticulaire::init(double pose[3], int nbParticule);
	void filtreParticulaire::measurementUpdate((vector<double> &z);
	void filtreParticulaire::computePose(double *pose[3]);
	void filtreParticulaire::resample();
	double poseUpdated[3];



private:
	IplImage * _OriginalMap;
	xmlConfig * config;

	double poseInit[3] = {0,0,0};
	vector<double> poid;

	utils utilsLidar;
	filtreParticulaire aParticule;

	int ScanLidar[55];
	int SimLidar[55];
	
	double deltaX;
	double deltaTheta;
	

};


class MAPSfiltreParticulaire : public MAPSComponent
{
	// Use standard header definition macro
	MAPS_COMPONENT_STANDARD_HEADER_CODE(MAPSfiltreParticulaire)
private:
	xmlConfig config;
	// Place here your specific methods and attributes
	utils utilsLidar;
	MAPSInput*	m_inputs[2];
	double Pose[3];
	float Vraisemblance;
	double gaussienne;
	int ScanLidar[55];
	
	int i;
	double total,varSim,varRel;
	double moySim,moyRel;
	vector<double> SimLidar;
	utils utils;
};

#endif
