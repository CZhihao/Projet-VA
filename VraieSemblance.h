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
class MAPSVraieSemblance : public MAPSComponent
{
	// Use standard header definition macro
	MAPS_COMPONENT_STANDARD_HEADER_CODE(MAPSVraieSemblance)
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
