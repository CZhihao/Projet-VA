#pragma once

#include <string>


class xmlConfigMap
{
public:
	typedef struct{
		int pixelX0;
		int pixelY0;
		double pixelSize;
	}TmapPixelCalib;


	TmapPixelCalib mapPixelCalib;
	std::string mapFile;


};

class xmlConfigRobot
{
public:
	typedef struct{
		double timeCst;
		double maxSpeed;
	}Tmotor;

	typedef struct{
		double x;
		double y;
		double theta;
	}TinitialPosition;

	Tmotor motor;
	TinitialPosition initialPosition;
	double wheelTrack;
	int odoTickPerMeter;
	double period;
};

class xmlConfigLidar
{
public:
	typedef struct{
		double sigmaNoise;
		double distanceCoef;
		double distanceMin;
		double distanceMax;
		int noEchoValue;
		//std::string name;
	}Tray;
	typedef struct{
		double startAngle;
		double endAngle;
		double resolution;
	}Tfov;

	double period;
	Tray ray;
	Tfov Hfov;

};

class xmlConfig
{

public:
	xmlConfigMap map;
	xmlConfigRobot robot;
	xmlConfigLidar lidar;


	xmlConfig(void);
	~xmlConfig(void);

	bool parseXmlConfigFile(std::string filename);
};