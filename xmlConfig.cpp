#include "xmlConfig.h"
#include "TinyXML/tinyxml.h"
using namespace std;

xmlConfig::xmlConfig(void)
{
}


xmlConfig::~xmlConfig(void)
{
}

bool xmlConfig::parseXmlConfigFile(string filename)
{
	TiXmlDocument doc(filename);
	bool loadOkay = doc.LoadFile();
	if (!loadOkay)
	{
		return false; // on arrete là
	}
	
	TiXmlHandle hDoc(&doc);
	TiXmlElement* element;
	string temp;

	

	//map
	element = hDoc.FirstChildElement("mapPixelsCalib").Element();
	element->QueryIntAttribute("pixelX0",&map.mapPixelCalib.pixelX0);
	element->QueryIntAttribute("pixelY0",&map.mapPixelCalib.pixelY0);
	element->QueryDoubleAttribute("pixelSize",&map.mapPixelCalib.pixelSize);


	
	element = hDoc.FirstChildElement("mapFile").Element();
	map.mapFile  = element->GetText();
	// robot
	element = hDoc.FirstChildElement("robot").Element();
	element->QueryDoubleAttribute("period",&robot.period);

	element = hDoc.FirstChildElement("initialPosition").Element();
	element->QueryDoubleAttribute("x",&robot.initialPosition.x);
	element->QueryDoubleAttribute("y",&robot.initialPosition.y);
	element->QueryDoubleAttribute("theta",&robot.initialPosition.theta);

	element = hDoc.FirstChildElement("geometry").Element();
	element->QueryDoubleAttribute("wheelTrack",&robot.wheelTrack);

	element = hDoc.FirstChildElement("motor").Element();
	element->QueryDoubleAttribute("timeCst",&robot.motor.timeCst);
	element->QueryDoubleAttribute("maxSpeed",&robot.motor.maxSpeed);

	element = hDoc.FirstChildElement("odometer").Element();
	element->QueryIntAttribute("tickPerMeter",&robot.odoTickPerMeter);
	//lidar
	element = hDoc.FirstChildElement("lidar").Element();
	element->QueryDoubleAttribute("period",&lidar.period);
	element = hDoc.FirstChildElement("ray").Element();
	element->QueryDoubleAttribute("sigmaNoise",&lidar.ray.sigmaNoise);
	element->QueryDoubleAttribute("distanceCoef",&lidar.ray.distanceCoef);
	element->QueryDoubleAttribute("distanceMin",&lidar.ray.distanceMin);
	element->QueryDoubleAttribute("distanceMax",&lidar.ray.distanceMax);
	element->QueryIntAttribute("noEchoValue",&lidar.ray.noEchoValue);
	element = hDoc.FirstChildElement("Hfov").Element();
	element->QueryDoubleAttribute("startAngle",&lidar.Hfov.startAngle);
	element->QueryDoubleAttribute("endAngle",&lidar.Hfov.endAngle);
	element->QueryDoubleAttribute("resolution",&lidar.Hfov.resolution);

	return true;
}