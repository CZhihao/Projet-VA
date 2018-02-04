#ifndef _XMLCONFIGFILENAMESTRUCT_H
#define _XMLCONFIGFILENAMESTRUCT_H
#include "maps.hpp"

#pragma pack(push,1)
/// \struct CalibrationStruct : structure de calibration échangé entre les composant rtmaps
struct XMLconfigFilenameStruct 
{
	char XMLconfigFilename[1024];					/// filename image 
	double version;									/// version du fichier
	char type[64];									/// type de config  
	char name[64];									/// name  

};

//Now re-enable structure alignment
#pragma pack(pop)

// The RTMaps input filter for the structure MyNewStructure
const MAPSTypeFilterBase MAPSFilterXMLconfigFilenameStruct=MAPSFilterUserStructure(XMLconfigFilenameStruct);


#endif