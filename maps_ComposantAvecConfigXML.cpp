////////////////////////////////
// RTMaps SDK Component
////////////////////////////////

////////////////////////////////
// Purpose of this module :
////////////////////////////////

#include "maps_ComposantAvecConfigXML.h"	// Includes the header of this component
#define _USE_MATH_DEFINES
#include <math.h>
#include "XmlconfigFilenameStruct.h"
#include <string>


// Use the macros to declare the inputs
MAPS_BEGIN_INPUTS_DEFINITION(MAPSComposantAvecConfigXML)
MAPS_INPUT("iConfigXml",MAPSFilterXMLconfigFilenameStruct,MAPS::FifoReader)
MAPS_INPUT("iOdometry",MAPS::FilterInteger32,MAPS::FifoReader)
MAPS_END_INPUTS_DEFINITION

// Use the macros to declare the outputs
MAPS_BEGIN_OUTPUTS_DEFINITION(MAPSComposantAvecConfigXML)
MAPS_OUTPUT("oPose",MAPS::Integer32,NULL,NULL,3)
MAPS_END_OUTPUTS_DEFINITION

// Use the macros to declare the properties
MAPS_BEGIN_PROPERTIES_DEFINITION(MAPSComposantAvecConfigXML)
    //MAPS_PROPERTY("pName",128,false,false)
MAPS_END_PROPERTIES_DEFINITION

// Use the macros to declare the actions
MAPS_BEGIN_ACTIONS_DEFINITION(MAPSComposantAvecConfigXML)
    //MAPS_ACTION("aName",MAPSComposantAvecConfigXML::ActionName)
MAPS_END_ACTIONS_DEFINITION

// Use the macros to declare this component (ComposantAvecConfigXML) behaviour
MAPS_COMPONENT_DEFINITION(MAPSComposantAvecConfigXML,"ComposantAvecConfigXML","1.0",128,
			  MAPS::Threaded,MAPS::Threaded,
			  -1, // Nb of inputs. Leave -1 to use the number of declared input definitions
			  -1, // Nb of outputs. Leave -1 to use the number of declared output definitions
			  -1, // Nb of properties. Leave -1 to use the number of declared property definitions
			  -1) // Nb of actions. Leave -1 to use the number of declared action definitions

//Initialization: Birth() will be called once at diagram execution startup.			  
void MAPSComposantAvecConfigXML::Birth()
{
    MAPSIOElt* ioEltIn = StartReading(Input("iConfigXml"));
	if (ioEltIn == NULL)
		return;

	XMLconfigFilenameStruct &ReceivedConfigStruct=*static_cast<XMLconfigFilenameStruct *>(ioEltIn->Data());
	std::string type=ReceivedConfigStruct.type;
	if(type!="config")
	{
		
		while(1)
		{
			ReportError("xmlFile receveid is not a config type.");
			Rest(2000000);//on se plante là et on attend
		}
	}
	else
	{
		if(!config.parseXmlConfigFile(ReceivedConfigStruct.XMLconfigFilename))
		{
			while(1)
			{
				ReportError("xmlFile does not exist or is not a robot type.");
				Rest(2000000);//on se plante là et on attend
			}
		}
		else
		{
			// faire les inits après chargement de la config ici
			utils::initRandom();
			utilsLidar.initScanLidar(&config);
		}
	}


}

//ATTENTION: 
//	Make sure there is ONE and ONLY ONE blocking function inside this Core method.
//	Consider that Core() will be called inside an infinite loop while the diagram is executing.
//	Something similar to: 
//		while (componentIsRunning) {Core();}
//
//	Usually, the one and only blocking function is one of the following:
//		* StartReading(MAPSInput& input); //Data request on a single BLOCKING input. A "blocking input" is an input declared as FifoReader, LastOrNextReader, Wait4NextReader or NeverskippingReader (declaration happens in MAPS_INPUT: see the beginning of this file). A SamplingReader input is non-blocking: StartReading will not block with a SamplingReader input.
//		* StartReading(int nCount, MAPSInput* inputs[], int* inputThatAnswered, int nCountEvents = 0, MAPSEvent* events[] = NULL); //Data request on several BLOCKING inputs.
//		* SynchroStartReading(int nb, MAPSInput** inputs, MAPSIOElt** IOElts, MAPSInt64 synchroTolerance = 0, MAPSEvent* abortEvent = NULL); // Synchronized reading - waiting for samples with same or nearly same timestamps on several BLOCKING inputs.
//		* Wait(MAPSTimestamp t); or Rest(MAPSDelay d); or MAPS::Sleep(MAPSDelay d); //Pauses the current thread for some time. Can be used for instance in conjunction with StartReading on a SamplingReader input (in which case StartReading is not blocking).
//		* Any blocking grabbing function or other data reception function from another API (device driver,etc.). In such case, make sure this function cannot block forever otherwise it could freeze RTMaps when shutting down diagram.
//**************************************************************************/
//	In case of no blocking function inside the Core, your component will consume 100% of a CPU.
//  Remember that the StartReading function used with an input declared as a SamplingReader is not blocking.
//	In case of two or more blocking functions inside the Core, this is likely to induce synchronization issues and data loss. (Ex: don't call two successive StartReading on FifoReader inputs.)
/***************************************************************************/
void MAPSComposantAvecConfigXML::Core() 
{
    // Reports this information to the RTMaps console. You can remove this line if you know when Core() is called in the component lifecycle.
    ReportInfo("Passing through Core() method");

    // Sleeps during 500 milliseconds (500000 microseconds).
	//This line will most probably have to be removed when you start programming your component.
	// Replace it with another blocking function. (StartReading?)
    //Rest(500000);
	
	//test d'un tirage aléatoire gaussien
	/* 
	std::ostringstream strs;
	double gaussienne= utils::gaussRnd(0.0,1.0);
	strs<<gaussienne;
	std::string str="N(mu=0,sigma=1)" + strs.str();
	ReportInfo(str.c_str());

	// test de simulation d'un scan lidar
	vector<double> distances;
	double pose[3]={0,0,0};
	utilsLidar.lidarSim(pose,distances);
	std::ostringstream strs2;
	strs2 << "scan lidar en [0,0,0] : ";
	for(int i=0;i<distances.size();i++)
		strs2 << distances[i] <<"|";
	ReportInfo(strs2.str().c_str());
	*/

	MAPSIOElt*ioeletInOdo = StartReading(Input("iOdometry"));
	if (ioeletInOdo->VectorSize() >= 2) {
		odoL = ioeletInOdo->Integer32(0);
		odoR = ioeletInOdo->Integer32(1);
	}
	Delta_X = (odoR + odoL - lastodoL - lastodoR) / 200.0;
	Delta_Theta = (odoR - odoL + lastodoL - lastodoR) / 100;
	lastodoL = odoL;
	lastodoR = odoR;
	
	//poseX = poseX + Delta_X*cos(Theta + Delta_Theta / 2);
	//poseY = poseY + Delta_X*sin(Theta + Delta_Theta / 2);
	//Theta = Theta + Delta_Theta;

	poseX = 1;
	poseY = 1;
	Theta = 1;

	MAPSIOElt* ioEltOut = StartWriting(Output("oPose"));
	ioEltOut->Integer32(0) = poseX;
	ioEltOut->Integer32(1) = poseY;
	ioEltOut->Integer32(2) = Theta;
	StopWriting(ioEltOut);

}

//De-initialization: Death() will be called once at diagram execution shutdown.
void MAPSComposantAvecConfigXML::Death()
{
    // Reports this information to the RTMaps console. You can remove this line if you know when Death() is called in the component lifecycle.
    ReportInfo("Passing through Death() method");
}
