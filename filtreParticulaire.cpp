////////////////////////////////
// RTMaps SDK Component
////////////////////////////////

////////////////////////////////
// Purpose of this module :
////////////////////////////////

#include "filtreParticulaire.h"	// Includes the header of this component
#define _USE_MATH_DEFINES
#define NumberParticle 100
#include <math.h>
#include "XmlconfigFilenameStruct.h"
#include <string>

//declarations of fonctions

filtreParticulaire::filtreParticulaire(void)
{
	config = NULL;
}


filtreParticulaire::~filtreParticulaire(void)
{
}

//declarations of the fonction init 
void filtreParticulaire::init(double poseInit[3], int nbParticule, vector< vector<double> > particule)
{
	double gaussienne;
	for (int i = 0; i < nbParticule; i++)
	{

		gaussienne = utils::gaussRnd(0.0, 0.2);//noise of the x
		particule[i][0] = poseInit[0] + gaussienne;
		gaussienne = utils::gaussRnd(0.0, 0.2);//noise of the y
		particule[i][1] = poseInit[1] + gaussienne;
		gaussienne = utils::gaussRnd(0.0, 1.0);//noise of the delta
		particule[i][2] = poseInit[2] + gaussienne;

	}
}

void filtreParticulaire::motionUpdate(double delta_x, double delta_theta, double poseUpdated[3])
{
	double gaussienne;
	gaussienne = utils::gaussRnd(0.0, 0.2);
	poseUpdated[0] = poseUpdated[0] + delta_x * cos(poseUpdated[2] + delta_theta / 2) + gaussienne;
	gaussienne = utils::gaussRnd(0.0, 0.2);
	poseUpdated[1] = poseUpdated[1] + delta_x * sin(poseUpdated[2] + delta_theta / 2) + gaussienne;
	gaussienne = utils::gaussRnd(0.0, 1);
	poseUpdated[2] = poseUpdated[2] + delta_theta + gaussienne;
}

void filtreParticulaire::measurementUpdate(vector<double> &z, vector< vector<double> > particule, int ScanLidar[55])
{
	vector<double> SimLidar;
	double moySim = 0.0;
	double moyRel = 0.0;
	double varRel = 0.0;
	double varSim = 0.0;


	for (int i = 0; i < particule.size(); i++)
	{
		double posParticule[3];
		posParticule[0] = particule[i][0];
		posParticule[1] = particule[i][1];
		posParticule[2] = particule[i][2];
		utilsLidar.lidarSim(posParticule, SimLidar);
		moySim = 0.0;
		moyRel = 0.0;
		varRel = 0.0;
		varSim = 0.0;
		for (i = 0; i < 55; i++)
		{
			moySim = SimLidar[i] + moySim;
			moyRel = ScanLidar[i] + moyRel;
		}
		moySim = moySim / 55 * 1000;
		moyRel = moyRel / 55;

		for (i = 0; i < 55; i++)
		{
			varSim = varSim + (SimLidar[i] * 1000 - moySim)*(SimLidar[i] * 1000 - moySim);
			varRel = varRel + (ScanLidar[i] - moyRel)*(ScanLidar[i] - moyRel);
		}

		if (varSim > varRel)
		{
			z[i] = varRel / varSim;
		}
		else
		{
			z[i] = varSim / varRel;
		}
	}


}

void filtreParticulaire::computePose(double pose[3], vector< vector<double> > particule, vector<double> poid)
{
	int sumFactor, sumFacMultParX, sumFacMultParY, sumFacMultParTheta;
	for (int i = 0; i < particule.size(); i++)
	{
		sumFactor += poid[i];
		sumFacMultParX += poid[i] * particule[i][0];
		sumFacMultParY += poid[i] * particule[i][1];
		sumFacMultParTheta += poid[i] * particule[i][2];

		//pose[2] += z[i] * (particule[i][2] % 3.6);
	}
	pose[0] = sumFacMultParX / sumFactor;
	pose[1] = sumFacMultParY / sumFactor;
	//pose[2] = pose[2] % 3.6 / particule.size();

}



// Use the macros to declare the inputs
MAPS_BEGIN_INPUTS_DEFINITION(MAPSfiltreParticulaire)
MAPS_INPUT("iConfigXml", MAPSFilterXMLconfigFilenameStruct, MAPS::FifoReader)
MAPS_INPUT("Delta_x", MAPS::FilterFloat64, MAPS::FifoReader)
MAPS_INPUT("Delta_theta", MAPS::FilterFloat64, MAPS::FifoReader)
MAPS_INPUT("Scanlidar", MAPS::FilterInteger32, MAPS::FifoReader)
MAPS_END_INPUTS_DEFINITION

// Use the macros to declare the outputs
MAPS_BEGIN_OUTPUTS_DEFINITION(MAPSfiltreParticulaire)
MAPS_OUTPUT("pose", MAPS::Float64, NULL, NULL, 3)
MAPS_OUTPUT("particules", MAPS::Float64, NULL, NULL, NumberParticle)
MAPS_END_OUTPUTS_DEFINITION

// Use the macros to declare the properties
MAPS_BEGIN_PROPERTIES_DEFINITION(MAPSfiltreParticulaire)
//MAPS_PROPERTY("pName",128,false,false)
MAPS_END_PROPERTIES_DEFINITION

// Use the macros to declare the actions
MAPS_BEGIN_ACTIONS_DEFINITION(MAPSfiltreParticulaire)
//MAPS_ACTION("aName",MAPSComposantAvecConfigXML::ActionName)
MAPS_END_ACTIONS_DEFINITION

// Use the macros to declare this component (ComposantAvecConfigXML) behaviour
MAPS_COMPONENT_DEFINITION(MAPSfiltreParticulaire, "MAPSVraieSemblance", "1.0", 128,
	MAPS::Threaded, MAPS::Threaded,
	-1, // Nb of inputs. Leave -1 to use the number of declared input definitions
	-1, // Nb of outputs. Leave -1 to use the number of declared output definitions
	-1, // Nb of properties. Leave -1 to use the number of declared property definitions
	-1) // Nb of actions. Leave -1 to use the number of declared action definitions

		//Initialization: Birth() will be called once at diagram execution startup.			  
	void MAPSfiltreParticulaire::Birth()
{
	m_inputs[0] = &Input("iDelta_x");
	m_inputs[1] = &Input("iDelta_theta");
	m_inputs[2] = &Input("iScanLidar");
	m_outputs[0] = &Output("oPose");
	m_outputs[1] = &Output("oParticules");
	MAPSIOElt* ioEltIn = StartReading(Input("iConfigXml"));
	if (ioEltIn == NULL)
		return;

	XMLconfigFilenameStruct &ReceivedConfigStruct = *static_cast<XMLconfigFilenameStruct *>(ioEltIn->Data());
	std::string type = ReceivedConfigStruct.type;
	if (type != "config")
	{

		while (1)
		{
			ReportError("xmlFile receveid is not a config type.");
			Rest(2000000);//on se plante l?et on attend
		}
	}
	else
	{
		if (!config.parseXmlConfigFile(ReceivedConfigStruct.XMLconfigFilename))
		{
			while (1)
			{
				ReportError("xmlFile does not exist or is not a robot type.");
				Rest(2000000);//on se plante l?et on attend
			}
		}
		else
		{
			// faire les inits après chargement de la config ici
			utils::initRandom();
			utilsLidar.initScanLidar(&config);
		}
	}

	//Initilizition of the position : inititial position is {0,0,0} with a noise
	aParticule.init(poseInit, NumberParticle, particule);


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
void MAPSfiltreParticulaire::Core()
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


	//Rtmaps input
	int inputThatAnswered;
	MAPSIOElt* ioEltIn = StartReading(3, m_inputs, &inputThatAnswered);
	if (ioEltIn == NULL)
		return;
	if (inputThatAnswered == 0) {
		deltaX = ioEltIn->Float64(0);
	}
	if (inputThatAnswered == 1) {
		deltaTheta = ioEltIn->Float64(0);
	}
	if (inputThatAnswered == 2)
	{
		int i;
		for (i = 0; i < 55; i++)
		{
			ScanLidar[i] = ioEltIn->Integer32(i);
		}
	}//End of the input

	 //Update the actual position
	aParticule.motionUpdate(deltaX, deltaTheta, poseUpdated);

	//Update the mesure: compare the actual position's lidar scan with every particle
	aParticule.measurementUpdate(poid, particule, ScanLidar);

	//Calculate the position with the position of the particles and their factor
	aParticule.computePose(poseUpdated, particule, poid);







	//Rtmaps output 
	MAPSIOElt* ioEltOut1 = StartWriting(Output("oPoseUpdated"));
	ioEltOut1->Float32(0) = poseUpdated[0];
	ioEltOut1->Float32(1) = poseUpdated[1];
	ioEltOut1->Float32(2) = poseUpdated[2];
	StopWriting(ioEltOut1);

	MAPSIOElt* ioEltOut2 = StartWriting(Output("oparticule"));
	for (int i = 0; i < 55; i++)
	{
	ioEltOut2->Float64(0) = particule[i][0];
	ioEltOut2->Float64(1) = particule[i][1];
	ioEltOut2->Float64(2) = particule[i][2];
	}
	StopWriting(ioEltOut2);


	

}

//De-initialization: Death() will be called once at diagram execution shutdown.
void MAPSfiltreParticulaire::Death()
{
	// Reports this information to the RTMaps console. You can remove this line if you know when Death() is called in the component lifecycle.
	ReportInfo("Passing through Death() method");
}
