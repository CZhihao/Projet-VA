////////////////////////////////
// RTMaps SDK Component
////////////////////////////////

////////////////////////////////
// Purpose of this module :
////////////////////////////////

#include "maps_monPremierComposant.h"	// Includes the header of this component

// Use the macros to declare the inputs
MAPS_BEGIN_INPUTS_DEFINITION(MAPSmonPremierComposant)
MAPS_INPUT("iOdometry",MAPS::FilterInteger32,MAPS::FifoReader)
MAPS_END_INPUTS_DEFINITION

// Use the macros to declare the outputs
MAPS_BEGIN_OUTPUTS_DEFINITION(MAPSmonPremierComposant)
MAPS_OUTPUT("oDelta_X",MAPS::Float64,NULL,NULL,1)
MAPS_OUTPUT("oDelta_Theta", MAPS::Float64, NULL, NULL, 1)
MAPS_END_OUTPUTS_DEFINITION

// Use the macros to declare the properties
MAPS_BEGIN_PROPERTIES_DEFINITION(MAPSmonPremierComposant)
//MAPS_PROPERTY("pName",128,false,false)
MAPS_END_PROPERTIES_DEFINITION

// Use the macros to declare the actions
MAPS_BEGIN_ACTIONS_DEFINITION(MAPSmonPremierComposant)
//MAPS_ACTION("aName",MAPSmonPremierComposant::ActionName)
MAPS_END_ACTIONS_DEFINITION

// Use the macros to declare this component (monPremierComposant) behaviour
MAPS_COMPONENT_DEFINITION(MAPSmonPremierComposant, "monPremierComposant", "1.0", 128,
	MAPS::Threaded, MAPS::Threaded,
	-1, // Nb of inputs. Leave -1 to use the number of declared input definitions
	-1, // Nb of outputs. Leave -1 to use the number of declared output definitions
	-1, // Nb of properties. Leave -1 to use the number of declared property definitions
	-1) // Nb of actions. Leave -1 to use the number of declared action definitions

		//Initialization: Birth() will be called once at diagram execution startup.			  
	void MAPSmonPremierComposant::Birth()
{
	maVariable_ = 1;
	// Reports this information to the RTMaps console. You can remove this line if you know when Birth() is called in the component lifecycle.
	ReportInfo("Passing through Birth() method");
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
void MAPSmonPremierComposant::Core()
{
	MAPSIOElt*ioeletInOdo = StartReading(Input("iOdometry"));
	if (ioeletInOdo->VectorSize() >= 2) {
		odoL = ioeletInOdo->Integer32(0);
		odoR = ioeletInOdo->Integer32(1);

	}

	// Sleeps during 500 milliseconds (500000 microseconds).
	//This line will most probably have to be removed when you start programming your component.
	// Replace it with another blocking function. (StartReading?)

	Delta_X = (odoR + odoL-lastodoL- lastodoR) / 200.0;
	Delta_Theta = (odoR - odoL + lastodoL - lastodoR) / 100.0;
	lastodoL = odoL;
	lastodoR = odoR;


	MAPSIOElt* ioEltOut = StartWriting(Output("oDelta_X"));
	ioEltOut->Float64() = Delta_X;
	StopWriting(ioEltOut);

	MAPSIOElt* ioEltOut2 = StartWriting(Output("oDelta_Theta"));
	ioEltOut2->Float64() = Delta_Theta;
	StopWriting(ioEltOut2);


	//Rest(500000);
}

//De-initialization: Death() will be called once at diagram execution shutdown.
void MAPSmonPremierComposant::Death()
{
	// Reports this information to the RTMaps console. You can remove this line if you know when Death() is called in the component lifecycle.
	ReportInfo("Passing through Death() method");
}


