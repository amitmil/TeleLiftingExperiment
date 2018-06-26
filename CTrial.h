//===========================================================================
/*
	Software License Agreement (BSD License)
	Copyright (c) 2003-2016, CHAI3D
	(www.chai3d.org)

	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions
	are met:

	* Redistributions of source code must retain the above copyright
	notice, this list of conditions and the following disclaimer.

	* Redistributions in binary form must reproduce the above
	copyright notice, this list of conditions and the following
	disclaimer in the documentation and/or other materials provided
	with the distribution.

	* Neither the name of CHAI3D nor the names of its contributors may
	be used to endorse or promote products derived from this software
	without specific prior written permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
	FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
	COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
	BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
	ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	POSSIBILITY OF SUCH DAMAGE.

	\author    <http://www.chai3d.org>
	\author    Francois Conti
	\version   3.2.0 $Rev: 1869 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef cTrialH
#define cTrialH
//---------------------------------------------------------------------------
#include "chai3d.h"
#include "CODE.h"
#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
//#include "TeleLiftingExperiment.cpp"

using namespace chai3d;
//---------------------------------------------------------------------------
//void startLogging(void);

class cTrial
{
	//-----------------------------------------------------------------------
	// CONSTRUCTOR & DESTRUCTOR:
	//-----------------------------------------------------------------------

public:

	//! Constructor of cTrial.
	cTrial(const std::string a_resourceRoot,
		const int a_numDevices,
		std::shared_ptr<cGenericHapticDevice> a_hapticDevice0,
		std::shared_ptr<cGenericHapticDevice> a_hapticDevice);

	//! Destructor of cTrial.
	virtual ~cTrial() {};


	//-----------------------------------------------------------------------
	// PUBLIC METHODS:
	//-----------------------------------------------------------------------

public:

	//! Initialize to demo
	virtual void initTrial();
	//! Update haptics
	virtual void updateHaptics();
	//! update protocol
	virtual void updateProtocol();
	//! update logging
	void updateLogging(void);
	//! Update graphics
	virtual void updateGraphics(int a_width, int a_height);
	//!
	virtual void loadTrial();
	//! Set stiffness
	virtual void setStiffness(double a_stiffness) {};

	//! Set offset
	virtual void setOffset(double a_offset);

	//! Set torque gain
	virtual void setTorqueGain(double a_torqueGain);


	//-----------------------------------------------------------------------
	// PUBLIC MEMBERS:
	//-----------------------------------------------------------------------

public:

	//! virtual world
	cWorld* m_world;

	//! camera
	cCamera* m_camera;

	//! light source 0
	cSpotLight *m_light0;

	//! light source 1
	cDirectionalLight *m_light1;

	//! radius of tools
	double m_toolRadius;

	//! tool 0
	cToolGripper* m_tool0;

	//! tool 1
	cToolGripper* m_tool1;

	//! table 
	cMesh* m_ground;

	//! base
	cMultiMesh* m_base;

	//! simulation clock
	cPrecisionClock simClock;

	//! logging clock 
	cPrecisionClock logClock;

	//! number of tools
	int m_numTools;

	//! table of tools
	cToolGripper* m_tools[2];

	//! ODE world
	cODEWorld* m_ODEWorld;

	//! ODE planes
	cODEGenericBody* m_ODEGPlane0;
	cODEGenericBody* m_ODEGPlane1;
	cODEGenericBody* m_ODEGPlane2;
	cODEGenericBody* m_ODEGPlane3;
	cODEGenericBody* m_ODEGPlane4;
	cODEGenericBody* m_ODEGPlane5;

	//! torque gain
	double m_torqueGain;

	//! mirroed display
	bool m_mirroredDisplay;

	// a small scope to display the interaction force signal
	cScope* scope;
	cScope* scope2;

	// a frequency counter to measure the simulation graphic rate
	cFrequencyCounter freqCounterGraphics;

	// a frequency counter to measure the simulation haptic rate
	cFrequencyCounter freqCounterHaptics;

	// a label to display the rate [Hz] at which the simulation is running
	cLabel* labelRates;
	cLabel* labelHaptics;
	cLabel* labelTrialInfo;
	cLabel* labelTrialInstructions;
	// a font for rendering text
	cFontPtr font;

	//! ODE objects and world related stuff
	cODEGenericBody* m_ODEBody1;
	cShapeBox* box;
	cShapeBox* box2;

	double boxSize = 0.02;
	cShapeLine* line;
	cShapeLine* spring;

	// haptic related stuff
	bool flagChangeBox = false;
	bool flagVisual = false;
	bool flagBox = true;
	double gap = boxSize;
	bool flagLoad = false;
	double loadForce = 0;
	double boundary = 0.05;
	bool loggingRunning = false;


	/*--------------------------------------------------------------------*/
	/* EXPERIMENT RELATED STUFF */

	// arrays for experiment properties from files
	// loaded in loadTrial()
	int visualCond[500];
	int springCond[500];
	int handleCond[500];
	double boundaryCond[500];

	// properties of current trial
	// setup of trial in updateProtocol()
	int kSpring = 200;
	int kHandle = 200;
	int kVisual = 1;
	double kBoundary;
	double upTarget = 0.08;
	double downTarget = 0.04;
	cMesh* m_downTarget;
	cMesh* m_upTarget;
	cMesh* m_boundary;
	// data logging properties
	cThread* loggingThread;
	double fs = 2000; // sampling frequency [Hz]
	std::string subjectName = "";
	bool appendToFile;
	// experiment statemachine
	int expState = 1;
	int trialState = 1;
	int trialNumber = 115;
	int liftingNumber;
	/*--------------------------------------------------------------------*/
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
