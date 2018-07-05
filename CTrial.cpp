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
	\version   3.2.0 $Rev: 1928 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "cTrial.h"
using namespace std;
//---------------------------------------------------------------------------
#include "CODE.h"
//---------------------------------------------------------------------------

//------------------------------------------------------------------------------
// DECLARED MACROS
//------------------------------------------------------------------------------
// convert to resource path
#define RESOURCE_PATH(p)    (char*)((a_resourceRoot+string(p)).c_str())


//===========================================================================
/*!
	Constructor of cTrial.
*/
//===========================================================================
cTrial::cTrial(const string a_resourceRoot,
	const int a_numDevices,
	shared_ptr<cGenericHapticDevice> a_hapticDevice0,
	shared_ptr<cGenericHapticDevice> a_hapticDevice1)
{
	// load the experiment conditions
	loadTrial();
	// display is not mirrored
	m_mirroredDisplay = false;

	// torque gain
	m_torqueGain = 2.0;

	// initialize tool radius
	m_toolRadius = 0.0025;

	// create world
	m_world = new cWorld();;

	// set background color
	m_world->m_backgroundColor.setWhite();

	// set shadow level
	m_world->setShadowIntensity(0.3);

	// create camera
	m_camera = new cCamera(m_world);
	m_world->addChild(m_camera);

	// position and oriente the camera
	m_camera->set(cVector3d(0.3, -0.1, 0.2),    // camera position (eye)
		cVector3d(0.0, -0.1, 0.0),    // lookat position (target)
		cVector3d(0.0, 0.0, 1.0));   // direction of the "up" vector

// set the near and far clipping planes of the camera
// anything in front/behind these clipping planes will not be rendered
	m_camera->setClippingPlanes(0.01, 4.0);
	m_camera->setUseMultipassTransparency(true);
	m_camera->setStereoEyeSeparation(0.005);
	m_camera->setStereoFocalLength(0.7);

	// create a positional light
	m_light0 = new cSpotLight(m_world);
	m_world->addChild(m_light0);                      // attach light to camera
	m_light0->setEnabled(true);                       // enable light source
	m_light0->setLocalPos(0.4, 0.4, 0.3);             // position the light source
	m_light0->setDir(-0.4, -0.4, -0.3);               // define the direction of the light beam
	m_light0->m_ambient.set(0.6, 0.6, 0.6);
	m_light0->m_diffuse.set(0.8, 0.8, 0.8);
	m_light0->m_specular.set(0.8, 0.8, 0.8);
	m_light0->m_shadowMap->setEnabled(true);
	m_light0->setCutOffAngleDeg(40);
	m_light0->m_shadowMap->setQualityVeryHigh();

	// create a directional light
	cDirectionalLight* m_light1 = new cDirectionalLight(m_world);
	m_world->addChild(m_light1);                   // attach light to camera
	m_light1->setEnabled(true);                    // enable light source
	m_light1->setDir(-1.0, 0.0, -1.0);             // define the direction of the light beam
	m_light1->m_ambient.set(0.3, 0.3, 0.3);
	m_light1->m_diffuse.set(0.6, 0.6, 0.6);
	m_light1->m_specular.set(0.2, 0.2, 0.2);

	// create a background
	cBackground* background = new cBackground();
	m_camera->m_backLayer->addChild(background);

	// set background properties
	background->setCornerColors(cColorf(1.00, 1.00, 1.00),
		cColorf(1.00, 1.00, 1.0),
		cColorf(0.90, 0.90, 0.90),
		cColorf(0.90, 0.90, 0.90));

	// create a ground
	m_ground = new cMesh();
	m_world->addChild(m_ground);

	cCreatePlane(m_ground, 0.4, 0.3);
	m_ground->m_material->setBlueCyan();
	m_ground->m_material->setStiffness(2000);
	m_ground->m_material->setDynamicFriction(0.2);
	m_ground->m_material->setStaticFriction(0.2);
	m_ground->createAABBCollisionDetector(m_toolRadius);
	m_ground->setShowEnabled(false);

	// create a base
	m_base = new cMultiMesh();
	m_world->addChild(m_base);

	bool fileload = m_base->loadFromFile(RESOURCE_PATH("../resources/models/base/base.obj"));
	if (!fileload)
	{
		fileload = m_base->loadFromFile("C:/Users/master/Desktop/chai3d-3.2.0/modules/ODE/bin/resources/models/base/base.obj");
	}
	if (!fileload)
	{
		printf("Error - 3D Model failed to load correctly.\n");
	}

	m_base->scale(0.003);
	m_base->setShowFrame(false);
	m_base->setShowBoundaryBox(false);
	m_base->setLocalPos(-0.05, 0.0, 0.0);
	m_base->setUseDisplayList(true);

	m_upTarget = new cMesh();
	m_world->addChild(m_upTarget);
	cCreatePlane(m_upTarget, boxSize*1.3, boxSize*1.3, cVector3d(0, 0, upTarget));
	m_upTarget->m_material->setGrayLevel(0.5);
	m_upTarget->setShowEnabled(true);
	m_upTarget->setTransparencyLevel(0.5);
	m_upTarget->setHapticEnabled(false);

	m_downTarget = new cMesh();
	m_world->addChild(m_downTarget);
	cCreatePlane(m_downTarget, boxSize*1.3, boxSize*1.3, cVector3d(0, 0, downTarget));
	m_downTarget->m_material->setGrayLevel(0.5);
	m_downTarget->setShowEnabled(true);
	m_downTarget->setTransparencyLevel(0.5);
	m_downTarget->setHapticEnabled(false);


	m_boundary = new cMesh();
	m_world->addChild(m_boundary);
	cCreatePlane(m_boundary, boxSize*1.3, boxSize*1.3, cVector3d(0, 0, kBoundary));
	m_boundary->m_material->setRed();
	m_boundary->setShowEnabled(false);
	m_boundary->setTransparencyLevel(0.5);
	m_boundary->setHapticEnabled(false);
	/*m_boundary = new cMesh();
	m_world->addChild(m_boundary);
	cCreatePlane(m_boundary, 0.01, 0.01, cVector3d(0, 0, boundary));
	m_boundary->m_material->setRed();
	m_boundary->setShowEnabled(true);
	m_boundary->setTransparencyLevel(0.5);
	m_boundary->setHapticEnabled(false);*/
	// create tools
	m_tools[0] = NULL;
	m_tools[1] = NULL;
	m_tool0 = NULL;
	m_tool1 = NULL;

	m_numTools = cMin(a_numDevices, 2);

	cMesh* mesh = new cMesh();
	cMatrix3d rot;
	rot.identity();
	rot.rotateAboutLocalAxisDeg(cVector3d(1, 0, 0), 90);
	cCreateRing(mesh, 0.001, 0.005, 4, 16, cVector3d(0, 0, 0), rot);
	mesh->m_material->setWhite();
	mesh->setTransparencyLevel(0.4f);

	if (m_numTools > 0)
	{
		m_tool0 = new cToolGripper(m_world);
		m_world->addChild(m_tool0);
		m_tool0->setHapticDevice(shared_ptr<cGenericHapticDevice>(a_hapticDevice0));
		m_tool0->setRadius(m_toolRadius);
		m_tool0->enableDynamicObjects(true);
		m_tool0->setWaitForSmallForce(true);
		m_tool0->start();
		m_tool0->setWaitForSmallForce(true);
		m_tools[0] = m_tool0;
		m_tool0->m_hapticPointFinger->setShow(false, true);
		m_tool0->m_hapticPointThumb->setShow(false, true);
		cMesh* mesh0 = mesh->copy();
		cMesh* mesh00 = mesh->copy();
		m_tool0->m_hapticPointThumb->m_sphereGoal->addChild(mesh0);
		m_tool0->m_hapticPointThumb->m_sphereProxy->addChild(mesh00);

		cMesh* mesh1 = mesh->copy();
		cMesh* mesh11 = mesh->copy();
		m_tool0->m_hapticPointFinger->m_sphereGoal->addChild(mesh1);
		m_tool0->m_hapticPointFinger->m_sphereProxy->addChild(mesh11);
	}
	setOffset(0.1);
	if (m_numTools > 1)
	{
		m_tool1 = new cToolGripper(m_world);
		m_world->addChild(m_tool1);
		m_tool1->setHapticDevice(a_hapticDevice1);
		m_tool1->setRadius(m_toolRadius);
		m_tool1->enableDynamicObjects(true);
		m_tool1->setWaitForSmallForce(true);
		m_tool1->start();
		m_tool1->setWaitForSmallForce(true);
		m_tools[1] = m_tool1;

		cMesh* mesh0 = mesh->copy();
		m_tool1->m_hapticPointThumb->m_sphereProxy->addChild(mesh0);

		cMesh* mesh1 = mesh->copy();
		m_tool1->m_hapticPointFinger->m_sphereProxy->addChild(mesh1);
	}


	meshForceFinger = new cMesh();
	cCreateRing(meshForceFinger, 0.001, 0.005, 4, 16, cVector3d(0, 0, 0), rot);
	meshForceFinger->m_material->setWhite();
	meshForceFinger->setTransparencyLevel(0.4f);
	m_world->addChild(meshForceFinger);
	meshForceFinger->setShowEnabled(false);
	meshForceFinger->setHapticEnabled(false);

	meshFingerSphere = m_tool0->m_hapticPointFinger->m_sphereProxy->copy();
	m_world->addChild(meshFingerSphere);
	meshFingerSphere->setShowEnabled(false);
	meshFingerSphere->setHapticEnabled(false);


	meshForceThumb = new cMesh();
	cCreateRing(meshForceThumb, 0.001, 0.005, 4, 16, cVector3d(0, 0, 0), rot);
	meshForceThumb->m_material->setWhite();
	meshForceThumb->setTransparencyLevel(0.4f);
	m_world->addChild(meshForceThumb);
	meshForceThumb->setShowEnabled(false);
	meshForceThumb->setHapticEnabled(false);

	meshThumbrSphere = m_tool0->m_hapticPointThumb->m_sphereProxy->copy();
	m_world->addChild(meshThumbrSphere);
	meshThumbrSphere->setShowEnabled(false);
	meshThumbrSphere->setHapticEnabled(false);

	// create an ODE world to simulate dynamic bodies
	m_ODEWorld = new cODEWorld(m_world);
	m_world->addChild(m_ODEWorld);

	// set some gravity
	m_ODEWorld->setGravity(cVector3d(0.0, 0.0, -9.81));
	m_ODEWorld->setLinearDamping(0.01);
	m_ODEWorld->setAngularDamping(0.01);

	// we create 6 static walls to contains the 3 cubes within a limited workspace
	m_ODEGPlane0 = new cODEGenericBody(m_ODEWorld);
	m_ODEGPlane1 = new cODEGenericBody(m_ODEWorld);
	m_ODEGPlane2 = new cODEGenericBody(m_ODEWorld);
	m_ODEGPlane3 = new cODEGenericBody(m_ODEWorld);
	m_ODEGPlane4 = new cODEGenericBody(m_ODEWorld);
	m_ODEGPlane5 = new cODEGenericBody(m_ODEWorld);

	double d = 0.14 / 2.0;
	double w = 0.20 / 2.0;
	double h = 0.60 / 2.0;

	m_ODEGPlane0->createStaticPlane(cVector3d(0.0, 0.0, h), cVector3d(0.0, 0.0, -1.0));
	m_ODEGPlane1->createStaticPlane(cVector3d(0.0, 0.0, 0.0), cVector3d(0.0, 0.0, 1.0));
	m_ODEGPlane2->createStaticPlane(cVector3d(0.0, w, 0.0), cVector3d(0.0, -1.0, 0.0));
	m_ODEGPlane3->createStaticPlane(cVector3d(0.0, -w, 0.0), cVector3d(0.0, 1.0, 0.0));
	m_ODEGPlane4->createStaticPlane(cVector3d(d, 0.0, 0.0), cVector3d(-1.0, 0.0, 0.0));
	m_ODEGPlane5->createStaticPlane(cVector3d(-d, 0.0, 0.0), cVector3d(1.0, 0.0, 0.0));

	// create a scope to plot haptic device position data
	scope = new cScope();
	m_camera->m_frontLayer->addChild(scope);
	scope->setSize(300, 200);
	scope->setRange(0.0, 20.0);
	scope->setSignalEnabled(true, true, false, false);
	scope->setShowPanel(true);
	scope->m_colorSignal0.setRedCrimson();
	scope->m_colorSignal1.setGreen();
	scope->m_colorSignal2.setYellow();
	scope->setShowEnabled(false);

	scope2 = new cScope();
	m_camera->m_frontLayer->addChild(scope2);
	scope2->setSize(300, 200);
	scope2->setRange(0.0, 20.0);
	scope2->setSignalEnabled(true, true, true, true);
	scope2->setShowPanel(true);
	scope2->m_colorSignal0.setBlue();
	scope2->m_colorSignal1.setGreen();
	scope2->setShowEnabled(false);

	// create a font
	font = NEW_CFONTCALIBRI72();
	// create a label to display the haptic and graphic rate of the simulation
	labelRates = new cLabel(font);
	m_camera->m_frontLayer->addChild(labelRates);
	labelRates->setShowEnabled(false);
	labelHaptics = new cLabel(font);
	labelHaptics->setShowEnabled(false);
	m_camera->m_frontLayer->addChild(labelHaptics);
	labelTrialInfo = new cLabel(font);
	labelTrialInfo->m_fontColor = cColorf(1.0, 0.0, 0.0);
	m_camera->m_frontLayer->addChild(labelTrialInfo);
	labelTrialInstructions = new cLabel(font);
	labelTrialInstructions->m_fontColor = cColorf(0.0, 0.0, 1.0);
	m_camera->m_frontLayer->addChild(labelTrialInstructions);

	// create a new ODE object that is automatically added to the ODE world
	m_ODEBody1 = new cODEGenericBody(m_ODEWorld);

	// create a virtual mesh  that will be used for the geometry
	// representation of the dynamic body
	cMesh* object1 = new cMesh();

	// crate a cube mesh
	cCreateBox(object1, boxSize, boxSize, boxSize);

	object1->createAABBCollisionDetector(m_toolRadius);

	// define some material properties for each cube
	cMaterial mat;
	mat.setGreen();
	//mat.m_specular.set(0.0, 0.0, 0.0);
	mat.setStiffness(1500);
	mat.setDynamicFriction(0.7);
	mat.setStaticFriction(0.6);
	object1->setMaterial(mat);

	// add mesh to ODE object
	m_ODEBody1->setImageModel(object1);
	m_ODEBody1->m_material->setGreen();

	// create a dynamic model of the ODE object. Here we decide to use a box just like
	// the object mesh we just defined
	m_ODEBody1->createDynamicBox(boxSize, boxSize, boxSize);


	// define some mass properties for each cube

	m_ODEBody1->setMass(0.1);
	m_ODEBody1->setShowEnabled(false);
	// create a line
	line = new cShapeLine(m_ODEGPlane0, m_ODEGPlane1);
	line = new cShapeLine(cVector3d(0.0, 0.0, 0.0), cVector3d(0.0, 0.0, 1.0));

	m_world->addChild(line);

	// set color at each point
	line->m_colorPointA.setGray();
	line->m_colorPointB.setGray();

	////////////////////////////////////////////////////////////////////////////
	// SPRING VISUAL - SPRING
	////////////////////////////////////////////////////////////////////////////

	spring = new cShapeLine(m_ODEGPlane0, m_ODEBody1);
	m_world->addChild(spring);
	//spring->m_material->setBlueAqua();
	spring->setLineWidth(7);
	spring->m_material->setColor(cColorb(1.0, 0.0, 0.0));

	// VISUAL BOX
	// creat visual box
	box = new cShapeBox(boxSize, boxSize, boxSize);
	// define some material properties for each cube
	m_world->addChild(box);
	box->m_material->setBlue();
	box->setFriction(0.5, 0.2);
	box->setHapticEnabled(false);
	box2 = new cShapeBox(boxSize*1.01, boxSize*1.01, boxSize / 10);
	// define some material properties for each cube
	m_world->addChild(box2);
	box2->m_material->setBlack();
	//box->setFriction(0.5, 0.2);
	box->setHapticEnabled(false);
	// initialize
	//init();
	logClock.reset();
	logClock.start();
}


//===========================================================================
/*!
	Update graphics.

	\param  a_width  Width of viewport.
	\param  a_height  Height of viewport.
*/
//===========================================================================
void cTrial::updateGraphics(int a_width, int a_height)
{
	// update haptic and graphic rate data
	labelTrialInstructions->setLocalPos((int)(0.4 * (a_width - labelTrialInfo->getWidth())), 0.8*a_height - labelTrialInfo->getHeight());
	// update position of label
	labelTrialInfo->setLocalPos((int)(0.1 * (a_width - labelTrialInfo->getWidth())), 0.9*a_height - labelTrialInfo->getHeight());
	// update haptic and graphic rate data
	labelRates->setText(cStr(freqCounterGraphics.getFrequency(), 0) + " Hz / " +
		cStr(freqCounterHaptics.getFrequency(), 0) + " Hz");
	// update position of label
	labelRates->setLocalPos((int)(0.5 * (a_width - labelRates->getWidth())), 100);


	// update haptic and graphic rate data
	labelRates->setText(cStr(freqCounterGraphics.getFrequency(), 0) + " Hz / " +
		cStr(freqCounterHaptics.getFrequency(), 0) + " Hz");
	// update position of label
	labelRates->setLocalPos((int)(0.5 * (a_width - labelRates->getWidth())), 100);

	string controlStr;
	if (forceControl)
		controlStr = "force control";
	else
		controlStr = "position control";

	labelHaptics->setText(cStr((double const)kHandle, 0) + " kHandle [N/m] \n " +
		cStr((double const)kSpring, 0) + " kSpring [N/m] \n" + controlStr);


	// update position of label
	labelHaptics->setLocalPos((int)(0.65 * (a_width - labelRates->getWidth())), 200);
	// update value of scope
	scope->setSignalValues(-m_tool0->getDeviceGlobalForce().z(), m_ODEBody1->getGlobalPos().z()*kSpring);
	scope2->setSignalValues(-m_tool0->getDeviceGlobalForce().z(), m_tool0->getGripperForce(), 5.0, 12.0);
	//cout << "k: " << m_ODEBody1->getImageModel()->m_material->getStiffness() << " gap: " << (m_tool0->m_hapticPointFinger->getLocalPosProxy() - m_tool0->m_hapticPointFinger->getLocalPosGoal()).length() << endl;

	if (kVisual == 2 && flagChangeBox)
	{
		box->setSize(boxSize, gap, boxSize);
		box2->setSize(boxSize*1.01, gap*1.01, boxSize / 10);
		if (forceControl)
		{
			meshForceThumb->setShowEnabled(true);
			meshForceFinger->setShowEnabled(true);
			meshThumbrSphere->setShowEnabled(true);
			meshFingerSphere->setShowEnabled(true);
			m_tool0->m_hapticPointFinger->m_sphereGoal->setShowEnabled(false);
			m_tool0->m_hapticPointThumb->m_sphereGoal->setShowEnabled(false);
		}
		else
		{
			meshForceThumb->setShowEnabled(false);
			meshForceFinger->setShowEnabled(false);
			meshThumbrSphere->setShowEnabled(false);
			meshFingerSphere->setShowEnabled(false);
		}
	}
	else
	{
		if (forceControl)
		{
			meshForceThumb->setShowEnabled(false);
			meshForceFinger->setShowEnabled(false);
			meshThumbrSphere->setShowEnabled(false);
			meshFingerSphere->setShowEnabled(false);

		}
		box->setSize(boxSize, boxSize, boxSize);
		box2->setSize(boxSize*1.01, boxSize*1.01, boxSize / 10);
	}

	if (!meshForceThumb->getShowEnabled() && kVisual == 2)
	{
		m_tool0->m_hapticPointFinger->m_sphereGoal->setShowEnabled(true);
		m_tool0->m_hapticPointThumb->m_sphereGoal->setShowEnabled(true);
	}



	// update shadow maps (if any)
	spring->m_colorPointA = cColorf(1 - (m_ODEBody1->getGlobalPos().z()) * 10, 0.0, 0.0);
	spring->m_colorPointB = cColorf(1 - (m_ODEBody1->getGlobalPos().z()) * 10, 0.0, 0.0);


	// update shadow maps (if any)
	m_world->updateShadowMaps(false, m_mirroredDisplay);

	// render view
	m_camera->renderView(a_width, a_height);
}

void cTrial::loadTrial()
{

	ifstream visualCondition("visualForce.txt");
	if (!visualCondition) {
		cout << "Unable to open file";
		exit(1); // terminate with error
	}
	int i = 0;
	while (visualCondition >> visualCond[i]) {
		//cout << visualCond[i] << " ";
		i += 1;
	}
	cout << endl;
	//delete &visualCondition;
	ifstream handleCondition("handleForce.txt");
	if (!handleCondition) {
		cout << "Unable to open file";
		exit(1); // terminate with error
	}
	i = 0;
	while (handleCondition >> handleCond[i]) {
		//cout << handleCond[i] << " ";
		i += 1;
	}
	cout << endl;
	//delete handleCondition;
	ifstream springCondition("springForce.txt");
	if (!springCondition) {
		cout << "Unable to open file";
		exit(1); // terminate with error
	}
	i = 0;
	while (springCondition >> springCond[i]) {
		//cout << springCond[i] << " ";
		i += 1;
	}
	cout << endl;
	//delete &springCondition;
	cout << "Trial Loaded" << endl << endl;
}


//===========================================================================
/*!
	Update haptics.
*/
//===========================================================================
void cTrial::updateHaptics()
{
	// compute global reference frames for each object
	freqCounterHaptics.signal(1);
	m_world->computeGlobalPositions(true);

	// update positions

	m_tool0->updateFromDevice();
	cMatrix3d rot = m_tool0->getDeviceGlobalRot();
	m_tool0->m_hapticPointFinger->m_sphereGoal->setLocalRot(rot);
	m_tool0->m_hapticPointThumb->m_sphereGoal->setLocalRot(rot);
	m_tool0->m_hapticPointFinger->m_sphereProxy->setLocalRot(rot);
	m_tool0->m_hapticPointThumb->m_sphereProxy->setLocalRot(rot);
	meshForceThumb->setLocalRot(rot);
	meshForceFinger->setLocalRot(rot);

	// compute interaction forces
	for (int i = 0; i < m_numTools; i++)
	{
		m_tools[i]->computeInteractionForces();
	}

	// apply forces to haptic devices
	for (int i = 0; i < m_numTools; i++)
	{
		/*if (gap<boxSize & m_tools[i]->getGripperForce()>0)
		m_tools[i]->setGripperForce(m_ODEBody1->getImageModel()->m_material->getStiffness()*(boxSize-gap));*/
		//m_tools[i]->setGripperForce(0.0); 
		/*	if(((m_ODEBody1->getLocalPos().z()) - boxSize / 2)>0)
			m_tools[i]->setDeviceGlobalForce(cVector3d(0.0, 0.0, -(m_ODEBody1->getGlobalPos().z() - boxSize / 2) * 30));*/
		if (m_tools[i]->getGripperForce() > 0)
		{
			gripForce = kHandle*((-m_tool0->m_hapticPointThumb->getLocalPosProxy().y() + m_tool0->m_hapticPointThumb->getLocalPosGoal().y()) + (m_tool0->m_hapticPointFinger->getLocalPosProxy().y() - m_tool0->m_hapticPointFinger->getLocalPosGoal().y()));
				m_tools[i]->setGripperForce(gripForce);
		}
		else
			gripForce = 0;
		if (flagLoad)
		{

			//m_tools[i]->setGripperForce(kHandle*(boxSize- fabs((m_tool0->m_hapticPointThumb->getLocalPosGoal() - m_tool0->m_hapticPointFinger->getLocalPosGoal()).length())));

			m_tools[i]->addDeviceGlobalForce(0.0, 0.0, loadForce - m_tool0->getDeviceGlobalForce().z());
		}
		m_tools[i]->applyToDevice();
	}
	flagLoad = false;
	loadForce = 0;
	// apply forces to ODE objects
	for (int i = 0; i < m_numTools; i++)
	{
		// for each interaction point of the tool we look for any contact events
		// with the environment and apply forces accordingly
		int numInteractionPoints = m_tools[i]->getNumHapticPoints();
		int nContact = 0;
		for (int j = 0; j < numInteractionPoints; j++)
		{
			// get pointer to next interaction point of tool
			cHapticPoint* interactionPoint = m_tools[i]->getHapticPoint(j);

			// check all contact points
			int numContacts = interactionPoint->getNumCollisionEvents();
			for (int k = 0; k < numContacts; k++)
			{
				cCollisionEvent* collisionEvent = interactionPoint->getCollisionEvent(k);

				// given the mesh object we may be touching, we search for its owner which
				// could be the mesh itself or a multi-mesh object. Once the owner found, we
				// look for the parent that will point to the ODE object itself.
				cGenericObject* object = collisionEvent->m_object->getOwner()->getOwner();

				// cast to ODE object
				cODEGenericBody* ODEobject = dynamic_cast<cODEGenericBody*>(object);

				// if ODE object, we apply interaction forces
				if (ODEobject != NULL)
				{
					/*if (interactionPoint->getLastComputedForce().length() == 0)
					ODEobject->addExternalForceAtPoint(cVector3d(0,0,1),
													 collisionEvent->m_globalPos);*/
													 //else
					ODEobject->addExternalForceAtPoint(-0.3*interactionPoint->getLastComputedForce(),
						collisionEvent->m_globalPos);
					/*const dReal * f;
					f = ODEobject->getForce();
					ODEobject->addExternalForceAtPoint(-0.3*cVector3d(f[0],f[1],f[2]),collisionEvent->m_globalPos);*/

					if (ODEobject == m_ODEBody1)
						nContact += 1;

					/*if (m_ODEBody1->getLocalPos().z() > kBoundary)
						flagLoad = true;*/

				}
			}

		}
		if (nContact == 2)
		{
			/*if (forceControl)
				m_tool0->setGripperWorkspaceScale(m_ODEBody1->m_material->getStiffness() / kHandle);
			gap = (m_tool0->m_hapticPointFinger->m_sphereGoal->getLocalPos() - m_tool0->m_hapticPointThumb->m_sphereGoal->getLocalPos()).length();*/

			if (!forceControl)
			{
				gap = (m_tool0->m_hapticPointFinger->m_sphereGoal->getLocalPos() - m_tool0->m_hapticPointThumb->m_sphereGoal->getLocalPos()).length();
				meshForceThumb->setLocalPos(m_tool0->m_hapticPointThumb->m_sphereGoal->getGlobalPos());
				meshForceFinger->setLocalPos(m_tool0->m_hapticPointFinger->m_sphereGoal->getGlobalPos());

			}
			else
			{
				gap = (m_tool0->m_hapticPointFinger->m_sphereGoal->getLocalPos() - m_tool0->m_hapticPointThumb->m_sphereGoal->getLocalPos()).length();
				gap = boxSize + m_ODEBody1->m_material->getStiffness() / kHandle *(gap - boxSize);
				meshForceThumb->setLocalPos(cVector3d(m_tool0->m_hapticPointThumb->m_sphereGoal->getLocalPos().x(), m_ODEBody1->getLocalPos().y() - gap / 2, m_tool0->m_hapticPointThumb->m_sphereProxy->getLocalPos().z() + 0.1));
				meshForceFinger->setLocalPos(cVector3d(m_tool0->m_hapticPointFinger->m_sphereGoal->getLocalPos().x(), m_ODEBody1->getLocalPos().y() + gap / 2, m_tool0->m_hapticPointFinger->m_sphereProxy->getLocalPos().z() + 0.1));

				//	meshForceFinger->setLocalPos(m_tool0->m_hapticPointFinger->m_sphereGoal->getGlobalPos().x(), gap / 2 + m_ODEBody1->getLocalPos().y(), m_tool0->m_hapticPointFinger->m_sphereProxy->getLocalPos().z()+ 0.1);
				meshThumbrSphere->setLocalPos(meshForceThumb->getLocalPos());
				meshFingerSphere->setLocalPos(meshForceFinger->getLocalPos());
				//cout << m_tool0->m_hapticPointThumb->m_sphereProxy->getLocalPos().z() << " " << m_tool0->m_hapticPointThumb->m_sphereProxy->getLocalPos().z() << "\r";
			}
			//	if (gap < boxSize)
			flagLoad = true;
		}
		else
		{
			meshForceThumb->setLocalPos(m_tool0->m_hapticPointThumb->m_sphereGoal->getLocalPos());
			meshForceFinger->setLocalPos(m_tool0->m_hapticPointFinger->m_sphereGoal->getLocalPos());
			//m_tool0->setGripperWorkspaceScale(1);
			gap = boxSize;
			//m_ODEBody1->setLocalPos(cVector3d(0.0, 0.0, boxSize / 2));
		}

	}
	if (gap < boxSize & kVisual == 2)
		flagChangeBox = true;
	else
		flagChangeBox = false;
	//scout << m_ODEBody1->getLocalPos().z() << "\r";
	m_ODEBody1->setLocalPos(0.0, 0.0, m_ODEBody1->getLocalPos().z());
	rot.identity();
	m_ODEBody1->setLocalRot(rot);
	//if (!flagLoad)
		// retrieve simulation time and compute next interval
	box->setLocalPos(m_ODEBody1->getLocalPos());
	box->setLocalRot(m_ODEBody1->getLocalRot());
	box2->setLocalPos(m_ODEBody1->getLocalPos());
	box2->setLocalRot(m_ODEBody1->getLocalRot());
	double time = simClock.getCurrentTimeSeconds();
	double nextSimInterval = cClamp(time, 0.0001, 0.001);

	// reset clock
	simClock.reset();
	simClock.start();
	if (flagLoad)
	{
		loadForce -= (m_ODEBody1->getGlobalPos().z() - kBoundary)* kSpring;
		if (loadForce > 0)
			loadForce = 0;
		//m_ODEBody1->addExternalForceAtPoint(cVector3d(0.0, 0.0, -0 * (m_ODEBody1->getLocalPos().z()) - boxSize / 2), m_ODEBody1->getLocalPos());
	}
	//loadForce -= (m_ODEBody1->getGlobalPos().z())* kSpring;

	// update simulation
	m_ODEWorld->updateDynamics(nextSimInterval);
}


//===========================================================================
/*!
	Set device offset.
*/
//===========================================================================
void cTrial::setOffset(double a_offset)
{
	if (m_tool0 != NULL)
	{
		cVector3d pos(0.0, 0.0, a_offset);
		m_tool0->setLocalPos(pos);
	}
	if (m_tool1 != NULL)
	{
		cVector3d pos(0.0, 0.0, a_offset);
		m_tool1->setLocalPos(pos);
	}
}


//===========================================================================
/*!
	Set torque gain.
*/
//===========================================================================
void cTrial::setTorqueGain(double a_torqueGain)
{
	m_torqueGain = a_torqueGain;
}
void cTrial::initTrial()
{
	// set starting position of cube and its color
	labelTrialInfo->setText(cStr(trialNumber + 1));
	cVector3d tmpvct;
	m_ODEBody1->setLocalPos(0.00, 0.00, boxSize / 2);
	box->m_material->setRed();
	m_ODEBody1->m_imageModel->m_material->setRed();
	//m_ODEBody1->m_imageModel->m_material->setRed();
	// set trial properties
	kSpring = springCond[trialNumber];
	kHandle = handleCond[trialNumber];
	m_ODEBody1->m_material->setStiffness(1000);
	kVisual = visualCond[trialNumber];
	kBoundary = boxSize / 2;
	kSpring = springCond[trialNumber];
	m_boundary->setShowEnabled(false);

	if (kVisual == 1) // rigid cube
	{
		m_tool0->m_hapticPointFinger->setShow(true, false);
		m_tool0->m_hapticPointThumb->setShow(true, false);
		box->setShowEnabled(true);
		box2->setShowEnabled(true);
		m_ODEBody1->setShowEnabled(false);
	}
	else if (kVisual == 2) // soft cube
	{
		m_tool0->m_hapticPointFinger->setShow(false, true);
		m_tool0->m_hapticPointThumb->setShow(false, true);
		m_ODEBody1->setShowEnabled(false);
		box->setShowEnabled(true);
		box2->setShowEnabled(true);
	}
}
void cTrial::updateProtocol()
{
	//if (expState < 3)
	//	cout << "exp: " << expState << " trial: " << trialState << "\r";

	switch (expState)
	{
	case 1: // Setup Next Trial
		if (trialNumber < 10)
			labelTrialInstructions->setShowEnabled(true);
		if (trialNumber > 10)
			labelTrialInstructions->setShowEnabled(false);
		if (!appendToFile)
		{
			labelTrialInstructions->setText("Align the middle\n of the cube with\n the bottom target");
		}
		initTrial();
		cout << endl << "starting Trial: " << trialNumber + 1 << endl << endl;
		expState += 1;

		break;
	case 2: // wait for subject to reach starting pos
		if (fabs(m_ODEBody1->getLocalPos().z() - downTarget) < 0.01)
			expState += 1;
		break;
	case 3: // wait for subject to reach stay at starting pos for a random time between 100-200 milliseconds
	{
		//cout << "------" << endl;
		labelTrialInstructions->setText("Get Ready");

		double waittime = (rand() % 100 + 101) / 100;
		double time0 = logClock.getCurrentTimeSeconds();
		while ((logClock.getCurrentTimeSeconds() - time0) < waittime)
		{
			cout << "now: " << logClock.getCurrentTimeSeconds() << " t0: " << time0 << " wait: " << waittime << "\r";
		}
		if (fabs(m_ODEBody1->getLocalPos().z() - downTarget) < 0.005)
		{
			box->m_material->setGreen();
			m_ODEBody1->m_imageModel->m_material->setGreen();
			loggingRunning = true;
			cout << "start logging" << endl << endl;
			expState += 1; //start trial
			trialState = 1;
			liftingNumber = 0;
			if (kVisual == 3) // rigid cube
			{
				m_tool0->m_hapticPointFinger->setShow(false, false);
				m_tool0->m_hapticPointThumb->setShow(false, false);
				box->setShowEnabled(false);
				box2->setShowEnabled(false);

				//m_ODEBody1->setShowEnabled(false);
			}
		}
		else
			expState -= 1; // go back to case 2 waiting...
		break;
	}
	case 4: // start the trial + trial running
			//cout << "------" << endl;

		if (liftingNumber == 0)
			labelTrialInstructions->setText("GO");
		if (liftingNumber > 4 && trialState != 3) // show the button for next trial
		{
			cout << 100 << endl << endl;
			if (kVisual == 3)
			{
				box->setShowEnabled(true);
				box2->setShowEnabled(true);
			}
			//expState += 1;
			trialState = 3;
			box->m_material->setBlueNavy();
			m_ODEBody1->m_material->setBlueNavy();
			box->m_material->setBlueNavy();
			m_ODEBody1->m_material->setBlueNavy();
		}
		else
			if (!forceControl)
			{
				if (m_ODEBody1->getLocalPos().z() < boxSize / 2 + 0.0003 && trialState != 3)
				{
					expState = 1;
					//loggingThread->stop();
					//delete &loggingThread;
					loggingRunning = false;
					appendToFile = true;
					cout << "stop logging" << endl;
					labelTrialInstructions->setShowEnabled(true);
					labelTrialInstructions->setText("Cube Slipped\n Start over");
					//copy the data file and call it bad at the end;
				}
				else if (gap < boxSize / 5 && trialState != 3)
				{
					expState = 1;
					//loggingThread->stop();
					//delete &loggingThread;
					loggingRunning = false;
					appendToFile = true;
					cout << "stop logging" << endl;
					labelTrialInstructions->setShowEnabled(true);
					labelTrialInstructions->setText("Cube Broke\n Start over");
					//copy the data file and call it bad at the end;
				}
			}
			else
			{
				if (m_ODEBody1->getLocalPos().z() < boxSize / 2 + 0.0003 && trialState != 3)
				{
					expState = 1;
					//loggingThread->stop();
					//delete &loggingThread;
					loggingRunning = false;
					appendToFile = true;
					cout << "stop logging" << endl;
					labelTrialInstructions->setText("Cube Slipped\n Start over");
					//copy the data file and call it bad at the end;
				}
				else if (gripForce > 10 && trialState != 3)
				{
					expState = 1;
					//loggingThread->stop();
					//delete &loggingThread;
					loggingRunning = false;
					appendToFile = true;
					cout << "stop logging" << endl;
					labelTrialInstructions->setText("Cube Broke\n too much force\n  Start over");
					//copy the data file and call it bad at the end;
				}
			}
		switch (trialState)
		{
		case 1: //midlift
			if (m_ODEBody1->getLocalPos().z() > upTarget)
				trialState += 1;
			break;
		case 2:
		{
			if (m_ODEBody1->getLocalPos().z() < downTarget)
			{
				trialState -= 1;
				liftingNumber += 1;
				labelTrialInstructions->setText(cStr(liftingNumber));
				cout << endl << "lift: " << liftingNumber << endl;
			}
			break;
		}
		case 3:
		{
			if (m_ODEBody1->getLocalPos().z() < boxSize / 2 + 0.0003)
				expState += 1;
			labelTrialInstructions->setText("Let go of the Cube");
			break;
		}
		}
		break;
	case 5:
		trialNumber += 1;
		expState = 1;
		loggingRunning = false;
		appendToFile = false;
		break;
	}
}
void cTrial::updateLogging(void)
{
	string filename;
	char trialString[3];
	double time;
	itoa(trialNumber + 1, trialString, 10);
	filename = +"dataLogs\\" + subjectName + "_trial_" + trialString + ".txt";
	/*sprintf(filename, sprintf("%c_trial_%i.txt",subjectName,trialNumber)*/
	std::ofstream trialFile;
	if (appendToFile)
		trialFile.open(filename, std::ios_base::app);
	else
	{
		trialFile.open(filename);
		trialFile << "Time\t"
			<< "finger_proxy_x\t" << "finger_proxy_x\t" << "finger_proxy_x\t"
			<< "finger_goal_x\t" << "finger_goal_y\t" << "finger_goal_z\t"
			<< "thumb_proxy_x\t" << "thumb_proxy_y\t" << "thumb_proxy_z\t"
			<< "thumb_goal_x\t" << "thumb_goal_y\t" << "thumb_goal_z\t"
			<< "cube_x\t" << "cube_y\t" << "cube_z\t"
			<< "force_x\t" << "force_y\t" << "force_z\t" << "force_grip\t"
			<< "lift_number\t" << "kSpring\t" << "kHandle\t" << "kBoundary\t" << "kVisual" << endl;
		logClock.reset();
		logClock.start();
	}

	time = logClock.getCurrentTimeSeconds();
	double oldtime = 0;
	while (loggingRunning)
	{
		do
		{
			time = logClock.getCurrentTimeSeconds();
		} while ((time - oldtime) < (1 / fs));
		oldtime = time;
		trialFile << std::fixed << time << "\t"
			<< m_tool0->m_hapticPointFinger->m_sphereProxy->getLocalPos().x() << "\t" << m_tool0->m_hapticPointFinger->m_sphereProxy->getLocalPos().y() << "\t" << m_tool0->m_hapticPointFinger->m_sphereProxy->getLocalPos().z() << "\t"
			<< m_tool0->m_hapticPointFinger->m_sphereGoal->getLocalPos().x() << "\t" << m_tool0->m_hapticPointFinger->m_sphereGoal->getLocalPos().y() << "\t" << m_tool0->m_hapticPointFinger->m_sphereGoal->getLocalPos().z() << "\t"
			<< m_tool0->m_hapticPointThumb->m_sphereProxy->getLocalPos().x() << "\t" << m_tool0->m_hapticPointThumb->m_sphereProxy->getLocalPos().y() << "\t" << m_tool0->m_hapticPointThumb->m_sphereProxy->getLocalPos().z() << "\t"
			<< m_tool0->m_hapticPointThumb->m_sphereGoal->getLocalPos().x() << "\t" << m_tool0->m_hapticPointThumb->m_sphereGoal->getLocalPos().y() << "\t" << m_tool0->m_hapticPointThumb->m_sphereGoal->getLocalPos().z() << "\t"
			<< m_ODEBody1->getLocalPos().x() << "\t" << m_ODEBody1->getLocalPos().y() << "\t" << m_ODEBody1->getLocalPos().z() << "\t"
			<< m_tool0->getDeviceGlobalForce().x() << "\t" << m_tool0->getDeviceGlobalForce().y() << "\t" << m_tool0->getDeviceGlobalForce().z() << "\t" << m_tool0->getGripperForce() << "\t"
			<< liftingNumber << "\t" << kSpring << "\t" << kHandle << "\t" << kBoundary << "\t" << kVisual << endl;
	}
	trialFile.close();
}