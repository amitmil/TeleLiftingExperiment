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
    \version   3.2.0 $Rev: 2007 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "chai3d.h"
//------------------------------------------------------------------------------
#include <GLFW/glfw3.h>
//---------------------------------------------------------------------------
using namespace chai3d;
using namespace std;
//---------------------------------------------------------------------------
#include "CODE.h"
#include "cTrial.h"
#include "CDemo1.h"
#include "CDemo2.h"
#include "CDemo3.h"
#include "CDemo4.h"
//---------------------------------------------------------------------------

//------------------------------------------------------------------------------
// GENERAL SETTINGS
//------------------------------------------------------------------------------

// stereo Mode
/*
    C_STEREO_DISABLED:            Stereo is disabled
    C_STEREO_ACTIVE:              Active stereo for OpenGL NVDIA QUADRO cards
    C_STEREO_PASSIVE_LEFT_RIGHT:  Passive stereo where L/R images are rendered next to each other
    C_STEREO_PASSIVE_TOP_BOTTOM:  Passive stereo where L/R images are rendered above each other
*/
cStereoMode stereoMode = C_STEREO_PASSIVE_LEFT_RIGHT;

// fullscreen mode
bool fullscreen = false;

// mirrored display
bool mirroredDisplay = false;


//---------------------------------------------------------------------------
// CHAI3D VARIABLES
//---------------------------------------------------------------------------

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
shared_ptr<cGenericHapticDevice> m_hapticDevice0;
shared_ptr<cGenericHapticDevice> m_hapticDevice1;


//---------------------------------------------------------------------------
// DEMOS
//---------------------------------------------------------------------------

//! currently active camera
cTrial* m_trial;



//---------------------------------------------------------------------------
// GENERAL VARIABLES
//---------------------------------------------------------------------------

// flag to indicate if the haptic simulation currently running
bool simulationRunning = false;

// flag to indicate if the haptic simulation has terminated
bool simulationFinished = true;

// haptic thread
cThread* hapticsThread;
// logging thread
cThread* loggingThread;
// protocol thread
cThread* protocolThread;
// a handle to window display context
GLFWwindow* window = NULL;

// current width of window
int width = 0;

// current height of window
int height = 0;

// swap interval for the display context (vertical synchronization)
int swapInterval = 1;


//---------------------------------------------------------------------------
// DECLARED FUNCTIONS
//---------------------------------------------------------------------------

// callback when the window display is resized
void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height);

// callback when an error GLFW occurs
void errorCallback(int error, const char* a_description);

// callback when a key is pressed
void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods);

// this function renders the scene
void updateGraphics(void);

// this function contains the main haptics simulation loop
void updateHaptics(void);
// this function contains the main logging loop
void updateLogging(void);
// this function contains the main protocol loop
void updateProtocol(void);
//void startLogging(void);
// this function closes the application
void close(void);

// initialize demos
void initDemo1();



//===========================================================================
/*
    DEMO:    06-ODE-explorations.cpp

    This example illustrates the use of the ODE framework for simulating
    haptic interaction with dynamic bodies. 
 */
//===========================================================================

int main(int argc, char* argv[])
{
    //-----------------------------------------------------------------------
    // INITIALIZATION
    //-----------------------------------------------------------------------

    cout << endl;
    cout << "-----------------------------------" << endl;
    cout << "Tele Lifting Experiment" << endl;
    cout << "Written By:" << endl;
    cout << "Amit Milstein" << endl;
	cout << "Last Update:" << endl;
	cout << "18/06/2018:" << endl;
    cout << "-----------------------------------" << endl << endl << endl;
    cout << "Keyboard Options:" << endl << endl;
    cout << "[1] - Select Demo 1" << endl;
    cout << "[q] - Exit application" << endl;
	cout << "-----------------------------------" << endl;
    cout << endl << endl;

    // parse first arg to try and locate resources
    string resourceRoot = string(argv[0]).substr(0,string(argv[0]).find_last_of("/\\")+1);


    //--------------------------------------------------------------------------
    // OPEN GL - WINDOW DISPLAY
    //--------------------------------------------------------------------------

    // initialize GLFW library
    if (!glfwInit())
    {
        cout << "failed initialization" << endl;
        cSleepMs(1000);
        return 1;
    }

    // set error callback
    glfwSetErrorCallback(errorCallback);

    // compute desired size of window
    const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
    int w = 0.8 * mode->height;
    int h = 0.5 * mode->height;
    int x = 0.5 * (mode->width - w);
    int y = 0.5 * (mode->height - h);

    // set OpenGL version
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

    // set active stereo mode
    if (stereoMode == C_STEREO_ACTIVE)
    {
        glfwWindowHint(GLFW_STEREO, GL_TRUE);
    }
    else
    {
        glfwWindowHint(GLFW_STEREO, GL_FALSE);
    }

    // create display context
    window = glfwCreateWindow(w, h, "CHAI3D", NULL, NULL);
    if (!window)
    {
        cout << "failed to create window" << endl;
        cSleepMs(1000);
        glfwTerminate();
        return 1;
    }

    // get width and height of window
    glfwGetWindowSize(window, &width, &height);

    // set position of window
    glfwSetWindowPos(window, x, y);

    // set key callback
    glfwSetKeyCallback(window, keyCallback);

    // set resize callback
    glfwSetWindowSizeCallback(window, windowSizeCallback);

    // set current display context
    glfwMakeContextCurrent(window);

    // sets the swap interval for the current display context
    glfwSwapInterval(swapInterval);

    // initialize GLEW library
#ifdef GLEW_VERSION
    if (glewInit() != GLEW_OK)
    {
        cout << "failed to initialize GLEW library" << endl;
        glfwTerminate();
        return 1;
    }
#endif


    //-----------------------------------------------------------------------
    // HAPTIC DEVICES 
    //-----------------------------------------------------------------------

    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // get number of haptic devices
    int numDevices = handler->getNumDevices();

    // default stiffness of scene objects
    double maxStiffness = 2000.0;

    // get access to the haptic devices found
    if (numDevices > 0)
    {
        handler->getDevice(m_hapticDevice0, 0);
        maxStiffness = cMin(maxStiffness, 0.5 * m_hapticDevice0->getSpecifications().m_maxLinearStiffness);
    }

    if (numDevices > 1)
    {
        handler->getDevice(m_hapticDevice1, 1);
        maxStiffness = cMin(maxStiffness, 0.5 * m_hapticDevice1->getSpecifications().m_maxLinearStiffness);
    }

 
    //-----------------------------------------------------------------------
    // DEMOS
    //-----------------------------------------------------------------------

    // setup demos
    m_trial = new cTrial(resourceRoot, numDevices, m_hapticDevice0, m_hapticDevice1);

	cout << "Please enter Subject's Name:" << endl;
	cin >> m_trial->subjectName;
	cout << endl << endl;
//	m_trial->subjectName = "testing";

    // set stereo mode
    m_trial->m_camera->setStereoMode(stereoMode);

    // set object stiffness in demos
    //m_trial->setStiffness(maxStiffness);
    

    // initialize demo 1
    initDemo1();


    //-----------------------------------------------------------------------
    // START SIMULATION
    //-----------------------------------------------------------------------

    // create a thread which starts the main haptics rendering loop
    hapticsThread = new cThread();
    hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);

	// This is to periodically log data (time, position, force, state) on a buffer
	loggingThread = new cThread(); // This is to periodically log data (time, position, force, state) on a buffer
	loggingThread->start(updateLogging, CTHREAD_PRIORITY_HAPTICS);

	protocolThread = new cThread(); // This is to periodically log data (time, position, force, state) on a buffer
	protocolThread->start(updateProtocol, CTHREAD_PRIORITY_HAPTICS);
    // setup callback when application exits
    atexit(close);


    //--------------------------------------------------------------------------
    // MAIN GRAPHIC LOOP
    //--------------------------------------------------------------------------

    // call window size callback at initialization
    windowSizeCallback(window, width, height);

    // main graphic loop
    while (!glfwWindowShouldClose(window))
    {
        // get width and height of window
        glfwGetWindowSize(window, &width, &height);

        // render graphics
        updateGraphics();

        // swap buffers
        glfwSwapBuffers(window);

        // process events
        glfwPollEvents();
		// signal frequency counter
		m_trial->freqCounterGraphics.signal(1);
    }

    // close window
    glfwDestroyWindow(window);

    // terminate GLFW library
    glfwTerminate();

    // exit
    return 0;
}

//---------------------------------------------------------------------------

void initDemo1()
{

  //  m_trial->init();
}

//---------------------------------------------------------------------------



//---------------------------------------------------------------------------

void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height)
{
    // update window size
    width  = a_width;
    height = a_height;
	// update position of scope
	m_trial->scope->setLocalPos((0.05 * (width - m_trial->scope->getWidth())), 120);
	m_trial->scope2->setLocalPos((0.05 * (width - m_trial->scope->getWidth())), 400);
}

//------------------------------------------------------------------------------

void errorCallback(int a_error, const char* a_description)
{
    cout << "Error: " << a_description << endl;
}

//---------------------------------------------------------------------------

void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods)
{
	// filter calls that only include a key press
	if ((a_action != GLFW_PRESS) && (a_action != GLFW_REPEAT))
	{
		return;
	}

	// option - exit
	else if ((a_key == GLFW_KEY_ESCAPE) || (a_key == GLFW_KEY_Q))
	{
		glfwSetWindowShouldClose(a_window, GLFW_TRUE);
	}

	// option - start demo 1
	else if (a_key == GLFW_KEY_1)
	{
		initDemo1();
	}


	// option - toggle fullscreen
	else if (a_key == GLFW_KEY_F)
	{
		// toggle state variable
		fullscreen = !fullscreen;

		// get handle to monitor
		GLFWmonitor* monitor = glfwGetPrimaryMonitor();

		// get information about monitor
		const GLFWvidmode* mode = glfwGetVideoMode(monitor);

		// set fullscreen or window mode
		if (fullscreen)
		{
			glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
			glfwSwapInterval(swapInterval);
		}
		else
		{
			int w = 0.8 * mode->height;
			int h = 0.5 * mode->height;
			int x = 0.5 * (mode->width - w);
			int y = 0.5 * (mode->height - h);
			glfwSetWindowMonitor(window, NULL, x, y, w, h, mode->refreshRate);
			glfwSwapInterval(swapInterval);
		}
	}

	// option - toggle vertical mirroring
	else if (a_key == GLFW_KEY_M)
	{
		mirroredDisplay = !mirroredDisplay;
		m_trial->m_camera->setMirrorVertical(mirroredDisplay);

		m_trial->m_mirroredDisplay = mirroredDisplay;

	}
	else if (a_key == GLFW_KEY_KP_ADD)
		m_trial->kHandle += 50;
	else if (a_key == GLFW_KEY_KP_SUBTRACT)
		m_trial->kHandle -= 50;
	else if (a_key == GLFW_KEY_PAGE_DOWN)
		m_trial->kSpring -= 50;
	else if (a_key == GLFW_KEY_PAGE_UP)
		m_trial->kSpring += 50;

	else if (a_key == GLFW_KEY_INSERT)
	{
		m_trial->scope->setShowEnabled(false);
		m_trial->scope2->setShowEnabled(false);
		m_trial->labelHaptics->setShowEnabled(false);
		/*m_trial->flagVisual = !m_trial->flagVisual;
		m_trial->m_tool0->m_hapticPointFinger->setShow(!m_trial->flagVisual, m_trial->flagVisual);
		m_trial->m_tool0->m_hapticPointThumb->setShow(!m_trial->flagVisual, m_trial->flagVisual);*/
	}
	else if (a_key == GLFW_KEY_DELETE)
	{
		m_trial->scope->setShowEnabled(true);
		m_trial->scope2->setShowEnabled(true);
		m_trial->labelHaptics->setShowEnabled(true);
	/*m_trial->flagBox = !m_trial->flagBox;
	m_trial->m_tool0->m_hapticPointFinger->setShow(!m_trial->flagVisual &  m_trial->flagBox, m_trial->flagVisual & m_trial->flagBox);
	m_trial->m_tool0->m_hapticPointThumb->setShow(!m_trial->flagVisual &  m_trial->flagBox, m_trial->flagVisual & m_trial->flagBox);
	m_trial->box->setShowEnabled(m_trial->flagBox);*/
	}
	else if (a_key == GLFW_KEY_5)
	{
	
	



	}
}

//---------------------------------------------------------------------------

void close(void)
{
    // stop the simulation
    simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!simulationFinished) { cSleepMs(100); }

    // delete resources
    delete hapticsThread;
    delete m_trial;
    delete handler;
}

//---------------------------------------------------------------------------

void updateGraphics(void)
{

    // render world
    m_trial->updateGraphics(width, height);

    // wait until all GL commands are completed
    glFinish();

    // check for any OpenGL errors
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) printf("Error:  %s\n", gluErrorString(err));
}

//---------------------------------------------------------------------------

void updateHaptics(void)
{
    // simulation in now running
    simulationRunning = true;
    simulationFinished = false;

    // main haptic simulation loop
    while(simulationRunning)
    {
        m_trial->updateHaptics();
    }

    // shutdown haptic devices
    if (m_trial->m_tool0 != NULL)
    {
        m_trial->m_tool0->stop();
    }
    if (m_trial->m_tool1 != NULL)
    {
        m_trial->m_tool1->stop();
    }

    // exit haptics thread
    simulationFinished = true;
}
void updateLogging(void)
{
	while (simulationRunning)
		// This is to periodically log data (time, position, force, state) on a buffer
		if (m_trial->loggingRunning)
		{
			cout << "start logging" << endl;
			m_trial->updateLogging();
		}
}
void updateProtocol(void)
{
	while (simulationRunning)
		m_trial->updateProtocol();
}

