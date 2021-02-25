/////////////////////////////////////////////////////////////////////////////////////////////
//
// FastLogger.cpp : implementation file
//
//  This file contains an implementation of the FastLogger class.  The FastLogger
//  class is used by the dynamics PC to log data concerning the transport and 
//  response times for the specified QTG tests.
//
//  Usage - 
//    During the Transport/Response QTG test, the dynamics routines should call this
//    class with a hard real-time call at a speed of <10Hz.  The class will fail
//    if this rate is not achieved.  
//    The routine will log the parameters associated with the tests (which are
//    Control position, Light Level and Pot position) every 10ms, for 
//    a test duration of 2.4 seconds.  After 2.4 seconds, the calling routine should 
//    copy the logged data to the output structure for transmission to the IOS.
//
//  Inputs - 
//    float control - Control position
//    float lightlevel - Visual light level
//    float potpos - Potentiometer position
//
//  Outputs - 
//    float arControl[240]    \
//    float arLightLevel[240]  >>> Arrays to be copied to output structure for xmit to the IOS. 
//    float arPotPos[240]     /
//  Author - D.Yeates
//
//  Returns - 
//    FALSE if function successful - continue logging
//    LOGCOMPLETE If all is well and done 
//    NOCOUNTER If no high performance counter is available on this system 
//
//  Project - BAE Systems Piper Seneca FNPTII 
//
//  Creation Date - 12/04/2000
//
//  (c) 2000 Intersim Limited
//  The copyright of this document is the property of Intersim Limited.
//  This document is supplied by Intersim Limited on the express terms that
//  it is to be treated in confidence and that it may not be copied, used or
//  disclosed to others for any purpose except as authorised in writing by Intersim Limited.
//
/////////////////////////////////////////////////////////////////////////////////////////////
// Example implementation - 
//
// Step 1 - Create instance of class (probably a global)
//          CFastLogger fl;
//
// Step 2 - Call the Logger member function in a loop (the dynamics loop will be fine)
//          Pass in values to be logged.
//          int result = fl.Logger( control, lightlevel, potpos );
//
// Step 3 - Continue until the Logger function returns LOGCOMPLETE. Then copy
//          the FastLogger member variable arrays, arControl, arLightLevel and
//          arPotPos to the xmint structure for dispatch to the IOS.
//
// Step 4 - Finish and clean-up
//
/////////////////////////////////////////////////////////////////////////////////////////////

//**************************
//   RCS Markers           *
//**************************


//$Header: C:/master/seneca/RCS/fastlogger.cpp 1.1 2000/05/10 15:13:12 darren Exp colinj $
//$Log: fastlogger.cpp $
//Revision 1.1  2000/05/10 15:13:12  darren
//Initial revision
//

//----------------------------------------------------------------------

#include <windows.h>
#include "FastLogger.h"

// Constructor
CFastLogger::CFastLogger()
{
	// Get this systems high performance timer resolution
	QueryPerformanceFrequency(&PerformanceFreq);
	Trigger = PerformanceFreq.QuadPart / LOGFREQ; // Sets up a 100Hz trigger
	// Do Initialisation
	LastBigNumber.QuadPart = 0;
	memset( arControl, 0, sizeof(arControl) );
	memset( arLightLevel, 0, sizeof(arLightLevel) );
	memset( arPotPos, 0, sizeof(arPotPos) );
	FrameTime = 0.0;
	i = 0;
}

// Destructor 
CFastLogger::~CFastLogger()
{

}

/*
++
  CFastLogger Member Function - Logger

  This function logs the passed in values into member variable arrays and
  then after 2.4 seconds, signals to the calling program that it has completed.

  See Header to file FastLogger.cpp for detailed instructions upon how to use
  this class.
--
*/
int CFastLogger::Logger( float control, float lightlevel, float potpos )
{
	LARGE_INTEGER PerformanceCount;
	int result;

	// Get time from high resolution timer
    result = QueryPerformanceCounter( &PerformanceCount );
	if( result == 0 )
		return(NOCOUNTER);

	if( PerformanceCount.QuadPart >= ( LastBigNumber.QuadPart + Trigger ) )
	{
        arControl[i]    = control;
        arLightLevel[i] = lightlevel;
        arPotPos[i]     = potpos;
		i++;
				       
		if( i == (ARRAYSIZE ) ){
		   return(LOGCOMPLETE);
		}

	    // Calculate time between frames
		FrameTime = ((double)Trigger / ((double)PerformanceCount.QuadPart - LastBigNumber.QuadPart))/100.0;
	//	printf("FrameTime = %f\n", FrameTime  );

		// Continue logging
		LastBigNumber.QuadPart = PerformanceCount.QuadPart;
	}
  return(FALSE);
}

/////////////////////////////////////////////////////////////////////////////
// End of File FastLogger.cpp










