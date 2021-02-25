/*
++
  File - qtgsetup.cpp

  Used by - Seneca Flight

  Description - QTG Setup Software

  Author - D.Yeates

  Created - 16/02/2000

  Updates - 
    26/04/2000 Darren: Better error checking.Function now returns a 
	success/fail flag.
--
*/

//**************************
//   RCS Markers           *
//**************************


//$Header: C:/master/seneca/RCS/qtgsetup.cpp 1.2 2000/05/10 15:26:04 darren Exp colinj $
//$Log: qtgsetup.cpp $
//Revision 1.2  2000/05/10 15:26:04  darren
//QtgSetup now returns an int.
//
//Revision 1.1  2000/04/14 17:35:51  darreny
//Initial revision
//
//

//----------------------------------------------------------------------

#include <windows.h>
#include <fstream.h>
#include "qtgdefs.h"

extern QTG qtg;  // The global QTG structure used by Flight to run the test

/*
++
  Function - QtgSetup

  Called by - Flight upon receiving a valid QTG test ID setup command.

  Description - This function accepts a QTG test Id as input, and initialises the 
                global QTG test structure with values relevent to the appropriate
				test Id as specified in an external data file.

  Inputs - QTG Test Id (See Header file for definitions)

  Outputs - Initialised global QTG structure

  Returns - 0 if successful
            -1 if bad things happens
--
*/
int QtgSetup( int testid )
{
    QTG local;
	ifstream in;
	char *junk=0;

	// Open qtg initialisation input file
	in.open("qtgdata.dat", ios::in, filebuf::openprot );
    if( in.good() == 0)
		return(-1); // Some sort of error opening file

	// Read in first two lines and junk them - they are just descriptive text.
	in.getline( junk, 1000, '\n');
	in.getline( junk, 1000, '\n');

	do
	{
	   // Read in data and populate local QTG structure
   	   in  >> local.testid 
	       >> local.temperature
		   >> local.pressure 
		   >> local.windspeed 
		   >> local.fueltankleft 
		   >> local.fueltankright 
		   >> local.weight 
		   >> local.cofg 
		   >> local.leftrpm 
		   >> local.rightrpm 
		   >> local.leftmap 
		   >> local.rightmap 
		   >> local.gear 
		   >> local.flaps 
		   >> local.parkbrake 
		   >> local.propsync 
		   >> local.leftcowlflap 
		   >> local.rightcowlflap 
		   >> local.heading 
		   >> local.roll 
		   >> local.pitch 
		   >> local.yaw 
		   >> local.flighttrim 
		   >> local.airspeed 
		   >> local.altitude 
		   >> local.testduration 
		   >> local.testeventtime 
		   >> local.et.wheelpos
		   >> local.et.columnpos
		   >> local.et.pedalpos
		   >> local.et.aob
		   >> local.et.gearpos
		   >> local.et.flappos
		   >> local.et.trimpos
		   >> local.et.targetspeed
		   >> local.et.leftrpm
		   >> local.et.rightrpm
		   >> local.et.leftmap
		   >> local.et.rightmap
		   >> local.et.holdheading
		   >> local.et.holdalt
		   >> local.et.holdspeed
		   >> local.et.holdaob;
	} while( ( local.testid != testid ) && ( in.eof() == 0 ) );

	// Close file 
	in.close();

	if( local.testid != testid )
	  return(-1);
	
	// Copy local structure to global
	memcpy( &qtg, &local, sizeof(QTG) );

	return(0);
}
