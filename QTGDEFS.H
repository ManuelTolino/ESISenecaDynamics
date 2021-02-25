/*
++
  File - qtgdefs.h

  Description - This file contains the Qtg test ID's, and the typedef for the
                Qtg data structure.
  
  Author - D.Yeates

  Created - 16/02/2000

  Updates - 
    26/04/2000 Darren: New Id's and function prototype
--
*/

//**************************
//   RCS Markers           *
//**************************


//$Header: C:/master/seneca/RCS/qtgdefs.h 1.2 2000/05/10 15:27:29 darren Exp $
//$Log: qtgdefs.h $
//Revision 1.2  2000/05/10 15:27:29  darren
//New Ids and prototype.
//
//Revision 1.1  2000/04/14 18:18:41  darreny
//Initial revision
//

//----------------------------------------------------------------------

#ifndef _QTGDEFS
#define _QTGDEFS

// QTG Test ID's
#define TEST1AI		1
#define TEST1AIIA	2
#define TEST1AIIB	3
#define TEST1BIA	4
#define TEST1BIB	5
#define TEST1BIC	6
#define TEST1CIA    7
#define TEST1CIB    8
#define TEST1CII	9
#define TEST2AIA    10
#define TEST2AIB    11
#define TEST2AIIA   12
#define TEST2AIIB   13
#define TEST2AIIIA  14
#define TEST2AIIIB  15
#define TEST2AIIIC  115
//#define TEST2AIIIC  16
#define TEST2AIV    17
#define TEST2AV     18
#define TEST2BIA    19
#define TEST2BIB    20
#define TEST2BIC    21
#define TEST2BID    22
#define TEST2BIIA   23
#define TEST2BIIB   24
#define TEST2BIIC   25
#define TEST2BIID   26
#define TEST2BIVA   27
#define TEST2BIVB   28
#define TEST2BIVC   29
#define TEST2BIVD   30
#define TEST2BVA    31
#define TEST2BVB    32
#define TEST2BVC    33
#define TEST2BVD    34
#define TEST2BVE    35
#define TEST2BVF    36
#define TEST2BVG    37
#define TEST2BVIA   38
#define TEST2BVIB   39
#define TEST2BVIIA  40
#define TEST2BVIIB  41
#define TEST2BVIIC  42
#define TEST2BVIII  43		// Woody Mod changed the tset name to have A at the end.
//#define TEST2BVIIIB  143		//Woody Mod added this test As new
#define TEST2BIX    44
#define TEST2BX     45
#define TEST2CI     46
#define TEST2CIIA   47
#define TEST2CIIB   48
#define TEST2CIII   49
#define TEST2CIV	50
#define TEST2CIVB   151 // Offset of 100 since this test was created by us.
#define TEST2CVA	51
#define TEST2CVB	52
#define TEST2CVI	53
#define TEST2CVIIA	54
#define TEST2CVIIB	55
#define TEST2CVIII	56		// Woody Mod changed the tset name to have A at the end.
//#define TEST2CVIIIB	156		//Woody Mod added this test As new
//#define TEST2CVIIIC	157		//Woody Mod added this test As new
//#define TEST2CVIIID	158		//Woody Mod added this test As new
#define TEST3AA		57
#define TEST3AB		58
#define TEST3AC		59
#define TEST3BA		60
#define TEST3BB		61
#define TEST3BC		62
#define TEST3BD		63
#define TEST3BE		64
#define TEST3BF		65
#define TEST3BG		66
#define TEST3BH		67
#define TEST3BI		68
#define TEST4A		69
#define TEST4BI		70
#define TEST4BII	71
#define TEST4BIII	72
#define TEST4C		73
#define TEST5A		74
#define TEST5B		75

// Misc Defines
// NONE

// Flight Trim defines
#define TAKEOFF  0
#define CLIMB    1
#define APPROACH 2
#define ONGND	 3
#define SAL		 4
#define CRUISE   5  

// Prototype for setup function
int QtgSetup( int );

// Event time structure (used internally in the _qtg_skeleton struct)
typedef struct _eventtime
{
   int wheelpos;   // Wheel position
   int columnpos;  // Column position
   int pedalpos;   // Pedel position
   int aob;        // Angle of bank
   int gearpos;    // Gear position
   int flappos;    // Flap position
   int trimpos;    // Trim position
   int targetspeed;// Target airspeed
   int leftrpm;    // RPM of left engine
   int rightrpm;   // RPM of right engine
   int leftmap;    // Manifold pressure of left engine
   int rightmap;   // Manifold pressure of right engine
   int holdheading;// Hold heading flag
   int holdalt;    // Hold altitude flag
   int holdspeed;  // Hold airspeed flag
   int holdaob;    // Hold Angle of bank flag
} ET;

// QTG Skeleton structure
typedef struct _qtg_skeleton
{
	int testid;
	// Environment
	int temperature;
	int pressure;
	int windspeed;
	// Aircraft Loading
	int fueltankleft;
	int fueltankright;
	int weight;
	float cofg;
	// Engines
	int leftrpm;
	int rightrpm;
	int leftmap;
	int rightmap;
	// Airframe
	int gear;
	int flaps;
	int parkbrake;
	int propsync;
	int leftcowlflap;
	int rightcowlflap;
	// Flight Conditions
	int heading;
	int roll;
	int pitch;
	int yaw;
	int flighttrim;
	int airspeed;
	int altitude;
	// Test Information
	int testduration;
	int testeventtime;
    // Event definition structure
	ET  et; 
} QTG;

#endif /* End of #ifndef _QTGDEFS */
// End of file
