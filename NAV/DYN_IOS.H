/*
File  : dyn_ios.h

Module: Dynamics and IOS for Seneca

Description: This file contains the ethernet INTERFACE structure for coms
             between the IOS and dynamics PC
*/

/***************************/
/*   RCS Markers           */
/***************************/

/*
$Header: C:/Master/seneca/NAV/RCS/dyn_ios.h 1.7 2000/06/16 10:45:12 ColinJ Exp $
$Log: dyn_ios.h $
Revision 1.7  2000/06/16 10:45:12  ColinJ
added a short to the end of weight data structure for padding.

Revision 1.6  2000/05/31 14:44:11  colinj
Format change.
Added groundtrack to QTGinput. Added 10 debug ints and floats
to dyn_to_ios. Renamed a spare to manualmode in ios_to_dyn.

Revision 1.5  2000/05/30 13:26:16  colinj
Renamed BearingFrom and Heading in StartData to reflect
that they are magnetic bearings.

Revision 1.4  2000/05/23 10:54:35  colinj
Added switch_status (long) to dyn_to_ios.
Renamed value to goflag in QTGinput.
Renamed value to mag_variation in ios_to_dyn.

Revision 1.3  2000/05/10 15:30:57  colinj
Added iosfastlogid.

Revision 1.2  2000/04/25 16:55:44  colinj
Change to ConditionalFailures structure.

Revision 1.1  2000/04/17 11:21:26  colinj
Initial revision

*/

/*---------------------------------------------------------------------*/

/*----------------*/
/* Include Files  */
/*----------------*/

#include "beacon.h"
#include "windows.h"
/*---------------------------------------------------------------------*/
/* Dynamics -> IOS ----------------------------------------------------*/
/*---------------------------------------------------------------------*/

#define PETEBUG

// For flags in dyn_to_ios
#define NAV1_BIT         0x00000001    /* If set, search for nav1 beacon */
#define NAV2_BIT         0x00000002    /* If set, search for nav2 beacon */
#define NDB_BIT          0x00000004    /* If set, search for non-directional beacon */
#define DME_BIT          0x00000008    /* If set, search for non-directional beacon */
#define RESET_DONE_BIT   0x00000010    /* If set, inform IOS to stop sending reset command */
#define COM1_TALK_BIT    0x00000020    /* If set, com1 push to talk is set */
#define COM2_TALK_BIT    0x00000040    /* If set, com2 push to talk is set */
#define SQUAWK_BIT       0x00000080    /* If set, squawk ident is true */
#define MARKER_WIDE_BIT  0x00000100    /* If set, marker detection fan is wider than normal */



// Flight instruments failure if set
#define FLI_P1_ALTITUDE_IND     0x00000001
#define FLI_P1_AIRSPEED_IND     0x00000002
#define FLI_P1_ALTIMETER_1      0x00000004
#define FLI_P1_ALTIMETER_2      0x00000008
#define FLI_P1_TURN_IND         0x00000010
#define FLI_P1_HORZ_POS_IND     0x00000020
#define FLI_P1_DELTA_CLIMB      0x00000040
#define FLI_P2_ALTITUDE_IND     0x00000080
#define FLI_P2_AIRSPEED_IND     0x00000100
#define FLI_P2_ALTIMETER_1      0x00000200
#define FLI_P2_TURN_IND         0x00000400
#define FLI_P2_HORZ_POS_IND     0x00000800
#define FLI_P2_DELTA_CLIMB      0x00001000
#define FLI_PITOT_BLOCK         0x00002000
#define FLI_STATIC_BLOCK        0x00004000
#define FLI_RIGHT_SUCTION_PUMP  0x00008000
#define FLI_LEFT_SUCTION_PUMP   0x00010000

// Avionics failures. These defines are never used!
#define AV_COM1         0x00000001
#define AV_COM_NAV2     0x00000002
#define AV_NAV1         0x00000004
#define AV_RADIO_PWR    0x00000008
#define AV_ADF          0x00000010
#define AV_RADAR        0x00000020
#define AV_AUTO_PILOT   0x00000040
#define AV_STANDBY_PWR  0x00000080
#define AV_LEFT_GYRO    0x00000100
#define AV_RIGHT_GYRO   0x00000200

// Engine instruments failure if set 
// Left Engine
#define LEFT_ENGI_MANIFOLD_ZERO         0x00000001
#define LEFT_ENGI_MANIFOLD_STUCK        0x00000002
#define LEFT_ENGI_MANIFOLD_MAX          0x00000003

#define LEFT_ENGI_RPM_IND_ZERO          0x00000004
#define LEFT_ENGI_RPM_IND_STUCK         0x00000008
#define LEFT_ENGI_RPM_IND_MAX           0x0000000c

#define LEFT_ENGI_EGT_IND_ZERO          0x00000010
#define LEFT_ENGI_EGT_IND_STUCK         0x00000020
#define LEFT_ENGI_EGT_IND_MAX           0x00000030

#define LEFT_ENGI_FUEL_FLOW_IND_ZERO    0x00000040
#define LEFT_ENGI_FUEL_FLOW_IND_STUCK   0x00000080
#define LEFT_ENGI_FUEL_FLOW_IND_MAX     0x000000c0

#define LEFT_ENGI_FUEL_CONT_IND_ZERO    0x00000100
#define LEFT_ENGI_FUEL_CONT_IND_STUCK   0x00000200
#define LEFT_ENGI_FUEL_CONT_IND_MAX     0x00000300

#define LEFT_ENGI_OIL_PRESS_IND_ZERO    0x00000400
#define LEFT_ENGI_OIL_PRESS_IND_STUCK   0x00000800
#define LEFT_ENGI_OIL_PRESS_IND_MAX     0x00000c00

#define LEFT_ENGI_OIL_TEMP_IND_ZERO     0x00001000
#define LEFT_ENGI_OIL_TEMP_IND_STUCK    0x00002000
#define LEFT_ENGI_OIL_TEMP_IND_MAX      0x00003000

#define LEFT_ENGI_CYL_TEMP_IND_ZERO     0x00004000
#define LEFT_ENGI_CYL_TEMP_IND_STUCK    0x00008000
#define LEFT_ENGI_CYL_TEMP_IND_MAX      0x0000c000

// Right Engine
#define RIGHT_ENGI_MANIFOLD_ZERO        0x00010000
#define RIGHT_ENGI_MANIFOLD_STUCK       0x00020000
#define RIGHT_ENGI_MANIFOLD_MAX         0x00030000

#define RIGHT_ENGI_RPM_IND_ZERO         0x00040000
#define RIGHT_ENGI_RPM_IND_STUCK        0x00080000
#define RIGHT_ENGI_RPM_IND_MAX          0x000c0000

#define RIGHT_ENGI_EGT_IND_ZERO         0x00100000
#define RIGHT_ENGI_EGT_IND_STUCK        0x00200000
#define RIGHT_ENGI_EGT_IND_MAX          0x00300000

#define RIGHT_ENGI_FUEL_FLOW_IND_ZERO   0x00400000
#define RIGHT_ENGI_FUEL_FLOW_IND_STUCK  0x00800000
#define RIGHT_ENGI_FUEL_FLOW_IND_MAX    0x00c00000

#define RIGHT_ENGI_FUEL_CONT_IND_ZERO   0x01000000
#define RIGHT_ENGI_FUEL_CONT_IND_STUCK  0x02000000
#define RIGHT_ENGI_FUEL_CONT_IND_MAX    0x03000000

#define RIGHT_ENGI_OIL_PRESS_IND_ZERO   0x04000000
#define RIGHT_ENGI_OIL_PRESS_IND_STUCK  0x08000000
#define RIGHT_ENGI_OIL_PRESS_IND_MAX    0x0c000000

#define RIGHT_ENGI_OIL_TEMP_IND_ZERO    0x10000000
#define RIGHT_ENGI_OIL_TEMP_IND_STUCK   0x20000000
#define RIGHT_ENGI_OIL_TEMP_IND_MAX     0x30000000

#define RIGHT_ENGI_CYL_TEMP_IND_ZERO    0x40000000
#define RIGHT_ENGI_CYL_TEMP_IND_STUCK   0x80000000
#define RIGHT_ENGI_CYL_TEMP_IND_MAX     0xc0000000


//flight model failures
#define FM_BLOCK_PITOT               0x00000001
#define FM_BLOCK_STATIC              0x00000002

#define FM_L_SUCTION                 0x00000004
#define FM_L_FUEL_STARVE             0x00000008
#define FM_L_MAGL_FAIL               0x00000010
#define FM_L_MAGR_FAIL               0x00000020
#define FM_L_OIL_TEMP                0x00000040
#define FM_L_OIL_PRESS               0x00000080
#define FM_L_CHT_HIGH                0x00000100
#define FM_L_PROP_CONTROL            0x00000200
#define FM_L_PROP_DEICE              0x00000400
#define FM_L_ALT_OVER                0x00000800
#define FM_L_ALT_ZERO                0x00001000

#define FM_R_SUCTION                 0x00002000
#define FM_R_FUEL_STARVE             0x00004000
#define FM_R_MAGL_FAIL               0x00008000
#define FM_R_MAGR_FAIL               0x00010000
#define FM_R_OIL_TEMP                0x00020000
#define FM_R_OIL_PRESS               0x00040000
#define FM_R_CHT_HIGH                0x00080000
#define FM_R_PROP_CONTROL            0x00100000
#define FM_R_PROP_DEICE              0x00200000
#define FM_R_ALT_OVER                0x00400000
#define FM_R_ALT_ZERO                0x00800000
//pop circuit breakers
#define CB_L_ALT                     0x01000000
#define CB_R_ALT                     0x02000000
#define CB_LG_PUMP                   0x04000000
#define CB_LG_CONTROL                0x08000000
#define CB_ELEC_FLAP                 0x10000000
#define CB_PITCH_TRIM                0x20000000
#define CB_AUTO_PILOT                0x40000000

typedef struct
{
   short id;
   short pad;
   float values[240];
   //float lightlevel[24];
   //float potpos[24];
} QTGfastlog;

typedef struct  // Control Calibration Screen data structure
	{
		short thrLeft;		// Left throttle position
		short thrRight;		// Right throttle position
		short propLeft;		// Left Prop pitch
		short propRight;	// Right prop pitch
		short mixtureLeft;	// Left engine mixture
		short mixtureRight;	// Right engine mixture
		short p1Lbrake;		// P1 Left brake position
		short p1Rbrake;		// P1 Right brake position
		short p2Lbrake;		// P2 Right brake position
		short p2Rbrake;		// P2 Left brake position
      short rudder_trim;   // rudder trim position
      short pad;
	} ccs;


typedef struct  // Inputs to QTG pages
	{
      char  event_start; // set when event running
      char  spare[3];    // 
      float timer;       // time of test
		short goflag;      // Set to true to tell ios to start drawing qtg plots
		short altitude;    // QNE altitude (pressure 1013mb) (ft)
		short roc;         // Rate of climb (Fpm)
		short airspeed;    // Airspeed (knots)
		short stallwarner; // Stall warner discrete (Off=0/On=1) 
		short leftmap;     // Left engine manifold pressure (Hg)
		short rightmap;    // Right engine minifold pressure (Hg)
		short leftrpm;	   // Left engine RPM
		short rightrpm;	   // Right engine RPM
		short bankangle;   // Bank angle (deg)
      short pitchangle;  // Pitch Angle (deg)
		short yawangle;    // Yaw Angle (deg)
		short heading;     // Aircraft Heading (deg)
		short columnpos;   // Column position (mm)
		short wheelpos;	   // Wheel position (mm)
		short pedalpos;	   // Pedal position (mm)
		short trimpos;	   // Trim position (mm)
		short flappos;	   // Flap position (deg)
		short gearpos;     // Landing gear position (UP=0/HALF=50/DOWN=100)
		short sideslip;	   // Sideslip angle (deg)
      short visualrvrdist;   // dist from touchdown
      short rollrate;        // Roll rate
      short groundtrack;
      short pad;
		float columnforce;     // Column force (being applied)
		float wheelforce;      // Wheel force (being applied)
		float pedalforce;      // Pedal force (being applied)
		float transportdelay;  // Transport delay for relevent test
      float distfromrw;
      float gforce;         
	} QTGinput;



typedef struct {
   //int counter;
   double host_lat;
   double host_lon;
   float  host_ht;      /* Above sea or terrain? (NB. If not above sea, then CANNOT compare with beacon height) */
   long   flags;        /* See defines for bit meanings */
   long   nav1_freq;    /* abcde in 100 mHz units (ie. abc.de mHz) */
   long   dme_freq;     /* abcde in 100 mHz units (ie. abc.de mHz) */
   long   nav2_freq;    /* abcde in 100 mHz units (ie. abc.de mHz) */
   long   ndb_freq;     /* abcde in 10  kHz units (ie. abcd.e kHz) */
   long   ndb_tol;      /* In Hz units (ie. h => 0.0h in 10 KHz units) */
   long   com1_freq;    /* abcde in 100 mHz units (ie. abc.de mHz) */
   long   com2_freq;    /* abcde in 100 mHz units (ie. abc.de mHz) */
   short  replay_time;  //amount of time recorded max 600 secs
   short  dynamics_state;          //change to dynamics state?
   long   switch_status;  //bit 1  heat on/off bits 2+3 fuel bits 3+4 defrost
   ccs      ios_ccs;
   QTGinput ios_ac;
   QTGfastlog ios_fastlog;
#ifdef PETEBUG
   float fdebug[10];
   int   idebug[10];
#endif

} dyn_to_ios;

/*---------------------------------------------------------------------*/
/* IOS -> Dynamics ----------------------------------------------------*/
/*---------------------------------------------------------------------*/

// For flags in ios_to_dyn
#define REPOSITION_BIT   0x00000001    /* If set, reset to given position */
#define NAV1_DONE        0x00000002    /* If set, nav1 beacon search has been performed */
#define NAV2_DONE        0x00000004    /* If set, nav2 beacon search has been performed */
#define NDB_DONE         0x00000008    /* If set, ndb  beacon search has been performed */
#define DME_DONE         0x00000010    /* If set, dme  beacon search has been performed */

#define DYN_HALT        2 //dynamics state flags
#define DYN_TRIM        3
#define DYN_RUN         4
#define DYN_PLAY        5
#define DYN_CQG         6
#define SHUTDOWN        7

typedef struct 
{
  // Atmospherics data
  unsigned short CurrentPressure;
  unsigned short TargetPressure;
  unsigned short RateOfChange;
	        short Temperature;
/////////////Potter Mod////////////////////////////////////
  unsigned short WindSpeed;
  unsigned short WindDirection;   //Sent as magnetic

/*	unsigned char SWindSpeed;
    unsigned char SWindDirection;
	unsigned char MWindSpeed;
    unsigned char MWindDirection;
  unsigned short Turbulance;
	unsigned char HWindSpeed;
    unsigned char HWindDirection;
*/
/////////////Potter Mod////////////////////////////////////
  unsigned short Turbulance;
  unsigned short Padding;

  float Vis;
  float CloudBase;
  float CloudThk;
  char  VisOn;
  char  CldOn;
  char  spare[2];

} AtmosphericsData;


#define START_FROM_AIRFIELD 0
#define START_FROM_BEACON   1

#define START_FROM_RUNWAY   0
#define START_FROM_APPRON   1
#define START_ON_APPROACH   2
typedef struct 
{
  char      FromWhat;      // airfield, beacon if set 
  char      FromWhich;     // airfield - runway,appron,approch  0,1,2 
  short     mBearingFrom;  // beacon, approach or appron, in tenths of a degree
  unsigned  short     DistanceFrom;  // beacon,appron ,approach, in tens of metres
  short     mHeading;      // craft, in tenths of a degree
  short     Altitude;      // craft
  short     Speed;         // craft

  double    Latitude;      // base lat  - use bearing from and distance from to compute craft's postion
  double    Longitude;     // base long - ditto
  
} StartData;

typedef struct 
{
   char SimState        : 4;// sixteen values
   
   char FreezeHeading   : 1;
   char FreezePosition  : 1;
   char FreezeAltitude  : 1;
} SimData;

typedef struct 
{
   unsigned char valid;
   unsigned char UseGlobalVis;
   unsigned char UseGlobalCloud;
   unsigned spare;
   
   float  AFheight;  //Metres
   double AFLat; // 8byte boundary
   double AFLng;

   BOOL 	 VisOn;
	BOOL	 CloudOn;
	BOOL	 GoRound;
	BOOL	 WindShear;
  
   UINT	CldBase;  //Metres
   UINT  CldThk;
	UINT	DewPt;
	UINT	ShearHeight;
	long  Temperature;
	
   long  Visibility;
	UINT	WindDir;   //Sent as magnetic
	UINT	WindSpeed;
	UINT	Pressure;
	
} AtisData;

typedef struct 
{
   BYTE	m_cLeftFuel;
	BYTE	m_cRightFuel;
	short	m_iPAXR1;

	short	m_iPAXR2;
	short	m_iPilotWeight;  
   short DryWeight;
   short m_iBaggage;
} WeightData;

typedef struct 
{
  char LeftEngFail;         // 0 = NO fail, 1 = Altitude , 2 = Speed , 3 = Time
  char RightEngFail;        // 0 = NO fail, 1 = Altitude , 2 = Speed , 3 = Time
  short Padding;          
  float LeftFailValue;      //- Alt in Ft , Speed in Knots , Time in Secs (failure countdown timer)
  float RightFailValue;     //- Alt in Ft , Speed in Knots , Time in Secs (failure countdown timer)
} ConditionalFailures;

#define CENTRE_VIEW  0
#define PILOT1_VIEW  1
#define PILOT2_VIEW  2

typedef struct 
{

   
   short             dynamics_state; // flag to control state of dynamics
   short             goto_time;      // number of seconds to rewind 0 = reset
   short             vis_length;     // count of following 'extra' data (if any)
   short             mag_variation;  //In tenths of a degree, positive West

   // Failures
   ConditionalFailures Conditional;
   int               FlightInstruments;   // INST : 2 bits each generally  0 ok 1 lower 2 stick 3 upper  
   int               EngineInstruments;   // INST : upper 16 bits left engine lower right engine each bit is off/on status 0 being ok

   int               Avionics;            // INST : 

   char              EngInstbitsAmm_Suct; // INST : part of group above
   unsigned char     Airframe;            // F/Model
   char              MiscInstruments;     // ??? whatever - used as a pad for now!!! 
   char              visual_view;         //0 = centre, 1 = pilot1, 2 = pilot2

   long             EngineMalfunctions;   // F/Model
   long             ElectricalMalfunctions;// F/Model; 

   long              flags; 


   // setup data 
   AtmosphericsData  Atmospherics;
   StartData         Start;
   AtisData          Atis; 
   WeightData        Weight; 

   // qtg's
   short TestNo;
   short manualmode;	//QTG Manual Mode Thing ?
   

   float dist_from_td;
   
   vhf_beac_attr  nav1_battr;
   vhf_beac_attr  dme_battr;
   vhf_beac_attr  nav2_battr;
   ndb_beac_attr  ndb_battr;
  
   char marker_type;    // 0 - none, 3 = inner , 2 = middle,  1 = outer
   char ExFlags;
   char iosfastlogid;
   char SimulationType;

} ios_to_dyn;




/* NOTES
   -----

When a beacon search is requested by the dynamics, the request should be
continually sent until the beacon_state received back is BEACON_DONE
(and then the BEACON_BIT should be cleared). Note that BEACON_DONE does
NOT mean that the beacon request was successful, only that the search
was performed. Success of the search is indicated in the valid_flag of
the beacon attributes.

*/


