// File: define.h

// general header file for global defines

//**************************
//   RCS Markers           *
//**************************

//$Header: C:/MASTER/seneca/RCS/define.h 1.6 2000/06/13 13:12:53 pete Exp $
//$Log: define.h $
//Revision 1.6  2000/06/13 13:12:53  pete
//Increased define for max egt from 500 to 1800.
//
//Revision 1.5  2000/05/25 09:34:19  colinj
//Changes to snd_in structure.
//
//Revision 1.4  2000/05/23 11:01:38  colinj
//Added switch_stat typedef.
//
//Revision 1.3  2000/05/11 07:14:45  colinj
//Extended snd_in structure.
//
//Revision 1.2  2000/05/10 09:01:05  colinj
//Added new defines for temp and pressure.
//
//Revision 1.1  2000/04/14 17:54:52  colinj
//Initial revision
//


/*---------------------------------------------------------------*/

#define INIT            1   //flags for dynamics state
#define DYN_HALT        2
#define DYN_TRIM        3
#define DYN_RUN         4
#define DYN_PLAY        5
#define DYN_QTG         6
#define SHUTDOWN        7
#define DYN_FREEZE      8

#define L_START         1
#define R_START         2


#define TRUE            1
#define FALSE           0
#define MAX_MANIFOLD   50
#define MAX_RPM      3500
#define MAX_EGT      1800
#define MAX_FUEL_FLOW  26
#define MAX_FUEL_CONT  60
#define MAX_OIL_PRESS 100
#define MAX_OIL_TEMP  240
#define MAX_CYL_TEMP  460
#define MAX_SUCTION     7
#define MIN_SUCTION     2
#define MAX_AMMETER   100
#define MIN_AMMETER  -100
#define INST_FINE       0
#define INST_MIN        1
#define INST_STUCK      2
#define INST_MAX        3


#define PI 3.141592f
#define DEG_TO_RAD (PI / 180.0f)
#define RAD_TO_DEG (180.0f / PI)

#define EARTH_RAD 6371000.0   /* Mean radius */
#define METRES_PER_DEGREE (DEG_TO_RAD * EARTH_RAD)

#define METRES_IN_NMILE 1852.0f
#define FT_IN_METRE     3.2808f

#define FEET_IN_MILLIBAR  27.23f
#define ISA_PRESURE     1013.25f
#define mb2Pa            100.0f
#define ISA_TEMPERATURE   15.0f

#define WARRIOR 1
#define SENECA  2

#define WARRIOR_BIT 1
#define SKYBOX_BIT  128
/*----------------*/
/* Typedefs       */
/*----------------*/

typedef struct {

   double lat;   //Degrees
   double lon;   //Degrees
   float  ht;    //Height above sea level, metres
   
} ll_pos;

typedef struct
{

   unsigned flag_deice       :1;  // Surface de-ice
   unsigned flag_geardown    :1; // Gear is moving up/down
   unsigned flag_gearoff     :1;
   unsigned flag_crash       :1;  // oops
   unsigned flag_skid        :1;  // Set this at touchdown
   unsigned flag_flapup      :1;   // Flap is moving up/down
   unsigned flag_flapoff     :1;
   unsigned flag_start_left  :1; // Ground start
   unsigned flag_start_right :1;
   unsigned flag_clunk       :1;
   unsigned flag_primer_left :1;    // primer pump
   unsigned flag_restart_left :1; // Engine has failed to start - loop starting sound
   unsigned flag_restart_right :1; // (flag_start_left should also be set)
   unsigned flag_on_left     :1;
   unsigned flag_on_right    :1;
   unsigned flag_strobe      :1; // beacon strobe
   unsigned flag_autopilot   :1;
   unsigned flag_manifold    :1;
   unsigned flag_primer_right:1;
   unsigned flag_deice_off   :1;
   unsigned flag_gyro        :1;
   unsigned flag_Lclunk      :1;
   unsigned flag_Rclunk      :1;
} snd_in;

typedef struct
{
   unsigned defrost_sw :1;
   unsigned l_fuel_sw  :2;
   unsigned r_fuel_sw  :2;
   unsigned heat_on_sw :2;
}switch_stat;
typedef struct
{
   short oat;
   short Lfuel;
   short Rfuel;
   short ACweight;
   short CofG;
   short Lrpm;
   short Lmap;
   short Rrpm;
   short Rmap;
   short gear;
   short flaps;
   short Lcowl;
   short Rcowl;
   short speed;
   short alt;
   short TestDuration;
   short EventTime;
}inital_cond;



