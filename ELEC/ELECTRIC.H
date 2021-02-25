//File: electric.h

//Module: Seneca Dynamics
//Description: This file ...

//**************************
//   RCS Markers           *
//**************************


//$Header: C:/master/seneca/ELEC/RCS/electric.h 1.2 2000/05/14 10:54:50 colinj Exp $
//$Log: electric.h $
//Revision 1.2  2000/05/14 10:54:50  colinj
//Landing lights now have 2 separate defines.
//
//Revision 1.1  2000/04/17 09:27:11  colinj
//Initial revision
//

//----------------------------------------------------------------------

#define LEFT 0
#define RIGHT 1

#define OFF 0
#define ON 1

/* Ammeter sources */
#define BATTERY 0
#define GENERATOR_L 1
#define GENERATOR_R 2
#define SOURCES 2

/* Possible loads */
#define AMBIENT 1
#define PROP_DEICE 2
#define PITOT_HEAT 4
#define GEAR_PUMP 8
#define FLAP_MOTOR 16
#define VENT_FAN 32
#define HEATER 64
#define DEFROST_FAN 128
#define RADIOS_TX 256
#define LANDING_LIGHTS1 512
#define RECOGNITION 1024
#define NAVIGATION 2048
#define STROBES 4096
#define LSTARTER 8192
#define RSTARTER 16384
#define LANDING_LIGHTS2 32768
#define LOADS 16

/* Faults *//*
#define ALT_L_OVERLOAD 1
#define ALT_R_OVERLOAD 2
#define ALT_L_ZERO 4
#define ALT_R_ZERO 8
#define BREAKER_ALT_L 16
#define BREAKER_ALT_R 32
#define BREAKER_GEAR_PUMP 64
#define BREAKER_GEAR_CTL 128
#define BREAKER_FLAP_CTL 256
#define BREAKER_PITCH_TRIM 512
#define AMMETER_ZERO 1024
#define AMMETER_MAX 2048
#define AMMETER_STUCK 4096
#define BATTERY_FLAT 8192
#define FAULTS 14
*/
#define ALT_L_OVERLOAD 1
#define ALT_L_ZERO 2
#define ALT_R_OVERLOAD 4
#define ALT_R_ZERO 8
#define BREAKER_ALT_L 16
#define BREAKER_ALT_R 32
#define BREAKER_GEAR_PUMP 64
#define BREAKER_GEAR_CTL 128
#define BREAKER_FLAP_CTL 256
#define BREAKER_PITCH_TRIM 512
#define AMMETER_ZERO 1024
#define BATTERY_FLAT 2048
#define AMMETER_MAX 4096
#define AMMETER_STUCK 8192
#define FAULTS 14
/* Switches */
#define EL_SW_LGEN 1
#define EL_SW_RGEN 2
#define EL_SW_BATT 4
#define EL_SWITCHES 3

/* electric.h */




/* High level interface */
/* Call EL_frame each frame with an EL_in. EL_out is returned. */

typedef struct EL_in
{
   double leftrpm, rightrpm; /* Engine : EL_update() */
   long unsigned int loads; /* Loads : EL_*load() */
   int ammeter; /* Ammeter source: EL_setammeter() */
   long unsigned int faults; /* Faults: EL_*fault() */
   long unsigned int switches; /* Switches: generators, battery */
} EL_in;

typedef struct EL_out
{
   double current; /* ammeter, A : EL_getammeter() */
   double charge; /* charge, Ah : EL_getcharge() */
   int lind, rind; /* alternator failure indicators : EL_getindicator() */
   int power; /* true if power is available : EL_power() */
   char left_pop;
   char right_pop;
} EL_out;


extern EL_out EL_frame(EL_in *);










/* Call EL_start before calling any other functions.
   This just initialises this module - it doesn't start any loads.
   The 'Ambient' load must be started later. */

/* Call EL_update with engine RPMs every frame */

extern void EL_start(void);
extern void EL_update(double leftrpm, double rightrpm);

/* Start, stop or set to a value loads */

extern void EL_load(int load);
extern void EL_noload(int load);

/* Get the ammeter reading (A) and battery charge (Ah) */

extern double EL_getcurrent(void);
extern double EL_getcharge(void);

/* Get the indicators for alternator failure */

extern int EL_getindicator(int);

/* Set/get the ammeter source */

extern void EL_setammeter(int);
extern int EL_getammeter(void);

/* Return true if power is available */

extern int EL_power(void);

/* Set and reset faults */

extern void EL_fault(int);
extern void EL_nofault(int);

extern long ElectricFaults;