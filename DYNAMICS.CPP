/*
File: dynamics.cpp
Prefix: DY

Description: calls Seneca flight model and places h/w inputs into flight model structures
*/


/***************************/
/*   Revision Record       */
/***************************/

/*
$Header: C:/Master/seneca/RCS/dynamics.cpp 1.14 2000/06/15 14:14:42 ColinJ Exp $
$Log: dynamics.cpp $
Revision 1.14  2000/06/15 14:14:42  ColinJ
Runway start now starts with engines running.
Apron start corrected.
Pitch stick input shaped.
Nose gear extension failure + code to simulate nose low attitude  on ground.
Crash flag set if wheels up landing.
Conditional failure now turns off mags not a sieze.

Revision 1.13  2000/05/31 10:25:02  colinj
reset_dynamics: IOS now sets up the starting height in
all cases (including approach). Tidy up.
DY_trim_flight: move call to simulate_model to end of each loop
Added fuel to U_A_MASS. New routine calc_cofg and variable
DY_calcedcofg.
Primers now depend on electric power. Comment out autopilot code.

Revision 1.12  2000/05/30 15:32:27  juliae
Call FC_reset_cloud in reset_dynamics.

Revision 1.11  2000/05/26 10:32:40  colinj
START heading and bearing from are now given in magnetic values
and converted to true.
Added fuel starvation fault. U_D_GEAR is now always setup.
Set U_D_S2_BLOCKAGE, U_D_PROP_ICE and U_D_PROP_DEICE.
Added prop sync control.
Changed condition on U_D_FAIL_GEAR_PUMP being set.

Revision 1.10  2000/05/24 16:53:06  colinj
Changes to support new flight model.
New DY_trim_flight routine. reset_dynamics changed for
proper trim model.

Revision 1.9  2000/05/14 09:37:01  colinj
Change to start conditions in reset_dynamics

Revision 1.8  2000/05/14 09:26:37  colinj
Allow for landing gear control tripped.
Enable manifold drain. Change to conditional engine fails.

Revision 1.7  2000/05/11 13:30:02  colinj
Added support for special move mode, SE_freeze_on.

Revision 1.6  2000/05/11 09:54:34  colinj
Fuel changes.

Revision 1.5  2000/05/10 12:33:43  colinj
Changes to support new flight model.

Revision 1.4  2000/05/10 09:09:07  colinj
Temperature and pressure changes.

Revision 1.3  2000/05/09 16:42:33  juliae
Call WD_reset_wind in reset_dynamics.

Revision 1.2  2000/04/25 16:57:42  colinj
Support for changes to conditional failures.

Revision 1.1  2000/04/14 16:33:05  colinj
Initial revision

 
*/

/*---------------------------------------------------------------*/

//----------------
// Include Files
//----------------

#include <stdio.h>
#include <stdlib.h>
#include <conio.h>
#include <dos.h>
#include <math.h>
#include <windows.h>


#include "fltdy\fmgdefs.h"
#include "fltdy\fmg.h"
#include "warrior_fltdy\war_fmg.h"
#include "fltdy\fms_s3fm.h"
#include "fltdy\fms_defs.h"
#include "define.h"
#include "const.h"
//#include "stddefs.h"
#include "dat_stru\in_head.h"
#include "dat_stru\out_head.h"
#include "faults.h"
#include "elec\electric.h"
#include "config.h"
#include "nav\dyn_ios.h"
#include "nav\cpit_def.h"

//----------------
// Defines
//----------------

#define HT_C_OF_G         2.0f
#define FUEL_DELAY        1.0f
#define FUEL_STARVE_RATE  0.01f
#define PROP_SYNC_LIMIT  50.0f
#define PROP_LEVER_LIMIT  7.5f

#define DB_CHECK 1
#define DEBUG1   1
#define BUF_START 18.0f
#define ZERO 0.00f
#define SWITCHING 1

//--------------------
// External Routines  
//--------------------

extern void UT_offset_along_dir (float x1, float y1, float z1, float h,
                                 float p, float dist, float *x, float *y, float *z);
extern void UT_local_xy_2_ll(float delta_x, float delta_y, double ref_lat, double ref_lon,
                             double *lat1, double *lon1);
extern void RR_init_db_check();
extern void WD_reset_wind();
extern void FC_reset_cloud();
extern void CS_gyro_ok();
extern void CS_gyro_init();
extern float CS_mag_to_true_head(float mag_head);
extern float CS_true_to_mag_head(float true_head);
extern void WD_wind_vec_to_speed_dir(float e,float n,float d,float *speed,float *dir);

//----------------
// Externals
//----------------
extern sen_in_struct    *IO_Sen_in;
extern OUTPUTin			*OUTPUTinbox;//simulinkudp
extern sen_out_struct   *IO_Sen_out;
extern sen_fault_struct IO_Sen_faults;

extern EL_out           EL_output;
extern scale_struct     RC_scale;
extern AtisData         IOS_atis;

extern double sim_run_time;

extern float y_eng;
extern float debug_angle;
extern float debug_angle1;
extern float col_err;
extern float col_al;
extern float h0;
extern float delta_time;
extern float terrain_height;
extern float CS_magnetic_var;
extern float vis_runway_head;
extern float SE_back_track;
extern float ah_air_press;
extern float P1_pitch_error;
extern float P1_roll_error ;
extern float P2_pitch_error;
extern float P2_roll_error ;
extern float Turb_vN, Turb_vE, Turb_vD;
extern float apKcdi_dot;
extern float apKcdi;


extern int IOS_simtype;
extern int IO_avionic_pwr;
extern int IO_save_state;
extern int dyn_state;
extern int SE_hz;
extern int SE_freeze_on;
extern int nav_valid;
extern int gs_valid;
extern int mode;

extern char trim_warn;
extern char hdg_off;
extern char pitch_trim_on;
extern char fd_switch;
//----------------
// Globals
//----------------

INPUTS inputs;
OUTPUTS outputs;
ATMOS atmosphere;
STATES states;
SIMULINKdouble simulinkdoubledata;
SIMULINKfloat SIMULINKfloatdata;
StartData DY_start;
AtmosphericsData DY_atmos;
WeightData DY_weight;
ConditionalFailures DY_con_fail;

float dum_cg = 94.5f;
float dt;
float DY_last_pressure = 1013.25;
float DY_new_pressure  = 1013.25;
float DY_temp          = ISA_TEMPERATURE;
float DY_calcedcofg    = 0.0;
float DY_total_weight = 4750.0;
float test_cofg   = 90.0;
float pitch_input[3]   = {-100.0, -20.0,100.0}; 
float pitch_output[3] = {-1.0,0.0,1.0};
float DY_wind_speed = 0.0,DY_wind_dir = 0.0,DY_Twind_dir = 0.0;
float pitch_stop = -90.0f;
float cg_change = 0.0f;
float cg_change1 = -1.0f;
float set_stick_pos = 0.0f;
float cg_change2 = 1.0f;
float trimed_pitchpos     = 0.0f;

float trimed_pedpos       = 0.0f;		//Woody Mod
float trimed_rollpos      = 0.0f;		//Woody Mod

float head_error = 0.0f;

int DY_need_db = FALSE;
int DY_freeze_status = 0;
int DY_init_ap = FALSE;

char DY_l_start       = FALSE;
char DY_r_start       = FALSE;
char hold_head        = FALSE;
char PETE_HEAD        = FALSE;
char DY_trimed        = FALSE;
char DY_startInFreeze = FALSE;

//----------------
// Locals
//----------------

static float left_failtimer = 0.0;
static float right_failtimer = 0.0;
static float LFuelDelay = 0.0,RFuelDelay = 0.0;
static float psi_error;
static float theta_dem           = 5.0;
static float gamma_int           = 0.0;
static float roll_int            = 0.0;

static float right_fuel_close    = 0.0;
static float left_fuel_close     = 0.0;
static float DryWeight, PilotWeight, Pax1Weight, Pax2Weight, BaggageWeight;
static float ng_pitch_attitude = 0.0;
static float alpha_error = ZERO;
static float cg_dem = ZERO,sw_neg = 1.0f;
static float kick_timer = 0.0f;
static float h0_timer = 0.0f;
static float h0_timer1 = 0.0f;
static float mod_pitch = 0.0f;
static float ap_ct = 0.0f;
static unsigned long int setupct    = 0;
// for trim in flight cases rpm,map and airspeed settings
// cases are approach , climb , circuit , cruise
static int rightrpm[4] = {2600,2600,2400,2400};
static int leftrpm[4]  = {2600,2600,2400,2400};
static int rightmap[4] = {17,35,21,33};
static int leftmap[4]  = {17,35,21,33};
static int airspeed[4]    = {100,120,120,155};

static float cg_aoa[3] = {-100.0f,15.5f,26.0f};
float cg_mov[3] = {0.0f,0.0f,5.0f};

static char last_LconFail = 0;
static char last_RconFail = 0;
static char left_eng_fail = FALSE;
static char right_eng_fail = FALSE;
static char last1_up = SW_OFF;
static char last1_dn = SW_OFF;

static char last2up = SW_OFF;
static char last2dn = SW_OFF;
   
static char last3up = SW_OFF;
static char last3dn = SW_OFF;

static char last4up = SW_OFF;
static char last4dn = SW_OFF;

//--------------------
// Forward References
//--------------------

void DY_trim_flight(void);

void set_inputs();
static void conditional_fail(void);
static float calc_cofg(void);

//----------------------------------------------------------------------

void reset_dynamics(void)
{
   double height, North, East, lat = 0.0f, lon = 0.0f;
   float  speed,heading,theta,gamma,phi;
   float  origin_x, origin_y, origin_z;
   float  gslope;
   float  dist;
   float  start_x, start_y, start_z;
   double start_lat, start_lon;
   float  iohz;

   //reset all inputs outpus and states to zero to stop flight model "rembering anything" from last time.
   memset(&inputs,0,sizeof(INPUTS));
   memset(&states,0,sizeof(STATES));
   memset(&outputs,0,sizeof(OUTPUTS));


#ifdef DEBUG1
printf("\nIOS what: %d where %d",DY_start.FromWhat,DY_start.FromWhich);
printf("\nSTART heading %d lat %lf lon %lf",DY_start.mHeading,DY_start.Latitude, DY_start.Longitude);
printf("\nSTART BearingFrom %d, DistanceFrom = %d",DY_start.mBearingFrom, DY_start.DistanceFrom);
printf("\nSTART alt = %d, speed = %d knots", DY_start.Altitude, DY_start.Speed);
#endif

   DY_trimed = FALSE;

   WD_reset_wind();
   FC_reset_cloud();

   init_model(NULL);

   iohz = (float) SE_hz;
   
	if (iohz < 20.0)
		 iohz =20.0;
	else
	if (iohz > 200 )
		 iohz = 200.0;

	dt = delta_time;


   origin_x = 0.0;
   origin_y = 0.0;
   origin_z = DY_start.Altitude;
   terrain_height = DY_start.Altitude - 1.0f;
   height  = origin_z + HT_C_OF_G;
   if (height < HT_C_OF_G)
      height = HT_C_OF_G;
   start_lat = DY_start.Latitude;
   start_lon = DY_start.Longitude;
   heading = CS_mag_to_true_head((float)(DY_start.mHeading)/10.0f);
   heading = (float)(heading  * DEG_TO_RAD);
   North   = 0.0;
   East    = 0.0;
   theta   = 0.0;   
   gamma   = 0.0;    
   phi     = 0.0;
   

   if (DY_start.FromWhat == START_FROM_AIRFIELD && (DY_start.FromWhich == START_FROM_RUNWAY || DY_start.FromWhich == START_FROM_APPRON))
   {
      set_inputs();
      speed            = 0.001f;
      DY_trimed        = TRUE;
      DY_startInFreeze = FALSE;
      gslope           =   0.0f;
      inputs.dinps[U_D_GEAR] = 1;

      if(DY_start.FromWhich == START_FROM_RUNWAY)
      {
         CS_gyro_ok();
         ah_air_press    = 240;
         P1_pitch_error  = 0.0;
         P1_roll_error   = 0.0;
         P2_pitch_error  = 0.0;
         P2_roll_error   = 0.0;
         inputs.dinps[U_D_LIVE_START] = TRUE;
         lat     = (start_lat * DEG_TO_RAD);
         lon     = (start_lon * DEG_TO_RAD);
      }
      else
      if(DY_start.FromWhich == START_FROM_APPRON)
      {
         CS_gyro_init();
         ah_air_press = 0;
         P1_pitch_error  = -25.0;
         P1_roll_error   = -45.0;
         P2_pitch_error  = -25.0;
         P2_roll_error   = -45.0;
         dist = (float)((DY_start.DistanceFrom/-10.0));// * -10.0);
         vis_runway_head = CS_mag_to_true_head((float)(DY_start.mBearingFrom/10.0));//; /10.0));//(DY_start.Heading/10.0);
         UT_offset_along_dir (origin_x, origin_y, origin_z, vis_runway_head, -gslope, -dist,
                              &start_x, &start_y, &start_z);


         UT_local_xy_2_ll(start_x, start_y, DY_start.Latitude, DY_start.Longitude,
                          &start_lat, &start_lon);

  			lat = (start_lat * DEG_TO_RAD);
         lon = (start_lon * DEG_TO_RAD);
      	height  = (origin_z + HT_C_OF_G);
	      if (height < HT_C_OF_G)
         height = HT_C_OF_G;

      }
   }
   else
	{
		if ((DY_start.FromWhat == START_FROM_AIRFIELD) && (DY_start.FromWhich == START_ON_APPROACH))
		{
         origin_z = DY_start.Altitude;
			DY_start.Speed = 0;
         dist = (float)((DY_start.DistanceFrom - 35.0) * -10.0);
         vis_runway_head = CS_mag_to_true_head((float)(DY_start.mBearingFrom/10.0));//(DY_start.Heading/10.0);

         gslope = 0.0;

		}
		else
		{
         origin_z = DY_start.Altitude;

         dist = (float)((DY_start.DistanceFrom) * -10.0);
         vis_runway_head = CS_mag_to_true_head((float)(DY_start.mBearingFrom/10.0));//; /10.0));//(DY_start.Heading/10.0);
         gslope = 0.0;
		}

      UT_offset_along_dir (origin_x, origin_y, origin_z, vis_runway_head, -gslope, -dist,
                           &start_x, &start_y, &start_z);


      UT_local_xy_2_ll(start_x, start_y, DY_start.Latitude, DY_start.Longitude,
                          &start_lat, &start_lon);
      set_inputs();
      CS_gyro_ok();
      ah_air_press = 240;
      P1_pitch_error  = 0.0;
      P1_roll_error   = 0.0;
      P2_pitch_error  = 0.0;
      P2_roll_error   = 0.0;

      inputs.dinps[U_D_LIVE_START] = TRUE;
      speed = (float)(airspeed[DY_start.Speed] * KTS_TO_MS);
      DY_startInFreeze = TRUE;
		lat = (start_lat * DEG_TO_RAD);
      lon = (start_lon * DEG_TO_RAD);
   }

#ifdef DB_CHECK
   //Need to ensure database is loaded before going in to RUN mode
   DY_need_db = TRUE;
   RR_init_db_check();
#endif

   DY_last_pressure = DY_atmos.CurrentPressure;
   DY_new_pressure  = DY_atmos.CurrentPressure;

   DryWeight = DY_weight.DryWeight;
   if (DryWeight < 3222.0)
       DryWeight = 3222.0;

   PilotWeight   = DY_weight.m_iPilotWeight;
   Pax1Weight    = DY_weight.m_iPAXR1;
   Pax2Weight    = DY_weight.m_iPAXR2;
   BaggageWeight = DY_weight.m_iBaggage;

   inputs.inps[U_A_CG]   = 90.0;
   inputs.inps[U_A_MASS] = (float)(DryWeight + Pax1Weight + Pax2Weight + PilotWeight + BaggageWeight + DY_weight.m_cLeftFuel + DY_weight.m_cRightFuel);
   left_eng_fail  = FALSE;
   right_eng_fail = FALSE;


   atmosphere.ht = DY_start.Altitude - 2.0f;

   if(speed < 5.0f)
   {
      inputs.inps[U_A_L_BRAKE] = 1.0f;
      inputs.inps[U_A_R_BRAKE] = 1.0f;
      inputs.dinps[U_D_PARK]   = TRUE;
   }

#ifdef SWITCHING
   if(IOS_simtype == WARRIOR)
   {
      reset_war_model(height,North,East,lat,lon,speed,heading,theta,gamma,phi,&inputs,&states,&outputs,&atmosphere);
   }
   else
#endif
      reset_model(height,North,East,lat,lon,speed,heading,theta,gamma,phi,&inputs,&states,&outputs,&atmosphere);

   float a,b,c,d,e;
   int fake[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
   int f,g;
   
   KFC150_exec(0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,&a,&b,&c,&d,&e,fake,fake,&f,&g,0);

   if(inputs.dinps[U_D_LIVE_START] == TRUE)
   {
      mode = KFC150_M_IDLE;
      DY_init_ap = TRUE;
      
   }
   else
      DY_init_ap = FALSE;
      

   LFuelDelay = 0.0;
   RFuelDelay = 0.0;
   setupct    = 0;

#ifdef DEBUG1
   printf("\n states lat %lf lon %lf",states.latitude * R2D,states.longitude * R2D);
#endif
}

//----------------------------------------------------------------------

float col=0;
void simulate()
{
	float iohz;
	iohz = (float) SE_hz;
	if (iohz < 20.0)
		 iohz = 20.0;
	else
	if (iohz > 200.0 )
		 iohz = 200.0;
	dt= delta_time;//0.02;//1/iohz;
	               /* one more frame done! */

   //temp stuff
      if(IO_Sen_in->test_switch_1_up == SW_ON && last1_up == SW_OFF)
      {
         //apKcdi_dot += 5.0f;
         y_eng += 0.1f;

      }
      else
      if(IO_Sen_in->test_switch_1_dn == SW_ON && last1_dn == SW_OFF)
      {
         //apKcdi_dot -= 5.0f;
         y_eng -= 0.1f;
      }


      last1_dn = IO_Sen_in->test_switch_1_dn;
      last1_up = IO_Sen_in->test_switch_1_up;

   if(IO_Sen_in->test_switch_2_up == SW_ON && last2up == SW_OFF)
   {
      ///apKcdi += 0.01f;
      debug_angle += 0.2f;

   }

   if(IO_Sen_in->test_switch_2_dn == SW_ON && last2dn == SW_OFF)
   {
      //apKcdi -= 0.01f;
      debug_angle -= 0.2f;

   }

   if(IO_Sen_in->test_switch_3_up == SW_ON && last3up == SW_OFF)
   {
      debug_angle1 += 0.2f;
   }

   if(IO_Sen_in->test_switch_3_dn == SW_ON && last3dn == SW_OFF)
   {
      debug_angle1 -= 0.2f;
   }

   if(IO_Sen_in->test_switch_4_up == SW_ON && last4up == SW_OFF)
   {
   }

   if(IO_Sen_in->test_switch_4_dn == SW_ON && last4dn == SW_OFF)
   {
   }

last2up = IO_Sen_in->test_switch_2_up;
last2dn = IO_Sen_in->test_switch_2_dn;
   
last3up = IO_Sen_in->test_switch_3_up;
last3dn = IO_Sen_in->test_switch_3_dn;

last4up = IO_Sen_in->test_switch_4_up;
last4dn = IO_Sen_in->test_switch_4_dn;
   set_inputs();

//   printf("\n THROTTLE %f RPM %f",IO_Sen_in->left_prop_rpm,inputs.inps[U_A_L_RPM]); 
   conditional_fail();
   outputs.theta += (ng_pitch_attitude * D2R);

   if(dyn_state == DYN_QTG)
   {
      atmosphere.ht = 0.01f;
      DY_freeze_status = FREEZE_NED;
      inputs.inps[U_A_MASS] = 4007;

   }

   if(DY_init_ap == TRUE)
   {
      ap_ct += delta_time;
   }

   if(ap_ct > 1.5f)
   {
      DY_init_ap = FALSE;
   }

   if(DY_init_ap == FALSE)
   {
      ap_ct = 0.0f;
   }
   
#ifdef SWITCHING
   if(IOS_simtype == WARRIOR)
      simulate_war_model(&inputs,&states,&outputs,&atmosphere,dt,DY_freeze_status);
   else
#endif
	   simulate_model(&inputs,&states,&outputs,&atmosphere,dt,DY_freeze_status);

   if((inputs.dinps[U_D_NOSE_GEAR_EXTEND_FAIL] == TRUE )&&( outputs.duser[Y_D_N_CONTACT_ACTUAL] == TRUE) && (states.user[X_N_GEAR] < 0.85))
   {
      if(ng_pitch_attitude < 10.0)
      ng_pitch_attitude += (10.0f * delta_time);
   }
   else
      ng_pitch_attitude = 0.0f;
  

   outputs.theta -= (ng_pitch_attitude * D2R);
   
   if(outputs.duser[Y_D_N_CONTACT_ACTUAL] == TRUE || 
      outputs.duser[Y_D_R_CONTACT_ACTUAL] == TRUE || 
      outputs.duser[Y_D_L_CONTACT_ACTUAL] == TRUE)
   {
      h0_timer1 += delta_time;
      if(h0_timer1 > 3.0f)
      {
         if(h0 < 0.75)
         {
            h0 +=  0.25f * delta_time/10.0f;
         }
      }
      h0_timer = 0.0f;
   }
   else
   if(h0 > 0.50)
   {
      h0_timer += delta_time;
      if(h0_timer > 5.0f)
      {
         h0 -=  0.25f * delta_time/15.0f;
      }
      h0_timer1 = 0.0f;
   }


   //if any "real contact flag is true and gear is left gear is not full down set crash flag
   
   if((outputs.duser[Y_D_N_CONTACT_ACTUAL] == TRUE || 
      outputs.duser[Y_D_R_CONTACT_ACTUAL] == TRUE || 
      outputs.duser[Y_D_L_CONTACT_ACTUAL] == TRUE)
      && (states.user[X_L_GEAR] < 0.9)) 
   {
      outputs.duser[Y_D_CRASH] = TRUE;
   }

	WD_wind_vec_to_speed_dir(atmosphere.wE,atmosphere.wN,0.0,&DY_wind_speed,&DY_Twind_dir);
	DY_wind_dir = CS_true_to_mag_head(DY_Twind_dir);

	simulinkdoubledata.latitude=states.latitude; //simulink
	simulinkdoubledata.longitude=states.longitude; //simulink
	simulinkdoubledata.N=states.N; //simulink
	simulinkdoubledata.E=states.E; //simulink
	simulinkdoubledata.D=states.D; //simulink

	//OUTPUT
	SIMULINKfloatdata.phi=outputs.phi;
	SIMULINKfloatdata.theta=outputs.theta;
	SIMULINKfloatdata.psi=outputs.psi;
	SIMULINKfloatdata.phi_dot=outputs.phi_dot;
	SIMULINKfloatdata.theta_dot=outputs.theta_dot;
	SIMULINKfloatdata.psi_dot=outputs.psi_dot;
	SIMULINKfloatdata.p_dot=outputs.p_dot;
	SIMULINKfloatdata.q_dot=outputs.q_dot;
	SIMULINKfloatdata.r_dot=outputs.r_dot;
	SIMULINKfloatdata.alpha=outputs.alpha;
	SIMULINKfloatdata.beta=outputs.beta;
	SIMULINKfloatdata.gamma=outputs.gamma;
	SIMULINKfloatdata.ax=outputs.ax;
	SIMULINKfloatdata.ay=outputs.ay;
	SIMULINKfloatdata.az=outputs.az;
	SIMULINKfloatdata.Vt=outputs.Vt;
	SIMULINKfloatdata.Vtas=outputs.Vtas;
	SIMULINKfloatdata.Vg=outputs.Vg;
	SIMULINKfloatdata.track=outputs.track;
	SIMULINKfloatdata.amach=outputs.amach;
	SIMULINKfloatdata.temperature=outputs.temperature;
	SIMULINKfloatdata.pressure=outputs.pressure;
	SIMULINKfloatdata.density=outputs.density;
	SIMULINKfloatdata.atm_theta=outputs.atm_theta;
	SIMULINKfloatdata.atm_delta=outputs.atm_delta;
	SIMULINKfloatdata.atm_sigma=outputs.atm_sigma;
	SIMULINKfloatdata.hdot=outputs.hdot;
	//STATES
	SIMULINKfloatdata.u=states.u;
	SIMULINKfloatdata.v=states.v;
	SIMULINKfloatdata.w=states.w;
	SIMULINKfloatdata.p=states.p;
	SIMULINKfloatdata.q=states.q;
	SIMULINKfloatdata.r=states.r;
	SIMULINKfloatdata.e0=states.e0;
	SIMULINKfloatdata.e1=states.e1;
	SIMULINKfloatdata.e2=states.e2;
	SIMULINKfloatdata.e3=states.e3;



  //SIMULINKfloat; //SIMULINK

}

//----------------------------------------------------------------------

void DY_trim_flight(void)
{
   float dt = 0.01f;
   

   outputs.user[Y_A_TRIM_COST] = 100;
   int freeze = (FREEZE_NED + FREEZE_LATLON);
   int innerct = 0;
   DY_start.Speed = 0;
   int setuptype = DY_start.Speed;
//   reset_model(height,North,East,lat,lon,speed,heading,theta,gamma,phi,&inputs,&states,&outputs,&atmosphere);
   DY_trimed = FALSE;
   if(setupct < 100001) //total loops to do before droping out as a failed trim
   {
      outputs.user[Y_A_TRIM_COST] = 10000.0; //Set trim cost high to avoid a previous trimed condition droping us out
      while(innerct < 100) //do 30 loops at one time
      {  // set rpm levers to achieve demanded RPM
         inputs.inps[U_A_L_RPM]     = (((leftrpm[setuptype] * RPM2RPS) - rpm_lo) / (rpm_hi - rpm_lo));
         if(inputs.inps[U_A_L_RPM] < 0.0) inputs.inps[U_A_L_RPM] = 0.0f;

         inputs.inps[U_A_R_RPM]     = (((rightrpm[setuptype] * RPM2RPS) - rpm_lo) / (rpm_hi - rpm_lo));
         if(inputs.inps[U_A_R_RPM] < 0.0) inputs.inps[U_A_R_RPM] = 0.0f;
         
         // allow trim function to drop out if a good trim is achieved
         if(outputs.user[Y_A_TRIM_COST] < 1.0)
         {
            setupct = 100001;
            innerct = 100;
            DY_trimed = TRUE;
         }

//TRIM LEFT MAP to demanded
//         inputs.dinps[U_D_GEAR] = 1;

         //inputs.dinps[U_D_FLAP] = 2;

         inputs.inps[U_A_L_THROT] +=  (0.1e-3f * (leftmap[setuptype] - outputs.user[Y_A_L_MAN_P]));
         if(inputs.inps[U_A_L_THROT] < 0.0) inputs.inps[U_A_L_THROT] = 0.0f;
         if(inputs.inps[U_A_L_THROT] > 1.0) inputs.inps[U_A_L_THROT] = 1.0f;
//TRIM RIGHT MAP to demanded
         inputs.inps[U_A_R_THROT] +=  (0.1e-3f * (rightmap[setuptype] - outputs.user[Y_A_R_MAN_P]));
         if(inputs.inps[U_A_R_THROT] < 0.0) inputs.inps[U_A_R_THROT] = 0.0f;
         if(inputs.inps[U_A_R_THROT] > 1.0) inputs.inps[U_A_R_THROT] = 1.0f;

//For S & Level flight set pitch stick to achieve zero ROC
         if(DY_start.Speed == 2 || DY_start.Speed == 3)
         {
            gamma_int += (outputs.gamma * dt);
            inputs.inps[U_A_LONG]  = ((2.0f * gamma_int) + (50.0f * (outputs.gamma)) + (10.0f * states.q));
            if(inputs.inps[U_A_LONG] > 1.0) inputs.inps[U_A_LONG] = 1.0;
            if(inputs.inps[U_A_LONG] < -1.0) inputs.inps[U_A_LONG] = -1.0;
         }
         else // trim to get air speed
         {
            theta_dem += (-1.0e-4f * (airspeed[setuptype] - outputs.user[Y_A_P1_ASI]));
            inputs.inps[U_A_LONG]  = ( (-10.0f * (theta_dem * D2R - outputs.theta)) + (1.0f * states.q));
            if(inputs.inps[U_A_LONG] > 1.0) inputs.inps[U_A_LONG] = 1.0;
            if(inputs.inps[U_A_LONG] < -1.0) inputs.inps[U_A_LONG] = -1.0;
         }

         //inputs.inps[U_A_LAT] = 0.0;
         float craft_head = outputs.psi * R2D;
         if(craft_head < 0.0f)
            craft_head += 360.0f;
         psi_error = craft_head - CS_mag_to_true_head((float)(DY_start.mHeading)/10.0f);
         inputs.inps[U_A_LAT]  = (float) ((10.0 * outputs.psi_dot) + (col_err * psi_error));//roll_int) + (25.0 * (qtg_outputs.psi)) +  (20.0 * qtg_states.p));

         if(inputs.inps[U_A_LAT] > 1.0) inputs.inps[U_A_LAT] = 1.0;
         if(inputs.inps[U_A_LAT] < -1.0) inputs.inps[U_A_LAT] = -1.0;

         inputs.inps[U_A_PED] = 0.0;

         float save_pitch = inputs.inps[U_A_LONG];
         if(inputs.inps[U_A_LONG] < -0.9f)
         {
            inputs.inps[U_A_LONG] = -0.9f;
         }
            float cg_change2 = 0.0f;
            inputs.inps[U_A_LONG]      = (table1D(pitch_input,pitch_output,3,inputs.inps[U_A_LONG]* 100.0f)) + (cg_change2)/100.0f;
            
        

         simulate_model(&inputs,&states,&outputs,&atmosphere,dt,freeze);
         inputs.inps[U_A_LONG] = save_pitch;
         set_stick_pos = inputs.inps[U_A_LONG];
         setupct ++;
         innerct ++;
      }//end of intenal loop


   }//end of trim loop

/*
   trimed_pitchpos = inputs.inps[U_A_LONG];
   IO_Sen_out->pitch_demand_position = inputs.inps[U_A_LONG];
   IO_Sen_out->roll_demand_position  = inputs.inps[U_A_LAT];
   IO_Sen_out->rudder_demand_position = 0.0f;
   IO_Sen_out->pitch_trim_demand_position = inputs.inps[U_A_LONG];
*/

   if(setupct >= 100001)
   {
      DY_trimed = TRUE;
      printf("\n left map %f right map %f",outputs.user[Y_A_L_MAN_P],outputs.user[Y_A_R_MAN_P]);
      //set_stick_pos = inputs.inps[U_A_LONG];
   }

}

//----------------------------------------------------------------------

void set_inputs(void)
{

   atmosphere.delta_T = (float) (DY_temp - ISA_TEMPERATURE);

   //NOTE SET delta_p to QNH and leave alt inputs at ISA_PRESURE. QNH has to be worked out for local airfields
   //QNH = 1mb = 27.23 ft so RW ht of 1000ft and QFE of 1000mb QNH is 1000(ft)/27.23 = 36.724
   //so QNH is QFE + (RW height/27.23) so QNH = 1036.724 so delta_p = (1036.724 - 1013.25) * 100.0

   // diff between ISA_PRESURE and real QNH in pascals
   atmosphere.delta_p = (float)((DY_new_pressure - ISA_PRESURE) * mb2Pa);
   atmosphere.delta_r = 0.0;
   DY_last_pressure = DY_new_pressure;

   atmosphere.ht      = terrain_height;
   

   mod_pitch = IO_Sen_in->control_yoke_pitch_position;

   if(mod_pitch < pitch_stop)
   {
      mod_pitch = pitch_stop;
   }

      

   cg_change2 = table1D(cg_aoa,cg_mov,3,col_al * R2D);

   kick_timer += delta_time;
   if(kick_timer > 6.0f)
   {
      if(kick_timer > 6.5f)
         kick_timer = 0.0f;
   }
   else
      cg_change2 = 0.0f;

   cg_change2 = 0.0f;

   inputs.inps[U_A_LAT]       = (((IO_Sen_in->control_yoke_roll_position + Turb_vD)/100) * -1);
   inputs.inps[U_A_LONG]      = (table1D(pitch_input,pitch_output,3,mod_pitch)) + (cg_change2)/100.0f;

   inputs.inps[U_A_LONG] += (Turb_vE / 100.0f);
   inputs.inps[U_A_PED]       = ((IO_Sen_in->rudder_pedal_position )/100);//(IO_Sen_in->rudder_pedal_position/200);
   inputs.inps[U_A_AIL_TRIM]  = 0.0; 
   inputs.inps[U_A_ELE_TRIM]  = (IO_Sen_in->pitch_trim_wheel/-100);
   inputs.inps[U_A_RUD_TRIM]  = (IO_Sen_in->rudder_trim_wheel/200);
   inputs.inps[U_A_DEBUG_P1]  = 1.0;
   inputs.inps[U_A_DEBUG_P2]  = 1.0;
   inputs.inps[U_A_DEBUG_P3]  = 1.0;
   inputs.inps[U_A_DEBUG_P4]  = 1.0;


   if(RC_scale.throt_scale[RC_LEFT] > 0.0)
      inputs.inps[U_A_L_THROT]  =
      (((IO_Sen_in->left_throttle - RC_scale.throt_zero[RC_LEFT])*RC_scale.throt_scale[RC_LEFT])/100);
   else
      inputs.inps[U_A_L_THROT]  = (IO_Sen_in->left_throttle/100);
   inputs.inps[U_A_L_THROT]  = (IO_Sen_in->left_throttle/100);

   if(inputs.inps[U_A_L_THROT] > 1.0) inputs.inps[U_A_L_THROT] = 1.0;
   if(inputs.inps[U_A_L_THROT] < 0.0) inputs.inps[U_A_L_THROT] = 0.0;


   if(RC_scale.throt_scale[RC_RIGHT] > 0.0)
      inputs.inps[U_A_R_THROT]  = 
      (((IO_Sen_in->right_throttle- RC_scale.throt_zero[RC_RIGHT])*RC_scale.throt_scale[RC_RIGHT])/100);
   else
      inputs.inps[U_A_R_THROT]  = (IO_Sen_in->right_throttle/100);
   inputs.inps[U_A_R_THROT]  = (IO_Sen_in->right_throttle/100);

   if(inputs.inps[U_A_R_THROT] > 1.0) inputs.inps[U_A_R_THROT] = 1.0;
   if(inputs.inps[U_A_R_THROT] < 0.0) inputs.inps[U_A_R_THROT] = 0.0;
   

   if(IO_Sen_in->yokes_trim_down_button == SW_ON || IO_Sen_in->yokes_trim_up_button == SW_ON)
   {
      pitch_trim_on = TRUE;
   }
   else
      pitch_trim_on = FALSE;
   // fuel starvation fault close fuel switch

   if (inputs.dinps[U_D_FAIL_L_FUEL] == TRUE)
   {
      //inputs.dinps[U_D_L_XFEED] = 1;
      left_fuel_close -= (float)(delta_time * FUEL_STARVE_RATE);
   }
   else
      left_fuel_close = 0.0;

   // fuel starvation fault close fuel switch
   if (inputs.dinps[U_D_FAIL_R_FUEL] == TRUE)
   {
      right_fuel_close -= (float)(delta_time * FUEL_STARVE_RATE);
      //inputs.dinps[U_D_R_XFEED] = 1;
   }
   else
      right_fuel_close = 0.0;


RC_scale.mix_scale[RC_LEFT] = -1.0;
   if(RC_scale.mix_scale[RC_LEFT] > 0.0) 
      inputs.inps[U_A_L_MIX]  =
      (((IO_Sen_in->left_engine_mixture - RC_scale.mix_zero[RC_LEFT])*RC_scale.mix_scale[RC_LEFT])/100);
   else
      inputs.inps[U_A_L_MIX]  = ((IO_Sen_in->left_engine_mixture/100) + left_fuel_close);
   
   if(RC_scale.mix_scale[RC_RIGHT] > 0.0)
      inputs.inps[U_A_R_MIX] =
      (((IO_Sen_in->right_engine_mixture - RC_scale.mix_zero[RC_RIGHT])*RC_scale.mix_scale[RC_RIGHT])/100);
   else
      inputs.inps[U_A_R_MIX] = ((IO_Sen_in->right_engine_mixture/100) + right_fuel_close);

   if(inputs.inps[U_A_L_MIX] > 1.0) inputs.inps[U_A_L_MIX] = 1.0;
   if(inputs.inps[U_A_L_MIX] < 0.0) inputs.inps[U_A_L_MIX] = 0.0;
   if(inputs.inps[U_A_R_MIX] > 1.0) inputs.inps[U_A_R_MIX] = 1.0;
   if(inputs.inps[U_A_R_MIX] < 0.0) inputs.inps[U_A_R_MIX] = 0.0;
   


   //find greater left toebrake position and pass to flight model
if(IO_Sen_in->panel_parking_brake == SW_OFF)
{
   if(IO_Sen_in->copilot_left_toebrake > IO_Sen_in->pilot_left_toebrake)
   {
      inputs.inps[U_A_L_BRAKE] =  (IO_Sen_in->copilot_left_toebrake/100);
   }
   else
      inputs.inps[U_A_L_BRAKE] =  (IO_Sen_in->pilot_left_toebrake/100);

   //find greater right toebrake position and pass to flight model
   if(IO_Sen_in->copilot_right_toebrake > IO_Sen_in->pilot_right_toebrake)
   {
      inputs.inps[U_A_R_BRAKE] = (IO_Sen_in->copilot_right_toebrake/100);
   }
   else
      inputs.inps[U_A_R_BRAKE]  = (IO_Sen_in->pilot_right_toebrake/100);
}

   inputs.inps[U_A_ALT_P1A] = 1013.00;
   inputs.inps[U_A_ALT_P1B] = 1013.00;
   inputs.inps[U_A_ALT_P2]  = 1013.00; //   inputs.inps[U_A_ALT_P2]  = 1012.25; //Valor original
   DY_last_pressure = DY_new_pressure;


   inputs.inps[U_A_MASS] = (float)(DY_weight.DryWeight + DY_weight.m_iPAXR1 +DY_weight.m_iBaggage + DY_weight.m_iPAXR2 +
                                   DY_weight.m_iPilotWeight + ((outputs.user[Y_A_L_FUEL] * 6.0) +
                                   (outputs.user[Y_A_R_FUEL] * 6.0)));

   alpha_error = BUF_START - (col_al * R2D);
   if(alpha_error > ZERO)
      alpha_error = ZERO;

   if(alpha_error < -5.0f)
      alpha_error = -5.0f;
   cg_dem = alpha_error * sw_neg;

   cg_change1 = 1.0f;

   if(cg_dem > 0.01f)
   {
      if(cg_change < cg_dem)
      {
         cg_change += delta_time * cg_change1;
      }
      else
         sw_neg = 1.0f;
   }
   else
   if(cg_dem < 0.01f)
   {
      if(cg_change > cg_dem)
      {
         cg_change -= delta_time * cg_change1;
      }
      else
         sw_neg = -1.0f;
   }
   else
   {
      sw_neg = 1.0f;
      if(cg_change > 0.01f)
      {
         cg_change -= delta_time * cg_change1;
      }
      if(cg_change < 0.01f)
      {
         cg_change += delta_time * cg_change1;
      }
   }


/*
   if((col_al * R2D) > 15.5f)
   {
      if(cg_change2 < 7.5f && cg_change2 > -7.5f)
      {
         cg_change2 += delta_time * cg_change1;
      }
   }
   else
   {
      if(cg_change2 > 0.01f)
         cg_change2 -= delta_time;
      else
      if(cg_change2 < -0.01f)
         cg_change2 += delta_time;
   }
*/
//static float cg_aoa[3] = {-100.0f,15.5f,26.0f};
//static float cg_mov[3] = {90.0,90.0,80.0}

   

   inputs.inps[U_A_CG] = dum_cg + cg_change;;//cg_change2;//calc_cofg(); 90.0f + cg_change2
   DY_total_weight = inputs.inps[U_A_MASS];
   test_cofg   = inputs.inps[U_A_CG];

   inputs.inps[U_A_L_INIT_FUEL] = DY_weight.m_cLeftFuel;
   inputs.inps[U_A_R_INIT_FUEL] = DY_weight.m_cRightFuel;
   DY_calcedcofg = calc_cofg();
   
   

   //discrete inputs


   
if(EL_output.power == TRUE)
{
/*   
   if(IO_Sen_in->flap_control_lever < 2)
   {
      inputs.dinps[U_D_FLAP] =  0;
   }
   else   
      if(IO_Sen_in->flap_control_lever > 8 && IO_Sen_in->flap_control_lever < 12 )
      {
         inputs.dinps[U_D_FLAP] =  1;
      }
      else
         if(IO_Sen_in->flap_control_lever > 23 && IO_Sen_in->flap_control_lever < 27)
         {
            inputs.dinps[U_D_FLAP] =  2;
         }
         else
            if(IO_Sen_in->flap_control_lever > 37)
            {
               inputs.dinps[U_D_FLAP] =  3;
            }
*/            
   inputs.inps[U_A_FLAP] = IO_Sen_in->flap_control_lever;

}
else
inputs.inps[U_A_FLAP] = inputs.inps[U_A_FLAP];

   if(IO_Sen_in->elec_left_start_switch == SW_ON && EL_output.power == TRUE)
   {
      inputs.dinps[U_D_START] = -1;
      DY_l_start = TRUE;
      DY_r_start = FALSE;
   }
   else
      if(IO_Sen_in->elec_right_start_switch == SW_ON && EL_output.power == TRUE)
      {
         inputs.dinps[U_D_START] = 1;
         DY_r_start = TRUE;
         DY_l_start = FALSE;

      }
      else
      {
         inputs.dinps[U_D_START] = FALSE;
         DY_l_start = FALSE;
         DY_r_start = FALSE;
         
      }

   if(IO_Sen_in->elec_left_magneto_1_switch == SW_OFF)
   {
      inputs.dinps[U_D_L_MAG_L] = TRUE;
   }
   else
      inputs.dinps[U_D_L_MAG_L] = FALSE;

   if(IO_Sen_in->elec_right_magneto_1_switch == SW_OFF)
   {
      inputs.dinps[U_D_L_MAG_R] = TRUE;
   }
   else
      inputs.dinps[U_D_L_MAG_R] = FALSE;

   if(IO_Sen_in->elec_left_magneto_2_switch == SW_OFF)
   {
      inputs.dinps[U_D_R_MAG_L] = TRUE;
   }
   else
      inputs.dinps[U_D_R_MAG_L] = FALSE;

   if(IO_Sen_in->elec_right_magneto_2_switch == SW_OFF)
   {
      inputs.dinps[U_D_R_MAG_R] = TRUE;
   }
   else
      inputs.dinps[U_D_R_MAG_R] = FALSE;
 
   inputs.dinps[U_D_L_BOOST] = IO_Sen_in->elec_left_fuel_switch == FUEL_HI;
   inputs.dinps[U_D_R_BOOST] = IO_Sen_in->elec_right_fuel_switch == FUEL_HI;

   inputs.dinps[U_D_LOW_L_BOOST] = IO_Sen_in->elec_left_fuel_switch == FUEL_LO;
   inputs.dinps[U_D_LOW_R_BOOST] = IO_Sen_in->elec_right_fuel_switch == FUEL_LO;

   if(IO_Sen_in->elec_left_primer_button == SW_ON && EL_output.power == TRUE)
   {
      inputs.dinps[U_D_L_PRIME] = TRUE;
   }
   else
      inputs.dinps[U_D_L_PRIME] = FALSE;

   if(IO_Sen_in->elec_right_primer_button == SW_ON && EL_output.power == TRUE)
   {
      inputs.dinps[U_D_R_PRIME] = TRUE;
   }
   else
      inputs.dinps[U_D_R_PRIME] = FALSE;
   
//   inputs.dinps[U_D_R_PRIME] = IO_Sen_in->elec_right_primer_button;
   //printf("\n PRIME_SWITCH %d",IO_Sen_in->elec_right_primer_button);
   inputs.inps[U_A_L_COWL] = (float) (IO_Sen_in->left_cowl_flap_control/100.0);
   inputs.inps[U_A_R_COWL] = (float) (IO_Sen_in->right_cowl_flap_control/100.0);
   

   //FUEL SWITCH HAS TO GO THROUGH OFF TO GO FROM XFEED TO ON SO PUT IN DELAY
   //TO STOP ENG CUT OUT
   if(IO_Sen_in->left_fuel_select != FUEL_OFF)
   {
      LFuelDelay = 0.0;
   }
   
   if(LFuelDelay <= FUEL_DELAY && IO_Sen_in->left_fuel_select == FUEL_OFF)
   {
      LFuelDelay += delta_time;
   }

   if(IO_Sen_in->right_fuel_select != FUEL_OFF)
   {
      RFuelDelay = 0.0;
   }
   
   if(RFuelDelay <= FUEL_DELAY && IO_Sen_in->right_fuel_select == FUEL_OFF)
   {
      RFuelDelay += delta_time;
   }


   if(IO_Sen_in->left_fuel_select == FUEL_OFF && LFuelDelay > FUEL_DELAY)
   {
      inputs.dinps[U_D_L_XFEED] = 1;
   }
   else if(IO_Sen_in->left_fuel_select == FUEL_ON)
   {
      inputs.dinps[U_D_L_XFEED] = 0;
   }
   else if(IO_Sen_in->left_fuel_select == FUEL_XFEED)
   {
      inputs.dinps[U_D_L_XFEED] = 2;
   }
   else
      inputs.dinps[U_D_L_XFEED] = inputs.dinps[U_D_L_XFEED];


   if(IO_Sen_in->right_fuel_select == FUEL_OFF && RFuelDelay > FUEL_DELAY)
   {
      inputs.dinps[U_D_R_XFEED] = 1;
   }
   else if(IO_Sen_in->right_fuel_select == FUEL_ON)
   {
      inputs.dinps[U_D_R_XFEED] = 0;
   }
   else if(IO_Sen_in->right_fuel_select == FUEL_XFEED)
   {
      inputs.dinps[U_D_R_XFEED] = 2;
   }
   else
      inputs.dinps[U_D_R_XFEED] = inputs.dinps[U_D_R_XFEED];

   //source switch is not active so set it to normal 
   inputs.dinps[U_D_S_SOURCE] = 0;

  if(IO_Sen_in->panel_parking_brake == SW_ON)
  {
     //inputs.dinps[U_D_PARK] = TRUE;
     if(inputs.inps[U_A_L_BRAKE] < (IO_Sen_in->pilot_left_toebrake/100.0f))
     {
        inputs.inps[U_A_L_BRAKE] = IO_Sen_in->pilot_left_toebrake/100.0f;
     }
     else
     if(inputs.inps[U_A_L_BRAKE] < (IO_Sen_in->copilot_left_toebrake/100.0f))
     {
        inputs.inps[U_A_L_BRAKE] = IO_Sen_in->copilot_left_toebrake/100.0f;
     }

     if(inputs.inps[U_A_R_BRAKE] < (IO_Sen_in->pilot_right_toebrake/100.0f))
     {
        inputs.inps[U_A_R_BRAKE] = IO_Sen_in->pilot_right_toebrake/100.0f;
     }
     else
     if(inputs.inps[U_A_R_BRAKE] < (IO_Sen_in->copilot_right_toebrake/100.0f))
     {
        inputs.inps[U_A_R_BRAKE] = IO_Sen_in->copilot_right_toebrake/100.0f;
     }

     if((inputs.inps[U_A_R_BRAKE] > 0.6) && (inputs.inps[U_A_L_BRAKE] > 0.6))
     {
        inputs.dinps[U_D_PARK] = TRUE;
     }


  }
  else
     inputs.dinps[U_D_PARK] = FALSE;

#ifndef AP_NOT_ACTIVE
  inputs.dinps[U_D_KFC150_PWR] = ((IO_avionic_pwr == TRUE) && (IO_Sen_faults.AVF.auto_pilot == FALSE) && (IO_Sen_in->MCB_autopilot_tripped == SW_ON));

if(SE_freeze_on == FALSE) //stop autopilot stuff in freeze mode
{
  if(IO_Sen_in->yokes_autopilot_disconnect_button == SW_ON)
  {
     inputs.dinps[U_D_AP_DISC] = TRUE;
  }
  else
  {
     inputs.dinps[U_D_AP_DISC] = FALSE;
  }

  if(IO_Sen_in->yokes_CWS_switch == SW_ON)
  {
     inputs.dinps[U_D_CWS] = TRUE;
  }
  else
     inputs.dinps[U_D_CWS] = FALSE;

  if(IO_Sen_in->yokes_trim_up_button == SW_ON)
  {
     inputs.dinps[U_D_ELEC_TRIM] = TRUE;
  }
  else
     if(IO_Sen_in->yokes_trim_down_button == SW_ON)
     {
        inputs.dinps[U_D_ELEC_TRIM] = -1;
     }
     else
        inputs.dinps[U_D_ELEC_TRIM] = FALSE;

     // KFC150 Autopilot stuff >> DARREN
     // Flight director

     if(IO_Sen_in->KFC150_FD_button == SW_ON)
     {
        inputs.dinps[U_D_KFC150_FD] = TRUE;
     }
     else
        inputs.dinps[U_D_KFC150_FD] = FALSE;

     // Altitude hold switch
     if(IO_Sen_in->KFC150_ALT_button == SW_ON)
     {
        inputs.dinps[U_D_KFC150_ALT] = TRUE;
     }
     else
        inputs.dinps[U_D_KFC150_ALT] = FALSE;

     // Heading select switch
     if(IO_Sen_in->KFC150_HDG_button == SW_ON)
     {
        inputs.dinps[U_D_KFC150_HDG] = TRUE;
     }
     else
        inputs.dinps[U_D_KFC150_HDG] = FALSE;

     // Navigation switch
     if(IO_Sen_in->KFC150_NAV_button == SW_ON)
     {
        inputs.dinps[U_D_KFC150_NAV] = TRUE;
     }
     else
        inputs.dinps[U_D_KFC150_NAV] = FALSE;

     // Approach switch
     if(IO_Sen_in->KFC150_APR_button == SW_ON)
     {
        inputs.dinps[U_D_KFC150_APR] = TRUE;
     }
     else
        inputs.dinps[U_D_KFC150_APR] = FALSE;

     // Back course switch
     if(IO_Sen_in->KFC150_BC_button == SW_ON)
     {
        inputs.dinps[U_D_KFC150_BC] = TRUE;
     }
     else
        inputs.dinps[U_D_KFC150_BC] = FALSE;

     // Self test switch
     if(IO_Sen_in->KFC150_TEST_button == SW_ON)
     {
        inputs.dinps[U_D_KFC150_TEST] = TRUE;
     }
     else
        inputs.dinps[U_D_KFC150_TEST] = FALSE;

     // Autopilot engage switch
     if(IO_Sen_in->KFC150_AP_ENG_button == SW_ON)
     {
        inputs.dinps[U_D_KFC150_AP_ENG] = TRUE;
     }
     else
        inputs.dinps[U_D_KFC150_AP_ENG] = FALSE;

     // Autopilot vertical trim switch
     if(IO_Sen_in->KFC150_UP_button == SW_ON)
     {
        inputs.dinps[U_D_KFC150_VT] = TRUE;
     }
     else
        if(IO_Sen_in->KFC150_DOWN_button == SW_ON)
        {
           inputs.dinps[U_D_KFC150_VT] = -1;
        }
        else
           inputs.dinps[U_D_KFC150_VT] = FALSE;
     // END of KFC150 Autopilot stuff << DARREN

   if(IO_Sen_in->panel_FD_pilot_copilot_switch == FD_PILOT)
   {
      inputs.inps[U_A_HDG_ERR] = -IO_Sen_in->pilot_hsi_heading_error;
      //pass cdi as - 0.5 tp 0.5
      inputs.inps[U_A_CDI]    = (float)(IO_Sen_out->pilot_hsi_left_right_needle/100.0);
      nav_valid = IO_Sen_out->pilot_hsi_nav_flag == HIDE_FLAG;
      inputs.inps[U_A_HDG] = IO_Sen_in->pilot_hsi_obs;
      inputs.inps[U_A_GS_DEV] = IO_Sen_out->pilot_hsi_GS_needle;
      gs_valid = IO_Sen_out->pilot_hsi_GS_enable;
      hdg_off  = IO_Sen_out->pilot_hsi_hdg_flag == SHOW_FLAG;
      head_error = (float)fabs(IO_Sen_out->pilot_hsi_compass_demand_position - IO_Sen_in->pilot_hsi_obs);
   }
   else
   {
      inputs.inps[U_A_HDG_ERR] = -IO_Sen_in->copilot_hsi_heading_error;
      //pass cdi as - 0.5 tp 0.5
      inputs.inps[U_A_CDI]    = (float)(IO_Sen_out->copilot_hsi_left_right_needle/100.0) ;
      nav_valid = IO_Sen_out->copilot_hsi_nav_flag == HIDE_FLAG;
      inputs.inps[U_A_HDG] = IO_Sen_in->copilot_hsi_obs;
      inputs.inps[U_A_GS_DEV] = IO_Sen_out->copilot_hsi_GS_needle;
      gs_valid = IO_Sen_out->copilot_hsi_GS_enable;
      hdg_off  = IO_Sen_out->copilot_hsi_hdg_flag == SHOW_FLAG;
      head_error = (float)fabs(IO_Sen_out->copilot_hsi_compass_demand_position - IO_Sen_in->copilot_hsi_obs);
   }

   fd_switch = IO_Sen_in->panel_FD_pilot_copilot_switch;

   //inputs.inps[U_A_GS_DEV] = 0.0;

   trim_warn = (IO_Sen_faults.AVF.trim_runaway || IO_Sen_faults.AVF.trim_fail || IO_Sen_in->MCB_pitch_trim_tripped == SW_OFF);
}   //End of SE_freeze_on
/*
   if(IO_Sen_in->panel_FD_pilot_copilot_switch == FD_PILOT)
   {
      inputs.inps[U_A_HDG_ERR] = -IO_Sen_in->pilot_hsi_heading_error;
      //pass cdi as - 0.5 tp 0.5
      inputs.inps[U_A_CDI]    = (float)(IO_Sen_out->pilot_hsi_left_right_needle/100.0);
      //nav_valid = !IO_Sen_out->pilot_hsi_nav_flag;
   }
   else
   {
      inputs.inps[U_A_HDG_ERR] = -IO_Sen_in->copilot_hsi_heading_error;
      //pass cdi as - 0.5 tp 0.5
      inputs.inps[U_A_CDI]    = (float)(IO_Sen_out->copilot_hsi_left_right_needle/100.0) ;
      //nav_valid = !IO_Sen_out->copilot_hsi_nav_flag;
   }
*/
   
#endif     //Of AP_NOT_ACTIVE


        //system and instrument failures
        inputs.dinps[U_D_S1_BLOCKAGE]    = IO_Sen_faults.FI.STATIC_BLOCK;
        inputs.dinps[U_D_S2_BLOCKAGE]    = IO_Sen_faults.FI.STATIC_BLOCK;
        inputs.dinps[U_D_P_BLOCKAGE]     = IO_Sen_faults.FI.PITOT_BLOCK;
        inputs.dinps[U_D_FAIL_L_SUCTION] = IO_Sen_faults.FI.Left_SUCTION_PUMP;
        inputs.dinps[U_D_FAIL_L_FUEL]    = IO_Sen_faults.FF.l_fuel_starv;
        inputs.dinps[U_D_FAIL_L_MAG_L]   = IO_Sen_faults.FF.l_mag_l_fail;
        inputs.dinps[U_D_FAIL_L_MAG_R]   = IO_Sen_faults.FF.l_mag_r_fail;
        inputs.dinps[U_D_FAIL_L_OIL_T]   = IO_Sen_faults.FF.l_oil_temp;
        inputs.dinps[U_D_FAIL_L_OIL_P]   = IO_Sen_faults.FF.l_oil_press;
        inputs.dinps[U_D_FAIL_L_CHT]     = IO_Sen_faults.FF.l_cht_high;
        inputs.dinps[U_D_FAIL_L_PROP]    = IO_Sen_faults.FF.l_prop_cntrl;
        inputs.dinps[U_D_FAIL_L_DEICE]   = IO_Sen_faults.FF.l_prop_deice;
        inputs.dinps[U_D_L_MAP_DRAIN]    = (IO_Sen_in->panel_left_manifold_gauge_drain_button == SW_ON);
        inputs.dinps[U_D_LEFT_FUEL_PUMP_FAIL] = IO_Sen_faults.FF.l_fuel_pump;

        inputs.dinps[U_D_FAIL_R_SUCTION] = IO_Sen_faults.FI.Right_SUCTION_PUMP;
        inputs.dinps[U_D_FAIL_R_FUEL]    = IO_Sen_faults.FF.r_fuel_starv;
        inputs.dinps[U_D_FAIL_R_MAG_L]   = IO_Sen_faults.FF.r_mag_l_fail;
        inputs.dinps[U_D_FAIL_R_MAG_R]   = IO_Sen_faults.FF.r_mag_r_fail;
        inputs.dinps[U_D_FAIL_R_OIL_T]   = IO_Sen_faults.FF.r_oil_temp;
        inputs.dinps[U_D_FAIL_R_OIL_P]   = IO_Sen_faults.FF.r_oil_press;
        inputs.dinps[U_D_FAIL_R_CHT]     = IO_Sen_faults.FF.r_cht_high;
        inputs.dinps[U_D_FAIL_R_PROP]    = IO_Sen_faults.FF.r_prop_cntrl;
        inputs.dinps[U_D_FAIL_R_DEICE]   = IO_Sen_faults.FF.r_prop_deice;
        inputs.dinps[U_D_R_MAP_DRAIN]    = (IO_Sen_in->panel_right_manifold_gauge_drain_button == SW_ON);
        inputs.dinps[U_D_DEICE]          = IO_Sen_out->panel_wing_tail_lamp;
        inputs.dinps[U_D_AIRFRAME_ICE]   = IO_Sen_faults.AF.airframe_icing;
        inputs.dinps[U_D_RIGHT_FUEL_PUMP_FAIL] = IO_Sen_faults.FF.r_fuel_pump;


        if(IO_Sen_faults.FF.r_mag_l_hot)
        {
           inputs.dinps[U_D_R_MAG_L] = TRUE;
        }

        if(IO_Sen_faults.FF.r_mag_r_hot)
        {
           inputs.dinps[U_D_R_MAG_R] = TRUE;
        }

        if(IO_Sen_faults.FF.l_mag_l_hot)
        {
           inputs.dinps[U_D_L_MAG_L] = TRUE;
        }

        if(IO_Sen_faults.FF.l_mag_r_hot)
        {
           inputs.dinps[U_D_L_MAG_R] = TRUE;
        }

        inputs.dinps[U_D_PROP_ICE]       = inputs.dinps[U_D_AIRFRAME_ICE];
        inputs.dinps[U_D_PROP_DEICE]     = (IO_Sen_in->panel_propellor_deice_switch == SW_ON  && EL_output.power == TRUE);


        
   inputs.inps[U_A_L_RPM]  = (float)(IO_Sen_in->left_prop_rpm/100.0);
   inputs.inps[U_A_R_RPM] = (float)(IO_Sen_in->right_prop_rpm/100.0);
   // prop sync control if within limits set both rpm levers the same
   if (IO_Sen_in->throttle_prop_sync_switch != PROP_OFF) 
   {
      if (IO_Sen_in->right_prop_rpm > (IO_Sen_in->left_prop_rpm - PROP_LEVER_LIMIT)
         && IO_Sen_in->right_prop_rpm < (IO_Sen_in->left_prop_rpm + PROP_LEVER_LIMIT))
      {
         inputs.inps[U_A_R_RPM] = inputs.inps[U_A_L_RPM];
      }
   }

   if(inputs.inps[U_A_L_RPM] > 1.0) inputs.inps[U_A_L_RPM] = 1.0;
   if(inputs.inps[U_A_L_RPM] < 0.0) inputs.inps[U_A_L_RPM] = 0.0;
   if(inputs.inps[U_A_R_RPM] > 1.0) inputs.inps[U_A_R_RPM] = 1.0;
   if(inputs.inps[U_A_R_RPM] < 0.0) inputs.inps[U_A_R_RPM] = 0.0;


        if(IO_Sen_in->MCB_landing_gear_pump_tripped == SW_OFF  
           || IO_Sen_faults.AF.land_gear_retrat == TRUE ||
           (outputs.duser[Y_D_N_CONTACT] == TRUE && outputs.duser[Y_D_R_GEAR])
            || IO_Sen_in->MCB_landing_gear_control_tripped == SW_OFF)
        {
            
           inputs.dinps[U_D_FAIL_GEAR_PUMP] = TRUE;
        }
        else
           inputs.dinps[U_D_FAIL_GEAR_PUMP] = FALSE;

        inputs.dinps[U_D_GEAR] = IO_Sen_in->panel_landing_gear_lever;

        if(IO_Sen_in->MCB_pitch_trim_tripped == SW_OFF)
           inputs.dinps[U_D_FAIL_PITCH_TRIM] = TRUE;
        else
           inputs.dinps[U_D_FAIL_PITCH_TRIM] = FALSE;


        if(IO_Sen_in->MCB_electric_flaps_tripped == SW_OFF || IO_Sen_faults.AF.flaps == TRUE )
           inputs.dinps[U_D_FAIL_FLAP_MOTOR] = TRUE;
        else
           inputs.dinps[U_D_FAIL_FLAP_MOTOR] = FALSE;

        if(IO_Sen_in->panel_emergency_gear_switch == SW_ON)
        {
           inputs.dinps[U_D_GEAR_EMERG] = TRUE;
        }
        else
           inputs.dinps[U_D_GEAR_EMERG] = FALSE;

        if(IO_Sen_faults.AF.land_gear_ext == TRUE)
        {
           inputs.dinps[U_D_NOSE_GEAR_EXTEND_FAIL] = TRUE;
        }
        else
           inputs.dinps[U_D_NOSE_GEAR_EXTEND_FAIL] = FALSE;

}

//----------------------------------------------------------------------

static void conditional_fail(void)
{
   if(DY_con_fail.LeftEngFail > 0)
   {
      //printf("\n Left con fails");
      switch (DY_con_fail.LeftEngFail)
      {
      case 1 : 
         if((DY_con_fail.LeftFailValue) <= (-states.D - terrain_height))
         {
            left_eng_fail = TRUE;
         }
         break;

      case 2 :

         if((DY_con_fail.LeftFailValue) <= (outputs.Vtas * MS_TO_KTS))
         {
            left_eng_fail = TRUE;
         }
         break;

      case 3 :
         // store time first time then decrement to zero at zero fail engine
         if(last_LconFail != DY_con_fail.LeftEngFail)
         {
            left_failtimer = (float)(DY_con_fail.LeftFailValue * 60.0);
         }

         if(left_failtimer > 0.0)
         {
            left_failtimer -= delta_time;
         }
         else
         {
            left_eng_fail = TRUE;
         }

         break;
      }
   }
   else
      left_eng_fail = FALSE;
      
   last_LconFail = DY_con_fail.LeftEngFail;


   if(DY_con_fail.RightEngFail > 0)
   {
      
      switch (DY_con_fail.RightEngFail)
      {
      case 1 : 
         if((DY_con_fail.RightFailValue) <= (-states.D - terrain_height))
         {
            right_eng_fail = TRUE;
         }
         break;

      case 2 :

         if((DY_con_fail.RightFailValue) <= (outputs.Vtas * MS_TO_KTS))
         {
            right_eng_fail = TRUE;
         }
         break;

      case 3 :
         // store time first time then decrement to zero at zero fail engine
         //printf("\n R TIME");
         if(last_RconFail != DY_con_fail.RightEngFail)
         {
            right_failtimer = (float)(DY_con_fail.RightFailValue * 60.0);
         }

         if(right_failtimer > 0.0)
         {
            right_failtimer -= delta_time;
         }
         else
         {
            right_eng_fail = TRUE;
         }

         break;
      }
   }
   else
      right_eng_fail = FALSE;


   if(right_eng_fail == TRUE)
   {
      inputs.dinps[U_D_FAIL_R_MAG_L] = TRUE;
      inputs.dinps[U_D_FAIL_R_MAG_R] = TRUE;
      //inputs.dinps[U_D_R_ENGINE_SEIZE] = TRUE;
   }
   else
   {
      inputs.dinps[U_D_R_ENGINE_SEIZE] = FALSE;
   }
      
   if(left_eng_fail == TRUE)
   {
      inputs.dinps[U_D_FAIL_L_MAG_L] = TRUE;
      inputs.dinps[U_D_FAIL_L_MAG_R] = TRUE;
      //inputs.dinps[U_D_L_ENGINE_SEIZE] = TRUE;
   }
   else
   {
      inputs.dinps[U_D_L_ENGINE_SEIZE] = FALSE;
   }

   last_RconFail = DY_con_fail.RightEngFail;
   last_LconFail = DY_con_fail.LeftEngFail;
}

//----------------------------------------------------------------------

float calc_cofg(void)
{
//position in inches of weight areas in respect to center of gravity
#define AIRCRAFT_DATUM  86.7f
#define PILOTS_DATUM   118.1f
#define PAX1_DATUM     119.1f
#define PAX2_DATUM     157.6f
#define FUEL_DATUM      93.6f
#define BAGAGE_DATUM   178.7f
#define AIRCRAFT_WEIGHT


   float aircraft_moment,total_moment;
   float fuel_moment,bagage_moment = 0.0;
   float pilots_moment,pax1_moment,pax2_moment;
   float total_weight,cofg;


// calculate individual moments.
   aircraft_moment = (float)(AIRCRAFT_DATUM * DryWeight);
   pilots_moment   = (float)(PILOTS_DATUM * PilotWeight);
   pax1_moment     = (float)(PAX1_DATUM * Pax1Weight);
   pax2_moment     = (float)(PAX2_DATUM * Pax2Weight);
   fuel_moment     = (float)(FUEL_DATUM * ((outputs.user[Y_A_L_FUEL] * 6.0) + (outputs.user[Y_A_R_FUEL] * 6.0)));
   bagage_moment   = (float)(BAGAGE_DATUM * BaggageWeight);
//calculate total weight and total moment
   if(DryWeight < 0.0) DryWeight = 3222.0;
   total_weight = (float)(DryWeight + Pax1Weight + Pax2Weight + PilotWeight + BaggageWeight +((outputs.user[Y_A_L_FUEL] * 6.0) + (outputs.user[Y_A_R_FUEL] * 6.0)));
   total_moment = aircraft_moment + pilots_moment + pax1_moment + pax2_moment + fuel_moment + bagage_moment;
//center of gravity is total moments over total weight;

   cofg = (float)(total_moment/total_weight);

   return cofg;
}

//----------------------------------------------------------------------

