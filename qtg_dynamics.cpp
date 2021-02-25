//File: qtg_dynamics.cpp
//Prefix: QD_

//Description: calls Seneca flight model and places h/w inputs into flight model structures



/***************************/
/*   Revision Record       */
/***************************/

/*
$Header: C:/master/seneca/RCS/qtg_dynamics.cpp 1.6 2000/05/31 12:47:28 colinj Exp $
$Log: qtg_dynamics.cpp $
Revision 1.6  2000/05/31 12:47:28  colinj
Comment out AIRSPEED_TRIM and MULTI_TRIM defines.

Revision 1.5  2000/05/26 11:25:17  colinj
Changes to support multi engine trim.

Revision 1.4  2000/05/25 07:53:03  colinj
Changes to support new flight model in qtg_reset_dynamics.

Revision 1.3  2000/05/10 16:40:44  colinj
*** empty log message ***

Revision 1.2  2000/05/10 12:35:37  colinj
Changes to support new flight model.

Revision 1.1  2000/04/14 16:37:53  colinj
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
#include "fltdy\fms_s3fm.h"
#include "fltdy\fms_defs.h"
#include "define.h"
#include "const.h"
#include "faults.h"
#include "config.h"
#include "qtgdefs.h"
#include "dat_stru\in_head.h"
#include "dat_stru\out_head.h"
#include "nav\dyn_ios.h"

//--------------------
// Defines
//--------------------

#define RW_QTG 1
#define T0 288.15
#define L 0.0065
#define P0 101325.0

#define VIS_LAT  36.749695f
#define VIS_LONG -6.061749f
#define ZERO 0.00f
#define BUF_START 17.0f
#define MAX_ROLL_DELTA 10.0f
//#define AIRSPEED_TRIM 1
//#define MULTI_TRIM 1 // define to do hugely bigly loop
//--------------------
// External Routines  
//--------------------

extern void UT_offset_along_dir (float x1, float y1, float z1, float h,
                                 float p, float dist, float *x, float *y, float *z);
extern void UT_local_xy_2_ll(float delta_x, float delta_y, double ref_lat, double ref_lon,
                             double *lat1, double *lon1);

extern void set_inputs();

//extern void DC_dcl_control(void); //Woody Mod
//extern void QC_control(void); //Woody Mod
//--------------------
// External Variables 
//--------------------


extern sen_in_struct  *IO_Sen_in;
extern OUTPUTin			*OUTPUTinbox;//simulinkudp
extern sen_out_struct *IO_Sen_out;
extern OUTPUTS         outputs;
extern STATES          states;
extern INPUTS          inputs;
extern ATMOS           atmosphere;
extern QTG             qtg;
extern QTGfastlog      IO_fastLog;

extern float h0;
extern float delta_time;
extern float terrain_height;
extern float dt;
extern float dum_cg;
extern float col_al;
extern float pitch_input[]; 
extern float pitch_output[];
extern float set_stick_pos;


extern int QC_manualmode;
extern int qtg_ManualMode;	//Woody Mod

extern int RR_runway_setup_ok;
extern int RR_rw_setup_mode;
extern int SE_hz;
extern int dyn_state;
extern int test_screen;
extern char no_dynamics_flag;

//----------------
// Globals
//----------------

//Jerez position
double QD_ref_lat   = 36.754269;
double QD_ref_lon   = -6.055406;

float QD_dist_td       = 1200.0;
float vis_runway_head  =    0.0;
float last_vis_heading =  -10.0;
float ave_roll_rate    =    0.0f;
float power_time       = 0.0f;


int QD_trimed = FALSE;
int qtg_freeze = 0;
int QD_do_fog = FALSE;
int QD_night  = FALSE;

char QD_first_reset = TRUE;
char slam_pitch = FALSE;
char slam_roll = FALSE;
char slam_yaw = FALSE;
//-----------------
// Local Variables
//-----------------

INPUTS  qtg_inputs;
OUTPUTS qtg_outputs;
ATMOS   qtg_atmosphere;
STATES  qtg_states;

static float real_pitch_position = 0.0;
static float real_roll_position  = 0.0;
static float trim_pitch_stick    = 0.0;
static float drf_timer           = 0.0;
static float drf1_timer          = 0.0;
static float cg_change;
static float td_x = 0.0,td_y     = 0.0;
static float trimed_pitchpos     = 0.0;

static float trimed_pedpos       = 0.0; //Woody Mod
static float trimed_rollpos      = 0.0; //Woody Mod

static float flapDelayTimer      = 0.0;
static float phi_error = 0.0f,phi = 0.0f;
static float i_phi_error = 0.0f,roll_servo_dem = 0.0f;
static float theta_dem           = 5.0;
static float gamma_int           = 0.0;
static float roll_int            = 0.0;
static float roll_pos            = 0.0;
//float col_err             = 0.90f;
float col_err             = 0.20f;
static float fsec                = 0.0;
static float alpha_error = ZERO;
static float cg_dem = ZERO,sw_neg = 1.0f;
static float cg_change1 = -1.0f;
static float spiral_timer = 0.0f;
static float save_roll = 0.0f;
static float roll_im_timer = 0.0f;
static float i_psi_error = 0.0f,last_psi_error = 0.0f; 
static float d_psi_error = 0.0f,roll_dem = 0.0f,psi_error = 0.0f;
static float i_roll_error = 0.0f,roll_error = 0.0f;
static float slam_ct = 0.0f;
static float total_roll_rate = 0.0f,roll_rate_timer = 0.0f;
static float throt_timer = 0.0f;
static unsigned long int airspeedct = 0;
static unsigned long int setupct    = 0;

static int free_pitch		 = FALSE;
static int free_roll		 = FALSE;
static int free_flaps		 = FALSE;		//Woody Mod
static int free_Throttle_Box = FALSE;		//Woody Mod
static int free_all_Controls = FALSE;		//Woody Mod
static int free_Elev		 = FALSE;		//Woody Mod
static int free_Ail			 = FALSE;		//Woody Mod
static int free_Rudd		 = FALSE;		//Woody Mod		

static char free_Gear   = FALSE;

static char dutch_roll_flag      = FALSE;
static char temp_delay_holdspeed = FALSE;
static char temp_bank_hold       = FALSE;
static char stepFlapDn           = FALSE;
static char stepFlapUp           = FALSE;
static char impulse              = FALSE;
static char roll_impulse         = FALSE; 
static char last1_up = SW_OFF;
static char last1_dn = SW_OFF;
static char stall_flag = FALSE;
static char roll_rate = FALSE;
static char done_ang = FALSE;
static char hold_head = FALSE;
static char slam_closed = FALSE;
static char slam_open   = FALSE;
static char emer_gear   = FALSE;
static char roll_rate_arm = FALSE;

//--------------------
// Forward References 
//--------------------
void qtg_reset_dynamics();
void qtg_set_inputs();

//----------------------------------------------------------------------

void qtg_reset_dynamics(void)
{
   memset(&qtg_inputs,0,sizeof(INPUTS));
   memset(&qtg_states,0,sizeof(STATES));
   memset(&qtg_outputs,0,sizeof(OUTPUTS));

   h0 = 0.5f;

//reset all special case flags
   stepFlapDn           = FALSE;
   stepFlapUp           = FALSE;
   impulse              = FALSE;
   free_pitch           = FALSE;
   free_roll            = FALSE;
   roll_rate            = FALSE;
   dutch_roll_flag      = FALSE;
   temp_delay_holdspeed = FALSE;
   temp_bank_hold       = FALSE;
   stepFlapDn           = FALSE;
   stepFlapUp           = FALSE;
   impulse              = FALSE;
   roll_impulse         = FALSE; 
   stall_flag           = FALSE;
   done_ang             = FALSE;
   hold_head            = FALSE;
   slam_closed          = FALSE;
   slam_open            = FALSE;
   slam_pitch           = FALSE;
   slam_roll            = FALSE;
   slam_yaw             = FALSE;
   slam_ct              = 0.0f;
   emer_gear            = FALSE;
   roll_rate_arm        = FALSE;
   ave_roll_rate        = 0.0f;
   total_roll_rate      = 0.0f;
   roll_rate_timer      = 0.0f;
   throt_timer          = 0.0f;
   power_time           = 0.0f;

   free_Throttle_Box	= FALSE;		//Woody Mod
   free_all_Controls	= FALSE;		//Woody Mod
   free_Elev			= FALSE;		//Woody Mod
   free_Ail				= FALSE;		//Woody Mod
   free_Rudd			= FALSE;		//Woody Mod
   
   free_flaps			= FALSE;		//Woody Mod
   free_Gear			= FALSE;		//Woody Mod

   theta_dem = 5.0;
   setupct   = 0;
   airspeedct = 0;

   double height, North, East, lat, lon;
   float  speed,heading,theta,gamma,phi;
   float  origin_x, origin_y, origin_z;
   float  rw_head;
   float  gslope;
   float  dist;
   float  start_x, start_y, start_z;
   double start_lat, start_lon;
   temp_delay_holdspeed = FALSE;
   temp_bank_hold       = FALSE;
   dutch_roll_flag      = FALSE;
   drf_timer  = 0.0f;
   drf1_timer = 0.0f;

	init_model(NULL);

float iohz;
	iohz = (float) SE_hz;
	if (iohz < 20.0)
		 iohz =20.0;
	else
	if (iohz > 200 )
		 iohz = 200.0;

	dt = delta_time;
   




   if(QD_first_reset == TRUE)
   {
      origin_x = 0.0;
      origin_y = 0.0;
      origin_z = 3.0;
      rw_head  = (float) qtg.heading ;
      vis_runway_head = (float) qtg.heading;
      gslope   = 0.0;
      dist     = 350;

  //    if(test_screen)
//         qtg.heading -= 1;
      UT_offset_along_dir (origin_x, origin_y, origin_z, rw_head, -gslope, dist,
               &td_x, &td_y, &start_z);
      

   }


   origin_x = td_x;
   origin_y = td_y;
   origin_z = 3.0;
   rw_head  =(float)  qtg.heading ;
   vis_runway_head = (float) qtg.heading;
   gslope   = 3.0;
   dist     = (float) QD_dist_td;

   UT_offset_along_dir (origin_x, origin_y, origin_z, rw_head, -gslope, -dist,
                        &start_x, &start_y, &start_z);

   
   UT_local_xy_2_ll(start_x, start_y, QD_ref_lat, QD_ref_lon,
                    &start_lat, &start_lon);

   if(no_dynamics_flag == TRUE)
   {
      height  = start_z;
      if(test_screen)
         height  = terrain_height + 2.5;
   }
   else
   if(qtg.altitude < 10)
   {
      height  = terrain_height + 2.5;
   }
   else
   {
      height  = (qtg.altitude/FT_IN_METRE + terrain_height); //units //  COLIN MAKE CHANGES FOR RUNWAY HEIGHT
   }
	
	North   = 0.0;
	East    = 0.0;
	speed   = (float) (qtg.airspeed * KTS_TO_MS);
	heading = (float) (qtg.heading * DEG_TO_RAD);
	theta   =0.0;// qtg.pitch * D2R; 
	gamma   =0.0;// qtg.yaw * D2R;   
   lat     = start_lat * DEG_TO_RAD;
   lon     = start_lon * DEG_TO_RAD;
	phi     =0.0;// qtg.roll * D2R;

   qtg_set_inputs();

   real_pitch_position = IO_Sen_in->control_yoke_pitch_position;
   real_roll_position  = IO_Sen_in->control_yoke_roll_position;


   qtg_inputs.inps[U_A_L_THROT]   = (float) 1.0;//0.50;
   qtg_inputs.inps[U_A_R_THROT]   = (float) 1.0;//0.50;
   if(qtg.airspeed < 10)
   {
      qtg_inputs.inps[U_A_L_BRAKE]   = (float) 1.0;
      qtg_inputs.inps[U_A_R_BRAKE]   = (float) 1.0;
      qtg_inputs.dinps[U_D_PARK]     = 1;
      speed = 0.0;
      qtg_inputs.inps[U_A_L_THROT]   = (float) 0.0f;//0.50;
      qtg_inputs.inps[U_A_R_THROT]   = (float) 0.0f;//0.50;

   }
   else
   {
      qtg_inputs.inps[U_A_L_BRAKE]   = (float) 0.0;
      qtg_inputs.inps[U_A_R_BRAKE]   = (float) 0.0;
      qtg_inputs.dinps[U_D_PARK]     = 0;
   }

   qtg_inputs.dinps[U_D_R_XFEED]  = 0;
   qtg_inputs.dinps[U_D_L_XFEED]  = 0;

   qtg_inputs.inps[U_A_L_RPM]     = (float) (((qtg.leftrpm * RPM2RPS) - rpm_lo) / (rpm_hi - rpm_lo));
   if(qtg_inputs.inps[U_A_L_RPM] < 0.0) qtg_inputs.inps[U_A_L_RPM] = 0.0;

   qtg_inputs.inps[U_A_R_RPM]     = (float) (((qtg.rightrpm * RPM2RPS) - rpm_lo) / (rpm_hi - rpm_lo));
   if(qtg_inputs.inps[U_A_R_RPM] < 0.0) qtg_inputs.inps[U_A_R_RPM] = 0.0;

   qtg_inputs.inps[U_A_L_MIX]     = (float) 1.0;//qtg_inputs.inps[U_A_L_MIX];
   qtg_inputs.inps[U_A_R_MIX]     = (float) 1.0;//qtg_inputs.inps[U_A_R_MIX];

   qtg_inputs.inps[U_A_L_INIT_FUEL] = (float)qtg.fueltankleft;
   qtg_inputs.inps[U_A_R_INIT_FUEL] = (float)qtg.fueltankright;

   qtg_inputs.dinps[U_D_LIVE_START] = TRUE;

   if(test_screen)
   {
      lat = VIS_LAT * DEG_TO_RAD;
      lon = VIS_LONG * DEG_TO_RAD;
      terrain_height = 0.0f;
      height = ((84.0f/FT_IN_METRE )+ terrain_height);
      //terrain_height = (float)height -2.5f;
      //theta = 0.4f;
      qtg_atmosphere.ht      = terrain_height;  
   }

   qtg_atmosphere.ht      = terrain_height; 
   if(QD_first_reset == TRUE)
   {
      RR_runway_setup_ok = FALSE;
      RR_rw_setup_mode = RW_QTG;
      QD_first_reset = FALSE;
   }

//   qtg.heading = 90; 
      heading = qtg.heading * DEG_TO_RAD;;

   if(qtg.testid == 18 || qtg.testid == 17)
   {
      lat     = start_lat * DEG_TO_RAD;
      lon     = start_lon * DEG_TO_RAD;
      heading = 180.0f * DEG_TO_RAD;
      height = terrain_height + 2.0f;
      qtg_inputs.dinps[U_D_GEAR] = 1;
      speed = 0.01f;
      if(qtg.testid == 17)
         qtg_inputs.dinps[U_D_LIVE_START] = FALSE;

   }

	reset_model(height,North,East,lat,lon,speed,heading,theta,gamma,phi,&qtg_inputs,&qtg_states,&qtg_outputs,&qtg_atmosphere);

   
   if ( (qtg.testid >= 69) && (qtg.testid <= 72) )
   {
      if(qtg.testid == 69)
      {
         QD_do_fog = TRUE;
      }
      else
         QD_do_fog = FALSE;

      qtg_outputs.theta = -2.5f * DEG_TO_RAD;
   }
   
      




   

         memcpy(&outputs,&qtg_outputs,sizeof(OUTPUTS));
         memcpy(&states,&qtg_states,sizeof(STATES));
         memcpy(&inputs,&qtg_inputs,sizeof(INPUTS));
         memcpy(&atmosphere,&qtg_atmosphere,sizeof(ATMOS));


   //last_vis_heading = vis_runway_head;

   
}

 

void qtg_simulate()
{
	float iohz;
	iohz = (float) SE_hz;
	if (iohz < 20.0)
		 iohz = 20.0;
	else
	if (iohz > 200.0 )
		 iohz = 200.0;
	dt= delta_time;//0.02;//1/iohz;
	fsec += dt; /* update real-time clock */
   qtg_set_inputs();



/*
   if((qtg.testid > 9) && (qtg.testid < 16)) 
   {
      qtg_inputs.inps[U_A_LAT]  = (float) 0.0;
      qtg_inputs.inps[U_A_LONG] = (float) 0.2;
      qtg_inputs.inps[U_A_PED]  = (float) 0.0;
   }
*/

   if(qtg.testid > 62 && qtg.testid < 66)
   {
      qtg_inputs.inps[U_A_LONG] = trimed_pitchpos;
      qtg_inputs.inps[U_A_LAT] = 0.0f;
      qtg_inputs.inps[U_A_PED] = 0.0f;//IO_Sen_in->rudder_pedal_position/100.0f;

   }

   if(slam_pitch)
   {
      if(slam_ct > 0.25)
         qtg_inputs.inps[U_A_LONG] = -1.0f;
      else
         slam_ct += delta_time;
   }

   if(slam_yaw)
   {
      if(slam_ct > 0.25)
         qtg_inputs.inps[U_A_PED] = -1.0f;
      else
         slam_ct += delta_time;

   }

   if(slam_roll)
   {
      if(slam_ct > 0.25)
         qtg_inputs.inps[U_A_LAT] = 1.0f;
      else
         slam_ct += delta_time;

   }

      float save_pitch = qtg_inputs.inps[U_A_LONG];

      if(qtg_inputs.inps[U_A_LONG] < -0.9f)
      {
         qtg_inputs.inps[U_A_LONG] = -0.9f;

      }
      float cg_change2 = 0.0f;
      qtg_inputs.inps[U_A_LONG]      = (table1D(pitch_input,pitch_output,3,qtg_inputs.inps[U_A_LONG]* 100.0f)) + (cg_change2)/100.0f;


      qtg_inputs.inps[U_A_PED] = qtg_inputs.inps[U_A_PED]/2.0f;

      if(qtg.testid != 18 && qtg.testid != 17)
	      simulate_model(&qtg_inputs,&qtg_states,&qtg_outputs,&qtg_atmosphere,dt,qtg_freeze);

      qtg_inputs.inps[U_A_LONG] = save_pitch;

      qtg_inputs.inps[U_A_PED] = qtg_inputs.inps[U_A_PED] * 2.0f;   


   
   memcpy(&outputs,&qtg_outputs,sizeof(OUTPUTS));
   memcpy(&states,&qtg_states,sizeof(STATES));
   memcpy(&inputs,&qtg_inputs,sizeof(INPUTS));
   memcpy(&atmosphere,&qtg_atmosphere,sizeof(ATMOS));

   if(free_pitch)
   {
      outputs.user[Y_A_PITCH_NULL] = trimed_pitchpos;
   }
   else
      outputs.user[Y_A_PITCH_NULL] = inputs.inps[U_A_LONG];


   inputs.inps[U_A_LAT] = qtg_inputs.inps[U_A_LAT];

// Woody Mod	/////////////////////////////////////////////////////////////////////////////////
/*
//////////////////with this in test's 2biva, 2bviia,b, 2bviiia,b will not run(the trim moves with stick)   
   if(free_Elev)
   {
      outputs.user[Y_A_PITCH_NULL] = trimed_pitchpos;//real_pitch_position;
	 
   }
   else
   {
      outputs.user[Y_A_PITCH_NULL] = inputs.inps[U_A_LONG];
   }


*/
  if(free_Ail)
	{
		outputs.user[Y_A_ROLL_SERVO] = trimed_rollpos;//qtg_inputs.inps[U_A_LAT];//real_roll_position;
		
	}
	else
	{
		outputs.user[Y_A_ROLL_SERVO] = inputs.inps[U_A_LAT];
	}
	


  if(free_Rudd)
	{
		outputs.user[Y_A_PEDAL_NULL] = trimed_pedpos;//qtg_inputs.inps[U_A_PED];
		
	}
	else
	{
		outputs.user[Y_A_PEDAL_NULL] = inputs.inps[U_A_PED];
	}

	if(free_flaps)
   {
   	   //qtg_inputs.inps[U_A_FLAP] = qtg.flaps * 1.0f;
       qtg_inputs.inps[U_A_FLAP] = IO_Sen_in->flap_control_lever/40.0f;
   }



// Woody Mod	/////////////////////////////////////////////////////////////////////////////////
   
/*
   if(qtg.testid == 47)
   {
      inputs.inps[U_A_LAT] = 0.0f;
   }
*/
   //printf("\n RPM L %f R %f",states.user[X_L_RPM],states.user[X_R_RPM]);
   //printf("\n THROTTLE %f RPM %f",qtg_inputs.inps[U_A_L_THROT],qtg_inputs.inps[U_A_R_THROT]);
   //printf("\n ROLL RATE %f",(outputs.phi_dot * R2D));
   

   if(qtg.testid == 18 || qtg.testid == 17)
   {
      set_inputs();
      inputs.inps[U_A_LONG] = IO_Sen_in->control_yoke_pitch_position/100.0f;
      memcpy(&qtg_inputs,&inputs,sizeof(INPUTS));
      qtg_freeze = FREEZE_RB_ONLY;
      simulate_model(&qtg_inputs,&qtg_states,&qtg_outputs,&qtg_atmosphere,dt,qtg_freeze);
      memcpy(&outputs,&qtg_outputs,sizeof(OUTPUTS));
      memcpy(&states,&qtg_states,sizeof(STATES));
      memcpy(&inputs,&qtg_inputs,sizeof(INPUTS));
      memcpy(&atmosphere,&qtg_atmosphere,sizeof(ATMOS));
   }

   if(qtg_ManualMode == FALSE &&(qtg.testid == 40 || qtg.testid == 41)) //Woody Mod
   {
      inputs.inps[U_A_LAT] = 0.0f;
   }


}

void qtg_trim_flight(void)
{
   
   float dt = (float) 0.01;

   qtg_freeze = 0;
   

   int freeze = (FREEZE_NED + FREEZE_LATLON);
   
#ifdef MULTI_TRIM
   int map_demand = 0;
   FILE *stream;
   FILE *stream1;

   stream=fopen("airspeed.dat","wt");
   stream1=fopen("trimres.dat","wt");
   fprintf(stream1,"  ALT  RPM  MAP   IAS   TAS  THETA   ROC   LONG      TRIM \n\n");
   fclose(stream1);
   fprintf(stream," rudder  roll  qtg_outputs.psi_dot    qtg_outputs.phi_dot  qtg_states.p  qtg_outputs.phi \n\n");
   fclose(stream);

int alt;   
   qtg.weight = (int)4450;

   for(alt = 1000;alt <= 5000;alt += 5000)
   {
    
      qtg.altitude = 1000;
      
      if(alt == 1000)
      {
         qtg.airspeed = 100;
         qtg.altitude = 3000;
      }
      else
      if(alt == 3000)
      {
         qtg.airspeed = 100;
         qtg.altitude = 3000;
      }
      else
      if(alt == 5000)
      {
         qtg.airspeed = 100;
         qtg.altitude = 3000;
      }

      for(qtg.leftrpm = 2600; qtg.leftrpm < 2900;qtg.leftrpm += 200)
      {
         qtg.rightrpm = qtg.leftrpm;
         qtg_reset_dynamics();
      
         for(map_demand = 17;map_demand <= 40;map_demand += 50)
         {


            qtg_reset_dynamics();
      
            //qtg.leftmap = 0.00f;//map_demand;
            //qtg.rightmap = map_demand;// 0.00f;

            qtg.leftmap = map_demand;
            qtg.rightmap = map_demand;
/*
            if(qtg.leftmap == 0)
            {
               qtg.leftrpm = 0;
            }

            if(qtg.rightmap == 0)
            {
               qtg.rightrpm = 0;
            }
            //qtg.leftmap = map_demand;
*/
            //qtg.leftmap  =   40;
            //qtg.rightmap =    4;
            //qtg.leftrpm  = 2600;
            //qtg.rightrpm = 2600;

            qtg_inputs.inps[U_A_L_RPM]     = (float) (((qtg.leftrpm * RPM2RPS) - rpm_lo) / (rpm_hi - rpm_lo));
            if(qtg_inputs.inps[U_A_L_RPM] < 0.0) qtg_inputs.inps[U_A_L_RPM] = 0.0;

            qtg_inputs.inps[U_A_R_RPM]     = (float) (((qtg.rightrpm * RPM2RPS) - rpm_lo) / (rpm_hi - rpm_lo));
            if(qtg_inputs.inps[U_A_R_RPM] < 0.0) qtg_inputs.inps[U_A_R_RPM] = 0.0;
         
            setupct = 0;
            dt = (float) 0.01;
            gamma_int = (float) 0.0;
            qtg_outputs.user[Y_A_TRIM_COST] = 10000.0;
    QD_trimed = FALSE;
   while(setupct < 100001) 
   {
      if(qtg_outputs.user[Y_A_TRIM_COST] < 1.0)
      {
         setupct = 100001;
         QD_trimed = TRUE;
      }

      qtg_inputs.dinps[U_D_GEAR] = 1;
      //qtg_inputs.dinps[U_D_FLAP] =  2;
      qtg_inputs.inps[U_A_L_COWL] = (float)0.50;//(float) (qtg.leftcowlflap/100.0);
      qtg_inputs.inps[U_A_R_COWL] = (float)0.50;//(float) (qtg.rightcowlflap/100.0);

      qtg_inputs.inps[Y_A_PED] = qtg_inputs.inps[Y_A_PED]/2.0f;
      simulate_model(&qtg_inputs,&qtg_states,&qtg_outputs,&qtg_atmosphere,dt,freeze);
     // TRIM MAP to demanded
      qtg_inputs.inps[U_A_L_THROT] += (float) (0.1e-3 * (qtg.leftmap - qtg_outputs.user[Y_A_L_MAN_P]));
      if(qtg_inputs.inps[U_A_L_THROT] < 0.0) qtg_inputs.inps[U_A_L_THROT] = 0.0;
      if(qtg_inputs.inps[U_A_L_THROT] > 1.0) qtg_inputs.inps[U_A_L_THROT] = 1.0;
      // TRIM MAP to demanded
      qtg_inputs.inps[U_A_R_THROT] += (float) (0.1e-3 * (qtg.rightmap - qtg_outputs.user[Y_A_R_MAN_P]));
      if(qtg_inputs.inps[U_A_R_THROT] < 0.0) qtg_inputs.inps[U_A_R_THROT] = 0.0;
      if(qtg_inputs.inps[U_A_R_THROT] > 1.0) qtg_inputs.inps[U_A_R_THROT] = 1.0;

#ifdef AIRSPEED_TRIM //TRIM TO SPECIFIC AIRSPEED
      if(qtg.airspeed != 0)
      {
         theta_dem +=   (float) (-1.0e-4 * (qtg.airspeed - qtg_outputs.user[Y_A_P1_ASI]));
//         qtg_inputs.inps[U_A_LONG]  = (float) ( (-1.0 * (theta_dem * D2R - qtg_outputs.theta)) + (1.0 * qtg_states.q));
         //qtg_inputs.inps[U_A_LONG]  = (float) ( (-5.0 * (theta_dem * D2R - qtg_outputs.theta)) + (1.0 * qtg_states.q));
         qtg_inputs.inps[U_A_LONG]  = (float) ( (-10.0 * (theta_dem * D2R - qtg_outputs.theta)) + (1.0 * qtg_states.q));
         if(qtg_inputs.inps[U_A_LONG] > 1.0) qtg_inputs.inps[U_A_LONG] = 1.0;
         if(qtg_inputs.inps[U_A_LONG] < -1.0) qtg_inputs.inps[U_A_LONG] = -1.0;
      }
#else  // TRIM TO GET ZERO RATE OF CLIMB
         gamma_int += (float)(qtg_outputs.gamma * dt);
         qtg_inputs.inps[U_A_LONG]  = (float) ((2.0 * gamma_int) + (50.0 * (qtg_outputs.gamma)) + (10.0 * qtg_states.q));
         if(qtg_inputs.inps[U_A_LONG] > 1.0) qtg_inputs.inps[U_A_LONG] = 1.0;
         if(qtg_inputs.inps[U_A_LONG] < -1.0) qtg_inputs.inps[U_A_LONG] = -1.0;
#endif
//trimer to get single engine flight

/*
         if(qtg.leftmap != qtg.rightmap)
         {
            qtg_inputs.inps[U_A_PED]  = (float)(10.0 * qtg_outputs.psi_dot);  ///qtg_outputs.psi_dot));
            if(qtg_inputs.inps[U_A_PED] > 1.0) qtg_inputs.inps[U_A_PED] = 1.0;
            if(qtg_inputs.inps[U_A_PED] < -1.0) qtg_inputs.inps[U_A_PED] = -1.0;

            roll_int += (float)(qtg_outputs.phi_dot * dt);
            qtg_inputs.inps[U_A_LAT]  = (float) ((2.0 * roll_int) + (25.0 * (qtg_outputs.phi)) +  (20.0 * qtg_states.p));
            if(qtg_inputs.inps[U_A_LAT] > 1.0) qtg_inputs.inps[U_A_LAT] = 1.0;
            if(qtg_inputs.inps[U_A_LAT] < -1.0) qtg_inputs.inps[U_A_LAT] = -1.0;
         }
         else
         {
            qtg_inputs.inps[U_A_PED] = 0.0;
            qtg_inputs.inps[U_A_LAT] = 0.0;
         }*/
         if(qtg.leftmap != qtg.rightmap)
         {
            //set pedel to achieve zero rate of change of heading
            qtg_inputs.inps[U_A_PED]  = (float)(100.0 * qtg_outputs.user[Y_A_SLIP_ANGLE]);//qtg_outputs.psi_dot);
            if(qtg_inputs.inps[U_A_PED] > 1.0) qtg_inputs.inps[U_A_PED] = 1.0;
            if(qtg_inputs.inps[U_A_PED] < -1.0) qtg_inputs.inps[U_A_PED] = -1.0;
            //set ailerons to achieve zero rate of roll angle change
            roll_int += (float)(qtg_outputs.psi_dot * dt);
            qtg_inputs.inps[U_A_LAT]  = (float) ((30.0 * qtg_outputs.psi_dot));//roll_int) + (25.0 * (qtg_outputs.psi)) +  (20.0 * qtg_states.p));
            if(qtg_inputs.inps[U_A_LAT] > 1.0) qtg_inputs.inps[U_A_LAT] = 1.0;
            if(qtg_inputs.inps[U_A_LAT] < -1.0) qtg_inputs.inps[U_A_LAT] = -1.0;
         }/*
         else
         {
            qtg_inputs.inps[U_A_LAT] = 0.0;
            qtg_inputs.inps[U_A_PED] = 0.0;
         }*/

         //stream=fopen("airspeed.dat","at");
         //fprintf(stream,"\n%+06.2f %+06.2f %+06.2f %+06.2f %+06.2f %+06.2f",
         //qtg_inputs.inps[U_A_PED], qtg_inputs.inps[U_A_LAT],  qtg_outputs.psi_dot,    qtg_outputs.phi_dot  ,qtg_states.p  ,qtg_outputs.phi);
         //fclose(stream);






      setupct ++;

   }//end of inner trim loop
   stream1=fopen("trimres.dat","at");
   fprintf(stream1,"%5.0f %4.0f %4.1f %5.1f %5.1f %+06.2f %+5.0f %+6.3f %9.1f %5.1f %5.2f  %3.1f %3.1f\n",-qtg_states.D / 0.3048,qtg_outputs.user[Y_A_L_RPM],qtg_outputs.user[Y_A_L_MAN_P],qtg_outputs.user[Y_A_P1_ASI],(qtg_outputs.Vt *MS_TO_KTS),(qtg_outputs.theta *  R2D),qtg_outputs.Vt * tan(qtg_outputs.gamma)* 60.0 / 0.3048,qtg_inputs.inps[U_A_LONG],qtg_outputs.user[Y_A_TRIM_COST],qtg_outputs.phi_dot * R2D,qtg_outputs.psi_dot * R2D,qtg_inputs.inps[U_A_PED],qtg_inputs.inps[U_A_LAT]);
   fclose(stream1);
   }// end of map change loop
   stream1=fopen("trimres.dat","at");
   fprintf(stream1,"\n");
   fclose(stream1);
   }//end of rpm change loop
   }//end of alt change loop

   //end of nested loops
   fclose(stream);
   trimed_pitchpos = qtg_inputs.inps[U_A_LONG];
   trimed_pedpos = qtg_inputs.inps[U_A_PED];		//Woody Mod
   trimed_rollpos = qtg_inputs.inps[U_A_LAT];		//Woody Mod
   QD_trimed = TRUE;
#else


int qtg_innerct = 0;

   QD_trimed = FALSE;
   if(setupct < 100001) //total loops to do before droping out as a failed trim
   {
      qtg_outputs.user[Y_A_TRIM_COST] = 10000.0; //Set trim cost high to avoid a previous trimed condition droping us out
      while(qtg_innerct < 5) //do 30 loops at one time
      {  // set rpm levers to achieve demanded RPM
         qtg_inputs.inps[U_A_L_RPM]     = (float) (((qtg.leftrpm * RPM2RPS) - rpm_lo) / (rpm_hi - rpm_lo));
         if(qtg_inputs.inps[U_A_L_RPM] < 0.0) qtg_inputs.inps[U_A_L_RPM] = 0.0;

         qtg_inputs.inps[U_A_R_RPM]     = (float) (((qtg.rightrpm * RPM2RPS) - rpm_lo) / (rpm_hi - rpm_lo));
         if(qtg_inputs.inps[U_A_R_RPM] < 0.0) qtg_inputs.inps[U_A_R_RPM] = 0.0;
         
         // allow trim function to drop out if a good trim is achieved
         if(qtg_outputs.user[Y_A_TRIM_COST] < 1.0)
         {
            setupct = 100001;
            qtg_innerct = 100;
            QD_trimed = TRUE;
         }
         float save_pitch = qtg_inputs.inps[U_A_LONG];
         if(qtg_inputs.inps[U_A_LONG] < -0.9f)
         {
            qtg_inputs.inps[U_A_LONG] = -0.9f;

         }

         float cg_change2 = 0.0f;
         qtg_inputs.inps[U_A_LONG]      = (table1D(pitch_input,pitch_output,3,qtg_inputs.inps[U_A_LONG]* 100.0f)) + (cg_change2)/100.0f;

         qtg_inputs.inps[U_A_PED] = qtg_inputs.inps[U_A_PED]/2.0f;

	simulate_model(&qtg_inputs,&qtg_states,&qtg_outputs,&qtg_atmosphere,dt,freeze);

   qtg_inputs.inps[U_A_LONG] = save_pitch;
   qtg_inputs.inps[U_A_PED] = qtg_inputs.inps[U_A_PED]*2.0f;

         //simulate_model(&qtg_inputs,&qtg_states,&qtg_outputs,&qtg_atmosphere,dt,freeze);
//TRIM LEFT MAP to demanded
         qtg_inputs.inps[U_A_L_THROT] += (float) (0.1e-3 * (qtg.leftmap - qtg_outputs.user[Y_A_L_MAN_P]));
         if(qtg_inputs.inps[U_A_L_THROT] < 0.0) qtg_inputs.inps[U_A_L_THROT] = 0.0;
         if(qtg_inputs.inps[U_A_L_THROT] > 1.0) qtg_inputs.inps[U_A_L_THROT] = 1.0;
//TRIM RIGHT MAP to demanded
         qtg_inputs.inps[U_A_R_THROT] += (float) (0.1e-3 * (qtg.rightmap - qtg_outputs.user[Y_A_R_MAN_P]));
         if(qtg_inputs.inps[U_A_R_THROT] < 0.0) qtg_inputs.inps[U_A_R_THROT] = 0.0;
         if(qtg_inputs.inps[U_A_R_THROT] > 1.0) qtg_inputs.inps[U_A_R_THROT] = 1.0;
//For S & Level flight set pitch stick to achieve zero ROC
         if(qtg.flighttrim == SAL)// || qtg.flighttrim == CRUISE)
         {
            gamma_int += (float)(qtg_outputs.gamma * dt);
            qtg_inputs.inps[U_A_LONG]  = (float) ((2.0 * gamma_int) + (50.0 * (qtg_outputs.gamma)) + (10.0 * qtg_states.q));
            if(qtg_inputs.inps[U_A_LONG] > 1.0) qtg_inputs.inps[U_A_LONG] = 1.0;
            if(qtg_inputs.inps[U_A_LONG] < -1.0) qtg_inputs.inps[U_A_LONG] = -1.0;
         }
         else // if not on ground trim to air speed
         if(qtg.flighttrim != ONGND)
         {
      if(qtg.airspeed != 0)
      {
         theta_dem +=   (float) (-1.0e-4 * (qtg.airspeed - qtg_outputs.user[Y_A_P1_ASI]));
//         qtg_inputs.inps[U_A_LONG]  = (float) ( (-1.0 * (theta_dem * D2R - qtg_outputs.theta)) + (1.0 * qtg_states.q));
         //qtg_inputs.inps[U_A_LONG]  = (float) ( (-5.0 * (theta_dem * D2R - qtg_outputs.theta)) + (1.0 * qtg_states.q));
         qtg_inputs.inps[U_A_LONG]  = (float) ( (-10.0 * (theta_dem * D2R - qtg_outputs.theta)) + (1.0 * qtg_states.q));
         if(qtg_inputs.inps[U_A_LONG] > 1.0) qtg_inputs.inps[U_A_LONG] = 1.0;
         if(qtg_inputs.inps[U_A_LONG] < -1.0) qtg_inputs.inps[U_A_LONG] = -1.0;
      }
         }

//trimer to get single engine flight
/*
         if(qtg.leftmap != qtg.rightmap)
         {
            //set pedel to achieve zero rate of change of heading
            qtg_inputs.inps[U_A_PED]  = (float)(10.0 * qtg_outputs.psi_dot);
            if(qtg_inputs.inps[U_A_PED] > 1.0) qtg_inputs.inps[U_A_PED] = 1.0;
            if(qtg_inputs.inps[U_A_PED] < -1.0) qtg_inputs.inps[U_A_PED] = -1.0;
            //set ailerons to achieve zero rate of roll angle change
            roll_int += (float)(qtg_outputs.phi_dot * dt);
            qtg_inputs.inps[U_A_LAT]  = (float) ((2.0 * roll_int) + (25.0 * (qtg_outputs.phi)) +  (20.0 * qtg_states.p));
            if(qtg_inputs.inps[U_A_LAT] > 1.0) qtg_inputs.inps[U_A_LAT] = 1.0;
            if(qtg_inputs.inps[U_A_LAT] < -1.0) qtg_inputs.inps[U_A_LAT] = -1.0;
         }*/

         float craft_head = qtg_outputs.psi * R2D;
         if(craft_head < 0.0f)
            craft_head += 360.0f;
         psi_error = craft_head - qtg.heading;
         qtg_inputs.inps[U_A_LAT]  = (float) ((10.0 * qtg_outputs.psi_dot) + (col_err * psi_error));//roll_int) + (25.0 * (qtg_outputs.psi)) +  (20.0 * qtg_states.p));

         if(qtg_inputs.inps[U_A_LAT] > 1.0) qtg_inputs.inps[U_A_LAT] = 1.0;
         if(qtg_inputs.inps[U_A_LAT] < -1.0) qtg_inputs.inps[U_A_LAT] = -1.0;

         if(qtg.leftmap != qtg.rightmap)
         {
            //set pedel to achieve zero sip angle
            qtg_inputs.inps[U_A_PED]  = (float)(85.0 * qtg_outputs.user[Y_A_SLIP_ANGLE]);//qtg_outputs.psi_dot);
            if(qtg_inputs.inps[U_A_PED] > 1.0) qtg_inputs.inps[U_A_PED] = 1.0;
            if(qtg_inputs.inps[U_A_PED] < -1.0) qtg_inputs.inps[U_A_PED] = -1.0;
         }
         else
         {
            qtg_inputs.inps[U_A_PED] = 0.0;
         }

         setupct ++;
         qtg_innerct ++;
      }//end of intenal loop


   }//end of trim loop


   set_stick_pos = trimed_pitchpos = qtg_inputs.inps[U_A_LONG];
   qtg_outputs.user[Y_A_PITCH_NULL] = qtg_inputs.inps[U_A_LONG];

   if(setupct >= 100001)
   {
      QD_trimed = TRUE;
      printf("\n left map %f right map %f",qtg_outputs.user[Y_A_L_MAN_P],qtg_outputs.user[Y_A_R_MAN_P]);
   }

            

#endif

   memcpy(&outputs,&qtg_outputs,sizeof(OUTPUTS));
   memcpy(&states,&qtg_states,sizeof(STATES));
   memcpy(&inputs,&qtg_inputs,sizeof(INPUTS));
   memcpy(&atmosphere,&qtg_atmosphere,sizeof(ATMOS));


}

void qtg_change_controls(void)
{

   stepFlapDn = FALSE;
   stepFlapUp = FALSE;
   impulse = FALSE;
   roll_impulse = FALSE;
   free_pitch = FALSE;
   free_roll = FALSE;
   free_flaps = FALSE;
   free_Gear = FALSE;
   free_all_Controls = FALSE;
   free_Throttle_Box = FALSE;
   free_Elev = FALSE;
   free_Ail = FALSE;
   free_Rudd = FALSE;

// Woody Mod	/////////////////////////////////////////////////////////////////////////////////

   if(qtg_ManualMode == FALSE &&(qtg.testid == 19 || qtg.testid == 23 || qtg.testid == 24 || qtg.testid == 27 || qtg.testid == 28 || qtg.testid == 43))
	   //Added 23 & 24 Test to here so that pitch is free for the test
   {
	  //qtg.testeventtime = TRUE;
      free_pitch = TRUE;
   }

   if(qtg_ManualMode == TRUE &&(qtg.testid == 19 || qtg.testid == 20 || qtg.testid == 43 || qtg.testid == 143))
   {
	  free_all_Controls = TRUE;
	  free_Throttle_Box = TRUE;
   }

    if(qtg_ManualMode == TRUE &&(qtg.testid == 23 || qtg.testid == 24  || qtg.testid == 25 ||
	   qtg.testid == 26))
   {
	  free_all_Controls = TRUE;
	  free_Throttle_Box = TRUE;
	  free_flaps = TRUE;
	  
   }


   if(qtg_ManualMode == TRUE &&(qtg.testid == 27 || qtg.testid == 28  || qtg.testid == 29 ||
	   qtg.testid == 30))
   {
	  free_all_Controls = TRUE;
	  free_Throttle_Box = TRUE;
	  free_Gear = TRUE;
   }


   if(qtg_ManualMode == FALSE &&(qtg.testid == 40 || qtg.testid == 41))
   {
      free_pitch = TRUE;
   }

   if(qtg_ManualMode == TRUE &&(qtg.testid == 40 || qtg.testid == 41))
   {
	  //qtg.leftrpm  = qtg.et.leftrpm;
	  //qtg.et.aob = 0;
	  free_all_Controls = TRUE;
	  free_Throttle_Box = TRUE;
   }

// Woody Mod	/////////////////////////////////////////////////////////////////////////////////

/*
  printf("\ncontrols where  P %f R %f Y %f\n",qtg_inputs.inps[U_A_LONG]
     ,qtg_inputs.inps[U_A_LAT],qtg_inputs.inps[U_A_PED]);
  // if(qtg.et.columnpos != 0)
//   qtg_inputs.inps[U_A_LONG] = (float) ((qtg.et.columnpos/100.0)*-1);
   
   //qtg_inputs.inps[U_A_PED]  = (float) (qtg.et.pedalpos/100.0);
   printf("\nchanging controls to P %f R %f Y %f\n",qtg_inputs.inps[U_A_LONG]
      ,qtg_inputs.inps[U_A_LAT],qtg_inputs.inps[U_A_PED]);


*/
   //qtg_inputs.inps[U_A_LAT]  = (float) (qtg.et.wheelpos/100.0);
   if(qtg.et.wheelpos > 0)
   {
      roll_impulse = TRUE;
      roll_im_timer = 0.0f;
   }
if(qtg.testid != 31 && qtg.testid != 32)
{
   qtg.flaps = qtg.et.flappos;
   qtg_inputs.inps[U_A_FLAP] = qtg.flaps * 1.0f;
/*   
   if(qtg.et.flappos == 0)
   {
      qtg_inputs.dinps[U_D_FLAP] =  0;
   }
   else   
      if(qtg.et.flappos == 10)
      {
         qtg_inputs.dinps[U_D_FLAP] =  1;
      }
      else
         if(qtg.et.flappos == 25)
         {
            qtg_inputs.dinps[U_D_FLAP] =  2;
         }
         else
            if(qtg.et.flappos == 40)
            {
               qtg_inputs.dinps[U_D_FLAP] =  3;
            }
  */          
}

// Woody Mod	/////////////////////////////////////////////////////////////////////////////////

   if(qtg_ManualMode == FALSE &&(qtg.testid == 31))
   {
//      printf("\n SETTING FLAPS UP FLAG");
      stepFlapUp = TRUE;
      flapDelayTimer = 6.0f;
   }

   if(qtg_ManualMode == TRUE &&(qtg.testid == 31))
   {
      //stepFlapUp = TRUE;
	  free_flaps = TRUE;
      //flapDelayTimer = 6.0f;
   }

   if(qtg_ManualMode == FALSE &&(qtg.testid == 32))
   {
      stepFlapDn = TRUE;
      flapDelayTimer = 6.0f;
   }

   if(qtg_ManualMode == TRUE &&(qtg.testid == 32))
   {
      //stepFlapDn = TRUE;
	  free_flaps = TRUE;
      //flapDelayTimer = 6.0f;
   }

   //if(qtg.testid == 44 || qtg.testid == 45)
   //{
    //  impulse = TRUE;
	  
   //}

// Woody Mod	/////////////////////////////////////////////////////////////////////////////////

	if(qtg_ManualMode == TRUE &&(qtg.testid == 1 || qtg.testid == 2))

	{
	  qtg.testeventtime = TRUE;
	  free_all_Controls = TRUE;
	  free_Throttle_Box = TRUE;
	}
	

	if(qtg_ManualMode == TRUE &&(qtg.testid == 4 || qtg.testid == 5))

	{
	  qtg.testeventtime = TRUE;
	  free_all_Controls = TRUE;
	  free_Throttle_Box = TRUE;
	}

	
	if(qtg_ManualMode == TRUE &&(qtg.testid == 38 || qtg.testid == 39))

	{
	  qtg.testeventtime = TRUE;
	  free_all_Controls = TRUE;
	  free_Throttle_Box = TRUE;
	}
   
	   if(qtg_ManualMode == TRUE &&(qtg.testid == 44 || qtg.testid == 45))
		   
   {
	  qtg.testeventtime = TRUE;
	  free_all_Controls = TRUE;
	  free_Throttle_Box = TRUE;

	}
	  

	   if(qtg_ManualMode == FALSE &&(qtg.testid == 44 || qtg.testid == 45))
		  
   {
	  impulse = TRUE;

	}
	   
//printf("\nQTG Mode: %f ",QC_manualmode);

// Woody Mod	/////////////////////////////////////////////////////////////////////////////////

/*
      if((qtg.airspeed != qtg.et.targetspeed )&& qtg.et.holdspeed == TRUE)
      {
         temp_delay_holdspeed = TRUE;
         qtg.airspeed = qtg.airspeed;
      }
      else
      {
         temp_delay_holdspeed = FALSE;
         
      }
*/
      //qtg.airspeed = qtg.et.targetspeed;

// Woody Mod	/////////////////////////////////////////////////////////////////////////////////


      if(qtg_ManualMode == FALSE &&(qtg.testid == 50 || qtg.testid == 151))
      {
         temp_bank_hold = TRUE;
         qtg.et.holdaob = FALSE;
         spiral_timer = 0.0f;
      }
      else
         temp_bank_hold = FALSE;

	  if(qtg_ManualMode == TRUE &&(qtg.testid == 50 || qtg.testid == 151))
      {
		 qtg.testeventtime = TRUE;
		 free_all_Controls = TRUE;
         temp_bank_hold = FALSE;
         qtg.et.holdaob = FALSE;
         spiral_timer = 0.0f;
      }

      if(qtg_ManualMode == FALSE &&(qtg.testid == 53))	//|| qtg.testid == 56	// Woody Mod
      {
         qtg_inputs.inps[U_A_PED] = qtg.et.pedalpos/100.0f;//-0.25f;
         //if(qtg.testid == 56)	// Woody Mod
         //{					// Woody Mod
         //   hold_head = TRUE;	// Woody Mod
         //}					// Woody Mod
         
      }

	  if(qtg_ManualMode == TRUE &&(qtg.testid == 53))		// Woody Mod
      {
          //qtg_inputs.inps[U_A_PED] = qtg.et.pedalpos/200.0f;//-0.25f;
		  qtg.testeventtime = TRUE;
		  free_all_Controls = TRUE;

	  }



	  if(qtg_ManualMode == FALSE &&(qtg.testid == 56 || qtg.testid == 156 || qtg.testid == 157 || qtg.testid == 158 ))
      {
         qtg_inputs.inps[U_A_PED] = qtg.et.pedalpos/100.0f;//-0.25f;
         if(qtg_ManualMode == FALSE &&(qtg.testid == 56 || qtg.testid == 156 || qtg.testid == 157 || qtg.testid == 158))
         {
            hold_head = TRUE;
         }
         
      }
	  
	  if(qtg_ManualMode == TRUE &&(qtg.testid == 56 || qtg.testid == 156 || qtg.testid == 157 || qtg.testid == 158 ))
      {
        // qtg_inputs.inps[U_A_PED] = qtg.et.pedalpos/100.0f;//-0.25f;
		  qtg.testeventtime = TRUE;
		  free_all_Controls = TRUE;
         if(qtg_ManualMode == TRUE &&(qtg.testid == 56 || qtg.testid == 156 || qtg.testid == 157 || qtg.testid == 158))
         {
            hold_head = FALSE;
         }
         
      }
// Woody Mod	/////////////////////////////////////////////////////////////////////////////////

      if(qtg_ManualMode == FALSE &&(qtg.testid == 54 || qtg.testid == 55))
      {
         dutch_roll_flag = TRUE;
         drf_timer  = 0.0f;
         drf1_timer = 0.0f;
      }
      else
         dutch_roll_flag = FALSE;

     if(qtg_ManualMode == TRUE &&(qtg.testid == 54 || qtg.testid == 55))
      {
         qtg.testeventtime = TRUE;
		 free_all_Controls = TRUE;
         drf_timer  = 0.0f;
         drf1_timer = 0.0f;
      }

// Woody Mod	/////////////////////////////////////////////////////////////////////////////////

      if(qtg_ManualMode == FALSE &&(qtg.testid == 8))
      {
         slam_open = TRUE;
	  }

	  if(qtg_ManualMode == TRUE &&(qtg.testid == 8))
      {
		 slam_open = FALSE;
		 qtg.testeventtime = TRUE;
         free_Throttle_Box = TRUE;
      }


      if(qtg_ManualMode == FALSE &&(qtg.testid == 9))
      {
         slam_closed = TRUE;
      }

	  if(qtg_ManualMode == TRUE &&(qtg.testid == 9))
      {
		 qtg.testeventtime = TRUE;
         free_Throttle_Box = TRUE;
      }

// Woody Mod	/////////////////////////////////////////////////////////////////////////////////
      
	  stall_flag = FALSE;
      qtg.leftmap  = qtg.et.leftmap;
      qtg.rightmap = qtg.et.rightmap;

//      if(qtg.testid != 4 && qtg.testid != 5)
 //     {
         qtg.airspeed = qtg.et.targetspeed;
   //   }
     // else
       //  stall_flag = TRUE;

      qtg.leftrpm  = qtg.et.leftrpm;
      qtg.rightrpm = qtg.et.rightrpm;

      if(qtg.testid != 35)
      {
         qtg.gear     = qtg.et.gearpos;
      }
      else
      {
         emer_gear = TRUE;
      }

      qtg.roll     = qtg.et.aob;

   qtg_inputs.inps[U_A_L_RPM]     = (float) (((qtg.et.leftrpm * RPM2RPS) - rpm_lo) / (rpm_hi - rpm_lo));
   if(qtg_inputs.inps[U_A_L_RPM] < 0.0) qtg_inputs.inps[U_A_L_RPM] = 0.0;

   qtg_inputs.inps[U_A_R_RPM]     = (float) (((qtg.et.rightrpm * RPM2RPS) - rpm_lo) / (rpm_hi - rpm_lo));
   if(qtg_inputs.inps[U_A_R_RPM] < 0.0) qtg_inputs.inps[U_A_R_RPM] = 0.0;



//   impulse = FALSE;
   temp_delay_holdspeed = FALSE;

//roll rate tests.

// Woody Mod	/////////////////////////////////////////////////////////////////////////////////

   
   if(qtg_ManualMode == FALSE &&(qtg.testid == 47 || qtg.testid == 48))
   {
      roll_rate = TRUE;
   }
   else
      roll_rate = FALSE;

   if(qtg_ManualMode == TRUE &&(qtg.testid == 47 || qtg.testid == 48))
   {
      free_all_Controls = TRUE;
	  free_Throttle_Box = TRUE;
   }

   if(qtg_ManualMode == FALSE &&(qtg.testid == 49))
   {
      roll_rate = TRUE;
   }

   if(qtg_ManualMode == TRUE &&(qtg.testid == 49))
   {
	  qtg.testeventtime = TRUE;
      free_all_Controls = TRUE;
	  free_Throttle_Box = TRUE;
   }
 
// Woody Mod	/////////////////////////////////////////////////////////////////////////////////
  

}




void qtg_set_inputs(void)
{

	qtg_atmosphere.wN      = 0.0; //wind speed and vel
	qtg_atmosphere.wE      = 0.0;
	qtg_atmosphere.wD      = 0.0;
	qtg_atmosphere.delta_T = (float) (qtg.temperature - 15.0);
	qtg_atmosphere.delta_p = 0.0;// XXX - qtg.pressure;
   qtg_atmosphere.delta_r = 0.0;

	qtg_atmosphere.ht      = terrain_height;  

      if(IO_Sen_in->test_switch_1_up == SW_ON && last1_up == SW_OFF)
      {
         //left_map_test = 0.5f;
         //right_map_test = 0.5f;
         //dum_cg += 0.5f;
//         h0 += 0.05f;
         //map_fix ++;
//         auto_scale += 0.5f;
         col_err += 0.01f;

      }
      else
      if(IO_Sen_in->test_switch_1_dn == SW_ON && last1_dn == SW_OFF)
      {
         //left_map_test = 0.0f;
         //right_map_test = 0.0f;
         //
         //h0 -= 0.05f;
         //auto_scale -= 0.5f;//map_fix --;
         col_err -= 0.01f;
      }

last1_up = IO_Sen_in->test_switch_1_up;
last1_dn = IO_Sen_in->test_switch_1_dn;
   

   ////////////////////////////////Woody Mod/////////////////////////////////////////////


   if(free_flaps == TRUE)
   {
   	   //qtg_inputs.inps[U_A_FLAP] = qtg.flaps * 0.0f;
       qtg_inputs.inps[U_A_FLAP] = IO_Sen_in->flap_control_lever/40.0f;
   }

//	else
//	{

//	  qtg_inputs.inps[U_A_FLAP] = qtg_inputs.inps[U_A_FLAP]; 
//	}

	 //inputs.inps[U_A_FLAP] = IO_Sen_in->flap_control_lever;

	 //inputs.inps[U_A_FLAP] = inputs.inps[U_A_FLAP];

	if(free_Gear == TRUE)
   {
	   //qtg_inputs.dinps[U_D_GEAR]       = IO_Sen_in->panel_landing_gear_lever;
	   //qtg_inputs.dinps[U_D_GEAR_EMERG] = IO_Sen_in->panel_emergency_gear_switch;
	   //inputs.dinps[U_D_GEAR]       = IO_Sen_in->panel_landing_gear_lever;
	   //inputs.dinps[U_D_GEAR_EMERG] = IO_Sen_in->panel_emergency_gear_switch;
		qtg_inputs.dinps[U_D_GEAR] = TRUE;
	   qtg_inputs.dinps[U_D_GEAR_EMERG] = TRUE;
   }
	else
	{
		
	   qtg_inputs.dinps[U_D_GEAR]     = qtg.gear/100;
	   qtg_inputs.dinps[U_D_GEAR_EMERG]   = emer_gear;

	}

   if(free_Throttle_Box == TRUE)
   {
      qtg_inputs.inps[U_A_L_THROT] = IO_Sen_in->left_throttle/100.0f;
	  qtg_inputs.inps[U_A_R_THROT] = IO_Sen_in->right_throttle/100.0f;
	  qtg_inputs.inps[U_A_L_RPM]   = IO_Sen_in->left_prop_rpm/100.0f;
      qtg_inputs.inps[U_A_R_RPM]   = IO_Sen_in->right_prop_rpm/100.0f;       
      qtg_inputs.inps[U_A_L_MIX]   = IO_Sen_in->left_engine_mixture/100.0f;
      qtg_inputs.inps[U_A_R_MIX]   = IO_Sen_in->right_engine_mixture/100.0f;
	  
      qtg_inputs.inps[U_A_L_COWL]  = IO_Sen_in->left_cowl_flap_control/100.0f;
      qtg_inputs.inps[U_A_R_COWL]  = IO_Sen_in->right_cowl_flap_control/100.0f;
	  
   }
   else
   {
		qtg_inputs.inps[U_A_L_THROT]   = qtg_inputs.inps[U_A_L_THROT];
		qtg_inputs.inps[U_A_R_THROT]   = qtg_inputs.inps[U_A_R_THROT];
		qtg_inputs.inps[U_A_L_RPM]     = qtg_inputs.inps[U_A_L_RPM];
		qtg_inputs.inps[U_A_R_RPM]     = qtg_inputs.inps[U_A_R_RPM];       
		qtg_inputs.inps[U_A_L_MIX]     = qtg_inputs.inps[U_A_L_MIX];
		qtg_inputs.inps[U_A_R_MIX]     = qtg_inputs.inps[U_A_R_MIX];

		qtg_inputs.inps[U_A_L_COWL]	= qtg_inputs.inps[U_A_L_COWL];
		qtg_inputs.inps[U_A_R_COWL]	= qtg_inputs.inps[U_A_R_COWL];

   }

   if (free_all_Controls == TRUE)
   {
	   //free_Elev = TRUE;
	   free_pitch = TRUE;
	   free_Ail = TRUE;
	   free_Rudd = TRUE;
	   //qtg_inputs.inps[U_A_LONG]	 = IO_Sen_in->control_yoke_pitch_position/100.0f;
	   //qtg_inputs.inps[U_A_LAT]		 = ((IO_Sen_in->control_yoke_roll_position/100) * -1);
	   //qtg_inputs.inps[U_A_PED]		 = IO_Sen_in->rudder_pedal_position/100.0f;
	   
	   
	   qtg_inputs.inps[U_A_ELE_TRIM] = IO_Sen_in->pitch_trim_wheel/-100.0f; //100.0f;
	   qtg_inputs.inps[U_A_RUD_TRIM] = IO_Sen_in->rudder_trim_wheel/200.0f; //100.0f;

   }
   else
   {
	   
	   //qtg_inputs.inps[U_A_LONG]      = qtg_inputs.inps[U_A_LONG];
	   //qtg_inputs.inps[U_A_LAT]       = qtg_inputs.inps[U_A_LAT];
	   //qtg_inputs.inps[U_A_PED]       = qtg_inputs.inps[U_A_PED];
	
	   qtg_inputs.inps[U_A_ELE_TRIM]  = qtg_inputs.inps[U_A_ELE_TRIM];
	   qtg_inputs.inps[U_A_RUD_TRIM]  = qtg_inputs.inps[U_A_RUD_TRIM];

   }
   
    
if(free_Elev == TRUE)
   {
      qtg_inputs.inps[U_A_LONG] = IO_Sen_in->control_yoke_pitch_position/100.0f;
   }



  if(free_Ail == TRUE)
   {
      qtg_inputs.inps[U_A_LAT] = ((IO_Sen_in->control_yoke_roll_position/100) * -1);

   }


  if(free_Rudd == TRUE)
   {
      qtg_inputs.inps[U_A_PED] = IO_Sen_in->rudder_pedal_position/100.0f;
   }


////////////////////////////////Woody Mod/////////////////////////////////////////////

     if(free_pitch == TRUE)
   {
      qtg_inputs.inps[U_A_LONG] = IO_Sen_in->control_yoke_pitch_position/100.0f;
   }
   else
   

   qtg_inputs.inps[U_A_LONG]      = qtg_inputs.inps[U_A_LONG];


   qtg_inputs.inps[U_A_PED]       = qtg_inputs.inps[U_A_PED];
   qtg_inputs.inps[U_A_AIL_TRIM]  = 0.0; //not found in seneca
   qtg_inputs.inps[U_A_ELE_TRIM]  = qtg_inputs.inps[U_A_ELE_TRIM];
   qtg_inputs.inps[U_A_RUD_TRIM]  = qtg_inputs.inps[U_A_RUD_TRIM];

   qtg_inputs.inps[U_A_L_THROT]   = qtg_inputs.inps[U_A_L_THROT];
   qtg_inputs.inps[U_A_R_THROT]   = qtg_inputs.inps[U_A_R_THROT];
   qtg_inputs.inps[U_A_L_RPM]     = qtg_inputs.inps[U_A_L_RPM];
   qtg_inputs.inps[U_A_R_RPM]     = qtg_inputs.inps[U_A_R_RPM];       
   qtg_inputs.inps[U_A_L_MIX]     = qtg_inputs.inps[U_A_L_MIX];
   qtg_inputs.inps[U_A_R_MIX]     = qtg_inputs.inps[U_A_R_MIX];
   qtg_inputs.inps[U_A_L_BRAKE]   = qtg_inputs.inps[U_A_L_BRAKE];
   qtg_inputs.inps[U_A_R_BRAKE]   = qtg_inputs.inps[U_A_R_BRAKE];
   //qtg_inputs.dinps[U_D_GEAR]     = qtg.gear/100;
   qtg_inputs.dinps[U_D_L_MAG_L]      = TRUE;
   qtg_inputs.dinps[U_D_R_MAG_L]      = TRUE;
   qtg_inputs.dinps[U_D_L_MAG_R]      = TRUE;
   qtg_inputs.dinps[U_D_R_MAG_R]      = TRUE;
   qtg_inputs.dinps[U_D_FAIL_L_MAG_L] = FALSE;
   qtg_inputs.dinps[U_D_FAIL_L_MAG_R] = FALSE;
   qtg_inputs.dinps[U_D_FAIL_R_MAG_L] = FALSE;
   qtg_inputs.dinps[U_D_FAIL_R_MAG_R] = FALSE;
   qtg_inputs.dinps[U_D_GEAR_EMERG]   = emer_gear;



   qtg_inputs.dinps[U_D_START] = 0;
   //airframe
   qtg_inputs.inps[U_A_MASS]        = (float) qtg.weight;


   alpha_error = BUF_START - (float)(col_al * R2D);


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

   qtg_inputs.inps[U_A_CG]          = dum_cg + cg_change;//(float) (qtg.cofg - 10.0);
   qtg_inputs.inps[U_A_L_INIT_FUEL] = (float) qtg.fueltankleft;
   qtg_inputs.inps[U_A_R_INIT_FUEL] = (float) qtg.fueltankright;

// NEED INPUT FROM IOS FROM ALT SETTINGS
   qtg_inputs.inps[U_A_ALT_P1A] = 1013.0;
   qtg_inputs.inps[U_A_ALT_P1B] = 1013.0;
   qtg_inputs.inps[U_A_ALT_P2] = 1013.0;
/*
         qtg_inputs.inps[U_A_PED]  = (float)( +(175.0 * qtg_outputs.psi_dot));
         if(qtg_inputs.inps[U_A_PED] > 1.0) qtg_inputs.inps[U_A_PED] = 1.0;
         if(qtg_inputs.inps[U_A_PED] < -1.0) qtg_inputs.inps[U_A_PED] = -1.0;

         qtg_inputs.inps[U_A_LAT]  = (float) (100.0 * qtg_outputs.phi);
         if(qtg_inputs.inps[U_A_LAT] > 1.0) qtg_inputs.inps[U_A_LAT] = 1.0;
         if(qtg_inputs.inps[U_A_LAT] < -1.0) qtg_inputs.inps[U_A_LAT] = -1.0;

         qtg_inputs.inps[U_A_PED]  = (float)(231.0 * qtg_outputs.psi_dot);  ///qtg_outputs.psi_dot));
         if(qtg_inputs.inps[U_A_PED] > 1.0) qtg_inputs.inps[U_A_PED] = 1.0;
         if(qtg_inputs.inps[U_A_PED] < -1.0) qtg_inputs.inps[U_A_PED] = -1.0;

         qtg_inputs.inps[U_A_LAT]  = (float) (60.0 * qtg_outputs.phi);
         if(qtg_inputs.inps[U_A_LAT] > 1.0) qtg_inputs.inps[U_A_LAT] = 1.0;
         if(qtg_inputs.inps[U_A_LAT] < -1.0) qtg_inputs.inps[U_A_LAT] = -1.0;
*//*
if(qtg.et.holdheading == TRUE)// && qtg.testid != 56)
{
//         qtg_inputs.inps[U_A_PED]  = (float)(231.0 * qtg_outputs.psi_dot);  ///qtg_outputs.psi_dot));
         qtg_inputs.inps[U_A_PED]  = (float)(50.0 * qtg_outputs.psi_dot);  ///qtg_outputs.psi_dot));
         if(qtg_inputs.inps[U_A_PED] > 1.0) qtg_inputs.inps[U_A_PED] = 1.0;
         if(qtg_inputs.inps[U_A_PED] < -1.0) qtg_inputs.inps[U_A_PED] = -1.0;
}
if((qtg.leftmap != qtg.rightmap))// || qtg.testid == 56)
{
         roll_int += (float)(qtg_outputs.phi_dot * dt);
         qtg_inputs.inps[U_A_LAT]  = (float) ((2.0 * roll_int) + (25.0 * (qtg_outputs.phi)) +  (20.0 * qtg_states.p));
         if(qtg_inputs.inps[U_A_LAT] > 1.0) qtg_inputs.inps[U_A_LAT] = 1.0;
         if(qtg_inputs.inps[U_A_LAT] < -1.0) qtg_inputs.inps[U_A_LAT] = -1.0;
}
*/
         if(qtg.leftmap != qtg.rightmap)
         {
            //set pedel to achieve zero rate of change of heading
            qtg_inputs.inps[U_A_PED]  = (float)(75.0 * qtg_outputs.user[Y_A_SLIP_ANGLE]);//qtg_outputs.psi_dot);
            if(qtg_inputs.inps[U_A_PED] > 1.0) qtg_inputs.inps[U_A_PED] = 1.0;
            if(qtg_inputs.inps[U_A_PED] < -1.0) qtg_inputs.inps[U_A_PED] = -1.0;
            //set ailerons to achieve zero rate of roll angle change
            //roll_int += (float)(qtg_outputs.psi_dot * dt);
            
            float craft_head = qtg_outputs.psi * R2D;
            if(craft_head < 0.0f)
               craft_head += 360.0f;
            psi_error = craft_head - qtg.heading;
            qtg_inputs.inps[U_A_LAT]  = (float) ((10.0 * qtg_outputs.psi_dot) + (col_err * psi_error));//roll_int) + (25.0 * (qtg_outputs.psi)) +  (20.0 * qtg_states.p));
            if(qtg_inputs.inps[U_A_LAT] > 1.0) qtg_inputs.inps[U_A_LAT] = 1.0;
            if(qtg_inputs.inps[U_A_LAT] < -1.0) qtg_inputs.inps[U_A_LAT] = -1.0;

         }







if(stepFlapDn == FALSE && stepFlapUp == FALSE)
{
   qtg_inputs.inps[U_A_FLAP] = qtg.flaps * 1.0f;
/*   
   if(qtg.flaps == 0)
   {
      qtg_inputs.dinps[U_D_FLAP] =  0;
   }
   else   
      if(qtg.flaps == 10)
      {
         qtg_inputs.dinps[U_D_FLAP] =  1;
      }
      else
         if(qtg.flaps == 25)
         {
            qtg_inputs.dinps[U_D_FLAP] =  2;
         }
         else
            if(qtg.flaps == 40)
            {
               qtg_inputs.dinps[U_D_FLAP] =  3;
            }
*/
}

   if(stepFlapUp == TRUE)
   {
      if(qtg_outputs.duser[Y_D_FLAPS] == FALSE)
      {
         flapDelayTimer += delta_time;
         
      }
      else
         flapDelayTimer = 0.0;


      if((flapDelayTimer > 5.5) && qtg_inputs.inps[U_A_FLAP] > 0.0f)
      {
         if(qtg_inputs.inps[U_A_FLAP] > 30)
            qtg_inputs.inps[U_A_FLAP] = 25.0f;
         else
         if(qtg_inputs.inps[U_A_FLAP] > 20.0)
            qtg_inputs.inps[U_A_FLAP] = 10.0f;
         else
            qtg_inputs.inps[U_A_FLAP] = 0.0f;
         //printf("\n MOVing Flaps to %d",qtg_inputs.dinps[U_D_FLAP]);
      }
   }

   if(stepFlapDn == TRUE)
   {
      if(qtg_outputs.duser[Y_D_FLAPS] == FALSE)
      {
         flapDelayTimer += delta_time;
         //printf("\n  Timer %f %d", flapDelayTimer,qtg_inputs.dinps[U_D_FLAP]);
      }
      else
         flapDelayTimer = 0.0;


      if((flapDelayTimer > 5.5) && qtg_inputs.inps[U_A_FLAP] < 40.0f)
      {
//         qtg_inputs.dinps[U_D_FLAP] ++;
//         printf("\n MOVing Flaps to %d",qtg_inputs.dinps[U_D_FLAP]);
         if(qtg_inputs.inps[U_A_FLAP] < 10.0f)
            qtg_inputs.inps[U_A_FLAP] = 10.0f;
         else
         if(qtg_inputs.inps[U_A_FLAP] < 20.0)
            qtg_inputs.inps[U_A_FLAP] = 25.0f;
         else
            qtg_inputs.inps[U_A_FLAP] = 40.0f;

      }
   }
/*
if(stepFlapDn == FALSE && stepFlapUp == FALSE)
{
   if(qtg.flaps == 0)
   {
      qtg_inputs.dinps[U_D_FLAP] =  0;
   }
   else   
      if(qtg.flaps == 10)
      {
         qtg_inputs.dinps[U_D_FLAP] =  1;
      }
      else
         if(qtg.flaps == 25)
         {
            qtg_inputs.dinps[U_D_FLAP] =  2;
         }
         else
            if(qtg.flaps == 40)
            {
               qtg_inputs.dinps[U_D_FLAP] =  3;
            }

}
   if(stepFlapUp == TRUE)
   {
      if(qtg_outputs.duser[Y_D_FLAPS] == FALSE)
      {
         flapDelayTimer += delta_time;
//         printf("\n  Timer %f %d", flapDelayTimer,qtg_inputs.dinps[U_D_FLAP]);
      }
      else
         flapDelayTimer = 0.0;


      if((flapDelayTimer > 5.5) && qtg_inputs.dinps[U_D_FLAP] > 0)
      {
         qtg_inputs.dinps[U_D_FLAP] --;
  //       printf("\n MOVing Flaps to %d",qtg_inputs.dinps[U_D_FLAP]);
      }
   }

   if(stepFlapDn == TRUE)
   {
      if(qtg_outputs.duser[Y_D_FLAPS] == FALSE)
      {
         flapDelayTimer += delta_time;
    //     printf("\n  Timer %f %d", flapDelayTimer,qtg_inputs.dinps[U_D_FLAP]);
      }
      else
         flapDelayTimer = 0.0;


      if((flapDelayTimer > 5.5) && qtg_inputs.dinps[U_D_FLAP] < 3)
      {
         qtg_inputs.dinps[U_D_FLAP] ++;
      //   printf("\n MOVing Flaps to %d",qtg_inputs.dinps[U_D_FLAP]);
      }
   }


*/
   qtg_inputs.inps[U_A_L_COWL] = (float)(float) (qtg.leftcowlflap/100.0);
   qtg_inputs.inps[U_A_R_COWL] = (float)(float) (qtg.rightcowlflap/100.0);
   qtg_inputs.dinps[U_D_PARK] = qtg.parkbrake;

/*
   if((IO_Sen_in->control_yoke_pitch_position > real_pitch_position + 1.0) || 
      (IO_Sen_in->control_yoke_pitch_position < (real_pitch_position - 1.0)))
   {
      qtg_inputs.inps[U_A_LONG] = (IO_Sen_in->control_yoke_pitch_position / 100);
   }
*/
//   if((IO_Sen_in->control_yoke_roll_position > real_roll_position + 1.0) || 
//      (IO_Sen_in->control_yoke_roll_position < real_roll_position - 1.0))
   if(free_roll == TRUE)
   {
      qtg_inputs.inps[U_A_LAT] = ((IO_Sen_in->control_yoke_roll_position/100) * -1);
   }

   if(temp_delay_holdspeed == TRUE && (qtg_outputs.user[Y_A_P1_ASI] < qtg.et.targetspeed))
   {
      temp_delay_holdspeed = FALSE;
      qtg_inputs.inps[U_A_LONG] = trimed_pitchpos;//trim_pitch_stick;
   }

  
   if(impulse == TRUE)
   {
      qtg_inputs.inps[U_A_LONG] = 0.50f;
   }

   if(impulse == TRUE && (qtg.airspeed > qtg_outputs.user[Y_A_P1_ASI]))
   {
      impulse = FALSE;
      qtg_inputs.inps[U_A_LONG] = trimed_pitchpos;
   }


   if(qtg.testid != 8 && qtg.testid != 9)
//      int x =1;
//   if(x)
   {
      qtg_inputs.inps[U_A_L_THROT] += (float) (0.4e-4 * (qtg.leftmap - qtg_outputs.user[Y_A_L_MAN_P]));
      if(qtg_inputs.inps[U_A_L_THROT] < 0.0) qtg_inputs.inps[U_A_L_THROT] = 0.0;
      if(qtg_inputs.inps[U_A_L_THROT] > 1.0) qtg_inputs.inps[U_A_L_THROT] = 1.0;
      // TRIM MAP to demanded
      qtg_inputs.inps[U_A_R_THROT] += (float) (0.4e-4 * (qtg.rightmap - qtg_outputs.user[Y_A_R_MAN_P]));
      if(qtg_inputs.inps[U_A_R_THROT] < 0.0) qtg_inputs.inps[U_A_R_THROT] = 0.0;
      if(qtg_inputs.inps[U_A_R_THROT] > 1.0) qtg_inputs.inps[U_A_R_THROT] = 1.0;
   }/*
   else // on accleration of throttle slow down rate of change
   {
      qtg_inputs.inps[U_A_L_THROT] += (float) (0.1e-4 * (qtg.leftmap - qtg_outputs.user[Y_A_L_MAN_P]));
      if(qtg_inputs.inps[U_A_L_THROT] < 0.0) qtg_inputs.inps[U_A_L_THROT] = 0.0;
      if(qtg_inputs.inps[U_A_L_THROT] > 1.0) qtg_inputs.inps[U_A_L_THROT] = 1.0;
      // TRIM MAP to demanded
      qtg_inputs.inps[U_A_R_THROT] += (float) (0.1e-4 * (qtg.rightmap - qtg_outputs.user[Y_A_R_MAN_P]));
      if(qtg_inputs.inps[U_A_R_THROT] < 0.0) qtg_inputs.inps[U_A_R_THROT] = 0.0;
      if(qtg_inputs.inps[U_A_R_THROT] > 1.0) qtg_inputs.inps[U_A_R_THROT] = 1.0;
   }*/
/*
   if(stall_flag == TRUE)
   {
      qtg.airspeed -= delta_time;
   }
*/

   if(slam_closed == TRUE)
   {
      qtg_inputs.inps[U_A_L_THROT] = 0.0f;
      qtg_inputs.inps[U_A_R_THROT] = 0.0f;
      if(qtg_outputs.user[Y_A_L_MAN_P] > 16.0f)
         throt_timer += delta_time;
      else
         power_time = throt_timer;


      
   }

   if(slam_open == TRUE)
   {
      qtg_inputs.inps[U_A_L_THROT] = 1.0f;
      qtg_inputs.inps[U_A_R_THROT] = 1.0f;
      if(qtg_outputs.user[Y_A_L_MAN_P] < 37.5f)
         throt_timer += delta_time;
      else
         power_time = throt_timer;

   }

   if(qtg.et.holdspeed == TRUE && temp_delay_holdspeed == FALSE && free_pitch == FALSE)
   {
      if(qtg.airspeed != 0)
      {
         theta_dem +=   (float) (-1.0e-4 * (qtg.airspeed - qtg_outputs.user[Y_A_P1_ASI]));
         qtg_inputs.inps[U_A_LONG]  = (float) ( (-10.0 * (theta_dem * D2R - qtg_outputs.theta)) + (1.0 * qtg_states.q));
         if(qtg_inputs.inps[U_A_LONG] > 1.0) qtg_inputs.inps[U_A_LONG] = 1.0;
         if(qtg_inputs.inps[U_A_LONG] < -1.0) qtg_inputs.inps[U_A_LONG] = -1.0;
      }
   }

   if(temp_bank_hold == TRUE && (qtg_outputs.phi*R2D >= qtg.roll) && (qtg.roll > 1))
   {
      spiral_timer += delta_time;
      if(spiral_timer > 2.0f)
      {
         temp_bank_hold = FALSE;
         qtg_inputs.inps[U_A_LAT] = 0.0f;
         spiral_timer = 0.0f;
      }
   }

   if(temp_bank_hold == TRUE && (qtg_outputs.phi*R2D <= qtg.roll) && (qtg.roll < -1))
   {
      spiral_timer += delta_time;
      if(spiral_timer > 2.0f)
      {
         temp_bank_hold = FALSE;
         qtg_inputs.inps[U_A_LAT] = 0.0f;
         spiral_timer = 0.0f;
      }
   }

if(roll_rate == FALSE && qtg.et.holdheading == FALSE &&(qtg.et.holdaob == TRUE || temp_bank_hold == TRUE ))
{
   
/*   
   roll_pos = (float) (qtg.roll * 1.0);
   if(roll_pos > (outputs.phi*RAD_TO_DEG))
   {
   
      //qtg_inputs.inps[U_A_LAT] = (float)  -(((qtg.roll+5) - (outputs.phi*RAD_TO_DEG))/100);
      qtg_inputs.inps[U_A_LAT] -= delta_time / 4.0f;
      //qtg_inputs.inps[U_A_LAT] = (float)  -((qtg.roll - (outputs.phi*RAD_TO_DEG))/100);
      if(qtg_inputs.inps[U_A_LAT] < -1.0)
         qtg_inputs.inps[U_A_LAT] = -1.0;
   }
   else
   if(roll_pos < (outputs.phi*RAD_TO_DEG))
   {
     
      //qtg_inputs.inps[U_A_LAT] =(float)  +(((outputs.phi*RAD_TO_DEG ) - (qtg.roll-5))/100);
      qtg_inputs.inps[U_A_LAT] += delta_time / 4.0f;
      //qtg_inputs.inps[U_A_LAT] =(float)  +(((outputs.phi*RAD_TO_DEG ) - qtg.roll)/100);
      if(qtg_inputs.inps[U_A_LAT] > 1.0)
         qtg_inputs.inps[U_A_LAT] = 1.0;
   }
*/

   
   roll_dem = (qtg.roll * 1.0f);
   if(roll_dem < 0.0f)
      roll_dem -= 2.0f;
   else
      roll_dem += 2.0f;
   roll_dem = (roll_dem * DEG_TO_RAD);// * DEG_TO_RAD);
   roll_error = ((outputs.phi) - roll_dem);
   i_roll_error = roll_error * delta_time;
   roll_servo_dem  = (ap_Kphi_p   *   roll_error) + (ap_Kphi_i   * i_roll_error);
   if (roll_servo_dem  > +1.0) roll_servo_dem  = +1.0;
   if (roll_servo_dem  < -1.0) roll_servo_dem  = -1.0;

   if(roll_servo_dem > 0.1f)
   {
      qtg_inputs.inps[U_A_LAT] += (delta_time * MAX_ROLL_DELTA/10.0f );
      //roll_null -= delta_time * MAX_ROLL_DELTA;
   }
   else
   if(roll_servo_dem < -0.1f)
   {
      qtg_inputs.inps[U_A_LAT] -= (delta_time * MAX_ROLL_DELTA /10.0f);
      //roll_null += delta_time * MAX_ROLL_DELTA;
   }

   qtg_inputs.inps[U_A_LAT] = roll_servo_dem;
   if(qtg_inputs.inps[U_A_LAT] > 0.3f)
      qtg_inputs.inps[U_A_LAT] = 0.3f;
   if(qtg_inputs.inps[U_A_LAT] < -0.3f)
      qtg_inputs.inps[U_A_LAT] = -0.3f;
   
   //qtg_inputs.inps[U_A_LAT] = 0.5f;


} 

   if(roll_rate == TRUE)
   {
      if((qtg.roll < (outputs.phi*RAD_TO_DEG)) && done_ang == FALSE)
      {
         if(qtg_inputs.inps[U_A_LAT] < 0.35 )
         {
            qtg_inputs.inps[U_A_LAT] += 2.0f * delta_time;
         }
      }
      else
      {
         done_ang = TRUE;
         if(qtg_inputs.inps[U_A_LAT] > 0.0f)
            qtg_inputs.inps[U_A_LAT] -= delta_time;
      }

   }

   if(roll_rate == TRUE)
   {
      if((qtg_outputs.phi * RAD_TO_DEG) > 31.0f)
      {
         roll_rate_arm = TRUE;
      }

      if(roll_rate_arm == TRUE)
      {
         if( ((qtg_outputs.phi * RAD_TO_DEG) < 30.0f) && ((qtg_outputs.phi * RAD_TO_DEG) > -30.0f))
         {
            if((qtg_outputs.phi_dot * R2D) < 0.0f)
            {
               total_roll_rate -= (qtg_outputs.phi_dot * R2D) * delta_time;
            }
            else
               total_roll_rate += (qtg_outputs.phi_dot * R2D) * delta_time;

            roll_rate_timer += delta_time;
         }
         else
         {
            if(roll_rate_timer > 0.0f)
               ave_roll_rate = total_roll_rate/roll_rate_timer;
         }
      }
   }

if (dutch_roll_flag == TRUE)
{
   if(drf_timer < 1.0f)
   {
      if(qtg_inputs.inps[U_A_PED] < (qtg.et.pedalpos/100.0f))
      {
         qtg_inputs.inps[U_A_PED] += delta_time;
      }
      else
         drf_timer += delta_time;
   }

      if(drf_timer >= 1.0f && drf1_timer < 1.0f)
      {
         if(qtg_inputs.inps[U_A_PED] > -(qtg.et.pedalpos/100.0f))
         {
             qtg_inputs.inps[U_A_PED] -= delta_time;
         }
         else
            drf1_timer += delta_time;
      }
      else
      if(drf_timer >= 1.0f && drf1_timer >= 1.0f)
      {
         if(qtg_inputs.inps[U_A_PED] < 0.0f)
            qtg_inputs.inps[U_A_PED] += delta_time;
         else
         {
            
            dutch_roll_flag = FALSE;
            qtg_inputs.inps[U_A_PED] = 0.0f;
            drf_timer = 0.0f;
            drf1_timer = 0.f;
         }
      }
/*
  
   if(drf_timer < 1.0)
   {
      qtg_inputs.inps[U_A_PED] = (float) ((qtg.et.pedalpos/100.0));
   }


   if(drf_timer > 1.0 && drf_timer < 2.0)
   {
      
      qtg_inputs.inps[U_A_PED] = (float) ((qtg.et.pedalpos/100.0)*-1);
   }
   else
      if(drf_timer > 2.0)
      {
         qtg_inputs.inps[U_A_PED] = 0.0;
         drf_timer = 0.0;
         dutch_roll_flag = FALSE;
      }
   drf_timer += delta_time;
   */
}

   if(!roll_impulse )
   {
      save_roll = qtg_inputs.inps[U_A_LAT];
   }
   if(roll_impulse)
   {
      if(roll_im_timer < 1.5f)
      {
         qtg_inputs.inps[U_A_LAT] = 0.5f;
         roll_im_timer += delta_time;
      }
      else
         qtg_inputs.inps[U_A_LAT] = 0.0f;
         //roll_impulse = FALSE;
   }

   if(qtg.testid >59 && qtg.testid < 63)
   {
      //qtg_inputs.inps[U_A_LONG] = 0.0f;//IO_Sen_in->control_yoke_pitch_position/100.0f;
      qtg_inputs.inps[U_A_PED] = 0.0f;
      qtg_inputs.inps[U_A_LAT] = 0.0f;
   }
   else
   if(qtg.testid > 62 && qtg.testid < 66)
   {
      //qtg_inputs.inps[U_A_LONG] = 0.0f;//IO_Sen_in->control_yoke_pitch_position/100.0f;
      qtg_inputs.inps[U_A_PED] = 0.0f;
      qtg_inputs.inps[U_A_LAT] = 0.0f;

   }
   else
   if(qtg.testid > 65 && qtg.testid < 69)
   {
      //qtg_inputs.inps[U_A_LONG] = 0.0f;//IO_Sen_in->control_yoke_pitch_position/100.0f;
      qtg_inputs.inps[U_A_PED] = 0.0f;
      qtg_inputs.inps[U_A_LAT] = 0.0f;

   }
                    

   if(hold_head && !free_roll)
   {
         
            float craft_head = qtg_outputs.psi * R2D;
            if(craft_head < 0.0f)
               craft_head += 360.0f;
            psi_error = craft_head - qtg.heading;
            qtg_inputs.inps[U_A_LAT]  = (float) ((10.0 * qtg_outputs.psi_dot) + (col_err * psi_error));
			//roll_int) + (25.0 * (qtg_outputs.psi)) +  (20.0 * qtg_states.p));
            if(qtg_inputs.inps[U_A_LAT] > 1.0) qtg_inputs.inps[U_A_LAT] = 1.0;
            if(qtg_inputs.inps[U_A_LAT] < -1.0) qtg_inputs.inps[U_A_LAT] = -1.0;
   }


}

   /*
      qtg_states.user[7]         =(float) (P0 * pow((T0 - L * height)/T0,5.2558797));
      qtg_states.user[8]         =(float) (P0 * pow((T0 - L * height)/T0,5.2558797));
      qtg_states.user[X_L_RPM]   =(float) 293.21;
      qtg_states.user[X_R_RPM]   =(float) 293.21;
      qtg_states.user[X_L_BLADE] =(float) 0.357;
      qtg_states.user[X_R_BLADE] =(float) 0.357;
      qtg_states.user[X_L_MAN_P] =(float) 25.7;
      qtg_states.user[X_R_MAN_P] =(float) 25.7;
      //qtg_states.user[X_L_FUEL]  =(float) -0.97;
      qtg_states.user[X_L_CHT]   =(float) 286.0;
      qtg_states.user[X_L_EGT]   =(float) 1201.0;
      //qtg_states.user[X_R_FUEL]  =(float) -0.97;
      qtg_states.user[X_R_CHT]   =(float) 286.0;
      qtg_states.user[X_R_EGT]   =(float) 1201.0;
      qtg_states.e0              =(float) -0.243;
      qtg_states.e1              =(float) -0.04;
      qtg_states.e2              =(float) -0.007;
      qtg_states.e3              =(float) -0.969;
*/
