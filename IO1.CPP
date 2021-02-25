 /*
IO: IO1.cpp
*/

/*--------------------- 
This file contains seneca input and output functions
---------------------*/

/***************************/
/*   RCS Markers           */
/***************************/

/*
$Header: C:/Master/seneca/RCS/io1.cpp 1.11 2000/06/16 08:52:31 ColinJ Exp $
$Log: io1.cpp $
Revision 1.11  2000/06/16 08:52:31  ColinJ
Changes to move control function and tidy up.

Revision 1.10  2000/05/31 14:50:03  colinj
Support for IO_engine_shhh. Check if ios_secs == avail_time
in play back mode. Combined alt is FALSE and control_ok is FALSE
cases. Primer sounds now depend on electric power.
Correction: initialize restart_left/right sound flags.
Set_controls changed to support delay in doing dcl. Support
for new hardware dcl modes.

Revision 1.9  2000/05/30 12:04:50  colinj
Support for screen saver.
Reset QC_goflag in DYN_HALT. Use DY_trim_flight.
Initialize altimeters to demand height.
Changed elec loads for gear pump and landing lights.
Special_move: support for 3 speeds. ALT only allowed if
airspeed > 80. Use true heading.

Revision 1.8  2000/05/25 09:42:51  colinj
Changes to IO_set_sound.

Revision 1.7  2000/05/17 17:30:48  colinj
Moved DYN_QTG code to new file.

Revision 1.6  2000/05/14 10:22:05  colinj
Change to demand_pos/force in DYN_HALT.
Ambient elec load now depends on avionic power.
Landing lights handled separately
Clear gear/stall horn lights when sound is off
Clear Y_D_CRASH after setting DYN_HALT.

Revision 1.5  2000/05/11 13:26:55  colinj
Add support for special move mode (SE_freeze) and ios freeze.

Revision 1.4  2000/05/11 09:06:32  colinj
Sound changes in set_sound.

Revision 1.3  2000/05/10 16:28:14  colinj
Qtg changes.

Revision 1.2  2000/05/10 09:07:51  colinj
Call BDY_init_pres when you go in to DYN_RUN

Revision 1.1  2000/04/14 17:23:59  colinj
Initial revision

*/

//----------------------------------------------------------------------

//----------------
// Include Files
//----------------
 
#include <stdlib.h>
#include <fstream.h>
#include <stdio.h>
#include <math.h>
#include <windows.h>
#include <string.h>
#include <conio.h>


#include "dat_stru\in_head.h"
#include "dat_stru\out_head.h"
#include "define.h"
#include "const.h"
#include "fastlogger.h"
#include "fltdy\fmgdefs.h"
#include "fltdy\fms_defs.h"
#include "fltdy\fms_s3fm.h"
#include "faults.h"
#include "elec\electric.h"
#include "nav\beacon.h"
#include "nav\dyn_ios.h"
#include "config.h"
#include "qtgdefs.h"
#include "sound\dynsnd.h"
#include "dyn_snd.h"
#include "record.h"
//----------------
// Defines
//----------------
#define MAX_REC_TIME   600  // max amount of time we can record for in secs
#define DEBUG_IO

#define SCREEN_SAVER_ON_TIME 12.0f //delay before screen saver comes on

#define HPA2INHG (29.92/101325.0)  // converts hecto pascal to inches Hg

#define UP_DOWN_SCALE    1.0f   //Meters/sec
#define FORE_AFT_RATE    30.0f  //Meters/sec
#define SLIP_RATE        30.0f  //Meters/sec
#define NORM_SPEED        1.0f  //MULTIPLER
#define SUPER_SPEED      10.0f  //MULTIPLER
#define SUPERSONIC_SPEED 20.0f  //MULTIPLER  //Julia supersonic!
#define SLOW_SPEED        0.5f

#define PITCH_RATE        2.0f  //DEGRES/sec
#define HEAD_RATE         3.0f  //DEGRES/sec
#define FREEZE_FLASH      0.3f

#define TRANS_FORE_AFT  3
#define TRANS_SLIP      2
#define TRANS_ALT       1

#define SET_CON_DCL     1
#define SET_CON_FRAME   2

//----------------
// Typedefs
//----------------
/*
typedef struct
{
   sen_out_struct IO_record;     //hardware outputs
   dyn_to_snd     snd_rec;       //sound computer outputs
   dynsound       dynSndRec;     //"internal" sounds
   STATES         rec_states;
   float          sim_time;      //time sim has been running 
   unsigned int   record_index;  //index number of record
   float          delta_time;    //delta time for record
   float          pitch ,roll,pitch_trim,rudder; //hardware inputs stored to enable controls to be driven
   float          z,h,p,r;            //craft height,heading,pitch,roll for visual
   double         lat,lon;            //lat lon for visuals

}record_struct;
*/

//--------------------
// External Routines
//--------------------

extern void reset_dynamics();
extern void simulate();    //flight model run 
extern void DY_trim_flight(void);
extern void qtg_trim_flight(void);

//extern void war_simulate();

extern void cwp_panel(void);
extern void HD_exchange_data(void);
extern void DYS_dynsnd(int);
extern void IS_deice(void);
extern void DC_dcl_control(void);
extern void UT_local_xy_2_ll(float delta_x, float delta_y, double ref_lat, double ref_lon,
                      double *lat1, double *lon1);
extern void UT_ll_2_local_xy(double lat, double lon, double ref_lat, double ref_lon, float *x, float *y);
extern void UT_offset_along_dir (float x1, float y1, float z1, float h,
                                 float p, float dist, float *x, float *y, float *z);

extern void BC_update_beacon_dials();
extern void ILS_init(void);
extern void VOR_init(void);
extern void RMI_init(void);
extern void CS_gyro_init();
extern void CS_update();
extern void CS_gyro_slave_button_changes();
extern void CS_calc_mag_head(float true_head, double host_lat, double host_lon);
extern void CS_gyro_ok();
extern void N1A_nav1_init();
extern void N1A_nav1_update();
extern void N2A_nav2_init();
extern void N2A_nav2_update();
extern void ADF_adf_init();
extern void ADF_adf_update();
extern void CA_com_init();
extern void CA_com_update();
extern void RA_rad_init();
extern void RA_rad_update();
extern void BC_convert_host_pos(ll_pos *host_pos);
extern void SC_init_snd_coms();
extern void WD_init_turb(void);
extern double SPH_get_sph_flat_dist_sq (double lat1, double lon1, float ht1,
                                        double lat2, double lon2, float ht2);
extern void FC_fog_ctrl (int valid_bdy, float hdist, float delta_ht);
extern void FC_cloud_ctrl (int valid_bdy, float hdist, float delta_ht);
extern void WD_wind_ctrl (int valid_bdy, float hdist, float delta_ht, float wind[3]);
extern void BDY_pres_ctrl (int valid_bdy, float hdist, float delta_ht, float pres_in, float *pres_out);
extern void BDY_temp_ctrl (int valid_bdy, float hdist, float delta_ht, float *temp_out);
extern void BDY_init_pres();
extern void WD_update_turb(int turb_percent);
extern void RR_refresh_vis();
extern void QC_control(void);

//----------------
// Externals
//----------------
//structures
extern OUTPUTS    outputs;
extern STATES     states;
extern STATES     qtg_states;
extern INPUTS     inputs;
extern ATMOS       atmosphere;
extern ll_pos     BC_host_pos;
extern ios_to_dyn *ET_ios_data_p;
extern StartData  DY_start;
extern dynsound   DYS_in;
extern dyn_to_snd *ET_tx_snd;
extern AtisData   IOS_atis;
extern AtmosphericsData DY_atmos;
extern inital_cond      init_vals;

//variables
extern float delta_time;
extern float DY_last_pressure;
extern float DY_new_pressure;
extern float dip_vac;
extern float last_vis_heading;
extern float terrain_height;
extern float DY_temp;
extern float ah_air_press;
extern float set_stick_pos;
extern float RC_rudder_scale;

extern int dyn_state;
extern int low_bus;
extern int qtg_testNum;
extern int IOS_marker_type;
extern int IOS_simtype;
extern int IOS_skybox;
extern int RR_vis_db_loaded;
extern int RR_tfb_ok;
extern int DY_need_db;
extern int DY_freeze_status;
extern int SE_freeze_on;
extern int AP_beep;
extern int QC_goflag;
extern int QC_event_flag;
extern int set_screen; // set true to turn on visual screen saver
extern int test_screen;
extern int qtg_special;
extern int mode;

extern int QC_manualmode;
extern int qtg_ManualMode;		//Woody Mod

extern unsigned int ios_secs;

extern char trim_caution;
extern char DY_trimed;
extern char DY_l_start,DY_r_start;
extern char ET_ios_data_in[];
extern char DY_startInFreeze;
extern char IC_ice_load;
extern char wing_deice_flag;

//----------------
// Globals
//----------------

ndb_beac_attr    IO_ndb_battr;
vhf_beac_attr    IO_nav2_battr;
vhf_beac_attr    IO_nav1_battr;
vhf_beac_attr    IO_dme_battr;

QTGfastlog       IO_fastLog;
record_struct    IO_rec_sen_out[MAX_REC];
sen_in_struct    *IO_Sen_in;
OUTPUTin		 *OUTPUTinbox;
sen_qtg_struct   *IO_Sen_qtg;
sen_out_struct   *IO_Sen_out;
sen_screens_struct   *IO_Sen_screens;
sen_fault_struct IO_Sen_faults;

EL_in            EL_input;
EL_out           EL_output;
dyn_to_ios       IO_dyn_ios;
switch_struct    switch_in;
load_struct      load_in;

double sim_run_time = 0.0;

float pitch_force_scale  = 0.0;
float roll_force_scale   = 0.0;
float rudder_force_scale = 0.0;

long int frame_ct   = 0;
long int last_frame = 0;

int IO_elec_pwr    = 0;     // Elec_power true/false flag 
int type           = 0;
int IO_engine_shhh     = FALSE;
int avail_time     = 0;
int IO_avionic_pwr = FALSE;
int PETE_SHHHH     = FALSE;
int ret_search = 0;
int IO_iosfreeze = FALSE;
int IO_last_state         = 0;
int IO_head_freeze = FALSE;
int IO_alt_freeze  = FALSE;
int IO_pos_freeze  = FALSE;
int trans_mode     = FALSE;
int set_cntrl      = FALSE;
//-----------------
// Local Variables
//-----------------

static snd_in dyn_snd;

static float r_wheelspd   = 0.0, n_wheelspd   = 0.0, l_wheelspd   = 0.0;
static float last_r_speed = 0.0, last_n_speed = 0.0, last_l_speed = 0.0;
static float last_flapPos    = 0.0;
static float last_ngear_state = 0.0;
static float delay_dcl_timer = 0.0;
static float halt_mode_timer = 0.0;
//For special move
static float freeze_timer    = 0.0;
static float super_mode      = 1.0; // can be 1 or ten
static float super_sonic     = 1.0;
static float min_sim_time = 0.0f;

static int last_nose  = FALSE, last_right = FALSE, last_left  = FALSE;
static int last_ngear = FALSE, last_lgear = FALSE, last_rgear = FALSE , last_crash = FALSE;
static int gear_retract = FALSE;
static int last_deice = UNLIT;
static int control_ok = FALSE;
static int last_Rprim  = SW_OFF;
static int last_Lprim  = SW_OFF;
static int last_Rdrain = SW_OFF;
static int last_Ldrain = SW_OFF;
static int co_alt      = FALSE;
static int p1_alt      = FALSE;
static int p2_alt      = FALSE;
static int alt         = FALSE;
static int last_main_pwr = FALSE;
static int last_g_transit = FALSE;
static int gear_in_transit = FALSE;
static int con_type = 2;
static int last_bc_butt   = SW_OFF;
static int last_ap_butt   = SW_OFF;
static int last_test_butt = SW_OFF;
static int last_freeze    = FALSE;
static int clear_crash    = 0;
static int rec_delay_counter  = 0;
static int play_delay_counter = 0;
static int find_fail          = 0;
static int flap_move = FALSE;
static int last_gear = 0,last_flap = 0,last_rstart = 0,last_lstart = 0;
static unsigned int s_time = 0;

//--------------------
// Forward References
//--------------------
//set elec module inputs and outputs
void IO_elec_in(void); 
void IO_elec_out(void);
void add_fault(int f);
void rem_fault(int f);
void Special_move();
//navigation instruments module
void IO_exchange_nav();
//dynamic sound data exchange
void IO_set_sound(int state);

//for playback and mid flight pauses to find correct record
static int search_frame(unsigned int secs);
//set up for flight model
static void reset_dyn(void);
static void init_dynamics(void);
// record / retrive data to/from record struct
static void rec_data(void);
static void ret_data(void);
int Set_controls(int frame, int mode);
static void beacon_flash(int type);
static void Boundary_ctrl(void);
static float clear_record(void);
//----------------------------------------------------------------------

void IO_handler(int state)
{ 

   

   if(state != INIT)
      //IO_Sen_in->test_control_column_force = IO_Sen_in->test_control_column_force - 24.4f;
   {
   }


   IO_fastLog.id = 0;
   set_cntrl = FALSE;
   test_screen = FALSE;
   qtg_special = FALSE;

   if(state != DYN_HALT)
   {
      set_screen = FALSE; //make sure screen saver is off
      halt_mode_timer = 0.0;
   }

   //called during program initialization to set up structs, ethernet links etc...
   if(state == INIT)
   {
      init_dynamics();
      RR_refresh_vis();
      
   }
   else //PLAY BACK MODE
   if(state == DYN_PLAY)
   {
      if(state != IO_last_state) //IF first time find record desired
      {
			control_ok = FALSE;
         con_type = SET_CON_FRAME;
         if(ios_secs == 0)
         {
            ios_secs = 1;
         }
         if(ios_secs == (unsigned int )avail_time)
         {
            ios_secs -= 1;
         }
         find_fail = search_frame(ios_secs);


         if(find_fail == 1)
         {
            dyn_state = DYN_HALT;
            printf("\n record not found");
         }
         else
         {
            control_ok = Set_controls(frame_ct, con_type);
         }

      }
      else   
      if(control_ok == FALSE)
      {
         control_ok = Set_controls(frame_ct,con_type);
         IO_Sen_out->panel_lo_bus_wing_tail_transit_lamp_PTT_enable = 0;
         IO_Sen_out->system_lighting_power_control                  = 0;
         IO_Sen_out->system_KMA24H_power_control                    = 0;

      }
      else
      if(frame_ct != (last_frame)) //as long as we have not got to the last frame keep going
      {
         if(SE_freeze_on == FALSE)
            play_delay_counter++;

         if(play_delay_counter == 4)
         {
         
            frame_ct++;
            if(frame_ct == MAX_REC)
            {
               frame_ct = 0;
            }
            ret_data(); //retrive data from current record slot
            play_delay_counter = 0;

         }            
            DYS_dynsnd(0);//no need to set inputs done in ret data
      }
      else// we have found last record so go into halt
      {
         dyn_state = DYN_HALT;
      }
   }
   else
   if(state == DYN_HALT)
   {
      //printf("\n Rudder_scale %f",RC_rudder_scale);
      if (state != IO_last_state)
      {
         //reset halt timer;
         halt_mode_timer = 0.0;
         clear_crash = 0;
      }
      halt_mode_timer += delta_time;
      if (halt_mode_timer > SCREEN_SAVER_ON_TIME)
      {
         set_screen = TRUE;
      }

      if (clear_crash > 10)
      {
         outputs.duser[Y_D_CRASH] = FALSE; //set to false to avoid crash repeating
      }
      else
         clear_crash ++;

      QC_goflag = FALSE; // make sure this is set to false;
      QC_event_flag = FALSE;

      //need to turn off all sound
      IO_set_sound(0);
      DYS_dynsnd(0);
      SC_init_snd_coms();
      IO_Sen_out->panel_lo_bus_wing_tail_transit_lamp_PTT_enable = UNLIT;
      IO_Sen_out->system_lighting_power_control                  = UNLIT;
      IO_Sen_out->system_KMA24H_power_control                    = UNLIT;
      IO_Sen_out->panel_gear_horn  = UNLIT;
      IO_Sen_out->panel_stall_horn = UNLIT;

   }
   else// qtg mode use qtg dynamics
   if(state == DYN_QTG)
   {
      printf("\n GOING TO QTG %d",qtg_testNum); 
	  //printf("\n QTG Test ID %d",qtg_testNum);		//Woody Mod
	  //printf("\n========== QTG Test Mode Inter 0 = Auto, 1 = Manual: %d ==========\n",QC_manualmode);	//Woody Mod
	  printf("\n########## QTG Test Mode 0 = Auto, 1 = Manual: %d ##########",qtg_ManualMode);
      //fflush(stdout);
      CS_gyro_ok();
      QC_control();
   }
   else
   if (state == DYN_TRIM)
   {
      if (IO_last_state != DYN_TRIM)
      {
         reset_dynamics();
         BC_convert_host_pos(&BC_host_pos);
         //DY_trim_flight();
      }
      printf("\n RUNWAY OK = %d",RR_vis_db_loaded);
      if (DY_need_db)
      {
         if(RR_vis_db_loaded && RR_tfb_ok)
         {
            dyn_state = DYN_RUN;
         }
      }
      else
         dyn_state = DYN_RUN;
   }
   else
   if(state == DYN_RUN)
   {
      if(state != IO_last_state)
      {
         BDY_init_pres();
         IO_Sen_out->dcl_mode = DCL_INIT;
         delay_dcl_timer = 0.0;
         RR_refresh_vis();

         if(IOS_simtype == WARRIOR)
            printf("\n ---------------->WARRIOR");
         else
            printf("\n ---------------->SENECA");
         
         if(IOS_skybox == TRUE)
         {
            printf("\n ---------------->CLOUD ON");
         }
         else
            printf("\n ---------------->CLOUD OFF");
            

         if(ios_secs == -1)   //Instruction to do a reset
         {
            reset_dyn();//should be a trim function but not yet implemented by MK
            min_sim_time = 0.0f;
            //
            //DY_trim_flight();
            //qtg_trim_flight();
            if(DY_startInFreeze == TRUE)
            {
               SE_freeze_on = TRUE;
            }
            else
               SE_freeze_on = FALSE;

            //HD_exchange_data(); //set outputs for launch control
            IO_Sen_out->pilot_altimeter_1_demand_height = (float)((-states.D - terrain_height) * FT_IN_METRE);//outputs.user[Y_A_P1A_ALT];
            IO_Sen_out->pilot_altimeter_2_demand_height = (float)((-states.D - terrain_height) * FT_IN_METRE);//outputs.user[Y_A_P1B_ALT];
            IO_Sen_out->copilot_altimeter_demand_height = (float)((-states.D - terrain_height) * FT_IN_METRE);//outputs.user[Y_A_P2_ALT];
            con_type   = SET_CON_DCL;

            control_ok = FALSE;
            alt = true; //MODIFICADO POR MI ANTES A FALSE       //set altimeters to demand position  
         }
         else
         {
            if (ios_secs == 0)
               ios_secs = 1;
            if (ios_secs == (unsigned int )avail_time)
            {
               ret_search = 0;
            }
            else
               ret_search = search_frame(ios_secs);
            if(ret_search == 1)
            {
               printf("\n ====================SEARCH FAILED ====================>");
               dyn_state = DYN_HALT;
            }
            else
            {
               //reset amount of data in storage to how much is left after rewind and reset
					//printf("\n ====================SEARCH OK ====================>");
               //sim_run_time = (sim_run_time - IO_rec_sen_out[frame_ct].sim_time);
               //avail_time = 
               //clear any records with a higher sim_time
               min_sim_time = clear_record();
               con_type = SET_CON_FRAME;
               ret_data();
               CS_gyro_ok();
               control_ok = Set_controls(frame_ct, con_type);
               alt = true;  //MODIFICADO POR MI ANTES A FALSE
               SE_freeze_on = TRUE;
               DY_trimed = TRUE;
               float a,b,c,d,e;
               int fake[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
               int f,g;
               KFC150_exec(0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,&a,&b,&c,&d,&e,fake,fake,&f,&g,0);
               mode = KFC150_M_IDLE;
   
   

            }
         }
      }//end of first time stuff
      else
      if(DY_trimed == FALSE)
      {
         //DY_trimed = TRUE;
         DY_trim_flight();

         
/*
         PETE_SHHHH = TRUE;
         
         IO_Sen_out->pilot_altimeter_1_demand_height = outputs.user[Y_A_P1A_ALT];
         IO_Sen_out->pilot_altimeter_2_demand_height = outputs.user[Y_A_P1B_ALT];
         IO_Sen_out->copilot_altimeter_demand_height = outputs.user[Y_A_P2_ALT];
*/
      }
      else
      if ((control_ok == FALSE) || (alt == FALSE))
      {
         PETE_SHHHH = TRUE;
         IO_Sen_out->panel_lo_bus_wing_tail_transit_lamp_PTT_enable = 0;
         IO_Sen_out->system_lighting_power_control                  = 0;
         IO_Sen_out->system_KMA24H_power_control                    = 0;

			if(con_type != SET_CON_FRAME)
			{
            DY_freeze_status = FREEZE_RB_ONLY;
            //simulate(); 
			}
         control_ok = Set_controls(frame_ct,con_type);
         control_ok = TRUE; //MODIFICADO POR MI ANTES COMENTADO

         PETE_SHHHH = TRUE;
         IO_Sen_out->pilot_altimeter_1_demand_height = (float)((-states.D - terrain_height) * FT_IN_METRE);//outputs.user[Y_A_P1A_ALT];
         IO_Sen_out->pilot_altimeter_2_demand_height = (float)((-states.D - terrain_height) * FT_IN_METRE);//outputs.user[Y_A_P1B_ALT];
         IO_Sen_out->copilot_altimeter_demand_height = (float)((-states.D - terrain_height) * FT_IN_METRE);//outputs.user[Y_A_P2_ALT];

         if((IO_Sen_in->copilot_altimeter_current_height < IO_Sen_out->copilot_altimeter_demand_height + 2.0)
               && (IO_Sen_in->copilot_altimeter_current_height > IO_Sen_out->copilot_altimeter_demand_height - 2.0))
         {
            co_alt = TRUE;
         }
         else
            co_alt = FALSE;

         if((IO_Sen_in->pilot_altimeter_1_current_height < IO_Sen_out->pilot_altimeter_1_demand_height + 2.0)
               && (IO_Sen_in->pilot_altimeter_1_current_height > IO_Sen_out->pilot_altimeter_1_demand_height - 2.0))
         {
            p1_alt = TRUE;
         }
         else
            p1_alt = FALSE;

         if((IO_Sen_in->pilot_altimeter_2_current_height < IO_Sen_out->pilot_altimeter_2_demand_height + 2.0)
               && (IO_Sen_in->pilot_altimeter_2_current_height > IO_Sen_out->pilot_altimeter_2_demand_height - 2.0))
         {
            p2_alt = TRUE;
         }
         else
            p2_alt = FALSE;

         alt = (p2_alt && p1_alt && co_alt);
		   
		 
		 alt = TRUE;//MODIFICADO POR MI ANTES COMENTADO

      }
      else
      {
         IO_Sen_out->dcl_mode = DCL_NORM;
         if( IO_iosfreeze)
         {
            if(DY_freeze_status == 0)
            {
               DY_freeze_status = FREEZE_RB_ONLY;
            }

            Special_move();
            PETE_SHHHH = TRUE;
         }
         else
         if(SE_freeze_on == TRUE)
         {
            if (DY_freeze_status == 0)
            {
               DY_freeze_status = FREEZE_RB_ONLY;
            }

            Special_move();
         }


         if(SE_freeze_on == FALSE && IO_iosfreeze == FALSE)
         {
            DY_freeze_status = 0;
            trans_mode = FALSE;
         }

// NEED WARRIOR TEST AND WARRIOR ELEC MODEL
         IO_elec_in();
         EL_output = EL_frame(& EL_input);       // Electrics module
         IO_elec_out();

         //last_freeze = DY_freeze_status;
/*
         if(DY_freeze_status != FREEZE_RB_ONLY)
         {
            DY_freeze_status = 0;
         }
*/
         if(IO_alt_freeze == TRUE)
         {
            DY_freeze_status += FREEZE_NED;
         }

         if(IO_pos_freeze == TRUE)
         {
            DY_freeze_status += FREEZE_LATLON;
         }

         if(IO_head_freeze == TRUE)
         {
            DY_freeze_status += FREEZE_HEAD;
         }

         if(IOS_simtype == WARRIOR)
            simulate();
         else
            simulate();                             // Flight model

         //DY_freeze_status = last_freeze;

         IO_exchange_nav();                        // navigation module
         IS_deice();
      
         sim_run_time += delta_time;
      
         beacon_flash(IOS_marker_type);
         Boundary_ctrl();
         DC_dcl_control();

         HD_exchange_data();
         cwp_panel();
         rec_delay_counter++;

         if(rec_delay_counter == 4) //slow recording down to 1 in 4 frames = 40Hz ish
         {
            frame_ct++;
            if(frame_ct == MAX_REC)
            {
               frame_ct = 0;
            }
            rec_data(); // put current data into current record slot
            //printf("\n TIME %f",IO_rec_sen_out[frame_ct].sim_time);
            rec_delay_counter = 0;
         }

      }
      if(PETE_SHHHH == TRUE) // Shhh mode turn off all sound (only dynamics at moment)
      {
         IO_set_sound(0);
         DYS_dynsnd(0);
         IO_Sen_out->panel_gear_horn  = UNLIT;
         IO_Sen_out->panel_stall_horn = UNLIT;
      }
      else
      {
         IO_set_sound(1);//set sound inputs
         DYS_dynsnd(0);
      }

      if(outputs.duser[Y_D_CRASH] == TRUE)
      {
         dyn_state = DYN_HALT;
      }


   }
   //end of run io

   if(state != IO_last_state)
   {
      switch (state)
      {

      case 1 :
         printf("\nINIT");
         break;

      case 2 : 
         printf("\nHALT");
		 //printf("\n========== QTG Test Mode Inter 0 = Auto, 1 = Manual: (%d) ==========\n",QC_manualmode);	//Woody Mod
		 printf("\n########## QTG Test Mode 0 = Auto, 1 = Manual: (%d) ##########",qtg_ManualMode);		//Woody Mod
         //printf("\n QTG_GO OFF %d",QC_goflag);

         break;

      case 3 :
         printf("\nTRIM");
         break;

      case 4 :
         printf("\nRUN");
         break;

      case 5 :
         printf("\nPLAY BACK");
         break;

      case 6 :
         printf("\nQTG");
         break;

      default :
         printf("state : %d",state);
         break;
      }
   }
   

      IO_last_state = state;
      avail_time = (int) (sim_run_time - min_sim_time);
      if(avail_time > MAX_REC_TIME)
         avail_time = MAX_REC_TIME;

      last_freeze = SE_freeze_on;
      last_main_pwr = EL_output.power;


      if(PETE_SHHHH == TRUE) // Shhh mode turn off all sound (only dynamics at moment)
      {
         IO_set_sound(0);
         DYS_dynsnd(0);
      }
}


//************************************************************
//
//************************************************************


void IO_elec_in(void)
{

      EL_input.leftrpm = outputs.user[Y_A_L_RPM];
      EL_input.rightrpm = outputs.user[Y_A_R_RPM];

//************************************************************
//                        switches
//************************************************************

      switch_in.batt_switch = !(IO_Sen_in->elec_battery_switch);
      switch_in.left_alt = !(IO_Sen_in->elec_left_alternator_switch);
      switch_in.right_alt = !(IO_Sen_in->elec_right_alternator_switch);
      memcpy(&EL_input.switches,&switch_in,sizeof(switch_struct));

// ammeter switches
      if(IO_Sen_in->panel_left_ammeter_button == SW_ON)
      {
         EL_input.ammeter = GENERATOR_L;
      }
      else
      if(IO_Sen_in->panel_right_ammeter_button == SW_ON)
      {
         EL_input.ammeter = GENERATOR_R;
      }
      else
         EL_input.ammeter = BATTERY;

//************************************************************
//                Faults
//************************************************************

   memcpy(&EL_input.faults,&IO_Sen_faults.EF,sizeof(elec_fail));
   
   if(IO_Sen_in->MCB_alt_field_left_tripped)
   {
//      printf("\n  LEFT ALT TRIPPED       LEFT ALT TRIPPED");
      add_fault(BREAKER_ALT_L);
   }
   else
      rem_fault(BREAKER_ALT_L);

   if(IO_Sen_in->MCB_alt_field_right_tripped)
   {
      add_fault(BREAKER_ALT_R);
   }
   else
      rem_fault(BREAKER_ALT_R);
//***************************************************
//           Elec loads                             *
//***************************************************
   if(EL_output.power == TRUE)
   {
      load_in.ambient        = IO_avionic_pwr; // if elec power avail load is on
      load_in.defrost_fan    = !(IO_Sen_in->interseat_on_defrost_switch);
      load_in.flap_motor     = outputs.duser[Y_D_FLAPS];
      load_in.gear_pump      = gear_retract;
      load_in.heater         = (IO_Sen_in->interseat_fan_heat_switch != SW_OFF);
      load_in.landing_lights = (IO_Sen_in->elec_labding_lights_1_switch == SW_ON && (outputs.duser[Y_D_N_GEAR] > 0.9));
      load_in.landlight2     = (IO_Sen_in->elec_labding_lights_2_switch == SW_ON && (outputs.duser[Y_D_N_GEAR] > 0.9));
      load_in.lstarter       = !(IO_Sen_in->elec_left_start_switch);
      load_in.navigation     = !(IO_Sen_in->elec_navigation_lights_switch);
      load_in.pitot_heat     = !(IO_Sen_in->elec_pitot_heater_switch);
      load_in.prop_deice     = IC_ice_load;//!(IO_Sen_in->panel_propellor_deice_switch);
      load_in.radios_tx      = (!(IO_Sen_in->comm_1_PTT) || !(IO_Sen_in->comm_2_PTT));
      load_in.recognition    = !(IO_Sen_in->elec_recognition_lights_switch);
      load_in.rstarter       = !(IO_Sen_in->elec_right_start_switch);
      load_in.strobes        = !(IO_Sen_in->elec_anti_collision_lights_switch);
      load_in.vent_fan       = FALSE;
      memcpy(&EL_input.loads,&load_in,sizeof(load_struct));

   }
   else
      EL_input.loads = 0; // no power source
}
//************************************************************
//     set global settings from electrical output            *
//************************************************************

void IO_elec_out(void)
{
   
   IO_Sen_out->panel_annunciator_alt_out_lamp = (EL_output.lind || EL_output.rind);
   IO_Sen_out->panel_low_bus_lamp = (low_bus && EL_output.power);
// Global power flag
   IO_elec_pwr    = EL_output.power;
// Avionic power available   
   if(IO_elec_pwr && 
      ((IO_Sen_in->panel_avionisc_power_switch == SW_ON && IO_Sen_faults.AVF.radio_pwr == FALSE)
    || ( IO_Sen_in->panel_avionisc_emergency_bus_switch == SW_ON && IO_Sen_faults.AVF.standby_pwr == FALSE)))
   {
      IO_avionic_pwr = TRUE;
   }
   else
      IO_avionic_pwr = FALSE;

}

//************************************************************
//     searches for time index in record structure           *
//************************************************************

int search_frame(unsigned int secs)
{
float max_time = 0.0f;
int ct = 0;

   //find last entry
   for(ct = 0;ct < MAX_REC; ct++)
   {
      if(IO_rec_sen_out[ct].sim_time > max_time)
      {
         max_time = IO_rec_sen_out[ct].sim_time;
      }
   }

   int pass = 0;
   secs = (avail_time - secs);
   s_time = (int) floor(max_time - secs);
   while(s_time != (unsigned int) floor(IO_rec_sen_out[frame_ct].sim_time))
   {
      frame_ct++;
      if(frame_ct == MAX_REC)
      {
         frame_ct = 0;
         pass ++;
      }
      if(pass >= 2)
      {
         return 1;
      }
   }
   return 0;
}

//************************************************************
//  resets dynamics and record structure                     *
//************************************************************

void reset_dyn(void)
{
   reset_dynamics();
   EL_start();
   unsigned int rec_init_ct = 0;
   for(rec_init_ct = 0 ;rec_init_ct < MAX_REC; rec_init_ct++)
   {
       memset(&IO_rec_sen_out[rec_init_ct],0,sizeof(record_struct));
   }
   frame_ct = 0;
   sim_run_time  = 0.0;
   avail_time = 0;
   last_main_pwr = FALSE;
}
//************************************************************
//  init function for dynamics                               *
//************************************************************

void init_dynamics()
{
      IO_Sen_in  = (sen_in_struct *)malloc(sizeof(sen_in_struct));
	  OUTPUTinbox  = (OUTPUTin *)malloc(sizeof(OUTPUTin));
      IO_Sen_out = (sen_out_struct *)malloc(sizeof(sen_out_struct));
	  IO_Sen_qtg  = (sen_qtg_struct *)malloc(sizeof(sen_qtg_struct));
	  IO_Sen_screens  = (sen_screens_struct *)malloc(sizeof(sen_screens_struct));
      ET_tx_snd  = (dyn_to_snd *)malloc(sizeof(dyn_to_snd));
      memset(IO_Sen_out,0,sizeof(sen_out_struct));
      memset(IO_Sen_in,1,sizeof(sen_in_struct));
	  memset(OUTPUTinbox,1,sizeof(OUTPUTin)); //SIMULINKUDPRECEIVE
	  memset(IO_Sen_qtg,1,sizeof(sen_qtg_struct));
	  memset(IO_Sen_screens,2,sizeof(sen_screens_struct));
      memset(&IO_Sen_faults,0,sizeof(sen_fault_struct));
      memset(&EL_input,0,sizeof(EL_in));
      memset(&EL_output,0,sizeof(EL_out));
      memset(&IO_ndb_battr,0,sizeof(ndb_beac_attr));
      memset(&IO_dyn_ios,0,sizeof(dyn_to_ios));
      memset(&inputs,0,sizeof(INPUTS));
      memset(&states,0,sizeof(STATES));
      memset(&outputs,0,sizeof(OUTPUTS));
      memset(&ET_ios_data_in,0,sizeof(ios_to_dyn));
      memset(&DY_start,0,sizeof(StartData));
      memset(ET_tx_snd,0,sizeof(dyn_to_snd));
      memset(&IO_fastLog,10,sizeof(IO_fastLog));
      //memset(&qtg_stuff,0,sizeof(test_qtg));
      memset(&init_vals,0,sizeof(inital_cond));
      ET_ios_data_p = (ios_to_dyn *) &ET_ios_data_in;
      IO_Sen_out->system_servo_motor_power_control = 1;
  	   IO_Sen_out->system_DCL_power_control = 1;
      IO_Sen_out->system_power_control = 1;
	   IO_Sen_out->system_stepper_motor_power_control = 1;
      IO_Sen_out->dcl_mode = DCL_INIT;

      EL_start();
      CS_gyro_init();
      ILS_init();
      VOR_init();
      RMI_init();
      N1A_nav1_init();
      N2A_nav2_init();
      ADF_adf_init();
      CA_com_init();
      RA_rad_init();
      SC_init_snd_coms();
      WD_init_turb();
      
      reset_dynamics();
      BC_convert_host_pos(&BC_host_pos);
      DYS_dynsnd(1);

      //qtg_stuff.state = DYN_HALT;
      dyn_state = DYN_HALT;
      IO_Sen_in->pack6 = DYN_HALT;
      

}

//----------------------------------------------------------------------

void IO_exchange_nav(void)
{
      BC_convert_host_pos(&BC_host_pos);
      CS_calc_mag_head((float)(outputs.psi * RAD_TO_DEG), BC_host_pos.lat, BC_host_pos.lon);
      CS_update();
      CS_gyro_slave_button_changes();
      ADF_adf_update();
      CA_com_update();
      N1A_nav1_update();
      N2A_nav2_update();
      RA_rad_update();
      BC_update_beacon_dials();
}

//----------------------------------------------------------------------

void IO_set_sound(int state)
{

   if(state == 1)// normal mode so set sounds up
   {

      DYS_in.airspeed          = (float)(outputs.Vtas * MS_TO_KTS);
      DYS_in.engine_left       = (outputs.user[Y_A_L_RPM]);
      DYS_in.engine_right      = (outputs.user[Y_A_R_RPM]);

      if(wing_deice_flag == TRUE && last_deice == FALSE)
      {
         dyn_snd.flag_deice = TRUE;
      }
      else
         dyn_snd.flag_deice = FALSE;

      if(wing_deice_flag == FALSE && last_deice == TRUE)
      {
         dyn_snd.flag_deice_off = TRUE;
      }
      else
         dyn_snd.flag_deice_off = FALSE;

      last_deice = wing_deice_flag;

      if(states.user[X_N_GEAR] < last_ngear_state - 0.001)
      {
         gear_in_transit = TRUE;
			gear_retract    = TRUE;
      }
      else
      {
         gear_retract    = FALSE;
         gear_in_transit = FALSE;
      }


      if(gear_in_transit == FALSE && last_g_transit == TRUE)
      {
         dyn_snd.flag_gearoff = TRUE;
      }
      else
         dyn_snd.flag_gearoff = FALSE;

      if(gear_in_transit == TRUE && last_g_transit == FALSE)
      {
         dyn_snd.flag_geardown = TRUE;
      }
      else
         dyn_snd.flag_geardown = FALSE;


      last_g_transit = gear_in_transit;
      last_ngear_state = states.user[X_N_GEAR];

      if(EL_output.power == TRUE)
      {
         dyn_snd.flag_gyro = TRUE;         
      }
      
      if(EL_output.power == FALSE )
      {
         dyn_snd.flag_gyro = FALSE;
      }



      if(outputs.user[Y_A_L_RPM] > 100.0)
      {
         dyn_snd.flag_on_left = TRUE;
      }
      else
         dyn_snd.flag_on_left = FALSE;

      if(outputs.user[Y_A_R_RPM] > 100.0)
      {
         dyn_snd.flag_on_right = TRUE;
      }
      else
         dyn_snd.flag_on_right = FALSE;
      
//when any gear leg is fully extended play clunk sound
      if(outputs.duser[Y_D_N_GEAR] == TRUE && last_ngear == FALSE) 
      {
         dyn_snd.flag_clunk = TRUE;
      }
      else
         dyn_snd.flag_clunk = FALSE;

      if(outputs.duser[Y_D_L_GEAR] == TRUE && last_lgear == FALSE)
      {
         dyn_snd.flag_Lclunk = TRUE;
      }
      else
         dyn_snd.flag_Lclunk = FALSE;

      if(outputs.duser[Y_D_R_GEAR] == TRUE && last_rgear == FALSE)
      {
         dyn_snd.flag_Rclunk = TRUE;
      }
      else
         dyn_snd.flag_Rclunk = FALSE;

//set flap sound flags

      if((states.user[X_FLAP] < (last_flapPos - 0.01)) || (states.user[X_FLAP] > (last_flapPos + 0.01))) // ie flaps in motion
      {
         flap_move = TRUE;
//         printf("\n flaps %f %f",states.user[X_FLAP],last_flapPos);
      }
      else
         flap_move = FALSE;

      last_flapPos = states.user[X_FLAP];


        
      if(flap_move == TRUE && last_flap == FALSE)
      {
         dyn_snd.flag_flapup = TRUE;

      }
      else
      {
         dyn_snd.flag_flapup = FALSE;

      }

      if(flap_move == FALSE && last_flap == TRUE)
      {
         dyn_snd.flag_flapoff = TRUE;

      }
      else
      {
         dyn_snd.flag_flapoff = FALSE;

      }

      if(IO_Sen_in->elec_anti_collision_lights_switch == SW_ON && EL_output.power == TRUE)
      {
         dyn_snd.flag_strobe = TRUE;
      }
      else
         dyn_snd.flag_strobe = FALSE;

// SET PRIMMER  FLAGS
      if (EL_output.power == TRUE &&
          (IO_Sen_in->elec_right_primer_button == SW_ON ||
           IO_Sen_in->elec_right_fuel_switch == FUEL_LO || 
           IO_Sen_in->elec_right_fuel_switch == FUEL_HI))
      {
         //set sound on flag on
         dyn_snd.flag_primer_right = TRUE;
      }
      else
      {
         //set sound on flag off
         dyn_snd.flag_primer_right = FALSE;
      }

      if (EL_output.power == TRUE &&
          (IO_Sen_in->elec_left_primer_button == SW_ON ||
           IO_Sen_in->elec_left_fuel_switch == FUEL_LO ||
           IO_Sen_in->elec_left_fuel_switch == FUEL_HI))
      {
         //set sound on flag on
         dyn_snd.flag_primer_left = TRUE;
      }
      else
      {
         //set sound on flag off
         dyn_snd.flag_primer_left = FALSE;
      }
     
//SET MANIFOLD DRAIN FLAGS


      if((IO_Sen_in->panel_right_manifold_gauge_drain_button == SW_ON && outputs.duser[Y_D_R_ENGINE_LIT] == TRUE)
         || (IO_Sen_in->panel_left_manifold_gauge_drain_button == SW_ON && outputs.duser[Y_D_L_ENGINE_LIT] == TRUE))
      {
         dyn_snd.flag_manifold = TRUE;
      }
      else
      {
         //set sound on flag off
         dyn_snd.flag_manifold = FALSE;
      }

//AUTOPILOT BEEP

      if(AP_beep == TRUE && outputs.duser[Y_D_KFC150_AP] == TRUE)
      {
         dyn_snd.flag_autopilot = TRUE;
      }
      else      
         dyn_snd.flag_autopilot = FALSE;

      if(trim_caution)
      {
         dyn_snd.flag_autopilot = TRUE;
      }



      if(DY_l_start == TRUE && last_lstart == FALSE && (outputs.user[Y_A_L_RPM] < 500.0))
      {
         dyn_snd.flag_start_left = TRUE;
      }
      else
         dyn_snd.flag_start_left = FALSE;

      if(outputs.duser[Y_D_L_ENGINE_LIT] == TRUE)
      {
			dyn_snd.flag_restart_left = FALSE;
            
      }
      else
      if(outputs.user[Y_A_L_RPM] < 500.0 && dyn_snd.flag_start_left == TRUE)
         dyn_snd.flag_restart_left = TRUE;


      if(DY_r_start == TRUE && last_rstart == FALSE && (outputs.user[Y_A_R_RPM] < 500.0))
      {
         dyn_snd.flag_start_right = TRUE;
      }
      else
         dyn_snd.flag_start_right = FALSE;

         if(outputs.duser[Y_D_R_ENGINE_LIT] == TRUE)
         {
            dyn_snd.flag_restart_right = FALSE;
         }
         else
            if(outputs.user[Y_A_R_RPM] < 500.0 && dyn_snd.flag_start_right == TRUE)
            dyn_snd.flag_restart_right = TRUE;

//printf("\n LEFT on %d restart %d start %d ",dyn_snd.flag_on_left,dyn_snd.flag_restart_left,dyn_snd.flag_start_left);

     
      if(outputs.duser[Y_D_N_CONTACT] == TRUE)
      {
         n_wheelspd = (float)(outputs.Vtas * MS_TO_KTS);
      }
      else
         if(n_wheelspd > 0.0) n_wheelspd -= (float)(delta_time * 10.0);

      if(outputs.duser[Y_D_L_CONTACT] == TRUE)
      {
         l_wheelspd = (float)(outputs.Vtas * MS_TO_KTS);
      }
      else
         if(l_wheelspd > 0.0) l_wheelspd -= (float)(delta_time * 10.0);

      if(outputs.duser[Y_D_R_CONTACT] == TRUE)
      {
         r_wheelspd = (float)(outputs.Vtas * MS_TO_KTS);
      }
      else // if not in contact slow down wheel speed slowly
         if(r_wheelspd > 0.0) r_wheelspd -= (float)(delta_time * 10.0);

// Play skid sound if wheel spped has increased greatly
      if(n_wheelspd > (last_n_speed + 20.0))
      {
         dyn_snd.flag_skid = TRUE;
      }
      else
      if(l_wheelspd > (last_l_speed + 20.0))
      {
         dyn_snd.flag_skid = TRUE;
      }
      else
      if(r_wheelspd > (last_r_speed + 20.0))
      {
         dyn_snd.flag_skid = TRUE;
      }
      else
         dyn_snd.flag_skid = FALSE;
// store current wheel speed for next time
last_n_speed = n_wheelspd;
last_l_speed = l_wheelspd;
last_r_speed = r_wheelspd;

      // do crash sound
      if(outputs.duser[Y_D_CRASH] == TRUE && last_crash == FALSE)
      {
         dyn_snd.flag_crash = TRUE;
      }
      else
         dyn_snd.flag_crash = FALSE;

      
                  
      
   }
   else // turn off all sounds
   {
      DYS_in.airspeed           = 0.0;
      DYS_in.engine_left        = 0.0;
      DYS_in.engine_right       = 0.0;
      DYS_in.prop_left          = 0.0;
      DYS_in.prop_right         = 0.0;
      dyn_snd.flag_deice        = 0;
      dyn_snd.flag_gearoff      = 0;
      dyn_snd.flag_flapup       = 0;
      dyn_snd.flag_flapoff      = 0;
      dyn_snd.flag_geardown     = 0;
      dyn_snd.flag_start_left   = 0;
      dyn_snd.flag_start_right  = 0;
      dyn_snd.flag_crash        = 0;
      dyn_snd.flag_skid         = 0;
      dyn_snd.flag_autopilot    = 0;
      dyn_snd.flag_clunk        = 0;
      dyn_snd.flag_manifold     = 0;
      dyn_snd.flag_on_left      = 0;
      dyn_snd.flag_on_right     = 0;
      dyn_snd.flag_strobe       = 0;
      dyn_snd.flag_primer_left  = 0;
      dyn_snd.flag_primer_right = 0;
      dyn_snd.flag_restart_left   = 0;
      dyn_snd.flag_restart_right  = 0;
      dyn_snd.flag_gyro         = 0;


   }

   if(IO_engine_shhh != FALSE)
   {

      DYS_in.airspeed           = 0.0;
      DYS_in.engine_left        = 0.0;
      DYS_in.engine_right       = 0.0;
      DYS_in.prop_left          = 0.0;
      DYS_in.prop_right         = 0.0;
      dyn_snd.flag_restart_left   = 0;
      dyn_snd.flag_restart_right  = 0;
      dyn_snd.flag_on_left        = 0;
      dyn_snd.flag_on_right       = 0;
      dyn_snd.flag_start_left    = 0;
      dyn_snd.flag_start_right   = 0;
   }

   memcpy(&DYS_in.flags,&dyn_snd,sizeof(snd_in));

   last_gear   = outputs.duser[Y_D_N_GEAR];
   last_ngear  = outputs.duser[Y_D_N_GEAR];
   last_lgear  = outputs.duser[Y_D_L_GEAR];
   last_rgear  = outputs.duser[Y_D_R_GEAR];
   last_flap   = flap_move;
   last_rstart = DY_r_start;
   last_lstart = DY_l_start;
   last_nose   = outputs.duser[Y_D_N_CONTACT];
   last_left   = outputs.duser[Y_D_L_CONTACT];
   last_right  = outputs.duser[Y_D_R_CONTACT];
   last_crash  = outputs.duser[Y_D_CRASH];
   last_Rprim  = IO_Sen_in->elec_right_primer_button;
   last_Lprim  = IO_Sen_in->elec_left_primer_button;
   last_Rdrain = IO_Sen_in->panel_right_manifold_gauge_drain_button;
   last_Ldrain = IO_Sen_in->panel_left_manifold_gauge_drain_button;
}

//----------------------------------------------------------------------

void rec_data(void)
{
   last_frame = frame_ct;
   IO_rec_sen_out[frame_ct].sim_time = (float)sim_run_time;
   IO_rec_sen_out[frame_ct].record_index = frame_ct;
//   IO_rec_sen_out[frame_ct].delta_time = delta_time;
//   IO_Sen_out->rudder_demand_position = (float)avail_time;
//save the whole of hardware structure
   memcpy(&IO_rec_sen_out[frame_ct].IO_record,IO_Sen_out,sizeof(sen_out_struct));
   memcpy(&IO_rec_sen_out[frame_ct].snd_rec,ET_tx_snd,sizeof(dyn_to_snd));
   memcpy(&IO_rec_sen_out[frame_ct].dynSndRec,&DYS_in,sizeof(dynsound));
   memcpy(&IO_rec_sen_out[frame_ct].rec_states,&states,sizeof(STATES));
//craft position for visual
   IO_rec_sen_out[frame_ct].z = (float)states.D;
   IO_rec_sen_out[frame_ct].p = outputs.theta;
   IO_rec_sen_out[frame_ct].r = outputs.phi;
   IO_rec_sen_out[frame_ct].h = outputs.psi;
   IO_rec_sen_out[frame_ct].lat = states.latitude;
   IO_rec_sen_out[frame_ct].lon = states.longitude;
//flight control positions
   IO_rec_sen_out[frame_ct].pitch  = IO_Sen_in->control_yoke_pitch_position;
   IO_rec_sen_out[frame_ct].roll   = IO_Sen_in->control_yoke_roll_position;
   IO_rec_sen_out[frame_ct].rudder = IO_Sen_in->rudder_pedal_position;
   IO_rec_sen_out[frame_ct].pitch_trim = IO_Sen_out->pitch_trim_demand_position; 
}

//----------------------------------------------------------------------

void ret_data(void)
{
//copy active record to output structure
   memcpy(IO_Sen_out,&IO_rec_sen_out[frame_ct].IO_record,sizeof(sen_out_struct));
   memcpy(ET_tx_snd,&IO_rec_sen_out[frame_ct].snd_rec,sizeof(dyn_to_snd));
   memcpy(&DYS_in,&IO_rec_sen_out[frame_ct].dynSndRec,sizeof(dynsound));
   memcpy(&states,&IO_rec_sen_out[frame_ct].rec_states,sizeof(STATES));
//move controls to recorded position
   IO_Sen_out->pitch_demand_position  = IO_rec_sen_out[frame_ct].pitch;  
   IO_Sen_out->roll_demand_position   = IO_rec_sen_out[frame_ct].roll;
   IO_Sen_out->rudder_demand_position = IO_rec_sen_out[frame_ct].rudder;
   IO_Sen_out->pitch_trim_demand_position = IO_rec_sen_out[frame_ct].pitch_trim;
//set force to maximum
   IO_Sen_out->rudder_demand_force    = 50;
   IO_Sen_out->pitch_demand_force     = 30;
   IO_Sen_out->roll_demand_force      = 1;
   IO_Sen_out->pitch_trim_demand_force = 100.0;
//set craft position for visuals
   states.D = IO_rec_sen_out[frame_ct].z;
   outputs.theta    = IO_rec_sen_out[frame_ct].p;
   outputs.phi      = IO_rec_sen_out[frame_ct].r;
   outputs.psi      = IO_rec_sen_out[frame_ct].h;
   states.latitude  = IO_rec_sen_out[frame_ct].lat;
   states.longitude = IO_rec_sen_out[frame_ct].lon;
   BC_convert_host_pos(&BC_host_pos);
}
 
//----------------------------------------------------------------------

void add_fault(int f)
{
   EL_input.faults |= f;
}

//----------------------------------------------------------------------

void rem_fault(int f)
{
   EL_input.faults &= ~f;
}

//----------------------------------------------------------------------

void beacon_flash(int type)
{
#define INNER   3
#define MIDDLE  2
#define OUTER   1
#define I_EQUAL 1/12.0
#define O_EQUAL 0.25
#define M_ON1   0.2
#define M_OFF1  0.4 
#define M_ON2   0.7
#define M_OFF2  1.2
#define I_ON    (1.0/12.0)
#define I_OFF   (1.0/6.0)
#define O_ON    (1.0/3.0)
#define O_OFF   (1.0/2.0)
//#define EQUAL_TIME

   static int   last_marker = 0;
   static float beac_timer = 0.0;


   if(last_marker != type) //reset timer when a new beacon is detected
   {
      beac_timer = 0.0;
   }


   if(type == INNER) //inner marker
   {
/*      
#ifdef EQUAL_TIME
      if(beac_timer > I_EQUAL)
      {
         beac_timer = 0.0;
         IO_Sen_out->KR21_A_token = !IO_Sen_out->KR21_A_token;
      }
#else*/
      if(beac_timer < I_ON)
      {
         IO_Sen_out->KR21_A_token = LIT;
      }
      else
         if(beac_timer < I_OFF)
         {
            IO_Sen_out->KR21_A_token = UNLIT;
         }
         else
            beac_timer = 0.0000000; // reset timer
//#endif

      beac_timer += delta_time;
      IO_Sen_out->KR21_M_token = UNLIT;
      IO_Sen_out->KR21_O_token = UNLIT;

   }

   else
   if(type == MIDDLE) //middle marker
   {
      if(beac_timer < M_ON1)
      {
         IO_Sen_out->KR21_M_token = LIT;
      }
      else
         if(beac_timer < M_OFF1)
         {
            IO_Sen_out->KR21_M_token = UNLIT;
         }
         else
            if(beac_timer < M_ON2)
            {
               IO_Sen_out->KR21_M_token = LIT;
            }
            else
               if(beac_timer < M_OFF2)
               {
                  IO_Sen_out->KR21_M_token = UNLIT;
               }
               else
                  beac_timer = 0.0;

      beac_timer += delta_time;
      IO_Sen_out->KR21_A_token = UNLIT;
      IO_Sen_out->KR21_O_token = UNLIT;
   }
   else
   if(type == OUTER)
   {
      /*
#ifdef EQUAL_TIME
      if(beac_timer > O_EQUAL)
      {
         beac_timer = 0.0;
         IO_Sen_out->KR21_O_token = !IO_Sen_out->KR21_O_token;
      }
#else
*/
      if(beac_timer < O_ON)
      {
         IO_Sen_out->KR21_O_token = LIT;
      }
      else
         if(beac_timer < O_OFF)
         {
            IO_Sen_out->KR21_O_token = UNLIT;
         }
         else
            beac_timer = 0.0;
//#endif
      beac_timer += delta_time;
      IO_Sen_out->KR21_A_token = UNLIT; 
      IO_Sen_out->KR21_M_token = UNLIT;
   }
   else
   {
      beac_timer = 0.0;                 //set timer to 0
      IO_Sen_out->KR21_A_token = UNLIT; //turn off all marker lamps
      IO_Sen_out->KR21_M_token = UNLIT; //turn off all marker lamps
      IO_Sen_out->KR21_O_token = UNLIT; //turn off all marker lamps
   }

   last_marker = type;


}

//----------------------------------------------------------------------

//This routine calculates our position with respect to centre of
//any LOCAL boundary and calls all necessary routines

static void Boundary_ctrl(void)
{
   double hdist_sq = 0.0;
   float  hdist = 0.0f;
   float  wind[3];
   float  delta_ht = 0.0f;

   if (IOS_atis.valid)   //A local area has been defined
   {
      /* Find horizontal distance from local origin to host */
      hdist_sq = SPH_get_sph_flat_dist_sq (BC_host_pos.lat, BC_host_pos.lon, 0.0f,
                                           IOS_atis.AFLat, IOS_atis.AFLng, 0.0f);
//printf("\nHOST lat, lon = %f, %f", BC_host_pos.lat, BC_host_pos.lon);
//printf("\nAF lat, lon = %f, %f", IOS_atis.AFLat, IOS_atis.AFLng);
      hdist = (float) sqrt(hdist_sq);

      delta_ht = BC_host_pos.ht - IOS_atis.AFheight;
   }

   FC_cloud_ctrl (IOS_atis.valid, hdist, delta_ht);
   FC_fog_ctrl (IOS_atis.valid, hdist, delta_ht);
   BDY_pres_ctrl (IOS_atis.valid, hdist, delta_ht,DY_last_pressure,&DY_new_pressure);
   BDY_temp_ctrl (IOS_atis.valid, hdist, delta_ht, &DY_temp);

   WD_update_turb(DY_atmos.Turbulance);
   WD_wind_ctrl (IOS_atis.valid, hdist, delta_ht, wind);
   atmosphere.wE = wind[0];
   atmosphere.wN = wind[1];
   atmosphere.wD = wind[2];

}

//----------------------------------------------------------------------

int Set_controls(int frame,int mode)
{
   char pitch_pos = FALSE;
   char roll_pos = FALSE;
   char rudder_pos = FALSE;
   char pitchTrim_pos = FALSE;
//   extern float trimed_pitchpos;
   delay_dcl_timer += delta_time;

//   printf("pitch %f %f %f\n",IO_Sen_in->control_yoke_pitch_position,IO_Sen_out->pitch_demand_position,IO_Sen_out->pitch_demand_force ); //MODIFICADO POR MI AÑADIODO PARA VER DONDE SE COLOCA
   

   if (delay_dcl_timer > 5.0)
   {	
      if(mode == SET_CON_DCL)
      {
         set_cntrl = TRUE;
         
         DC_dcl_control();
   //IO_Sen_out->pitch_demand_position = -10.0;
//         IO_Sen_out->pitch_demand_position = trimed_pitchpos * 100.0f;
         if((IO_Sen_in->control_yoke_pitch_position < (IO_Sen_out->pitch_demand_position + 10)) &&
            (IO_Sen_in->control_yoke_pitch_position > (IO_Sen_out->pitch_demand_position - 10)))
         {
		      pitch_pos = TRUE;
            IO_Sen_out->pitch_demand_force = 10.0;
         }
	      else
		      IO_Sen_out->pitch_demand_force = 10.0;


         if((IO_Sen_in->control_yoke_roll_position < (IO_Sen_out->roll_demand_position + 8)) &&
      		(IO_Sen_in->control_yoke_roll_position > (IO_Sen_out->roll_demand_position - 8)))
         {
      		roll_pos = TRUE;
		      IO_Sen_out->roll_demand_force = 100.0;
         }
	      else
      		IO_Sen_out->roll_demand_force = 100.0;

         if((IO_Sen_in->rudder_pedal_position < (IO_Sen_out->rudder_demand_position + 8)) &&
		      (IO_Sen_in->rudder_pedal_position > (IO_Sen_out->rudder_demand_position - 8)))
         {
      		rudder_pos = TRUE;
		      IO_Sen_out->rudder_demand_force = 50.0;
         }
	      else
      		IO_Sen_out->rudder_demand_force = 50.0;

      

         IO_Sen_out->pitch_trim_demand_force = 100.0;
//         IO_Sen_out->pitch_trim_demand_position = -18.0;

         if((IO_Sen_in->pitch_trim_wheel < IO_Sen_out->pitch_trim_demand_position + 12.0) &&
            (IO_Sen_in->pitch_trim_wheel > IO_Sen_out->pitch_trim_demand_position - 12.0))
         {
            pitchTrim_pos = TRUE;
         }
         //printf("\n setting controls TRUE");
      }//end of dcl mode
      else
      if(mode == SET_CON_FRAME)
      {

         IO_Sen_out->pitch_demand_position      = IO_rec_sen_out[frame].pitch;  
         IO_Sen_out->roll_demand_position       = IO_rec_sen_out[frame].roll;
         IO_Sen_out->rudder_demand_position     = IO_rec_sen_out[frame].rudder;
         IO_Sen_out->pitch_trim_demand_position = IO_rec_sen_out[frame].pitch_trim;
   
   
         if((IO_Sen_in->control_yoke_pitch_position < (IO_Sen_out->pitch_demand_position + 10)) &&
		      (IO_Sen_in->control_yoke_pitch_position > (IO_Sen_out->pitch_demand_position - 10)))
         {
		      pitch_pos = TRUE;
		      IO_Sen_out->pitch_demand_force = 10.0;
         }
	      else
		      IO_Sen_out->pitch_demand_force = 10.0;


         if((IO_Sen_in->control_yoke_roll_position < (IO_Sen_out->roll_demand_position + 8)) &&
		      (IO_Sen_in->control_yoke_roll_position > (IO_Sen_out->roll_demand_position - 8)))
         {
		      roll_pos = TRUE;
		      IO_Sen_out->roll_demand_force = 100.0;
         }
	      else
		      IO_Sen_out->roll_demand_force = 100.0;

         if((IO_Sen_in->rudder_pedal_position < (IO_Sen_out->rudder_demand_position + 8)) &&
		      (IO_Sen_in->rudder_pedal_position > (IO_Sen_out->rudder_demand_position - 8)))
         {
		      rudder_pos = TRUE;
		      IO_Sen_out->rudder_demand_force = 50.0;
         }
	      else
		      IO_Sen_out->rudder_demand_force = 50.0;

      

         IO_Sen_out->pitch_trim_demand_force = 100.0;
         IO_Sen_out->pitch_trim_demand_position = -18.0;

         if((IO_Sen_in->pitch_trim_wheel < IO_Sen_out->pitch_trim_demand_position + 2.0) &&
            (IO_Sen_in->pitch_trim_wheel > IO_Sen_out->pitch_trim_demand_position - 2.0))
         {
            pitchTrim_pos = TRUE;
         }
         //printf("\n setting controls TRUE");
      }//end of frame mode
   } 
   else
   {
      IO_Sen_out->pitch_demand_position  = IO_Sen_in->control_yoke_pitch_position;
      IO_Sen_out->roll_demand_position   = IO_Sen_in->control_yoke_roll_position;
      IO_Sen_out->rudder_demand_position =  IO_Sen_in->rudder_pedal_position;
      IO_Sen_out->pitch_demand_force     = 0.0;
      IO_Sen_out->roll_demand_force      = 0.0;
      IO_Sen_out->rudder_demand_force    = 0.0;
      IO_Sen_out->pitch_trim_demand_position = (-set_stick_pos * auth_pitch_trim)*100.0f;
      IO_Sen_out->pitch_trim_demand_force = 100.0f;
      outputs.user[Y_A_PITCH_NULL] = set_stick_pos * auth_pitch_trim;
   }
   //return 1;
   if(pitch_pos == TRUE && roll_pos == TRUE && rudder_pos == TRUE && pitchTrim_pos == TRUE)
   {
      return 1;
   }
   else
      return 0;
}

//----------------------------------------------------------------------

//For use in special move mode only

void Special_move()
{
   float  start_x, start_y, start_z;
   float  dist, head;   //Need to work with true head
   double start_lat, start_lon;

   if (IO_Sen_in->KFC150_BC_button == SW_ON && last_bc_butt == SW_OFF)
   {
      if(DY_freeze_status == FREEZE_RB_ONLY)
      {
         DY_freeze_status = FREEZE_NED + FREEZE_LATLON;
         
      }
      else
      {
         IO_Sen_out->copilot_airspeed = 85.0;
         DY_freeze_status = FREEZE_RB_ONLY;
      }
   }

   if(DY_freeze_status == FREEZE_RB_ONLY)
   {
      IO_Sen_out->copilot_airspeed = 85.0;
   }

   if (IO_Sen_in->KFC150_TEST_button == SW_ON && last_test_butt == SW_OFF)
   {
      if (super_mode < SUPER_SPEED)
      {
         super_mode = SUPER_SPEED;
      }
      else
      {
         super_mode = NORM_SPEED;
      }
   }

if(!qtg_special)
{
   if (IO_Sen_in->KFC150_AP_ENG_button == SW_ON && last_ap_butt == SW_OFF)
   {
      if (super_sonic < SUPER_SPEED)
      {
         super_sonic = SUPERSONIC_SPEED;
      }
      else
      {
         super_sonic = NORM_SPEED;
      }
   }
}
else
{
   if (IO_Sen_in->KFC150_AP_ENG_button == SW_ON && last_ap_butt == SW_OFF)
   {
      if (super_sonic > SLOW_SPEED)
      {
         super_sonic = SLOW_SPEED;
      }
      else
      {
         super_sonic = NORM_SPEED;
      }
   }
}



   freeze_timer += delta_time;
   if(freeze_timer > FREEZE_FLASH)
   {
      if(IO_Sen_out->panel_annunciator_baggage_door_lamp == LIT)
      {
         IO_Sen_out->panel_annunciator_baggage_door_lamp = UNLIT;
         IO_Sen_out->panel_annunciator_cabin_door_lamp   = LIT;
      }
      else
      {
         IO_Sen_out->panel_annunciator_baggage_door_lamp = LIT;
         IO_Sen_out->panel_annunciator_cabin_door_lamp   = UNLIT;
      }
      freeze_timer = 0.0;
   }


   if(IO_Sen_in->KFC150_ALT_button == SW_ON && IO_Sen_out->copilot_airspeed > 80)
   {
      trans_mode = TRANS_ALT;
   }
   else if(IO_Sen_in->KFC150_HDG_button == SW_ON)
   {
      trans_mode = TRANS_SLIP;
   }
   else if(IO_Sen_in->KFC150_FD_button == SW_ON)
   {
      trans_mode = TRANS_FORE_AFT;
   }

   if(EL_output.power == FALSE)
   {
      trans_mode = FALSE;
      super_mode = NORM_SPEED;
   }

   switch(trans_mode)
   {
      case TRANS_ALT:
         IO_Sen_out->KFC150_ALT_token  = LIT;
         IO_Sen_out->KFC150_HDG_token  = UNLIT;
         IO_Sen_out->KFC150_FD_token   = UNLIT;
            
         if(IO_Sen_in->KFC150_DOWN_button == SW_ON && IO_Sen_out->copilot_airspeed > 80)
         {
            states.D += (float)(delta_time * UP_DOWN_SCALE * super_mode * super_sonic);
         }
         if(IO_Sen_in->KFC150_UP_button == SW_ON && IO_Sen_out->copilot_airspeed > 80)
         {
            states.D -= (float)(delta_time * UP_DOWN_SCALE * super_mode * super_sonic);
         }
         if((states.D * -1.0) < (terrain_height + 3.0))
            states.D = (float)((terrain_height + 3.0) * -1.0);
         break;

      case TRANS_SLIP:

         IO_Sen_out->KFC150_ALT_token  = UNLIT;
         IO_Sen_out->KFC150_HDG_token  = LIT;
         IO_Sen_out->KFC150_FD_token   = UNLIT;
            
         if(IO_Sen_in->KFC150_DOWN_button == SW_ON)
         {
            head = (float)(outputs.psi * R2D) + 90.0f;  //True
            if(head > 360.0) head -= 360.0;
            dist = (float)(delta_time * SLIP_RATE * super_mode * super_sonic); 

            UT_offset_along_dir (0, 0, 0, head, 0.0, dist,
                                 &start_x, &start_y, &start_z);

            UT_local_xy_2_ll(start_x, start_y, states.latitude * R2D, states.longitude * R2D,
                                    &start_lat, &start_lon);
            states.latitude  = (start_lat * D2R);
            states.longitude = (start_lon * D2R);
         }

         if(IO_Sen_in->KFC150_UP_button == SW_ON)
         {
            head = (float)(outputs.psi * R2D) - 90.0f;  //True
            if(head < 0.0) head += 360.0;
            dist = (float)(delta_time * SLIP_RATE * super_mode * super_sonic); 

            UT_offset_along_dir (0, 0, 0, head, 0.0, dist,
                                 &start_x, &start_y, &start_z);


            UT_local_xy_2_ll(start_x, start_y, states.latitude * R2D, states.longitude * R2D,
                             &start_lat, &start_lon);
            states.latitude  = (start_lat * D2R);
            states.longitude = (start_lon * D2R);
         }
         break;

      case TRANS_FORE_AFT:

         IO_Sen_out->KFC150_ALT_token  = UNLIT;
         IO_Sen_out->KFC150_HDG_token  = UNLIT;
         IO_Sen_out->KFC150_FD_token   = LIT;
         if(IO_Sen_in->KFC150_DOWN_button == SW_ON)
         {
            head = (float)(outputs.psi * R2D);  //True
            if(head > 360.0) head -= 360.0;
            dist = (float)(delta_time * FORE_AFT_RATE * super_mode * super_sonic); 

            UT_offset_along_dir (0, 0, 0, head, 0.0, dist,
                                 &start_x, &start_y, &start_z);


            UT_local_xy_2_ll(start_x, start_y, states.latitude * R2D, states.longitude * R2D,
                             &start_lat, &start_lon);
            states.latitude  = (start_lat * D2R);
            states.longitude = (start_lon * D2R);
         }

         if(IO_Sen_in->KFC150_UP_button == SW_ON)
         {
            head = (float)(outputs.psi * R2D);
            if(head < 0.0) head += 360.0;
            dist = (float)(delta_time * -FORE_AFT_RATE * super_mode * super_sonic); 

            UT_offset_along_dir (0, 0, 0, head, 0.0, dist,
                                 &start_x, &start_y, &start_z);


             UT_local_xy_2_ll(start_x, start_y, states.latitude * R2D, states.longitude * R2D,
                                 &start_lat, &start_lon);
             states.latitude  = (start_lat * D2R);
             states.longitude = (start_lon * D2R);
         }
         break;

      default:

         IO_Sen_out->KFC150_FD_token   = UNLIT;
         IO_Sen_out->KFC150_ALT_token  = UNLIT;
         IO_Sen_out->KFC150_HDG_token  = UNLIT;
         IO_Sen_out->KFC150_GS_token   = UNLIT;
         IO_Sen_out->KFC150_NAV_token  = UNLIT;
         IO_Sen_out->KFC150_APR_token  = UNLIT;

         IO_Sen_out->KFC150_AP_token   = UNLIT;
         break;
   }
         
   if(super_mode < SUPER_SPEED)
      IO_Sen_out->KFC150_TRIM_token   = UNLIT; // stop bc lamp going out in supermode and no mode selected
   else
      IO_Sen_out->KFC150_TRIM_token   = LIT;
            
   if(super_sonic < NORM_SPEED)
      IO_Sen_out->KFC150_AP_token   = LIT;
   else
   if(super_sonic < SUPERSONIC_SPEED)
      IO_Sen_out->KFC150_AP_token   = UNLIT; // stop bc lamp going out in supermode and no mode selected
   else
      IO_Sen_out->KFC150_AP_token   = LIT;
         

   if(DY_freeze_status == FREEZE_RB_ONLY)
   {
      IO_Sen_out->KFC150_BC_token = LIT;
   }
   else
      IO_Sen_out->KFC150_BC_token = UNLIT;


   if(qtg_special)
   {
      SE_freeze_on = TRUE;
      memcpy(&qtg_states,&states,sizeof(STATES));
   }


   last_bc_butt = IO_Sen_in->KFC150_BC_button;
   last_ap_butt = IO_Sen_in->KFC150_AP_ENG_button;
   last_test_butt = IO_Sen_in->KFC150_TEST_button;
}

//----------------------------------------------------------------------
static float clear_record(void)
{

   float min_time = IO_rec_sen_out[frame_ct].sim_time;
   int ct = 0;
   for(ct = 0;ct < MAX_REC; ct++)
   {
      if(IO_rec_sen_out[frame_ct].sim_time < IO_rec_sen_out[ct].sim_time)
      {
         memset(&IO_rec_sen_out[ct],0,sizeof(record_struct));
      }
      else
      {
         if(IO_rec_sen_out[ct].sim_time < min_time)
         {
            min_time = IO_rec_sen_out[ct].sim_time;
         }
      }

   }
   return min_time;

}