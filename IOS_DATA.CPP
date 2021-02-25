/*
File  : ios_data.cpp
Prefix: IOS_

Module: Dynamics For Seneca

Description: This file .......
*/

/***************************/
/*   RCS Markers           */
/***************************/

/*
$Header: C:/Master/seneca/RCS/ios_data.cpp 1.8 2000/06/16 09:27:28 ColinJ Exp $
$Log: ios_data.cpp $
Revision 1.8  2000/06/16 09:27:28  ColinJ
pass wind speed and direction and ground speed to IOS.
remove some debug.

Revision 1.7  2000/05/31 15:14:46  colinj
groundtrack set to previous yawangle. yawangle now set
to difference between aircraft heading and groundtrack.
Set up QC_manualmode and IO_engine_shhh.
Temporary fix for cloud top (still called CloudThk).

Revision 1.6  2000/05/30 14:34:27  colinj
Make magnetic variation active.
Send ac yawangle and heading to ios in magnetic.
Convert wind direction from ios in to true.

Revision 1.5  2000/05/23 10:59:37  colinj
Set up switch status data.

Revision 1.4  2000/05/18 10:03:53  juliae
SQUAWK ident now depends on RA_squawk_on flag.

Revision 1.3  2000/05/17 17:33:09  colinj
Renamed IO_iosfastLogid to QC_iosfastLogid.

Revision 1.2  2000/05/10 15:50:46  colinj
Qtg data to ios: changed throttle, prop and mixture values and
flappos and gforce.
Set up IO_iosfastlogid, IO_iosfreeze and IO_release values.
Compensate for atis cloud base being with respect to airfield height.
Prevent fog being turned off completely.

Revision 1.1  2000/04/14 17:27:15  colinj
Initial revision

*/

//----------------------------------------------------------------------

//----------------
// Include Files
//----------------

#include <math.h>
#include <windows.h>
#include <stdio.h>
#include "dat_stru/in_head.h"
#include "dat_stru/out_head.h"
#include "faults.h"
#include "nav/dyn_ios.h"
#include "fltdy/fmgdefs.h"
#include "define.h"
#include "fltdy\fms_defs.h"
#include "const.h"
#include "qtgdefs.h"
//----------------
// Defines
//----------------

#define MAX_VIS  50000

#define PETEBUG

//----------------
// Typedefs
//----------------

//--------------------
// External Routines
//--------------------

extern void BIO_read_beacon_from_ios(ios_to_dyn *ios2dyn);
extern void BIO_setup_beacon_2_ios(dyn_to_ios *dyn2ios);
extern float CS_true_to_mag_head(float true_head);
extern float CS_mag_to_true_head(float mag_head);

#ifdef TEST_HARNESS_BEAC
extern void HI_write_to_ios( dyn_to_ios *dyn_2_ios);
extern int HI_read_from_ios( ios_to_dyn *ios_2_dyn);
#endif

//--------------------
// External Variables
//--------------------

extern ll_pos              BC_host_pos;
extern STATES              states;
extern OUTPUTS             outputs;
extern INPUTS              inputs;
extern ATMOS               atmosphere;
extern sen_out_struct     *IO_Sen_out;
extern sen_in_struct      *IO_Sen_in;
extern OUTPUTin			*OUTPUTinbox;//simulinkudp
extern sen_fault_struct    IO_Sen_faults;
extern AtmosphericsData    DY_atmos;
extern StartData           DY_start;
extern WeightData          DY_weight;
extern ConditionalFailures DY_con_fail;
extern QTGfastlog          IO_fastLog;
extern QTG                 qtg;
extern inital_cond         init_vals;


extern float ail_max;
extern float apKcdi_dot;
extern float apKcdi;
extern float ave_roll_rate;
extern float power_time;
extern float nose_press;
extern float y_eng;
extern float debug_angle;
extern float debug_angle1;
extern float QD_dist_td;
extern float CS_magnetic_var;
extern float DY_calcedcofg;
extern float DY_total_weight;
extern float DY_wind_speed,DY_wind_dir;
extern float debug_cdi_dot;
extern float debug_cdi_dot_dem;
extern float rud_force_scale;
extern float rud_force_scale1;
extern float rud_force_scale2;
extern float QC_ave_force_value;
extern float QC_timer;

extern int QC_event_flag;
extern int IO_avionic_pwr;
extern int dyn_state;
extern int ios_state_flag;
extern int qtg_testNum;
extern int avail_time;
extern int PETE_SHHHH;
extern int IO_iosfreeze;
extern int IO_engine_shhh;
extern int QC_manualmode;	//QTG Manual Mode Thing ?
extern int QC_goflag;
extern int RA_squawk_on;

int qtg_ManualMode = FALSE;

extern int IO_head_freeze;
extern int IO_alt_freeze;
extern int IO_pos_freeze;

extern unsigned int ios_secs;

extern char QC_iosfastLogid;

///////////////////////////////Potter mod ////////////////////////////////////
/*float SWindSpeed;
float SWindDirection;
float MWindSpeed;
float MWindDirection;		//removed 06/09/2006
float HWindSpeed;
float HWindDirection;
*/
///////////////////////////////Potter mod ////////////////////////////////////

//----------------
// Globals
//----------------

AtisData IOS_atis;

int      IOS_marker_type = 0;
int      IOS_vis_view = CENTRE_VIEW;
int      IOS_simtype = 0;
int      IOS_skybox = 0;
//-----------------
// Local Variables
//-----------------

static switch_stat cpit_switch;

static float col_fix = 0.0f;
//--------------------
// Forward References
//--------------------

static void Set_run_ios_data(dyn_to_ios *dyn2ios);
static void Qtg_ex(dyn_to_ios *dyn2ios);
void        ret_init(dyn_to_ios *dyn2ios);
//----------------------------------------------------------------------

// Sets up the data to send to IOS. This depends on the current
// dyn_state
// Inputs: dyn_state

void IOS_set_data_to_ios(dyn_to_ios *dyn2ios)
{
   Qtg_ex(dyn2ios);

   dyn2ios->flags = 0;
   dyn2ios->host_lat = BC_host_pos.lat;
   dyn2ios->host_lon = BC_host_pos.lon;

   dyn2ios->host_ht  = (float) -states.D;    /* Above sea level */

   switch (dyn_state)
   {
      case INIT:
         break;
      case DYN_PLAY:
         break;
      case DYN_HALT:
         ret_init(dyn2ios); //when in halt send to ios init conditions for qtg
         break;
      case DYN_QTG:
         break;

      case DYN_RUN:
      case DYN_TRIM:
         Set_run_ios_data(dyn2ios);
         break;

      default:
         break;
   }
}

//----------------------------------------------------------------------

static void Qtg_ex(dyn_to_ios *dyn2ios)
{


   memcpy(&dyn2ios->ios_fastlog,&IO_fastLog,sizeof(QTGfastlog));
   //memset(&dyn2ios->ios_fastlog.values,10,sizeof(dyn2ios->ios_fastlog.values));
   //dyn2ios->ios_fastlog.values[30] = 30.0f;
   //dyn2ios->ios_fastlog.values[40] = 20.0f;
   //dyn2ios->ios_fastlog.values[50] = 30.0f;
   //dyn2ios->ios_fastlog.values[60] = 20.0f;
   //dyn2ios->ios_fastlog.values[70] = 30.0f;
   //dyn2ios->ios_fastlog.values[80] = 20.0f;
/*
   int i;
   for(i=-100;i<100;i++)
   {
      dyn2ios->ios_fastlog.values[i+100] = dyn2ios->ios_fastlog.id * 1.0f;
   }
*/




   cpit_switch.defrost_sw = IO_Sen_in->interseat_on_defrost_switch == SW_ON;

   if(IO_Sen_in->left_fuel_select == FUEL_OFF)
     cpit_switch.l_fuel_sw = 0;
   else
   if(IO_Sen_in->left_fuel_select == FUEL_ON)
      cpit_switch.l_fuel_sw = 1;
   else
      cpit_switch.l_fuel_sw = 2;

   if(IO_Sen_in->right_fuel_select == FUEL_OFF)
     cpit_switch.r_fuel_sw = 0;
   else
   if(IO_Sen_in->right_fuel_select == FUEL_ON)
      cpit_switch.r_fuel_sw = 1;
   else
      cpit_switch.r_fuel_sw = 2;

   if(IO_Sen_in->interseat_fan_heat_switch == SW_OFF)
   {
      cpit_switch.heat_on_sw = 0;
   }
   else
      cpit_switch.heat_on_sw = 1;

   memcpy(&dyn2ios->switch_status,&cpit_switch,sizeof(cpit_switch));

   if(qtg.testid == 8 || qtg.testid == 9)
   {
      col_fix = power_time;
   }
   else
   if(qtg.testid == 47 || qtg.testid == 48)
      col_fix = ave_roll_rate;      
   else
   if(qtg.testid == 40 || qtg.testid == 41 || qtg.testid == 19 || qtg.testid == 27
      || qtg.testid == 28 || qtg.testid == 23 || qtg.testid == 24 || qtg.testid == 43)
   {
      col_fix = QC_ave_force_value;
   }
   else
      col_fix = outputs.az / -9.81f;


   dyn2ios->ios_ac.event_start    = QC_event_flag;
   dyn2ios->ios_ac.timer          = QC_timer;
   dyn2ios->ios_ccs.thrLeft       = (short) floor (inputs.inps[U_A_L_THROT] * 100.0f);//(IO_Sen_in->left_throttle); 
   dyn2ios->ios_ccs.thrRight      = (short) floor (inputs.inps[U_A_R_THROT] * 100.0f);
   dyn2ios->ios_ccs.propLeft      = (short) floor (IO_Sen_in->left_prop_rpm);
   dyn2ios->ios_ccs.propRight     = (short) floor (IO_Sen_in->right_prop_rpm);
   dyn2ios->ios_ccs.mixtureLeft   = (short) floor (IO_Sen_in->left_engine_mixture);
   dyn2ios->ios_ccs.mixtureRight  = (short) floor (IO_Sen_in->right_engine_mixture);
   dyn2ios->ios_ccs.p1Lbrake      = (short) floor (IO_Sen_in->pilot_left_toebrake);
   dyn2ios->ios_ccs.p1Rbrake      = (short) floor (IO_Sen_in->pilot_right_toebrake);
   dyn2ios->ios_ccs.p2Lbrake      = (short) floor (IO_Sen_in->copilot_left_toebrake);
   dyn2ios->ios_ccs.p2Rbrake      = (short) floor (IO_Sen_in->copilot_right_toebrake);
   dyn2ios->ios_ccs.rudder_trim   = (short) floor (IO_Sen_in->rudder_trim_wheel);

   dyn2ios->ios_ac.goflag         =  QC_goflag;
   dyn2ios->ios_ac.altitude       = (short) floor ((-states.D - atmosphere.ht) * FT_IN_METRE);//(outputs.user[Y_A_P1A_ALT]);
   
   dyn2ios->ios_ac.roc            = (short) floor (outputs.user[Y_A_P1_ROC]);
   dyn2ios->ios_ac.airspeed       = (short) floor (outputs.user[Y_A_P1_ASI]);
   dyn2ios->ios_ac.stallwarner    = outputs.duser[Y_D_STALL];
   dyn2ios->ios_ac.gearpos        = (short) floor (states.user[X_R_GEAR]*100);
   dyn2ios->ios_ac.leftmap        = (short) floor (outputs.user[Y_A_L_MAN_P]);
   dyn2ios->ios_ac.rightmap       = (short) floor (outputs.user[Y_A_R_MAN_P]);
   dyn2ios->ios_ac.leftrpm        = (short) floor (outputs.user[Y_A_L_RPM]);
   dyn2ios->ios_ac.rightrpm       = (short) floor (outputs.user[Y_A_R_RPM]);

   dyn2ios->ios_ac.transportdelay = IO_Sen_in->test_visuals;

   dyn2ios->ios_ac.columnpos      = (short) floor (inputs.inps[U_A_LONG]*100);//(IO_Sen_in->control_yoke_pitch_position);
   dyn2ios->ios_ac.wheelpos       = (short) floor (inputs.inps[U_A_LAT]*100);//(IO_Sen_in->control_yoke_roll_position);
   dyn2ios->ios_ac.pedalpos       = (short) floor (inputs.inps[U_A_PED]*100);//(IO_Sen_in->rudder_pedal_position);
  
   //dyn2ios->ios_ac.wheelpos       = (short) (IO_Sen_in->control_yoke_roll_position); //(short) floor (inputs.inps[U_A_LAT]*100);//(IO_Sen_in->control_yoke_roll_position);
  // dyn2ios->ios_ac.pedalpos       = (short) (IO_Sen_in->rudder_pedal_position); //(short) floor (inputs.inps[U_A_PED]*100);//(IO_Sen_in->rudder_pedal_position);
  


   if(qtg.testid == 14 || qtg.testid == 15 || qtg.testid == 115)		//Woody Mod
      dyn2ios->ios_ac.pedalpos       = (short) floor (IO_Sen_in->rudder_pedal_position);

   dyn2ios->ios_ac.trimpos        = (short) floor (IO_Sen_in->pitch_trim_wheel);
   dyn2ios->ios_ac.flappos        = (short) (states.user[X_FLAP]);// * (100.0/40.0));
   //printf("\n FLAP POS %d",dyn2ios->ios_ac.flappos);
   dyn2ios->ios_ac.sideslip       = (short) floor (IO_Sen_out->copilot_slip/10.0f); //(outputs.user[Y_A_SLIP_ANGLE]);

   dyn2ios->ios_ac.columnforce    = IO_Sen_in->test_control_column_force;
   //if(dyn2ios->ios_ac.columnforce < 0.0f)
     // dyn2ios->ios_ac.columnforce = dyn2ios->ios_ac.columnforce * -1.0f;

   dyn2ios->ios_ac.wheelforce    = IO_Sen_in->test_control_column_force;
   //if(dyn2ios->ios_ac.wheelforce < 0.0f)
     // dyn2ios->ios_ac.wheelforce = dyn2ios->ios_ac.columnforce * -1.0f;

   dyn2ios->ios_ac.pedalforce    = IO_Sen_in->test_control_column_force;
   //if(dyn2ios->ios_ac.pedalforce < 0.0f)
     // dyn2ios->ios_ac.pedalforce = dyn2ios->ios_ac.columnforce * -1.0f;

   dyn2ios->ios_ac.distfromrw     = 0;
   dyn2ios->ios_ac.rollrate       = (short)(outputs.phi_dot * R2D);
   if(dyn2ios->ios_ac.rollrate < 0)
      dyn2ios->ios_ac.rollrate = (dyn2ios->ios_ac.rollrate * -1);
   dyn2ios->ios_ac.gforce         = col_fix;//ave_roll_rate;//(float)(outputs.az / -9.81);
   dyn2ios->ios_ac.visualrvrdist  = 0;//999;

 
   dyn2ios->ios_ac.bankangle      = (short) floor (outputs.phi*R2D);
   dyn2ios->ios_ac.pitchangle     = (short) floor (outputs.theta*R2D);
   if((short) floor (outputs.psi*R2D) < 0)
   {
      dyn2ios->ios_ac.heading        = (short) floor (CS_true_to_mag_head((float)(360 + (outputs.psi * R2D))));
   }
   else
      dyn2ios->ios_ac.heading        = (short) floor (CS_true_to_mag_head((float)(outputs.psi * R2D)));

    
   dyn2ios->replay_time    = avail_time; //amount of time in record struct
   dyn2ios->dynamics_state = dyn_state;
   if(dyn2ios->dynamics_state == DYN_TRIM)
      dyn2ios->dynamics_state = DYN_RUN;

   dyn2ios->ios_ac.groundtrack  = (short) floor (CS_true_to_mag_head((float)(outputs.track * R2D)));
   dyn2ios->ios_ac.yawangle = dyn2ios->ios_ac.heading - dyn2ios->ios_ac.groundtrack ;
   if(dyn2ios->ios_ac.yawangle < -90.0)
      dyn2ios->ios_ac.yawangle += 360;
	else
   if(dyn2ios->ios_ac.yawangle > +90.0)
      dyn2ios->ios_ac.yawangle -= 360;
   
#ifdef PETEBUG
int fdebugct = 0;
for (fdebugct=0;fdebugct<10;fdebugct++)
{
   dyn2ios->fdebug[fdebugct] = 0.0;
   dyn2ios->idebug[fdebugct] = 0;
}

 
 
   dyn2ios->fdebug[0] = DY_wind_speed;
   dyn2ios->fdebug[1] = DY_wind_dir;
   dyn2ios->fdebug[2] = outputs.Vg * MPS2KN;
   dyn2ios->fdebug[3] = DY_total_weight;

   dyn2ios->fdebug[4] = DY_calcedcofg;

   dyn2ios->fdebug[5] = outputs.user[Y_A_L_FUEL];
   dyn2ios->fdebug[6] = outputs.user[Y_A_R_FUEL];

   dyn2ios->fdebug[7] = y_eng;//IO_Sen_in->test_control_column_force;  //IO_Sen_in->control_yoke_roll_position;  //rud_force_scale;//atmosphere.wE;//IO_Sen_in->control_yoke_roll_position;  
   dyn2ios->fdebug[8] = debug_angle1;//dyn2ios->ios_fastlog.values[30];//IO_Sen_in->rudder_pedal_position; //debug_cdi_dot;//apKcdi;//atmosphere.wN;//ail_max;

   dyn2ios->fdebug[9] = debug_angle;//IO_Sen_out->rudder_demand_force;   //QC_timer;//IO_Sen_in->test_control_column_force;//y_eng;//IO_Sen_in->test_control_column_force;// rud_force_scale2;//dyn2ios->ios_ac.gforce ;//apKcdi_dot;;//inputs.inps[U_A_PED]*100.0f;

   dyn2ios->idebug[0] = QC_manualmode;	//QTG Manual Mode Thing ?
   dyn2ios->idebug[1] = QC_goflag;

   dyn2ios->idebug[2] = (int)IO_Sen_in->control_yoke_pitch_position;
   dyn2ios->idebug[3] = (int)IO_Sen_in->control_yoke_roll_position;
   dyn2ios->idebug[4] = (int)IO_Sen_in->rudder_pedal_position;

   dyn2ios->idebug[7] = IO_Sen_in->panel_propellor_deice_switch;

   
   
   
   
   
   

   
   
   

#endif
}

//----------------------------------------------------------------------

static void Set_run_ios_data(dyn_to_ios *dyn2ios)
{
   BIO_setup_beacon_2_ios(dyn2ios);

   //Setup COM info
   if ((IO_Sen_in->KY196_off_on_switch == SW_OFF) || (!IO_avionic_pwr) || IO_Sen_faults.AVF.com1)
      dyn2ios->com1_freq = 0;
   else
   {
      dyn2ios->com1_freq = IO_Sen_out->KY196_USE_display;
      if (IO_Sen_in->comm_1_PTT == SW_ON)
         dyn2ios->flags |= COM1_TALK_BIT;
   }
   if ((IO_Sen_in->KX155_off_on_switch == SW_OFF) || (!IO_avionic_pwr) || IO_Sen_faults.AVF.com_nav2)
      dyn2ios->com2_freq = 0;
   else
   {
      dyn2ios->com2_freq = IO_Sen_out->KX155_COMM_USE_display;
      if (IO_Sen_in->comm_2_PTT == SW_ON)
         dyn2ios->flags |= COM2_TALK_BIT;
   }

   //Setup RADAR info
   if (IO_avionic_pwr && (IO_Sen_in->KT76_off_on == KT76_ON_ALT) &&
       RA_squawk_on && !IO_Sen_faults.AVF.radar)
      dyn2ios->flags |= SQUAWK_BIT;

   if (IO_Sen_in->KR21_beacon_sensitivity == SW_ON)
      dyn2ios->flags |= MARKER_WIDE_BIT;

#ifdef TEST_HARNESS_BEAC
   HI_write_to_ios( dyn2ios);
#endif
   
}

//----------------------------------------------------------------------
 
void IOS_get_data_from_ios(ios_to_dyn *ios2dyn)
{
   //put failures into faults stucture
   memcpy(&IO_Sen_faults.FF,  &(ios2dyn->EngineMalfunctions),   sizeof(IO_Sen_faults.FF));   
   memcpy(&IO_Sen_faults.EI,  &(ios2dyn->EngineInstruments),    sizeof(IO_Sen_faults.EI));
   memcpy(&IO_Sen_faults.FI,  &(ios2dyn->FlightInstruments),    sizeof(IO_Sen_faults.FI));   
   memcpy(&IO_Sen_faults.EF,  &ios2dyn->ElectricalMalfunctions, sizeof(IO_Sen_faults.EF));
   memcpy(&IO_Sen_faults.AF,  &ios2dyn->Airframe,               sizeof(IO_Sen_faults.AF));
   memcpy(&IO_Sen_faults.AVF, &ios2dyn->Avionics,               sizeof(IO_Sen_faults.AVF));
   memcpy(&IO_Sen_faults.AS,  &ios2dyn->EngInstbitsAmm_Suct,    sizeof(IO_Sen_faults.AS));

   ios_state_flag = ios2dyn->dynamics_state;
   qtg_testNum = ios2dyn->TestNo;
   qtg_ManualMode = ios2dyn->manualmode;		//Woody Mod
   QC_manualmode   = ios2dyn->manualmode;		//Woody Mod Add This as it looks to only be in if the visual is tested
   ios_secs = ios2dyn->goto_time;
   QD_dist_td  = ios2dyn->dist_from_td;
   QC_iosfastLogid = ios2dyn->iosfastlogid;
   CS_magnetic_var = (float)(ios2dyn->mag_variation)/10.0f;

   if(ios2dyn->SimulationType & WARRIOR_BIT)
      IOS_simtype = WARRIOR;
   else
      IOS_simtype = SENECA;

   if(ios2dyn->SimulationType & SKYBOX_BIT)
      IOS_skybox = TRUE;
   else
      IOS_skybox = FALSE;
   

   memcpy(&DY_start,    &ios2dyn->Start,        sizeof(DY_start));
   memcpy(&DY_atmos,    &ios2dyn->Atmospherics, sizeof(DY_atmos));
   memcpy(&DY_weight,   &ios2dyn->Weight,       sizeof(DY_weight));
   memcpy(&IOS_atis,    &ios2dyn->Atis,         sizeof(IOS_atis));
   memcpy(&DY_con_fail, &ios2dyn->Conditional,  sizeof(DY_con_fail));

   //Compensate for local cloud base being given with respect to airfield height
   IOS_atis.CldBase += (unsigned int)IOS_atis.AFheight;

   //If no fog required, keep fog ON but at full range
   if(IOS_atis.VisOn == FALSE)
   {
      IOS_atis.VisOn = TRUE;
      IOS_atis.Visibility = MAX_VIS;
   }

   if(DY_atmos.VisOn == FALSE)
   {
      DY_atmos.VisOn = TRUE;
      DY_atmos.Vis   = MAX_VIS;
   }

///////////////////////////////Potter mod ////////////////////////////////////

   //Wind direction from IOS is magnetic. Convert to true bearing.
/*	SWindDirection = CS_mag_to_true_head((unsigned int) (DY_atmos.SWindDirection << 1));
	MWindDirection = CS_mag_to_true_head((unsigned int) (DY_atmos.MWindDirection << 1));
	HWindDirection = CS_mag_to_true_head((unsigned int) (DY_atmos.HWindDirection << 1));
	SWindSpeed = (float) DY_atmos.SWindSpeed;
	MWindSpeed = (float) DY_atmos.MWindSpeed;
	HWindSpeed = (float) ((unsigned int) (DY_atmos.HWindSpeed << 1));
*/
///////////////////////////////Potter mod ////////////////////////////////////

	DY_atmos.WindDirection = (short) (CS_mag_to_true_head(DY_atmos.WindDirection));

	IOS_atis.WindDir	     = (unsigned int) (CS_mag_to_true_head((float)IOS_atis.WindDir));

   
   IOS_marker_type = ios2dyn->marker_type;
   IOS_vis_view    = ios2dyn->visual_view;
   QC_manualmode   = ios2dyn->manualmode;		//QTG Manual Mode Thing ?
   PETE_SHHHH      = ios2dyn->ExFlags &0x01;
   //IO_iosfreeze    = ios2dyn->ExFlags &0x02;
   IO_engine_shhh  = ios2dyn->ExFlags &0x04;
   if(ios2dyn->ExFlags &0x10)
      IO_head_freeze  = TRUE;
   else
      IO_head_freeze = FALSE;

   if(ios2dyn->ExFlags &0x08)
      IO_alt_freeze = TRUE;
   else
      IO_alt_freeze = FALSE;
   
   if(ios2dyn->ExFlags &0x02)
      IO_pos_freeze   = TRUE;
   else
      IO_pos_freeze   = FALSE;

   

   

#ifdef TEST_HARNESS_BEAC
   HI_read_from_ios(ios2dyn);
#endif

   BIO_read_beacon_from_ios(ios2dyn);
}

//----------------------------------------------------------------------
void ret_init(dyn_to_ios *dyn2ios)
{
   dyn2ios->ios_ac.bankangle   = init_vals.oat;
   dyn2ios->ios_ac.pitchangle  = init_vals.Lfuel;
   dyn2ios->ios_ac.yawangle    = init_vals.Rfuel;
   dyn2ios->ios_ac.heading     = init_vals.ACweight;
   dyn2ios->ios_ac.columnpos   = init_vals.CofG;
   dyn2ios->ios_ac.leftrpm     = init_vals.Lrpm; 
   dyn2ios->ios_ac.leftmap     = init_vals.Lmap;
   dyn2ios->ios_ac.rightrpm    = init_vals.Rrpm;
   dyn2ios->ios_ac.rightmap    = init_vals.Rmap;
   dyn2ios->ios_ac.gearpos     = init_vals.gear;
   dyn2ios->ios_ac.flappos     = init_vals.flaps;
   dyn2ios->ios_ac.wheelpos    = init_vals.Lcowl;
   dyn2ios->ios_ac.pedalpos    = init_vals.Rcowl;
   dyn2ios->ios_ac.airspeed    = init_vals.speed;
   dyn2ios->ios_ac.altitude    = init_vals.alt;
   dyn2ios->ios_ac.rollrate    = init_vals.TestDuration;
   dyn2ios->ios_ac.groundtrack = init_vals.EventTime;
}