//QC qtg_control.cpp


//--------------------- 
//This file handles qtg modes 
//---------------------

//**************************
//   RCS Markers           *
//**************************


//$Header: C:/Master/seneca/RCS/qtg_control.cpp 1.5 2000/06/16 09:32:00 ColinJ Exp $
//$Log: qtg_control.cpp $
//Revision 1.5  2000/06/16 09:32:00  ColinJ
//tidy up.
//
//Revision 1.4  2000/05/31 11:33:30  colinj
//Full support for manual or automatic qtg mode, based on
//QC_manualmode flag.
//
//Revision 1.3  2000/05/26 11:48:43  colinj
//Rearrangement and use of QD_trimed flag.
//Support for manual flight of qtg.
//
//Revision 1.2  2000/05/23 11:03:35  colinj
//Correction to calling all cockpit system routines twice.
//New flag, QC_goflag.
//
//Revision 1.1  2000/05/17 17:32:26  colinj
//Initial revision
//


//----------------------------------------------------------------------

//----------------
// Include Files
//----------------

#include <stdio.h>

#include "dat_stru\in_head.h"
#include "dat_stru\out_head.h"
#include "define.h"



#include "fltdy\fmgdefs.h"
#include "fltdy\fmg.h"
#include "fltdy\fms_defs.h"
#include "fastlogger.h"
#include "qtgdefs.h"
#include "nav\dyn_ios.h"
#include "elec\electric.h"
//----------------
// Defines
//----------------

#define RIGHT_RUDDER 100
#define LEFT_RUDDER  -100
#define INACTIVE     0

#define QC_num_to_smooth 10
//#define EVENT_TRUE_ALL 1
//#define IGNORE_HARDWARE 1
//----------------
// Typedefs
//----------------

//--------------------
// External Routines
//--------------------
extern int QtgSetup( int testid );

extern void qtg_reset_dynamics();
extern void qtg_trim_flight(void);
extern void qtg_simulate();    // qtg flight model run
extern void simulate();    // flight model run
extern void qtg_change_controls(void);
extern void cwp_panel(void);
extern void IS_deice(void);

extern void IO_elec_in(void); 
extern void IO_elec_out(void);
extern void add_fault(int f);
extern void rem_fault(int f);

extern void IO_exchange_nav();
extern int Set_controls(int frame,int mode);
extern void HD_exchange_data(void);
extern void DC_dcl_control(void);
extern void DC_qtgdcl_control(void);

extern void DYS_dynsnd(int);
extern void IO_set_sound(int state);
extern void Special_move();
extern void BC_convert_host_pos(ll_pos *host_pos);


//----------------
// Externals
//----------------

extern sen_in_struct    *IO_Sen_in;
extern OUTPUTin			*OUTPUTinbox;//simulinkudp
extern sen_qtg_struct    *IO_Sen_qtg;
extern sen_out_struct   *IO_Sen_out;
extern QTGfastlog       *IO_fastLogsend;
extern INPUTS  inputs;
extern OUTPUTS outputs;
extern ATMOS   atmosphere;
extern STATES  states;

extern EL_in            EL_input;
extern EL_out           EL_output;
extern QTGfastlog       IO_fastLog;
extern ll_pos     BC_host_pos;
extern INPUTS  qtg_inputs;
extern OUTPUTS qtg_outputs;
extern ATMOS   qtg_atmosphere;
extern STATES  qtg_states;

extern float ah_air_press;
extern float last_vis_heading;
extern float delta_time;

extern int qtg_ManualMode;		//Woody Mod

extern int IO_last_state;
extern int QD_trimed;
extern int qtg_testNum;
extern int dyn_state;
extern int SE_freeze_on;
extern int IO_iosfreeze;
extern int DY_freeze_status;
extern int trans_mode;
extern int qtg_freeze;
extern int test_screen;
extern int QD_night;
extern char QD_first_reset;
extern char slam_pitch;
extern char slam_roll;
extern char slam_yaw;

//----------------
// Globals
//----------------

QTG              qtg;
inital_cond      init_vals;
float QC_ave_force_value = 0.0f;
float QC_timer = 0.0f;

int QC_manualmode = FALSE;
int QC_goflag = FALSE;
int QC_event_flag = FALSE;
int qtg_special = FALSE;
char no_dynamics_flag = FALSE;
char QC_iosfastLogid  = FALSE;

//-----------------
// Local Variables
//-----------------

static CFastLogger fastlogger;
static float tot_force = 0.0f;
static float qtg_run_timer = 0.0f;
static float trans_delay_ct = 0.0f;
static float tot_force_timer = 0.0f;
static float last_active_time = 0.0f;

static int set_con = FALSE;
static int con_type = TRUE;
static int qtgInitFlag   = FALSE;
static int qtgInitDelay  = FALSE;
static int qtg_fill_test = TRUE;
static int flret         = 0;
static int fastlgct      = 0;
static int fill_ct       = 0;
static int last_start    = 0;
static int change_rudder = FALSE;
static int smooth_rudd = TRUE;
static int rud_visit[200];

static char qtg_event = FALSE;
static char last_Ramm_but = SW_OFF;
static char last_Lamm_but = SW_OFF;
static char rudder_id     = INACTIVE;

//--------------------
// Forward References
//--------------------
static void Send_fastlogdata(void);
static void search_fastLog(int start);//,int *high, int *low);
static void fillholes(int high,int low);
static void set_init();

//----------------------------------------------------------------------
void QC_control(void)
{
      ah_air_press = 240;
	  //BEATERIO MODIFICADO AÑADIDO LAS DOS LINEAS
	  IO_Sen_out->pack28[0]=qtg.testid;
	  IO_Sen_out->qtgid =qtg.testid; 

      //printf("\n qtg timer %f ios timer %f",qtg_run_timer,QC_timer);
      if(qtg.testid == 71 || qtg.testid == 72 || ((qtg.testid > 56) && (qtg.testid < 60 )))
      {
         QD_night = TRUE;
      }
      else
         QD_night = FALSE;

      if(IO_last_state != DYN_QTG)
      {
            QtgSetup(qtg_testNum);
            smooth_rudd = TRUE;
            set_init();
            last_active_time = 0.0f;
            qtg_reset_dynamics();
            change_rudder = FALSE;
            rudder_id = INACTIVE;
            tot_force = 0.0f;
            QC_ave_force_value = 0.0f;
            tot_force_timer = 0.0f;
            qtg_run_timer = 0.0f;
            QC_timer = 0.0f;
            trans_delay_ct = 0.0f;
            qtg_event = FALSE;
            QD_first_reset = TRUE;
            qtg_fill_test  = TRUE;
            QC_event_flag = FALSE;
            memset(&IO_fastLog.values,0,sizeof(IO_fastLog.values));
            int i;
            for(i=0;i<200;i++)
            {
               rud_visit[i] = 0;
            }
            fastlgct = 0;
            flret = 0;
            fastlogger.i = 0;
            last_start = IO_Sen_in->system_freeze_button;
            

      }

      qtg_special = FALSE;
      if ( (qtg.testid >= 69) && (qtg.testid <= 72) ) // visual only test so do not run dynamics
      {
         if (QtgSetup(qtg_testNum) != 0)
         {
            dyn_state = DYN_HALT;
            return;
         }

         qtg_reset_dynamics();
         no_dynamics_flag = TRUE;
      }
      else
      if ((qtg.testid > 56) && (qtg.testid < 60 ))
      {
         if((IO_last_state != DYN_QTG) )
         {
            qtg_run_timer = 0.0;
            qtg_event = FALSE;
            QD_first_reset = TRUE;
            qtg_fill_test  = TRUE;

            memset(&IO_fastLog.values,0,sizeof(IO_fastLog.values));
            fastlgct = 0;
            flret = 0;
            fastlogger.i = 0;

            qtg_freeze = FREEZE_RB_ONLY;
            test_screen = TRUE;
            no_dynamics_flag = TRUE;
            
            qtg_reset_dynamics();
         }

         test_screen = TRUE;

         IO_Sen_out->dcl_mode = DCL_NORM;
         IO_Sen_out->pitch_demand_force = 30.0f;
         IO_Sen_out->roll_demand_force = 30.0f;
         IO_Sen_out->rudder_demand_force = 40.0f;
         IO_Sen_out->pitch_demand_position = -2.0f;
         IO_Sen_out->roll_demand_position = 0.0f;
         IO_Sen_out->rudder_demand_position = -4.5f;
         QC_goflag = FALSE;
         QC_event_flag = FALSE;
         no_dynamics_flag = TRUE;
         qtg_freeze = FREEZE_RB_ONLY;
         qtg_simulate();
         //DC_qtgdcl_control();
         qtg_special = TRUE;
         Special_move();
            //EL_output = EL_frame(& EL_input);       // Electrics module
         EL_output.power = TRUE;
         IO_elec_out();
         IO_exchange_nav();
         IS_deice();
         
         HD_exchange_data();
         IO_Sen_out->panel_gear_horn = UNLIT;
         cwp_panel();
         IO_set_sound(1);//set sound inputs
         DYS_dynsnd(0);
         BC_convert_host_pos(&BC_host_pos);
         qtg_run_timer += delta_time;;
         QC_timer = qtg_run_timer;

//////////////////////////////Woody Mod//////////////////////////////////////////////////////////////////////////////
/*
         if(qtg.testid == 57)
         {
            outputs.theta = (float)(IO_Sen_in->control_yoke_pitch_position * -D2R);
            outputs.psi = -175.5f *D2R;
            outputs.phi = 0.0f;
            IO_Sen_out->copilot_FCI_pitch_demand_position = -IO_Sen_in->control_yoke_pitch_position;
            IO_Sen_out->pilot_FCI_pitch_demand_position   = -IO_Sen_in->control_yoke_pitch_position;
            
            printf("\n trans delay_ct %f " ,trans_delay_ct);

            if(last_start != IO_Sen_in->system_freeze_button)
            {
               
               
               if(trans_delay_ct > 0.55f)
               {
                  IO_Sen_out->pitch_demand_force    = 70.0f;
                  IO_Sen_out->pitch_demand_position = -90.0f;
               }
               trans_delay_ct += delta_time;
               if(flret != LOGCOMPLETE)
               {
                  flret = fastlogger.Logger(IO_Sen_in->control_yoke_pitch_position,IO_Sen_in->test_light_sensor,IO_Sen_in->copilot_FCI_actual_pitch * 4.0f);
               }
               else
               {
                  QC_goflag = TRUE;
                  Send_fastlogdata();
               }
            }
            else
               qtg_run_timer = 0.0;
         }
         else if(qtg.testid == 58)
         {
            outputs.theta = 0.4f * D2R;
            outputs.phi = 0.0f;
            outputs.psi = (float)(180.0 - IO_Sen_in->rudder_pedal_position);
/*          if(outputs.psi > 359.0)
            {
               outputs.psi  = (float)(outputs.psi - 359.0);
            }
*/               
/*            IO_Sen_out->copilot_hsi_compass_demand_position = outputs.psi;
            IO_Sen_out->pilot_hsi_compass_demand_position   = outputs.psi;
            outputs.psi = (float)(outputs.psi * D2R);
            if(last_start != IO_Sen_in->system_freeze_button)
            {
               if(trans_delay_ct > 0.55f)
               {
                  IO_Sen_out->rudder_demand_force    = 70.0f;
                  IO_Sen_out->rudder_demand_position = 90.0f;
               }
               trans_delay_ct += delta_time;

               if(flret != LOGCOMPLETE)
               {
                  flret = fastlogger.Logger(IO_Sen_in->rudder_pedal_position,IO_Sen_in->test_light_sensor,(IO_Sen_in->copilot_hsi_compass - 185.0f)*5.0f);
               }
               else
               {
                  QC_goflag = TRUE;
                  Send_fastlogdata();
               }
            }
            else
               qtg_run_timer = 0.0;
         }
         else if(qtg.testid == 59)
         {
            outputs.psi = -175.5f *D2R;
            outputs.theta = 0.4f * D2R;
            outputs.phi = IO_Sen_in->control_yoke_roll_position;
            IO_Sen_out->copilot_FCI_roll_demand_position = outputs.phi;
            IO_Sen_out->pilot_FCI_roll_demand_position   = outputs.phi;
            outputs.phi = (float)(outputs.phi * D2R);
            if(last_start != IO_Sen_in->system_freeze_button)
            {
               if(trans_delay_ct > 0.55f)
               {
                  IO_Sen_out->roll_demand_force    = 70.0f;
                  IO_Sen_out->roll_demand_position = -90.0f;
               }
               trans_delay_ct += delta_time;

               if(flret != LOGCOMPLETE)
               {
                  flret = fastlogger.Logger(IO_Sen_in->control_yoke_roll_position,IO_Sen_in->test_light_sensor,IO_Sen_in->copilot_FCI_actual_roll);
               }
               else
               {
                  QC_goflag = TRUE;
                  Send_fastlogdata();
               }
            }
            else
               qtg_run_timer = 0.0;
         }
      }  //End of qtg.testid > 56 and < 60
*/
//////////////////////////////Woody Mod//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////Woody Mod////////////////////////////////////////////////////////////
         //if(qtg.testid == 57)
		if(qtg_ManualMode == FALSE &&(qtg.testid == 57))
         {
            outputs.theta = (float)(IO_Sen_in->control_yoke_pitch_position * -D2R);
            outputs.psi = -175.5f *D2R;
            outputs.phi = 0.0f;
            IO_Sen_out->copilot_FCI_pitch_demand_position = -IO_Sen_in->control_yoke_pitch_position;
            IO_Sen_out->pilot_FCI_pitch_demand_position   = -IO_Sen_in->control_yoke_pitch_position;
            //printf("pitch %f \n",IO_Sen_qtg->qtg_pilot_pitch);

            printf("\n trans delay_ct %f " ,trans_delay_ct);

            if(last_start != IO_Sen_in->system_freeze_button)
            {
               
               
               if(trans_delay_ct > 0.55f)
               {
                  IO_Sen_out->pitch_demand_force    = 70.0f;
                  IO_Sen_out->pitch_demand_position = -90.0f;
               }
               trans_delay_ct += delta_time;
               if(flret != LOGCOMPLETE)
               {
                  flret = fastlogger.Logger(IO_Sen_in->control_yoke_pitch_position,IO_Sen_in->test_light_sensor,IO_Sen_qtg->qtg_pilot_pitch);//  * 4.0f); //MODIFICADO POR MI ANTES COPILOT_FCI_actual_pitch
				printf("%f",IO_Sen_qtg->qtg_pilot_pitch);
			   }
               else
               {
                  QC_goflag = TRUE;
                  Send_fastlogdata();
               }
            }
            else
               qtg_run_timer = 0.0;
         }
		if(qtg_ManualMode == TRUE &&(qtg.testid == 57))
         {
            outputs.theta = (float)(IO_Sen_in->control_yoke_pitch_position * -D2R);
            outputs.psi = -175.5f *D2R;
            outputs.phi = 0.0f;
            IO_Sen_out->copilot_FCI_pitch_demand_position = -IO_Sen_in->control_yoke_pitch_position;
            IO_Sen_out->pilot_FCI_pitch_demand_position   = -IO_Sen_in->control_yoke_pitch_position;
            
            printf("\n trans delay_ct %f " ,trans_delay_ct);

            if(last_start != IO_Sen_in->system_freeze_button)
            {
               
               
               if(trans_delay_ct > 0.55f)
               {
                  IO_Sen_out->pitch_demand_force    = 70.0f;
                  IO_Sen_out->pitch_demand_position = 0.0f;
				 
               }
               trans_delay_ct += delta_time;
               if(flret != LOGCOMPLETE)
               {
                  flret = fastlogger.Logger(IO_Sen_in->control_yoke_pitch_position,IO_Sen_in->test_light_sensor,IO_Sen_qtg->qtg_pilot_pitch );// * 4.0f); //MODIFICADO POR MI ANTES COPILOT_FCI_actual_pitch
               }
               else
               {
                  QC_goflag = TRUE;
                  Send_fastlogdata();
               }
            }
            else
               qtg_run_timer = 0.0;
         }

//////////////////////////////////////Woody Mod////////////////////////////////////////////////////////////
         //else if(qtg.testid == 58)
			 else if(qtg_ManualMode == FALSE &&(qtg.testid == 58))
         {
            outputs.theta = 0.4f * D2R;
            outputs.phi = 0.0f;
            outputs.psi = (float)(180.0 - IO_Sen_in->rudder_pedal_position);
/*          if(outputs.psi > 359.0)
            {
               outputs.psi  = (float)(outputs.psi - 359.0);
            }
*/               
            IO_Sen_out->copilot_hsi_compass_demand_position = outputs.psi;
            IO_Sen_out->pilot_hsi_compass_demand_position   = outputs.psi;
            outputs.psi = (float)(outputs.psi * D2R);
            if(last_start != IO_Sen_in->system_freeze_button)
            {
               if(trans_delay_ct > 0.55f)
               {
                  IO_Sen_out->rudder_demand_force    = 70.0f;
                  IO_Sen_out->rudder_demand_position = 90.0f;
               }
               trans_delay_ct += delta_time;

               if(flret != LOGCOMPLETE)
               {
                  flret = fastlogger.Logger(-IO_Sen_in->rudder_pedal_position,IO_Sen_in->test_light_sensor,-(IO_Sen_qtg->qtg_pilot_compass  - 185.0f));//*5.0f); //MODIFICADO POR MI ANTES COPILOT_hsi_compass
               }
               else
               {
                  QC_goflag = TRUE;
                  Send_fastlogdata();
               }
            }
            else
               qtg_run_timer = 0.0;
         }

		  else if(qtg_ManualMode == TRUE &&(qtg.testid == 58))
         {
            outputs.theta = 0.4f * D2R;
            outputs.phi = 0.0f;
            outputs.psi = (float)(180.0 - IO_Sen_in->rudder_pedal_position);
/*          if(outputs.psi > 359.0)
            {
               outputs.psi  = (float)(outputs.psi - 359.0);
            }
*/               
            IO_Sen_out->copilot_hsi_compass_demand_position = outputs.psi;
            IO_Sen_out->pilot_hsi_compass_demand_position   = outputs.psi;
            outputs.psi = (float)(outputs.psi * D2R);
            if(last_start != IO_Sen_in->system_freeze_button)
            {
               if(trans_delay_ct > 0.55f)
               {
                  IO_Sen_out->rudder_demand_force    = 15.0f;
                  IO_Sen_out->rudder_demand_position = 0.0f;
               }
               trans_delay_ct += delta_time;

               if(flret != LOGCOMPLETE)
               {
                  flret = fastlogger.Logger(-IO_Sen_in->rudder_pedal_position,IO_Sen_in->test_light_sensor,-(IO_Sen_qtg->qtg_pilot_compass  - 185.0f));//*5.0f); //MODIFICADO POR MI ANTES COPILOT_hsi_compass
               }
               else
               {
                  QC_goflag = TRUE;
                  Send_fastlogdata();
               }
            }
            else
               qtg_run_timer = 0.0;
         }
//////////////////////////////////////Woody Mod////////////////////////////////////////////////////////////

         else if(qtg_ManualMode == FALSE &&(qtg.testid == 59))
         {
            outputs.psi = -175.5f *D2R;
            outputs.theta = 0.4f * D2R;
            outputs.phi = IO_Sen_in->control_yoke_roll_position;
            IO_Sen_out->copilot_FCI_roll_demand_position = outputs.phi;
            IO_Sen_out->pilot_FCI_roll_demand_position   = outputs.phi;
            outputs.phi = (float)(outputs.phi * D2R);
            if(last_start != IO_Sen_in->system_freeze_button)
            {
               if(trans_delay_ct > 0.55f)
               {
                  IO_Sen_out->roll_demand_force    = 70.0f;
                  IO_Sen_out->roll_demand_position = -90.0f;
               }
               trans_delay_ct += delta_time;

               if(flret != LOGCOMPLETE)
               {
                  flret = fastlogger.Logger(IO_Sen_in->control_yoke_roll_position,IO_Sen_in->test_light_sensor,IO_Sen_qtg->qtg_pilot_roll ); //MODIFICADO POR MI ANTES COPILOT_FCI_actual_roll
               }
               else
               {
                  QC_goflag = TRUE;
                  Send_fastlogdata();
               }
            }
            else
               qtg_run_timer = 0.0;
         }

		 else if(qtg_ManualMode == TRUE &&(qtg.testid == 59))
         {
            outputs.psi = -175.5f *D2R;
            outputs.theta = 0.4f * D2R;
            outputs.phi = IO_Sen_in->control_yoke_roll_position;
            IO_Sen_out->copilot_FCI_roll_demand_position = outputs.phi;
            IO_Sen_out->pilot_FCI_roll_demand_position   = outputs.phi;
            outputs.phi = (float)(outputs.phi * D2R);
            if(last_start != IO_Sen_in->system_freeze_button)
            {
               if(trans_delay_ct > 0.55f)
               {
                  IO_Sen_out->roll_demand_force    = 70.0f;
                  IO_Sen_out->roll_demand_position = 0.0f;
               }
               trans_delay_ct += delta_time;

               if(flret != LOGCOMPLETE)
               {
                  flret = fastlogger.Logger(IO_Sen_in->control_yoke_roll_position,IO_Sen_in->test_light_sensor,IO_Sen_qtg->qtg_pilot_roll ); //MODIFICADO POR MI ANTES COPILOT_FCI_actual_roll
               }
               else
               {
                  QC_goflag = TRUE;
                  Send_fastlogdata();
               }
            }
            else
               qtg_run_timer = 0.0;
         }

//////////////////////////////////////Woody Mod////////////////////////////////////////////////////////////
      }  //End of qtg.testid > 56 and < 60
      else
      if((IO_last_state != DYN_QTG) )
      {
         last_vis_heading = -10.0;

         if (QtgSetup(qtg_testNum) == 0)
         {
            if(qtg.testid >59 && qtg.testid < 69)
            {
               test_screen = TRUE;
               qtg_freeze = FREEZE_NED + FREEZE_LATLON;
               qtg.heading = 185;
            }
            set_init();
            qtg_reset_dynamics();            
            qtg_run_timer = 0.0;
            qtg_event = FALSE;
            QD_first_reset = TRUE;
            qtg_fill_test  = TRUE;
            memset(&IO_fastLog.values,0,sizeof(IO_fastLog.values));
            fastlgct = 0;
            flret = 0;
            fastlogger.i = 0;


            //printf("\n GO TO TRIM");
            //fflush(stdout);
            if(qtg.testid != 18 && qtg.testid != 17)
               qtg_trim_flight();
            else
               QD_trimed = TRUE;

            qtgInitDelay = 0;
            qtgInitFlag = FALSE;
         }
         else
         {
            //Failed to find QTG data
            printf("\n invalid qtg test num %d",qtg_testNum);
            dyn_state = DYN_HALT;
         }

      
      }//end qtg init
      else
      if(QD_trimed == FALSE)
      {
         qtg_trim_flight();
         set_con = FALSE;
         con_type = TRUE;
         memcpy(&outputs,&qtg_outputs,sizeof(OUTPUTS));
         memcpy(&states,&qtg_states,sizeof(STATES));
         memcpy(&inputs,&qtg_inputs,sizeof(INPUTS));
         memcpy(&atmosphere,&qtg_atmosphere,sizeof(ATMOS));

      }
      else
      if(set_con != TRUE)
      {
          IO_Sen_out->dcl_mode = DCL_INIT;
          DC_qtgdcl_control();
          set_con = Set_controls(0,con_type);
#ifdef IGNORE_HARDWARE
          set_con = TRUE;
#endif
          qtg_states.D = qtg.altitude / -FT_IN_METRE;
          states.D = qtg_states.D;
      }
      else
      {

         IO_Sen_out->dcl_mode = DCL_NORM;
         QC_goflag = TRUE;
         no_dynamics_flag = FALSE;
         qtg_run_timer += delta_time;
         QC_timer = qtg_run_timer;

         //printf("\n timer %f",qtg_run_timer);

         QC_manualmode = FALSE;		//Woody Mod to commnet out this
         if (QC_manualmode == TRUE)
         {
            /*
            if(SE_freeze_on == TRUE)//IO_release == FALSE)
            {
               if(DY_freeze_status == 0)
               {
                  DY_freeze_status = FREEZE_RB_ONLY;
               }
               Special_move();
            }
      
            if(SE_freeze_on == FALSE && IO_iosfreeze == FALSE)
            {
               DY_freeze_status = 0;
               trans_mode = FALSE;
            }*/
            simulate();
            DC_dcl_control();
            if(outputs.duser[Y_D_CRASH] == TRUE)
            {
               dyn_state = DYN_HALT;
            }
            IO_elec_in();        
            EL_output = EL_frame(& EL_input);       // Electrics module
            IO_elec_out();
            IO_exchange_nav();
            IS_deice();
            //DC_dcl_control();
            HD_exchange_data();
            //IO_Sen_out->panel_gear_horn = UNLIT;
            cwp_panel();
            IO_set_sound(1);//set sound inputs
            DYS_dynsnd(0);
         }
         else
         {
            if(qtg.testid >59 && qtg.testid < 69)
            {
               test_screen = TRUE;
               qtg_freeze = FREEZE_NED + FREEZE_LATLON;
               
               if(last_start != IO_Sen_in->system_freeze_button)
               {
                  if(flret != LOGCOMPLETE)
                  {
                     if(qtg.testid >59 && qtg.testid < 63)
                     {
                        slam_pitch = TRUE;
                        flret = fastlogger.Logger(IO_Sen_in->control_yoke_pitch_position,IO_Sen_in->test_light_sensor,IO_Sen_in->copilot_FCI_actual_pitch * 4.0f);
                     }
                     else
                     if(qtg.testid > 62 && qtg.testid < 66)
                     {
                        slam_yaw = TRUE;
                        flret = fastlogger.Logger(IO_Sen_in->rudder_pedal_position,IO_Sen_in->test_light_sensor,(IO_Sen_in->copilot_hsi_compass - 185.0f)*5.0f);
                     }
                     else
                     if(qtg.testid > 65 && qtg.testid < 69)
                     {
                        slam_roll = TRUE;
                        flret = fastlogger.Logger(IO_Sen_in->control_yoke_roll_position,IO_Sen_in->test_light_sensor,IO_Sen_in->copilot_FCI_actual_roll);
                     }

                  
                  }
                  else
                  {
                     QC_goflag = TRUE;
                     Send_fastlogdata();
                  }
               }
               else
                  qtg_run_timer = 0.0;

            }
            //if(qtg.testid != 18)
               qtg_simulate();
            //else
               //simulate();

            DC_qtgdcl_control();
            if(qtg.testid >59 && qtg.testid < 63)
            {
               outputs.psi   = -175.5f *D2R;
               outputs.theta = outputs.theta;
               outputs.phi   = 0.0f;
            }
            else
            if(qtg.testid > 62 && qtg.testid < 66)
            {
               outputs.psi   = outputs.psi;
               outputs.theta = 0.4f * D2R;
               outputs.phi   = 0.0f;
            }
            else
            if(qtg.testid > 65 && qtg.testid < 69)
            {
               outputs.psi   = -175.5f *D2R;
               outputs.theta = 0.4f * D2R;
               outputs.phi   = outputs.phi;
            }


            
            //EL_output = EL_frame(& EL_input);       // Electrics module
            EL_output.power = TRUE;
            IO_elec_out();
            IO_exchange_nav();
            IS_deice();

            HD_exchange_data();
            IO_Sen_out->panel_gear_horn = UNLIT;
            cwp_panel();
            IO_set_sound(1);//set sound inputs
            DYS_dynsnd(0);
            if(qtg.testid == 40 || qtg.testid == 41 || qtg.testid == 19 || qtg.testid == 27
                || qtg.testid == 28 || qtg.testid == 23 || qtg.testid == 24 || qtg.testid == 43)
            {
               if(qtg_run_timer > 30.0f)
               {
                  tot_force_timer += delta_time;
                  tot_force       += IO_Sen_in->test_control_column_force * delta_time;
               }
               if(qtg_run_timer > qtg.testduration -1)
               {
                  QC_ave_force_value = tot_force/tot_force_timer;
               }
            }

            if((qtg_run_timer > qtg.testeventtime) && qtg_event == FALSE && qtg.testeventtime > 0)
            {
               qtg_event = TRUE;
               QC_event_flag = TRUE;
               qtg_change_controls();
            }
#ifdef EVENT_TRUE_ALL
            else
            if(qtg.testeventtime == 0)
            {
               QC_event_flag = TRUE;
            }
#endif

            if((qtg.testid == 10) || (qtg.testid == 11 ))
				{
				if(qtg_run_timer < (qtg.testduration - 3))
					{
					if(IO_Sen_in->control_yoke_pitch_position < IO_Sen_out->pitch_demand_position)
						{
						if(IO_Sen_in->test_control_column_force > IO_fastLog.values[(int)(IO_Sen_in->control_yoke_pitch_position+100.0)])
							{
							IO_fastLog.values[(int)(IO_Sen_in->control_yoke_pitch_position+100.0)] = IO_Sen_in->test_control_column_force;
							}
						}
					else
					if(IO_Sen_in->control_yoke_pitch_position > IO_Sen_out->pitch_demand_position)
						{
						if(IO_Sen_in->test_control_column_force < IO_fastLog.values[(int)(IO_Sen_in->control_yoke_pitch_position+100.0)])
							{
							IO_fastLog.values[(int)(IO_Sen_in->control_yoke_pitch_position+100.0)] = IO_Sen_in->test_control_column_force;
							}
						}
					}
			   
			   
// Pruebas de tomas 01102013			   
			   else
					{//MODIFICADO POR MI TODO EL ELSE ABO
					int i =0;
					int x=0;
					int x1=0;
                    float newey[240];
					for ( x=0;x<240;x++)
						{
						newey[x]=0;
						x1=x;
						if (abs(IO_fastLog.values[x1-1]-IO_fastLog.values[x1]) >3 || IO_fastLog.values[x1]==0)
							{
							if (IO_fastLog.values[x1-1]!=0)
								{
								newey[x1]=(IO_fastLog.values[x1-1]);
								}
							if (IO_fastLog.values[x1+1]!=0)
								{
								newey[x1]=(IO_fastLog.values[x1+1]);
								}
							if (IO_fastLog.values[x1-1]==0 && IO_fastLog.values[x1+1]==0)
								{
								if (IO_fastLog.values[x1-2]!=0)
									{
									newey[x1]=(IO_fastLog.values[x1-1]);
									}
								if (IO_fastLog.values[x1+2]!=0)
									{
									newey[x1]=(IO_fastLog.values[x1+1]);
									}
								}
							}
						else
							{
							newey[x1]=IO_fastLog.values[x1];
							}
						}


                     for(i=0;i<240;i++)//MODIFICADO POR MI ANTES 195 en vez de 200
                     {
                        IO_fastLog.values[i] = newey[i];
                     }

			  }

// Fin pruebas de Tomas 
            }

            if((qtg.testid == 12) || (qtg.testid == 13 ))
            {
               if(qtg_run_timer < (qtg.testduration - 3))
               {
                  if(IO_Sen_in->control_yoke_roll_position < IO_Sen_out->roll_demand_position)
                  {
                     if(IO_Sen_in->test_control_column_force < IO_fastLog.values[(int)(IO_Sen_in->control_yoke_roll_position+100.0)])
                     {
                        IO_fastLog.values[(int)(IO_Sen_in->control_yoke_roll_position+100.0)] = IO_Sen_in->test_control_column_force;
 					 }
				  }
                  else
                  if(IO_Sen_in->control_yoke_roll_position > IO_Sen_out->roll_demand_position)
                  {
                     if(IO_Sen_in->test_control_column_force > IO_fastLog.values[(int)(IO_Sen_in->control_yoke_roll_position+100.0)])
                     {
                        IO_fastLog.values[(int)(IO_Sen_in->control_yoke_roll_position+100.0)] = IO_Sen_in->test_control_column_force;
					 }
                  }

			}else{//MODIFICADO POR MI TODO EL ELSE


                     int i =0;
                     float newey[240];



				for (int x=0;x<240;x++)
				{
					if (IO_fastLog.values[x]==0 && x>0)
					{
					//	if (x<98 && x>102)
					//	{
						//	if (IO_fastLog.values[x-1]!=0 && IO_fastLog.values[x+1]!=0)
						//	{
						//		newey[x]=(IO_fastLog.values[x-1]+IO_fastLog.values[x+1])/2;
						//	}else{
								if (IO_fastLog.values[x-1]!=0)
								{
									newey[x]=(IO_fastLog.values[x-1]);
								}
								if (IO_fastLog.values[x+1]!=0)
								{
									newey[x]=(IO_fastLog.values[x+1]);
								}

								if (IO_fastLog.values[x-1]==0 && IO_fastLog.values[x+1]==0)
								{
									if (IO_fastLog.values[x-2]!=0)
									{
										newey[x]=(IO_fastLog.values[x-1]);
									}
									if (IO_fastLog.values[x+2]!=0)
									{
										newey[x]=(IO_fastLog.values[x+1]);
									}

								}


						//	}
					//	}
					}else{
						newey[x]=IO_fastLog.values[x];
					}
				}


                     for(i=0;i<240;i++)//MODIFICADO POR MI ANTES 195 en vez de 200
                     {
                        IO_fastLog.values[i] = newey[i];
                     }
			   
			}
//               if (IO_fastLog.values[(int)(IO_Sen_in->control_yoke_roll_position+100.0)] < 19.0) 
  //                IO_fastLog.values[(int)(IO_Sen_in->control_yoke_roll_position+100.0)] = IO_Sen_in->test_control_column_force;
            }

            if((qtg.testid == 14) || (qtg.testid == 15) || (qtg.testid == 115 ))	//Woody Mod
            {
//               if (IO_fastLog.values[(int)(IO_Sen_in->rudder_pedal_position+100.0)] < 14.0)
//                 IO_fastLog.values[(int)(IO_Sen_in->rudder_pedal_position+100.0)] = IO_Sen_in->test_control_column_force;
/*               
               if(qtg_run_timer < (qtg.testduration - 3))
               {
                  if(IO_Sen_in->rudder_pedal_position < IO_Sen_out->rudder_demand_position)
                  {
                     if(IO_Sen_in->test_control_column_force < IO_fastLog.values[(int)(IO_Sen_in->rudder_pedal_position+100.0)])
                     {
                        IO_fastLog.values[(int)(IO_Sen_in->rudder_pedal_position+100.0)] = IO_Sen_in->test_control_column_force;
                     }
                  }
                  else
                  if(IO_Sen_in->rudder_pedal_position > IO_Sen_out->rudder_demand_position)
                  {
                     if(IO_Sen_in->test_control_column_force < IO_fastLog.values[(int)(IO_Sen_in->rudder_pedal_position+100.0)])
                     {
                        IO_fastLog.values[(int)(IO_Sen_in->rudder_pedal_position+100.0)] = IO_Sen_in->test_control_column_force;
                     }
                  }

               }
*/
               if(qtg_run_timer < qtg.testduration - 1)
               {
                  if(IO_Sen_in->panel_left_ammeter_button == SW_ON && last_Lamm_but == SW_OFF)
                  {
                     if(rudder_id == LEFT_RUDDER || rudder_id == RIGHT_RUDDER)
                     {
                        rudder_id = INACTIVE;
                     }
                     else
                        rudder_id = LEFT_RUDDER;
                  }

                  if(IO_Sen_in->panel_right_ammeter_button == SW_ON && last_Ramm_but == SW_OFF)
                  {
                     if(rudder_id == LEFT_RUDDER || rudder_id == RIGHT_RUDDER)
                     {
                        rudder_id = INACTIVE;
                     }
                     else
                        rudder_id = RIGHT_RUDDER;
                  }

                  if(rudder_id != INACTIVE)
                  {


                     if((IO_Sen_in->test_control_column_force * -1.0f) > IO_fastLog.values[(int)(IO_Sen_in->rudder_pedal_position+100.0)])
                     IO_fastLog.values[(int)(IO_Sen_in->rudder_pedal_position+100.0)] = IO_Sen_in->test_control_column_force * -1.0f;
                     //rud_visit[(int)(IO_Sen_in->rudder_pedal_position+100.0)] ++;
                  } 

                  if(rudder_id == INACTIVE)
                  {
                     qtg_run_timer = last_active_time;
                  }
                  else
                     last_active_time = qtg_run_timer;
               }
               else
               {
                  if(smooth_rudd == TRUE)
                  {
                     /*
                     int ct = 0;
                     float ave_rud = 0.0f;
                     float tot_rud = 0.0f;
                     float hi_rud  = 0.0f;
                     float low_rud = 0.0f;
                     float step_rud = 0.0f;
                     for(ct = 0; ct < 100; ct += QC_num_to_smooth)
                     {
                        ave_rud = 0.0f;
                        tot_rud = 0.0f;
                        step_rud  = 0.0f;
                        hi_rud = 0.0f;
                        low_rud = 0.0f;

                        
                        int a = 0;
                        for(a = 0;a < QC_num_to_smooth;a++)
                        {
                           tot_rud += IO_fastLog.values[ct+a];
                        }

                        ave_rud = tot_rud/QC_num_to_smooth;
                        step_rud = 

                        for(a = 1;a < QC_num_to_smooth -1;a++)
                        {
                           IO_fastLog.values[ct+a] = hi_rud -= step_rud;
                        }
                     }*/
                     /*
                     int ct = 0;
                     for(ct = 0;ct<200;ct++)
                     {
                        if(rud_visit[ct] > 0)
                        {
                           IO_fastLog.values[ct] = IO_fastLog.values[ct]/rud_visit[ct];
                        }
                     }*/

			
					 int divide=0;
                     int i =0;
                     float newey[200];
                     for(i=0;i<200;i++)//MODIFICADO POR MI ANTES 195 en vez de 200
                     {
						 if (i<5)
						 {
							divide=0;
							for (int yyy=0;yyy<10;yyy++)
							{
								if (IO_fastLog.values[i+yyy]!=0)
								{
									divide++;
								}
							}
							newey[i] = (IO_fastLog.values[i] + 
								IO_fastLog.values[i+1] + IO_fastLog.values[i+2] + IO_fastLog.values[i+3]
								+ IO_fastLog.values[i+4] + IO_fastLog.values[i+5]+ IO_fastLog.values[i+6]+ IO_fastLog.values[i+7]
								+ IO_fastLog.values[i+8]+ IO_fastLog.values[i+9])/divide;///11.0f;

						 }else{
							divide=0;
							for (int yyy=-5;yyy<6;yyy++)
							{
								if (IO_fastLog.values[i+yyy]!=0)
								{
									divide++;
								}
							}
							newey[i] = (IO_fastLog.values[i-5] + IO_fastLog.values[i-4] + IO_fastLog.values[i-3] +
								IO_fastLog.values[i-2] + IO_fastLog.values[i-1] + IO_fastLog.values[i] + 
								IO_fastLog.values[i+1] + IO_fastLog.values[i+2] + IO_fastLog.values[i+3]
								+ IO_fastLog.values[i+4] + IO_fastLog.values[i+5])/divide;///11.0f;

						 }
                     }


                     for(i=5;i<200;i++)//MODIFICADO POR MI ANTES 195 en vez de 200
                     {
                        IO_fastLog.values[i] = newey[i];
                     }
                        
                     

                     
                     smooth_rudd = FALSE;
                  }
               }


               IO_Sen_out->ammeter = rudder_id;

            }

            if((qtg.testid > 9) && (qtg.testid < 16) || (qtg.testid == 115)) 
            {
               //IO_fastLog.id = 4;

               DC_dcl_control();
               //qtg_fill_test = TRUE;
               if(qtg_run_timer > (qtg.testduration - 1))
               {
                  IO_fastLog.id = 4;
                  /*
                  
                  for(fill_ct = 1;fill_ct<200;fill_ct++)
                  {
                     if((qtg.testid == 10) || (qtg.testid == 11 ))
                     {
                        //IO_fastLog.values[fill_ct] = IO_fastLog.values[fill_ct] * -1.0f;

                     }

                  }*/ /*int ct = 0;
                  float col_force[41];
                  float col_pos[41] = {0,5,10,15,20,25,30,35,40,45,50,55,60,65,70,75,80,85,90,95,100,105,110,115,120,125,130,135,140,145,150,155,160,165,170,175,180,185,190,195,200};

                  for(fill_ct = 0;fill_ct < 205;fill_ct+=5)
                  {
                     col_force[ct] = IO_fastLog.values[fill_ct];
                     ct++;
                  }
                  for(fill_ct =0;fill_ct<205;fill_ct++)
                  {
                     IO_fastLog.values[fill_ct] = table1D(col_pos,col_force,41,(float)fill_ct);
                  }*/


                     
                     
/*

                  for(fill_ct = 0;fill_ct<200;fill_ct++)
                  {
                     if(IO_fastLog.values[fill_ct] < 2.0f)
                     {
                        //need a function to search both directions for a value!
                        search_fastLog(fill_ct);
                     }
                     else
                        if(IO_fastLog.values[fill_ct] < (IO_fastLog.values[fill_ct-1] - 5.0f))
                        {
                           search_fastLog(fill_ct);
                        }
                     
                  }

*/


               }
            }


            if (qtg_run_timer > qtg.testduration)
            {
               dyn_state = DYN_HALT;
               QC_goflag = FALSE;
               QC_event_flag = FALSE;
            }
         }//end if automode
      }
      if (qtg_run_timer > qtg.testduration)
      {
         dyn_state = DYN_HALT;
         QC_goflag = FALSE;
         QC_event_flag = FALSE;
          
      }

      //last_start = IO_Sen_in->system_freeze_button;
      last_Lamm_but = IO_Sen_in->panel_left_ammeter_button;
      last_Ramm_but = IO_Sen_in->panel_right_ammeter_button;
}
//----------------------------------------------------------------------

static void Send_fastlogdata(void)
{
   fastlgct = QC_iosfastLogid + 1;
   if(fastlgct < 4)
   {
      IO_fastLog.id = (short)fastlgct;
      if(IO_fastLog.id == 1)
      {
         memcpy(&IO_fastLog.values,fastlogger.arControl,sizeof(IO_fastLog.values));
      }
      else
      if(IO_fastLog.id == 2)
      {
         memcpy(&IO_fastLog.values,fastlogger.arLightLevel,sizeof(IO_fastLog.values));
      }
      if(IO_fastLog.id == 3)
      {
         memcpy(&IO_fastLog.values,fastlogger.arPotPos,sizeof(IO_fastLog.values));
      }
   }
//   else
  //    QC_goflag = TRUE;

//   else
  //    dyn_state = DYN_HALT;
}

//----------------------------------------------------------------------
static void search_fastLog(int start)//,int *high, int *low)
{
int ct = 0;
int findlow = -1,findhigh = -1;
   //find last value
   ct = start;
   while(findlow < 0 && ct > 0)
   {
      ct --;
      if(IO_fastLog.values[ct] > 2.0f || IO_fastLog.values[ct] < -2.0f)
         findlow = ct;
   }
   //find next value
   ct = start;
   while(findhigh < 0 && ct < 200)
   {
      ct ++;
      if(IO_fastLog.values[ct] > 2.0f || IO_fastLog.values[ct] < -2.0f )
         findhigh = ct;
   }

   if(findhigh > 0 && findlow > 0 )
   {
      fillholes(findhigh,findlow);
   }

}

static void fillholes(int high,int low)
{
   float values[2];
   float array_num[2];
   int no_to_fill,ct;

   no_to_fill = high - low;

   if(IO_fastLog.values[low] < IO_fastLog.values[high])
   {
      values[0]   = IO_fastLog.values[low];
      array_num[0] = low * 1.0f;
      values[1] = IO_fastLog.values[high];
      array_num[1] = high * 1.0f;
   }
   else
      values[1]   = IO_fastLog.values[low];
      array_num[1] = low * 1.0f;
      values[0] = IO_fastLog.values[high];
      array_num[0] = high * 1.0f;



   for(ct = 1;ct < no_to_fill + 1;ct ++)
   {
      IO_fastLog.values[low+ct] = table1D(array_num,values,2,(low + ct * 1.0f));
   }
   
   


}

static void set_init()
{
   init_vals.oat          = qtg.temperature;
   init_vals.Lfuel        = qtg.fueltankleft;
   init_vals.Rfuel        = qtg.fueltankright;
   init_vals.ACweight     = qtg.weight;
   init_vals.CofG         = (short)(qtg.cofg * 10.0f);
   init_vals.Lrpm         = qtg.leftrpm;
   init_vals.Lmap         = qtg.leftmap;
   init_vals.Rrpm         = qtg.rightrpm;
   init_vals.Rmap         = qtg.rightmap;
   init_vals.gear         = qtg.gear;
   init_vals.flaps        = qtg.flaps;
   init_vals.Lcowl        = qtg.leftcowlflap;
   init_vals.Rcowl        = qtg.rightcowlflap;
   init_vals.speed        = qtg.airspeed;
   init_vals.alt          = qtg.altitude;
   init_vals.EventTime    = qtg.testeventtime;
   init_vals.TestDuration = qtg.testduration;
}

