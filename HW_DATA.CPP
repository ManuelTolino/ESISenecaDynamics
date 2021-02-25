//HD : hw_data.cpp


//--------------------- 
//This file swops data to hardware computer
//---------------------

//**************************
//   RCS Markers           *
//**************************


//$Header: C:/Master/seneca/RCS/hw_data.cpp 1.6 2000/06/15 15:20:16 ColinJ Exp $
//$Log: hw_data.cpp $
//Revision 1.6  2000/06/15 15:20:16  ColinJ
//changes to AH failure and run up methods.
//Turn co-ordinator scale changed.
//instrument max min times tweaked.
//
//Revision 1.5  2000/05/31 11:05:37  colinj
//suction now depends on Y_D_L/R_ENGINE_LIT.
//Extra case when auxiliary fuel lamps are lit.
//Stall warning now turns a lamp on if config value says so.
//
//Revision 1.4  2000/05/25 08:52:49  colinj
//rate_of_climb depends on SE_freeze_on, pilot_turn depends on elec power,
//changed fault for attitude_ind (different for pilot and copilot),
//changed gyro air lamps, changed FCI_pitch/roll_demand_pos dependency on
//ah_air_pres.
//Added faults for cht running hot, low oil pressure, high oil temp.
//Changed error rate of tacho. Changed gear horn activation.
//Allow for MCB_landing_gear_control_tripped.
//
//Revision 1.3  2000/05/14 10:37:16  colinj
//Suction changes.
//Overboost lamp and door lamp changes.
//
//Revision 1.2  2000/05/11 13:31:02  colinj
//Added support for special move mode, SE_freeze_on.
//
//Revision 1.1  2000/04/14 17:30:13  colinj
//Initial revision
//


//----------------------------------------------------------------------

//----------------
// Include Files
//----------------
#include <stdio.h>
#include <math.h>

#include "dat_stru\out_head.h"
#include "dat_stru\in_head.h"
#include "faults.h"
#include "define.h"
#include "const.h"
#include "fltdy\fmgdefs.h"
#include "fltdy\fms_defs.h"
#include "elec\electric.h"
#include "nav\cpit_def.h"

//----------------
// Defines
//----------------
#define DEBUG_IO            1
#define MAN_FLASH_TIME      0.7f
#define MIN_PRESS         130.0f
#define AH_START_ROLL      45.0f
#define AH_START_PITCH     30.0f
#define HI_OILP            85.0f
#define LO_OILP            25.0f
#define G_HORN_ON_TIMER     (90.0f/120.0f)//0.50f
#define G_HORN_OFF_TIMER    (90.0f/60.0f)//1.0f
#define INST_OVER_PERCENT   4.0f // for high temp failures add 25 % (devide by 4)
#define OIL_TEMP_PERCENT    3.0f 
#define CHT_RISE            7.0f // degress / second rise for cht
#define OIL_T_RISE          5.0f // degress / second rise for cht
#define OIL_P_DROP          5.0f // units / sec (PSI?)
#define AH_RUN_UP          10.0f // time for AH gyro to settle on first run up
#define TACHO_ERROR_RATE  100.0f
#define TURN_CO_ORD_SCALE (33.0f/180.0f)

#define SWITCHABLE 1
//----------------
// Typedefs
//----------------

//--------------------
// External Routines
//--------------------

//----------------
// Externals
//----------------
extern sen_out_struct   *IO_Sen_out;
extern sen_in_struct    *IO_Sen_in;
extern OUTPUTin			*OUTPUTinbox;//simulinkudp
extern sen_fault_struct IO_Sen_faults;
extern OUTPUTS          outputs;
extern EL_out           EL_output;

extern float delta_time;
extern float dip_vac;

extern long int frame_ct;

extern int IOS_simtype;
extern int SE_freeze_on;
extern int RC_alt_stall_warner;
extern char DY_l_start,DY_r_start;
//extern double GeneratorCurrent[];
//----------------
// Globals
//----------------
float ah_air_press = 0.0;
float P1_pitch_error  = 0.0;
float P1_roll_error   = 0.0;
float P2_pitch_error  = 0.0;
float P2_roll_error   = 0.0;
//-----------------
// Local Variables
//-----------------
static float p1_hsi_compass  = 0.0 , p2_hsi_compass  = 0.0;
static float p1_hsi_t_needle = 0.0 , p2_hsi_t_needle = 0.0;
static float p1_hsi_g_needle = 0.0 , p2_hsi_g_needle = 0.0;

static float rmi_head        = 0.0;
static float rmi_adf         = 0.0;
static float rmi_vor         = 0.0;
static float nav2_to_flag    = 0.0;
static float suc_gauge       = 0.0;
static float left_air_press  = 0.0;
static float right_air_press = 0.0;
static float P1_ahUpPitch    = 0.0;
static float P1_ahDnPitch    = 0.0;
static float P1_ahLroll      = 0.0;
static float P1_ahRroll      = 0.0;
static float gear_horn_timer = 0.0;
static float l_oil_temp = 0.0 , r_oil_temp = 0.0;
static float l_oil_pres = 0.0 , r_oil_pres = 0.0;
static float l_cht_temp = 0.0 , r_cht_temp = 0.0;
static float wobble_ah  = 0.0;
static float last_P1_ah_pitch = 0.0;
static float last_P1_ah_roll  = 0.0;
static float pitch_delta      = 0.0;
static float pitch_roc        = 0.0;
static float left_map_disp    = 0.0;
static float left_map_frig    = 0.0;
static float right_map_frig   = 0.0;
float left_map_test    = 0.5f; //was 0.5f orig
float right_map_test = 0.5f;
static char last_p1_hsi = FALSE , last_p2_hsi = FALSE;
static char last_rmi    = FALSE , rmi_fail    = FALSE;
static char last_nav2   = FALSE , nav2_fail   = FALSE;
static char ah_up_right = FALSE;
static char last_ah_up_right = FALSE;
static char last1_dn = SW_OFF;
static char last1_up = SW_OFF;

static char P1_rollSwop  = 5;//FALSE;
static char P1_pitchSwop = 5;//FALSE;

//--------------------
// Forward References
//--------------------

//----------------------------------------------------------------------
void HD_exchange_data(void)
{
   

  //exchange data from Flight model to hardware structure before sending ethernet comms
  //check that relevant instrument has not been failed


//**************************************************************************
//                    pilot instrument failures                          *
//**************************************************************************

      if(IO_Sen_faults.FI.P1_AIRSPEED_IND == TRUE)
      {
         if(IO_Sen_out->pilot_airspeed > 0.0)
            IO_Sen_out->pilot_airspeed -= delta_time;
      }
      else
      {

#ifdef SWITCHABLE
         if(IOS_simtype == WARRIOR)
            IO_Sen_out->pilot_airspeed  = outputs.user[Y_A_P1_ASI]/3.0f;
         else
#endif
            IO_Sen_out->pilot_airspeed  = outputs.user[Y_A_P1_ASI];
      }

      if(IO_Sen_faults.FI.P1_ALTIMETER_1 == TRUE)
      {
         if(IO_Sen_out->pilot_altimeter_1_demand_height > 0.0)
            IO_Sen_out->pilot_altimeter_1_demand_height -= delta_time;
      }
      else
      {
         IO_Sen_out->pilot_altimeter_1_demand_height = outputs.user[Y_A_P1A_ALT];
      }

      if(IO_Sen_faults.FI.P1_ALTIMETER_2 == TRUE)
      {
         if(IO_Sen_out->pilot_altimeter_2_demand_height > 0.0)
            IO_Sen_out->pilot_altimeter_2_demand_height -= delta_time;
      }
      else
      {
         IO_Sen_out->pilot_altimeter_2_demand_height = outputs.user[Y_A_P1B_ALT];
      }
      
      if(IO_Sen_faults.FI.P1_DELTA_CLIMB == TRUE)
      {
         if(IO_Sen_out->pilot_rate_of_climb > 1.0)
            IO_Sen_out->pilot_rate_of_climb -= delta_time;
         else
            IO_Sen_out->pilot_rate_of_climb += delta_time;
         
      }
      else
      {
         if(SE_freeze_on == FALSE)
         {
#ifdef SWITCHABLE
            if(IOS_simtype == WARRIOR)
               IO_Sen_out->pilot_rate_of_climb = outputs.user[Y_A_P1_VSI_GAUGE]/3.0f;
            else
#endif
               IO_Sen_out->pilot_rate_of_climb = outputs.user[Y_A_P1_VSI_GAUGE];
         }
         else
            IO_Sen_out->pilot_rate_of_climb = (float)(outputs.hdot * MS_FTM);
            
      }
      
      if(IO_Sen_faults.FI.P1_TURN_IND == TRUE)
      {
         if(IO_Sen_out->pilot_turn > 1.0)
            IO_Sen_out->pilot_turn -= delta_time;
         else
            IO_Sen_out->pilot_turn += delta_time;
      }
      else
      {
         IO_Sen_out->pilot_turn = (outputs.user[Y_A_TURN_RATE] * EL_output.power * TURN_CO_ORD_SCALE);
      }

      if(IO_Sen_faults.FI.P1_ATITUDE_IND == TRUE)
      {
         P1_pitch_error -= delta_time;
         P1_roll_error  -= delta_time;

      }
      else
      if(ah_air_press < MIN_PRESS)
      {
         P1_pitch_error -= delta_time;
         P1_roll_error  -= delta_time;
         if(P1_pitch_error < -40.0) P1_pitch_error = -40.0;
         if(P1_roll_error < -45.0) P1_roll_error = -45.0;
         if(P1_pitch_error > 40.0) P1_pitch_error = 40.0;
         if(P1_roll_error > 45.0) P1_roll_error = 45.0;
      }
      else
      if(wobble_ah < AH_RUN_UP)
      {
         wobble_ah += delta_time;
         if(IO_Sen_out->pilot_FCI_pitch_demand_position > 3.0)
         {
            if(IO_Sen_in->pilot_FCI_actual_pitch > 4.0)
            {
               P1_pitch_error =  -25.0;
               P1_roll_error  = -25.0;

            }
         }
         else
            if(IO_Sen_in->pilot_FCI_actual_pitch < -4.0)
            {
               P1_pitch_error = 25.0;
               P1_roll_error =  25.0;
            }
      }
      else
 
      {
         if((P1_pitch_error < 0.0) || P1_pitch_error > 0.0)
            P1_pitch_error = 0.0;
         if((P1_roll_error < 0.0) || P1_roll_error > 0.0)
            P1_roll_error = 0.0;
      }
      if(P1_pitch_error < -40.0) P1_pitch_error = -40.0;
      if(P1_roll_error < -90.0) P1_roll_error = -90.0;
      if(P1_pitch_error > 40.0) P1_pitch_error = 40.0;
      if(P1_roll_error > 90.0) P1_roll_error = 90.0;

      IO_Sen_out->pilot_FCI_pitch_demand_position = (float)( (outputs.theta*R2D) + P1_pitch_error);
      IO_Sen_out->pilot_FCI_roll_demand_position  = (float)( (outputs.phi*R2D)   + P1_roll_error);

      if(IO_Sen_faults.FI.P1_ATITUDE_IND == TRUE)
      {
         //last_P1_ah_pitch
         //last_P1_ah_roll
         pitch_roc = (1.0f/((P1_pitch_error*-1.0f) + 1.0f));
         pitch_delta = (last_P1_ah_pitch - IO_Sen_out->pilot_FCI_pitch_demand_position);
         if(fabs(pitch_delta) > pitch_roc)
         {
            if(IO_Sen_out->pilot_FCI_pitch_demand_position > last_P1_ah_pitch)
            {
               IO_Sen_out->pilot_FCI_pitch_demand_position = (last_P1_ah_pitch + (pitch_roc * delta_time));
            }
            else
               IO_Sen_out->pilot_FCI_pitch_demand_position = (last_P1_ah_pitch - (pitch_roc * delta_time));
         }



      }

last_P1_ah_pitch = IO_Sen_out->pilot_FCI_pitch_demand_position;

      if(IO_Sen_faults.FI.P1_HORZ_POS_IND == TRUE && last_p1_hsi == FALSE)
      {
         p1_hsi_compass  = IO_Sen_out->pilot_hsi_compass_demand_position;         
         p1_hsi_t_needle = IO_Sen_out->pilot_hsi_to_from_needle;
         p1_hsi_g_needle = IO_Sen_out->pilot_hsi_GS_needle;
      }

      last_p1_hsi = IO_Sen_faults.FI.P1_HORZ_POS_IND;

      if(IO_Sen_faults.FI.P1_HORZ_POS_IND == TRUE)
      {
         IO_Sen_out->pilot_hsi_compass_demand_position = p1_hsi_compass;
         IO_Sen_out->pilot_hsi_left_right_needle       = CENTRE_NEEDLE;
         IO_Sen_out->pilot_hsi_to_from_needle          = p1_hsi_t_needle;
         IO_Sen_out->pilot_hsi_GS_needle               = p1_hsi_g_needle;
         IO_Sen_out->pilot_hsi_nav_flag                = SHOW_FLAG;
         IO_Sen_out->pilot_hsi_hdg_flag                = SHOW_FLAG;
         IO_Sen_out->pilot_hsi_GS_enable               = DISABLE;
      }

      if(IO_Sen_faults.FI.rmi_fail == TRUE && last_rmi == FALSE)
      {
         rmi_head = IO_Sen_out->RMI_demand_heading;
         rmi_adf  = IO_Sen_out->RMI_ADF_needle;
         rmi_vor  = IO_Sen_out->RMI_VOR_needle;
      }

      last_rmi = IO_Sen_faults.FI.rmi_fail;

      if(IO_Sen_faults.FI.rmi_fail == TRUE)
      {
         IO_Sen_out->RMI_demand_heading = rmi_head;
         IO_Sen_out->RMI_ADF_needle     = rmi_adf;
         IO_Sen_out->RMI_VOR_needle     = rmi_vor;
      }

      if(IO_Sen_faults.FI.nav2_fail == TRUE && last_nav2 == FALSE)
      {
         nav2_to_flag = IO_Sen_out->nav2_to_from_flag;
      }

      last_nav2 = IO_Sen_faults.FI.nav2_fail;

      if(IO_Sen_faults.FI.nav2_fail == TRUE)
      {
         IO_Sen_out->nav2_GS_needle         = CENTRE_NEEDLE;
         IO_Sen_out->nav2_left_right_needle = CENTRE_NEEDLE;
         IO_Sen_out->nav2_to_from_flag      = nav2_to_flag;
         IO_Sen_out->nav2_GS_flag           = DISABLE;
         IO_Sen_out->nav2_nav_flag          = DISABLE;
      }


//**************************************************************************
//                    copilot instrument failures                          *
//**************************************************************************
      if(IO_Sen_faults.FI.P2_AIRSPEED_IND == TRUE)
      {
         if(IO_Sen_out->copilot_airspeed > 0.0)
            IO_Sen_out->copilot_airspeed -= delta_time;
      }
      else
      {
         IO_Sen_out->copilot_airspeed  = outputs.user[Y_A_P2_ASI];
      }

      if(IO_Sen_faults.FI.P2_ALTIMETER_1 == TRUE)
      {
         if(IO_Sen_out->copilot_altimeter_demand_height > 0.0)
            IO_Sen_out->copilot_altimeter_demand_height -= delta_time;
      }
      else
      {
         IO_Sen_out->copilot_altimeter_demand_height = outputs.user[Y_A_P2_ALT];
      }

      
      if(IO_Sen_faults.FI.P2_DELTA_CLIMB == TRUE)
      {
         if(IO_Sen_out->copilot_rate_of_climb > 1.0)
            IO_Sen_out->copilot_rate_of_climb -= delta_time;
         else
            IO_Sen_out->copilot_rate_of_climb += delta_time;
         
      }
      else
      {
         if(SE_freeze_on == FALSE)
         {
            IO_Sen_out->copilot_rate_of_climb = outputs.user[Y_A_P2_VSI_GAUGE];
         }
         else
            IO_Sen_out->copilot_rate_of_climb = (float)(outputs.hdot * MS_FTM);
         
      }
      
      if(IO_Sen_faults.FI.P2_TURN_IND == TRUE)
      {
         if(IO_Sen_out->copilot_turn > 1.0)
            IO_Sen_out->copilot_turn -= delta_time;
         else
            IO_Sen_out->copilot_turn += delta_time;
      }
      else
      {
         IO_Sen_out->copilot_turn = (outputs.user[Y_A_TURN_RATE] * EL_output.power * TURN_CO_ORD_SCALE);
      }
//copilot attitude indicator

      if(IO_Sen_faults.FI.P2_ATITUDE_IND == TRUE)
      {
         P2_pitch_error -= delta_time;
         P2_roll_error  -= delta_time;

      }
      else
      if(ah_air_press < MIN_PRESS)
      {
         P2_pitch_error -= delta_time;
         P2_roll_error  -= delta_time;
         if(P2_pitch_error < -40.0) P2_pitch_error = -40.0;
         if(P2_roll_error  < -45.0) P2_roll_error  = -45.0;
         if(P2_pitch_error >  40.0) P2_pitch_error =  40.0;
         if(P2_roll_error  >  45.0) P2_roll_error  =  45.0;
      }
      else
      if(wobble_ah < AH_RUN_UP)
      {
         wobble_ah += delta_time;
         if(IO_Sen_out->copilot_FCI_pitch_demand_position > 3.0)
         {
            if(IO_Sen_in->copilot_FCI_actual_pitch > 4.0)
            {
               P2_pitch_error = -25.0;
               P2_roll_error  =  25.0;

            }
         }
         else
            if(IO_Sen_in->copilot_FCI_actual_pitch < -4.0)
            {
               P2_pitch_error =  25.0;
               P2_roll_error  = -25.0;
            }
      }
      else
      {
         if((P2_pitch_error < 0.0) || P2_pitch_error > 0.0)
            P2_pitch_error = 0.0;
         if((P2_roll_error < 0.0) || P2_roll_error > 0.0)
            P2_roll_error = 0.0;
      }

      if(P2_pitch_error < -40.0) P2_pitch_error = -40.0;
      if(P2_roll_error  < -90.0) P2_roll_error  = -90.0;
      if(P2_pitch_error >  40.0) P2_pitch_error =  40.0;
      if(P2_roll_error  >  90.0) P2_roll_error  =  90.0;

      IO_Sen_out->copilot_FCI_pitch_demand_position =(float)( (outputs.theta * R2D) + P2_pitch_error);
      IO_Sen_out->copilot_FCI_roll_demand_position  =(float)( (outputs.phi * R2D)   + P2_roll_error);

      if(IO_Sen_faults.FI.P2_HORZ_POS_IND == TRUE && last_p2_hsi == FALSE)
      {
         p2_hsi_compass  = IO_Sen_out->copilot_hsi_compass_demand_position;         
         p2_hsi_t_needle = IO_Sen_out->copilot_hsi_to_from_needle;
         p2_hsi_g_needle = IO_Sen_out->copilot_hsi_GS_needle;
      }

      last_p2_hsi = IO_Sen_faults.FI.P2_HORZ_POS_IND;

      if(IO_Sen_faults.FI.P2_HORZ_POS_IND == TRUE)
      {
         IO_Sen_out->copilot_hsi_compass_demand_position = p2_hsi_compass;
         IO_Sen_out->copilot_hsi_left_right_needle       = CENTRE_NEEDLE;
         IO_Sen_out->copilot_hsi_to_from_needle          = p2_hsi_t_needle;
         IO_Sen_out->copilot_hsi_GS_needle               = p2_hsi_g_needle;
         IO_Sen_out->copilot_hsi_nav_flag                = SHOW_FLAG;
         IO_Sen_out->copilot_hsi_hdg_flag                = SHOW_FLAG;
         IO_Sen_out->copilot_hsi_GS_enable               = DISABLE;
      }




      IO_Sen_out->copilot_slip = (float)(outputs.user[Y_A_SLIP_ANGLE] * -1000.0f);//-1000=Org -500=Woody mod
      IO_Sen_out->pilot_slip   = (float)(outputs.user[Y_A_SLIP_ANGLE] * -1000.0f);//-1000=Org -500=Woody mod
      





      if(outputs.user[Y_A_L_RPM] < 500.0 || IO_Sen_faults.FI.Left_SUCTION_PUMP)
      {
         IO_Sen_out->panel_annunciator_left_gyro_air_lamp  = LIT;
      }
      else
      {
            IO_Sen_out->panel_annunciator_left_gyro_air_lamp  = UNLIT;
      }

         

      if(outputs.user[Y_A_R_RPM] < 500.0 || IO_Sen_faults.FI.Right_SUCTION_PUMP)
      {
         IO_Sen_out->panel_annunciator_right_gyro_air_lamp  = LIT;
      }
      else
      {
         IO_Sen_out->panel_annunciator_right_gyro_air_lamp  = UNLIT;
      }


      if(outputs.user[Y_A_VACUUM] < 4.5)// && (outputs.duser[Y_D_R_ENGINE_LIT] == TRUE || outputs.duser[Y_D_L_ENGINE_LIT] == TRUE))
      {//This will be based on suction when included in flight model
         //engines not producing suction so slow down gyro slowly
         if(ah_air_press > 0.0)
         {
            ah_air_press -= (float)(delta_time/3.0);
            if(suc_gauge  > 0.0)
            suc_gauge    -= (float)(delta_time/2.0);
         }
         ah_air_press = 0.0;
      }
      else
         // engines producing suction so spin gyros up
         if(ah_air_press < (MIN_PRESS + 10.0))
         {
            if (outputs.duser[Y_D_R_ENGINE_LIT] == TRUE || outputs.duser[Y_D_L_ENGINE_LIT] == TRUE)
            {
               ah_air_press = MIN_PRESS + 11.0;
               wobble_ah = 0.0;
               if(suc_gauge  < 5.3)
               suc_gauge    += (float)(delta_time/2.0);
            }
         }

//**************************************************************************      
//engine instrument failures can be either max value min value or stuck    *
//if set to max or min increase or decrease is on delta time at the moment *
//may need scaleing in future                                              *
//**************************************************************************

      //test scaleing
      if(IO_Sen_in->test_switch_1_up == SW_ON && last1_up == SW_OFF)
      {
         //left_map_test = 0.5f;
         //right_map_test = 0.5f;
      }
      else
      if(IO_Sen_in->test_switch_1_dn == SW_ON && last1_dn == SW_OFF)
      {
         //left_map_test = 0.0f;
         //right_map_test = 0.0f;
      }

//      printf("\n switch1UP %d switch1dn %d",IO_Sen_in->test_switch_2_up,IO_Sen_in->test_switch_2_dn);
      last1_dn = IO_Sen_in->test_switch_1_dn;
      last1_up = IO_Sen_in->test_switch_1_up;


	  if(IO_Sen_faults.EI.LEFT_MANIFOLD != INST_FINE)
	  {
		  switch(IO_Sen_faults.EI.LEFT_MANIFOLD)
		  {
		  case INST_MAX  :      //instrument should display max value
			  if(IO_Sen_out->left_manifold_pressure < MAX_MANIFOLD)
			  {
				  IO_Sen_out->left_manifold_pressure += delta_time;
			  }
			  else
				  IO_Sen_out->left_manifold_pressure = MAX_MANIFOLD;
			  break;

		  case INST_MIN :
			  if(IO_Sen_out->left_manifold_pressure > 0.0)
			  {
				  IO_Sen_out->left_manifold_pressure -= delta_time;
			  }
			  else
				  IO_Sen_out->left_manifold_pressure = 0.0;
			  break;

		  default : IO_Sen_out->left_manifold_pressure = IO_Sen_out->left_manifold_pressure;
			  break;
		  }
	  }
	  else
	  {
		  if(outputs.user[Y_A_L_MAN_P] > 0.0)
		  {
			  left_map_frig = (float)((30.0 - outputs.user[Y_A_L_MAN_P]) * left_map_test);

		     IO_Sen_out->left_manifold_pressure = (float) (outputs.user[Y_A_L_MAN_P] + left_map_frig);
		  }
		  
		  IO_Sen_out->left_manifold_pressure = (float) (outputs.user[Y_A_L_MAN_P]);
	  }

//************************************************************************************
//     simulate cht running hot so calulate a percentage of actual temp and add this *
//      to the actual temp.                                                          *
//************************************************************************************

     if(IO_Sen_faults.FF.l_cht_high == TRUE)
     {
        if(l_cht_temp < (outputs.user[Y_A_L_CHT] / INST_OVER_PERCENT))
           l_cht_temp += (float)(delta_time * CHT_RISE);
        else
        if(l_cht_temp > (outputs.user[Y_A_L_CHT] / INST_OVER_PERCENT))
           l_cht_temp -= (float)(delta_time * CHT_RISE);
     }
     else
     {
        if(l_cht_temp >  0.0)
           l_cht_temp -= (float)(delta_time * CHT_RISE);
     }


     if(IO_Sen_faults.EI.LEFT_CYL_TEMP_IND != INST_FINE)
     {
        switch(IO_Sen_faults.EI.LEFT_CYL_TEMP_IND)
        {
        case INST_MAX :
           if(IO_Sen_out->left_cyclic_temperature < MAX_CYL_TEMP)
           {
              IO_Sen_out->left_cyclic_temperature += (delta_time * 5.0f);
           }
           else
              IO_Sen_out->left_cyclic_temperature = MAX_CYL_TEMP;
           break;

        case INST_MIN :
           if(IO_Sen_out->left_cyclic_temperature > 0.0)
           {
              IO_Sen_out->left_cyclic_temperature -= (delta_time * 5.0f);
           }
           else
              IO_Sen_out->left_cyclic_temperature = 0.0;
           break;

        default :
           IO_Sen_out->left_cyclic_temperature = IO_Sen_out->left_cyclic_temperature;
        }
     }
     else
        IO_Sen_out->left_cyclic_temperature = (outputs.user[Y_A_L_CHT] + l_cht_temp); 


     if(IO_Sen_faults.EI.LEFT_EGT_IND != INST_FINE)
     {
        switch(IO_Sen_faults.EI.LEFT_EGT_IND)
        {
        case INST_MAX :           
           if(IO_Sen_out->left_EGT < MAX_EGT)
           {              
              IO_Sen_out->left_EGT += (delta_time * 5.0f);
           }
           else
              IO_Sen_out->left_EGT = MAX_EGT;
           break;

        case INST_MIN :
           if(IO_Sen_out->left_EGT > 0.0)
           {
              IO_Sen_out->left_EGT -= (delta_time * 5.0f);
           }
           else
              IO_Sen_out->left_EGT = 0.0;
           break;

        default:
           IO_Sen_out->left_EGT = IO_Sen_out->left_EGT;
        }
     }
     else
     {
        IO_Sen_out->left_EGT = outputs.user[Y_A_L_EGT]; 
     }

     if(IO_Sen_faults.EI.LEFT_FUEL_CONT_IND != INST_FINE)
     {
        switch(IO_Sen_faults.EI.LEFT_FUEL_CONT_IND)
        {
        case INST_MAX :
           if(IO_Sen_out->left_fuel_quantity < MAX_FUEL_CONT)
           {
              IO_Sen_out->left_fuel_quantity += delta_time;
           }
           else
              IO_Sen_out->left_fuel_quantity = MAX_FUEL_CONT;
           break;

        case INST_MIN :
           if(IO_Sen_out->left_fuel_quantity > 0.0)
           {
              IO_Sen_out->left_fuel_quantity -= delta_time;
           }
           else
              IO_Sen_out->left_fuel_quantity = 0.0;
           break;

        default:
           IO_Sen_out->left_fuel_quantity = IO_Sen_out->left_fuel_quantity;
           break;
        }
     }
     else
        IO_Sen_out->left_fuel_quantity = outputs.user[Y_A_L_FUEL];

     if(IO_Sen_faults.EI.LEFT_FUEL_FLOW_IND != INST_FINE)
     {
        switch(IO_Sen_faults.EI.LEFT_FUEL_FLOW_IND)
        {
        case INST_MAX :
           if(IO_Sen_out->left_fuel_flow < MAX_FUEL_FLOW)
           {
              IO_Sen_out->left_fuel_flow += delta_time;
           }
           else
              IO_Sen_out->left_fuel_flow = MAX_FUEL_FLOW;
           break;

        case INST_MIN :
           if(IO_Sen_out->left_fuel_flow > 0.0)
           {
              IO_Sen_out->left_fuel_flow -= delta_time;
           }
           else
              IO_Sen_out->left_fuel_flow = 0.0;
           break;

        default :
           IO_Sen_out->left_fuel_flow = IO_Sen_out->left_fuel_flow;
           break;
        }
     }
     else
        IO_Sen_out->left_fuel_flow = outputs.user[Y_A_L_FF];

     if(IO_Sen_faults.EI.LEFT_RPM_IND != INST_FINE)
     {
        switch(IO_Sen_faults.EI.LEFT_RPM_IND)
        {
        case INST_MAX :
           if(IO_Sen_out->left_tacho < MAX_RPM)
           {
              IO_Sen_out->left_tacho += (float)(delta_time * TACHO_ERROR_RATE);
           }
           else
              IO_Sen_out->left_tacho = MAX_RPM;
           break;

        case INST_MIN :
           if(IO_Sen_out->left_tacho > 0.0)
           {
              IO_Sen_out->left_tacho -= (float)(delta_time * TACHO_ERROR_RATE);
           }
           else
              IO_Sen_out->left_tacho = 0.0;
           break;

        default :
           IO_Sen_out->left_tacho = IO_Sen_out->left_tacho;
           break;
        }
     }
     else
     {
#ifdef SWITCHABLE
         if(IOS_simtype == WARRIOR)
            IO_Sen_out->left_tacho = outputs.user[Y_A_L_RPM]/2.0f;
         else
#endif
            IO_Sen_out->left_tacho = outputs.user[Y_A_L_RPM];

     }

//************************************************************************************
//     simulate low oil pressure so calulate a percentage of actual pressure         *
//     and subtract this  from the actual pressure.                                  *
//************************************************************************************

     if(IO_Sen_faults.FF.l_oil_press == TRUE)
     {
        if(IO_Sen_out->left_oil_pressure > 25.0)
           l_oil_pres += (float)(delta_time * OIL_P_DROP);
     }
     else
     {
        if(l_oil_pres >  0.0)
           l_oil_pres -= (float)(delta_time * OIL_P_DROP);
     }

     if(IO_Sen_faults.EI.LEFT_OIL_PRESS_IND != INST_FINE)
     {
        switch(IO_Sen_faults.EI.LEFT_OIL_PRESS_IND)
        {
        case INST_MAX :
           if(IO_Sen_out->left_oil_pressure < MAX_OIL_PRESS)
           {
              IO_Sen_out->left_oil_pressure += delta_time;
           }
           else
              IO_Sen_out->left_oil_pressure = MAX_OIL_PRESS;
           break;

        case INST_MIN:
           if(IO_Sen_out->left_oil_pressure > 0.0)
           {
              IO_Sen_out->left_oil_pressure -= delta_time;
           }
           else
              IO_Sen_out->left_oil_pressure = 0.0;
           break;

        default :
           IO_Sen_out->left_oil_pressure = IO_Sen_out->left_oil_pressure;
           break;
        }
     }
     else
        IO_Sen_out->left_oil_pressure = (outputs.user[Y_A_L_OIL_P] - l_oil_pres);

//************************************************************************************
//     simulate high oil temp so calulate a percentage of actual temp and add this   *
//      to the actual temp.                                                          *
//************************************************************************************
     
     if(IO_Sen_faults.FF.l_oil_temp == TRUE)
     {
        if(l_oil_temp < (outputs.user[Y_A_L_OIL_T] / OIL_TEMP_PERCENT))
           l_oil_temp += (float)(delta_time * OIL_T_RISE);
        else
        if(l_oil_temp > (outputs.user[Y_A_L_OIL_T] / OIL_TEMP_PERCENT))
           l_oil_temp -= (float)(delta_time * OIL_T_RISE);
     }
     else
     {
        if(l_oil_temp >  0.0)
           l_oil_temp -= (float)(delta_time * OIL_T_RISE);
     }


     if(IO_Sen_faults.EI.LEFT_OIL_TEMP_IND != INST_FINE)
     {
        switch(IO_Sen_faults.EI.LEFT_OIL_TEMP_IND)
        {
        case INST_MAX :
           if(IO_Sen_out->left_oil_temperature < MAX_OIL_TEMP)
           {
              IO_Sen_out->left_oil_temperature += delta_time;
           }
           else
              IO_Sen_out->left_oil_temperature = MAX_OIL_TEMP;
           break;

        case INST_MIN :
           if(IO_Sen_out->left_oil_temperature > 0.0)
           {
              IO_Sen_out->left_oil_temperature -= delta_time;
           }
           else
              IO_Sen_out->left_oil_temperature = 0.0;
           break;

        default :
           IO_Sen_out->left_oil_temperature = IO_Sen_out->left_oil_temperature;
           break;
        }
     }
     else
        IO_Sen_out->left_oil_temperature = (outputs.user[Y_A_L_OIL_T] + l_oil_temp);

//**************************************************************
//                     right engine failures                   *
//**************************************************************

	  if(IO_Sen_faults.EI.RIGHT_MANIFOLD != INST_FINE)
	  {
		  switch(IO_Sen_faults.EI.RIGHT_MANIFOLD)
		  {
		  case INST_MAX  :      //instrument should display max value
			  if(IO_Sen_out->right_manifold_pressure < MAX_MANIFOLD)
			  {
				  IO_Sen_out->right_manifold_pressure += delta_time;
			  }
			  else
				  IO_Sen_out->right_manifold_pressure = MAX_MANIFOLD;
			  break;

		  case INST_MIN :
			  if(IO_Sen_out->right_manifold_pressure > 0.0)
			  {
				  IO_Sen_out->right_manifold_pressure -= delta_time;
			  }
			  else
				  IO_Sen_out->right_manifold_pressure = 0.0;
			  break;

		  default : IO_Sen_out->right_manifold_pressure = IO_Sen_out->right_manifold_pressure;
			  break;
		  }
	  }
	  else
	  {
		  if(outputs.user[Y_A_R_MAN_P] > 0.0)
		  {
			  right_map_frig = (float)((30.0 - outputs.user[Y_A_R_MAN_P]) * right_map_test);

		     IO_Sen_out->right_manifold_pressure = (float) (outputs.user[Y_A_R_MAN_P] + right_map_frig);
		  }
		  //else
		  IO_Sen_out->right_manifold_pressure = (float) (outputs.user[Y_A_R_MAN_P]);
	  }

//************************************************************************************
//     simulate cht running hot so calulate a percentage of actual temp and add this *
//      to the actual temp.                                                          *
//************************************************************************************

     if(IO_Sen_faults.FF.r_cht_high == TRUE)
     {
        if(r_cht_temp < (outputs.user[Y_A_R_CHT] / INST_OVER_PERCENT))
           r_cht_temp += (float)(delta_time * CHT_RISE);
        else
        if(r_cht_temp > (outputs.user[Y_A_R_CHT] / INST_OVER_PERCENT))
           r_cht_temp -= (float)(delta_time * CHT_RISE);
     }
     else
     {
        if(r_cht_temp >  0.0)
           r_cht_temp -= (float)(delta_time * CHT_RISE);
     }


     if(IO_Sen_faults.EI.RIGHT_CYL_TEMP_IND != INST_FINE)
     {
        switch(IO_Sen_faults.EI.RIGHT_CYL_TEMP_IND)
        {
        case INST_MAX :
           if(IO_Sen_out->right_cyclic_temperature < MAX_CYL_TEMP)
           {
              IO_Sen_out->right_cyclic_temperature += (delta_time * 5.0f);
           }
           else
              IO_Sen_out->right_cyclic_temperature = MAX_CYL_TEMP;
           break;

        case INST_MIN :
           if(IO_Sen_out->right_cyclic_temperature > 0.0)
           {
              IO_Sen_out->right_cyclic_temperature -= (delta_time * 5.0f);
           }
           else
              IO_Sen_out->right_cyclic_temperature = 0.0;
           break;

        default :
           IO_Sen_out->right_cyclic_temperature = IO_Sen_out->right_cyclic_temperature;
        }
     }
     else
        IO_Sen_out->right_cyclic_temperature = (outputs.user[Y_A_R_CHT] + r_cht_temp); 

     if(IO_Sen_faults.EI.RIGHT_EGT_IND != INST_FINE)
     {
        switch(IO_Sen_faults.EI.RIGHT_EGT_IND)
        {
        case INST_MAX :
           if(IO_Sen_out->right_EGT < MAX_EGT)
           {              
              IO_Sen_out->right_EGT += (delta_time * 5.0f);
           }
           else
              IO_Sen_out->right_EGT = MAX_EGT;
           break;

        case INST_MIN :
           if(IO_Sen_out->right_EGT > 0.0)
           {
              IO_Sen_out->right_EGT -= (delta_time * 5.0f);
           }
           else
              IO_Sen_out->right_EGT = 0.0;
           break;

        default:

           IO_Sen_out->right_EGT = IO_Sen_out->right_EGT;
        }
     }
     else
     {        
        IO_Sen_out->right_EGT = outputs.user[Y_A_R_EGT]; 
     }

     if(IO_Sen_faults.EI.RIGHT_FUEL_CONT_IND != INST_FINE)
     {
        switch(IO_Sen_faults.EI.RIGHT_FUEL_CONT_IND)
        {
        case INST_MAX :
           if(IO_Sen_out->right_fuel_quantity < MAX_FUEL_CONT)
           {
              IO_Sen_out->right_fuel_quantity += delta_time;
           }
           else
              IO_Sen_out->right_fuel_quantity = MAX_FUEL_CONT;
           break;

        case INST_MIN :
           if(IO_Sen_out->right_fuel_quantity > 0.0)
           {
              IO_Sen_out->right_fuel_quantity -= delta_time;
           }
           else
              IO_Sen_out->right_fuel_quantity = 0.0;
           break;

        default:
           IO_Sen_out->right_fuel_quantity = IO_Sen_out->right_fuel_quantity;
           break;
        }
     }
     else
        IO_Sen_out->right_fuel_quantity = outputs.user[Y_A_R_FUEL];

     if(IO_Sen_faults.EI.RIGHT_FUEL_FLOW_IND != INST_FINE)
     {
        switch(IO_Sen_faults.EI.RIGHT_FUEL_FLOW_IND)
        {
        case INST_MAX :
           if(IO_Sen_out->right_fuel_flow < MAX_FUEL_FLOW)
           {
              IO_Sen_out->right_fuel_flow += delta_time;
           }
           else
              IO_Sen_out->right_fuel_flow = MAX_FUEL_FLOW;
           break;

        case INST_MIN :
           if(IO_Sen_out->right_fuel_flow > 0.0)
           {
              IO_Sen_out->right_fuel_flow -= delta_time;
           }
           else
              IO_Sen_out->right_fuel_flow = 0.0;
           break;

        default :
           IO_Sen_out->right_fuel_flow = IO_Sen_out->right_fuel_flow;
           break;
        }
     }
     else
        IO_Sen_out->right_fuel_flow = outputs.user[Y_A_R_FF];

     if(IO_Sen_faults.EI.RIGHT_RPM_IND != INST_FINE)
     {
        switch(IO_Sen_faults.EI.RIGHT_RPM_IND)
        {
        case INST_MAX :
           if(IO_Sen_out->right_tacho < MAX_RPM)
           {
              IO_Sen_out->right_tacho += (float)(delta_time * TACHO_ERROR_RATE);
           }
           else
              IO_Sen_out->right_tacho = MAX_RPM;
           break;

        case INST_MIN :
           if(IO_Sen_out->right_tacho > 0.0)
           {
              IO_Sen_out->right_tacho -= (float)(delta_time * TACHO_ERROR_RATE);
           }
           else
              IO_Sen_out->right_tacho = 0.0;
           break;

        default :
           IO_Sen_out->right_tacho = IO_Sen_out->right_tacho;
           break;
        }
     }
     else
     {
#ifdef SWITCHABLE
         if(IOS_simtype == WARRIOR)
            IO_Sen_out->right_tacho = outputs.user[Y_A_R_RPM]/2.0f;
         else
#endif
            IO_Sen_out->right_tacho = outputs.user[Y_A_R_RPM];
     }

//************************************************************************************
//     simulate low oil pressure so calulate a percentage of actual pressure         *
//     and subtract this  from the actual pressure.                                  *
//************************************************************************************

     if(IO_Sen_faults.FF.r_oil_press == TRUE)
     {
        if(IO_Sen_out->right_oil_pressure > 25.0)
           r_oil_pres += (float)(delta_time * OIL_P_DROP);
     }
     else
     {
        if(r_oil_pres >  0.0)
           r_oil_pres -= (float)(delta_time * OIL_P_DROP);
     }

     if(IO_Sen_faults.EI.RIGHT_OIL_PRESS_IND != INST_FINE)
     {
        switch(IO_Sen_faults.EI.RIGHT_OIL_PRESS_IND)
        {
        case INST_MAX :
           if(IO_Sen_out->right_oil_pressure < MAX_OIL_PRESS)
           {
              IO_Sen_out->right_oil_pressure += delta_time;
           }
           else
              IO_Sen_out->right_oil_pressure = MAX_OIL_PRESS;
           break;

        case INST_MIN:
           if(IO_Sen_out->right_oil_pressure > 0.0)
           {
              IO_Sen_out->right_oil_pressure -= delta_time;
           }
           else
              IO_Sen_out->right_oil_pressure = 0.0;
           break;

        default :
           IO_Sen_out->right_oil_pressure = IO_Sen_out->right_oil_pressure;
           break;
        }
     }
     else
        IO_Sen_out->right_oil_pressure = (outputs.user[Y_A_R_OIL_P] - r_oil_pres);

//************************************************************************************
//     simulate high oil temp so calulate a percentage of actual temp and add this   *
//      to the actual temp.                                                          *
//************************************************************************************

     if(IO_Sen_faults.FF.r_oil_temp == TRUE)
     {
        if(r_oil_temp < (outputs.user[Y_A_R_OIL_T] / OIL_TEMP_PERCENT))
           r_oil_temp += (float)(delta_time * OIL_T_RISE);
        else
        if(r_oil_temp > (outputs.user[Y_A_R_OIL_T] / OIL_TEMP_PERCENT))
           r_oil_temp -= (float)(delta_time * OIL_T_RISE);
     }
     else
     {
        if(r_oil_temp >  0.0)
           r_oil_temp -= (float)(delta_time * OIL_T_RISE);
     }
     
     if(IO_Sen_faults.EI.RIGHT_OIL_TEMP_IND != INST_FINE)
     {
        switch(IO_Sen_faults.EI.RIGHT_OIL_TEMP_IND)
        {
        case INST_MAX :
           if(IO_Sen_out->right_oil_temperature < MAX_OIL_TEMP)
           {
              IO_Sen_out->right_oil_temperature += delta_time;
           }
           else
              IO_Sen_out->right_oil_temperature = MAX_OIL_TEMP;
           break;

        case INST_MIN :
           if(IO_Sen_out->right_oil_temperature > 0.0)
           {
              IO_Sen_out->right_oil_temperature -= delta_time;
           }
           else
              IO_Sen_out->right_oil_temperature = 0.0;
           break;

        default :
           IO_Sen_out->right_oil_temperature = IO_Sen_out->right_oil_temperature;
           break;
        }
     }
     else
        IO_Sen_out->right_oil_temperature = (outputs.user[Y_A_R_OIL_T] + r_oil_temp);
           

     if(IO_Sen_faults.AS.suction != INST_FINE)
     {
        switch(IO_Sen_faults.AS.suction)
        {
        case INST_MAX :
           if(IO_Sen_out->suction < MAX_SUCTION)
           {
              IO_Sen_out->suction += (float)(delta_time / 10.0);
           }
           break;

        case INST_MIN:
           if(IO_Sen_out->suction > MIN_SUCTION)
           {
              IO_Sen_out->suction -= (float)(delta_time / 10.0);
           }
           break;

        default :
           IO_Sen_out->suction = IO_Sen_out->suction;
           break;
        }
     }
     else
        //IO_Sen_out->suction = (float) (suc_gauge - dip_vac);//(float)(outputs.user[Y_A_VACUUM] - dip_vac);     
        IO_Sen_out->suction = (float)(outputs.user[Y_A_VACUUM] - dip_vac);


     if(IO_Sen_faults.AS.ammeter != INST_FINE)
     {
        switch(IO_Sen_faults.AS.ammeter)
        {
        case INST_MAX :
           if(IO_Sen_out->ammeter < MAX_AMMETER)
           {
              IO_Sen_out->ammeter += (float)(delta_time * 8.0);
           }
           break;

        case INST_MIN:
           if(IO_Sen_out->ammeter > MIN_AMMETER)
           {
              IO_Sen_out->ammeter -= (float)(delta_time * 8.0);
           }
           break;

        default :
           IO_Sen_out->ammeter = IO_Sen_out->ammeter;
           break;
        }
     }
     else
        IO_Sen_out->ammeter =(float) EL_output.current;//  charge;

     
     
//**************************************************************
//                     general failures                        *
//**************************************************************


     IO_Sen_out->MCB_alt_field_left_trigger        = (IO_Sen_faults.EF.l_alt_mcb || EL_output.left_pop);
     IO_Sen_out->MCB_alt_field_right_trigger       = (IO_Sen_faults.EF.r_alt_mcb || EL_output.right_pop);
     IO_Sen_out->MCB_electric_flaps_trigger        = IO_Sen_faults.EF.electric_flaps_mcb;
     IO_Sen_out->MCB_landing_gear_control_trigger  = IO_Sen_faults.EF.landing_gear_control_mcb;
     IO_Sen_out->MCB_landing_gear_pump_trigger     = IO_Sen_faults.EF.landing_gear_pump_mcb;
     IO_Sen_out->MCB_pitch_trim_trigger            = IO_Sen_faults.EF.pitch_trim_mcb;
     
     


      if(EL_output.power == TRUE)
      {
         
         if(IO_Sen_in->elec_left_fuel_switch == FUEL_HI || (/*outputs.duser[Y_D_L_ENGINE_LIT] == TRUE &&  */IO_Sen_in->elec_left_primer_button == SW_ON) 
            || (outputs.duser[Y_D_L_ENGINE_LIT] == FALSE && IO_Sen_in->elec_left_fuel_switch == FUEL_LO))
            IO_Sen_out->panel_annunciator_left_auxiliary_fuel_lamp  = LIT;
         else
            IO_Sen_out->panel_annunciator_left_auxiliary_fuel_lamp  = UNLIT;

         if(IO_Sen_in->elec_right_fuel_switch == FUEL_HI || (/*outputs.duser[Y_D_R_ENGINE_LIT] == TRUE &&  */IO_Sen_in->elec_right_primer_button == SW_ON) 
            || (outputs.duser[Y_D_R_ENGINE_LIT] == FALSE && IO_Sen_in->elec_right_fuel_switch == FUEL_LO))
            IO_Sen_out->panel_annunciator_right_auxiliary_fuel_lamp  = LIT;
         else
            IO_Sen_out->panel_annunciator_right_auxiliary_fuel_lamp  = UNLIT;

         IO_Sen_out->panel_landing_gear_nose_lamp       = (unsigned char) outputs.duser[Y_D_N_GEAR];  
         IO_Sen_out->panel_landing_gear_left_lamp       = (unsigned char) outputs.duser[Y_D_L_GEAR];
         IO_Sen_out->panel_landing_gear_right_lamp      = (unsigned char) outputs.duser[Y_D_R_GEAR];

         if(((outputs.user[Y_A_R_OIL_P] - r_oil_pres) < LO_OILP) || (outputs.user[Y_A_R_OIL_P] > HI_OILP)
            || ((outputs.user[Y_A_L_OIL_P] - l_oil_pres) < LO_OILP) || (outputs.user[Y_A_L_OIL_P] > HI_OILP))
         {
            IO_Sen_out->panel_annunciator_oil_press_lamp = LIT;
         }
         else
            IO_Sen_out->panel_annunciator_oil_press_lamp = UNLIT;


         if(outputs.duser[Y_D_N_CONTACT] || outputs.duser[Y_D_L_CONTACT] || outputs.duser[Y_D_R_CONTACT])
         {
            IO_Sen_out->panel_stall_horn                   = FALSE;
         }
         else
            IO_Sen_out->panel_stall_horn = (unsigned char) outputs.duser[Y_D_STALL];

         if(IO_Sen_in->test_switch_4_up == SW_ON)
         {
            IO_Sen_out->panel_stall_horn = TRUE;
         }

         
         IO_Sen_out->panel_annunciator_gear_unsafe_lamp = (unsigned char) (outputs.duser[Y_D_GEAR]) ;
         IO_Sen_out->panel_left_start_engage_lamp       = DY_l_start;
         IO_Sen_out->panel_right_start_engage_lamp      = DY_r_start;
         IO_Sen_out->panel_flap_transit_lamp            = (unsigned char)outputs.duser[Y_D_FLAPS];
         IO_Sen_out->panel_annunciator_left_overboost_lamp = (outputs.user[Y_A_L_MAN_P] > 39.5);
         IO_Sen_out->panel_annunciator_right_overboost_lamp = (outputs.user[Y_A_R_MAN_P] > 39.8);
//         IO_Sen_out->panel_annunciator_left_gyro_air_lamp   =  IO_Sen_faults.FI.Left_SUCTION_PUMP;
//         IO_Sen_out->panel_annunciator_right_gyro_air_lamp  =  IO_Sen_faults.FI.Right_SUCTION_PUMP;
         if(RC_alt_stall_warner == TRUE)
         {
			   IO_Sen_out->panel_annunciator_heater_overheat_lamp = IO_Sen_out->panel_stall_horn;
         }
         else
         {
            IO_Sen_out->panel_annunciator_heater_overheat_lamp = UNLIT;
         }

         if(outputs.user[Y_A_R_RPM] > (outputs.user[Y_A_L_RPM] + 10.0))
         {
            IO_Sen_out->throttle_prop_sync_lamp = LIT;
         }
         else
            if(outputs.user[Y_A_R_RPM] < (outputs.user[Y_A_L_RPM] - 10.0))
            {
               IO_Sen_out->throttle_prop_sync_lamp = LIT;
            }
            else 
               IO_Sen_out->throttle_prop_sync_lamp = UNLIT;

if(SE_freeze_on == FALSE)
{
      IO_Sen_out->KFC150_FD_token                    = (unsigned char) outputs.duser[Y_D_KFC150_FD];
      IO_Sen_out->KFC150_ALT_token                   = (unsigned char) outputs.duser[Y_D_KFC150_ALT];
      IO_Sen_out->KFC150_HDG_token                   = (unsigned char) outputs.duser[Y_D_KFC150_HDG];
      IO_Sen_out->KFC150_GS_token                    = (unsigned char) outputs.duser[Y_D_KFC150_GS];
      IO_Sen_out->KFC150_NAV_token                   = (unsigned char) outputs.duser[Y_D_KFC150_NAV];
      IO_Sen_out->KFC150_APR_token                   = (unsigned char) outputs.duser[Y_D_KFC150_APR];
      IO_Sen_out->KFC150_BC_token                    = (unsigned char) outputs.duser[Y_D_KFC150_BC];
      IO_Sen_out->KFC150_TRIM_token                  = (unsigned char) outputs.duser[Y_D_KFC150_TRIM];
      IO_Sen_out->KFC150_AP_token                    = (unsigned char) outputs.duser[Y_D_KFC150_AP];
      IO_Sen_out->panel_annunciator_baggage_door_lamp = UNLIT;
      IO_Sen_out->panel_annunciator_cabin_door_lamp   = UNLIT;

}

//gear horn activates if left gear leg is not extended (i.e in contact with ground) and gear switch is placed in up position
// or gear is not locked down and throttle is in a low power setting and any MAP is below 14 inches
         if((outputs.duser[Y_D_L_CONTACT] == TRUE && IO_Sen_in->panel_landing_gear_lever == SW_ON) 
            ||((outputs.duser[Y_D_N_GEAR] < 0.85) && ((IO_Sen_in->left_throttle < 20.0) || (IO_Sen_in->right_throttle < 20.0))))
         {
            IO_Sen_out->panel_annunciator_gear_unsafe_lamp = LIT;
            gear_horn_timer += delta_time;

            if(gear_horn_timer < G_HORN_ON_TIMER)
            {
               IO_Sen_out->panel_gear_horn = LIT;
            }
            else
            if(gear_horn_timer < G_HORN_OFF_TIMER)
            {
               IO_Sen_out->panel_gear_horn = UNLIT;
            }
            else
               gear_horn_timer = 0.0;

         }
         else
         {
            IO_Sen_out->panel_gear_horn = 0;
            gear_horn_timer = 0.0;
         }
      }
      else
      {
         IO_Sen_out->panel_landing_gear_nose_lamp                = UNLIT;
         IO_Sen_out->panel_landing_gear_left_lamp                = UNLIT;
         IO_Sen_out->panel_landing_gear_right_lamp               = UNLIT;

         IO_Sen_out->panel_gear_horn                             = UNLIT;
         IO_Sen_out->panel_stall_horn                            = UNLIT;

         
         IO_Sen_out->panel_left_start_engage_lamp                = UNLIT;
         IO_Sen_out->panel_right_start_engage_lamp               = UNLIT;
         IO_Sen_out->panel_flap_transit_lamp                     = UNLIT;
         IO_Sen_out->throttle_prop_sync_lamp                     = UNLIT;
         IO_Sen_out->panel_low_bus_lamp                          = UNLIT;
         IO_Sen_out->panel_wing_tail_lamp                        = UNLIT;

         IO_Sen_out->panel_annunciator_gear_unsafe_lamp          = UNLIT;
         IO_Sen_out->panel_annunciator_left_overboost_lamp       = UNLIT;
         IO_Sen_out->panel_annunciator_right_overboost_lamp      = UNLIT;
         IO_Sen_out->panel_annunciator_oil_press_lamp            = UNLIT;
         IO_Sen_out->panel_annunciator_left_gyro_air_lamp        = UNLIT;
         IO_Sen_out->panel_annunciator_right_gyro_air_lamp       = UNLIT;
         IO_Sen_out->panel_annunciator_baggage_door_lamp         = UNLIT;
         IO_Sen_out->panel_annunciator_alt_out_lamp              = UNLIT;
         IO_Sen_out->panel_annunciator_heater_overheat_lamp      = UNLIT;
         IO_Sen_out->panel_annunciator_cabin_door_lamp           = UNLIT;
         IO_Sen_out->panel_annunciator_left_auxiliary_fuel_lamp  = UNLIT;
         IO_Sen_out->panel_annunciator_right_auxiliary_fuel_lamp = UNLIT;

         
         IO_Sen_out->KFC150_ALT_token                            = UNLIT;
         IO_Sen_out->KFC150_AP_token                             = UNLIT;
         IO_Sen_out->KFC150_APR_token                            = UNLIT;
         IO_Sen_out->KFC150_BC_token                             = UNLIT;
         IO_Sen_out->KFC150_FD_token                             = UNLIT;
         IO_Sen_out->KFC150_GS_token                             = UNLIT;
         IO_Sen_out->KFC150_HDG_token                            = UNLIT;
         IO_Sen_out->KFC150_NAV_token                            = UNLIT;
         IO_Sen_out->KFC150_TRIM_token                           = UNLIT;
         IO_Sen_out->KFC150_YD_token                             = UNLIT;
         IO_Sen_out->KR21_A_token                                = UNLIT;
         IO_Sen_out->KR21_M_token                                = UNLIT;
         IO_Sen_out->KR21_O_token                                = UNLIT;
			IO_Sen_out->left_fuel_quantity                          = 0.0;
         IO_Sen_out->right_fuel_quantity                         = 0.0;
         IO_Sen_out->left_oil_pressure                           = 0.0;
         IO_Sen_out->right_oil_pressure                          = 0.0;
         IO_Sen_out->left_oil_temperature                        = 0.0;
         IO_Sen_out->right_oil_temperature                       = 0.0;
         IO_Sen_out->left_cyclic_temperature                     = 0.0;
         IO_Sen_out->right_cyclic_temperature                    = 0.0;
      }


      
 
      IO_Sen_out->outside_air_temperature            = (outputs.temperature - 273);
// if control circuit breaker is pulled no indications are seen
      if(IO_Sen_in->MCB_landing_gear_control_tripped == SW_OFF)
      {
         IO_Sen_out->panel_annunciator_gear_unsafe_lamp = UNLIT;
         IO_Sen_out->panel_landing_gear_nose_lamp       = UNLIT;
         IO_Sen_out->panel_landing_gear_left_lamp       = UNLIT;
         IO_Sen_out->panel_landing_gear_right_lamp      = UNLIT;
         IO_Sen_out->panel_gear_horn                    = UNLIT;
      }

      // if power is available then turn on push to test char to enable push to test buttons
      // and enable panel power bit
      IO_Sen_out->panel_lo_bus_wing_tail_transit_lamp_PTT_enable = (unsigned char)EL_output.power;
      IO_Sen_out->system_lighting_power_control                  = (unsigned char)EL_output.power;
      IO_Sen_out->system_KMA24H_power_control                    = (unsigned char)EL_output.power;
      
   if(outputs.duser[Y_D_KFC150_FD] == TRUE)
   {
      if(IO_Sen_in->panel_FD_pilot_copilot_switch == FD_PILOT)
      {
         IO_Sen_out->pilot_FCI_cmd_enable   = ENABLE;
         IO_Sen_out->copilot_FCI_cmd_enable = DISABLE;
         IO_Sen_out->pilot_FCI_cmd_pitch   = (float) (outputs.user[Y_A_FD_PITCH] * R2D);
         IO_Sen_out->pilot_FCI_cmd_roll    = (float) (outputs.user[Y_A_FD_ROLL] * R2D);
         IO_Sen_out->copilot_FCI_cmd_pitch = 0.0;
         IO_Sen_out->copilot_FCI_cmd_roll  = 0.0;
      }
      else
      {
         IO_Sen_out->copilot_FCI_cmd_enable = ENABLE;
         IO_Sen_out->pilot_FCI_cmd_enable   = DISABLE;
         IO_Sen_out->copilot_FCI_cmd_pitch = (float) (outputs.user[Y_A_FD_PITCH] * R2D);
         IO_Sen_out->copilot_FCI_cmd_roll  = (float) (outputs.user[Y_A_FD_ROLL] * R2D);
         IO_Sen_out->pilot_FCI_cmd_pitch   = 0.0;
         IO_Sen_out->pilot_FCI_cmd_roll    = 0.0;

      }
   }
   else
   {
      IO_Sen_out->copilot_FCI_cmd_enable = DISABLE;
      IO_Sen_out->pilot_FCI_cmd_enable   = DISABLE;
      IO_Sen_out->copilot_FCI_cmd_pitch = 0.0;
      IO_Sen_out->copilot_FCI_cmd_roll  = 0.0;
      IO_Sen_out->pilot_FCI_cmd_pitch   = 0.0;
      IO_Sen_out->pilot_FCI_cmd_roll    = 0.0;

   }

   
}

/*      
float calc_engine_instruments(float rate,float real_data,int inst_fault,float last_value,int real_fault,float max_value)
{
   float instrument_value;

   switch(inst_fault)
   {
      case INST_MAX :
         if(last_value < max_value)
           {
              last_value += delta_time;
           }
           else
              last_value = max_value;
           break;

        case INST_MIN :
           if(last_value > 0.0)
           {
              last_value -= delta_time;
           }
           else
              last_value = 0.0;
           break;

        case INST_STUCK :
           last_value = last_value;
           break;
        default :
           
           break;
        }


   return 0.0;
}      
*/
