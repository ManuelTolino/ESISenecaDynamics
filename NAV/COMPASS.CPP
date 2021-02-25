/*
File: compass.c
Prefix: CS_

Module: Dynamics for Seneca

Created: 4 February 2000

Description: This file calculates all the different heading values
             and compass values

Inputs: Correct magnetic heading
        Mode of gyro slaving unit
        Gyro correction value, left or right
        Aircraft acceleration and rate of turn (for standby compass only)

Outputs: Left and right gyro heading
         Standby compass heading
*/

/***************************/
/*   RCS Markers           */
/***************************/

/*
$Header: C:/master/seneca/NAV/RCS/compass.cpp 1.9 2000/05/30 14:21:14 colinj Exp $
$Log: compass.cpp $
Revision 1.9  2000/05/30 14:21:14  colinj
Make magnetic variation active.

Revision 1.8  2000/05/25 15:09:15  colinj
If left or right gyro fault, set pilot/copilot_hdg_flag.
New routines, CS_true_to_mag_head and CS_mag_to_true_head.

Revision 1.7  2000/05/16 18:07:45  colinj
Simulate hunting of compass in gyro warm up.

Revision 1.6  2000/05/16 16:34:06  colinj
Change gyro run up time. New CS_pilot/copilot_hdg_flags
Different gyro error rate for gyro fault and free mode drift.
In slave mode, impose max rate at which error is corrected.
Slaving meter now shows any difference between gyro and real mhead.
Gyro depends on avionic power.

Revision 1.5  2000/04/17 12:07:21  juliae
Changed SLAVE_CHANGE_RATE and SLAVE_DSP_RANGE

Revision 1.4  2000/04/12 10:03:10  juliae
(Colin) Allow for gyro drift and gyro faults.

 * Revision 1.3  2000/04/07  15:53:29  juliae
 * Set_stdby_compass now handles acceleration and turn errors.
 *
 * Revision 1.2  2000/03/02  16:59:44  juliae
 * Correction to setting slaving_meter values and resetting it when
 * you change to Slave mode.
 *
 * Revision 1.1  2000/02/22  01:21:49  juliae
 * Initial revision
 *
*/

/*----------------------------------------------------------------------*/

/*----------------*/
/* Include Files  */
/*----------------*/

#include <stdio.h>
#include <math.h>

#include "c:\bae-sims\seneca\dat_stru/in_head.h"
#include "c:\bae-sims\seneca\dat_stru/out_head.h"
#include "c:\bae-sims\seneca\define.h"
#include "cpit_def.h"
#include "c:\bae-sims\seneca\fltdy/fmgdefs.h"
#include "c:\bae-sims\seneca\faults.h"

/*----------------*/
/* Defines        */
/*----------------*/

#define TRUE  1
#define FALSE 0

#define SLAVE_CHANGE_RATE (180.0f/35.0f)   //180 deg in 35 sec
#define MAX_SLAVE_SPEED   (180.0f/56.0f)
#define GYRO_RUN_UP_TIME  36.0f  //Seconds

#define SLAVE_DSP_RANGE   30.0f   //Show +/- 30 degrees

//Set drift rate at 1 degree/min
#define STD_DRIFT_RATE    (1.0f / 60.0f)

#define FAULT_CHANGE_RATE (1.0f / 6.0f)

/* Same units as outputs.ax. This is the acceleration which
   gives maximum compass error */
#define MAX_FWD_ACC    8.0f

/* Same units as states.r (rad/s). This is the yaw rate which
   gives maximum compass error */
#define MAX_YAW_RATE   (PI / 60.0f)    //180 deg per min

/* Maximum deflection of the compass needle that acceleration
   can cause. In degrees */
#define MAX_COMP_ACC_ERR   30.0f

/* Maximum deflection of the compass needle that turn
   can cause. In degrees */
#define MAX_COMP_TURN_ERR  30.0f

/* Maximum total deflection of the compass needle from correct
   position. In degrees */
#define MAX_COMP_ERR       40.0f

//#define IGNORE_MAG_VAR 1

/*----------------*/
/* Typedefs       */
/*----------------*/

/*--------------------*/
/* External Routines  */
/*--------------------*/

/*--------------------*/
/* External Variables */
/*--------------------*/

extern sen_in_struct  *IO_Sen_in;
extern OUTPUTin			*OUTPUTinbox;//simulinkudp
extern sen_out_struct *IO_Sen_out;
extern sen_fault_struct IO_Sen_faults;
extern float           delta_time;
extern int             IO_elec_pwr;
extern int             IO_avionic_pwr;

extern OUTPUTS       outputs;
extern STATES        states;

/*----------------*/
/* Globals        */
/*----------------*/

float  CS_gyro_copilot_mhead = 0.0;
float  CS_gyro_pilot_mhead = 0.0;

#ifdef IGNORE_MAG_VAR
float  CS_magnetic_var = 0.0;
#else
float  CS_magnetic_var = 5.0;
#endif
float  CS_real_host_mhead = 0.0;   //I dont know if this needs to be global

int    CS_pilot_hdg_flag = FALSE;  // set true if compass is motoring
int    CS_copilot_hdg_flag = FALSE;  // set true if compass is motoring
int    CS_gyro_ok_speed = FALSE;   //We use a single flag for both gyros
/*-----------------*/
/* Local Variables */    
/*-----------------*/

static float  Gyro_run_time = 0.0;
static float  P1_gyro_error = 0.000;
static float  P2_gyro_error = 0.000;
static float  P1_left_error    = 0.0;
static float  P1_right_error   = 0.0;
static float  P2_left_error    = 0.0;
static float  P2_right_error   = 0.0;

static char   P1_swap = FALSE;
static char   P2_swap = FALSE;
/*--------------------*/
/* Forward References */
/*--------------------*/

/*----------------------------------------------------------------------*/

void CS_gyro_init()
{
   CS_gyro_ok_speed = FALSE;   //ie. gyro not up to speed
   Gyro_run_time = 0.0;

   IO_Sen_out->pilot_KA51_slaving_meter = 0.0;
   IO_Sen_out->copilot_KA51_slaving_meter = 0.0;
   P1_gyro_error =  0.000;
   P2_gyro_error =  0.000;
   P1_left_error    = -5.5;
   P1_right_error   =  4.5;
   P2_left_error    = -4.5;
   P2_right_error   =  5.5;
   P1_swap       = TRUE;
   P2_swap       = FALSE;
}

/*----------------------------------------------------------------------*/

/* Sets up magnetic heading of host from its true heading.
   The magnetic variation applied should be calculated according
   to the host position.

   Inputs: true heading in degrees
           lat/lon position in degrees

   Outputs: magnetic heading in degrees
            CS_magnetic_var
*/

void CS_calc_mag_head(float true_head, double host_lat, double host_lon)
{
   CS_real_host_mhead = true_head + CS_magnetic_var;
}

/*----------------------------------------------------------------------*/

/* Inputs: correct magnetic heading
           aircraft acceleration
           aircraft rate of turn
*/

static void Set_stdby_compass()
{
   float err1_compass_north;
   float err2_compass_north;
   float err_compass_north;
   float sin_h;
   float cos_h;
   float c_of_g_dev;
   float fwd_acc;
   float yaw_rate;
   float constant1;
   float constant2;

   /* This varies from real magnetic heading depending on
      aircraft acceleration and rate of turn */

   /* Acceleration and rate of turn alter the standby compass
      needle from its proper magnetic north direction. Consequently
      the reading of the host's current heading is affected.
      So we calculate the position of the compass North (0 for
      correct readings) and then use this to calculate the
      read value of the magnetic heading */

   /* The errors occur as a result of the deviation of the
      centre of gravity of the compass from the pivot (which
      is done to overcome dip) so set up a value for this.
      It increases as you go away from the equator */

   c_of_g_dev = (float) sin (states.latitude);   /* Range -1 to +1 */


   /* Acceleration errors */
   /* ------------------- */

   /* Basically, along East/West the error is proportional to
      the acceleration in the other direction, ie. error clockwise
      as you accelerate West.
      Other factors multiply the acceleration to reduce the error
      as you near the equator and as the heading changes from
      West or East.
      The factor sin_h is ONLY used as a control factor so that
      the error is max at West and East and 0 at North and South.
      A different factor may be required to reduce the error
      more quickly as the heading changes from West or East */

   sin_h = (float) sin(CS_real_host_mhead * DEG_TO_RAD);
   constant1 = MAX_COMP_ACC_ERR / MAX_FWD_ACC;
   fwd_acc = outputs.ax;
   if (fwd_acc > MAX_FWD_ACC)
      fwd_acc = MAX_FWD_ACC;
   else if (fwd_acc < -MAX_FWD_ACC)
      fwd_acc = -MAX_FWD_ACC;

   err1_compass_north = constant1 * c_of_g_dev * fwd_acc * sin_h;


   /* Turn errors */
   /* ----------- */

   cos_h = (float) cos(CS_real_host_mhead * DEG_TO_RAD);
   constant2 = (float) (MAX_COMP_TURN_ERR / MAX_YAW_RATE);
   yaw_rate = states.r;   /* Assume +ve clockwise */
   if (yaw_rate > MAX_YAW_RATE)
      yaw_rate = (float) MAX_YAW_RATE;
   else if (yaw_rate < -MAX_YAW_RATE)
      yaw_rate = (float) -MAX_YAW_RATE;
   err2_compass_north = constant2 * c_of_g_dev * yaw_rate * cos_h;


   /* Total error */
   /* ----------- */

   err_compass_north = err1_compass_north + err2_compass_north;

   if (err_compass_north > MAX_COMP_ERR)
      err_compass_north = MAX_COMP_ERR;
   else if (err_compass_north < -MAX_COMP_ERR)
      err_compass_north = -MAX_COMP_ERR;


   IO_Sen_out->compass_heading = CS_real_host_mhead - err_compass_north;

   /* Keep in range 0 to 360 */
   if (IO_Sen_out->compass_heading >= 360.0)
      IO_Sen_out->compass_heading -= 360.0;
   if (IO_Sen_out->compass_heading < 0.0)
      IO_Sen_out->compass_heading += 360.0;
}

/*----------------------------------------------------------------------*/

static void Calc_pilot_gyro_head()
{
   float max_slave_change;

   if (!IO_avionic_pwr)
      return;

   CS_pilot_hdg_flag = FALSE;

   //GYRO FAILURE
   if (IO_Sen_faults.AVF.left_gyro == TRUE)
   {
      P1_gyro_error += (FAULT_CHANGE_RATE * delta_time);
      CS_pilot_hdg_flag = TRUE;
   }
   else //NORMAL OPERATION
   {
      if (IO_Sen_in->pilot_KA51_mode_switch == KA51_SLAVE)
      {
         if (CS_gyro_ok_speed == TRUE)
         {
            max_slave_change = delta_time * MAX_SLAVE_SPEED;   //This frame

            if (P1_gyro_error > 0.1)   //Positive GYRO ERROR
            {
               if (P1_gyro_error > max_slave_change)
               {
                  P1_gyro_error -= max_slave_change;
                  CS_pilot_hdg_flag = TRUE;
               }
               else
                  P1_gyro_error = 0.0;
            }
            else if (P1_gyro_error < -0.1)  //Negative GYRO ERROR
            {
               if (P1_gyro_error < max_slave_change)
               {
                  P1_gyro_error += max_slave_change;
                  CS_pilot_hdg_flag = TRUE;
               }
               else
                  P1_gyro_error = 0.0;
            }
         }   //End of gyro_ok_speed
      }   //End of SLAVE mode
      else
      {
         P1_gyro_error += (STD_DRIFT_RATE * delta_time);
      }
   }

   if (P1_gyro_error > 360.0)
      P1_gyro_error -= 360.0;


   //The slaving meter always shows the error between the gyro_mhead and real_mhead
   if (P1_gyro_error < 180.0)
      IO_Sen_out->pilot_KA51_slaving_meter = ((P1_gyro_error) / SLAVE_DSP_RANGE) * 100.0f;
   else
      IO_Sen_out->pilot_KA51_slaving_meter = ((P1_gyro_error-360.0f) / SLAVE_DSP_RANGE) * 100.0f;

   if (IO_Sen_out->pilot_KA51_slaving_meter > 100.0)
      IO_Sen_out->pilot_KA51_slaving_meter = 100.0;
   else if (IO_Sen_out->pilot_KA51_slaving_meter < -100.0)
      IO_Sen_out->pilot_KA51_slaving_meter = -100.0;

   CS_gyro_pilot_mhead = CS_real_host_mhead + P1_gyro_error;

   /* Keep in range 0 to 360 */
   if (CS_gyro_pilot_mhead >= 360.0)
      CS_gyro_pilot_mhead -= 360.0;
   if (CS_gyro_pilot_mhead < 0.0)
      CS_gyro_pilot_mhead += 360.0;
}

/*----------------------------------------------------------------------*/

static void Calc_copilot_gyro_head()
{
   float max_slave_change = 0.0f;

   if (!IO_avionic_pwr)
      return;

   CS_copilot_hdg_flag = FALSE;

   //GYRO FAILURE
   if (IO_Sen_faults.AVF.right_gyro == TRUE)
   {
      P2_gyro_error += (FAULT_CHANGE_RATE * delta_time);
      CS_copilot_hdg_flag = TRUE;
   }
   else //NORMAL OPERATION
   {
      if (IO_Sen_in->copilot_KA51_mode_switch == KA51_SLAVE)
      {
         if (CS_gyro_ok_speed == TRUE)
         {
            max_slave_change = delta_time * MAX_SLAVE_SPEED;   //This frame

            if (P2_gyro_error > 0.1)   //Positive GYRO ERROR
            {
               if (P2_gyro_error > max_slave_change)
               {
                  P2_gyro_error -= max_slave_change;
                  CS_copilot_hdg_flag = TRUE;
               }
               else
                  P2_gyro_error = 0.0;
            }
            else if (P2_gyro_error < -0.1)   //Negative GYRO ERROR
            {
               if (P2_gyro_error < max_slave_change)
               {
                  P2_gyro_error += max_slave_change;
                  CS_copilot_hdg_flag = TRUE;
               }
               else
                  P2_gyro_error = 0.0;
            }
         }   //End of gyro_ok_speed
      }   //End of SLAVE mode
      else
      {
         P2_gyro_error += (STD_DRIFT_RATE * delta_time);
      }
   }

   if (P2_gyro_error > 360.0)
      P2_gyro_error -= 360.0;


   //The slaving meter always shows the error between the gyro_mhead and real_mhead
   if (P2_gyro_error < 180.0)
      IO_Sen_out->copilot_KA51_slaving_meter = ((P2_gyro_error) / SLAVE_DSP_RANGE) * 100.0f;
   else
      IO_Sen_out->copilot_KA51_slaving_meter = ((P2_gyro_error-360.0f) / SLAVE_DSP_RANGE) * 100.0f;

   if (IO_Sen_out->copilot_KA51_slaving_meter > 100.0)
      IO_Sen_out->copilot_KA51_slaving_meter = 100.0;
   else if (IO_Sen_out->copilot_KA51_slaving_meter < -100.0)
      IO_Sen_out->copilot_KA51_slaving_meter = -100.0;

   CS_gyro_copilot_mhead = CS_real_host_mhead + P2_gyro_error;

   /* Keep in range 0 to 360 */
   if (CS_gyro_copilot_mhead >= 360.0)
      CS_gyro_copilot_mhead -= 360.0;
   if (CS_gyro_copilot_mhead < 0.0)
      CS_gyro_copilot_mhead += 360.0;
}

/*----------------------------------------------------------------------*/

/* This should be called whenever the power is CHANGED to on or off */

void CS_master_power_change()
{
   if (IO_elec_pwr)   //Power just turned on
   {
      Gyro_run_time = 0.0;
      CS_gyro_ok_speed = FALSE;
   }
   else
      CS_gyro_ok_speed = FALSE;
}

/*----------------------------------------------------------------------*/

/* In cases when we want to quick start to a flying position,
   need the gyro immediately up to speed */

void CS_gyro_ok()
{
   CS_gyro_ok_speed = TRUE;
}

/*----------------------------------------------------------------------*/

/* This should be called regularly */

void CS_update()
{
   if (IO_avionic_pwr)
   {
      if (!CS_gyro_ok_speed)
      {
         
         //SET UP GYRO ERRORS TO SIMULATE "HUNTING" OF REAL COMPASS
         if (P1_swap == TRUE)
         {
            if (P1_gyro_error > P1_left_error)
            {
               P1_gyro_error -= (float)(delta_time * 3.5);
            }
            else
            {
               P1_swap = FALSE;
               P1_left_error += 0.25;
            }
         }
         else if (P1_gyro_error < P1_right_error)
         {
            P1_gyro_error += (float)(delta_time * 3.0);
         }
         else
         {
            P1_swap = TRUE;
            P1_right_error -= 0.25;
         }

         if (P2_swap == TRUE)
         {
            if (P2_gyro_error > P2_left_error)
            {
               P2_gyro_error -= (float)(delta_time * 3.5);
            }
            else
            {
               P2_swap = FALSE;
               P2_left_error += 0.25;
            }
         }
         else if (P2_gyro_error < P2_right_error)
         {
            P2_gyro_error += (float)(delta_time * 3.0);
         }
         else
         {
            P2_swap = TRUE;
            P2_right_error -= 0.25;
         }


         Gyro_run_time += delta_time;
         if (Gyro_run_time > GYRO_RUN_UP_TIME)
         {
            CS_gyro_ok_speed = TRUE;
         }
      }
   }
   else
   {
      if(Gyro_run_time > 0.0)
      {
         Gyro_run_time -= (float)(delta_time/5.0);
         CS_gyro_ok_speed = FALSE;
      }
   }

   Set_stdby_compass();
   Calc_pilot_gyro_head();
   Calc_copilot_gyro_head();
}

/*----------------------------------------------------------------------*/

/* This detects and acts on changes to the compass slave control
   KA51 unit.
   If we want to "shake" the slaving meter values, need to do this
   AFTER this routine */

void CS_gyro_slave_button_changes()
{
   /* PILOT */
   /* ----- */
   if (IO_Sen_faults.AVF.left_gyro == FALSE)
   {
      if (IO_Sen_in->pilot_KA51_slew_CW_switch == SW_ON && IO_Sen_in->pilot_KA51_mode_switch != KA51_SLAVE)
      {
         P1_gyro_error += (SLAVE_CHANGE_RATE * delta_time);
         CS_pilot_hdg_flag = TRUE;
      }
      if (IO_Sen_in->pilot_KA51_slew_CCW_switch == SW_ON && IO_Sen_in->pilot_KA51_mode_switch != KA51_SLAVE)
      {
         P1_gyro_error -= (SLAVE_CHANGE_RATE * delta_time);
         CS_pilot_hdg_flag = TRUE;
      }
   }

   /* COPILOT */
   /* ------- */
   if (IO_Sen_faults.AVF.right_gyro == FALSE)
   {
      if (IO_Sen_in->copilot_KA51_slew_CW_switch == SW_ON && IO_Sen_in->copilot_KA51_mode_switch != KA51_SLAVE)
      {
         P2_gyro_error += (SLAVE_CHANGE_RATE * delta_time);
         CS_copilot_hdg_flag = TRUE;
      }
      if (IO_Sen_in->copilot_KA51_slew_CCW_switch == SW_ON && IO_Sen_in->copilot_KA51_mode_switch != KA51_SLAVE)
      {
         P2_gyro_error -= (SLAVE_CHANGE_RATE * delta_time);
         CS_copilot_hdg_flag = TRUE;
      }
   }
}

/*----------------------------------------------------------------------*/

float CS_true_to_mag_head(float true_head)
{
   float mhead;

   mhead = true_head + CS_magnetic_var;

   /* Keep in range 0 to 360 */
   if (mhead >= 360.0)
      mhead -= 360.0;
   if (mhead < 0.0)
      mhead += 360.0;

   return mhead;
}

/*----------------------------------------------------------------------*/

float CS_mag_to_true_head(float mag_head)
{
   float thead;

   thead = mag_head - CS_magnetic_var;

   /* Keep in range 0 to 360 */
   if (thead >= 360.0)
      thead -= 360.0;
   if (thead < 0.0)
      thead += 360.0;

   return thead;
}

/*----------------------------------------------------------------------*/
