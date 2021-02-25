/*
File  : adf_av.c
Prefix: ADF_

Module: Dynamics for Seneca
Created: February 7th 2000

Description: This file handles user input on the KR87 ADF avionics box
*/

/***************************/
/*   RCS Markers           */
/***************************/

/*
$Header: C:/Master/seneca/NAV/RCS/adf_av.cpp 1.6 2000/06/16 10:27:42 ColinJ Exp $
$Log: adf_av.cpp $
Revision 1.6  2000/06/16 10:27:42  ColinJ
In freeze mode stop flight timers.

Revision 1.5  2000/04/25 16:12:06  juliae
Save and restore data to file. New routine Adf_pwr_on.

Revision 1.4  2000/04/18 15:08:24  juliae
Tune freq when power is turned on. Set old beacon info
invalid when FRQ button is pressed.
Added support for requesting beacon info constantly.

Revision 1.3  2000/04/17 12:10:57  juliae
Support for leading zero on freq display when flashing.
FLT/ET button returns to previous time display.

 * Revision 1.2  2000/03/17  16:33:17  juliae
 * Change IO_elec_pwr to IO_avionic_pwr. Allow for adf fault.
 *
 * Revision 1.1  2000/03/01  14:29:14  juliae
 * Initial revision
 *
*/

/*----------------------------------------------------------------------*/

/*----------------*/
/* Include Files  */
/*----------------*/

#include <stdio.h>
#include <math.h>

#include "beacon.h"
#include "../dat_stru/out_head.h"
#include "../dat_stru/in_head.h"
#include "../define.h"
#include "../faults.h"

/*----------------*/
/* Defines        */
/*----------------*/

#define MAX_COUNTUP_TIME     15.0   /* Countdown will count back up to 15 */
#define MAX_IDLE_ET_SET_TIME 15.0
#define ENABLE_ET_SET_TIME    2.0   /* If button pressed for 2 sec, enable ET_SET_MODE */

#define MAX_NDB_FREQ  1799          /* Whole KHz */
#define MIN_NDB_FREQ  200
#define DEFAULT_NDB_FREQ MIN_NDB_FREQ

#define FLASH_INTERVAL 0.5       /* Seconds */

#define TOKEN_ON   LIT
#define TOKEN_OFF  UNLIT

#define ET_NORMAL_MODE    0
#define ET_SET_MODE       1
#define ET_COUNTDOWN_MODE 2
#define ET_COUNTUP_MODE   3

#define FREQ_DISPLAY 0
#define FLT_DISPLAY  1
#define ET_DISPLAY   2

#define MAX_MIN    59   /* Max display is 59:59 */
#define MAX_10_SEC  5
#define MAX_1_SEC   9

#define MAX_100_NDB_FREQ  17
#define MIN_100_NDB_FREQ   2

/* Keeps track of changes to SET_RST button */
#define CDOWN_ON_RELEASE 3
#define CDOWN_PENDING    2
#define LONG_PRESS       1
#define NORMAL_USE       0

#define ADF_OPCODE 4

/*----------------*/
/* Typedefs       */
/*----------------*/

typedef struct
{
   unsigned char opcode;
   unsigned char stdby_mode;
   unsigned char stdby_time_mode;
   char          spare;
   long          use_freq;
   long          stby_freq;

} adf_kept_memory;

/*--------------------*/
/* External Routines  */
/*--------------------*/

/*--------------------*/
/* External Variables */
/*--------------------*/

extern sen_out_struct *IO_Sen_out;
extern sen_in_struct  *IO_Sen_in;
extern OUTPUTin			*OUTPUTinbox;//simulinkudp
extern sen_fault_struct IO_Sen_faults;
extern ndb_beac_attr   IO_ndb_battr;

extern float           delta_time;

extern int             IO_avionic_pwr;
extern int             SE_freeze_on;
extern int             IO_iosfreeze;

/*----------------*/
/* Globals        */
/*----------------*/

int ADF_new_ndb_bfreq = FALSE;
int ADF_search_mode = FALSE;

/*-----------------*/
/* Local Variables */    
/*-----------------*/

/* Flags for momentary button changes */
static int Last_KR87_FRQ_button;
static int Last_KR87_FLT_ET_button;
static int Last_KR87_SET_RST_button;

/* For KR87 turn dial counts */
static long Last_turn1 = 0;
static long Last_turn2 = 0;
static long Last_turn3 = 0;

static int Last_pwr_on = FALSE;
static int Stage_SET_RST = NORMAL_USE;
static int Mode_ET  = ET_NORMAL_MODE;

static int Stdby_mode = FREQ_DISPLAY;
static long Stdby_freq = DEFAULT_NDB_FREQ;
static int Stdby_time_mode = FLT_DISPLAY;

static float Flt_timer = 0.0;
static float Elapsed_time = 0.0;

static float Press_SET_RST_button_time = 0.0;
static float Idle_set_mode_time = 0.0;

static int   Flash_ET_display = FALSE;   //Actually STDBY display
static float Flash_ET_display_time = 0.0;
static int   Flash_ET = FALSE;
static float Flash_ET_time = 0.0;

/*--------------------*/
/* Forward References */
/*--------------------*/

static void Adf_box_active();
static void Adf_box_blank();
static void Adf_update_alt_but_tokens();
static void Adf_update_for_stdby_mode();
static void Stdby_freq_update();
static void Active_freq_update();
static void Time_update();
static void Tune_ndb_beacon();
static void Tune_off_ndb_beacon();
static void Reset_turn_counts();
static void Init_kept_memory();
static void Read_kept_memory(char *filename);
static long Set_time_display(float seconds);
/*----------------------------------------------------------------------*/

void ADF_adf_init()
{
   IO_ndb_battr.valid_flag = FALSE;

   Last_pwr_on = FALSE;
   if (IO_avionic_pwr && (IO_Sen_in->KR87_off_on_switch == SW_ON) && !IO_Sen_faults.AVF.adf)
      Last_pwr_on = TRUE;

   /* Flags for momentary button changes */
   Last_KR87_FRQ_button     = IO_Sen_in->KR87_FRQ_button;
   Last_KR87_FLT_ET_button  = IO_Sen_in->KR87_FLT_ET_button;
   Last_KR87_SET_RST_button = IO_Sen_in->KR87_SET_RST_button;

   /* For KR87 turn dial counts */
   Reset_turn_counts();

   Stage_SET_RST = NORMAL_USE;
   Mode_ET = ET_NORMAL_MODE;

   Press_SET_RST_button_time = 0.0;

   Flash_ET_display = FALSE;
   Flash_ET_display_time = 0.0;
   Flash_ET = FALSE;
   Flash_ET_time = 0.0;

   Init_kept_memory();
   Read_kept_memory("adf.cfg");

   if (!Last_pwr_on)
   {
      Adf_box_blank();
      Tune_off_ndb_beacon();
   }
   else
   {
      Adf_box_active();
      Tune_ndb_beacon();
   }

   /* Dont need to update tokens for switches, since this is
      done in Adf_button_changes */
}

/*----------------------------------------------------------------------*/

/* This performs all the initialization required when power is restored
   to adf box. See also ADF_adf_init. Do not initialize anything here
   that is included in "kept memory" */

static void Adf_pwr_on()
{
   /* Flags for momentary button changes */
   Last_KR87_FRQ_button     = IO_Sen_in->KR87_FRQ_button;
   Last_KR87_FLT_ET_button  = IO_Sen_in->KR87_FLT_ET_button;
   Last_KR87_SET_RST_button = IO_Sen_in->KR87_SET_RST_button;

   /* For KR87 turn dial counts */
   Reset_turn_counts();

   Stage_SET_RST = NORMAL_USE;
   Mode_ET = ET_NORMAL_MODE;

   Press_SET_RST_button_time = 0.0;

   Flash_ET_display = FALSE;
   Flash_ET_display_time = 0.0;
   Flash_ET = FALSE;
   Flash_ET_time = 0.0;

   /* Reset timer to 0 */
   Flt_timer = 0.0;      /* In seconds */
   Elapsed_time = 0.0;   /* In seconds */

   Adf_box_active();
   IO_ndb_battr.valid_flag = FALSE;
   Tune_ndb_beacon();
}

/*----------------------------------------------------------------------*/

static void Reset_turn_counts()
{
   /* For KR87 turn dial counts */
   Last_turn1 = IO_Sen_in->KR87_delta_outer_encoder;
   Last_turn2 = IO_Sen_in->KR87_delta_inner_encoder;
   Last_turn3 = IO_Sen_in->KR87_delta_inner_encoder_pull;
}

/*----------------------------------------------------------------------*/

static void Adf_button_changes()
{
   long ltmp;

   if (!IO_avionic_pwr || IO_Sen_faults.AVF.adf)
   {
      if (Last_pwr_on)
      {
         Adf_box_blank();
         Tune_off_ndb_beacon();
      }
      Last_pwr_on = FALSE;
      return;
   }

   if (IO_Sen_in->KR87_off_on_switch == SW_OFF)
   {
      if (Last_pwr_on)
      {
         /* Blank displays */
         Adf_box_blank();
         Tune_off_ndb_beacon();
      }
      Last_pwr_on = FALSE;
      return;
   }
   else   /* SW_ON */
   {
      if (!Last_pwr_on)
      {
         /* Have just restored power, EITHER by the local on/off
            switch or global power (global power could have
            been turned on with local switch still on).
            Reset timer to 0 */
         Adf_pwr_on();
      }
      Last_pwr_on = TRUE;
   }

   /* Test for change to momentary FRQ pushbutton */
   if ((Last_KR87_FRQ_button == PRESSED) &&
       (IO_Sen_in->KR87_FRQ_button == NOT_PRESSED))
   {
      if (Stdby_mode == FREQ_DISPLAY)
      {
         /* This button swaps the standby and active
            frequencies */
         ltmp = IO_Sen_out->KR87_USE_display;
         IO_Sen_out->KR87_USE_display = Stdby_freq;
         Stdby_freq = ltmp;
         IO_Sen_out->KR87_STBY_TIME_display = Stdby_freq;

         IO_ndb_battr.valid_flag = FALSE;
         Tune_ndb_beacon();
      }
      else
      {
         Stdby_mode = FREQ_DISPLAY;
         Adf_update_for_stdby_mode();
      }
   }

   Last_KR87_FRQ_button = IO_Sen_in->KR87_FRQ_button;


   /* Test for change to momentary FLT_ET pushbutton.
      Should this be ignored if in ET_SET_MODE? */
   if ((Last_KR87_FLT_ET_button == PRESSED) &&
       (IO_Sen_in->KR87_FLT_ET_button == NOT_PRESSED))
   {
      if (Stdby_mode == FREQ_DISPLAY)
      {
         //Change back to the last used time mode
         Stdby_mode = Stdby_time_mode;
      }
      else
      {
         if (Stdby_mode == FLT_DISPLAY)
            Stdby_mode = ET_DISPLAY;
         else
            Stdby_mode = FLT_DISPLAY;
         Stdby_time_mode = Stdby_mode;
      }

      Adf_update_for_stdby_mode();
   }

   Last_KR87_FLT_ET_button = IO_Sen_in->KR87_FLT_ET_button;

   /* Test for change to momentary SET_RST pushbutton.
      A simple press and release resets the time, a
      longer press enables countdown mode */

   if ((Last_KR87_SET_RST_button == NOT_PRESSED) &&
       (IO_Sen_in->KR87_SET_RST_button == PRESSED))
   {
      Press_SET_RST_button_time = 0.0;
      if (Stage_SET_RST == CDOWN_PENDING)
         Stage_SET_RST = CDOWN_ON_RELEASE;
   }
   if ((Last_KR87_SET_RST_button == PRESSED) &&
       (IO_Sen_in->KR87_SET_RST_button == NOT_PRESSED))
   {
      Press_SET_RST_button_time = 0.0;

      if (Stage_SET_RST == LONG_PRESS)   /* Do nothing until button is pressed again */
         Stage_SET_RST = CDOWN_PENDING;
      else if (Stage_SET_RST == CDOWN_ON_RELEASE)
      {
         /* Activate countdown */
         Flash_ET = FALSE;
         /* Make sure the flashing stops with the token on */
         IO_Sen_out->KR87_ET_token = TOKEN_ON;

         Stage_SET_RST = NORMAL_USE;
         Mode_ET = ET_COUNTDOWN_MODE;
      }
      else
      {
         Elapsed_time = 0.0;
         /* Update display */
         IO_Sen_out->KR87_STBY_TIME_display = Set_time_display(Elapsed_time);
      }
   }
   
   Last_KR87_SET_RST_button = IO_Sen_in->KR87_SET_RST_button;

   Adf_update_alt_but_tokens();
}

/*----------------------------------------------------------------------*/

/* Need to setup 1 min, 42 seconds as 142 */

static long Set_time_display(float seconds)
{
   long num_min, num_sec;
   long total_sec;
   long num_hr;

   total_sec = (long) seconds;
   if(total_sec >= 3600) // 1Hour
   {
      num_hr = total_sec / 3600;
      num_min = (total_sec % 3600) /60;
      return((num_hr * 100) + num_min);
   }
   else
   {

      num_min = (long) total_sec / 60;
      num_sec = (long) total_sec % 60;

      return ((num_min * 100) + num_sec);
   }
}


/*----------------------------------------------------------------------*/

static void Adf_update_alt_but_tokens()
{
   if (IO_Sen_in->KR87_ADF_ANT_switch == KR87_ADF)
   {
      IO_Sen_out->KR87_ADF_token = TOKEN_ON;
      IO_Sen_out->KR87_ANT_token = TOKEN_OFF;
   }
   else
   {
      IO_Sen_out->KR87_ANT_token = TOKEN_ON;
      IO_Sen_out->KR87_ADF_token = TOKEN_OFF;
   }

   if (IO_Sen_in->KR87_BFO_switch == SW_ON)
      IO_Sen_out->KR87_BFO_token = TOKEN_ON;
   else
      IO_Sen_out->KR87_BFO_token = TOKEN_OFF;
}

/*----------------------------------------------------------------------*/

/* This is used to update the KR87 values to correspond
   to standby mode

   Inputs:  Stdby_mode
   Outputs: IO_Sen_out for KR87 box
*/

static void Adf_update_for_stdby_mode()
{
   switch (Stdby_mode)
   {
      case FLT_DISPLAY:
         IO_Sen_out->KR87_FLT_token   = TOKEN_ON;
         IO_Sen_out->KR87_ET_token    = TOKEN_OFF;
         IO_Sen_out->KR87_FRQ_token   = TOKEN_OFF;
         IO_Sen_out->KR87_colon_token = TOKEN_ON;

         IO_Sen_out->KR87_STBY_TIME_display = Set_time_display(Flt_timer);
         break;

      case ET_DISPLAY:
         IO_Sen_out->KR87_FLT_token   = TOKEN_OFF;
         IO_Sen_out->KR87_ET_token    = TOKEN_ON;
         IO_Sen_out->KR87_FRQ_token   = TOKEN_OFF;
         IO_Sen_out->KR87_colon_token = TOKEN_ON;

         IO_Sen_out->KR87_STBY_TIME_display = Set_time_display(Elapsed_time);
         break;

      default:
      case FREQ_DISPLAY:
         IO_Sen_out->KR87_FLT_token   = TOKEN_OFF;
         IO_Sen_out->KR87_ET_token    = TOKEN_OFF;
         IO_Sen_out->KR87_FRQ_token   = TOKEN_ON;
         IO_Sen_out->KR87_colon_token = TOKEN_OFF;

         IO_Sen_out->KR87_STBY_TIME_display = Stdby_freq;
         break;
   }
}

/*----------------------------------------------------------------------*/

static void Init_ET_set_mode()
{
   Mode_ET = ET_SET_MODE;
   /* ET token flashes until user starts to enter a time */
   Flash_ET = TRUE;
   Flash_ET_time = 0.0;
   Stage_SET_RST = LONG_PRESS;

   /* Reset time to 0 */
   Elapsed_time = 0.0;

   Idle_set_mode_time = 0.0;

   /* Change standby mode to ET if not already */
   Stdby_mode = ET_DISPLAY;
   Stdby_time_mode = Stdby_mode;
   Adf_update_for_stdby_mode();

   /* Update display */
   IO_Sen_out->KR87_STBY_TIME_display = Set_time_display(Elapsed_time);
}

/*----------------------------------------------------------------------*/

static void Timeout_ET_set_mode()
{
   Mode_ET = ET_NORMAL_MODE;
   Flash_ET = FALSE;   /* Could already be FALSE! */
   Stage_SET_RST = NORMAL_USE;
   /* Reset time to 0 */
   Elapsed_time = 0.0;
   IO_Sen_out->KR87_STBY_TIME_display = Set_time_display(Elapsed_time);
   /* If we are in ET mode, make sure the flashing stops with
      the token on */
   if (Stdby_mode == ET_DISPLAY)
      IO_Sen_out->KR87_ET_token = TOKEN_ON;
}

/*----------------------------------------------------------------------*/

/* General updates to support the functions of the KR87 box. */

static void Adf_gen_update()
{
   if (!IO_avionic_pwr || IO_Sen_faults.AVF.adf)
      return;

   if (IO_Sen_in->KR87_off_on_switch == SW_OFF)
      return;

if(SE_freeze_on == FALSE && IO_iosfreeze == FALSE)
{
   /* Update flight timer. Also, update display if currently
      displaying flight time */

   Flt_timer += delta_time;
   if (Stdby_mode == FLT_DISPLAY)
      IO_Sen_out->KR87_STBY_TIME_display = Set_time_display(Flt_timer);
   

   if (Mode_ET == ET_NORMAL_MODE)
      Elapsed_time += delta_time;
   else if (Mode_ET == ET_COUNTDOWN_MODE)
   {
      Elapsed_time -= delta_time;
      if (Elapsed_time < 0.0)
      {
         Elapsed_time = -Elapsed_time;
         Mode_ET = ET_COUNTUP_MODE;
         Flash_ET_display = TRUE;        //Flash the stdby display whatever it is!
         Flash_ET_display_time = 0.0;
         if (Stdby_mode == FREQ_DISPLAY)
         {
            //Require FRQ display to show leading 0 in the flash mode
            IO_Sen_out->KR87_STBY_TIME_display_mode = KR87_STBY_LEADING_ZERO;
         }
      }
   }
   else if (Mode_ET == ET_COUNTUP_MODE)
   {
      Elapsed_time += delta_time;
      if (Elapsed_time > MAX_COUNTUP_TIME)
      {
         /* Change to normal ET mode */
         Flash_ET_display = FALSE;
         Mode_ET = ET_NORMAL_MODE;
         /* Make sure the flashing stops with mode active */
         IO_Sen_out->KR87_STBY_TIME_display_mode = ACTIVE;
      }
   }
   else if (Mode_ET == ET_SET_MODE)
   {
      Idle_set_mode_time += delta_time;

      /* If the user does not enter a time within 15 seconds,
         time resets to 0 and mode reverts to normal */
      if (Idle_set_mode_time >= MAX_IDLE_ET_SET_TIME)
      {
         Timeout_ET_set_mode();
      }
   }

   if (Stdby_mode == ET_DISPLAY)
      IO_Sen_out->KR87_STBY_TIME_display = Set_time_display(Elapsed_time);

   if (IO_Sen_in->KR87_SET_RST_button == PRESSED)
   {
      if (Last_KR87_SET_RST_button == PRESSED)   /* ie. Adf_button_changes has seen this */
      {
         Press_SET_RST_button_time += delta_time;

         if (Press_SET_RST_button_time > ENABLE_ET_SET_TIME)
         {
            /* If already in ET set mode, we still do this (unless
               we haven't yet released button from first entry
               to set mode) */
            if (Stage_SET_RST != LONG_PRESS)
               Init_ET_set_mode();
         }
      }
   }
}//End of freeze
   if (Flash_ET)
   {
      Flash_ET_time += delta_time;
      if (Flash_ET_time > FLASH_INTERVAL)
      {
         Flash_ET_time = 0.0;
         /* Toggle the ET token */
         if (IO_Sen_out->KR87_ET_token == TOKEN_OFF)
            IO_Sen_out->KR87_ET_token = TOKEN_ON;
         else
            IO_Sen_out->KR87_ET_token = TOKEN_OFF;
      }
   }
   if (Flash_ET_display)
   {
      Flash_ET_display_time += delta_time;
      if (Flash_ET_display_time > FLASH_INTERVAL)
      {
         Flash_ET_display_time = 0.0;
         /* Toggle the Standby display (whatever mode is being displayed) */
         if (Stdby_mode == FREQ_DISPLAY)
         {
            //Require FRQ display to show leading 0 in the flash mode
            if (IO_Sen_out->KR87_STBY_TIME_display_mode == BLANK)
               IO_Sen_out->KR87_STBY_TIME_display_mode = KR87_STBY_LEADING_ZERO;
            else
               IO_Sen_out->KR87_STBY_TIME_display_mode = BLANK;
         }
         else
         {
            if (IO_Sen_out->KR87_STBY_TIME_display_mode == BLANK)
               IO_Sen_out->KR87_STBY_TIME_display_mode = ACTIVE;
            else
               IO_Sen_out->KR87_STBY_TIME_display_mode = BLANK;
         }
      }
   }
}

/*----------------------------------------------------------------------*/

void ADF_master_power_off()
{
   if (!IO_avionic_pwr || IO_Sen_faults.AVF.adf)
   {
      /* Blank ADF box and reset timer */
      Adf_box_blank();
   }
}

/*----------------------------------------------------------------------*/

/* This should be called after Adf_button_changes, so that
   the turn dial counts are reset if power is restored. */

static void Adf_turn_dial_changes()
{
   if (!IO_avionic_pwr || IO_Sen_faults.AVF.adf)
      return;

   if (IO_Sen_in->KR87_off_on_switch == SW_OFF)
      return;

   if (Mode_ET == ET_SET_MODE)
      Time_update();
   else if (Stdby_mode == FREQ_DISPLAY)
      Stdby_freq_update();
   else
      Active_freq_update();
}

/*----------------------------------------------------------------------*/

void ADF_adf_update()
{
   Adf_gen_update();
   Adf_button_changes();
   Adf_turn_dial_changes();
}

/*----------------------------------------------------------------------*/

static void Adf_box_blank()
{
   IO_Sen_out->KR87_USE_display_mode = BLANK;
   IO_Sen_out->KR87_STBY_TIME_display_mode = BLANK;

   /* Turn off all tokens */
   IO_Sen_out->KR87_ANT_token   = TOKEN_OFF;
   IO_Sen_out->KR87_ADF_token   = TOKEN_OFF;
   IO_Sen_out->KR87_BFO_token   = TOKEN_OFF;
   IO_Sen_out->KR87_FRQ_token   = TOKEN_OFF;
   IO_Sen_out->KR87_FLT_token   = TOKEN_OFF;
   IO_Sen_out->KR87_ET_token    = TOKEN_OFF;
   IO_Sen_out->KR87_colon_token = TOKEN_OFF;
}

/*----------------------------------------------------------------------*/

/* Activate only those tokens and modes that are ONLY dependent
   on power. All others will be updated elsewhere */

static void Adf_box_active()
{
   IO_Sen_out->KR87_USE_display_mode = ACTIVE;
   IO_Sen_out->KR87_STBY_TIME_display_mode = ACTIVE;

   Adf_update_alt_but_tokens();
   Adf_update_for_stdby_mode();
}

/*----------------------------------------------------------------------*/

/* This sets up necessary variables to request new beacon
   search from the IOS.
   Note that the current beacon flag is not set to invalid here. SHOULD IT BE???? */

static void Tune_ndb_beacon()
{
   ADF_new_ndb_bfreq = TRUE;
   ADF_search_mode = TRUE;
}

/*----------------------------------------------------------------------*/

/* This routine is not essential since it does no harm to request
   info from the IOS even if it will not be used */

static void Tune_off_ndb_beacon()
{
   ADF_new_ndb_bfreq = FALSE;

   /* This is not really needed here since all tune off cases are
      handled where the valid_flag is used */
   IO_ndb_battr.valid_flag = FALSE;
}

/*----------------------------------------------------------------------*/

/* Each turn has its own wrap, so CANNOT just calculate an overall
   delta */

static void Get_dial_deltas(long *delta_turn1, long *delta_turn2, long *delta_turn3)
{
   long num_step1;  /* Current turn value */
   long num_step2;
   long num_step3;
   long ds1;        /* Change in turn value */
   long ds2;
   long ds3;

   /* Need to work out the delta turn of each knob
      from the last turn values stored. */

   num_step1 = IO_Sen_in->KR87_delta_outer_encoder;
   num_step2 = IO_Sen_in->KR87_delta_inner_encoder;
   num_step3 = IO_Sen_in->KR87_delta_inner_encoder_pull;

   ds1 = num_step1 - Last_turn1;
   ds2 = num_step2 - Last_turn2;
   ds3 = num_step3 - Last_turn3;

   *delta_turn1 = 0;
   *delta_turn2 = 0;
   *delta_turn3 = 0;

   /* Only do necessary calculations */

   if (ds1 != 0)
      *delta_turn1 = ds1;
   if (ds2 != 0)
      *delta_turn2 = ds2;
   if (ds3 != 0)
      *delta_turn3 = ds3;

   Last_turn1 = num_step1;
   Last_turn2 = num_step2;
   Last_turn3 = num_step3;

   return;
}

/*----------------------------------------------------------------------*/

/* Handles changes to data input knobs when updating frequency.
   Returns FALSE if no change made. The frequency display is
   in KHz with NO decimal, ie. no decimal part at all.
*/
   
static int Freq_update(long *freq)
{
   long delta_turn1;
   long delta_turn2;
   long delta_turn3;
   long step1 = 1;   /* 1 unit, ie. step1_value = 100, 100 KHz */
   long step2 = 1;   /* 1 unit, ie. step2_value = 10,  10  KHz */
   long step3 = 1;   /* 1 unit, ie. step3_value = 1,   1   KHz */
   long num_val1;
   long num_val2;
   long num_val3;
   long tmp;

   /* Outer knob updates in steps of 100
      Centre knob in updates in steps of 10
      Centre knob out updates in steps of 1 */

   Get_dial_deltas(&delta_turn1, &delta_turn2, &delta_turn3);

   if ((delta_turn1 == 0) && (delta_turn2 == 0) && (delta_turn3 == 0))
      return FALSE;

   num_val1 = *freq / 100;             /* The hundreds */
   tmp = *freq % 100;
   num_val2 = tmp / 10;                /* The tens */
   num_val3 = tmp % 10;                /* The units */

   num_val1 += (delta_turn1 * step1);
   while (num_val1 > MAX_100_NDB_FREQ)
      num_val1 = num_val1 - (MAX_100_NDB_FREQ + 1 - MIN_100_NDB_FREQ);
   while (num_val1 < MIN_100_NDB_FREQ)
      num_val1 = num_val1 + (MAX_100_NDB_FREQ + 1 - MIN_100_NDB_FREQ);

   num_val2 += (delta_turn2 * step2);
   while (num_val2 > 9)
      num_val2 = num_val2 - 10;
   while (num_val2 < 0)
      num_val2 = num_val2 + 10;

   num_val3 += (delta_turn3 * step3);
   while (num_val3 > 9)
      num_val3 = num_val3 - 10;
   while (num_val3 < 0)
      num_val3 = num_val3 + 10;

   *freq = (num_val1 * 100) + (num_val2 * 10) + (num_val3 * 1);

   return TRUE;
}

/*----------------------------------------------------------------------*/

static void Stdby_freq_update()
{
   if (!Freq_update(&Stdby_freq))
      return;

   /* This is only called if this is the currently displayed item,
      so update display value */
   IO_Sen_out->KR87_STBY_TIME_display = Stdby_freq;
}

/*----------------------------------------------------------------------*/

/* Handles changes to data input knobs when changing active frequency

   Inputs:  Last_turn1/2/3
            IO_Sen_out->KR87_USE_display

   Outputs: Last_turn1/2/3
            IO_Sen_out->KR87_USE_display
*/

static void Active_freq_update()
{
   long freq;

   freq = IO_Sen_out->KR87_USE_display;

   if (!Freq_update(&freq))
      return;

   IO_Sen_out->KR87_USE_display = freq;

   IO_ndb_battr.valid_flag = FALSE;
   Tune_ndb_beacon();
}

/*----------------------------------------------------------------------*/

/* Handles changes to data input knobs when in ET_SET_MODE
   
   Inputs:  Last_turn1/2/3
            Elaspsed_time

   Outputs: Last_turn1/2/3
            Elaspsed_time
            IO_Sen_out->KR87_STBY_TIME_display
*/
   
static void Time_update()
{
   long delta_turn1;
   long delta_turn2;
   long delta_turn3;
   long step1 = 1;   /* 1 unit,  ie. step1_value = 100, 1 min */
   long step2 = 1;   /* 1 unit,  ie. step2_value = 10,  10 seconds */
   long step3 = 1;   /* 1 units, ie. step3_value = 1,   1 second */
   long num_val1;
   long num_val2;
   long num_val3;
   long total_secs;
   long num_secs;

   Get_dial_deltas(&delta_turn1, &delta_turn2, &delta_turn3);

   if ((delta_turn1 == 0) && (delta_turn2 == 0) && (delta_turn3 == 0))
      return;

   /* Get current values of the separate components */
   total_secs = (long) Elapsed_time;
   num_val1 = total_secs / 60; /* The 1 mins */
   num_secs = total_secs % 60;
   num_val2 = num_secs / 10;   /* The 10 secs */
   num_val3 = num_secs % 10;   /* The units */

   num_val1 += (delta_turn1 * step1);
   while (num_val1 > MAX_MIN)
      num_val1 = num_val1 - (MAX_MIN + 1);
   while (num_val1 < 0)
      num_val1 = num_val1 + (MAX_MIN + 1);

   num_val2 += (delta_turn2 * step2);
   while (num_val2 > MAX_10_SEC)
      num_val2 = num_val2 - (MAX_10_SEC + 1);
   while (num_val2 < 0)
      num_val2 = num_val2 + (MAX_10_SEC + 1);

   num_val3 += (delta_turn3 * step3);
   while (num_val3 > MAX_1_SEC)
      num_val3 = num_val3 - (MAX_1_SEC + 1);
   while (num_val3 < 0)
      num_val3 = num_val3 + (MAX_1_SEC + 1);

   total_secs = (num_val1 * 60) + (num_val2 * 10) + (num_val3 * 1);

   Elapsed_time = (float) total_secs;

   /* This is only called if this is the currently displayed item,
      so update display value */
   IO_Sen_out->KR87_STBY_TIME_display = Set_time_display(Elapsed_time);


   /* If we are in ET set mode, the ET token stops flashing as
      soon as user starts to enter a time (or after an idle time,
      handled elsewhere) */
   if (Mode_ET == ET_SET_MODE)
   {
      Flash_ET = FALSE;
      Idle_set_mode_time = 0.0;
      if (Stdby_mode == ET_DISPLAY)
         IO_Sen_out->KR87_ET_token = TOKEN_ON;
   }
}

/*----------------------------------------------------------------------*/

/* This initializes all data that may be reset by data saved in a file
   (to simulate data kept after a power off) */

static void Init_kept_memory()
{
   IO_Sen_out->KR87_USE_display = DEFAULT_NDB_FREQ;

   Stdby_mode = FREQ_DISPLAY;
   Stdby_freq = DEFAULT_NDB_FREQ;
   Stdby_time_mode = FLT_DISPLAY;
}

/*----------------------------------------------------------------------*/

static void Read_kept_memory(char *filename)
{
   FILE            *stream;
   long            file_len;
   int             bytes_read;
   adf_kept_memory kmem;

   stream = fopen( filename, "rb" );
   if (!stream)
   {
      /* No error message */
      return;
   }

   fseek (stream, 0, SEEK_END);
   file_len = ftell (stream);
   if (file_len == 0)
   {
      /* File has zero length. No error message */
      fflush (stdout);
      fclose (stream);
      return;
   }
   /* Return stream pointer to start of file */
   fseek (stream, 0, SEEK_SET);

   if (file_len != sizeof(adf_kept_memory))
   {
      printf("\nError: File %s has unexpected size (%d instead of %d)", filename, file_len, sizeof(adf_kept_memory));
      fflush (stdout);
      return;
   }

   bytes_read = fread (&kmem, 1, file_len, stream);
   if (bytes_read != file_len)
   {
      printf("\nError in reading file %s (bytes_read = %d)", filename, bytes_read);
      fflush (stdout);
      fclose (stream);
      return;
   }

   fclose (stream);

   if (kmem.opcode != ADF_OPCODE)
   {
      printf("\nError: file %s is not an adf type file", filename);
      fflush (stdout);
      return;
   }

   IO_Sen_out->KR87_USE_display = kmem.use_freq;
   Stdby_mode = kmem.stdby_mode;
   Stdby_freq = kmem.stby_freq;
   Stdby_time_mode = kmem.stdby_time_mode;

   return;
}

/*----------------------------------------------------------------------*/

static void Write_kept_memory(char *filename)
{
   FILE            *stream;
   adf_kept_memory kmem;
   int             num_bytes;

   kmem.opcode       = ADF_OPCODE;
   kmem.stdby_mode   = Stdby_mode;
   kmem.stdby_time_mode = Stdby_time_mode;
   kmem.spare        = 0;
   kmem.use_freq     = IO_Sen_out->KR87_USE_display;
   kmem.stby_freq    = Stdby_freq;

   stream = fopen( filename, "wb" );

   if (!stream)
   {
      printf("\nError in opening file %s for write", filename);
      fflush (stdout);
      return;
   }

   num_bytes = fwrite (&kmem, 1, sizeof(adf_kept_memory), stream);

   fclose (stream);

   return;
}

/*----------------------------------------------------------------------*/

void ADF_exit()
{
   Write_kept_memory("adf.cfg");
}

/*----------------------------------------------------------------------*/
