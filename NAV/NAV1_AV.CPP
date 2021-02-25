/*
File  : nav1_av.c
Prefix: N1A_

Module: Dynamics for Seneca
Created: January 2000

Description: This file handles user input on the KNS80 avionics box
*/

/***************************/
/*   RCS Markers           */
/***************************/

/*
$Header: C:/Master/seneca/NAV/RCS/nav1_av.cpp 1.8 2000/06/16 10:54:10 ColinJ Exp $
$Log: nav1_av.cpp $
Revision 1.8  2000/06/16 10:54:10  ColinJ
new function N1A_ils_freq_num

Revision 1.7  2000/04/27 16:59:03  juliae
ILS token now controlled by frequency value and not
type of signal received.

Revision 1.6  2000/04/25 16:16:47  juliae
Increase amount of data saved to file. Changed file name
to nav1.cfg.

Revision 1.5  2000/04/18 15:22:37  juliae
Correction: Initialize N1A_dme_hold
Added support for requesting beacon info constantly.

Revision 1.4  2000/04/17 11:47:01  juliae
Correction to USE_display_mode.

Revision 1.3  2000/03/17 16:26:04  juliae
Change IO_elec_pwr to IO_avionic_pwr. Set KNS80_FRQ_RAD_DST_display_mode
according to type of data being displayed. Allow for nav1 fault.

 * Revision 1.2  2000/03/07  11:41:49  juliae
 * Added save and restore of data to file.
 *
 * Revision 1.1  2000/03/03  09:01:08  juliae
 * Initial revision
 *
*/

/*----------------------------------------------------------------------*/

/*----------------*/
/* Include Files  */
/*----------------*/

#include <stdio.h>
#include <math.h>
#include <string.h>

#include "c:\bae-sims\seneca\nav/beacon.h"
#include "c:\bae-sims\seneca\nav/nav1.h"
#include "c:\bae-sims\seneca\dat_stru/out_head.h"
#include "c:\bae-sims\seneca\dat_stru/in_head.h"
#include "c:\bae-sims\seneca\define.h"
#include "c:\bae-sims\seneca\faults.h"

/*----------------*/
/* Defines        */
/*----------------*/

#define DEFAULT_NAV1_FREQ 11000
#define MIN_NAV1_FREQ     108
#define MAX_NAV1_FREQ     117
#define MAX_RAD_10_DIST    20     //200 nm

#define MIN_ILS_FREQ      10800
#define MAX_ILS_FREQ      11200

#define MAX_NUM_STORAGE_BIN 4
#define FREQ_DISPLAY    0
#define WP_RAD_DISPLAY  1
#define WP_DIST_DISPLAY 2

#define TOKEN_ON   LIT
#define TOKEN_OFF  UNLIT

#define FLASH_INTERVAL 0.5       /* Seconds */

#define NAV1_OPCODE 1
#define AUTO_DME_TUNE 1

/*----------------*/
/* Typedefs       */
/*----------------*/

typedef struct
{
   long freq;
   long radial;    /* Units: 0.01 degress */
   long distance;  /* Units: 0.01 nm */
   float dist_sq;  /* Units: nm_sq. We store this INTERNAL use only */

} n1_store_bin;

typedef struct
{
   unsigned char opcode;
   unsigned char num_bins;
   unsigned char data_display;
   unsigned char use_display;
   unsigned char dsp_display;
   unsigned char nav1_mode;
   unsigned char dme_hold;
   char          spare;
   long          dme_hold_freq;
   n1_store_bin  bin[MAX_NUM_STORAGE_BIN];

} n1_kept_memory;

/*--------------------*/
/* External Routines  */
/*--------------------*/

extern void BC_reset_KT_MIN_calc();

/*--------------------*/
/* External Variables */
/*--------------------*/

extern sen_out_struct *IO_Sen_out;
extern sen_in_struct  *IO_Sen_in;
extern OUTPUTin			*OUTPUTinbox;//simulinkudp
extern sen_fault_struct IO_Sen_faults;
extern vhf_beac_attr  IO_nav1_battr;
extern vhf_beac_attr  IO_dme_battr;
extern int            IO_avionic_pwr;
extern float          delta_time;

/*----------------*/
/* Globals        */
/*----------------*/

int    N1A_dme_hold = FALSE;
int    N1A_nav1_mode = NAV1_VOR;
/* This is used to say if new beacon info is wanted from IOS */
int    N1A_new_nav1_bfreq = FALSE;
double N1A_nav1_search_time = 0.0;
int    N1A_search_mode = FALSE;
int    N1A_new_dme_bfreq = FALSE;
double N1A_dme_search_time = 0.0;
int    N1A_dme_search_mode = FALSE;

/*-----------------*/
/* Local Variables */    
/*-----------------*/

/* Flags for momentary button changes */
static int Last_KNS80_VOR_button;
static int Last_KNS80_RNV_button;
static int Last_KNS80_USE_button;
static int Last_KNS80_DSP_button;
static int Last_KNS80_DATA_button;

static int Data_display = FREQ_DISPLAY;
static n1_store_bin Store_bin[MAX_NUM_STORAGE_BIN];
static int Flash_dsp = FALSE;
static float Flash_dsp_time = 0.0;
static long Hold_freq;

/* For KNS80 turn dial counts */
static long Last_turn1 = 0;
static long Last_turn2 = 0;
static long Last_turn3 = 0;

static int Last_pwr_on = FALSE;

/*--------------------*/
/* Forward References */
/*--------------------*/

static void Nav1_box_active();
static void Nav1_box_blank();
static void Nav1_update_for_mode();
static void Nav1_update_for_hold();
static void Nav1_update_for_freq_type();
static void Nav1_update_for_freq_num();
static void Nav1_update_for_data_disp();
static void Freq_update();
static void Wp_radial_update();
static void Wp_distance_update();
static void Tune_nav1_beacon();
static void Tune_dme_beacon();
static void Tune_off_nav1_beacon();
static void Tune_off_dme_beacon();
static void Reset_turn_counts();
static void Init_kept_memory();
static void Read_kept_memory(char *filename);

long N1A_get_active_n1_freq();
int N1A_ils_freq_num(long freq);
/*----------------------------------------------------------------------*/

void N1A_nav1_init()
{
   IO_nav1_battr.valid_flag = FALSE;
   IO_dme_battr.valid_flag = FALSE;

   Last_pwr_on = FALSE;
   if (IO_avionic_pwr && (IO_Sen_in->KNS80_off_on_switch == SW_ON) && !IO_Sen_faults.AVF.nav1)
      Last_pwr_on = TRUE;

   /* Flags for momentary button changes */
   Last_KNS80_VOR_button  = IO_Sen_in->KNS80_VOR_button;
   Last_KNS80_RNV_button  = IO_Sen_in->KNS80_RNAV_button;
   Last_KNS80_USE_button  = IO_Sen_in->KNS80_USE_button;
   Last_KNS80_DSP_button  = IO_Sen_in->KNS80_DSP_button;
   Last_KNS80_DATA_button = IO_Sen_in->KNS80_DATA_button;

   /* For KNS80 turn dial counts */
   Reset_turn_counts();

   BC_reset_KT_MIN_calc();

   Flash_dsp = FALSE;
   Flash_dsp_time = 0.0;

   Init_kept_memory();
   Read_kept_memory("nav1.cfg");

   if (!Last_pwr_on)
   {
      Nav1_box_blank();
      Tune_off_nav1_beacon();
      Tune_off_dme_beacon();
   }
   else
   {
      Nav1_box_active();
      Tune_nav1_beacon();
#ifdef AUTO_DME_TUNE
      if (N1A_dme_hold)
      {
         /* Cannot just use value saved in file because this also depends
            on whether HLD button is still pressed.
            If the HLD button has since been released, this will get handled
            in Nav1_button_changes */
         /* This does not really correspond to what happened in practise! */
         Tune_dme_beacon();
      }
#endif
   }
}

/*----------------------------------------------------------------------*/

/* This performs all the initialization required when power is restored
   to adf box. See also N1A_nav1_init. Do not initialize anything here
   that is included in "kept memory" */

static void Nav1_pwr_on()
{
   /* Flags for momentary button changes */
   Last_KNS80_VOR_button  = IO_Sen_in->KNS80_VOR_button;
   Last_KNS80_RNV_button  = IO_Sen_in->KNS80_RNAV_button;
   Last_KNS80_USE_button  = IO_Sen_in->KNS80_USE_button;
   Last_KNS80_DSP_button  = IO_Sen_in->KNS80_DSP_button;
   Last_KNS80_DATA_button = IO_Sen_in->KNS80_DATA_button;

   /* For KNS80 turn dial counts */
   Reset_turn_counts();

   BC_reset_KT_MIN_calc();

   Nav1_box_active();

   IO_nav1_battr.valid_flag = FALSE;
   IO_dme_battr.valid_flag = FALSE;

   Tune_nav1_beacon();
#ifdef AUTO_DME_TUNE
   if (N1A_dme_hold)
   {
      /* This does not really correspond to what happened in practise! */
      Tune_dme_beacon();
   }
#endif
}

/*----------------------------------------------------------------------*/

static void Reset_turn_counts()
{
   /* For KNS80 turn dial counts */
   Last_turn1 = IO_Sen_in->KNS80_delta_outer_encoder;
   Last_turn2 = IO_Sen_in->KNS80_delta_inner_encoder;
   Last_turn3 = IO_Sen_in->KNS80_delta_inner_encoder_pull;
}

/*----------------------------------------------------------------------*/

static void Init_dme_hold()
{
   N1A_dme_hold = TRUE;

   /* Save current active frequency */
   Hold_freq = N1A_get_active_n1_freq();
   if (IO_nav1_battr.valid_flag)
   {
      /* We don't need to get the dme data from the IOS since
         we already have it (this is the usual case) */
      memcpy(&IO_dme_battr, &IO_nav1_battr, sizeof(vhf_beac_attr));

      Tune_dme_beacon();   //To support GET_BEAC_EVERY_TIME
   }
   else
   {
      IO_dme_battr.valid_flag = FALSE;
      Tune_dme_beacon();
   }
}

/*----------------------------------------------------------------------*/

static void Nav1_button_changes()
{
   int old_hold;

   if (!IO_avionic_pwr || IO_Sen_faults.AVF.nav1)
   {
      if (Last_pwr_on)
      {
         Nav1_box_blank();
         Tune_off_nav1_beacon();
         Tune_off_dme_beacon();
      }
      Last_pwr_on = FALSE;
      return;
   }

   if (IO_Sen_in->KNS80_off_on_switch == SW_OFF)
   {
      if (Last_pwr_on)
      {
         /* Blank displays */
         Nav1_box_blank();
         Tune_off_nav1_beacon();
         Tune_off_dme_beacon();
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
            been turned on with local switch still on). */
         Nav1_pwr_on();
      }
      Last_pwr_on = TRUE;
   }

   /* Test for change to momentary VOR pushbutton */
   if ((Last_KNS80_VOR_button == PRESSED) &&
       (IO_Sen_in->KNS80_VOR_button == NOT_PRESSED))
   {
      /* This button cycles between VOR and VOR/PAR.
         If the RNAV button was pressed last time, this
         restarts at the VOR mode */
      if (N1A_nav1_mode == NAV1_VOR)
         N1A_nav1_mode = NAV1_VOR_PAR;
      else
         N1A_nav1_mode = NAV1_VOR;

      Nav1_update_for_mode();
      BC_reset_KT_MIN_calc();
   }

   Last_KNS80_VOR_button = IO_Sen_in->KNS80_VOR_button;

   /* Test for change to momentary RNAV pushbutton */
   if ((Last_KNS80_RNV_button == PRESSED) &&
       (IO_Sen_in->KNS80_RNAV_button == NOT_PRESSED))
   {
      /* This button cycles between RNV and RNAV/APR.
         If the VOR button was pressed last time, this
         restarts at the RNV mode */
      if (N1A_nav1_mode == NAV1_RNV)
         N1A_nav1_mode = NAV1_RNV_APR;
      else
         N1A_nav1_mode = NAV1_RNV;

      Nav1_update_for_mode();
      BC_reset_KT_MIN_calc();
   }

   Last_KNS80_RNV_button = IO_Sen_in->KNS80_RNAV_button;

   old_hold = N1A_dme_hold;
   if (IO_Sen_in->KNS80_HOLD_button == PRESSED)
   {
      if (!N1A_dme_hold)
      {
         /* The current active frequency is used from now on
            for dme beacon information. It makes NO difference
            whether this frequency has valid dme info or not: the
            aircraft scans for it because it may receive a valid
            signal when the aircraft moves, even if current signal
            is invalid. (It does not have the advantage of
            Jeppeson data to know if its search will always fail!). */

         Init_dme_hold();
      }
   }
   else
   {
      if (N1A_dme_hold)
      {
         N1A_dme_hold = FALSE;
         Tune_off_dme_beacon();
      }
   }

   if (old_hold != N1A_dme_hold)
      Nav1_update_for_hold();

   /* Test for change to momentary USE pushbutton */
   if ((Last_KNS80_USE_button == PRESSED) &&
       (IO_Sen_in->KNS80_USE_button == NOT_PRESSED))
   {
      /* Make the display data storage bin the use one */
      /* If this means a change to active frequency, set
         the current beacon data to invalid and set flag
         to get new info from IOS */

      if (IO_Sen_out->KNS80_USE_display != IO_Sen_out->KNS80_DSP_display)
      {
         IO_Sen_out->KNS80_USE_display = IO_Sen_out->KNS80_DSP_display;
         Nav1_update_for_hold();
         IO_nav1_battr.valid_flag = FALSE;
         Tune_nav1_beacon();
         BC_reset_KT_MIN_calc();
         Nav1_update_for_freq_num();

         /* Data display goes to FREQ */
         Data_display = FREQ_DISPLAY;
         Nav1_update_for_data_disp();
      }
      Flash_dsp = FALSE;
      /* Make sure the flashing stops with the mode active */
      IO_Sen_out->KNS80_DSP_display_mode = ACTIVE;
   }

   Last_KNS80_USE_button = IO_Sen_in->KNS80_USE_button;

   /* Test for change to momentary DSP pushbutton.
      This cycles displaying info stored in 4 storage bins */
   if ((Last_KNS80_DSP_button == PRESSED) &&
       (IO_Sen_in->KNS80_DSP_button == NOT_PRESSED))
   {
      IO_Sen_out->KNS80_DSP_display++;
      if (IO_Sen_out->KNS80_DSP_display > MAX_NUM_STORAGE_BIN)
         IO_Sen_out->KNS80_DSP_display = 1;

      if (IO_Sen_out->KNS80_DSP_display == IO_Sen_out->KNS80_USE_display)
      {
         Flash_dsp = FALSE;
         /* Make sure the flashing stops with the mode active */
         IO_Sen_out->KNS80_DSP_display_mode = ACTIVE;
      }
      else
      {
         Flash_dsp = TRUE;
         Flash_dsp_time = 0.0;
      }
      /* Data display goes to FREQ */
      Data_display = FREQ_DISPLAY;

      Nav1_update_for_data_disp();
   }

   Last_KNS80_DSP_button = IO_Sen_in->KNS80_DSP_button;

   /* Test for change to momentary DATA pushbutton.
      This cycles displaying freq, wp radial, wp distance */
   if ((Last_KNS80_DATA_button == PRESSED) &&
       (IO_Sen_in->KNS80_DATA_button == NOT_PRESSED))
   {
      Data_display++;
      if (Data_display > WP_DIST_DISPLAY)
         Data_display = FREQ_DISPLAY;

      Nav1_update_for_data_disp();
   }

   Last_KNS80_DATA_button = IO_Sen_in->KNS80_DATA_button;

}

/*----------------------------------------------------------------------*/

/* General updates to support the functions of the KNS80 box. */

static void Nav1_gen_update()
{
   if (!IO_avionic_pwr || IO_Sen_faults.AVF.nav1)
      return;

   if (IO_Sen_in->KNS80_off_on_switch == SW_OFF)
      return;

   if (Flash_dsp)
   {
      Flash_dsp_time += delta_time;
      if (Flash_dsp_time > FLASH_INTERVAL)
      {
         Flash_dsp_time = 0.0;
         /* Toggle the DSP value on and off */
         if (IO_Sen_out->KNS80_DSP_display_mode == ACTIVE)
            IO_Sen_out->KNS80_DSP_display_mode = BLANK;
         else
            IO_Sen_out->KNS80_DSP_display_mode = ACTIVE;
      }
   }
}

/*----------------------------------------------------------------------*/

/* This should be called after Nav1_button_changes, so that
   the turn dial counts are reset if power is restored. */

static void Nav1_turn_dial_changes()
{
   if (!IO_avionic_pwr || IO_Sen_faults.AVF.nav1)
      return;

   if (IO_Sen_in->KNS80_off_on_switch == SW_OFF)
      return;

   switch (Data_display)
   {
      case FREQ_DISPLAY:
         Freq_update();
         break;
      case WP_RAD_DISPLAY:
         Wp_radial_update();
         break;
      case WP_DIST_DISPLAY:
         Wp_distance_update();
         break;
      default:
         break;
   }
}

/*----------------------------------------------------------------------*/

void N1A_nav1_update()
{
   Nav1_gen_update();
   Nav1_button_changes();
   Nav1_turn_dial_changes();
}

/*----------------------------------------------------------------------*/

long N1A_get_active_n1_freq()
{
   int index;

   index = IO_Sen_out->KNS80_USE_display - 1;
   if (index < 0)
      index = 0;
   
   return ( Store_bin[index].freq );
}

/*----------------------------------------------------------------------*/

void N1A_set_active_n1_freq( long freq )
{
   int index;

   index = IO_Sen_out->KNS80_USE_display - 1;
   if (index < 0)
      index = 0;
   
   Store_bin[index].freq = freq;
}

/*----------------------------------------------------------------------*/

long N1A_get_dme_hold_freq()
{
   return Hold_freq;
}

/*----------------------------------------------------------------------*/

/* Returns a value in nautical miles */

float N1A_nav1_get_active_wp_dist()
{
   int   index;
   float scaled_nm;

   index = IO_Sen_out->KNS80_USE_display - 1;
   if (index < 0)
      index = 0;

   /* Value in Store_bin is in units of 0.01 nm */
   scaled_nm = (float) Store_bin[index].distance;

   return (scaled_nm / 100.0f);
}

/*----------------------------------------------------------------------*/

float N1A_nav1_get_active_wp_dist_sq()
{
   int index;

   index = IO_Sen_out->KNS80_USE_display - 1;
   if (index < 0)
      index = 0;

   return ( Store_bin[index].dist_sq );
}

/*----------------------------------------------------------------------*/

/* Returns a value in degrees */

float N1A_nav1_get_active_wp_radial()
{
   int index;
   float scaled_deg;

   index = IO_Sen_out->KNS80_USE_display - 1;
   if (index < 0)
      index = 0;

   scaled_deg = (float) Store_bin[index].radial;

   return (scaled_deg / 100.0f);
}

/*----------------------------------------------------------------------*/

/* Called when a new beacon has been received */

void N1A_refresh_beacon()
{
   if (Last_pwr_on)
   {
      Nav1_update_for_freq_type();
   }
}

/*----------------------------------------------------------------------*/

/* This is used to update the KNS80 values to correspond to N1A_nav1_mode.
   NB. If the beacon is an ILS type, the ILS token is shown IN ADDITION,
   not instead of the others. The ILS token needs to be updated
   whenever the frequency changes.

   Inputs:  N1A_nav1_mode
   Outputs: IO_Sen_out for KNS80 box
*/

static void Nav1_update_for_mode()
{
   switch (N1A_nav1_mode)
   {
      case NAV1_VOR_PAR:
         IO_Sen_out->KNS80_VOR_token = TOKEN_ON;
         IO_Sen_out->KNS80_PAR_token = TOKEN_ON;
         IO_Sen_out->KNS80_ENR_token = TOKEN_OFF;
         IO_Sen_out->KNS80_APR_token = TOKEN_OFF;
         IO_Sen_out->KNS80_RNV_token = TOKEN_OFF;
         break;
      case NAV1_RNV:
         IO_Sen_out->KNS80_VOR_token = TOKEN_OFF;
         IO_Sen_out->KNS80_PAR_token = TOKEN_OFF;
         IO_Sen_out->KNS80_ENR_token = TOKEN_ON;
         IO_Sen_out->KNS80_RNV_token = TOKEN_ON;
         IO_Sen_out->KNS80_APR_token = TOKEN_OFF;
         break;
      case NAV1_RNV_APR:
         IO_Sen_out->KNS80_VOR_token = TOKEN_OFF;
         IO_Sen_out->KNS80_PAR_token = TOKEN_OFF;
         IO_Sen_out->KNS80_ENR_token = TOKEN_OFF;
         IO_Sen_out->KNS80_RNV_token = TOKEN_ON;
         IO_Sen_out->KNS80_APR_token = TOKEN_ON;
         break;
      case NAV1_VOR:
      default:
         IO_Sen_out->KNS80_VOR_token = TOKEN_ON;
         IO_Sen_out->KNS80_PAR_token = TOKEN_OFF;
         IO_Sen_out->KNS80_ENR_token = TOKEN_OFF;
         IO_Sen_out->KNS80_RNV_token = TOKEN_OFF;
         IO_Sen_out->KNS80_APR_token = TOKEN_OFF;
         break;
   }
}

/*----------------------------------------------------------------------*/

/* The HLD light is on if the dme is holding to a different frequency
   to the NAV1 receiver */

static void Nav1_update_for_hold()
{
   if (N1A_dme_hold && (Hold_freq != N1A_get_active_n1_freq()))
      IO_Sen_out->KNS80_HLD_token = TOKEN_ON;
   else
      IO_Sen_out->KNS80_HLD_token = TOKEN_OFF;
}

/*----------------------------------------------------------------------*/

static int Ils_beacon(vhf_beac_attr *battr)
{
   if (battr->valid_flag &&
       battr->valid_vor_ils &&
       (battr->type == ILS_TYPE))
      return TRUE;
   else
      return FALSE;
}

/*----------------------------------------------------------------------*/

static void Nav1_update_for_freq_type()
{
   /* The ILS token does NOT depend on whether an ILS type beacon signal
      is received but purely on whether the frequency number corresponds
      to an ILS allocated one (108 to 112 at odd first decimal, eg.
      108.10, 108.35, etc */

#ifdef OLD_ILS_TOKEN
   IO_Sen_out->KNS80_ILS_token = TOKEN_OFF;

   if (IO_nav1_battr.valid_flag)
   {
      if ((IO_nav1_battr.valid_vor_ils) && (IO_nav1_battr.type == ILS_TYPE))
         IO_Sen_out->KNS80_ILS_token = TOKEN_ON;
   }
#endif

   return;
}

/*----------------------------------------------------------------------*/

static void Nav1_update_for_freq_num()
{
   long freq;
   int  first_dec;

   /* The ILS token does NOT depend on whether an ILS type beacon signal
      is received but purely on whether the frequency number corresponds
      to an ILS allocated one (108 to 112 at odd first decimal, eg.
      108.10, 108.35, etc */

   IO_Sen_out->KNS80_ILS_token = TOKEN_OFF;

   freq = N1A_get_active_n1_freq();
   if (freq >= MAX_ILS_FREQ)
      return;
   if (freq <= MIN_ILS_FREQ)
      return;
   first_dec = (freq % 100) / 10;
   if (first_dec % 2)   //Odd
      IO_Sen_out->KNS80_ILS_token = TOKEN_ON;
}

/*----------------------------------------------------------------------*/

/* This is used to update the KNS80 values to correspond to Data_display
   value and N1A_display index. Update the type of data being displayed
   in the DSP and USE data fields.
   
   Inputs:  Data_display
            
   Outputs: IO_Sen_out for KNS80 box
*/

static void Nav1_update_for_data_disp()
{
   int index;

   index = IO_Sen_out->KNS80_DSP_display - 1;
   if (index < 0)
      index = 0;

   switch (Data_display)
   {
      case FREQ_DISPLAY:
         IO_Sen_out->KNS80_FREQ_RAD_DST_display = Store_bin[index].freq;
         IO_Sen_out->KNS80_FREQ_token = TOKEN_ON;
         IO_Sen_out->KNS80_RAD_token = TOKEN_OFF;
         IO_Sen_out->KNS80_DST_token = TOKEN_OFF;
         IO_Sen_out->KNS80_FRQ_RAD_DST_display_mode = KNS80_FRQ;
         break;
      case WP_RAD_DISPLAY:
         IO_Sen_out->KNS80_FREQ_RAD_DST_display = Store_bin[index].radial;
         IO_Sen_out->KNS80_FREQ_token = TOKEN_OFF;
         IO_Sen_out->KNS80_RAD_token = TOKEN_ON;
         IO_Sen_out->KNS80_DST_token = TOKEN_OFF;
         IO_Sen_out->KNS80_FRQ_RAD_DST_display_mode = KNS80_RAD_DST;
         break;
      case WP_DIST_DISPLAY:
         IO_Sen_out->KNS80_FREQ_RAD_DST_display = Store_bin[index].distance;
         IO_Sen_out->KNS80_FREQ_token = TOKEN_OFF;
         IO_Sen_out->KNS80_RAD_token = TOKEN_OFF;
         IO_Sen_out->KNS80_DST_token = TOKEN_ON;
         IO_Sen_out->KNS80_FRQ_RAD_DST_display_mode = KNS80_RAD_DST;
         break;
      default:
         break;
   }
}

/*----------------------------------------------------------------------*/

static void Nav1_box_blank()
{
   IO_Sen_out->KNS80_USE_display_mode         = BLANK;
   IO_Sen_out->KNS80_DSP_display_mode         = KNS80_DSP_BLANK;
   IO_Sen_out->KNS80_NM_display_mode          = BLANK;
   IO_Sen_out->KNS80_KT_display_mode          = BLANK;
   IO_Sen_out->KNS80_MIN_display_mode         = BLANK;
   IO_Sen_out->KNS80_FRQ_RAD_DST_display_mode = BLANK;

   /* Turn off all tokens */
   IO_Sen_out->KNS80_NM_token   = TOKEN_OFF;
   IO_Sen_out->KNS80_KT_token   = TOKEN_OFF;
   IO_Sen_out->KNS80_MIN_token  = TOKEN_OFF;
   IO_Sen_out->KNS80_FREQ_token = TOKEN_OFF;
   IO_Sen_out->KNS80_RAD_token  = TOKEN_OFF;
   IO_Sen_out->KNS80_DST_token  = TOKEN_OFF;
   IO_Sen_out->KNS80_PAR_token  = TOKEN_OFF;
   IO_Sen_out->KNS80_VOR_token  = TOKEN_OFF;
   IO_Sen_out->KNS80_ENR_token  = TOKEN_OFF;
   IO_Sen_out->KNS80_APR_token  = TOKEN_OFF;
   IO_Sen_out->KNS80_RNV_token  = TOKEN_OFF;
   IO_Sen_out->KNS80_HLD_token  = TOKEN_OFF;
   IO_Sen_out->KNS80_ILS_token  = TOKEN_OFF;
   IO_Sen_out->KNS80_USE_token  = TOKEN_OFF;
   IO_Sen_out->KNS80_DSP_token  = TOKEN_OFF;
}

/*----------------------------------------------------------------------*/

/* Activate only those tokens and modes that are ONLY dependent
   on power. All others will be updated elsewhere */

static void Nav1_box_active()
{
   IO_Sen_out->KNS80_USE_display_mode = ACTIVE;
   IO_Sen_out->KNS80_DSP_display_mode = ACTIVE;
   IO_Sen_out->KNS80_USE_token = TOKEN_ON;
   IO_Sen_out->KNS80_DSP_token = TOKEN_ON;

   /* This is changed to KNS80_FRQ or KNS80_RAD_DST depending
      on type of data being displayed */
   IO_Sen_out->KNS80_FRQ_RAD_DST_display_mode = ACTIVE;

   IO_Sen_out->KNS80_NM_token  = TOKEN_ON;
   IO_Sen_out->KNS80_KT_token  = TOKEN_ON;
   IO_Sen_out->KNS80_MIN_token = TOKEN_ON;
   /* The display modes for dme related info depends
      on whether the dme is valid, so is not changed
      here */

   Nav1_update_for_mode();
   Nav1_update_for_data_disp();
   Nav1_update_for_hold();
   Nav1_update_for_freq_type();
   Nav1_update_for_freq_num();

   if (IO_Sen_out->KNS80_DSP_display != IO_Sen_out->KNS80_USE_display)
   {
      Flash_dsp = TRUE;
      Flash_dsp_time = 0.0;
   }
   else
   {
      Flash_dsp = FALSE;
   }
}

/*----------------------------------------------------------------------*/

/* This sets up necessary variables to request new beacon
   search from the IOS.
   Note that the current beacon flag is NOT set to invalid here. SHOULD IT BE???? */

static void Tune_nav1_beacon()
{
   N1A_new_nav1_bfreq = TRUE;
   N1A_nav1_search_time = 0.0;
   N1A_search_mode = TRUE;

#ifdef OLD_ILS_TOKEN
   /* Turn ILS token off */
   IO_Sen_out->KNS80_ILS_token = TOKEN_OFF;
#endif
}

/*----------------------------------------------------------------------*/

/* This routine is not essential since it does no harm to request
   info from the IOS even if it will not be used */

static void Tune_off_nav1_beacon()
{
   N1A_new_nav1_bfreq = FALSE;

   /* This is not really needed here since all tune off cases are
      handled where the valid_flag is used */
   IO_nav1_battr.valid_flag = FALSE;
}

/*----------------------------------------------------------------------*/

/* This sets up necessary variables to request new dme beacon
   search from the IOS. This only occurs when using DME hold
   and is only used as a safeguard against the active freq
   info from the IOS not having been received before HOLD
   was pressed. */

static void Tune_dme_beacon()
{
   N1A_new_dme_bfreq = TRUE;
   N1A_dme_search_time = 0.0;
   N1A_dme_search_mode = TRUE;
}

/*----------------------------------------------------------------------*/

/* This routine is not essential since it does no harm to request
   info from the IOS even if it will not be used */

static void Tune_off_dme_beacon()
{
   N1A_new_dme_bfreq = FALSE;

   /* This is not really needed here since all tune off cases are
      handled where the valid_flag is used */
   IO_dme_battr.valid_flag = FALSE;
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

   num_step1 = IO_Sen_in->KNS80_delta_outer_encoder;
   num_step2 = IO_Sen_in->KNS80_delta_inner_encoder;
   num_step3 = IO_Sen_in->KNS80_delta_inner_encoder_pull;

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

/* Handles changes to data input knobs when in FREQ_DISPLAY 
   Data_display mode. If the USE is the same as DSP, the
   new value gets used immediately, so need to make current
   values invalid until new data has been received from the
   IOS.
   We refer to the value without the decimal point, so hundreds
   is actually 1.00 MHz
   
   Inputs:  KNS80_DSP_display
            KNS80_USE_display
            Store_bin[].freq

   Outputs: Store_bin[].freq
            KNS80_FREQ_RAD_DST_display
            N1A_new_nav1_bfreq
            N1A_nav1_search_start_time;
            N1A_search_mode;
*/
   
static void Freq_update()
{
   int  index;
   long delta_turn1;
   long delta_turn2;
   long delta_turn3;
   long step1 = 1;   /* 1 unit,  ie. step1_value = 100, 1 MHz */
   long step2 = 1;   /* 1 unit,  ie. step2_value = 10,  0.1 MHz */
   long step3 = 5;   /* 5 units, ie. step3_value = 5,   0.05 MHz */
   long num_val1;
   long num_val2;
   long num_val3;
   long tmp;

   /* Outer knob updates in steps of 1 MHz
      Centre knob in updates in steps of 0.1 MHz
      Centre knob out updates in steps of 0.05 MHz */

   Get_dial_deltas(&delta_turn1, &delta_turn2, &delta_turn3);

   if ((delta_turn1 == 0) && (delta_turn2 == 0) && (delta_turn3 == 0))
      return;

   index = IO_Sen_out->KNS80_DSP_display - 1;

   num_val1 = Store_bin[index].freq / 100;   /* The hundreds */
   tmp = (Store_bin[index].freq - (num_val1 * 100));
   num_val2 = tmp / 10;                      /* The tens */
   num_val3 = tmp - (num_val2 * 10);         /* The units */

   num_val1 += (delta_turn1 * step1);
   while (num_val1 > MAX_NAV1_FREQ)
      num_val1 = num_val1 - (MAX_NAV1_FREQ + 1 - MIN_NAV1_FREQ);
   while (num_val1 < MIN_NAV1_FREQ)
      num_val1 = num_val1 + (MAX_NAV1_FREQ + 1 - MIN_NAV1_FREQ);

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

   Store_bin[index].freq = (num_val1 * 100) + (num_val2 * 10) + (num_val3 * 1);

   /* This is only called if this is the currently displayed item,
      so update display value */
   IO_Sen_out->KNS80_FREQ_RAD_DST_display = Store_bin[index].freq;


   if (IO_Sen_out->KNS80_DSP_display == IO_Sen_out->KNS80_USE_display)
   {
      IO_nav1_battr.valid_flag = FALSE;
      Tune_nav1_beacon();
      BC_reset_KT_MIN_calc();
      Nav1_update_for_hold();
      Nav1_update_for_freq_num();
   }
}

/*----------------------------------------------------------------------*/

/* Handles changes to data input knobs when in WP_RAD_DISPLAY 
   Data_display mode
   
   Inputs:  KNS80_DSP_display
            Store_bin[].radial

   Outputs: Store_bin[].radial
            KNS80_FREQ_RAD_DST_display
*/
   
static void Wp_radial_update()
{
   int  index;
   long delta_turn1;
   long delta_turn2;
   long delta_turn3;
   long step1 = 1;   /* 1 unit, ie. step1_value = 1000, 10 deg */
   long step2 = 1;   /* 1 unit, ie. step2_value = 100,  1 deg */
   long step3 = 1;   /* 1 unit, ie. step3_value = 10,   0.1 deg */
   long num_val1;
   long num_val2;
   long num_val3;
   long tmp;

   /* Outer knob updates in steps of 10 deg
      Centre knob in updates in steps of 1 deg
      Centre knob out updates in steps of 0.1 deg */

   Get_dial_deltas(&delta_turn1, &delta_turn2, &delta_turn3);

   if ((delta_turn1 == 0) && (delta_turn2 == 0) && (delta_turn3 == 0))
      return;

   index = IO_Sen_out->KNS80_DSP_display - 1;

   num_val1 = Store_bin[index].radial / 1000;   /* The 10 deg */
   tmp = (Store_bin[index].radial - (num_val1 * 1000));
   num_val2 = tmp / 100;                        /* The 1 deg */
   num_val3 = (tmp % 100) / 10;                 /* The 0.1 deg */

   num_val1 += (delta_turn1 * step1);
   while (num_val1 >= 36)
      num_val1 = num_val1 - 36;
   while (num_val1 < 0)
      num_val1 = num_val1 + 36;

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

   Store_bin[index].radial = (num_val1 * 1000) + (num_val2 * 100) + (num_val3 * 10);;

   /* This is only called if this is the currently displayed item,
      so update display value */
   IO_Sen_out->KNS80_FREQ_RAD_DST_display = Store_bin[index].radial;

   if (IO_Sen_out->KNS80_DSP_display == IO_Sen_out->KNS80_USE_display)
   {
      /* If we are in a mode that uses radial distance, reset the
         distance to go calculations */
      if (!Ils_beacon(&IO_nav1_battr))
      {
         if ((N1A_nav1_mode == NAV1_RNV) || (N1A_nav1_mode == NAV1_RNV_APR))
            BC_reset_KT_MIN_calc();
      }
   }
}

/*----------------------------------------------------------------------*/

/* Handles changes to data input knobs when in WP_DIST_DISPLAY 
   Data_display mode
   
   Inputs:  Data_display
*/
   
static void Wp_distance_update()
{
   int  index;
   long delta_turn1;
   long delta_turn2;
   long delta_turn3;
   long step1 = 1;   /* 1 unit, ie. step1_value = 1000, 10 nm */
   long step2 = 1;   /* 1 unit, ie. step2_value = 100,  1 nm */
   long step3 = 1;   /* 1 unit, ie. step3_value = 10,   0.1 nm */
   long num_val1;
   long num_val2;
   long num_val3;
   long tmp;
   float dist_nm;

   /* Outer knob updates in steps of 10 nm
      Centre knob in updates in steps of 1 nm
      Centre knob out updates in steps of 0.1 nm */

   Get_dial_deltas(&delta_turn1, &delta_turn2, &delta_turn3);

   if ((delta_turn1 == 0) && (delta_turn2 == 0) && (delta_turn3 == 0))
      return;

   index = IO_Sen_out->KNS80_DSP_display - 1;

   num_val1 = Store_bin[index].distance / 1000;   /* The 10 nm */
   tmp = (Store_bin[index].distance - (num_val1 * 1000));
   num_val2 = tmp / 100;                        /* The 1 nm */
   num_val3 = (tmp % 100) / 10;                 /* The 0.1 nm */

   num_val1 += (delta_turn1 * step1);
   while (num_val1 >= MAX_RAD_10_DIST)
      num_val1 = num_val1 - MAX_RAD_10_DIST;
   while (num_val1 < 0)
      num_val1 = num_val1 + MAX_RAD_10_DIST;

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

   Store_bin[index].distance = (num_val1 * 1000) + (num_val2 * 100) + (num_val3 * 10);;

   /* Must update the squared value too */
   dist_nm = ((float) Store_bin[index].distance) / 100.0f;
   Store_bin[index].dist_sq = dist_nm * dist_nm;


   /* This is only called if this is the currently displayed item,
      so update display value */
   IO_Sen_out->KNS80_FREQ_RAD_DST_display = Store_bin[index].distance;


   if (IO_Sen_out->KNS80_DSP_display == IO_Sen_out->KNS80_USE_display)
   {
      /* If we are in a mode that uses radial distance, reset the
         distance to go calculations */
      if (!Ils_beacon(&IO_nav1_battr))
      {
         if ((N1A_nav1_mode == NAV1_RNV) || (N1A_nav1_mode == NAV1_RNV_APR))
            BC_reset_KT_MIN_calc();
      }
   }
}

/*----------------------------------------------------------------------*/

/* This initializes all data that may be reset by data saved in a file
   (to simulate data kept after a power off) */

static void Init_kept_memory()
{
   int i;

   /* States controlled by momentary buttons */
   Data_display = FREQ_DISPLAY;

   IO_Sen_out->KNS80_USE_display = 1;
   IO_Sen_out->KNS80_DSP_display = 1;

   N1A_nav1_mode = NAV1_VOR;
   N1A_dme_hold = FALSE;

   for (i = 0; i < MAX_NUM_STORAGE_BIN; i++)
   {
      Store_bin[i].freq = DEFAULT_NAV1_FREQ;
      Store_bin[i].radial = 0;     /* x 100 deg */
      Store_bin[i].distance = 0;   /* x 100 nm */
      Store_bin[i].dist_sq = 0;    /* nm sq */
   }
}

/*----------------------------------------------------------------------*/

static void Read_kept_memory(char *filename)
{
   FILE           *stream;
   long           file_len;
   int            bytes_read;
   n1_kept_memory kmem;
   int            i;

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

   if (file_len != sizeof(n1_kept_memory))
   {
      printf("\nError: File %s has unexpected size (%d instead of %d)", filename, file_len, sizeof(n1_kept_memory));
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

   if (kmem.opcode != NAV1_OPCODE)
   {
      printf("\nError: file %s is not a nav1 type file", filename);
      fflush (stdout);
      return;
   }

   /* Copy data to relevant places */
   for (i = 0; i < MAX_NUM_STORAGE_BIN; i++)
   {
      memcpy(&(Store_bin[i]), &(kmem.bin[i]), sizeof(n1_store_bin));
   }

   Data_display = kmem.data_display;
   IO_Sen_out->KNS80_USE_display = kmem.use_display;
   IO_Sen_out->KNS80_DSP_display = kmem.dsp_display;

   N1A_nav1_mode = kmem.nav1_mode;
   N1A_dme_hold = kmem.dme_hold;
   if (N1A_dme_hold)    //NB. This also depends on the HLD button being pressed!
   {
      /* This does not really correspond to what happened in practise! */
      Hold_freq = kmem.dme_hold_freq;
   }

   return;
}

/*----------------------------------------------------------------------*/

static void Write_kept_memory(char *filename)
{
   FILE           *stream;
   n1_kept_memory kmem;
   int            i;
   int            num_bytes;

   kmem.opcode       = NAV1_OPCODE;
   kmem.num_bins     = MAX_NUM_STORAGE_BIN;
   kmem.data_display = Data_display;
   kmem.use_display  = (unsigned char) IO_Sen_out->KNS80_USE_display;
   kmem.dsp_display  = (unsigned char) IO_Sen_out->KNS80_DSP_display;
   kmem.nav1_mode    = N1A_nav1_mode;
   kmem.dme_hold     = N1A_dme_hold;
   kmem.spare        = 0;
   if (N1A_dme_hold)
      kmem.dme_hold_freq = Hold_freq;
   else
      kmem.dme_hold_freq = 0;

   for (i = 0; i < MAX_NUM_STORAGE_BIN; i++)
   {
      memcpy(&(kmem.bin[i]), &(Store_bin[i]), sizeof(n1_store_bin));
   }

   stream = fopen( filename, "wb" );

   if (!stream)
   {
      printf("\nError in opening file %s for write", filename);
      fflush (stdout);
      return;
   }

   num_bytes = fwrite (&kmem, 1, sizeof(n1_kept_memory), stream);

   fclose (stream);

   return;
}

/*----------------------------------------------------------------------*/

void N1A_exit()
{
   Write_kept_memory("nav1.cfg");
}

/*----------------------------------------------------------------------*/

int N1A_ils_freq_num(long freq)
{
   int first_dec;

   if(freq >= MAX_ILS_FREQ)
      return FALSE;
   if(freq <= MIN_ILS_FREQ)
      return FALSE;
   first_dec = (freq % 100) / 10;
   if(first_dec % 2) // ODD
      return TRUE;

   return FALSE;
}
/*----------------------------------------------------------------------*/