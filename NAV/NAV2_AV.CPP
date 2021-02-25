/*
File  : nav2_av.c
Prefix: N2A_

Module: Dynamics for Seneca
Created: February 12 2000

Description: This file handles user input on the KX 155 avionics box
*/

/***************************/
/*   RCS Markers           */
/***************************/

/*
$Header: C:/Master/seneca/NAV/RCS/nav2_av.cpp 1.6 2000/06/16 10:58:13 ColinJ Exp $
$Log: nav2_av.cpp $
Revision 1.6  2000/06/16 10:58:13  ColinJ
New function long N2A_get_active_n2_freq.

Revision 1.5  2000/05/18 10:31:34  juliae
Added support for the T token.

Revision 1.4  2000/04/25 16:03:31  juliae
Added save and restore of data to file.

Revision 1.3  2000/04/18 15:28:48  juliae
Added support for requesting beacon info constantly.

Revision 1.2  2000/03/17 16:31:27  juliae
Change IO_elec_pwr to IO_avionic_pwr. Allow for nav2 fault.

Revision 1.1  2000/02/22  00:20:01  juliae
Initial revision
*/

/*----------------------------------------------------------------------*/

/*----------------*/
/* Include Files  */
/*----------------*/

#include <stdio.h>
#include <math.h>

#include "c:\bae-sims\seneca\nav/beacon.h"
#include "c:\bae-sims\seneca\dat_stru/out_head.h"
#include "c:\bae-sims\seneca\dat_stru/in_head.h"
#include "c:\bae-sims\seneca\define.h"
#include "c:\bae-sims\seneca\faults.h"

/*----------------*/
/* Defines        */
/*----------------*/

#define DEFAULT_NAV2_FREQ 10800
#define MIN_NAV2_MHZ_FREQ 108
#define MAX_NAV2_MHZ_FREQ 117
/* NB Must be very careful about default freq. The last digit
   can only be the '10' component of a multiple of 25 and
   allowance is made for this when updating via turn dials. */
#define DEFAULT_COM2_FREQ 11800
#define MIN_COM2_MHZ_FREQ 118
#define MAX_COM2_MHZ_FREQ 136

#define TOKEN_ON   LIT
#define TOKEN_OFF  UNLIT

#define NAV2_OPCODE 2

/*----------------*/
/* Typedefs       */
/*----------------*/

typedef struct
{
   unsigned char opcode;
   char          spare[3];
   long          nav_use_freq;
   long          nav_stby_freq;
   long          comm_use_freq;
   long          comm_stby_freq;

} n2_kept_memory;

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
extern vhf_beac_attr  IO_nav2_battr;
extern int            IO_avionic_pwr;

/*----------------*/
/* Globals        */
/*----------------*/

/* This is used to say if new beacon info is wanted from IOS */
int    N2A_new_nav2_bfreq = FALSE;
double N2A_nav2_search_time = 0.0;
int    N2A_nav2_search_mode = FALSE;

/*-----------------*/
/* Local Variables */    
/*-----------------*/

/* Flags for momentary button changes */
static int Last_KX155_COMM_FRQ_button;
static int Last_KX155_NAV_FRQ_button;

/* For KX155 turn dial counts */
static long Last_nav2_turn1 = 0;
static long Last_nav2_turn2 = 0;
static long Last_nav2_turn3 = 0;
static long Last_com2_turn1 = 0;
static long Last_com2_turn2 = 0;
static long Last_com2_turn3 = 0;

static int Last_pwr_on = FALSE;

/*--------------------*/
/* Forward References */
/*--------------------*/

long N2A_get_activen2_freq();

static void Nav2_box_active();
static void Nav2_box_blank();
static void Tune_nav2_beacon();
static void Tune_off_nav2_beacon();
static void Reset_turn_counts();
static void Nav2_freq_update();
static void Com2_freq_update();
static void Turn_dial_changes();
static void Init_kept_memory();
static void Read_kept_memory(char *filename);

/*----------------------------------------------------------------------*/

void N2A_nav2_init()
{
   IO_nav2_battr.valid_flag = FALSE;

   Last_pwr_on = FALSE;
   if (IO_avionic_pwr && (IO_Sen_in->KX155_off_on_switch == SW_ON) && !IO_Sen_faults.AVF.com_nav2)
      Last_pwr_on = TRUE;

   /* Flags for momentary button changes */
   Last_KX155_COMM_FRQ_button = IO_Sen_in->KX155_COMM_FRQ_button;
   Last_KX155_NAV_FRQ_button  = IO_Sen_in->KX155_NAV_FRQ_button;

   /* For KX155 turn dial counts */
   Reset_turn_counts();

   Init_kept_memory();
   Read_kept_memory("nav2.cfg");

   if (!Last_pwr_on)
   {
      Nav2_box_blank();
      Tune_off_nav2_beacon();
   }
   else
   {
      Nav2_box_active();
      Tune_nav2_beacon();
   }
}

/*----------------------------------------------------------------------*/

/* This performs all the initialization required when power is restored
   to adf box. See also N2A_nav2_init. Do not initialize anything here
   that is included in "kept memory" */

static void Nav2_pwr_on()
{
   /* Flags for momentary button changes */
   Last_KX155_COMM_FRQ_button = IO_Sen_in->KX155_COMM_FRQ_button;
   Last_KX155_NAV_FRQ_button  = IO_Sen_in->KX155_NAV_FRQ_button;

   /* For KX155 turn dial counts */
   Reset_turn_counts();

   Nav2_box_active();
   IO_nav2_battr.valid_flag = FALSE;
   Tune_nav2_beacon();
}

/*----------------------------------------------------------------------*/

static void Reset_turn_counts()
{
   /* For KX155 turn dial counts */
   Last_nav2_turn1 = IO_Sen_in->KX155_NAV_delta_outer_encoder;
   Last_nav2_turn2 = IO_Sen_in->KX155_NAV_delta_inner_encoder;
   Last_nav2_turn3 = IO_Sen_in->KX155_NAV_delta_inner_encoder_pull;
   Last_com2_turn1 = IO_Sen_in->KX155_COMM_delta_outer_encoder;
   Last_com2_turn2 = IO_Sen_in->KX155_COMM_delta_inner_encoder;
   Last_com2_turn3 = IO_Sen_in->KX155_COMM_delta_inner_encoder_pull;
}

/*----------------------------------------------------------------------*/

static void Nav2_button_changes()
{
   long ltmp;

   if (!IO_avionic_pwr || IO_Sen_faults.AVF.com_nav2)
   {
      if (Last_pwr_on)
      {
         Nav2_box_blank();
         Tune_off_nav2_beacon();
      }
      Last_pwr_on = FALSE;
      return;
   }

   if (IO_Sen_in->KX155_off_on_switch == SW_OFF)
   {
      if (Last_pwr_on)
      {
         /* Blank displays */
         Nav2_box_blank();
         Tune_off_nav2_beacon();
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
         Nav2_pwr_on();
      }
      Last_pwr_on = TRUE;
   }

   /* Test for change to momentary COMM_FRQ pushbutton */
   if ((Last_KX155_COMM_FRQ_button == PRESSED) &&
       (IO_Sen_in->KX155_COMM_FRQ_button == NOT_PRESSED))
   {
      /* This button swaps the standby and active
         frequencies */
      ltmp = IO_Sen_out->KX155_COMM_USE_display;
      IO_Sen_out->KX155_COMM_USE_display = IO_Sen_out->KX155_COMM_STBY_display;
      IO_Sen_out->KX155_COMM_STBY_display = ltmp;
   }

   Last_KX155_COMM_FRQ_button = IO_Sen_in->KX155_COMM_FRQ_button;

   /* Test for change to momentary NAV_FRQ pushbutton */
   if ((Last_KX155_NAV_FRQ_button == PRESSED) &&
       (IO_Sen_in->KX155_NAV_FRQ_button == NOT_PRESSED))
   {
      /* This button swaps the standby and active
         frequencies */
      ltmp = IO_Sen_out->KX155_NAV_USE_display;
      IO_Sen_out->KX155_NAV_USE_display = IO_Sen_out->KX155_NAV_STBY_display;
      IO_Sen_out->KX155_NAV_STBY_display = ltmp;

      IO_nav2_battr.valid_flag = FALSE;
      Tune_nav2_beacon();
   }

   Last_KX155_NAV_FRQ_button = IO_Sen_in->KX155_NAV_FRQ_button;
}

/*----------------------------------------------------------------------*/

static void Nav2_gen_update()
{
   if (!IO_avionic_pwr || IO_Sen_faults.AVF.com_nav2)
      return;

   if (IO_Sen_in->KX155_off_on_switch == SW_OFF)
      return;

   if (IO_Sen_in->comm_2_PTT == SW_ON)
      IO_Sen_out->KX155_T_token = TOKEN_ON;
   else
      IO_Sen_out->KX155_T_token = TOKEN_OFF;
}

/*----------------------------------------------------------------------*/

/* This should be called after Nav2_button_changes, so that
   the turn dial counts are reset if power is restored. */

static void Turn_dial_changes()
{
   if (!IO_avionic_pwr || IO_Sen_faults.AVF.com_nav2)
      return;

   if (IO_Sen_in->KX155_off_on_switch == SW_OFF)
      return;

   Nav2_freq_update();
   Com2_freq_update();
}

/*----------------------------------------------------------------------*/

void N2A_nav2_update()
{
   Nav2_gen_update();
   Nav2_button_changes();
   Turn_dial_changes();
}

/*----------------------------------------------------------------------*/

static void Nav2_box_blank()
{
   IO_Sen_out->KX155_mode = BLANK;

   /* Turn off all tokens */
   IO_Sen_out->KX155_T_token = TOKEN_OFF;
}

/*----------------------------------------------------------------------*/

/* Activate only those tokens and modes that are ONLY dependent
   on power. All others will be updated elsewhere */

static void Nav2_box_active()
{
   IO_Sen_out->KX155_mode = ACTIVE;

   if (IO_Sen_in->comm_2_PTT == SW_ON)
      IO_Sen_out->KX155_T_token = TOKEN_ON;
   else
      IO_Sen_out->KX155_T_token = TOKEN_OFF;
}

/*----------------------------------------------------------------------*/

/* This sets up necessary variables to request new beacon
   search from the IOS.
   Note that the current beacon flag is NOT set to invalid here. SHOULD IT BE???? */

static void Tune_nav2_beacon()
{
   N2A_new_nav2_bfreq = TRUE;
   N2A_nav2_search_time = 0.0;
   N2A_nav2_search_mode = TRUE;
}

/*----------------------------------------------------------------------*/

/* This routine is not essential since it does no harm to request
   info from the IOS even if it will not be used */

static void Tune_off_nav2_beacon()
{
   N2A_new_nav2_bfreq = FALSE;

   /* This is not really needed here since all tune off cases are
      handled where the valid_flag is used */
   IO_nav2_battr.valid_flag = FALSE;
}

/*----------------------------------------------------------------------*/

/* Each turn has its own wrap, so CANNOT just calculate an overall
   delta */

static void Get_nav2_dial_deltas(long *delta_turn1, long *delta_turn2, long *delta_turn3)
{
   long num_step1;  /* Current turn value */
   long num_step2;
   long num_step3;
   long ds1;        /* Change in turn value */
   long ds2;
   long ds3;

   /* Need to work out the delta turn of each knob
      from the last turn values stored. */

   num_step1 = IO_Sen_in->KX155_NAV_delta_outer_encoder;
   num_step2 = IO_Sen_in->KX155_NAV_delta_inner_encoder;
   num_step3 = IO_Sen_in->KX155_NAV_delta_inner_encoder_pull;

   ds1 = num_step1 - Last_nav2_turn1;
   ds2 = num_step2 - Last_nav2_turn2;
   ds3 = num_step3 - Last_nav2_turn3;

   *delta_turn1 = ds1;
   *delta_turn2 = ds2;
   *delta_turn3 = ds3;

   Last_nav2_turn1 = num_step1;
   Last_nav2_turn2 = num_step2;
   Last_nav2_turn3 = num_step3;

   return;
}

/*----------------------------------------------------------------------*/

/* Each turn has its own wrap, so CANNOT just calculate an overall
   delta */

static void Get_com2_dial_deltas(long *delta_turn1, long *delta_turn2, long *delta_turn3)
{
   long num_step1;  /* Current turn value */
   long num_step2;
   long num_step3;
   long ds1;        /* Change in turn value */
   long ds2;
   long ds3;

   /* Need to work out the delta turn of each knob
      from the last turn values stored. */

   num_step1 = IO_Sen_in->KX155_COMM_delta_outer_encoder;
   num_step2 = IO_Sen_in->KX155_COMM_delta_inner_encoder;
   num_step3 = IO_Sen_in->KX155_COMM_delta_inner_encoder_pull;

   ds1 = num_step1 - Last_com2_turn1;
   ds2 = num_step2 - Last_com2_turn2;
   ds3 = num_step3 - Last_com2_turn3;

   *delta_turn1 = ds1;
   *delta_turn2 = ds2;
   *delta_turn3 = ds3;

   Last_com2_turn1 = num_step1;
   Last_com2_turn2 = num_step2;
   Last_com2_turn3 = num_step3;

   return;
}

/*----------------------------------------------------------------------*/

/* Handles changes to data input knobs. Only the STBY values are
   updated directly.
   
   Inputs:  

   Outputs: 
*/
   
static void Nav2_freq_update()
{
   long freq;
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

   Get_nav2_dial_deltas(&delta_turn1, &delta_turn2, &delta_turn3);

   if ((delta_turn1 == 0) && (delta_turn2 == 0) && (delta_turn3 == 0))
      return;

   freq = IO_Sen_out->KX155_NAV_STBY_display;
   num_val1 = freq / 100;       /* The hundreds */
   tmp = freq % 100;
   num_val2 = tmp / 10;         /* The tens */
   num_val3 = tmp % 10;         /* The units */

   num_val1 += (delta_turn1 * step1);
   while (num_val1 > MAX_NAV2_MHZ_FREQ)
      num_val1 = num_val1 - (MAX_NAV2_MHZ_FREQ + 1 - MIN_NAV2_MHZ_FREQ);
   while (num_val1 < MIN_NAV2_MHZ_FREQ)
      num_val1 = num_val1 + (MAX_NAV2_MHZ_FREQ + 1 - MIN_NAV2_MHZ_FREQ);

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

   IO_Sen_out->KX155_NAV_STBY_display = (num_val1 * 100) + (num_val2 * 10) + (num_val3 * 1);
}

/*----------------------------------------------------------------------*/

/* Handles changes to data input knobs. Only the STBY values are
   updated directly.
   NB. The fine control is in 0.025 units but only 2 decimal places
   are displayed, SO need a separate variable to store the 0.001 MHz
   value (otherwise 4 updates of 0.025 units will not give
   0.1 update!).
   There appears to be no direct update of the 0.1 unit place.
   
   Inputs:  KX155_COMM_STBY_display

   Outputs: KX155_COMM_STBY_display
*/
   
static void Com2_freq_update()
{
   long freq;
   long delta_turn1;
   long delta_turn2;
   long delta_turn3;
   long step1 = 1;   /* 1 unit,    ie. step1_value = 100, 1 MHz */
   long step2 = 50;  /* In units of 0.001 MHz, ie. this is 0.05 MHz = 50 KHz */
   long step3 = 25;  /* In units of 0.001 MHz, ie. this is 0.025 MHz = 25 KHz */
   long num_val1;
   long dec_val_100;
   long dec_val_1000;
   long unit_khz;

   /* Outer knob updates in steps of 1 MHz
      Centre knob in updates in steps of 0.1 MHz
      Centre knob out updates in steps of 0.05 MHz */

   Get_com2_dial_deltas(&delta_turn1, &delta_turn2, &delta_turn3);

   if ((delta_turn1 == 0) && (delta_turn2 == 0) && (delta_turn3 == 0))
      return;

   /* This differs from the other dial update routines */

   freq = IO_Sen_out->KX155_COMM_STBY_display;
   if (freq % 5)
      unit_khz = 5;   /* ie. abcd7 => abcd7.5, abcd2 => abcd2.5 */
   else
      unit_khz = 0;
   num_val1 = freq / 100;       /* The hundreds */
   dec_val_100 = freq % 100;    /* ie. 75 => 0.75 */
   dec_val_1000 = (dec_val_100 * 10) + unit_khz;  /* ie. 755 => 0.755 */

   num_val1 += (delta_turn1 * step1);
   while (num_val1 > MAX_COM2_MHZ_FREQ)
      num_val1 = num_val1 - (MAX_COM2_MHZ_FREQ + 1 - MIN_COM2_MHZ_FREQ);
   while (num_val1 < MIN_COM2_MHZ_FREQ)
      num_val1 = num_val1 + (MAX_COM2_MHZ_FREQ + 1 - MIN_COM2_MHZ_FREQ);


   dec_val_1000 += (delta_turn2 * step2);
   dec_val_1000 += (delta_turn3 * step3);
   while (dec_val_1000 > 999)
      dec_val_1000 = dec_val_1000 - 1000;
   while (dec_val_1000 < 0)
      dec_val_1000 = dec_val_1000 + 1000;

   dec_val_100 = dec_val_1000 / 10;

   IO_Sen_out->KX155_COMM_STBY_display = (num_val1 * 100) + dec_val_100;
}

/*----------------------------------------------------------------------*/

/* This initializes all data that may be reset by data saved in a file
   (to simulate data kept after a power off) */

static void Init_kept_memory()
{
   IO_Sen_out->KX155_NAV_USE_display = DEFAULT_NAV2_FREQ;
   IO_Sen_out->KX155_NAV_STBY_display = DEFAULT_NAV2_FREQ;

   IO_Sen_out->KX155_COMM_USE_display = DEFAULT_COM2_FREQ;
   IO_Sen_out->KX155_COMM_STBY_display = DEFAULT_COM2_FREQ;
}

/*----------------------------------------------------------------------*/

static void Read_kept_memory(char *filename)
{
   FILE           *stream;
   long           file_len;
   int            bytes_read;
   n2_kept_memory kmem;

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

   if (file_len != sizeof(n2_kept_memory))
   {
      printf("\nError: File %s has unexpected size (%d instead of %d)", filename, file_len, sizeof(n2_kept_memory));
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

   if (kmem.opcode != NAV2_OPCODE)
   {
      printf("\nError: file %s is not a com_nav2 type file", filename);
      fflush (stdout);
      return;
   }

   IO_Sen_out->KX155_NAV_USE_display = kmem.nav_use_freq;
   IO_Sen_out->KX155_NAV_STBY_display = kmem.nav_stby_freq;

   IO_Sen_out->KX155_COMM_USE_display = kmem.comm_use_freq;
   IO_Sen_out->KX155_COMM_STBY_display = kmem.comm_stby_freq;

   return;
}

/*----------------------------------------------------------------------*/

static void Write_kept_memory(char *filename)
{
   FILE           *stream;
   n2_kept_memory kmem;
   int            num_bytes;

   kmem.opcode       = NAV2_OPCODE;
   kmem.spare[0]     = 0;
   kmem.spare[1]     = 0;
   kmem.spare[2]     = 0;

   kmem.nav_use_freq  = IO_Sen_out->KX155_NAV_USE_display;
   kmem.nav_stby_freq = IO_Sen_out->KX155_NAV_STBY_display;

   kmem.comm_use_freq  = IO_Sen_out->KX155_COMM_USE_display;
   kmem.comm_stby_freq = IO_Sen_out->KX155_COMM_STBY_display;

   stream = fopen( filename, "wb" );

   if (!stream)
   {
      printf("\nError in opening file %s for write", filename);
      fflush (stdout);
      return;
   }

   num_bytes = fwrite (&kmem, 1, sizeof(n2_kept_memory), stream);

   fclose (stream);

   return;
}

/*----------------------------------------------------------------------*/

void N2A_exit()
{
   Write_kept_memory("nav2.cfg");
}

/*----------------------------------------------------------------------*/
/* Returns 0 if radio power is off */
long N2A_get_active_n2_freq(void)
{
   if(IO_avionic_pwr == FALSE)
   {
      return 0;
   }
   if(IO_Sen_in->KX155_off_on_switch == SW_OFF)
   {
      return 0;
   }
   if(IO_Sen_faults.AVF.com_nav2)
   {
      return 0;
   }

   return(IO_Sen_out->KX155_NAV_USE_display);
}
/*----------------------------------------------------------------------*/
