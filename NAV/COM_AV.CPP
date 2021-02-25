/*
File  : com_av.c
Prefix: CA_

Module: Dynamics for Seneca
Created: February 14 2000

Description: This file handles user input on the KY 196 comms box
*/

/***************************/
/*   RCS Markers           */
/***************************/

/*
$Header: C:/master/seneca/NAV/RCS/com_av.cpp 1.4 2000/05/23 10:47:56 juliae Exp $
$Log: com_av.cpp $
Revision 1.4  2000/05/23 10:47:56  juliae
If box is off, ignore turn dial changes.

Revision 1.3  2000/04/25 16:03:31  juliae
Added save and restore of data to file.

Revision 1.2  2000/03/17 16:59:41  juliae
Replaced IO_elec_pwr with IO_avionic_pwr. Allow for com1 fault.

 * Revision 1.1  2000/02/22  02:22:47  juliae
 * Initial revision
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

/* NB Must be very careful about default freq. The last digit
   can only be the '10' component of a multiple of 25 and
   allowance is made for this when updating via turn dials. */
#define DEFAULT_COM_FREQ 11800
#define MIN_COM_MHZ_FREQ 118
#define MAX_COM_MHZ_FREQ 136

#define TOKEN_ON   LIT
#define TOKEN_OFF  UNLIT

#define COM1_OPCODE 6

/*----------------*/
/* Typedefs       */
/*----------------*/

typedef struct
{
   unsigned char opcode;
   char          spare[3];
   long          use_freq;
   long          stby_freq;

} c1_kept_memory;

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
extern int             IO_avionic_pwr;
extern float           delta_time;

/*----------------*/
/* Globals        */
/*----------------*/

/*-----------------*/
/* Local Variables */    
/*-----------------*/

/* Flags for momentary button changes */
static int Last_KY196_FRQ_button;

/* For KY196 turn dial counts */
static long Last_turn1 = 0;
static long Last_turn2 = 0;
static long Last_turn3 = 0;

static int Last_pwr_on = FALSE;

/*--------------------*/
/* Forward References */
/*--------------------*/

static void Com_box_active();
static void Com_box_blank();
static void Reset_turn_counts();
static int  Stdby_freq_update();
static void Init_kept_memory();
static void Read_kept_memory(char *filename);

/*----------------------------------------------------------------------*/

void CA_com_init()
{
   Last_pwr_on = FALSE;
   if (IO_avionic_pwr && (IO_Sen_in->KY196_off_on_switch == SW_ON) && !IO_Sen_faults.AVF.com1)
      Last_pwr_on = TRUE;

   /* Flags for momentary button changes */
   Last_KY196_FRQ_button  = IO_Sen_in->KY196_FRQ_button;

   /* For KY196 turn dial counts */
   Reset_turn_counts();

   Init_kept_memory();
   Read_kept_memory("comm1.cfg");

   if (!Last_pwr_on)
      Com_box_blank();
   else
      Com_box_active();
}

/*----------------------------------------------------------------------*/

/* This performs all the initialization required when power is restored
   to adf box. See also CA_com_init. Do not initialize anything here
   that is included in "kept memory" */

static void Com_pwr_on()
{
   /* Flags for momentary button changes */
   Last_KY196_FRQ_button  = IO_Sen_in->KY196_FRQ_button;

   /* For KY196 turn dial counts */
   Reset_turn_counts();

   Com_box_active();
}

/*----------------------------------------------------------------------*/

static void Reset_turn_counts()
{
   /* For KY196 turn dial counts */
   Last_turn1 = IO_Sen_in->KY196_delta_outer_encoder;
   Last_turn2 = IO_Sen_in->KY196_delta_inner_encoder;
   Last_turn3 = IO_Sen_in->KY196_delta_inner_encoder_pull;
}

/*----------------------------------------------------------------------*/

static void Com_button_changes()
{
   long ltmp;
   int  index = 0;

   if (!IO_avionic_pwr || IO_Sen_faults.AVF.com1)
   {
      if (Last_pwr_on)
      {
         Com_box_blank();
      }
      Last_pwr_on = FALSE;
      return;
   }

   if (IO_Sen_in->KY196_off_on_switch == SW_OFF)
   {
      if (Last_pwr_on)
      {
         /* Blank displays */
         Com_box_blank();
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
         Com_pwr_on();
      }
      Last_pwr_on = TRUE;
   }

   /* Test for change to momentary FRQ pushbutton */
   if ((Last_KY196_FRQ_button == PRESSED) &&
       (IO_Sen_in->KY196_FRQ_button == NOT_PRESSED))
   {
      /* This button swaps the standby and active
         frequencies */
      ltmp = IO_Sen_out->KY196_USE_display;
      IO_Sen_out->KY196_USE_display = IO_Sen_out->KY196_STBY_display;
      IO_Sen_out->KY196_STBY_display = ltmp;
   }

   Last_KY196_FRQ_button = IO_Sen_in->KY196_FRQ_button;
}

/*----------------------------------------------------------------------*/

static void Com_gen_update()
{
   if (!IO_avionic_pwr || IO_Sen_faults.AVF.com1)
      return;

   if (IO_Sen_in->KY196_off_on_switch == SW_OFF)
      return;

   if (IO_Sen_in->comm_1_PTT == SW_ON)
      IO_Sen_out->KY196_T_token = TOKEN_ON;
   else
      IO_Sen_out->KY196_T_token = TOKEN_OFF;
}

/*----------------------------------------------------------------------*/

/* This should be called after Com_button_changes, so that
   the turn dial counts are reset if power is restored. */

static void Com_turn_dial_changes()
{
   if (!IO_avionic_pwr || IO_Sen_faults.AVF.com1)
      return;

   //This is needed so changes in turn dials when box if off
   //do NOT change values!
   if (IO_Sen_in->KY196_off_on_switch == SW_OFF)
      return;

   Stdby_freq_update();
}

/*----------------------------------------------------------------------*/

void CA_com_update()
{
   Com_gen_update();
   Com_button_changes();
   Com_turn_dial_changes();
}

/*----------------------------------------------------------------------*/

static void Com_box_blank()
{
   IO_Sen_out->KY196_mode = BLANK;

   /* Turn off all tokens */
   IO_Sen_out->KY196_T_token = TOKEN_OFF;
}

/*----------------------------------------------------------------------*/

/* Activate only those tokens and modes that are ONLY dependent
   on power. All others will be updated elsewhere */

static void Com_box_active()
{
   IO_Sen_out->KY196_mode = ACTIVE;

   if (IO_Sen_in->comm_1_PTT == SW_ON)
      IO_Sen_out->KY196_T_token = TOKEN_ON;
   else
      IO_Sen_out->KY196_T_token = TOKEN_OFF;
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

   num_step1 = IO_Sen_in->KY196_delta_outer_encoder;
   num_step2 = IO_Sen_in->KY196_delta_inner_encoder;
   num_step3 = IO_Sen_in->KY196_delta_inner_encoder_pull;

   ds1 = num_step1 - Last_turn1;
   ds2 = num_step2 - Last_turn2;
   ds3 = num_step3 - Last_turn3;

   *delta_turn1 = ds1;
   *delta_turn2 = ds2;
   *delta_turn3 = ds3;

   Last_turn1 = num_step1;
   Last_turn2 = num_step2;
   Last_turn3 = num_step3;

   return;
}

/*----------------------------------------------------------------------*/

/* Handles changes to data input knobs. Only the STBY values are
   updated directly.
   NB. The fine control is in 0.025 units but only 2 decimal places
   are displayed, SO need a separate variable to store the 0.001 MHz
   value (otherwise 4 updates of 0.025 units will not give
   0.1 update!).
   There appears to be no direct update of the 0.1 unit place.
   Return TRUE if a change is made.
   
   Inputs:  KY196_STBY_display

   Outputs: KY196_STBY_display
*/
   
static int Stdby_freq_update()
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

   Get_dial_deltas(&delta_turn1, &delta_turn2, &delta_turn3);

   if ((delta_turn1 == 0) && (delta_turn2 == 0) && (delta_turn3 == 0))
      return FALSE;

   /* This differs from the other dial update routines */

   freq = IO_Sen_out->KY196_STBY_display;
   if (freq % 5)
      unit_khz = 5;   /* ie. abcd7 => abcd7.5, abcd2 => abcd2.5 */
   else
      unit_khz = 0;
   num_val1 = freq / 100;       /* The hundreds */
   dec_val_100 = freq % 100;    /* ie. 75 => 0.75 */
   dec_val_1000 = (dec_val_100 * 10) + unit_khz;  /* ie. 755 => 0.755 */

   num_val1 += (delta_turn1 * step1);
   while (num_val1 > MAX_COM_MHZ_FREQ)
      num_val1 = num_val1 - (MAX_COM_MHZ_FREQ + 1 - MIN_COM_MHZ_FREQ);
   while (num_val1 < MIN_COM_MHZ_FREQ)
      num_val1 = num_val1 + (MAX_COM_MHZ_FREQ + 1 - MIN_COM_MHZ_FREQ);


   dec_val_1000 += (delta_turn2 * step2);
   dec_val_1000 += (delta_turn3 * step3);
   while (dec_val_1000 > 999)
      dec_val_1000 = dec_val_1000 - 1000;
   while (dec_val_1000 < 0)
      dec_val_1000 = dec_val_1000 + 1000;

   dec_val_100 = dec_val_1000 / 10;

   IO_Sen_out->KY196_STBY_display = (num_val1 * 100) + dec_val_100;

   return TRUE;
}

/*----------------------------------------------------------------------*/

/* This initializes all data that may be reset by data saved in a file
   (to simulate data kept after a power off) */

static void Init_kept_memory()
{
   IO_Sen_out->KY196_USE_display = DEFAULT_COM_FREQ;
   IO_Sen_out->KY196_STBY_display = DEFAULT_COM_FREQ;
}

/*----------------------------------------------------------------------*/

static void Read_kept_memory(char *filename)
{
   FILE           *stream;
   long           file_len;
   int            bytes_read;
   c1_kept_memory kmem;

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

   if (file_len != sizeof(c1_kept_memory))
   {
      printf("\nError: File %s has unexpected size (%d instead of %d)", filename, file_len, sizeof(c1_kept_memory));
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

   if (kmem.opcode != COM1_OPCODE)
   {
      printf("\nError: file %s is not a com1 type file", filename);
      fflush (stdout);
      return;
   }

   IO_Sen_out->KY196_USE_display = kmem.use_freq;
   IO_Sen_out->KY196_STBY_display = kmem.stby_freq;

   return;
}

/*----------------------------------------------------------------------*/

static void Write_kept_memory(char *filename)
{
   FILE           *stream;
   c1_kept_memory kmem;
   int            num_bytes;

   kmem.opcode       = COM1_OPCODE;
   kmem.spare[0]     = 0;
   kmem.spare[1]     = 0;
   kmem.spare[2]     = 0;

   kmem.use_freq  = IO_Sen_out->KY196_USE_display;
   kmem.stby_freq = IO_Sen_out->KY196_STBY_display;

   stream = fopen( filename, "wb" );

   if (!stream)
   {
      printf("\nError in opening file %s for write", filename);
      fflush (stdout);
      return;
   }

   num_bytes = fwrite (&kmem, 1, sizeof(c1_kept_memory), stream);
   fclose (stream);

   return;
}

/*----------------------------------------------------------------------*/

void CA_exit()
{
   Write_kept_memory("comm1.cfg");
}

/*----------------------------------------------------------------------*/
