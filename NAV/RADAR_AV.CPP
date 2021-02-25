/*
File  : radar_av.c
Prefix: RA_

Module: Dynamics For Seneca
Created: 29/2/2000

Description: This file handles inputs and outputs of the KT76 box
             Note that KT76_OFF   => OFF or STDBY
                       KT76_ON    => ON  or ALT
*/

/***************************/
/*   RCS Markers           */
/***************************/

/*
$Header: C:/master/seneca/NAV/RCS/radar_av.cpp 1.3 2000/05/18 09:57:05 juliae Exp $
$Log: radar_av.cpp $
Revision 1.3  2000/05/18 09:57:05  juliae
Lamp lights when ident button pressed and both lamp and SQUAWK
signal remain on for fixed time after button release.

Revision 1.2  2000/03/17 16:57:52  juliae
Reduced flash time and allowed for transponder fault.

 * Revision 1.1  2000/03/16  10:50:47  juliae
 * Initial revision
 *
*/

/*----------------------------------------------------------------------*/

/*----------------*/
/* Include Files  */
/*----------------*/

#include "c:\bae-sims\seneca\dat_stru/out_head.h"
#include "c:\bae-sims\seneca\dat_stru/in_head.h"
#include "c:\bae-sims\seneca\define.h"
#include "c:\bae-sims\seneca\faults.h"

/*----------------*/
/* Defines        */
/*----------------*/

#define RAD_IDLE_MODE    0
#define RAD_NORMAL_MODE  1    /* Flashes due to radar interrogation */
#define RAD_WARM_UP_MODE 2

#define WARM_UP_TIME 47.0

#define FLASH_INTERVAL 11.5       /* Seconds */
#define FLASH_ON_TIME   0.1       /* Seconds */

#define SQUAWK_EXTRA_TIME 15.0     /* Seconds */

/*----------------*/
/* Typedefs       */
/*----------------*/

/*--------------------*/
/* External Routines  */
/*--------------------*/

/*--------------------*/
/* External Variables */
/*--------------------*/

extern sen_out_struct *IO_Sen_out;
extern sen_in_struct  *IO_Sen_in;
extern OUTPUTin			*OUTPUTinbox;//simulinkudp
extern int            IO_avionic_pwr;
extern float          delta_time;
extern sen_fault_struct IO_Sen_faults;

/*----------------*/
/* Globals        */
/*----------------*/

//This is TRUE for a SQUAWK_EXTRA time after
//ident_button is released, hence need extra variable
int RA_squawk_on = FALSE;

/*-----------------*/
/* Local Variables */    
/*-----------------*/

static int Last_pwr_on = FALSE;
static int Last_function_sw = KT76_OFF_STBY;
static int Last_ident_button;
static int Mode = RAD_IDLE_MODE;
static int Warm_up = FALSE;
static float Warm_up_time = 0.0;
static float Flash_time = 0.0;
static int   Flash_on = FALSE;
static int   Ident_extra_mode = FALSE;
static float Ident_extra_time = 0.0f;

/*--------------------*/
/* Forward References */
/*--------------------*/

static void Rad_box_active();
static void Rad_box_blank();

/*----------------------------------------------------------------------*/

void RA_rad_init()
{
   Last_pwr_on = FALSE;
   if (IO_avionic_pwr && (IO_Sen_in->KT76_off_on != KT76_OFF_STBY) && !IO_Sen_faults.AVF.radar)
      Last_pwr_on = TRUE;

   Last_function_sw = IO_Sen_in->KT76_off_on;
   Last_ident_button = IO_Sen_in->KT76_IDENT_button;
   RA_squawk_on = FALSE;

   Mode = RAD_IDLE_MODE;
   Warm_up_time = 0.0;
   Ident_extra_mode = FALSE;

   if (!Last_pwr_on)
      Rad_box_blank();
   else
      Rad_box_active();
}

/*----------------------------------------------------------------------*/

/* In normal mode the lamp flashes due to radar interrogation */

static void Init_normal_mode()
{
   Mode = RAD_NORMAL_MODE;
   Flash_time = 0.0;
   Flash_on = TRUE;
   IO_Sen_out->KT76_IDENT_lamp = LIT;

   Ident_extra_mode = FALSE;
   RA_squawk_on = FALSE;
}

/*----------------------------------------------------------------------*/

static void Rad_button_changes()
{
   if (!IO_avionic_pwr || IO_Sen_faults.AVF.radar)
   {
      if (Last_pwr_on)
      {
         Rad_box_blank();
         RA_squawk_on = FALSE;
         Ident_extra_mode = FALSE;
      }
      Last_pwr_on = FALSE;
      return;
   }

   if (IO_Sen_in->KT76_off_on == KT76_OFF_STBY)
   {
      if (Last_pwr_on)
      {
         /* Blank displays */
         Rad_box_blank();
         RA_squawk_on = FALSE;
         Ident_extra_mode = FALSE;
      }
      Last_pwr_on = FALSE;
      return;
   }
   else   /* ON / ALT / TEST */
   {
      if (!Last_pwr_on)
      {
         /* Have just restored power, EITHER by the local on/off
            switch or global power (global power could have
            been turned on with local switch still on). */
         Rad_box_active();
      }
      Last_pwr_on = TRUE;

      if (Last_function_sw != IO_Sen_in->KT76_off_on)
      {
#ifdef NO_SEP_STBY
         if (IO_Sen_in->KT76_off_on == KT76_STBY)
         {
            Warm_up_time = 0.0;
            Mode = RAD_WARM_UP_MODE;
            IO_Sen_out->KT76_IDENT_lamp = LIT;
         }
         /* What happens if we change to ON before warm up has finished? */
#endif
         if (IO_Sen_in->KT76_off_on == KT76_ON_ALT)
         {
            /* Change to normal mode */
            Init_normal_mode();
         }
         else if (IO_Sen_in->KT76_off_on == KT76_TEST)
         {
            Mode = RAD_IDLE_MODE;
            IO_Sen_out->KT76_IDENT_lamp = LIT;
            Ident_extra_mode = FALSE;
            RA_squawk_on = FALSE;
         }
      }
   }

   Last_function_sw = IO_Sen_in->KT76_off_on;

   if (IO_Sen_in->KT76_off_on == KT76_ON_ALT)
   {
      if (IO_Sen_in->KT76_IDENT_button == PRESSED)
      {
         IO_Sen_out->KT76_IDENT_lamp = LIT;
         RA_squawk_on = TRUE;
      }
      else if (Last_ident_button == PRESSED)
      {
         //Button just released. Lamp remains on for SQUAWK_EXTRA time
         //Signal to IOS also remains for same time
         IO_Sen_out->KT76_IDENT_lamp = LIT;
         RA_squawk_on = TRUE;
         Ident_extra_mode = TRUE;
         Ident_extra_time = 0.0f;
      }
   }

   Last_ident_button = IO_Sen_in->KT76_IDENT_button;
}

/*----------------------------------------------------------------------*/

/* General updates to support the functions of the KT76 box. */

static void Rad_gen_update()
{
   if (!IO_avionic_pwr || IO_Sen_faults.AVF.radar)
      return;

   if (IO_Sen_in->KT76_off_on == KT76_OFF_STBY)
      return;

#ifdef NO_SEP_STBY
   if (Mode == RAD_WARM_UP_MODE)
   {
      Warm_up_time += delta_time;
      if (Warm_up_time > WARM_UP_TIME)
      {
         /* Change to normal mode. */
         Init_normal_mode();
      }
   }
   else
#endif
   if (Mode == RAD_NORMAL_MODE)  // && (IO_Sen_in->KT76_off_on == KT76_ON) ?
   {
      Flash_time += delta_time;
      if (Flash_on)
      {
         if (Flash_time > FLASH_ON_TIME)
         {
            Flash_time = 0.0;
            /* Turn the lamp off */
            IO_Sen_out->KT76_IDENT_lamp = UNLIT;
            Flash_on = FALSE;
         }
      }
      else
      {
         if (Flash_time > FLASH_INTERVAL)
         {
            Flash_time = 0.0;
            /* Turn the lamp on */
            IO_Sen_out->KT76_IDENT_lamp = LIT;
            Flash_on = TRUE;
         }
      }

      if (Ident_extra_mode)
      {
         //Over-rule the general lamp flash
         IO_Sen_out->KT76_IDENT_lamp = LIT;

         Ident_extra_time += delta_time;
         if (Ident_extra_time > SQUAWK_EXTRA_TIME)
         {
            Ident_extra_mode = FALSE;
            RA_squawk_on = FALSE;
            Ident_extra_time = 0.0f;
            /* Turn the lamp off */
            IO_Sen_out->KT76_IDENT_lamp = UNLIT;
         }
      }
   }
}

/*----------------------------------------------------------------------*/

void RA_rad_update()
{
   Rad_gen_update();
   Rad_button_changes();
}

/*----------------------------------------------------------------------*/

static void Rad_box_blank()
{
   IO_Sen_out->KT76_IDENT_lamp = UNLIT;
}

/*----------------------------------------------------------------------*/

/* Called when box is on and receives power. It performs all
   required initialization for pwr_on */

static void Rad_box_active()
{
   Last_function_sw = IO_Sen_in->KT76_off_on;
   Last_ident_button = IO_Sen_in->KT76_IDENT_button;
   Ident_extra_mode = FALSE;
   RA_squawk_on = FALSE;

#ifdef NO_SEP_STBY
   if (IO_Sen_in->KT76_off_on == KT76_STBY)
   {
      Warm_up_time = 0.0;
      Mode = RAD_WARM_UP_MODE;
      IO_Sen_out->KT76_IDENT_lamp = LIT;
   }
   else
#endif
   if (IO_Sen_in->KT76_off_on == KT76_TEST)
   {
      Mode = RAD_IDLE_MODE;
      IO_Sen_out->KT76_IDENT_lamp = LIT;
   }
   else  /* ON and ALT */
   {
      Init_normal_mode();

      if (IO_Sen_in->KT76_IDENT_button == PRESSED)
         RA_squawk_on = TRUE;
   }
}

/*----------------------------------------------------------------------*/
/*----------------------------------------------------------------------*/
