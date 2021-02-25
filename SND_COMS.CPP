/*
File: snd_coms.c
Prefix: SC_

Module: Dynamics for Seneca

Created: 9 February 2000

Description: This file sets up the coms info to send to sound PC
*/

/***************************/
/*   RCS Markers           */
/***************************/

/*
$Header: C:/master/seneca/RCS/snd_coms.cpp 1.3 2000/05/25 15:36:55 juliae Exp $
$Log: snd_coms.cpp $
Revision 1.3  2000/05/25 15:36:55  juliae
Correction to morse ident string when dme_hold is on.

Revision 1.2  2000/05/11 09:27:26  juliae
Allow for ident buttons on Nav1 and Nav2 boxes.
Change DME morse rate.

Revision 1.1  2000/04/14 17:37:24  juliae
Initial revision

*/

/*----------------------------------------------------------------------*/

/*-------------------------*/
/* Includes                */
/*-------------------------*/

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "nav/beacon.h"
#include "dyn_snd.h"
#include "dat_stru/in_head.h"

/*-------------------------*/
/* Defines                 */
/*-------------------------*/

#define TRUE  1
#define FALSE 0

#define SIZE_MORSE_STR 4
#define NO_MARKER 0

#define DME_MORSE_RATE 2   //Number of times a minute

/*----------------------------*/
/* External Routines          */
/*----------------------------*/

/*----------------------------*/
/* Externals                  */
/*----------------------------*/

extern sen_in_struct  *IO_Sen_in;
extern OUTPUTin			*OUTPUTinbox;//simulinkudp

extern vhf_beac_attr IO_nav2_battr;
extern vhf_beac_attr IO_nav1_battr;
extern vhf_beac_attr IO_dme_battr;
extern ndb_beac_attr IO_ndb_battr;

extern int BC_valid_n1_b_signal;
extern int BC_valid_n2_b_signal;
extern int BC_valid_dme_signal;
extern int BC_valid_ndb_signal;

extern int IOS_marker_type;
extern int N1A_dme_hold;

/*----------------*/
/* Globals        */
/*----------------*/

int SC_nav1_audio   = FALSE;
int SC_nav2_audio   = FALSE;
int SC_dme_audio    = FALSE;
int SC_marker_audio = FALSE;
int SC_adf_audio    = FALSE;
int SC_adf_snd_type = MORSE_TYPE;

/*----------------*/
/* Locals         */
/*----------------*/

/*----------------------------------------------------------------*/

void SC_init_snd_coms()
{
   SC_nav1_audio   = FALSE;
   SC_nav2_audio   = FALSE;
   SC_dme_audio    = FALSE;
   SC_marker_audio = FALSE;
   SC_adf_audio    = FALSE;
   SC_adf_snd_type = MORSE_TYPE;
}

/*----------------------------------------------------------------*/

/* This sets up the sound information to be sent to the sound PC
   This needs to be called after beacon dial updates since that
   is where the audio flags are set up.

   Inputs: Audio flags (setup in BC_update_beacon_dials)
           Beacon data
*/

void SC_setup_eth_snd_data(dyn_to_snd *dyn2snd)
{
   /* NAV1 */
   if (SC_nav1_audio && (IO_Sen_in->KNS80_off_on_pull_switch == SW_ON))
   {
      dyn2snd->nav1.on = TRUE;
      dyn2snd->nav1.signal_strength = BC_valid_n1_b_signal;
      dyn2snd->nav1.type = MORSE_TYPE;
      dyn2snd->nav1.morse_rate = IO_nav1_battr.ident_emmision_rate;
      memcpy (dyn2snd->nav1.morse_str, IO_nav1_battr.ident, SIZE_MORSE_STR);
   }
   else
      dyn2snd->nav1.on = FALSE;

   /* NAV2 */
   if (SC_nav2_audio && (IO_Sen_in->KX155_NAV_ident_pull_switch == SW_ON))
   {
      dyn2snd->nav2.on = TRUE;
      dyn2snd->nav2.signal_strength = BC_valid_n2_b_signal;
      dyn2snd->nav2.type = MORSE_TYPE;
      dyn2snd->nav2.morse_rate = IO_nav2_battr.ident_emmision_rate;
      memcpy (dyn2snd->nav2.morse_str, IO_nav2_battr.ident, SIZE_MORSE_STR);
   }
   else
      dyn2snd->nav2.on = FALSE;

   /* DME: Only used when from NAV1 */
   if (SC_dme_audio && (IO_Sen_in->KNS80_off_on_pull_switch == SW_ON))
   {
      dyn2snd->dme.on = TRUE;
      dyn2snd->dme.signal_strength = BC_valid_dme_signal;
      dyn2snd->dme.type = MORSE_TYPE;
      /* Use a fixed rate for DME */
      dyn2snd->dme.morse_rate = DME_MORSE_RATE;
      if (N1A_dme_hold)
         memcpy (dyn2snd->dme.morse_str, IO_dme_battr.dme_ident, SIZE_MORSE_STR);
      else
         memcpy (dyn2snd->dme.morse_str, IO_nav1_battr.dme_ident, SIZE_MORSE_STR);
   }
   else
      dyn2snd->dme.on = FALSE;
   
   /* ADF */
   if (SC_adf_audio)
   {
      dyn2snd->adf.on = TRUE;
      dyn2snd->adf.signal_strength = BC_valid_ndb_signal;
      dyn2snd->adf.type = SC_adf_snd_type;
      dyn2snd->adf.morse_rate = IO_ndb_battr.ident_emmision_rate;
      memcpy (dyn2snd->adf.morse_str, IO_ndb_battr.ident, SIZE_MORSE_STR);
   }
   else
      dyn2snd->adf.on = FALSE;
   
   /* MARKER */
   if (IOS_marker_type != NO_MARKER)
   {
      dyn2snd->marker_on = TRUE;
      dyn2snd->marker_type = IOS_marker_type - 1;
   }
   else
   {
      dyn2snd->marker_on = FALSE;
   }

}

/*----------------------------------------------------------------*/
