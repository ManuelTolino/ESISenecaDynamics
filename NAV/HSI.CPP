/*
File: hsi.c
Prefix: HSI_

Module: Navigation (Dynamics) for Seneca

Description: For use with the HSI instrument (also called Pictorial
             Navigation Instrument (PNI).
---------------------*/

/***************************/
/*   RCS Markers           */
/***************************/

/*
$Header: C:/master/seneca/NAV/RCS/hsi.cpp 1.4 2000/05/16 17:12:49 colinj Exp $
$Log: hsi.cpp $
Revision 1.4  2000/05/16 17:12:49  colinj
Now pass a parameter to HSI_set_hsi_head to say if gyro is ok.

Revision 1.3  2000/04/25 17:32:37  juliae
HSI_set_hsi_nav now allows for a valid glide signal
when no ils signal.

Revision 1.2  2000/03/17 16:44:37  juliae
valid_dme flag now passed to HSI_set_hsi_nav.

 * Revision 1.1  2000/02/22  01:56:47  juliae
 * Initial revision
 *
*/

/*----------------------------------------------------------------------*/

/*-------------------------*/
/* Includes                */
/*-------------------------*/

#include <stdio.h>
#include <math.h>
#include "c:\bae-sims\seneca\define.h"
#include "b_dials.h"
#include "c:\bae-sims\seneca\nav/beacon.h"
#include "c:\bae-sims\seneca\nav/nav1.h"
#include "c:\bae-sims\seneca\dat_stru/in_head.h"
#include "c:\bae-sims\seneca\nav/cpit_def.h"

/*-------------------------*/
/* Defines                 */
/*-------------------------*/

#define HSI_RNV_MAX_LR_DEV_NM     5.0
#define HSI_RNV_APR_MAX_LR_DEV_NM 1.25
#define HSI_VPAR_MAX_LR_DEV_NM    5.0

/*-------------------------*/
/* Typedefs                */
/*-------------------------*/

/*----------------------------*/
/* External Routines          */
/*----------------------------*/

extern void ILS_get_lr_adev(float approach_head, float bear_host_beacon,
                            float max_dev_ang,
                           float *lr_dev, char *to_flag);
extern void ILS_get_to_from_flag(float approach_head, float bear_host_beacon, char *to_flag);
extern void ILS_calc_ils_data (vhf_beac_attr *ils_in, ll_pos *host_pos, int valid_signal,
                               float recip_radial, ils_inst_out_data *ils_out);
extern void VOR_calc_vor_data (int valid_signal, float recip_radial, float obs,
                               vor_inst_out_data *vor_out);

/*----------------------------*/
/* Externals                  */
/*----------------------------*/

extern sen_in_struct *IO_Sen_in;
extern OUTPUTin			*OUTPUTinbox;//simulinkudp
extern int N1A_nav1_mode;
extern int N1A_dme_hold;
extern int IO_elec_pwr;

/*----------------------------*/
/* Static Variables           */
/*----------------------------*/

/*----------------------------------------------------------------*/

/* This routine determines the HDG related data for the HSI.
   This routine is called for both pilot and copilot, so IO_Sen_out
   values are NOT updated here. If heading is invalid, it is left
   at previous value.

   Inputs:  IO_elec_pwr
*/

void HSI_set_hsi_head (float gyro_mhead, int gyro_ok, hsi_out_data *hsi)
{
   /* We need to show the compass position slowly turn
      during gyro run up */

   if (!IO_elec_pwr)
   {
      hsi->hdg_flag = SHOW_FLAG;
   }
   else
   {
      if (!gyro_ok)
         hsi->hdg_flag = SHOW_FLAG;
      else
         hsi->hdg_flag = HIDE_FLAG;

      /* Compass card rotated so that heading is at top */
      hsi->compass_demand_position = gyro_mhead;   /* In degrees */
   }
}

/*----------------------------------------------------------------*/

static void Set_no_signal_hsi(hsi_out_data *hsi)
{
   hsi->nav_flag = SHOW_FLAG;
   /* Course deviation needle centres */
   hsi->left_right_needle = CENTRE_NEEDLE;
}

/*----------------------------------------------------------------------*/

/* Returns TRUE if beacon has a valid glide signal */

static int Glides_beacon(vhf_beac_attr *battr)
{
   if (battr->valid_flag &&
       (battr->type == ILS_TYPE) &&
       battr->valid_glide_tx)
      return TRUE;
   else
      return FALSE;
}

/*----------------------------------------------------------------*/

/* This routine determines the NAV related data for the HSI.
   For use with NAV 1 only.
   This instrument has items in common with the VOR/ILS instrument
   (NAV2 KI 206)
   This routine is called for both pilot and copilot, so IO_Sen_out
   values are NOT updated here.
   The validity of the beacon signal is checked OUTSIDE this routine.
   
   NB. cross_dev is only used with NAV1_RNV, NAV1_RNV_APR and
   NAV1_VOR_PAR modes. DME must be active for these modes. If
   not, the CALLING routine passes 0 for the cross_dev.
   In all modes, the nav_flag is shown if there is no valid vor or
   ils signal (valid_signal). In addition NAV1_RNV, NAV1_RNV_APR and
   NAV1_VOR_PAR modes also show the nav flag if the dme signal is
   invalid. N1A_dme_hold has been taken in to account in setting up
   the valid_dme flag

   recip_radial is the magnetic bearing from host to beacon (ish)

   Inputs:  N1A_nav1_mode

   Ouputs: No global data
*/

void HSI_set_hsi_nav (vhf_beac_attr *battr, ll_pos *host_pos, int valid_signal,
                      int valid_dme, float recip_radial, float hsi_obs,
                      float cross_dev, hsi_out_data *hsi)
{
   int               valid_vor = TRUE;
   ils_inst_out_data ils_out;
   vor_inst_out_data vor_out;
   float             max_dev;
   char              to_flag;
   
   if (!valid_signal)    //For ILS types, may still have a valid glide signal
   {
      Set_no_signal_hsi(hsi);
      if (!Glides_beacon(battr))
         return;
   }
   else
      hsi->nav_flag = HIDE_FLAG;

   /* Left/right deviation bar. */
   if (battr->type == VOR_TYPE)
   {
      /* Glideslope not active for this type */
      hsi->GS_enable = DISABLE;
      hsi->GS_needle = CENTRE_NEEDLE;


      /* The meaning of the left/right needle depends on the NAV1
         mode, if not an ILS freq */

      if ((N1A_nav1_mode == NAV1_RNV) || (N1A_nav1_mode == NAV1_RNV_APR))
      {
         /* Want lr_dev to be +- 5nm for RNV and 1.25nm for RNV_APR
            DME must be active for these modes */
         if (!valid_dme)
         {
            /* Area Navigation Computer is invalid. */
            hsi->nav_flag = SHOW_FLAG;
            hsi->left_right_needle = CENTRE_NEEDLE;
         }
         else
         {
            if (N1A_nav1_mode == NAV1_RNV)
               max_dev = HSI_RNV_MAX_LR_DEV_NM;
            else
               max_dev = HSI_RNV_APR_MAX_LR_DEV_NM;

            if (cross_dev > max_dev)
               hsi->left_right_needle = RIGHT_NEEDLE;
            else if (cross_dev < -max_dev)
               hsi->left_right_needle = LEFT_NEEDLE;
            else
            {
               hsi->left_right_needle = (cross_dev * 100.0f)/ max_dev;
            }

            /* The to/from needle refers to the waypoint acting as
               a beacon, NOT the battr beacon. In this case, the
               CALLING ROUTINE sets this up, so we LEAVE IT ALONE */
         }
      }
      else if (N1A_nav1_mode == NAV1_VOR_PAR)
      {
         /* Want lr_dev to be +- 5nm. DME must be active
            for this mode */
         if (!valid_dme)
         {
            /* Area Navigation Computer is invalid. */
            hsi->nav_flag = SHOW_FLAG;
            hsi->left_right_needle = CENTRE_NEEDLE;
         }
         else
         {
            max_dev = HSI_VPAR_MAX_LR_DEV_NM;

            if (cross_dev > max_dev)
               hsi->left_right_needle = RIGHT_NEEDLE;
            else if (cross_dev < -max_dev)
               hsi->left_right_needle = LEFT_NEEDLE;
            else
               hsi->left_right_needle = (cross_dev * 100.0f)/ max_dev;

            ILS_get_to_from_flag(hsi_obs, recip_radial, &to_flag);

            if (to_flag == TO_FLAG)
               hsi->to_from_needle = TO_NEEDLE;
            else if (to_flag == FROM_FLAG)
               hsi->to_from_needle = FROM_NEEDLE;
            else
               hsi->to_from_needle = CENTRE_NEEDLE;
         }
      }
      else  /* N1A_nav1_mode == NAV1_VOR */
      {
         /* Valid signal tests already done */
         VOR_calc_vor_data (valid_signal, recip_radial, hsi_obs, &vor_out);

         /* Now update hsi_out values */

         hsi->left_right_needle = vor_out.radial_dev * 100.0f;
         if (vor_out.to_flag == TO_FLAG)
            hsi->to_from_needle = TO_NEEDLE;
         else if (vor_out.to_flag == FROM_FLAG)
            hsi->to_from_needle = FROM_NEEDLE;
         else
            hsi->to_from_needle = CENTRE_NEEDLE;
      }
   }
   else   /* ILS */
   {
      /* Note that there may be a valid glide slope even if ILS signal is invalid */

      /* To/from not active for this type */
      hsi->to_from_needle = CENTRE_NEEDLE;

      if (valid_signal)
         ILS_calc_ils_data (battr, host_pos, valid_signal, recip_radial, &ils_out);
      else
         ILS_calc_ils_data (battr, host_pos, FALSE, 0.0f, &ils_out);

      /* Now update hsi_out values */
      if (ils_out.valid_glide)
      {
         hsi->GS_enable = ENABLE;
         hsi->GS_needle = ils_out.vert_dev * 100.0f;
      }
      else
      {
         hsi->GS_enable = DISABLE;
         hsi->GS_needle = CENTRE_NEEDLE;
      }
      hsi->left_right_needle = ils_out.lt_rt_dev * 100.0f;
   }
}

/*----------------------------------------------------------------*/
