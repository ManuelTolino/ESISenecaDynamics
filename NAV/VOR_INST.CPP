/*
File: vor_inst.c
Prefix: VOR_

Module: Dynamics for Seneca

Created: December 1999

Description: This file calculates navigation dial outputs depending
             on VOR navigation data.
*/

/***************************/
/*   RCS Markers           */
/***************************/

/*
$Header: C:/master/seneca/NAV/RCS/vor_inst.cpp 1.5 2000/05/09 12:31:49 juliae Exp $
$Log: vor_inst.cpp $
Revision 1.5  2000/05/09 12:31:49  juliae
Correction for MAX_VOR_RANGE being in nm.

Revision 1.4  2000/04/17 12:01:30  juliae
Safeguard against invalid merit range index

Revision 1.3  2000/03/27 09:22:07  juliae
New routine for how VHF signals vary with altitude.
Impose a max signal range. Use slant instead of horizontal
distance for range checks. If signal is ok, return an
indication of signal strength.

 * Revision 1.2  2000/03/02  10:23:29  juliae
 * Renamed host_pos->alt to ht and allow for beacon elevation.
 *
 * Revision 1.1  2000/02/22  00:18:05  juliae
 * Initial revision
 *
*/

/*----------------------------------------------------------------------*/

/*-------------------------*/
/* Includes                */
/*-------------------------*/

#include <stdio.h>
#include <math.h>
#include "c:\bae-sims\seneca\fltdy/fmgdefs.h"
#include "c:\bae-sims\seneca/define.h"
#include "c:\bae-sims\seneca\nav/beacon.h"
#include "c:\bae-sims\seneca\nav/b_dials.h"
#include "c:\bae-sims\seneca\nav/bc_range.h"

/*-------------------------*/
/* Defines                 */
/*-------------------------*/

#define VOR_MAX_LR_DEV_ANG 10.0f
#define MAX_MERIT_RANGES 3

#define ALT_RANGE_TEST 1

#define MAX_SIGNAL 100
#define MIN_SIGNAL 1

//#define DEBUG_FAIL 1

/*-------------------------*/
/* Typedefs                */
/*-------------------------*/

/*----------------------------*/
/* External Routines          */
/*----------------------------*/

extern void ILS_get_lr_adev(float approach_head, float bear_host_beacon,
                            float max_dev_ang,
                            float *lr_dev, char *to_flag);
extern double SPH_get_sph_flat_dist_sq (double lat1, double lon1, float ht1,
                                        double lat2, double lon2, float ht2);

/*----------------------------*/
/* Externals                  */
/*----------------------------*/

/*----------------------------*/
/* Static Variables           */
/*----------------------------*/

static float Merit_range[MAX_MERIT_RANGES];
static char  Unlimited_range_ind;
static char  Out_of_service_ind;

/*--------------------*/
/* Forward References */
/*--------------------*/

/*----------------------------------------------------------------*/

/* This initializes merit ranges */

void VOR_init()
{
   /* Be aware of MAX_MERIT_RANGES */
   Merit_range[0] = 25.0  * METRES_IN_NMILE;
   Merit_range[1] = 40.0  * METRES_IN_NMILE;
   Merit_range[2] = 130.0 * METRES_IN_NMILE;

   Merit_range[0] = 50.0  * METRES_IN_NMILE;
   Merit_range[1] = 80.0  * METRES_IN_NMILE;
   Merit_range[2] = 260.0 * METRES_IN_NMILE;

   //Make signal available outside true range
   Merit_range[0] *= RANGE_INC;
   Merit_range[1] *= RANGE_INC;
   Merit_range[2] *= RANGE_INC;

   Unlimited_range_ind = 3;
   Out_of_service_ind = 9;
}

/*----------------------------------------------------------------*/

/* For VHF, range also varies with ht according to
   range_nm = 1.25 * (sqrt(Ht_transmitter) + sqrt(Ht_host_rec))
   where Ht is above SEA level in FEET.

   Inputs: Heights in metres above sea level
   Outputs: Slant range in metres */

float VOR_get_vhf_alt_range(float beac_elev, float host_ht)
{
   float ht_tx;
   float ht_rx;
   float range;
   
   ht_tx = beac_elev * (float) FT_IN_METRE;
   ht_rx = host_ht * (float) FT_IN_METRE;
   range = (float) (1.25 * (sqrt(ht_tx) + sqrt(ht_rx)) * METRES_IN_NMILE);

   return range;
}

/*----------------------------------------------------------------*/

/* Returns non-zero if host is within valid region (DOC) for vor.
   In this case return a value in range MIN to MAX_SIGNAL as a
   measure of signal strength.
   This checks for range and "cone of confusion" (when pitch
   line (host to VOR) is greater than CONE_CONFUSION_PITCH) */

int VOR_valid_vor_signal(vhf_beac_attr *vor_in, ll_pos *host_pos)
{
   double dist_sq;
   double sdist_host_vor;
   int    range_ind;
   float  max_range;
   float  sin_pitch;
   float  pitch;
   float  alt_range;    //Metres
   float  max;
   float  min;
   float  fraction;
   int    strength;

   if (!vor_in->valid_flag)
      return FALSE;

   if (!vor_in->valid_vor_ils)
      return FALSE;

   /* Get distance between host and localizer, to see if with range.
      This gives slant distance, since we pass correct heights */

   dist_sq = SPH_get_sph_flat_dist_sq (host_pos->lat, host_pos->lon, host_pos->ht,
                                       vor_in->beac_lat,
                                       vor_in->beac_lon,
                                       vor_in->beac_elev);

   sdist_host_vor = sqrt(dist_sq);

   range_ind = vor_in->valid_vor_ind;
   if (range_ind == Out_of_service_ind)
      return FALSE;
   if (range_ind == Unlimited_range_ind)
   {
      max_range = MAX_VOR_RANGE * METRES_IN_NMILE;  //Metres
      if (sdist_host_vor > max_range)
      {
#ifdef DEBUG_FAIL
         printf("\nFail 1: dist = %.2f, max_range = %.2f", sdist_host_vor, max_range);
#endif
         return FALSE;
      }
   }
   else if ((range_ind < MAX_MERIT_RANGES) && (range_ind >= 0))
   {
      max_range = Merit_range[range_ind];   //Metres
      if (sdist_host_vor > max_range)
      {
#ifdef DEBUG_FAIL
         printf("\nFail 1: dist = %.2f, Merit_range = %.2f", sdist_host_vor, Merit_range[range_ind]);
#endif
         return FALSE;
      }
   }
   else
   {
      return FALSE;
   }

   /* Find pitch angle of host above vor */

   if (sdist_host_vor < 0.1)
      return FALSE;

   sin_pitch =  (float) ((host_pos->ht - vor_in->beac_elev) / sdist_host_vor);
   pitch = (float) (asin(sin_pitch) * RAD_TO_DEG);

   if (pitch > CONE_CONFUSION_PITCH)
   {
#ifdef DEBUG_FAIL
      printf("\nFail 2: pitch = %.2f, Confusion pitch = %.2f", pitch, CONE_CONFUSION_PITCH);
#endif
      return FALSE;
   }

#ifdef ALT_RANGE_TEST
   /* In addition to simple range tests, range also varies with ht
      due to nature of VHF signals */

   alt_range = VOR_get_vhf_alt_range(vor_in->beac_elev, host_pos->ht);
   if (sdist_host_vor > alt_range)
   {
#ifdef DEBUG_FAIL
      printf("\nFail 3: dist = %.2f, alt_range = %.2f", sdist_host_vor, alt_range);
#endif
      return FALSE;
   }
#else
   alt_range = max_range;
#endif

   /* Return an indication of signal strength */
   if (alt_range < max_range)
      fraction = (float) sdist_host_vor / alt_range;
   else
      fraction = (float) sdist_host_vor / max_range;

   max = (float) MAX_SIGNAL;
   min = (float) MIN_SIGNAL;
   strength = (int) (max - fraction * (max - min));
   if (strength < MIN_SIGNAL)
      strength = MIN_SIGNAL;
   return strength;
}

/*----------------------------------------------------------------*/

/* Radials are measured from MAGNETIC north.
   NB. Although many of these dial values match those of the HSI,
   the obs value for these 2 dials could be different!
   
   This routine does NOT directly update IO_Sen_out structure since it
   it is used by both NAV2 and HSI dials.
*/

void VOR_calc_vor_data (int valid_signal, float recip_radial, float obs,
                        vor_inst_out_data *vor_out)
{
   float mbear_host_vor;
   float lr_dev;
   char  to_flag;

   if (valid_signal)
   {
      /* Find bearing from host to vor. */
      mbear_host_vor = recip_radial;

      ILS_get_lr_adev(obs, mbear_host_vor, VOR_MAX_LR_DEV_ANG,
                      &lr_dev, &to_flag);
   }

   vor_out->valid = valid_signal;
   if (valid_signal)
   {
      vor_out->radial_dev = lr_dev;
      vor_out->to_flag = to_flag;
   }
   else
   {
      vor_out->radial_dev = 0.0;
      vor_out->to_flag = NO_FLAG;
   }

   /* Glideslope not active for this type */
}

/*----------------------------------------------------------------*/
