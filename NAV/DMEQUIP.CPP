/*
File: beacon.c
Prefix: BC_

Module: Navigation (Dynamics) for Seneca
Created: January 7 2000

Description: For use with the DME calculations.
---------------------*/

/***************************/
/*   RCS Markers           */
/***************************/

/*
$Header: C:/master/seneca/NAV/RCS/dmequip.cpp 1.4 2000/05/09 07:48:46 juliae Exp $
$Log: dmequip.cpp $
Revision 1.4  2000/05/09 07:48:46  juliae
Allow for dme offset. Replaced DME_get_srange_sq_nm with
DME_get_srange_nm.

Revision 1.3  2000/03/27 11:48:02  juliae
Added VHF range dependent on altitude. Setup a measure of
signal strength.

 * Revision 1.2  2000/03/02  10:20:42  juliae
 * Renamed host alt to ht and dme_alt to dme_elev.
 *
 * Revision 1.1  2000/02/22  01:53:28  juliae
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
#include "beacon.h"
#include "bc_range.h"

/*-------------------------*/
/* Defines                 */
/*-------------------------*/

#define DME_RANGE_NM_SQ (DME_RANGE_NM * DME_RANGE_NM)

#define ALT_RANGE_TEST 1

#define MAX_SIGNAL 100
#define MIN_SIGNAL 1

//#define DEBUG_FAIL 1
/*----------------------------*/
/* External Routines          */
/*----------------------------*/

extern double SPH_get_sph_flat_dist_sq (double lat1, double lon1, float ht1,
                                        double lat2, double lon2, float ht2);
extern float VOR_get_vhf_alt_range(float beac_elev, float host_ht);

/*----------------------------------------------------------------*/

/* If out of range, valid_signal is set to 0. Otherwise this gives a
   value in range MIN to MAX_SIGNAL as a measure of signal strength. */

void DME_get_srange_nm (vhf_beac_attr *battr, ll_pos *host_pos, float *srange_nm, int *valid_signal)
{
   double dist_sq;
   double dist;
   double dist_nm;
   float  max;
   float  min;
   float  fraction;
   int    strength;
   float  alt_range;
   float  alt_range_nm = DME_RANGE_NM;
   float  dme_offset;

   /* This works in metres */
   dist_sq = SPH_get_sph_flat_dist_sq (host_pos->lat, host_pos->lon, host_pos->ht,
                                       battr->dme_lat,
                                       battr->dme_lon,
                                       battr->dme_elev);

   dist = sqrt(dist_sq);

   /* Convert to NM */
   dist_nm = dist / METRES_IN_NMILE;
   if (dist_nm > DME_RANGE_NM)
   {
      *srange_nm = 0.0;
      *valid_signal = FALSE;
      return;
   }

#ifdef ALT_RANGE_TEST
   /* In addition to simple range tests, range also varies with ht
      due to nature of VHF signals */

   alt_range = VOR_get_vhf_alt_range(battr->dme_elev, host_pos->ht);
   alt_range_nm = (float) (alt_range / METRES_IN_NMILE);
   if (alt_range_nm < DME_RANGE_NM)
   {
      if (dist_nm > alt_range_nm)
      {
#ifdef DEBUG_FAIL
         printf("\nFail 10: sdist = %.2f, alt_range_nm = %.2f", dist_nm, alt_range_nm);
#endif
         *srange_nm = 0.0;
         *valid_signal = FALSE;
         return;
      }
   }
#endif

   max = (float) MAX_SIGNAL;
   min = (float) MIN_SIGNAL;
   if (alt_range_nm < DME_RANGE_NM)
      fraction = (float) dist_nm / alt_range_nm;
   else
      fraction = (float) dist_nm / DME_RANGE_NM;
   strength = (int) (max - fraction * (max - min));
   if (strength < MIN_SIGNAL)
      strength = MIN_SIGNAL;
   *valid_signal = strength;


   //Allow for dme offset
   dme_offset = ((float) battr->dme_offset) / 10.0f;   //Offset given in 10th of a nm
   dist_nm = dist_nm - dme_offset;
   if (dist_nm < 0.0)
      dist_nm = 0.0;
   *srange_nm = (float) dist_nm;

   return;
}

/*----------------------------------------------------------------*/
