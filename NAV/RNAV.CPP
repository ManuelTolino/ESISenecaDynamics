/*
File  : rnav.c   (RNAV, RNV)
Prefix: RN_

Module: Area Navigation for Seneca Dynamics

Description:

For Virtual VORs

The aircraft distance from waypoint and its cross-track deviation are
calculated by the RNAV computer using

1. VOR station radial passing through waypoint (this is part of the
   waypoint description)
2. distance of waypoint from VOR (this is part of the waypoint
   description)
3. radial from the VOR that the aircraft is on
4. distance from VOR to aircraft (DME distance => slant range)
5. selected course (OBS) to waypoint

In order to attempt to do the same calculation as the RNAV computer
we use these inputs in these routine (though we first have to
calculate the VOR radial and distance to aircraft to input!)

---------------------*/

/***************************/
/*   RCS Markers           */
/***************************/

/*
$Header: C:/master/seneca/NAV/RCS/rnav.cpp 1.3 2000/05/09 09:24:52 juliae Exp $
$Log: rnav.cpp $
Revision 1.3  2000/05/09 09:24:52  juliae
Ensure cos_j is in valid range (-1 to 1).

Revision 1.2  2000/04/17 11:53:47  juliae
Changes to avoid compiler warnings

Revision 1.1  2000/02/22 01:57:22  juliae
Initial revision

*/

/*----------------------------------------------------------------------*/

/*----------------*/
/* Include Files  */
/*----------------*/

#include <stdio.h>
#include <math.h>

#include "c:\bae-sims\seneca\fltdy/fmgdefs.h"
#include "c:\bae-sims\seneca/define.h"
#include "c:\bae-sims\seneca\nav/b_dials.h"

/*----------------*/
/* Defines        */
/*----------------*/

#define RN_MIN_WP_HOST_DIST    0.01   // In nm
#define RN_MIN_HOST_BEAC_DIST  0.01   // In nm

/*----------------*/
/* Typedefs       */
/*----------------*/

/*--------------------*/
/* External Routines  */
/*--------------------*/

/*--------------------*/
/* External Variables */
/*--------------------*/

/*----------------*/
/* Globals        */
/*----------------*/

/*-----------------*/
/* Local Variables */    
/*-----------------*/

/*--------------------*/
/* Forward References */
/*--------------------*/

/*----------------------------------------------------------------------*/

/* This is only called when NAV1 is in an RNAV mode.
   This finds distance of host to waypoint for display on KNS80 box.
   Valid signal checks are assumed to be done OUTSIDE this routine. */

void RN_nav1_wpoint_dist(float host_radial, float wp_dist, float wp_dist_sq,
                        float wp_radial, float srange_nm, float srange_sq_nm,
                        float *wp_host_dist_nm)
{
   float wp_host_dist_sq;
   float wp_host_dist;
   float ang_a;
   float cos_a;

   /* This works ok if ang_a is 0 */
   ang_a = wp_radial - host_radial;

   /* Allow for going over 360: 10 - 350  = -340 => 20,
                                350 - 90  =  260 => -100, etc */

   if (ang_a > 180.0f)
      ang_a = ang_a - 360.0f;
   else if (ang_a < -180.0f)
      ang_a = 360.0f + ang_a;

   cos_a = (float) cos(ang_a * DEG_TO_RAD);

   /* Cosine rule: d3_sq = d1_sq + d2_sq - 2*d1*d2*cos_a */

   wp_host_dist_sq = wp_dist_sq + srange_sq_nm -
                     (2.0f * wp_dist * srange_nm * cos_a);

   wp_host_dist = (float) sqrt(wp_host_dist_sq);

   *wp_host_dist_nm = wp_host_dist;
}

/*----------------------------------------------------------------------*/

/* This is only called when NAV1 is an RNAV mode.
   This finds cross track deviation for HSI (PNI). This is for course
   with specified obs to pass through waypoint.
   This uses triangle formula, so need to safeguard against sides
   of the "triangle" being zero.
   
   Inputs: distances in nm
           angles in degrees
*/

int RN_nav1_wpoint_dev(float host_radial, float wp_dist_sq, float wp_radial,
                        float srange_nm, float srange_sq_nm, float wp_host_dist_nm, 
                        float obs, float *cross_dev_nm, char *to_flag)
{
   float bear_host2beac;
   float wp_host_dist_sq;
   float wp_host_dist;
   float bear_host2wp;
   float ang_j;
   float cos_j;
   float delta_radial;
   float delta_ang;
   float mag_delta_ang;
   float sin_ang;
   float cross_dev;

   bear_host2beac = host_radial - 180.0f;
   /* Keep in range 0 to 360 */
   if (bear_host2beac > 360.0f)
      bear_host2beac -= 360.0f;
   if (bear_host2beac < 0.0f)
      bear_host2beac += 360.0f;

   wp_host_dist = wp_host_dist_nm;
   wp_host_dist_sq = wp_host_dist * wp_host_dist;   /* Hmm... Should I pass this in too? */

   /* Find angle in triangle made by line from Beacon to Host
      and Host to Waypoint.
      By cosine rule:
      wp_dist_sq = srange_sq + wp_host_dist_sq -
                  2 * srange * wp_host_dist * cos_j */

   if (wp_host_dist < RN_MIN_WP_HOST_DIST)
   {
      /* Host is at waypoint */
      *cross_dev_nm = 0.0;
      *to_flag = NO_FLAG;
      return TRUE;
   }

   if (srange_nm < RN_MIN_HOST_BEAC_DIST)
   {
      /* Host radial is meaningless in this case */
      /* Bearing host to waypoint is wp_radial.
         Should I return FALSE, or make the calculation ok? */
      bear_host2wp = wp_radial;
   }
   else
   {
      cos_j = (srange_sq_nm + wp_host_dist_sq - wp_dist_sq ) /
                                          (2.0f * srange_nm * wp_host_dist);

      if (cos_j >= 1.0)
         cos_j = 1.0;
      if (cos_j <= -1.0)
         cos_j = -1.0;
      
      ang_j = (float) (acos(cos_j) * RAD_TO_DEG);

      /* Find bearing from host to waypoint. Note if wp_radial and host_radial
         are equal, ang_j is 0 */
      delta_radial = wp_radial - host_radial;
      /* Allow for going over 360 (This puts value in range -180 to 180) */
      if (delta_radial > 180.0f)
         delta_radial = delta_radial - 360.0f;
      else if (delta_radial < -180.0f)
         delta_radial += 360.0f;

      if (delta_radial < 0.0)
         bear_host2wp = bear_host2beac + ang_j;
      else
         bear_host2wp = bear_host2beac - ang_j;
   
      /* Keep in range 0 to 360 */
      if (bear_host2wp > 360.0)
         bear_host2wp -= 360.0;
      if (bear_host2wp < 0.0)
         bear_host2wp += 360.0;
   }

   /* Find angle between the obs and line from host to waypoint */
   delta_ang = bear_host2wp - obs;

   /* Allow for going over 360, eg. 10 - 350 => 20,
                                    350 - 10 => -20 */

   if (delta_ang > 180.0f)
      delta_ang = delta_ang - 360.0f;
   else if (delta_ang < -180.0f)
      delta_ang += 360.0f;

   mag_delta_ang = delta_ang;
   if (mag_delta_ang < 0.0)
      mag_delta_ang = -mag_delta_ang;

   if (mag_delta_ang <= 90.0)
      *to_flag = TO_FLAG;
   else
      *to_flag = FROM_FLAG;

   sin_ang = (float) sin(delta_ang * DEG_TO_RAD);

   cross_dev = wp_host_dist * sin_ang;  /* SIGNED: +ve=>go to right */

   *cross_dev_nm = cross_dev;

   return TRUE;
}

/*----------------------------------------------------------------------*/

/* Finds cross track deviation from host to line through beacon with
   the desired obs value.
   
   Inputs: Host (reciprocal) radial
           Desired course, obs
           Distance from host to beacon
           
   Outputs: Distance in nautical miles */

float RN_get_vor_par_cross_dev(float host_recip_radial, float obs, float srange_nm)
{
   float mbear_host_beac;
   float ang_a;
   float sin_a;
   float cross_dev;

   mbear_host_beac = host_recip_radial;

   ang_a = mbear_host_beac - obs;

   /* Allow for going over 360, eg. 10 - 350 => 20,
                                    350 - 10 => -20 */

   if (ang_a > 180.0f)
      ang_a = ang_a - 360.0f;
   else if (ang_a < -180.0f)
      ang_a += 360.0f;

   /* The sign gives left or right deviation */

   sin_a = (float) sin(ang_a * DEG_TO_RAD);

   cross_dev = srange_nm * sin_a;

   return cross_dev;
}

/*----------------------------------------------------------------------*/
