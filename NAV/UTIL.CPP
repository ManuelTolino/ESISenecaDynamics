/*
File  : util.c
Prefix: UT_

Module: Dynamics for Seneca
Created: 18 February 2000

Description: This file contains general support routines
*/

/***************************/
/*   RCS Markers           */
/***************************/

/*
$Header: C:/MASTER/SENECA/NAV/RCS/util.cpp 1.3 2000/04/17 11:56:24 juliae Exp $
$Log: util.cpp $
Revision 1.3  2000/04/17 11:56:24  juliae
Changes to avoid compiler warnings

Revision 1.2  2000/02/24 16:08:30  juliae
New routine: UT_offset_along_dir

 * Revision 1.1  2000/02/21  20:25:41  juliae
 * Initial revision
 *
*/

/*----------------------------------------------------------------------*/

/*----------------*/
/* Include Files  */
/*----------------*/

#include <math.h>
#include "c:\bae-sims\seneca\fltdy/fmgdefs.h"
#include "c:\bae-sims\seneca/define.h"

/*----------------*/
/* Defines        */
/*----------------*/

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

/*----------------------------------------------------------------*/

/* METHOD: Regard Earth as a perfect sphere.

   Along a line of increasing latitude (ie. vertical line), the distance between
   any 2 points can be found as a fraction of the Earth's circumference.

   Along the equator, the same is true. Along a different line of latitude, a
   scaling factor of cos(lat) is applied to handle the circumference getting
   smaller to 0 at the poles.

   From any point on the Earth you choose to call the origin, you can work out a
   distance horizontal (x) and vertical (y) from that.

   Here, we have a reference lat and lon which we want to correspond to x = 0.0,
   y = 0.0. So, simply work out the delta lat and lon from this point, find
   the corresponding delta distances x and y from that reference point (which
   then are the required x and y, since the reference point is the x,y origin). */


void UT_ll_2_local_xy(double lat, double lon, double ref_lat, double ref_lon, float *x, float *y)
{
   double delta_lat;
   double delta_lon;
   double cos_lat;
   float  delta_x, delta_y;

   delta_lat = lat - ref_lat;   /* Degrees */
   delta_lon = lon - ref_lon;

   /* Allow for wrap, eg. 179 to -179 is a delta of 2,
      359 to 2 is a delta of 3 */
   if (delta_lat > 180.0)
      delta_lat -= 360.0;
   else if (delta_lat < -180.0)
      delta_lat += 360.0;

   if (delta_lon > 180.0)
      delta_lon -= 360.0;
   else if (delta_lon < -180.0)
      delta_lon += 360.0;
   

   cos_lat = cos (lat * DEG_TO_RAD);

   delta_x = (float) (delta_lon * METRES_PER_DEGREE * cos_lat);
   delta_y = (float) (delta_lat * METRES_PER_DEGREE);

   *x = delta_x;
   *y = delta_y;
}

/*----------------------------------------------------------------*/

/* This routine gives the lat/lon position of a point
   specified as an x/y offset from a reference lat/lon position.
   x = East, y = North */

void UT_local_xy_2_ll(float delta_x, float delta_y, double ref_lat, double ref_lon,
                      double *lat1, double *lon1)
{
   double delta_lat;
   double delta_lon;
   double cos_ref_lat;
   double lat, lon;

   cos_ref_lat = cos (ref_lat * DEG_TO_RAD);

   delta_lon = delta_x / (METRES_PER_DEGREE * cos_ref_lat);
   delta_lat = delta_y / METRES_PER_DEGREE;

   lat = delta_lat + ref_lat;   /* Degrees */
   lon = delta_lon + ref_lon;

   /* Allow for wrap */
   if (lon > 180.0)
      lon -= 360.0;
   else if (lon < -180.0)
      lon += 360.0;

   *lat1 = lat;
   *lon1 = lon;
}

/*----------------------------------------------------------------------*/

/* From a start point (x1,y1,z1), direction heading h, this finds
   point (x,y,z) at distance 'dist' from start, along direction line
   with pitch 'p'.
   Input angles in degrees */

void UT_offset_along_dir (float x1, float y1, float z1, float h,
                          float p, float dist, float *x, float *y, float *z)
{
   float  dir_vec_x, dir_vec_y, dir_vec_z;
   double rad_h;
   double rad_p;
   double cos_p;

   rad_h = h * DEG_TO_RAD;
   rad_p = p * DEG_TO_RAD;
   cos_p = cos(rad_p);

   dir_vec_x = (float) (sin(rad_h) * cos_p);
   dir_vec_y = (float) (cos(rad_h) * cos_p);
   dir_vec_z = (float) sin(rad_p);

   *x = x1 + dist * dir_vec_x;
   *y = y1 + dist * dir_vec_y;
   *z = z1 + dist * dir_vec_z;
}

/*----------------------------------------------------------------------*/
