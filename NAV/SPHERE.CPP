/*
File: sphere.c
Prefix: SPH_

Module: Dynamics for Seneca
Created: December 1999

Description: This file contains functions concerning spherical coordinates.
---------------------*/

/***************************/
/*   RCS Markers           */
/***************************/

/*
$Header: C:/master/seneca/NAV/RCS/sphere.cpp 1.2 2000/05/25 15:28:01 juliae Exp $
$Log: sphere.cpp $
Revision 1.2  2000/05/25 15:28:01  juliae
Safeguard against cos being out of range.

Revision 1.1  2000/02/22 00:21:22  juliae
Initial revision

*/

/*----------------------------------------------------------------------*/

/*----------------*/
/* Include Files  */
/*----------------*/

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "c:\bae-sims\seneca/fltdy/fmgdefs.h"
#include "c:\bae-sims\seneca/define.h"

/*----------------*/
/* Defines        */
/*----------------*/

#define TRUE  1
#define FALSE 0
#define CWISE_DIR  TRUE
#define ACWISE_DIR FALSE

/*----------------*/
/* Typedefs       */
/*----------------*/

/*--------------------*/
/* Forward References */
/*--------------------*/

/*----------------------------------------------------------------*/

/* Gives Cartesian coordinates, in metres, of lat/lon/ht point,
   where Cartesian coordinate frame has its origin at centre
   of the Earth with Z vertical.
   X = R sin (90-lat) cos (lon) = R cos (lat) cos (lon)
   Y = R sin (90-lat) sin (lon) = R cos (lat) sin (lon)
   Z = R cos (90-lat)           = R sin (lat)
*/

static void Get_earth_centre_cart (double rlat, double rlon, float ht,
                                   double *X, double *Y, double *Z )
{
   double cos_lat, sin_lat;
   double cos_lon, sin_lon;

   cos_lat = cos (rlat);
   cos_lon = cos (rlon);
   sin_lat = sin (rlat);
   sin_lon = sin (rlon);

   *X = ((EARTH_RAD + ht) * cos_lat * cos_lon);
   *Y = ((EARTH_RAD + ht) * cos_lat * sin_lon);
   *Z = ((EARTH_RAD + ht) * sin_lat);
}

/*----------------------------------------------------------------*/

/* Finds distance between 2 points given in lat/lon coordinates on
   the ground, allowing for curvature of Earth.

   Method: Convert the spherical coordinates in to an Earth centred
   Cartesian system:
   
   X = R sin (90-lat) cos (lon) = R cos (lat) cos (lon)
   Y = R sin (90-lat) sin (lon) = R cos (lat) sin (lon)
   Z = R cos (90-lat)           = R sin (lat)

   Then find angle between the 2 vectors (lines from centre to
   each point). Then use this to find arc length of arc
   of this angle on a circle of radius R.
   NB. This ignores height above ground.

   Units: metres
*/

double SPH_get_sph_curve_distance (double lat1, double lon1, double lat2, double lon2)
{
   double rlat1, rlon1;    /* Radians */
   double rlat2, rlon2;
   double X1, Y1, Z1;
   double X2, Y2, Z2;
   double dot_prod;
   double cos_ang;
   double angle;
   double arc_length;

   rlat1 = lat1 * DEG_TO_RAD;
   rlon1 = lon1 * DEG_TO_RAD;
   rlat2 = lat2 * DEG_TO_RAD;
   rlon2 = lon2 * DEG_TO_RAD;

   Get_earth_centre_cart (rlat1, rlon1, 0.0, &X1, &Y1, &Z1);
   Get_earth_centre_cart (rlat2, rlon2, 0.0, &X2, &Y2, &Z2);

   dot_prod = (X1 * X2) + (Y1 * Y2) + (Z1 * Z2);
   /* Magnitude of each vector is EARTH_RAD */
   cos_ang = dot_prod /(EARTH_RAD * EARTH_RAD);

   if (cos_ang > 1.0f)
      cos_ang = 1.0f;
   else if (cos_ang < -1.0f)
      cos_ang = -1.0f;
   
   angle = acos(cos_ang);   /* Radians */

   /* Arc length of arc subtended by "angle" on circle of
      radius EARTH_RAD */

   arc_length = EARTH_RAD * angle;
   
   return arc_length;
}

/*----------------------------------------------------------------*/

/* Finds straight slant distance between 2 points given in
   lat/lon/ht coordinates.

   Method: Convert the spherical coordinates in to an Earth centred
   Cartesian system:
   
   X = R sin (90-lat) cos (lon) = R cos (lat) cos (lon)
   Y = R sin (90-lat) sin (lon) = R cos (lat) sin (lon)
   Z = R cos (90-lat)           = R sin (lat)

   Then use Pythagoras. This ignores curvature of the Earth but
   does allow for height above the ground.

   Units: metres squared
*/

double SPH_get_sph_flat_dist_sq (double lat1, double lon1, float ht1,
                                 double lat2, double lon2, float ht2)
{
   double rlat1, rlon1;    /* Radians */
   double rlat2, rlon2;
   double X1, Y1, Z1;
   double X2, Y2, Z2;
   double dX, dY, dZ;
   double dist_sq;

   rlat1 = lat1 * DEG_TO_RAD;
   rlon1 = lon1 * DEG_TO_RAD;
   rlat2 = lat2 * DEG_TO_RAD;
   rlon2 = lon2 * DEG_TO_RAD;

   Get_earth_centre_cart (rlat1, rlon1, ht1, &X1, &Y1, &Z1);
   Get_earth_centre_cart (rlat2, rlon2, ht2, &X2, &Y2, &Z2);

   dX = X1 - X2;
   dY = Y1 - Y2;
   dZ = Z1 - Z2;

   dist_sq = (dX * dX) + (dY * dY) + (dZ * dZ);

   return dist_sq;
}

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

/* For a plane defined by r.n = dist, find the point on the plane
   which is the projection of p along the plane's normal.
   ASSUME normal has unit length */

void Project_to_plane (double px, double py, double pz,
                       double nx, double ny, double nz,
                       double dist, double *x, double *y, double *z)
{
   double dot_prod;
   double scale;

   /* Required point is
      r = p + (scale * n), where r must satisfy r.n = dist

      This gives
      scale = ( dist - (p.n) ) / mag_n_sq
   */

   dot_prod = (px * nx) + (py * ny) + (pz * nz);

   scale = (dist - dot_prod);
   *x = (px + (scale * nx));
   *y = (py + (scale * ny));
   *z = (pz + (scale * nz));
}

/*--------------------------------------------------------------------------*/

void Normalize_dvec(double norm[])
{
   double mag;

   mag =  (norm[0] * norm[0]) +
          (norm[1] * norm[1]) +
          (norm[2] * norm[2]);

   mag = sqrt(mag);
   if (mag > 0.0001)
   {
      norm[0] = norm[0] / mag;
      norm[1] = norm[1] / mag;
      norm[2] = norm[2] / mag;
   }
}

/*--------------------------------------------------------------------------*/

/* Finds the angle between the given vectors.
   If direction of turn from vec1 to vec2 is CLOCKWISE when
   view along the normal to the surface they are in, angle is +ve.
   norm is the normal to the surface you are looking DOWN on when considering
   the direction of the turn, eg. for standard rotation in the xy plane,
   norm is the z-axis.
   Range: 0 to 180 */

float Get_angle_between_dvec(double vec1[], double vec2[])
{
   double  mag1, mag2;
   double  dot_prod;
   double  cos_ang;
   float   angle;

   mag1 = (vec1[0] * vec1[0]) + (vec1[1] * vec1[1]) + (vec1[2] * vec1[2]);
   mag1 = sqrt(mag1);

   if (mag1 < 0.0001)
   {
      return 0.0;
   }
   mag2 = (vec2[0] * vec2[0]) + (vec2[1] * vec2[1]) + (vec2[2] * vec2[2]);
   mag2 = sqrt(mag2);

   if (mag2 < 0.0001)
   {
      return 0.0;
   }

   dot_prod = vec1[0] * vec2[0] +
              vec1[1] * vec2[1] +
              vec1[2] * vec2[2];

   cos_ang = dot_prod /(mag1 * mag2);

   if (cos_ang > 1.0f)
      cos_ang = 1.0f;
   else if (cos_ang < -1.0f)
      cos_ang = -1.0f;

   angle = (float) (acos(cos_ang) * RAD_TO_DEG);   /* Value in range 0 to 180 */

   return angle;
}

/*--------------------------------------------------------------------------*/

void Cross_prodd (double a[], double b[], double c_prod[])
{
   c_prod[0] = a[1] * b[2] - a[2] * b[1];
   c_prod[1] = a[2] * b[0] - a[0] * b[2];
   c_prod[2] = a[0] * b[1] - a[1] * b[0];
}

/*--------------------------------------------------------------------------*/

/* Return TRUE if direction of turn from vec1 to vec2 is CLOCKWISE when
   view along the normal to the surface they are in.
   norm is the normal to the surface you are looking DOWN on when considering
   the direction of the turn, eg. for standard rotation in the xy plane,
   norm is the z-axis */

int Get_turn_dir (double vec1[], double vec2[], double norm[])
{
   double c_prod[3];
   double dot_prod;

   Cross_prodd (vec1, vec2, c_prod);

   dot_prod = c_prod[0] * norm[0] +
              c_prod[1] * norm[1] +
              c_prod[2] * norm[2];

   /* Anticlockwise turn gives a +ve dot product */
   if (dot_prod <= 0.0)
      return CWISE_DIR;    /* Clockwise */
   else
      return ACWISE_DIR;
}

/*--------------------------------------------------------------------------*/

/* Find heading from point1 to point 2 (+ve clockwise) */
/* Heading from point1 to point2 is defined as the angle between
   the great circleS passing through p1 and North and through p1
   and p2.

   Output: Heading in the range 0 to 360 */ 

float SPH_get_sph_heading (double lat1, double lon1, double lat2, double lon2)
{
   double rlat1, rlon1;    /* Radians */
   double rlat2, rlon2;
   double X1, Y1, Z1;
   double X2, Y2, Z2;
   double px2, py2, pz2;
   double pxn, pyn, pzn;
   double vec1[3];
   double vec2[3];
   float  angle;
   double norm[3];

   rlat1 = lat1 * DEG_TO_RAD;
   rlon1 = lon1 * DEG_TO_RAD;
   rlat2 = lat2 * DEG_TO_RAD;
   rlon2 = lon2 * DEG_TO_RAD;

   Get_earth_centre_cart (rlat1, rlon1, 0.0, &X1, &Y1, &Z1);
   Get_earth_centre_cart (rlat2, rlon2, 0.0, &X2, &Y2, &Z2);

   /* Want to project point2 on to the tangent plane at point1.
      The normal of this plane is the vector of point1.
      The plane equation distance is EARTH_RAD, WHEN normal
      is unitized!! */

   norm[0] = X1;
   norm[1] = Y1;
   norm[2] = Z1;
   Normalize_dvec(norm);

   Project_to_plane (X2, Y2, Z2, norm[0], norm[1], norm[2], EARTH_RAD, &px2, &py2, &pz2);

   /* Also want to project the North pole (0,0,R) */
   Project_to_plane (0.0, 0.0, EARTH_RAD, norm[0], norm[1], norm[2], EARTH_RAD, &pxn, &pyn, &pzn);

   /* Get the vector from p1 to projected north */
   vec1[0] = pxn - X1;
   vec1[1] = pyn - Y1;
   vec1[2] = pzn - Z1;

   /* Get the vector from p1 to projected p2 */
   vec2[0] = px2 - X1;
   vec2[1] = py2 - Y1;
   vec2[2] = pz2 - Z1;

   /* Find angle between these 2 (direction matters) */
   angle = Get_angle_between_dvec(vec1, vec2);
   norm[0] = X1;
   norm[1] = Y1;
   norm[2] = Z1;
   if (Get_turn_dir (vec1, vec2, norm) == ACWISE_DIR)
   {
      angle = -angle;
      /* Put in range 0 to 360 */
      angle += 360.0;
   }

   return angle;
}

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/
