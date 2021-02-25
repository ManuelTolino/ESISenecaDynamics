/*
File: ils_inst.c
Prefix: ILS_

Module: Navigation (Dynamics) for Seneca

Created: 4 January 2000

Description: Calculates ILS Instrument displays
*/

/***************************/
/*   RCS Markers           */
/***************************/

/*
$Header: C:/master/seneca/NAV/RCS/ils_inst.cpp 1.5 2000/05/09 09:14:07 juliae Exp $
$Log: ils_inst.cpp $
Revision 1.5  2000/05/09 09:14:07  juliae
Added support for offset of glide slope transmitter from
runway centre line.

Revision 1.4  2000/04/25 17:24:23  juliae
ILS_valid_ils_signal now checks for valid_vor_ils flag.
ILS_calc_ils_data now allows for a valid glide signal when
ils signal is invalid.

Revision 1.3  2000/03/27 09:57:52  juliae
Active ranges set to 130% of proper ranges. Added VHF
range tests based on alt. Valid signal routine now
returns a measure of signal strength.

 * Revision 1.2  2000/03/02  10:19:11  juliae
 * Renamed host_pos->alt to ht and allow for beacon elevation.
 *
 * Revision 1.1  2000/02/22  00:15:21  juliae
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
#include "c:\bae-sims\seneca\fltdy/define.h"
#include "beacon.h"
#include "b_dials.h"
#include "bc_range.h"

/*-------------------------*/
/* Defines                 */
/*-------------------------*/

#define ILS_MAX_LR_DEV_ANG 2.5f
#define ILS_MAX_GD_DEV_ANG 0.7f

#define ALT_RANGE_TEST 1

#define MAX_SIGNAL 100
#define MIN_SIGNAL 1

#define GLIDE_TX_OFFSET 1

//#define DEBUG_FAIL 1
//#define DEBUG_FAIL2 1

/*-------------------------*/
/* Typedefs                */
/*-------------------------*/

typedef struct {

   float loc_h_range1;     /* Smaller than range2 */
   float loc_h_half_ang1;  /* Greater than half_ang2 */
   float loc_h_range2;     /* Greater (or same) than range1 */
   float loc_h_half_ang2;  /* Smaller (or same) than half_ang1 */
   float loc_v_ang;
   float loc_v_max_alt;
   float glide_h_range;
   float glide_h_half_ang;
   float glide_v_min_ang_fr;
   float glide_v_max_ang_fr;
   
} ils_categ_data;

/*----------------------------*/
/* External Routines          */
/*----------------------------*/

extern double SPH_get_sph_flat_dist_sq (double lat1, double lon1, float ht1,
                                        double lat2, double lon2, float ht2);
extern float SPH_get_sph_heading (double lat1, double lon1, double lat2, double lon2);
extern float BC_make_magnetic(float true_head, float magnetic_var);
extern float BC_get_host_recip_radial(vhf_beac_attr *battr, ll_pos *host_pos);
extern float VOR_get_vhf_alt_range(float beac_elev, float host_ht);

/*----------------------------*/
/* External Variables         */
/*----------------------------*/

/*----------------------------*/
/* Static Variables           */
/*----------------------------*/

static ils_categ_data Ils_cat;

/*----------------------------------------------------------------*/

void ILS_init(void)
{
   Ils_cat.loc_h_range1 = 20.0 * METRES_IN_NMILE;// ILS Range @ height 1
   Ils_cat.loc_h_range2 = 30.0 * METRES_IN_NMILE;// ILS Range @ height 2
   Ils_cat.loc_h_half_ang1 = 35;  /* Degrees */
   Ils_cat.loc_h_half_ang2 = 10;  /* Degrees */
   Ils_cat.loc_v_ang = 7.0f;
   Ils_cat.loc_v_max_alt = (float) (6250.0 / FT_IN_METRE); 
   Ils_cat.glide_h_range = 10.0f * METRES_IN_NMILE;
   Ils_cat.glide_h_half_ang = 8.0f;
   Ils_cat.glide_v_min_ang_fr = 0.45f;   /* Fractions of glide slope */
   Ils_cat.glide_v_max_ang_fr = 1.75f;

   //Make signal available outside true range
   Ils_cat.loc_h_range2  *= RANGE_INC;
   Ils_cat.loc_v_max_alt *= RANGE_INC;
   Ils_cat.glide_h_range *= RANGE_INC;
}

/*----------------------------------------------------------------*/

/* Returns non-zero if host is within valid region for ils left/right
   indication. In this case return a value in range MIN to MAX_SIGNAL
   as a measure of signal strength.
 */

int ILS_valid_ils_signal(vhf_beac_attr *ils_in,
                         ll_pos *host_pos, float *recip_radial)
{
   double dist_sq;
   double hdist;        //Metres
   float  ang;
   float  delta_ang;
   ils_categ_data *cat;
   float  max_range_alt;
   float  host_alt;
   float  alt_range;    //Metres
   double alt_range_sq;
   double sdist_sq;
   float  max;
   float  min;
   float  fraction1;
   float  fraction2;
   float  fraction;
   int    strength;

   if (!ils_in->valid_vor_ils)
      return FALSE;

   /* Get distance between host and localizer, to see if with range.
      This gives horiz distance, since we pass host height as 0.0 */

   dist_sq = SPH_get_sph_flat_dist_sq (host_pos->lat, host_pos->lon, 0.0f,
                                       ils_in->beac_lat,
                                       ils_in->beac_lon,
                                       0.0f);

   hdist = sqrt(dist_sq);

   cat = &Ils_cat;
   if (hdist > cat->loc_h_range2)
   {
#ifdef DEBUG_FAIL
      printf("\nFail 1: hdist = %.2f, loc_h_range2 = %.2f", hdist, cat->loc_h_range2);
#endif
      return FALSE;
   }

   /* Now check altitude difference */
   host_alt = host_pos->ht - ils_in->beac_elev;
   if (host_alt > cat->loc_v_max_alt)
   {
#ifdef DEBUG_FAIL
      printf("\nFail 2: ht = %.2f, ils_elev = %.2f, loc_v_max_alt = %.2f", host_pos->ht, ils_in->beac_elev, cat->loc_v_max_alt);
#endif
      return FALSE;
   }

/*
   Glide slope may still be valid below this?
   if (host_alt < cat->loc_v_min_lat)
      return FALSE;
*/

   /* Find horizontal angular offset from centre line.

      To do this, find heading from localizer to host in lat/lon.
      
      ALTERNATIVELY (maybe quicker) convert host position to
      local cartesian coordinates, using localizer position as
      cartesian origin and find heading of this offset vector.
      
      Then find difference between this heading and approach
      heading. */

   /* Note that approach heading is heading IN TO localizer, so
      find angle from host IN TO localizer too */

   ang = BC_get_host_recip_radial(ils_in, host_pos);

   /* This is the same as bearing from host to ils, if magnetic
      variation is the same at host and ils */
   *recip_radial = ang;

   delta_ang = ang - ils_in->approach_head;
   /* Allow for wrap, eg. 179 to -179 is a delta of 2,
      359 to 2 is a delta of 3 */
   if (delta_ang > 180.0)
      delta_ang -= 360.0;
   else if (delta_ang < -180.0)
      delta_ang += 360.0;
   if (delta_ang < 0.0)
      delta_ang = -delta_ang;

   if (delta_ang > cat->loc_h_half_ang1)
   {
#ifdef DEBUG_FAIL
      printf("\nFail 3: offset from approach = %.2f deg, h_ang = %.2f", delta_ang,
                                                              cat->loc_h_half_ang1);
#endif
      return FALSE;
   }

   /* Further range and angle boundaries depend on one another */
   if (hdist > cat->loc_h_range1)
   {
      /* Under range2 restrictions */
      if (delta_ang > cat->loc_h_half_ang2)
      {
#ifdef DEBUG_FAIL
         printf("\nFail 4: offset from approach = %.2f deg, h_half_ang2 = %.2f",
                                                 delta_ang, cat->loc_h_half_ang2);
#endif
         return FALSE;
      }
   }

   /* Find max valid height at this range (we have already tested for
      max alt) */
   max_range_alt = (float) (hdist * tan (cat->loc_v_ang * DEG_TO_RAD));
   if (host_alt > max_range_alt)
   {
#ifdef DEBUG_FAIL
      printf("\nFail 5: ht = %.2f, ils_elev = %.2f, max_range_alt = %.2f", host_pos->ht, ils_in->beac_elev, max_range_alt);
#endif
      return FALSE;
   }

#ifdef ALT_RANGE_TEST
   /* In addition to simple range tests, range also varies with ht
      due to nature of VHF signals */

   alt_range = VOR_get_vhf_alt_range(ils_in->beac_elev, host_pos->ht);  //Metres
   /* Get slant distance by Pythagoras */
   sdist_sq = (hdist * hdist) + (host_alt * host_alt);
   alt_range_sq = alt_range * alt_range;
   if (sdist_sq > alt_range_sq)
   {
#ifdef DEBUG_FAIL
      printf("\nFail 10: sdist_sq = %.2f, alt_range = %.2f", sdist_sq, alt_range);
#endif
      return FALSE;
   }
#endif



   /* Return an indication of signal strength */
   fraction1 = (float) (hdist / cat->loc_h_range2);

   if (hdist > cat->loc_h_range1)
      fraction2 = delta_ang / cat->loc_h_half_ang2;
   else
      fraction2 = delta_ang / cat->loc_h_half_ang1;

   fraction = fraction1 + fraction2 - (fraction1 * fraction2);
   if (fraction > 1.0)
      fraction = 1.0;
   else if (fraction < 0.0)
      fraction = 0.0;

   max = (float) MAX_SIGNAL;
   min = (float) MIN_SIGNAL;
   strength = (int) (max - fraction * (max - min));
   if (strength < MIN_SIGNAL)
      strength = MIN_SIGNAL;
   return strength;
}

/*----------------------------------------------------------------*/

void ILS_get_to_from_flag(float approach_head, float bear_host_beacon, char *to_flag)
{
   float delta_ang;

   /* Compare bearing from host to beacon with desired approach
      If mag of diff < 90, set 'to' else 'from' */

   delta_ang = bear_host_beacon - approach_head;
   /* Allow for wrap, eg. 179 to -179 is a delta of 2,
      359 to 2 is a delta of 3 */
   if (delta_ang > 180.0)
      delta_ang -= 360.0;
   else if (delta_ang < -180.0)
      delta_ang += 360.0;

   if (delta_ang < 0.0)
   {
      delta_ang = -delta_ang;
   }

   if (delta_ang <= 90.0)
   {
      *to_flag = TO_FLAG;
   }
   else
   {
      *to_flag = FROM_FLAG;
   }
}

/*----------------------------------------------------------------*/

/* Left/right deviation in angular terms.
   Deviation relates to difference between radial host is on and
   desired radial.
   This routine is used for both ILS and VOR displays.

   INPUTS:  The input headings need to be consistent as to whether
            they are with respect to magnetic or true north.

   OUTPUTS: lr_dev: This is given as a signed fraction of the
                    maximum deflection from the centre */

void ILS_get_lr_adev(float approach_head, float bear_host_beacon,
                     float max_dev_ang,
                     float *lr_dev, char *to_flag)
{
   float delta_ang;
   int   pos_flag;
   int   right_flag;

   /* Compare bearing from host to beacon with desired approach
      If mag of diff < 90, set 'to' else 'from' */

   delta_ang = bear_host_beacon - approach_head;
   /* Allow for wrap, eg. 179 to -179 is a delta of 2,
      359 to 2 is a delta of 3 */
   if (delta_ang > 180.0)
      delta_ang -= 360.0;
   else if (delta_ang < -180.0)
      delta_ang += 360.0;

   pos_flag = TRUE;
   if (delta_ang < 0.0)
   {
      delta_ang = -delta_ang;
      pos_flag = FALSE;
   }


   right_flag = TRUE;
   *to_flag = TO_FLAG;

   if (delta_ang <= 90.0)
   {
      *to_flag = TO_FLAG;       /* To */
      if (pos_flag)
      {
         /* We are currently on a radial to the left of selected one
            (approaching beacon) so need to turn right */
         right_flag = TRUE;     /* Right */
      }
      else
      {
         right_flag = FALSE;    /* Left */
      }
   }
   else
   {
      *to_flag = FROM_FLAG;      /* From */

      /* ******* VOR ONLY *******
         Host is nearer the reciprocal radial. This is indicated by
         setting the "from" flag. The deviation is measured to this
         reciprocal radial, so re-calculate delta_ang */
         
      delta_ang = bear_host_beacon - (approach_head + 180.0f);
      while (delta_ang > 180.0)
         delta_ang -= 360.0;
      while (delta_ang < -180.0)
         delta_ang += 360.0;

      pos_flag = TRUE;
      if (delta_ang < 0.0)
      {
         delta_ang = -delta_ang;
         pos_flag = FALSE;
      }

      if (pos_flag)
      {
         right_flag = FALSE;    /* Left */
      }
      else
      {
         right_flag = TRUE;     /* Right */
      }
   }

   if (delta_ang < max_dev_ang)
   {
      if (right_flag)
         *lr_dev = delta_ang/max_dev_ang;
      else
         *lr_dev = -delta_ang/max_dev_ang;
   }
   else
   {
      if (right_flag)
         *lr_dev = 1.0;
      else
         *lr_dev = -1.0;
   }
}

/*----------------------------------------------------------------*/

/* Glideslope deviation.
   Return FALSE, if invalid, eg. due to outside valid region.

   OUTPUTS: glide_dev: This is given as a signed fraction of the
                       maximum deflection from the centre */

int Get_glide_dev(vhf_beac_attr *ils_in,
                  double host_lat, double host_lon, float host_ht,
                  float max_dev_ang, float *glide_dev)
{
   double dist_sq;           //Metres sq
   float  hdist_host_glide;  //Metres
   float  ang;               //Degrees
   float  delta_h_ang;       //Degrees
   ils_categ_data *cat;
   float  tan_pitch;
   float  pitch;             //Degrees
   int    pos_flag;
   float  delta_pit;         //Degrees
   float  max_pitch;         //Degrees
   float  min_pitch;         //Degrees
#ifdef GLIDE_TX_OFFSET
   float  r_delta_h_ang;     //In radians
   float  ap_dist;           //Metres
   float  max_cross_dist;    //Metres
   float  cross_dist;        //Metres
#endif

   /* Find distance from host to glide transmitter.
      This gives horiz distance, since we pass height as 0.0 */

   dist_sq = SPH_get_sph_flat_dist_sq (host_lat, host_lon, 0.0f,
                                       ils_in->glide_tx_lat,
                                       ils_in->glide_tx_lon,
                                       0.0f);

   hdist_host_glide = (float) sqrt(dist_sq);

   cat = &Ils_cat;
   if (hdist_host_glide > cat->glide_h_range)
   {
#ifdef DEBUG_FAIL2
      printf("\nFail 6: host hdist = %.2f, glide horiz range = %.2f", hdist_host_glide,
                                                                cat->glide_h_range);
#endif
      return FALSE;
   }

   /* Test if within horiz angular range.
      To do this, find heading from localizer to host in lat/lon.
      Then find difference between this heading and approach
      heading. */

   ang = SPH_get_sph_heading (host_lat, host_lon,
                              ils_in->glide_tx_lat, ils_in->glide_tx_lon);

   ang = BC_make_magnetic(ang, ils_in->magnetic_var);

   delta_h_ang = ang - ils_in->approach_head;
   /* Allow for wrap, eg. 179 to -179 is a delta of 2,
      359 to 2 is a delta of 3 */
   if (delta_h_ang > 180.0)
      delta_h_ang -= 360.0;
   else if (delta_h_ang < -180.0)
      delta_h_ang += 360.0;
   if (delta_h_ang < 0.0)
      delta_h_ang = -delta_h_ang;

   if (delta_h_ang > cat->glide_h_half_ang)
   {
#ifdef GLIDE_TX_OFFSET
      if (delta_h_ang < 90.0f)
      {
         /* To allow for the glide slope transmitter being offset from the
            runway centreline, we add the offset to the valid cross distance,
            so the valid horizontal beam is extended on both sides by
            the offset distance */

         /* Find distance along approach direction of host from transmitter */
         r_delta_h_ang = (float)(delta_h_ang * DEG_TO_RAD);
         ap_dist = (float)(hdist_host_glide * cos(r_delta_h_ang));   //-ve if ang > 90

         /* At this approach distance, find max horiz cross distance based on
            max horiz half angle */
         max_cross_dist = (float)(ap_dist * tan(cat->glide_h_half_ang * DEG_TO_RAD));

         /* Find actual cross distance from transmitter to host */
         cross_dist = (float)(hdist_host_glide * sin(r_delta_h_ang));

         /* Increase max cross dist to allow for glide offset */
         max_cross_dist += ils_in->glide_offset;

         if (cross_dist > max_cross_dist)
         {
#ifdef DEBUG_FAIL2
            printf("\nFail 7.1: cross_dist = %.2f, max_cross_dist = %.2f (at app dist = %.2f) (metres)",
                                                                      cross_dist, max_cross_dist, ap_dist);
#endif
            return FALSE;
         }
      }
      else
      {
#ifdef DEBUG_FAIL2
         printf("\nFail 7.2: delta_h_ang = %.2f, so past tx", delta_h_ang);
#endif
         return FALSE;
      }
#else

#ifdef DEBUG_FAIL2
      printf("\nFail 7: delta_h_ang = %.2f, cat->glide_h_half_ang = %.2f", delta_h_ang, cat->glide_h_half_ang);
#endif
      return FALSE;

#endif
   }

   /* Find pitch angle of host above glide transmitter */

   tan_pitch =  (host_ht - ils_in->glide_elev) / hdist_host_glide;
   pitch = (float) (atan(tan_pitch) * RAD_TO_DEG);

   max_pitch = cat->glide_v_max_ang_fr * ils_in->glideslope_ang;
   if (pitch > max_pitch)
   {
#ifdef DEBUG_FAIL2
      printf("\nFail 8: pitch = %.2f, max_pitch = %.2f", pitch, max_pitch);
#endif
      return FALSE;
   }
   min_pitch = cat->glide_v_min_ang_fr * ils_in->glideslope_ang;
   if (pitch < min_pitch)
   {
#ifdef DEBUG_FAIL2
      printf("\nFail 9: pitch = %.2f, min_pitch = %.2f", pitch, min_pitch);
#endif
      return FALSE;
   }

   delta_pit = pitch - ils_in->glideslope_ang;
   pos_flag = FALSE;
   if (delta_pit < 0.0)
   {
      /* Host craft is below glideslope, do deflection shows UP */
      pos_flag = TRUE;
      delta_pit = -delta_pit;
   }

   if (delta_pit < max_dev_ang)
   {
      if (pos_flag)
         *glide_dev = delta_pit/max_dev_ang;
      else
         *glide_dev = -delta_pit/max_dev_ang;
   }
   else
   {
      if (pos_flag)
         *glide_dev = 1.0;
      else
         *glide_dev = -1.0;
   }

   return TRUE;
}

/*----------------------------------------------------------------*/

/* This routine does NOT directly update IO_Sen_out structure since it
   it is used by both NAV2 and HSI dials */

void ILS_calc_ils_data (vhf_beac_attr *ils_in, ll_pos *host_pos, int valid_ils,
                        float recip_radial, ils_inst_out_data *ils_out)
{
   float mbear_host_ils;   /* Magnetic bearing, in degrees */
   char  cdummy;
   int   valid_glide;
   float lr_dev;
   float glide_dev;

   valid_glide = FALSE;
   if (valid_ils)
   {
      mbear_host_ils = recip_radial;
      ILS_get_lr_adev(ils_in->approach_head, mbear_host_ils, ILS_MAX_LR_DEV_ANG,
                      &lr_dev, &cdummy);

   }

   if (ils_in->valid_glide_tx)
   {
      valid_glide = Get_glide_dev(ils_in,
                                  host_pos->lat, host_pos->lon, host_pos->ht,
                                  ILS_MAX_GD_DEV_ANG, &glide_dev);
   }

   ils_out->valid_loc = valid_ils;
   ils_out->valid_glide = valid_glide;
   if (valid_ils)
      ils_out->lt_rt_dev = lr_dev;
   else
      ils_out->lt_rt_dev = 0.0;
   if (valid_glide)
      ils_out->vert_dev = glide_dev;
   else
      ils_out->vert_dev = 0.0;
}

/*----------------------------------------------------------------*/
