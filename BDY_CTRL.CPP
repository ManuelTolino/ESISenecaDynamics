/*
File  : bdy_ctrl.c
Prefix: BDY_

Module: Dynamics for Seneca
Created: 11 April 2000

Description: This file handles some of the conditions that
             depend on whether we are inside or outside a
             local airfield boundary
*/

/***************************/
/*   RCS Markers           */
/***************************/

/*
$Header: C:/MASTER/seneca/RCS/bdy_ctrl.cpp 1.3 2000/06/13 11:34:24 COLINJ Exp $
$Log: bdy_ctrl.cpp $
Revision 1.3  2000/06/13 11:34:24  COLINJ
atis pressure is now QNH not QFE. changed sea_lev_press to equal atis_pres.

Revision 1.2  2000/05/10 09:02:14  juliae
Change so that rate of change in pressure applies to global
areas only and within local boundary pressure blends from
Last_global_pres to atis pressure.
Also allow for atis pressure being given as pressure at airfield
height and not above sea level.

Revision 1.1  2000/04/14 16:20:50  juliae
Initial revision

*/

/*----------------------------------------------------------------------*/

/*----------------*/
/* Include Files  */
/*----------------*/

#include <math.h>
#include <stdio.h>

#include "define.h"
#include "Const.h"
#include "nav\dyn_ios.h"

/*----------------*/
/* Defines        */
/*----------------*/

#define FT_TO_M  (1.0 / FT_IN_METRE)

#define LOCAL_COND_HT_BDY            (3500.0 * FT_TO_M)    //Above GROUND level
#define LOCAL_COND_DIST_BDY          (10.0 * METRES_IN_NMILE)

/*----------------*/
/* Typedefs       */
/*----------------*/

/*--------------------*/
/* External Routines  */
/*--------------------*/

/*--------------------*/
/* External Variables */
/*--------------------*/

extern AtisData         IOS_atis;
extern AtmosphericsData DY_atmos;   //Global conditions
extern float            delta_time;

/*----------------*/
/* Globals        */
/*----------------*/

/*-----------------*/
/* Local Variables */    
/*-----------------*/

static float Last_global_pres = 0.0f;

/*--------------------*/
/* Forward References */
/*--------------------*/

static float Pres_atis2sea_level(float atis_pres, float pres_ht);

/*--------------------------------------------------------------------------*/

static int Inside_local_bdy(float hdist, float delta_ht)
{
   if ((delta_ht > LOCAL_COND_HT_BDY) ||
       (hdist > LOCAL_COND_DIST_BDY))
   {
      return FALSE;
   }

   return TRUE;
}

/*--------------------------------------------------------------------------*/

static float Pres_atis2sea_level(float atis_pres, float pres_ht)
{
   float sea_lev_pres;

   //sea_lev_pres = atis_pres + ((pres_ht * (float)FT_IN_METRE)/(float)FEET_IN_MILLIBAR);
   sea_lev_pres = atis_pres;

   return sea_lev_pres;
}

/*--------------------------------------------------------------------------*/

/* Pressure is continually changed from its current value to a target
   value at a global "rate of change" value. The value of the target
   pressure depends on whether we are inside or outside an airfield local
   conditions boundary.
   (Unlike fog and wind, the fade is time based, not distance based. The
   boundary serves ONLY to decide the value of the target pressure)

   Inputs: offsets from ORIGIN of local region: horizontal distance,
           ht = ht above origin ht
           current pressure
   Outputs: new current pressure
*/

void BDY_pres_ctrl (int valid_bdy, float hdist, float delta_ht, float pres_in, float *pres_out)
{
   int   use_local = FALSE;
   float target_pres;
   float rate;
   float max_change;
   float delta_p;
   float fraction1;
   float fraction2;
   float fraction;

   if (valid_bdy)
      use_local = Inside_local_bdy(hdist, delta_ht);

   if (use_local)
   {
      target_pres = Pres_atis2sea_level( (float) IOS_atis.Pressure, (float) IOS_atis.AFheight);

      /* When in local area, the pressure is blended in a linear way from
         the last pressure when in global area to required atis pressure */

      /* If you find the fraction you need due to height, f1 and
         then the fraction you need due to horiz dist, f2, the
         combined fraction required is
         fraction = f1 + f2 - f1*f2
      
         (Alternatively, do the first blend to give an answer
         and use that answer as input to the second blend) */

      fraction1 = (float) (delta_ht / LOCAL_COND_HT_BDY);
      fraction2 = (float) (hdist / LOCAL_COND_DIST_BDY);

      fraction = fraction1 + fraction2 - (fraction1 * fraction2);
      if (fraction > 1.0)
         fraction = 1.0;
      else if (fraction < 0.0)
         fraction = 0.0;

      //Blend
   
      *pres_out = target_pres + fraction * (Last_global_pres - target_pres);
   }
   else
   {
      target_pres = (float) DY_atmos.TargetPressure;

      rate = (float) DY_atmos.RateOfChange;    //mb per hour
      max_change = rate * delta_time / 3600.0f;

      if (target_pres > pres_in)
      {
         delta_p = (target_pres - pres_in);
         if (delta_p < max_change)
            *pres_out = target_pres;
         else
            *pres_out = (pres_in + max_change);
      }
      else
      {
         delta_p = (pres_in - target_pres);
         if (delta_p < max_change)
            *pres_out = target_pres;
         else
            *pres_out = (pres_in - max_change);
      }

      Last_global_pres = *pres_out;
   }   //Global stuff
}

/*--------------------------------------------------------------------------*/

//Needs to be called when we go in to RUN mode AFTER ok data
//received from IOS

void BDY_init_pres()
{
   Last_global_pres = DY_atmos.CurrentPressure;
}

/*--------------------------------------------------------------------------*/

/* (Similar to wind control)
   Sea level temperature (deg C)
   Temperature inputs from IOS may need to be modified depending on whether
   we are inside or outside an airfield local conditions boundary.

   1. If outside this boundary at all, temp is global temp.
   2. If inside the boundary, the temp is a blend of global and local (atis).

   The blend used is a linear blend from outer boundary to origin.
   The boundary is a cylinder.

   Inputs: offsets from ORIGIN of local region: horizontal distance,
           ht = ht above origin ht
   Outputs: temperature
*/

void BDY_temp_ctrl (int valid_bdy, float hdist, float delta_ht, float *temp_out)
{
   float ltemp;
   float gtemp;
   float fraction1;
   float fraction2;
   float fraction;

   if (!valid_bdy)
   {
      *temp_out = (float) DY_atmos.Temperature;
      return;
   }

   if ((delta_ht > LOCAL_COND_HT_BDY) ||
       (hdist > LOCAL_COND_DIST_BDY))
   {
      *temp_out = (float) DY_atmos.Temperature;
      return;
   }
   
   //We are within local conditions region. Need to blend temp

   ltemp = (float) IOS_atis.Temperature;
   gtemp = (float) DY_atmos.Temperature;

   /* If you find the fraction you need due to height, f1 and
      then the fraction you need due to horiz dist, f2, the
      combined fraction required is
      fraction = f1 + f2 - f1*f2
      
      (Alternatively, do the first blend to give an answer
      and use that answer as input to the second blend) */

   fraction1 = (float) (delta_ht / LOCAL_COND_HT_BDY);
   fraction2 = (float) (hdist / LOCAL_COND_DIST_BDY);

   fraction = fraction1 + fraction2 - (fraction1 * fraction2);
   if (fraction > 1.0)
      fraction = 1.0;
   else if (fraction < 0.0)
      fraction = 0.0;

   *temp_out = ltemp + fraction * (gtemp - ltemp);
}

/*--------------------------------------------------------------------------*/
