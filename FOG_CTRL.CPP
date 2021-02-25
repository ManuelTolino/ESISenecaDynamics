/*
File  : fog_ctrl.c
Prefix: FC_

Module: Seneca Dynamics
Created: 22 March 2000

Description: This file handles fog and fog changes
*/

/***************************/
/*   RCS Markers           */
/***************************/

/*
$Header: C:/Master/seneca/RCS/fog_ctrl.cpp 1.4 2000/06/15 14:32:22 ColinJ Exp $
$Log: fog_ctrl.cpp $
Revision 1.4  2000/06/15 14:32:22  ColinJ
Changed local fog to only work below cloud. global is used above.
IOS change has been implemented.

Revision 1.3  2000/06/01 10:21:56  juliae
Changed fog (and cloud) local boundary to 20nm, with 17nm
blend start.
In fog_ctrl, correction to deciding if above cloud top.
Changed cloud control to support single cloud top in global
data, default base in global data, modified to be with respect
to local airfield, or specific local base. No cloud blend done
now for clouds turning on or off. Check_cloud_change now
pass cloud top and base separately.

Revision 1.2  2000/05/30 14:53:01  juliae
Blend cloud changes (supporting routine FC_reset_cloud).
FC_fog_ctrl now uses the global cloud data set up by
FC_cloud_ctrl instead of Atis/Atmos values.
2 methods of handling clouds (via compiler switch CLD_OLD).

Revision 1.1  2000/04/14 17:41:16  juliae
Initial revision

*/

/*----------------------------------------------------------------------*/

/*----------------*/
/* Include Files  */
/*----------------*/
#include <stdio.h>
#include "define.h"
#include "nav\dyn_ios.h"
#include "fltdy/fmgdefs.h"

/*----------------*/
/* Defines        */
/*----------------*/

#define FOG_OUT_VIS 0.01f
#define DEFAULT_VIS 50000.0f

#define FT_TO_M  (1.0 / FT_IN_METRE)

//These may need to be changed on line to ensure no
//airfields have overlapping fog boundaries

#define LOCAL_COND_HT_BDY            (float)(3500.0 * FT_TO_M)    //Above GROUND level
#define START_LOCAL_COND_BLEND_HT    (float)(3000.0 * FT_TO_M)

#define START_LOCAL_COND_BLEND_DIST  (float)(17.0 * METRES_IN_NMILE)   //9
#define LOCAL_COND_DIST_BDY          (float)(20.0 * METRES_IN_NMILE)   //10

#define LOCAL_FOG_HT_BDY         LOCAL_COND_HT_BDY
#define START_LOCAL_FOG_BLEND_HT START_LOCAL_COND_BLEND_HT

#define START_FOG_BLEND_DIST  START_LOCAL_COND_BLEND_DIST

#define FOG_BDY_DIST          LOCAL_COND_DIST_BDY

#define START_FOG_OUT_HT      4000.0f   //Rick would like 1500
#define FULL_FOG_OUT_HT       5000.0f   //Rick would like 1700

#define MAX_CLOUD_DIST_CHANGE_RATE  20.0f  //Metres per sec
#define SNAP_CLOUD_ON_ABOVE_HT   50.0f     //Metres above host
#define SNAP_CLOUD_OFF_ABOVE_HT  20.0f     //Metres above host

#define MIN_CLD_THICKNESS (50.0f / (float) FT_IN_METRE)

#define BLEND_CLOUD_CHANGE 1
#define USE_GLOBAL_ABOVE_CLD 1
#define WHEN_IOS_CHANGED 1
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
extern AtmosphericsData DY_atmos;
extern STATES           states;
extern float            delta_time;

/*----------------*/
/* Globals        */
/*----------------*/

float FC_vis = DEFAULT_VIS;
int   FC_cld_on = FALSE;
float FC_cld_base = 100.0f;
float FC_cld_thickness = 20.0f;

/*-----------------*/
/* Local Variables */    
/*-----------------*/

static int   Last_cld_data_ok = FALSE;
static int   Last_cld_on = FALSE;
static float Last_cld_base;
static float Last_cld_thickness;

/*--------------------*/
/* Forward References */
/*--------------------*/

static void Check_cloud_change(int target_cld_on, float target_cld_base, float target_cld_thickness,
                               int *cld_on, float *cld_base, float *cld_thickness);

/*--------------------------------------------------------------------------*/

/* Fog inputs from IOS may need to be modified depending on whether
   we are inside or outside an airfield fog boundary or in the boundary
   fade region.
   The fog boundary is a cylinder with 2 heights, 1 for global/local
   conditions and 1 for fogging out. Horizontally, the fog_out
   range is the same as global/local boundary range.

   1. If outside this boundary horizontally AND below any cloud, or
      with no cloud, the fog is dense (fog out).
   2. If above the fade boundary vertically AND below any cloud, or
      with no cloud, the fog is dense (fog out).
   3. If above the local boundary vertically (but below fade ht), the
      fog is as the global value (regardless of any cloud position)
   4. If outside the boundary horizontally AND above cloud, the fog is as
      the global value.
   5. Inside the boundary, the fog is either the global value or a
      local override value. Near the boundary edges, the fog fades
      (if necessary) from outside to inside fog.

   If you are above cloud, there is no fog out but may need to blend
   between global and local over-ride values.

   Note that the Visual software handles the interaction of active
   fog with any clouds. This routine just works out what the "active"
   fog should be set to to handle fog out with fog boundary.

   Note that I do not want to constantly send fog requests to the
   Visual.

   Inputs: offsets from ORIGIN of fog region: horizontal distance,
           ht = ht ABOVE GROUND LEVEL
           FC_cld_base, FC_cld_thickness, FC_cld_on

   REQUIREMENTS: FC_cloud_ctrl() has been called before.
*/

void FC_fog_ctrl (int valid_bdy, float hdist, float delta_ht)
{

#ifdef USE_GLOBAL_ABOVE_CLD
   float local_fog_ht_bdy;
   float start_local_fog_blend_ht;
   local_fog_ht_bdy = LOCAL_FOG_HT_BDY;
   start_local_fog_blend_ht = START_LOCAL_FOG_BLEND_HT;
#endif

   float fraction;
   float outside_vis;
   float inside_vis;
   float cloud_top_ht;
   int   above_cld;
   float host_ht;

   above_cld = FALSE;
   if (FC_cld_on)
   {
      cloud_top_ht = FC_cld_base + FC_cld_thickness;

      host_ht = (float) IOS_atis.AFheight + delta_ht;   //AMSL
      if (host_ht > cloud_top_ht)
         above_cld = TRUE;
   }


   /* First detect all non-blend cases */
   
   if (!valid_bdy)
   {
#ifdef DEBUG
      printf("\nFC: no valid_bdy");
#endif
      if (above_cld)
         FC_vis = DY_atmos.Vis;
      else
         FC_vis = (float) FOG_OUT_VIS;
      return;
   }


   if (delta_ht > FULL_FOG_OUT_HT)
   {
#ifdef DEBUG
printf("\nFC: above FOG_OUT");
#endif
      if (above_cld)   //Assume this is higher than global/local ht boundary
         FC_vis = DY_atmos.Vis;
      else                       //Case 2.
         FC_vis = FOG_OUT_VIS;
      return;
   }

#ifdef USE_GLOBAL_ABOVE_CLD
   if(FC_cld_on)
   {
      /* if there is a local cloud, the local cloud top acts as the "local_fog_ht_bdy"
      and the cloud base acts as the "start_local_fog_blend_ht" instead of using 
      the standard defines */

      if(valid_bdy)
      {
         /* note these values are only used if hdist <= FOG_BDY_DIST */
         local_fog_ht_bdy = cloud_top_ht - (float) IOS_atis.AFheight; // AGL
         start_local_fog_blend_ht = FC_cld_base - (float) IOS_atis.AFheight; //AGL
      }
   }
#endif

   if ((hdist < START_FOG_BLEND_DIST) &&
       (delta_ht < start_local_fog_blend_ht))     //Case 5a
   {
      //Does this airfield have a local override?
      if (!IOS_atis.UseGlobalVis)
         FC_vis = (float) IOS_atis.Visibility;
      else
         FC_vis = DY_atmos.Vis;
#ifdef DEBUG
printf("\nFC: INSIDE ok");
#endif
      return;
   }

   if (hdist > FOG_BDY_DIST)     //Case 1 or 4.
   {
#ifdef DEBUG
printf("\nFC: outside FOG_BDY, hdist = %f", hdist);
#endif
      if (!above_cld)                //Case 1.
         FC_vis = FOG_OUT_VIS;
      else                           //Case 4.
         FC_vis = DY_atmos.Vis;
      return;
   }
   else
   {
      /* Need to know the effect of our height in this region */

      if (delta_ht < start_local_fog_blend_ht)
      {
         //Does this airfield have a local override?
         if (!IOS_atis.UseGlobalVis)
            FC_vis = (float) IOS_atis.Visibility;
         else
            FC_vis = DY_atmos.Vis;
      }
      else if (delta_ht < local_fog_ht_bdy)   //In local/global blend region
      {
         //If there is no local over-ride, there is no vertical blending to do
         if (IOS_atis.UseGlobalVis)
            FC_vis = DY_atmos.Vis;
         else
         {
            outside_vis = DY_atmos.Vis;
            inside_vis = (float) IOS_atis.Visibility;

            fraction =(float) ( (delta_ht - start_local_fog_blend_ht) /
                       (local_fog_ht_bdy - start_local_fog_blend_ht));

            FC_vis = inside_vis + fraction * (outside_vis - inside_vis);
         }
      }
      else
         FC_vis = DY_atmos.Vis;
      
      if (delta_ht > START_FOG_OUT_HT)   /* Fade out region */
      {
         /* Fog out due to height. Ignore edges where there
            should be an extra reduction due to horizontal range
            (done afterwards). */

         outside_vis = FOG_OUT_VIS;

         inside_vis = FC_vis;   //Calculated above

         fraction = (float) ((delta_ht - START_FOG_OUT_HT) /
                    (FULL_FOG_OUT_HT - START_FOG_OUT_HT));

         FC_vis = inside_vis + fraction * (outside_vis - inside_vis);

         /* Do not return here because this may be modified by xy */
      }



      if (hdist < START_FOG_BLEND_DIST)   // No horiz blend
      {
         if (delta_ht <= start_local_fog_blend_ht)
         {
            //Does this airfield have a local override?
            if (!IOS_atis.UseGlobalVis)
               FC_vis = (float) IOS_atis.Visibility;
            else
               FC_vis = DY_atmos.Vis;
         }
         else
         {
            /* Use the value already calculated */
            ;
         }
#ifdef DEBUG
printf("\nFC: INSIDE HORIZ, BLEND due to ht, vis = %f", FC_vis);
#endif
         return;
      }
      else   /* Horizontal Fade region */
      {
         inside_vis = FC_vis;   //Calculated above
         outside_vis = FOG_OUT_VIS;

         fraction = (float) ((hdist - START_FOG_BLEND_DIST) /
                    (FOG_BDY_DIST - START_FOG_BLEND_DIST));

         FC_vis = inside_vis + fraction * (outside_vis - inside_vis);

#ifdef DEBUG
printf("\nFC: horiz & vert BLEND, vis = %f", FC_vis);
#endif
         return;
      }
   }
}

/*--------------------------------------------------------------------------*/

/* Selects current cloud from global or local data depending on
   where host craft is.
   Method: Cloud top specified in global structure
           Default cloud base specified in global structure */

void FC_cloud_ctrl (int valid_bdy, float hdist, float delta_ht)
{
   int   use_global = TRUE;
   int   in_local = FALSE;
   int   target_cld_on;
   float target_cld_base;
   float target_cld_top;
   float cld_top;   //ASL

   in_local = FALSE;

   if (!valid_bdy)   
   {
      use_global = TRUE;
   }
   else   //Need to know whether we are in local area
   {
      if (hdist > LOCAL_COND_DIST_BDY)
         use_global = TRUE;
#ifdef OLD_CLD
      else if (delta_ht > LOCAL_COND_HT_BDY)
         use_global = TRUE;
#endif
      else
      {
         use_global = FALSE;
         in_local = TRUE;
      }
   }

#ifdef WHEN_IOS_CHANGED
   if (in_local)
   {
      if (IOS_atis.UseGlobalCloud)
      {
         use_global = TRUE;
      }
   }
#endif

   //Cloud is either on for both global and local or off, we disallow
   //a mixture
   target_cld_on = DY_atmos.CldOn;
   target_cld_top = DY_atmos.CloudThk;   //ASL. Need to rename variable to Top

   if (use_global)   //May be inside local area
   {
#ifdef WHEN_IOS_CHANGED
      target_cld_base = DY_atmos.CloudBase;   //Specified AGL
#else
      target_cld_base = DY_atmos.CloudThk - MIN_CLD_THICKNESS;   //Specified AGL
#endif
      if (in_local)
      {
         target_cld_base += (float) IOS_atis.AFheight;
      }
   }
   else
   {
      target_cld_base = (float) IOS_atis.CldBase;
   }

   //Check that the top is above the base by MIN_CLD_THICKNESS
   if (target_cld_top < (target_cld_base + MIN_CLD_THICKNESS))
      target_cld_top = target_cld_base + MIN_CLD_THICKNESS;

#ifdef BLEND_CLOUD_CHANGE
   if (Last_cld_data_ok)
   {
      Check_cloud_change(target_cld_on, target_cld_base, target_cld_top,
                         &FC_cld_on, &FC_cld_base, &cld_top);

      FC_cld_thickness = cld_top - FC_cld_base;
   }
   else
#endif
   {
      FC_cld_on = target_cld_on;
      FC_cld_base = target_cld_base;
      FC_cld_thickness = target_cld_top - target_cld_base;

      Last_cld_on = FC_cld_on;
      Last_cld_base = FC_cld_base;
      Last_cld_thickness = FC_cld_thickness;
      Last_cld_data_ok = TRUE;
   }

   return;
}

/*--------------------------------------------------------------------------*/

/* This attempts to smooth out changes to clouds, mainly to smooth the
   transition from global to local cloud conditions.

   1. No cloud to cloud:
   Snap cloud on

   2. Cloud to no cloud:
   Snap cloud off

   3. Cloud to cloud:
   Smooth change to top and base

   Inputs: Last_cld_on, Last_cld_base, Last_cld_thickness
           MAX_CLOUD_DIST_CHANGE_RATE (used for base and thickness values)
           (-states.D)
*/

static void Check_cloud_change(int target_cld_on, float target_cld_base, float target_cld_top,
                               int *cld_on, float *cld_base, float *cld_top)
{
   int   apply_base_limits;
   int   apply_top_limits;
   float delta_base;
   float delta_top;
   float max_change;
   int   pos_base_change;
   int   pos_top_change;
   float last_cld_top;

   apply_base_limits = FALSE;
   apply_top_limits = FALSE;

   if (!target_cld_on && !Last_cld_on)
   {
      *cld_on = FALSE;
   }
   else if (target_cld_on && !Last_cld_on)
   {
      *cld_on = TRUE;

      /* No smoothing */
   }
   else if (!target_cld_on && Last_cld_on)
   {
      *cld_on = FALSE;

      /* No smoothing */
   }
   else
   {
      *cld_on = TRUE;
      apply_base_limits = TRUE;
      apply_top_limits = TRUE;
   }

   if (apply_base_limits)
   {
      pos_base_change = TRUE;
      delta_base = target_cld_base - Last_cld_base;
      if (delta_base < 0.0f)
      {
         delta_base = -delta_base;
         pos_base_change = FALSE;
      }

      max_change = MAX_CLOUD_DIST_CHANGE_RATE * delta_time;   //For this frame
      if (delta_base > max_change)
      {
         if (pos_base_change)
            *cld_base = Last_cld_base + max_change;
         else
            *cld_base = Last_cld_base - max_change;
      }
      else
      {
         *cld_base = target_cld_base;
      }
   }

   if (apply_top_limits)   //In this case, we recalculate cld_thickness
   {
      last_cld_top = Last_cld_base + Last_cld_thickness;
      pos_top_change = TRUE;
      delta_top = target_cld_top - last_cld_top;
      if (delta_top < 0.0f)
      {
         delta_top = -delta_top;
         pos_top_change = FALSE;
      }

      max_change = MAX_CLOUD_DIST_CHANGE_RATE * delta_time;   //For this frame
      if (delta_top > max_change)
      {
         if (pos_top_change)
            *cld_top = last_cld_top + max_change;
         else
            *cld_top = last_cld_top - max_change;
      }
      else
      {
         *cld_top = target_cld_top;
      }

      //Check that the top is above the base by MIN_CLD_THICKNESS
      if (*cld_top < (*cld_base + MIN_CLD_THICKNESS))
         *cld_top = *cld_base + MIN_CLD_THICKNESS;
   }

   Last_cld_on = *cld_on;
   Last_cld_base = *cld_base;
   Last_cld_thickness = (*cld_top - *cld_base);
}

/*--------------------------------------------------------------------------*/

/* Cannot set up the Last_cld values here because they depend on
   host position. So instead, set a flag to stop and blending
   being done. This flag gets resets when values setup. */

void FC_reset_cloud()
{
   Last_cld_data_ok = FALSE;
}

/*--------------------------------------------------------------------------*/
