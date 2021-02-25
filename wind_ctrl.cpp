/*
File  : wind_ctl.c
Prefix: WD_

Module: Seneca Dynamics
Created: 22 March 2000

Description: This file sets up the wind components
*/

/***************************/
/*   RCS Markers           */
/***************************/

/*
New wind_ctrl back in for Dyn231
$Header: C:/MASTER/seneca/RCS/wind_ctrl.cpp 1.4 2000/06/13 13:44:44 COLINJ Exp $
$Log: wind_ctrl.cpp $
Revision 1.4  2000/06/13 13:44:44  COLINJ
New function to convert wind vectors to a direction and speed.

Revision 1.3  2000/05/30 17:30:34  juliae
Changed max wind change rates from 10 to 0.5 m/s

Revision 1.2  2000/05/09 16:26:33  juliae
Correction to local wind speed value.
Correction: allow for turbulence when outside local boundary
and when there is wind shear.
Added control of the maximum rate of wind change.
New routine: WD_reset_wind.

Revision 1.1  2000/04/14 17:42:55  juliae
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
#include "fltdy\fmgdefs.h"
#include "fltdy\fmg.h"

/*----------------*/
/* Defines        */
/*----------------*/

#define FT_TO_M  (1.0 / FT_IN_METRE)


#define LOCAL_COND_HT_BDY            (3500.0 * FT_TO_M)    //Above GROUND level
#define LOCAL_COND_DIST_BDY          (10.0 * METRES_IN_NMILE)

/////////////Potter mod//////////////////////////////////
/* 
#define M_WIND_LEVEL ( 5000.0 * FT_TO_M) 
#define H_WIND_LEVEL (10000.0 * FT_TO_M) 
/*
#define SWindSpeed (10.0)
#define SWindDirection (0.0)
#define MWindSpeed (50.0)
#define MWindDirection (90.0)
#define HWindSpeed (400.0)
#define HWindDirection (180.0)
*/
/////////////Potter mod//////////////////////////////////

//#define MAX_TURB 8.0f
#define MAX_TURB 100.0f
//#define MAX_TURB 200.0f

#define TURB_RATE 0.85f   //Note sure this is an accurate description (was F1)
//#define TURB_RATE 1.7f   //Note sure this is an accurate description (was F1)
//#define TURB_INTERVAL 0.5f
float TURB_INTERVAL = 0.1f;
#define MAX_N_WIND_CHANGE_RATE  20.0f//0.5f //10.0f    //m/s
#define MAX_E_WIND_CHANGE_RATE  20.0f//0.5f //10.0f    //m/s //Woody Mod from .5 to 20
#define MAX_D_WIND_CHANGE_RATE  20.0f//0.5f //10.0f    //m/s


//When using an array for wind components
#define N_IND  0
#define E_IND  1
#define D_IND  2

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

/*
extern float SWindSpeed;
extern float SWindDirection;
extern float MWindSpeed;
extern float MWindDirection;
extern float HWindSpeed;
extern float HWindDirection;
*/

/*----------------*/
/* Globals        */
/*----------------*/

/*-----------------*/
/* Local Variables */    
/*-----------------*/

static float Last_wind[3] = {0.0f, 0.0f, 0.0f};

static float Turb_dtime = 0.0f;
/*static*/ float Turb_vN = 0.0f, Turb_vE = 0.0f, Turb_vD = 0.0f;

static float turb_freq[5]    = {0.1f,  0.09f, 0.07f, 0.05f,  0.03f};
static float percent[5] = {1.0f, 25.0f ,50.0f ,75.0f ,100.0f};

static float pitch_scale[5] = {0.0f,25.0f, 50.0f, 75.0f,100.0f};
static float roll_scale[5]  = {0.0f,25.0f,100.0f,150.0f,200.0f};
static float s_input_per[5] = {0.0f,25.0f, 50.0f, 75.0f,100.0f};
//3 components representing N, E and D (Vertical)
float Turb_1[3], Turb_2[3], Turb_3[3], Turb_4[3];

/*--------------------*/
/* Forward References */
/*--------------------*/

static void Check_max_wind_change(float target_wind[], float wind[]);
static void Add_turbulence(float wind[]);
//static float lint(float xl, float yl, float xh, float yh, float x);

/*--------------------------------------------------------------------------*/

/* Wind inputs from IOS may need to be modified depending on whether
   we are inside or outside an airfield local conditions boundary.
   (Unlike fog, we fade from outer boundary to origin)
   The boundary is a cylinder.

   1. If outside this boundary at all, wind is global wind.
   2. If inside the boundary, the wind is a blend of global and local (atis).
   3. If wind shear is on, then above wind shear height wind is global wind 
      and below it, it has the blended value it would have had if there
      was no wind shear existed.

   Inputs: offsets from ORIGIN of local region: horizontal distance,
           ht = ht above origin ht
   Outputs: wind[0] = east wind component
            wind[1] = north wind component
            wind[2] = up wind component
            magnitude of wind = wind speed   (m/s)
*/

/* Potter version 2004/02/24
   
   Uses 3 level model with surface, medium and high level winds.
   Velocities are inerpolated between these linearly, except when
   the shear feature is invoked. In this case, the transition from 
   the interpolated value is made abrupt.

*/

void WD_wind_ctrl (int valid_bdy, float hdist, float delta_ht, float wind[3])
{
    //Potter mod
   float fraction1;
   float fraction2;
   float fraction;
   float gwind_dir;
   float gwind_speed;   //m/s
   float lwind_dir;
   float lwind_speed;   //m/s
   float gwind[3];
   float lwind[3];
   
  // float swind[3], mwind[3], hwind[3];
   float shear_ht;      //metres
    int   i;
   float target_wind[3];

 ///////////////////Potter Mod////////////////////////
 /*  
   gwind_dir = (float) (DY_atmos.SWindDirection * DEG_TO_RAD);
   gwind_speed = (float) (DY_atmos.SWindSpeed * KTS_TO_MS);
*/
 ///////////////////Potter Mod////////////////////////

    gwind_dir = (float) (DY_atmos.WindDirection * DEG_TO_RAD);
    gwind_speed = (float) (DY_atmos.WindSpeed * KTS_TO_MS);
   if (!valid_bdy)
   {
      target_wind[0] = (float) (gwind_speed * sin(gwind_dir));
      target_wind[1] = (float) (gwind_speed * cos(gwind_dir));
      target_wind[2] = (float) (0.0);
      Check_max_wind_change(target_wind, wind);
      Add_turbulence(wind);
      return;
   }
   
////////////////////////Potter Mod////////////////////
/*
   swind[0] = (float) (SWindSpeed * KTS_TO_MS * sin(SWindDirection * DEG_TO_RAD));
   swind[1] = (float) (SWindSpeed * KTS_TO_MS * cos(SWindDirection * DEG_TO_RAD));
   swind[2] = 0.0f;

   mwind[0] = (float) (MWindSpeed * KTS_TO_MS * sin(MWindDirection * DEG_TO_RAD));
   mwind[1] = (float) (MWindSpeed * KTS_TO_MS * cos(MWindDirection * DEG_TO_RAD));
   mwind[2] = 0.0f;

   hwind[0] = (float) (HWindSpeed * KTS_TO_MS * sin(HWindDirection * DEG_TO_RAD));
   hwind[1] = (float) (HWindSpeed * KTS_TO_MS * cos(HWindDirection * DEG_TO_RAD));
   hwind[2] = 0.0f;



   if (delta_ht > H_WIND_LEVEL) // we are above the top level
   {   // use high level wind
	   target_wind[0] = hwind[0];
	   target_wind[1] = hwind[1];
	   target_wind[2] = hwind[2];
       Check_max_wind_change(target_wind, wind);
       Add_turbulence(wind);
       return;
   }

   if (delta_ht > M_WIND_LEVEL) // we are above the medium level and below the top
   {   // interpolate between medium and high level wind
	   target_wind[0] = lint((float)M_WIND_LEVEL, mwind[0], (float)H_WIND_LEVEL, hwind[0], delta_ht);
	   target_wind[1] = lint((float)M_WIND_LEVEL, mwind[1], (float)H_WIND_LEVEL, hwind[1], delta_ht);
	   target_wind[2] = lint((float)M_WIND_LEVEL, mwind[2], (float)H_WIND_LEVEL, hwind[2], delta_ht);
       Check_max_wind_change(target_wind, wind);
       Add_turbulence(wind);
       return;
   }

   if (IOS_atis.WindShear) // we are below the medium level and there is wind shear selected
   {   
	   shear_ht = (float) IOS_atis.ShearHeight; //Metres
	   if (delta_ht > shear_ht) // we are in the shear zone
	   {   // so use mid level winds
		   target_wind[0] = mwind[0];
		   target_wind[1] = mwind[1];
		   target_wind[2] = mwind[2];
		   Check_max_wind_change(target_wind, wind);
		   Add_turbulence(wind);
           return;
	   }
   }
       // if we get here there is no wind shear and we are in the low region
       // interpolate between surface and medium level wind
	   target_wind[0] = lint(0.0, swind[0], (float)M_WIND_LEVEL, mwind[0], delta_ht);
	   target_wind[1] = lint(0.0, swind[1], (float)M_WIND_LEVEL, mwind[1], delta_ht);
	   target_wind[2] = lint(0.0, swind[2], (float)M_WIND_LEVEL, mwind[2], delta_ht);
       Check_max_wind_change(target_wind, wind);
       Add_turbulence(wind);
       return;
   

   */
//////////////////Potter mod/////////////////////////////////////////////////

   if ((delta_ht > LOCAL_COND_HT_BDY) ||
       (hdist > LOCAL_COND_DIST_BDY))
   {
      target_wind[0] = (float) (gwind_speed * sin(gwind_dir));
      target_wind[1] = (float) (gwind_speed * cos(gwind_dir));
      target_wind[2] = (float) (0.0);
      Check_max_wind_change(target_wind, wind);
      Add_turbulence(wind);
      return;
   }
   
   //We are within local conditions region. Need to blend winds
   //unless we are above an active wind shear height.

   if (IOS_atis.WindShear)
   {
      shear_ht = (float) IOS_atis.ShearHeight;   //Metres
      if (delta_ht > shear_ht)
      {
         target_wind[0] = (float) (gwind_speed * sin(gwind_dir));
         target_wind[1] = (float) (gwind_speed * cos(gwind_dir));
         target_wind[2] = (float) (0.0);
         Check_max_wind_change(target_wind, wind);
         Add_turbulence(wind);
         return;
      }
   }

   lwind_dir = (float) (IOS_atis.WindDir * DEG_TO_RAD);
   lwind_speed = (float) (IOS_atis.WindSpeed * KTS_TO_MS);

   gwind[0] = (float) (gwind_speed * sin(gwind_dir));
   gwind[1] = (float) (gwind_speed * cos(gwind_dir));
   gwind[2] = (float) (0.0);

   lwind[0] = (float) (lwind_speed * sin(lwind_dir));
   lwind[1] = (float) (lwind_speed * cos(lwind_dir));
   lwind[2] = (float) (0.0);

 /*   If you find the fraction you need due to height, f1 and
      then the fraction you need due to horiz dist, f2, the
      combined fraction required is
      fraction = f1 + f2 - f1*f2
      
      (Alternatively, do the first blend to give an answer
      and use that answer as input to the second blend) 
*/
   fraction1 = (float) (delta_ht / LOCAL_COND_HT_BDY);
   fraction2 = (float) (hdist / LOCAL_COND_DIST_BDY);

   fraction = fraction1 + fraction2 - (fraction1 * fraction2);
   if (fraction > 1.0)
      fraction = 1.0;
   else if (fraction < 0.0)
      fraction = 0.0;

   //Blend the wind vectors by blending each component
   //separately (for now anyway)
   
   for (i = 0; i < 3; i++)
   {
      target_wind[i] = lwind[i] + fraction * (gwind[i] - lwind[i]);
   }

   Check_max_wind_change(target_wind, wind);
   Add_turbulence(wind);
   
   
}
///////////////////////////Potter Mod///////////////////

/* linear interpolation routine 

static float lint(float xl, float yl, float xh, float yh, float x)
{
	return (yl + (x - xl) / (xh -xl) * (yh - yl));
}
*/
///////////////////////////Potter Mod///////////////////

/*--------------------------------------------------------------------------*/

/* Inputs: Last_wind[]
           MAX_N/E/D_WIND_CHANGE_RATE defines
*/

static void Check_max_wind_change(float target_wind[], float wind[])
{
   float delta_wind;
   float max_change;
   int   pos_wind_change;

   /* Allow each wind component to have a different max change rate.
      Cannot use a loop since defines involved */
   /* NORTH */
   pos_wind_change = TRUE;
   delta_wind = target_wind[N_IND] - Last_wind[N_IND];
   if (delta_wind < 0.0f)
   {
      delta_wind = -delta_wind;
      pos_wind_change = FALSE;
   }

   max_change = MAX_N_WIND_CHANGE_RATE * delta_time;   //For this frame
   if (delta_wind > max_change)
   {
      if (pos_wind_change)
         wind[N_IND] = Last_wind[N_IND] + max_change;
      else
         wind[N_IND] = Last_wind[N_IND] - max_change;
   }
   else
   {
      wind[N_IND] = target_wind[N_IND];
   }

   /* EAST */
   pos_wind_change = TRUE;
   delta_wind = target_wind[E_IND] - Last_wind[E_IND];
   if (delta_wind < 0.0f)
   {
      delta_wind = -delta_wind;
      pos_wind_change = FALSE;
   }

   max_change = MAX_E_WIND_CHANGE_RATE * delta_time;   //For this frame
   if (delta_wind > max_change)
   {
      if (pos_wind_change)
         wind[E_IND] = Last_wind[E_IND] + max_change;
      else
         wind[E_IND] = Last_wind[E_IND] - max_change;
   }
   else
   {
      wind[E_IND] = target_wind[E_IND];
   }

   /* DOWN */
   pos_wind_change = TRUE;
   delta_wind = target_wind[D_IND] - Last_wind[D_IND];
   if (delta_wind < 0.0f)
   {
      delta_wind = -delta_wind;
      pos_wind_change = FALSE;
   }

   max_change = MAX_D_WIND_CHANGE_RATE * delta_time;   //For this frame
   if (delta_wind > max_change)
   {
      if (pos_wind_change)
         wind[D_IND] = Last_wind[D_IND] + max_change;
      else
         wind[D_IND] = Last_wind[D_IND] - max_change;
   }
   else
   {
      wind[D_IND] = target_wind[D_IND];
   }

   Last_wind[N_IND] = wind[N_IND];
   Last_wind[E_IND] = wind[E_IND];
   Last_wind[D_IND] = wind[D_IND];
}

/*--------------------------------------------------------------------------*/

static void Add_turbulence(float wind[])
{
   //Add turbulence values
   /*wind[0] += Turb_vN;
   wind[1] += Turb_vE;
   wind[2] -= Turb_vD;*/
}

/*--------------------------------------------------------------------------*/

void WD_reset_wind()
{
   Last_wind[N_IND] = 0.0f;
   Last_wind[E_IND] = 0.0f;
   Last_wind[D_IND] = 0.0f;
}

/*--------------------------------------------------------------------------*/

/* (Martin Kellet)
   This updates local turbulence values which are added to the wind
   components, when wind is updated.
   NB. This routine does not have to be called every time
   the wind is updated. The rate at which this routine is called
   affects the turbulence calculations, so this routine should be
   called at a fixed rate

   Inputs: Turb_1[*], Turb_2[*], Turb_3[*], Turb_4[*]
   Outputs: Turb_vN, Turb_vE, Turb_vD
*/

void WD_update_turb(int turb_percent)
{
   float u[3];   //For components 0 = N, 1 = E and 2 = Vertical
   int   i;
   float strength;

   Turb_dtime += delta_time;
   TURB_INTERVAL = table1D(percent,turb_freq,5,(float)turb_percent);

   if (Turb_dtime >= TURB_INTERVAL)
   {
      Turb_dtime = 0.0f;

      Turb_vN = 0.0; Turb_vE = 0.0; Turb_vD = 0.0;

      for (i = 0; i < 3; i++)
      {
         u[i]  = (float)(rand()-16384) / 16384.0f;
         Turb_1[i] = (TURB_RATE * Turb_1[i] + (1.0f - TURB_RATE) * u[i] );
         Turb_2[i] = (TURB_RATE * Turb_2[i] + (1.0f - TURB_RATE) * Turb_1[i]);
         Turb_3[i] = (TURB_RATE * Turb_3[i] + (1.0f - TURB_RATE) * Turb_2[i]);
         Turb_4[i] = (TURB_RATE * Turb_4[i] + (1.0f - TURB_RATE) * Turb_3[i]);
      }

      //Now handle turbulence strength

      //strength = ((float) turb_percent / 100.0f) * MAX_TURB;
      strength = table1D(s_input_per,pitch_scale,5,(float)turb_percent);

      Turb_vE = Turb_4[1] * strength; //pitch

      Turb_vN = Turb_4[0] * strength; 

      strength = table1D(s_input_per,roll_scale,5,(float)turb_percent);
      
      Turb_vD = Turb_4[2] * strength;//roll
   }
}

/*--------------------------------------------------------------------------*/

void WD_init_turb(void)
{
   int i;

   for (i = 0; i < 3; i++)   //For components N, E and Vertical
   {
      Turb_1[i] = 0.0;
      Turb_2[i] = 0.0;
      Turb_3[i] = 0.0;
      Turb_4[i] = 0.0;
   }

   Turb_vN = 0.0f;
   Turb_vE = 0.0f;
   Turb_vD = 0.0f;

   Turb_dtime = 0.0f;
}

/*--------------------------------------------------------------------------*/
//convert wind vectors to bearing and speed

void WD_wind_vec_to_speed_dir(float e,float n,float d,float *speed,float *dir)
{
	float wspeed_sq;
	float wspeed_ms;
	float wdir;

	wspeed_sq = (e * e) + (n * n) + (d * d);
	wspeed_ms = (float)sqrt(wspeed_sq);

	*speed = (float)(wspeed_ms / KTS_TO_MS);

	/* East component prop to sin(dir)
	   North                  cos(dir)
		so tan_dir = e / n  */

   wdir = (float)(atan2(e,n) * RAD_TO_DEG);

	*dir = wdir;
}
/*--------------------------------------------------------------------------*/
