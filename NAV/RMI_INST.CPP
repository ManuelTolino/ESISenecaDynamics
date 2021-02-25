/*
File: rmi_inst.c
Prefix: RMI_

Module: Navigation (Dynamics) for Seneca

Description: This handles update of RMI dial depending on selected
non-directional beacon.
*/

/***************************/
/*   RCS Markers           */
/***************************/

/*
$Header: C:/Master/seneca/NAV/RCS/rmi_inst.cpp 1.10 2000/06/16 11:14:24 ColinJ Exp $
$Log: rmi_inst.cpp $
Revision 1.10  2000/06/16 11:14:24  ColinJ
If ils freq is selected vor needle parks.

Revision 1.9  2000/06/01 10:36:03  juliae
Change to handling of ADF DIP error.

Revision 1.8  2000/05/25 15:55:42  juliae
Slow down speed of ADF needle with range.

Revision 1.7  2000/05/18 09:50:43  juliae
RMI VOR needle now not affected by ILS signals.

Revision 1.6  2000/04/25 15:49:49  juliae
Allow for adf dip error.

Revision 1.5  2000/03/27 09:36:01  juliae
Added range checks based on NDB power and calculate
a measure of signal strength. New routine RMI_init must
be called.

 * Revision 1.4  2000/03/17  16:51:00  juliae
 * Correction: ADF, not VOR needle is in 90 position in KR87_ANT mode.
 * VOR needle can still work if KR87 box is off.
 * Allow for adf avionics fault.
 *
 * Revision 1.3  2000/03/02  10:24:24  juliae
 * Renamed host_pos->alt to ht.
 *
 * Revision 1.2  2000/03/01  15:19:23  juliae
 * Correction to setting VOR needle to 90 in ANT mode.
 *
 * Revision 1.1  2000/02/21  20:22:55  juliae
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
#include "c:\bae-sims\seneca\nav/beacon.h"
#include "c:\bae-sims\seneca\nav/b_dials.h"
#include "c:\bae-sims\seneca\dat_stru/in_head.h"
#include "c:\bae-sims\seneca\dat_stru/out_head.h"
#include "c:\bae-sims\seneca\nav/cpit_def.h"
#include "c:\bae-sims\seneca\faults.h"
#include "c:\bae-sims\seneca\nav/bc_range.h"
#include "c:\bae-sims\seneca\fltdy\fms_defs.h"
#include "c:\bae-sims\seneca\fltdy\fmgdefs.h"
#include "c:\bae-sims\seneca\fltdy\fmg.h"


/*-------------------------*/
/* Defines                 */
/*-------------------------*/

#define ILS_PARKS_VOR_NEEDLE 1
#define MAX_ADF_DIP_ERROR 10.0f
#define MAX_ROLL           50.0f   /* Roll at which you get max error */
#define NUM_NDB_POWER_BANDS 4

#define MAX_SIGNAL 100
#define MIN_SIGNAL 1

#define ADF_DIP_ERROR 1
#define SLOW_NEEDLE_WITH_RANGE 1

//Minmum rate is actually 0.99*min + 0.01*max
#define MIN_ADF_TURN_RATE   (180.0f/12.0f)    //180 deg in 12 sec
#define MAX_ADF_TURN_RATE   (180.0f/0.5f)    //180 deg per half sec

/*-------------------------*/
/* Typedefs                */
/*-------------------------*/

/*----------------------------*/
/* External Routines          */
/*----------------------------*/

extern float SPH_get_sph_heading (double lat1, double lon1, double lat2, double lon2);
extern double SPH_get_sph_flat_dist_sq (double lat1, double lon1, float ht1,
                                        double lat2, double lon2, float ht2);
extern float BC_make_magnetic(float true_head, float magnetic_var);
extern long N2A_get_active_n2_freq();
extern int N1A_ils_freq_num(long freq);

/*----------------------------*/
/* Externals                  */
/*----------------------------*/

extern sen_out_struct *IO_Sen_out;
extern sen_in_struct  *IO_Sen_in;
extern OUTPUTin			*OUTPUTinbox;//simulinkudp
extern sen_fault_struct IO_Sen_faults;
extern OUTPUTS        outputs;
extern float          CS_magnetic_var;
extern float          CS_real_host_mhead;
extern int            BC_valid_n1_b_signal;
extern int            BC_valid_n2_b_signal;
extern float          BC_n1_recip_radial;
extern float          BC_n2_recip_radial;
extern vhf_beac_attr  IO_nav2_battr;
extern vhf_beac_attr  IO_nav1_battr;
extern float          delta_time;

/*----------------------------*/
/* Static Variables           */
/*----------------------------*/

float Ndb_range[NUM_NDB_POWER_BANDS];

static float roll_angle[8] = { 0.0, 5.0, 10.0,15.0 , 20.0,30.0,40.0,50.0 };//roll angle of aircraft
static float adf_dip[8]    = { 0.0, 5.0,  8.0,10.0,  11.0,12.0,13.0,13.0 }; // ammount of deflection on adf neddle

#ifdef ILS_PARKS_VOR_NEEDLE
int  park_vor_needle = FALSE;
long n2_freq = 0;
#endif

/*----------------------------------------------------------------*/

void RMI_init()
{
   /* Array to cross reference power to range */
   Ndb_range[0] = (float) (NDB_RANGE_0_NM * METRES_IN_NMILE);
   Ndb_range[1] = (float) (NDB_RANGE_1_NM * METRES_IN_NMILE);
   Ndb_range[2] = (float) (NDB_RANGE_2_NM * METRES_IN_NMILE);
   Ndb_range[3] = (float) (NDB_RANGE_3_NM * METRES_IN_NMILE);
}

/*----------------------------------------------------------------*/

/* Returns TRUE if host is within valid region (DOC) for vor.
   This checks for range and "cone of confusion" */

int Valid_ndb_region(ndb_beac_attr *ndb,
                     double host_lat, double host_lon, float host_ht)
{
   double dist_sq;          //Metres sq
   float  sdist_host_ndb;   //Metres
   int    power;
   float  min;
   float  max;
   float  fraction;
   int    strength;

   if (!ndb->valid_flag)
      return FALSE;

   /* Get distance between host and localizer, to see if with range.
      This gives slant distance, since we pass correct heights */

   dist_sq = SPH_get_sph_flat_dist_sq (host_lat, host_lon, host_ht,
                                       ndb->beac_lat,
                                       ndb->beac_lon,
                                       ndb->beac_elev);

   sdist_host_ndb = (float) sqrt(dist_sq);


   /* Range tests */
   power = ndb->power;
   if (power >= NUM_NDB_POWER_BANDS)
      power = NUM_NDB_POWER_BANDS - 1;

   if (sdist_host_ndb > Ndb_range[power])
      return FALSE;

   /* Other tests? */

   /* Return an indication of signal strength */
   max = (float) MAX_SIGNAL;
   min = (float) MIN_SIGNAL;
   fraction = sdist_host_ndb / Ndb_range[power];
   strength = (int) (max - fraction * (max - min));
   if (strength < MIN_SIGNAL)
      strength = MIN_SIGNAL;

   return strength;
}

/*----------------------------------------------------------------*/

/* Works out the required needle positions of the RMI AND says
   whether signal is valid.
   The RMI on one side of cockpit uses magnetic heading from other
   side.
   MAGNETIC heading always at top of dial.
   NB. Unlike HSI, the 2 needles on this dial are specified with
   respect to the value at the top of the dial.
   
   Inputs:  IO_Sen_in: KR87_off_on_switch
                       KR87_ADF_ANT_switch
                       panel_RMI_nav1_nav2_switch
            IO_Sen_faults.AVF.adf
            BC_valid_n1_b_signal
            BC_valid_n2_b_signal
            BC_n1_recip_radial
            BC_n2_recip_radial

   Outputs:
*/

void RMI_set_rmi (ndb_beac_attr *ndb_attr, ll_pos *host_pos, float gyro_mhead,
                  int *valid_ndb_signal, rmi_out_data *rmi)
{
   float         bear_host_ndb;
   float         mbear_host_ndb;
   float         recip_radial;
   float         qdm;    /* Radial to go TO beacon */
   int           valid_flag;
   int           done_adf;
#ifdef ADF_DIP_ERROR
   float         true_head;
   float         delta;
   float         constant;
   float         roll;
   float         error;
#endif
#ifdef SLOW_NEEDLE_WITH_RANGE
   float         strength;
   float         fraction;
   float         move_rate;
   float         max_change;
   float         delta_ang;
   float         target_ADF_needle;
#endif

   *valid_ndb_signal = FALSE;
   done_adf = FALSE;

   /* Compass card rotates so that aircraft heading is at top */
   rmi->demand_head = gyro_mhead;   /* In degrees */

   if ((IO_Sen_in->KR87_off_on_switch == SW_OFF) || IO_Sen_faults.AVF.adf)
   {
      /* The VOR needle still works */
      *valid_ndb_signal = FALSE;
   }
   else
   {
      *valid_ndb_signal = Valid_ndb_region(ndb_attr, host_pos->lat,
                                           host_pos->lon, host_pos->ht);

      if (IO_Sen_in->KR87_ADF_ANT_switch == KR87_ANT)
      {
         /* Bearing pointer is in 90 deg relative position during this mode */
         rmi->ADF_needle = 90.0;
         done_adf = TRUE;
      }
   }

   /* Bearing pointer points in the direction of the station
      relative to the aircraft heading */

   if (*valid_ndb_signal && !done_adf)
   {
      /* Find bearing from host to ndb. This routine gives true bearing. */
      bear_host_ndb = SPH_get_sph_heading (host_pos->lat, host_pos->lon,
                                           ndb_attr->beac_lat, ndb_attr->beac_lon);

      /* NB. For this type of beacon, I should be using the magnetic variation
         at the aircraft. NB. This is NOT the same as a radial from a VOR
         at the same position as the NDB unless the magnetic variations
         at the aircraft position and VOR are the same */
      mbear_host_ndb = BC_make_magnetic(bear_host_ndb, CS_magnetic_var);

      /* The relative bearing is
         rel_head = (mbear_host_ndb - gyro_mhead).
         We must pass the relative value to the needle. Since the
         entire dial is rotated so heading is at the top, this results
         in the needle actually pointing to the actual bearing. */

      target_ADF_needle = mbear_host_ndb - CS_real_host_mhead;//gyro_mhead;//CJ WAS HERE


#ifdef ADF_DIP_ERROR
      /* Introduce errors for bank. Unlike VOR signals, which have
         a direction encoded as part of the signal, the ADF uses
         the direction in which it detects the NDB signals coming from.
         So you get errors with bank when heading is across direct
         path from NDB to host */
      true_head = (float) (outputs.psi * RAD_TO_DEG);
      delta = bear_host_ndb - true_head;
      if (delta > 180.0)
         delta -= 360.0;
      if (delta <= -180.0)
         delta += 360.0;

      /* This delta is just used to scale the error and
         does not affect the sign. */
      if (delta < 0.0)
         delta = -delta;

      /* Max error is 10 deg at roll of 5 and over */

      roll = (float) (outputs.phi * RAD_TO_DEG);
      if (roll > MAX_ROLL)
         roll = MAX_ROLL;
      else if (roll < -MAX_ROLL)
         roll = -MAX_ROLL;

      constant = table1D(roll_angle,adf_dip,8,(float)fabs(roll));
      error = constant * (float) cos(delta * DEG_TO_RAD);

/*
      // The needle LAGS by this error, ie. subtract.

      target_ADF_needle -= error;
*/

      if (error < 0.0)
         error = -error;
      
      /* If bank left, take error,
         if bank right, add error */

      if (roll > 0.0)    //Right
         target_ADF_needle += error;
      else               //Left
         target_ADF_needle -= error;
#endif

#ifdef SLOW_NEEDLE_WITH_RANGE
      /* The speed at which the ADF needle can change from its test
         position of 90 to correct position decreases with range. So
         impose a general rate of change of movement from current
         to target value, which decreases with range */

      strength = *valid_ndb_signal / (float) MAX_SIGNAL;    //Range [0.01, 1]
      if (strength > 1.0f)
         strength = 1.0f;

      /* Do not use a linear decrease. Instead try a sin function, eg.
         at 1/2 strength, move rate is sin(0.5 * 90) (=0.7) of full rate */

      fraction = (float) sin(strength * 90.0 * DEG_TO_RAD);

      move_rate = MIN_ADF_TURN_RATE + fraction * (MAX_ADF_TURN_RATE - MIN_ADF_TURN_RATE);

      max_change = move_rate * delta_time;   //This frame

      delta_ang = target_ADF_needle - rmi->ADF_needle;

      /* Allow for wrap, eg. 179 to -179 is a delta of 2,
         359 to 2 is a delta of 3 */
      if (delta_ang > 180.0)
         delta_ang -= 360.0;
      else if (delta_ang < -180.0)
         delta_ang += 360.0;

      if (delta_ang >= 0.0)
      {
         if (delta_ang <= max_change)
            rmi->ADF_needle = target_ADF_needle;
         else
            rmi->ADF_needle = rmi->ADF_needle + max_change;
      }
      else
      {
         delta_ang = -delta_ang;
         if (delta_ang <= max_change)
            rmi->ADF_needle = target_ADF_needle;
         else
            rmi->ADF_needle = rmi->ADF_needle - max_change;
      }
      
#else
      rmi->ADF_needle = target_ADF_needle;
#endif

      /* Keep in range 0 to 360 */
      if (rmi->ADF_needle >= 360.0)
         rmi->ADF_needle -= 360.0;
      if (rmi->ADF_needle < 0.0)
         rmi->ADF_needle += 360.0;
   }

   /* Calculation of QDM needle. This relates to either NAV1 or NAV2 beacon.
      It uses the RADIAL value that the BEACON says the aircraft is on
      (so magnetic variation used is that at beacon).
      NOTE: Only for VOR types, not ILS */
   valid_flag = FALSE;
#ifdef ILS_PARKS_VOR_NEEDLE
   park_vor_needle = FALSE;
#endif
   if (IO_Sen_in->panel_RMI_nav1_nav2_switch == RMI_NAV1)
   {
      if (BC_valid_n1_b_signal)  //VOR or ILS type
      {
         if (IO_nav1_battr.type == VOR_TYPE)
         {
            valid_flag = TRUE;
            recip_radial = BC_n1_recip_radial;
         }
      }
#ifdef ILS_PARKS_VOR_NEEDLE
      if(!valid_flag)
      {
         /* If frequency entered is an ILS frequency, the VOR needle goes to 90 degrees
         regardles of whether an ILS signal is being received. Using theILS token takes
         care of when the NAV1 box is off */

         if(IO_Sen_out->KNS80_ILS_token == LIT)
            park_vor_needle = TRUE;
      }
#endif
   }
   else
   {
      if (BC_valid_n2_b_signal)  //VOR or ILS type
      {
         if (IO_nav2_battr.type == VOR_TYPE)
         {
            valid_flag = TRUE;
            recip_radial = BC_n2_recip_radial;
         }
      }
#ifdef ILS_PARKS_VOR_NEEDLE
      if(!valid_flag)
      {
         /* If frequency entered is an ILS frequency, the VOR needle goes to 90 degrees
         regardles of whether an ILS signal is being received. Using theILS token takes
         care of when the NAV1 box is off */

         n2_freq = N2A_get_active_n2_freq();
         if(N1A_ils_freq_num(n2_freq))
            park_vor_needle = TRUE;
      }
#endif

   }
   if (valid_flag)
   {
      /* This is bearing from host to beacon, using beacons magnetic var */
      qdm = recip_radial;

      /* We have to pass value relative to aircraft heading, since whole
         dial rotates */
      rmi->VOR_needle = qdm - CS_real_host_mhead;//gyro_mhead;//CJ_WAS HERE

      /* Keep in range 0 to 360 */
      if (rmi->VOR_needle >= 360.0)
         rmi->VOR_needle -= 360.0;
      if (rmi->VOR_needle < 0.0)
         rmi->VOR_needle += 360.0;
   }
#ifdef ILS_PARKS_VOR_NEEDLE
   else if (park_vor_needle)
   {
      rmi->VOR_needle = 90.0;
   }
#endif

}

/*----------------------------------------------------------------*/
