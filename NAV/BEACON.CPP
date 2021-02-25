/*
File: beacon.c
Prefix: BC_

Module: Navigation (Dynamics) for Seneca

Description: This file handles the update of dials and displays connected
             with beacon signals.
---------------------*/

/***************************/
/*   RCS Markers           */
/***************************/

/*
$Header: C:/Master/seneca/NAV/RCS/beacon.cpp 1.9 2000/05/25 15:49:18 juliae Exp $
$Log: beacon.cpp $
Revision 1.9  2000/05/25 15:49:18  juliae
Allow for copilot HSI being controlled by NAV2.
Separate out HSI heading data from navigation data.
Set new sound type for adf morse when BFO is on.

Revision 1.8  2000/05/18 10:18:49  juliae
Correction for MIN display flickering between dash and number.

Revision 1.7  2000/05/16 17:14:28  colinj
Now pass a parameter to HSI_set_hsi_head to say if gyro is ok.

Revision 1.6  2000/05/09 08:25:16  juliae
Replaced DME_get_srange_sq_nm with DME_get_srange_nm

Revision 1.5  2000/04/25 17:38:14  juliae
Allow for a valid glide slope signal when the ils is invalid.

Revision 1.4  2000/03/27 10:04:32  juliae
New BC_valid_dme_signal and BC_valid_ndb_signal. BC_valid_XXX_signal
now give a measure of signal strength. Correction for BFO switch.

 * Revision 1.3  2000/03/17  16:40:43  juliae
 * Allow for no IO_avionic_pwr. Allow for avionic faults.
 * Dash MIN value on KNS80 if time > 99
 * Correction to dme_hold - only relevant if holding to a
 * different frequency.
 * Pass valid_dme flag to HSI routine.
 *
 * Revision 1.2  2000/03/02  10:25:01  juliae
 * Renamed host_pos->alt to ht. Correction to sign of this.
 *
 * Revision 1.1  2000/02/22  02:37:13  juliae
 * Initial revision
 *
*/

/*----------------------------------------------------------------------*/

/*----------------*/
/* Include Files  */
/*----------------*/

#include <stdio.h>
#include <math.h>
#include "c:\bae-sims\seneca\dat_stru/in_head.h"
#include "c:\bae-sims\seneca\dat_stru/out_head.h"
#include "cpit_def.h"
#include "b_dials.h"
#include "beacon.h"
#include "nav1.h"
#include "c:\bae-sims\seneca\fltdy/fmgdefs.h"
#include "c:\bae-sims\seneca\define.h"
#include "c:\bae-sims\seneca\faults.h"
#include "c:\bae-sims\seneca\dyn_snd.h"

/*----------------*/
/* Defines        */
/*----------------*/

#define DME_DTIME_CALC 1.0

#define TOKEN_ON   LIT
#define TOKEN_OFF  UNLIT

#define NAV2_COPILOT_HSI 1

/*----------------*/
/* Typedefs       */
/*----------------*/

/*--------------------*/
/* External Routines  */
/*--------------------*/

extern float SPH_get_sph_heading (double lat1, double lon1, double lat2, double lon2);
extern void  RMI_set_rmi (ndb_beac_attr *ndb_attr, ll_pos *host_pos, float gyro_mhead,
                          int *valid_signal, rmi_out_data *rmi);
extern int   ILS_valid_ils_signal(vhf_beac_attr *ils_in,
                                  ll_pos *host_pos, float *recip_radial);
extern void  ILS_calc_ils_data (vhf_beac_attr *ils_in, ll_pos *host_pos, int valid_signal,
                                float recip_radial, ils_inst_out_data *ils_out);
extern int   VOR_valid_vor_signal(vhf_beac_attr *vor_in, ll_pos *host_pos);
extern void  VOR_calc_vor_data (int valid_signal, float recip_radial,
                                float obs, vor_inst_out_data *vor_out);
extern void  HSI_set_hsi_nav (vhf_beac_attr *battr, ll_pos *host_pos, int valid_signal,
                              int valid_dme, float recip_radial, float hsi_obs,
                              float cross_dev, hsi_out_data *hsi);
extern float RN_get_vor_par_cross_dev(float host_recip_radial, float obs, float srange_nm);
extern void  RN_nav1_wpoint_dist(float host_radial, float wp_dist, float wp_dist_sq,
                                 float wp_radial, float srange_nm, float srange_sq_nm,
                                 float *wp_host_dist_nm);
extern int   RN_nav1_wpoint_dev(float host_radial, float wp_dist_sq, float wp_radial,
                                float srange_nm, float srange_sq_nm, float wp_host_dist_nm, 
                                float obs, float *cross_dev_nm, char *to_flag);
extern void  DME_get_srange_nm (vhf_beac_attr *battr, ll_pos *host_pos, float *srange_nm,
                                int *valid_signal);
extern float N1A_nav1_get_active_wp_dist();
extern float N1A_nav1_get_active_wp_dist_sq();
extern float N1A_nav1_get_active_wp_radial();

/*--------------------*/
/* External Variables */
/*--------------------*/

extern float  CS_gyro_pilot_mhead;
extern float  CS_gyro_copilot_mhead;
extern int    CS_gyro_ok_speed;   //We use a single flag for both gyros
extern int    CS_pilot_hdg_flag;
extern int    CS_copilot_hdg_flag;

extern vhf_beac_attr IO_nav2_battr;
extern vhf_beac_attr IO_nav1_battr;
extern vhf_beac_attr IO_dme_battr;
extern ndb_beac_attr IO_ndb_battr;

extern sen_in_struct  *IO_Sen_in;
extern OUTPUTin		  *OUTPUTinbox;//simulinkudp
extern sen_out_struct *IO_Sen_out;
extern sen_fault_struct IO_Sen_faults;
extern int            IO_elec_pwr;
extern int            IO_avionic_pwr;
extern float          delta_time;
extern STATES         states;

extern int N1A_nav1_mode;
extern int N1A_dme_hold;

extern int SC_nav2_audio;
extern int SC_nav1_audio;
extern int SC_dme_audio;
extern int SC_adf_audio;
extern int SC_adf_snd_type;

/*----------------*/
/* Globals        */
/*----------------*/

ll_pos BC_host_pos;
/* If non-zero, these give a measure of signal strength */
int    BC_valid_n1_b_signal = FALSE;
int    BC_valid_n2_b_signal = FALSE;
int    BC_valid_dme_signal  = FALSE;
int    BC_valid_ndb_signal  = FALSE;
float  BC_n1_recip_radial = 0.0;
float  BC_n2_recip_radial = 0.0;

/*-----------------*/
/* Local Variables */    
/*-----------------*/

static int   Last_dist_ok = FALSE;
static float Last_dist_nm = 0.0;
static float Dme_time_interval = 0.0;

/*--------------------*/
/* Forward References */
/*--------------------*/

static void Update_nav1_dme_hsi(ll_pos *host_pos);
static void Set_pilot_hsi_head ();
static void Set_copilot_hsi_head ();
float BC_get_host_recip_radial(vhf_beac_attr *battr, ll_pos *host_pos);

/*----------------------------------------------------------------*/

/* The beacon routines require the host position in degrees,
   so convert once for general use
   
   Inputs: host position from states structure
*/

void BC_convert_host_pos(ll_pos *host_pos)
{
   host_pos->lat = states.latitude * RAD_TO_DEG;
   host_pos->lon = states.longitude * RAD_TO_DEG;
   host_pos->ht  = (float) -states.D;
}

/*----------------------------------------------------------------*/

/* When the NAV2 ils/vor instrument is being used for ILS.
   Convert calculated ils data into correct instrument data.

   Outputs: IO_Sen_out for nav2 instrument
*/

static void Set_ils_nav2_inst (ils_inst_out_data *ils_out)
{
   /* To/from not valid for ILS */
   IO_Sen_out->nav2_to_from_flag = CENTRE_NEEDLE;

   if (ils_out->valid_loc)
      IO_Sen_out->nav2_nav_flag = HIDE_FLAG;
   else
      IO_Sen_out->nav2_nav_flag = SHOW_FLAG;

   if (ils_out->valid_glide)
   {
      IO_Sen_out->nav2_GS_flag = HIDE_FLAG;
      IO_Sen_out->nav2_GS_needle = ils_out->vert_dev * 100.0f;
   }
   else
   {
      IO_Sen_out->nav2_GS_flag = SHOW_FLAG;
      IO_Sen_out->nav2_GS_needle = CENTRE_NEEDLE;
   }

   IO_Sen_out->nav2_left_right_needle = ils_out->lt_rt_dev * 100.0f;
}

/*----------------------------------------------------------------*/

static void Set_vor_nav2_inst (vor_inst_out_data *vor_out)
{
   /* Glideslope not valid for VOR */
   IO_Sen_out->nav2_GS_flag = SHOW_FLAG;;
   IO_Sen_out->nav2_GS_needle = CENTRE_NEEDLE;

   if (vor_out->valid)
      IO_Sen_out->nav2_nav_flag = HIDE_FLAG;
   else
      IO_Sen_out->nav2_nav_flag = SHOW_FLAG;

   if (vor_out->to_flag == TO_FLAG)
      IO_Sen_out->nav2_to_from_flag = TO_NEEDLE;
   else if (vor_out->to_flag == FROM_FLAG)
      IO_Sen_out->nav2_to_from_flag = FROM_NEEDLE;
   else
      IO_Sen_out->nav2_to_from_flag = CENTRE_NEEDLE;

   IO_Sen_out->nav2_left_right_needle = vor_out->radial_dev * 100.0f;
}

/*----------------------------------------------------------------*/

/* This is called when the freq on the NAV2 box is invalid or
   the NAV2 computer is turned off */

static void Set_no_signal_nav2_inst()
{
   IO_Sen_out->nav2_GS_flag = SHOW_FLAG;;
   IO_Sen_out->nav2_nav_flag = SHOW_FLAG;

/* Leave needles at previous positions
   IO_Sen_out->nav2_GS_needle = CENTRE_NEEDLE;
   IO_Sen_out->nav2_to_from_flag = CENTRE_NEEDLE;
   IO_Sen_out->nav2_left_right_needle = CENTRE_NEEDLE;
*/
}

/*----------------------------------------------------------------*/

/* This is called when there is no power or the dial has a fault */

static void Set_no_pwr_nav2_inst()
{
   IO_Sen_out->nav2_GS_flag = SHOW_FLAG;;
   IO_Sen_out->nav2_nav_flag = SHOW_FLAG;
   IO_Sen_out->nav2_GS_needle = CENTRE_NEEDLE;
   IO_Sen_out->nav2_left_right_needle = CENTRE_NEEDLE;

   /* Leave rest at previous positions */
}

/*----------------------------------------------------------------*/

/* This is called when there is no power or the dial has a fault */

static void Set_no_pwr_pilot_hsi()
{
   IO_Sen_out->pilot_hsi_hdg_flag = SHOW_FLAG;
   IO_Sen_out->pilot_hsi_nav_flag = SHOW_FLAG;
   /* Course deviation needle centres (since sprung) */
   IO_Sen_out->pilot_hsi_left_right_needle = CENTRE_NEEDLE;
   IO_Sen_out->pilot_hsi_GS_enable = DISABLE;

   /* Leave rest at previous positions */
}

/*----------------------------------------------------------------*/

/* This is called when there is no power or the dial has a fault */

static void Set_no_pwr_copilot_hsi()
{
   IO_Sen_out->copilot_hsi_hdg_flag = SHOW_FLAG;
   IO_Sen_out->copilot_hsi_nav_flag = SHOW_FLAG;
   /* Course deviation needle centres (since sprung) */
   IO_Sen_out->copilot_hsi_left_right_needle = CENTRE_NEEDLE;
   IO_Sen_out->copilot_hsi_GS_enable = DISABLE;

   /* Leave rest at previous positions */
}

/*----------------------------------------------------------------*/

/* This is called when there is no power or the dial has a fault */

static void Set_no_pwr_rmi()
{
   /* Leave everything at previous positions */
   return;
}

/*----------------------------------------------------------------*/

/* Direct copy of values from 1 structure to another

   Inputs: IO_Sen_out for pilot hsi
*/

static void Get_pilot_hsi_inst (hsi_out_data *hsi)
{
   hsi->GS_enable               = IO_Sen_out->pilot_hsi_GS_enable;
   hsi->GS_needle               = IO_Sen_out->pilot_hsi_GS_needle;
   hsi->nav_flag                = IO_Sen_out->pilot_hsi_nav_flag;
   hsi->left_right_needle       = IO_Sen_out->pilot_hsi_left_right_needle;
   hsi->to_from_needle          = IO_Sen_out->pilot_hsi_to_from_needle;
}

/*----------------------------------------------------------------*/

/* Direct copy of values from 1 structure to another

   Inputs: IO_Sen_out for copilot hsi
*/

static void Get_copilot_hsi_inst (hsi_out_data *hsi)
{
   hsi->GS_enable               = IO_Sen_out->copilot_hsi_GS_enable;
   hsi->GS_needle               = IO_Sen_out->copilot_hsi_GS_needle;
   hsi->nav_flag                = IO_Sen_out->copilot_hsi_nav_flag;
   hsi->left_right_needle       = IO_Sen_out->copilot_hsi_left_right_needle;
   hsi->to_from_needle          = IO_Sen_out->copilot_hsi_to_from_needle;
}

/*----------------------------------------------------------------*/

/* Direct copy of values from 1 structure to another

   Outputs: IO_Sen_out for pilot hsi
*/

static void Set_pilot_hsi_inst (hsi_out_data *hsi)
{
   IO_Sen_out->pilot_hsi_GS_enable               = hsi->GS_enable;
   IO_Sen_out->pilot_hsi_GS_needle               = hsi->GS_needle;
   IO_Sen_out->pilot_hsi_nav_flag                = hsi->nav_flag;
   IO_Sen_out->pilot_hsi_left_right_needle       = hsi->left_right_needle;
   IO_Sen_out->pilot_hsi_to_from_needle          = hsi->to_from_needle;
}

/*----------------------------------------------------------------*/

/* Direct copy of values from 1 structure to another

   Outputs: IO_Sen_out for copilot hsi
*/

static void Set_copilot_hsi_inst (hsi_out_data *hsi)
{
   IO_Sen_out->copilot_hsi_GS_enable               = hsi->GS_enable;
   IO_Sen_out->copilot_hsi_GS_needle               = hsi->GS_needle;
   IO_Sen_out->copilot_hsi_nav_flag                = hsi->nav_flag;
   IO_Sen_out->copilot_hsi_left_right_needle       = hsi->left_right_needle;
   IO_Sen_out->copilot_hsi_to_from_needle          = hsi->to_from_needle;
}

/*----------------------------------------------------------------*/

/* Used when the HSI is known to be set from an ILS signal */

static void Set_ils_copilot_hsi_inst (ils_inst_out_data *ils_out)
{
   /* To/from not valid for ILS */
   IO_Sen_out->copilot_hsi_to_from_needle = CENTRE_NEEDLE;

   if (ils_out->valid_loc)
      IO_Sen_out->copilot_hsi_nav_flag = HIDE_FLAG;
   else
      IO_Sen_out->copilot_hsi_nav_flag = SHOW_FLAG;

   if (ils_out->valid_glide)
   {
      IO_Sen_out->copilot_hsi_GS_enable = ENABLE;
      IO_Sen_out->copilot_hsi_GS_needle = ils_out->vert_dev * 100.0f;
   }
   else
   {
      IO_Sen_out->copilot_hsi_GS_enable = DISABLE;
      IO_Sen_out->copilot_hsi_GS_needle = CENTRE_NEEDLE;
   }

   IO_Sen_out->copilot_hsi_left_right_needle = ils_out->lt_rt_dev * 100.0f;
}

/*----------------------------------------------------------------*/

/* Used when the HSI is known to be set from a VOR signal in VOR mode */

static void Set_vor_copilot_hsi_inst (vor_inst_out_data *vor_out)
{
   /* Glideslope not valid for VOR */
   IO_Sen_out->copilot_hsi_GS_enable = DISABLE;
   IO_Sen_out->copilot_hsi_GS_needle = CENTRE_NEEDLE;

   if (vor_out->valid)
      IO_Sen_out->copilot_hsi_nav_flag = HIDE_FLAG;
   else
      IO_Sen_out->copilot_hsi_nav_flag = SHOW_FLAG;

   if (vor_out->to_flag == TO_FLAG)
      IO_Sen_out->copilot_hsi_to_from_needle = TO_NEEDLE;
   else if (vor_out->to_flag == FROM_FLAG)
      IO_Sen_out->copilot_hsi_to_from_needle = FROM_NEEDLE;
   else
      IO_Sen_out->copilot_hsi_to_from_needle = CENTRE_NEEDLE;

   IO_Sen_out->copilot_hsi_left_right_needle = vor_out->radial_dev * 100.0f;
}

/*----------------------------------------------------------------*/

/* This is called when the freq on the NAV2 box is invalid or
   the NAV2 computer is turned off */

static void Set_no_signal_copilot_hsi_inst()
{
   IO_Sen_out->copilot_hsi_GS_enable = DISABLE;
   IO_Sen_out->copilot_hsi_nav_flag = SHOW_FLAG;

   IO_Sen_out->copilot_hsi_left_right_needle = CENTRE_NEEDLE;

   /* Leave rest at previous positions */
}

/*----------------------------------------------------------------*/

/* Direct copy of values from 1 structure to another

   Inputs: IO_Sen_out for pilot rmi
*/

static void Get_pilot_rmi (rmi_out_data *rmi)
{
   rmi->demand_head = IO_Sen_out->RMI_demand_heading;
   rmi->ADF_needle  = IO_Sen_out->RMI_ADF_needle;
   rmi->VOR_needle  = IO_Sen_out->RMI_VOR_needle;
}

/*----------------------------------------------------------------*/

/* Direct copy of values from 1 structure to another

   Outputs: IO_Sen_out for pilot rmi
*/

static void Set_pilot_rmi (rmi_out_data *rmi)
{
   IO_Sen_out->RMI_demand_heading = rmi->demand_head;
   IO_Sen_out->RMI_ADF_needle     = rmi->ADF_needle;
   IO_Sen_out->RMI_VOR_needle     = rmi->VOR_needle;
}

/*----------------------------------------------------------------*/

/* This updates the dials that depend on beacon signals and
   also range based information on KNS80 box.

   Inputs:  IO_avionic_pwr

   Outputs: Related IO_Sen_out fields
            SC_nav2_audio
            SC_adf_audio
            (SC_nav1_audio
            SC_dme_audio)
            BC_valid_n2_b_signal
            BC_valid_n1_b_signal
            BC_valid_dme_signal
            BC_valid_ndb_signal
*/

void BC_update_beacon_dials()
{
   ils_inst_out_data ils_out;
   vor_inst_out_data vor_out;
   rmi_out_data      rmi;
   int               valid_signal;
   int               done_inst;
   float             recip_radial;

   BC_valid_n1_b_signal = FALSE;
   BC_valid_n2_b_signal = FALSE;
   BC_valid_dme_signal  = FALSE;
   BC_valid_ndb_signal  = FALSE;
   BC_n1_recip_radial = 0.0;
   BC_n2_recip_radial = 0.0;
   SC_nav2_audio = FALSE;
   SC_adf_audio = FALSE;
   SC_nav1_audio = FALSE;
   SC_dme_audio = FALSE;

   if (!IO_avionic_pwr)
   {
      Set_no_pwr_nav2_inst();
      Set_no_pwr_pilot_hsi();
      Set_no_pwr_copilot_hsi();
      Set_no_pwr_rmi();
      return;
   }

   /* If the dials have faults, similar routines need to be called */

   /* On the HSI, heading data is independent of navigation data */
   Set_pilot_hsi_head();
   Set_copilot_hsi_head();


   /*-----------------------------------------------------*/
   /* NAV2
   */
   /*-----------------------------------------------------*/

   /* Update NAV2 dial (VOR & ILS) AND copilot HSI */
   valid_signal = FALSE;
   done_inst = FALSE;

   if (IO_nav2_battr.valid_flag &&
       (IO_Sen_in->KX155_off_on_switch == SW_ON) && (!IO_Sen_faults.AVF.com_nav2) &&
       (IO_nav2_battr.valid_vor_ils || IO_nav2_battr.valid_glide_tx))
   {
      if (IO_nav2_battr.type == ILS_TYPE)
      {
         //If this is FALSE, there may still be a valid glide slope signal
         valid_signal = ILS_valid_ils_signal(&IO_nav2_battr, &BC_host_pos, &recip_radial);
         if (valid_signal)
         {
            ILS_calc_ils_data (&IO_nav2_battr, &BC_host_pos, valid_signal, recip_radial, &ils_out);
            Set_ils_nav2_inst (&ils_out);
            
#ifdef NAV2_COPILOT_HSI
            Set_ils_copilot_hsi_inst (&ils_out);
#endif
         }
         else
         {
            ILS_calc_ils_data (&IO_nav2_battr, &BC_host_pos, FALSE, 0.0f, &ils_out);
            if (ils_out.valid_glide)
            {
               Set_ils_nav2_inst (&ils_out);
               done_inst = TRUE;

#ifdef NAV2_COPILOT_HSI
               Set_ils_copilot_hsi_inst (&ils_out);
#endif
            }
         }
      }
      else
      {
         valid_signal = VOR_valid_vor_signal(&IO_nav2_battr, &BC_host_pos);
         if (valid_signal)
         {
            recip_radial = BC_get_host_recip_radial(&IO_nav2_battr, &BC_host_pos);
            VOR_calc_vor_data (valid_signal, recip_radial, IO_Sen_in->nav2_obs,
                               &vor_out);
            Set_vor_nav2_inst (&vor_out);

#ifdef NAV2_COPILOT_HSI
            VOR_calc_vor_data (valid_signal, recip_radial, IO_Sen_in->copilot_hsi_obs,
                               &vor_out);
            Set_vor_copilot_hsi_inst (&vor_out);
#endif
         }
      }
   }

   BC_valid_n2_b_signal = valid_signal;

   if (valid_signal)
   {
      //Note that for ILS and Glide, the ILS controlls the audio
      SC_nav2_audio = TRUE;
      BC_n2_recip_radial = recip_radial;
   }
   else
   {
      if (!done_inst)
      {
         Set_no_signal_nav2_inst();
#ifdef NAV2_COPILOT_HSI
         Set_no_signal_copilot_hsi_inst();
#endif
      }
   }


   /*-----------------------------------------------------*/
   /* NAV1

      This involves updates to the KNS80 box for DME
      calculations and updates to the HSI. The HSI may
      use some of the DME data, depending on what mode
      the NAV1 is in
   */
   /*-----------------------------------------------------*/

   /* BC_valid_n1_b_signal and BC_n1_recip_radial are also
      set up here. */

   Update_nav1_dme_hsi(&BC_host_pos);



   /*-----------------------------------------------------*/
   /* ADF
   */
   /*-----------------------------------------------------*/

   /* Update RMI dial (NDB/ADF). There is a single RMI on pilot's side */

   Get_pilot_rmi (&rmi);
   /* The magnetic heading on 1 side is used by the RMI on the other side */
   RMI_set_rmi (&IO_ndb_battr, &BC_host_pos, CS_gyro_copilot_mhead, &valid_signal, &rmi );
   Set_pilot_rmi (&rmi);

   BC_valid_ndb_signal = valid_signal;

   if (valid_signal)
   {
      SC_adf_snd_type = MORSE_TYPE;
      /* If frequency is not in normal audible range, need BFO switch on to hear it */
      if (IO_ndb_battr.mod_freq_flag)   //Audible
      {
         SC_adf_audio = TRUE;
         if (IO_Sen_in->KR87_BFO_switch == SW_OFF)
            SC_adf_snd_type = MORSE_TYPE;
         else
            SC_adf_snd_type = BFO_TONE_TYPE;
      }
      else
      {
         if (IO_Sen_in->KR87_BFO_switch == SW_ON)
         {
            SC_adf_audio = TRUE;
            SC_adf_snd_type = BFO_ON_MORSE_TYPE;
         }
      }
   }
}

/*----------------------------------------------------------------------*/
/*----------------------------------------------------------------*/

/* Inputs: true_head in range 0 to 360
           magnetic variation: +ve =>West, -ve => East
*/

float BC_make_magnetic(float true_head, float magnetic_var)
{
   float mag_head;

   mag_head = true_head + magnetic_var;

   /* Ensure mag_head is in range 0 to 360 */
   if (mag_head >= 360.0)
      mag_head -= 360.0;
   if (mag_head < 0.0)
      mag_head += 360.0;

   return mag_head;
}

/*----------------------------------------------------------------------*/

/* This gives the bearing from beacon to host, in range 0 to 360
   Inputs: Beacon lat/lon
           Magnetic variation of beacon
           Host lat/lon
*/

float BC_get_host_radial(vhf_beac_attr *battr, ll_pos *host_pos)
{
   float bear_beac_host;
   float mbear_beac_host;

   bear_beac_host = SPH_get_sph_heading (battr->beac_lat, battr->beac_lon,
                                         host_pos->lat, host_pos->lon);
   /* Convert to magnetic bearing (since radials are specified with
      respect to magnetic north) */
   mbear_beac_host = BC_make_magnetic(bear_beac_host, battr->magnetic_var);

   return (mbear_beac_host);
}

/*----------------------------------------------------------------------*/

/* This gives the bearing from host to beacon, in range 0 to 360
   Inputs: Beacon lat/lon
           Magnetic variation of beacon
           Host lat/lon
*/

float BC_get_host_recip_radial(vhf_beac_attr *battr, ll_pos *host_pos)
{
   float bear_host_beac;
   float mbear_host_beac;

   bear_host_beac = SPH_get_sph_heading (host_pos->lat, host_pos->lon,
                                         battr->beac_lat, battr->beac_lon);
   /* Convert to magnetic bearing */
   mbear_host_beac = BC_make_magnetic(bear_host_beac, battr->magnetic_var);

   return (mbear_host_beac);
}

/*----------------------------------------------------------------------*/

static float Recip_radial_to_radial(float recip_radial)
{
   float radial;

   radial = recip_radial - 180.0f;
   
   /* Keep in range 0 to 360 */
   if (radial >= 360.0)
      radial -= 360.0;
   if (radial < 0.0)
      radial += 360.0;

   return radial;
}

/*----------------------------------------------------------------------*/

/* Returns TRUE if beacon has a valid ILS or glide signal */

static int Ils_beacon(vhf_beac_attr *battr)
{
   if (battr->valid_flag &&
       (battr->valid_vor_ils || battr->valid_glide_tx) &&
       (battr->type == ILS_TYPE))
      return TRUE;
   else
      return FALSE;
}

/*----------------------------------------------------------------------*/

void BC_reset_KT_MIN_calc()
{
   Last_dist_ok = FALSE;
   Last_dist_nm = 0.0;
   Dme_time_interval = 0.0;

   //Need to initialize these somewhere
   IO_Sen_out->KNS80_KT_display_mode  = KNS80_KT_DASH;
   IO_Sen_out->KNS80_MIN_display_mode = KNS80_MIN_DASH;
}

/*----------------------------------------------------------------------*/

/* The distance passed is either distance to beacon or distance
   to waypoint and Last_dist_nm is the equivalent distance last
   time interval.
   
   Inputs:  Last_dist_ok
            Last_dist_nm

   Outputs: KNS80_KT_display
            KNS80_MIN_display
            KNS80_KT_display_mode
            KNS80_MIN_display_mode
            Last_dist_ok
            Last_dist_nm
*/

static void Set_KT_and_MIN(float dist_nm)
{
   float delta_dist;
   float speed_kt;
   float time_to_go = 0.0;

   if (Last_dist_ok)
   {
      Dme_time_interval += delta_time;
      if (Dme_time_interval > DME_DTIME_CALC)
      {
         IO_Sen_out->KNS80_KT_display_mode  = ACTIVE;

         delta_dist = Last_dist_nm - dist_nm;
         speed_kt = (delta_dist / Dme_time_interval) * 3600.0f;   /* nm per hour */
         /* Time to go is in minutes */
         if (speed_kt <= 1.0)
         {
            IO_Sen_out->KNS80_MIN_display_mode = KNS80_MIN_DASH;
            if (speed_kt < 0.0)
               speed_kt = -speed_kt;
         }
         else
         {
            IO_Sen_out->KNS80_MIN_display_mode = ACTIVE;
            time_to_go = (dist_nm / speed_kt) * 60.0f;
         }

         IO_Sen_out->KNS80_KT_display = (long) (speed_kt + 0.5);
         if (time_to_go > 99.0)
            IO_Sen_out->KNS80_MIN_display_mode = KNS80_MIN_DASH;
         else
            IO_Sen_out->KNS80_MIN_display = (long) (time_to_go + 0.5);

         Last_dist_nm = dist_nm;
         Dme_time_interval = 0.0;
      }
   }
   else
   {
      IO_Sen_out->KNS80_KT_display_mode  = KNS80_KT_DASH;
      IO_Sen_out->KNS80_MIN_display_mode = KNS80_MIN_DASH;

      Last_dist_nm = dist_nm;
      Last_dist_ok = TRUE;
      Dme_time_interval = 0.0;
   }
}

/*----------------------------------------------------------------------*/

/* Note that the special case of no IO_avionic_pwr has been done
   outside this routine.

   Inputs:  CS_gyro_*_mhead
            KNS80_off_on_switch
            IO_nav1_battr
            IO_dme_battr
            N1A_dme_hold
            
   Outputs: BC_valid_n1_b_signal
            BC_n1_recip_radial
            SC_nav1_audio
            SC_dme_audio
            KNS80 box displays
*/

static void Update_nav1_dme_hsi(ll_pos *host_pos)
{
   float srange_sq_nm;
   float srange_nm;
   int   valid_dme_signal;
   int   valid_hsi_dme;
   int   valid_signal;
   float recip_radial;
   char  dme_possible;
   int   valid_dme_display;
   int   ils_beacon;
   float wp_dist, wp_dist_sq;   /* Nautical miles */
   float wp_radial;             /* Degrees */
   float host_radial;
   float wp_host_dist_nm;
   float pilot_cross_dev = 0.0;
   hsi_out_data pilot_hsi_out;
   char  pilot_to_flag;
#ifndef NAV2_COPILOT_HSI
   float copilot_cross_dev = 0.0;
   hsi_out_data copilot_hsi_out;
   char  copilot_to_flag;
#endif

   /* If the beacon is invalid, the HSI routine does not change all the
      values. We leave them at their old values, so first copy these old
      values */

   Get_pilot_hsi_inst (&pilot_hsi_out);
#ifndef NAV2_COPILOT_HSI
   Get_copilot_hsi_inst (&copilot_hsi_out);
#endif


   /* Initialize */
   BC_valid_n1_b_signal = FALSE;
   BC_valid_dme_signal = FALSE;
   BC_n1_recip_radial = 0.0;
   SC_nav1_audio = FALSE;
   SC_dme_audio = FALSE;
   valid_signal = FALSE;
   valid_dme_signal = FALSE;
   ils_beacon = FALSE;


   if ((IO_Sen_in->KNS80_off_on_switch == SW_OFF) || (IO_Sen_faults.AVF.nav1))
   {
      /* No signals. The KNS80 box is blanked by N1A_nav1_button_changes() */
      
      HSI_set_hsi_nav (&IO_nav1_battr, host_pos, FALSE, FALSE, 0.0, IO_Sen_in->pilot_hsi_obs,
                       pilot_cross_dev, &pilot_hsi_out);
      Set_pilot_hsi_inst (&pilot_hsi_out);

#ifndef NAV2_COPILOT_HSI
      HSI_set_hsi_nav (&IO_nav1_battr, host_pos, FALSE, FALSE, 0.0, c,
                       copilot_cross_dev, &copilot_hsi_out);
      Set_copilot_hsi_inst (&copilot_hsi_out);
#endif

      return;
   }


   /* Whatever mode of operation the NAV1 is in, we use the same test for
      a valid beacon signal */
   ils_beacon = Ils_beacon(&IO_nav1_battr);
   if (ils_beacon)
      valid_signal = ILS_valid_ils_signal(&IO_nav1_battr, host_pos, &recip_radial);
   else
   {
      valid_signal = VOR_valid_vor_signal(&IO_nav1_battr, host_pos);
      if (valid_signal)
         recip_radial = BC_get_host_recip_radial(&IO_nav1_battr, host_pos);
   }


   BC_valid_n1_b_signal = valid_signal;

   if (valid_signal)
   {
      SC_nav1_audio = TRUE;
      BC_n1_recip_radial = recip_radial;
   }

   if (N1A_dme_hold)
   {
      /* Take in to account the current N1A_nav1_mode. In RNAV modes,
         dme_hold makes everything invalid */
      /* NB. We do not test this in VOR_PAR mode since the KNS80 values
         are still valid BUT it does affect the HSI (see below) */

      dme_possible = TRUE;

      if ((N1A_nav1_mode == NAV1_RNV) ||
          (N1A_nav1_mode == NAV1_RNV_APR))
      {
         if (ils_beacon)
         {
            /* RNAV modes not in use due to ILS */
            ;
         }
         else
         {
            /* If DME is holding to a different frequency to VOR, then
               invalid in this mode.
               This test has already been done in controlling the
               HLD token */
            if (IO_Sen_out->KNS80_HLD_token == TOKEN_ON)
               dme_possible = FALSE;
         }
      }

      if (dme_possible)
      {
         if (IO_dme_battr.valid_flag && IO_dme_battr.valid_dme)
         {
            DME_get_srange_nm (&IO_dme_battr, host_pos, &srange_nm, &valid_dme_signal);
         }
      }
   }
   else if (IO_nav1_battr.valid_flag && IO_nav1_battr.valid_dme)
   {
      DME_get_srange_nm (&IO_nav1_battr, host_pos, &srange_nm, &valid_dme_signal);
   }

   pilot_cross_dev = 0.0;
#ifndef NAV2_COPILOT_HSI
   copilot_cross_dev = 0.0;
#endif
   valid_dme_display = FALSE;

   if (valid_dme_signal)
   {
      SC_dme_audio = TRUE;
      BC_valid_dme_signal = valid_dme_signal;
      valid_dme_display = TRUE;

      IO_Sen_out->KNS80_NM_display_mode  = ACTIVE;

/* This is done in Set_KT_and_MIN and initialized in BC_reset_KT_MIN_calc
      IO_Sen_out->KNS80_KT_display_mode  = ACTIVE;
      IO_Sen_out->KNS80_MIN_display_mode = ACTIVE;
*/

      if ((N1A_nav1_mode == NAV1_VOR) ||
          (N1A_nav1_mode == NAV1_VOR_PAR) ||
          ils_beacon)
      {
         /* Display units are 10th of a nm */
         IO_Sen_out->KNS80_NM_display = (long) ((srange_nm * 10.0) + 0.5);

         /* Get approach speed and time */
         Set_KT_and_MIN(srange_nm);

         /* Calculate the dme based values used later by the HSI */
         if (BC_valid_n1_b_signal)
         {
            if (!ils_beacon)
            {
               if (N1A_nav1_mode == NAV1_VOR_PAR)
               {
                  pilot_cross_dev = RN_get_vor_par_cross_dev(BC_n1_recip_radial, IO_Sen_in->pilot_hsi_obs, srange_nm);
#ifndef NAV2_COPILOT_HSI
                  copilot_cross_dev = RN_get_vor_par_cross_dev(BC_n1_recip_radial, IO_Sen_in->copilot_hsi_obs, srange_nm);
#endif
               }
            }
         }
      }
      else  /* Both RNV modes */
      {
         /* In these modes, both beacon signal and dme must be valid for any
            display on KNS80 box */
         if (BC_valid_n1_b_signal)
         {
            srange_sq_nm = srange_nm * srange_nm;
            wp_dist = N1A_nav1_get_active_wp_dist();
            wp_dist_sq = N1A_nav1_get_active_wp_dist_sq();
            wp_radial = N1A_nav1_get_active_wp_radial();
            host_radial = Recip_radial_to_radial(BC_n1_recip_radial);

            /* Want distance from host to waypoint */
            RN_nav1_wpoint_dist(host_radial, wp_dist, wp_dist_sq, wp_radial,
                                srange_nm, srange_sq_nm, &wp_host_dist_nm);

            /* Display units are 10th of a nm */
            IO_Sen_out->KNS80_NM_display = (long) ((wp_host_dist_nm * 10.0) + 0.5);

            /* Get approach speed and time */
            Set_KT_and_MIN(wp_host_dist_nm);

            RN_nav1_wpoint_dev(host_radial, wp_dist_sq, wp_radial,
                               srange_nm, srange_sq_nm, wp_host_dist_nm, 
                               IO_Sen_in->pilot_hsi_obs, &pilot_cross_dev, &pilot_to_flag);

            /* Now set up the to/from value */
            if (pilot_to_flag == TO_FLAG)
               pilot_hsi_out.to_from_needle = TO_NEEDLE;
            else if (pilot_to_flag == FROM_FLAG)
               pilot_hsi_out.to_from_needle = FROM_NEEDLE;
            else
               pilot_hsi_out.to_from_needle = CENTRE_NEEDLE;

#ifndef NAV2_COPILOT_HSI
            RN_nav1_wpoint_dev(host_radial, wp_dist_sq, wp_radial,
                               srange_nm, srange_sq_nm, wp_host_dist_nm, 
                               IO_Sen_in->copilot_hsi_obs, &copilot_cross_dev, &copilot_to_flag);

            if (copilot_to_flag == TO_FLAG)
               copilot_hsi_out.to_from_needle = TO_NEEDLE;
            else if (copilot_to_flag == FROM_FLAG)
               copilot_hsi_out.to_from_needle = FROM_NEEDLE;
            else
               copilot_hsi_out.to_from_needle = CENTRE_NEEDLE;
#endif

         }   /* End of valid_signal */
         else
         {
            valid_dme_display = FALSE;
         }
      }   /* End of RNV modes */
   }   /* End of valid dme */

   if (!valid_dme_display)
   {
      /* Dash the dme displays */
      IO_Sen_out->KNS80_NM_display_mode  = KNS80_NM_DASH;
      IO_Sen_out->KNS80_KT_display_mode  = KNS80_KT_DASH;
      IO_Sen_out->KNS80_MIN_display_mode = KNS80_MIN_DASH;

      BC_reset_KT_MIN_calc();
   }


   /* Now update the HSI dials. Some of the inputs have been calculated above,
      depending on the N1A_nav1_mode */

   /* In VOR_PAR mode, dme_hold can affect the HSI but not the above,
      so adjust the dme_valid flag if necessary */

   valid_hsi_dme = valid_dme_signal;
   if (valid_dme_signal)
   {
      if (!ils_beacon)
      {
         if (N1A_dme_hold && (N1A_nav1_mode == NAV1_VOR_PAR))
         {
            /* If DME is holding to a different frequency to VOR, then
               invalid in this mode.
               This test has already been done in controlling the
               HLD token */
            if (IO_Sen_out->KNS80_HLD_token == TOKEN_ON)
               valid_hsi_dme = FALSE;
         }
      }
   }

   HSI_set_hsi_nav (&IO_nav1_battr, host_pos, BC_valid_n1_b_signal, valid_hsi_dme,
                    BC_n1_recip_radial, IO_Sen_in->pilot_hsi_obs, pilot_cross_dev,
                    &pilot_hsi_out);

   Set_pilot_hsi_inst (&pilot_hsi_out);

#ifndef NAV2_COPILOT_HSI
   HSI_set_hsi_nav (&IO_nav1_battr, host_pos, BC_valid_n1_b_signal, valid_hsi_dme,
                    BC_n1_recip_radial, IO_Sen_in->copilot_hsi_obs, copilot_cross_dev,
                    &copilot_hsi_out);

   Set_copilot_hsi_inst (&copilot_hsi_out);
#endif
}

/*----------------------------------------------------------------*/

/* This routine determines the HDG related data for the HSI.
   This is independent of the NAV related data.

   Inputs:  IO_elec_pwr
            CS_gyro_ok_speed  (We use a single flag for both gyros)
            CS_pilot_hdg_flag
*/

static void Set_pilot_hsi_head ()
{
   /* We need to show the compass position slowly turn
      during gyro run up */

   if (!IO_elec_pwr)
   {
      IO_Sen_out->pilot_hsi_hdg_flag = SHOW_FLAG;
   }
   else
   {
      if (!CS_gyro_ok_speed)
         IO_Sen_out->pilot_hsi_hdg_flag = SHOW_FLAG;
      else if (CS_pilot_hdg_flag == TRUE)
         IO_Sen_out->pilot_hsi_hdg_flag = SHOW_FLAG;
      else
         IO_Sen_out->pilot_hsi_hdg_flag = HIDE_FLAG;

      /* Compass card rotated so that heading is at top */
      IO_Sen_out->pilot_hsi_compass_demand_position = CS_gyro_pilot_mhead;   /* In degrees */
   }
}

/*----------------------------------------------------------------*/

/* This routine determines the HDG related data for the HSI.
   This is independent of the NAV related data.

   Inputs:  IO_elec_pwr
            CS_gyro_ok_speed  (We use a single flag for both gyros)
            CS_copilot_hdg_flag
*/

static void Set_copilot_hsi_head ()
{
   /* We need to show the compass position slowly turn
      during gyro run up */

   if (!IO_elec_pwr)
   {
      IO_Sen_out->copilot_hsi_hdg_flag = SHOW_FLAG;
   }
   else
   {
      if (!CS_gyro_ok_speed)
         IO_Sen_out->copilot_hsi_hdg_flag = SHOW_FLAG;
      else if (CS_copilot_hdg_flag == TRUE)
         IO_Sen_out->copilot_hsi_hdg_flag = SHOW_FLAG;
      else
         IO_Sen_out->copilot_hsi_hdg_flag = HIDE_FLAG;

      /* Compass card rotated so that heading is at top */
      IO_Sen_out->copilot_hsi_compass_demand_position = CS_gyro_copilot_mhead;   /* In degrees */
   }
}

/*----------------------------------------------------------------------*/
