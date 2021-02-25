/*
File  : nav_dbug.c
Prefix: DDG_

Module: Dynamics for Seneca
Created: 11 February 2000

Description: This file .......
*/

/***************************/
/*   RCS Markers           */
/***************************/

/*
$Header: C:/MASTER/SENECA/NAV/RCS/nav_dbug.cpp 1.1 2000/04/17 12:29:49 juliae Exp $
$Log: nav_dbug.cpp $
Revision 1.1  2000/04/17 12:29:49  juliae
Initial revision

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
#include "c:\bae-sims\seneca\nav/beacon.h"
#include "c:\bae-sims\seneca\define.h"

/*----------------*/
/* Defines        */
/*----------------*/

#define TOKEN_ON   LIT
#define TOKEN_OFF  UNLIT

/*----------------*/
/* Typedefs       */
/*----------------*/

/*--------------------*/
/* External Routines  */
/*--------------------*/

extern sen_in_struct  *IO_Sen_in;
extern OUTPUTin			*OUTPUTinbox;//simulinkudp
extern sen_out_struct *IO_Sen_out;
extern vhf_beac_attr IO_nav1_battr;
extern vhf_beac_attr IO_nav2_battr;

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

void NDG_debug_adf_box()
{
   long tmp1, tmp2;

   printf("\n-------------------------------------------------------\n");

   if (IO_Sen_out->KR87_FLT_token == TOKEN_ON)
      printf("                                                     FLT\r");

   if (IO_Sen_out->KR87_STBY_TIME_display_mode == BLANK)
      printf("                                             BLANK\r");
   else if (IO_Sen_out->KR87_STBY_TIME_display_mode == TEST)
      printf("                                             TEST\r");
   else
   {
      if (IO_Sen_out->KR87_colon_token == TOKEN_ON)
      {
         tmp1 = IO_Sen_out->KR87_STBY_TIME_display / 100;
         tmp2 = IO_Sen_out->KR87_STBY_TIME_display % 100;
         printf("                                         %3d:%2d\r", tmp1, tmp2);
      }
      else
      {
         if (IO_Sen_out->KR87_STBY_TIME_display_mode == KR87_STBY_LEADING_ZERO)
            printf("                                         %05d\r", IO_Sen_out->KR87_STBY_TIME_display);
         else
            printf("                                         %5d\r", IO_Sen_out->KR87_STBY_TIME_display);
      }
   }

   if (IO_Sen_out->KR87_BFO_token == TOKEN_ON)
      printf("                            BFO\r");

   if (IO_Sen_out->KR87_USE_display_mode == BLANK)
      printf("                 BLANK\r", IO_Sen_out->KR87_USE_display);
   else if (IO_Sen_out->KR87_USE_display_mode == TEST)
      printf("                 TEST\r", IO_Sen_out->KR87_USE_display);
   else
      printf("                 %d\r", IO_Sen_out->KR87_USE_display);

   if (IO_Sen_out->KR87_ANT_token == TOKEN_ON)
      printf("ANT\r");


   printf("\n");   /* Second line */

   if (IO_Sen_out->KR87_ET_token == TOKEN_ON)
      printf("                                                      ET\r");

   if (IO_Sen_out->KR87_FRQ_token == TOKEN_ON)
      printf("                              FRQ\r");

   if (IO_Sen_out->KR87_ADF_token == TOKEN_ON)
      printf("ADF\r");



   printf("\n-------------------------------------------------------\n");
}

/*----------------------------------------------------------------------*/

void NDG_debug_nav1_box()
{
   long tmp1, tmp2;

   printf("\n-------------------------------------------------------\n");

   if (IO_Sen_out->KNS80_FRQ_RAD_DST_display_mode == BLANK)
      printf("                                            BLANK \r");
   else if (IO_Sen_out->KNS80_FRQ_RAD_DST_display_mode == TEST)
      printf("                                            ------\r");
   else
   {
      /* There is always a decimal point after the 1st 3 digits */
      tmp1 = IO_Sen_out->KNS80_FREQ_RAD_DST_display / 100;
      tmp2 = IO_Sen_out->KNS80_FREQ_RAD_DST_display % 100;
      printf("                                              %3d.%2d\r", tmp1, tmp2);
   }
   
   if (IO_Sen_out->KNS80_MIN_display_mode == BLANK)
      printf("                         BLANK \r");
   else if (IO_Sen_out->KNS80_MIN_display_mode == KNS80_MIN_DASH)
      printf("                         ------\r");
   else
   {
      printf("                         %d\r", IO_Sen_out->KNS80_MIN_display);
   }

   if (IO_Sen_out->KNS80_KT_display_mode == BLANK)
      printf("              BLANK\r");
   else if (IO_Sen_out->KNS80_KT_display_mode == KNS80_KT_DASH)
      printf("              ------\r");
   else
   {
      printf("              %d\r", IO_Sen_out->KNS80_KT_display);
   }

   if (IO_Sen_out->KNS80_NM_display_mode == BLANK)
      printf("  BLANK\r");
   else if (IO_Sen_out->KNS80_NM_display_mode == KNS80_NM_DASH)
      printf("  ------\r");
   else
   {
      tmp1 = IO_Sen_out->KNS80_NM_display / 10;
      tmp2 = IO_Sen_out->KNS80_NM_display % 10;
      printf("  %d.%d\r", tmp1, tmp2);
   }


   printf("\n");   /* Second line */


   if (IO_Sen_out->KNS80_DST_token == TOKEN_ON)
      printf("                                                DST\r");

   if (IO_Sen_out->KNS80_RAD_token == TOKEN_ON)
      printf("                                           RAD\r");

   if (IO_Sen_out->KNS80_FREQ_token == TOKEN_ON)
      printf("                                      FRQ\r");

   if (IO_Sen_out->KNS80_MIN_token == TOKEN_ON)
      printf("                         MIN\r");

   if (IO_Sen_out->KNS80_KT_token == TOKEN_ON)
      printf("              KT\r");

   if (IO_Sen_out->KNS80_NM_token == TOKEN_ON)
      printf("  NM\r");


   printf("\n");   /* Third line */

   if (IO_Sen_out->KNS80_DSP_display_mode == KNS80_DSP_BLANK)
      printf("                                          BLANK \r");
   else if (IO_Sen_out->KNS80_DSP_display_mode == TEST)
      printf("                                          ------\r");
   else
   {
      printf("                                            %d\r",
                                       IO_Sen_out->KNS80_DSP_display);
   }

   if (IO_Sen_out->KNS80_USE_display_mode == BLANK)
      printf("                             BLANK\r");
   else if (IO_Sen_out->KNS80_USE_display_mode == TEST)
      printf("                             ------\r");
   else
   {
      printf("                             %d\r", IO_Sen_out->KNS80_USE_display);
   }

   if (IO_Sen_out->KNS80_APR_token == TOKEN_ON)
      printf("                 APR\r");

   if (IO_Sen_out->KNS80_ENR_token == TOKEN_ON)
      printf("            ENR\r");

   if (IO_Sen_out->KNS80_PAR_token == TOKEN_ON)
      printf("PAR\r");


   printf("\n");   /* Fourth line */

   if (IO_Sen_out->KNS80_DSP_token == TOKEN_ON)
      printf("                                          DSP\r");

   if (IO_Sen_out->KNS80_USE_token == TOKEN_ON)
      printf("                             USE\r");

   if (IO_Sen_out->KNS80_ILS_token == TOKEN_ON)
      printf("                        ILS\r");

   if (IO_Sen_out->KNS80_HLD_token == TOKEN_ON)
      printf("                    HLD\r");

   if (IO_Sen_out->KNS80_RNV_token == TOKEN_ON)
      printf("              RNV\r");

   if (IO_Sen_out->KNS80_VOR_token == TOKEN_ON)
      printf("VOR\r");

   printf("\n-------------------------------------------------------\n");
}

/*----------------------------------------------------------------------*/

void NDG_debug_nav2_box()
{
   long tmp1, tmp2;

   printf("\n-------------------------------------------------------\n");

   if (IO_Sen_out->KX155_mode == BLANK)
   {
      printf("            BLANK                 BLANK       BLANK \r");
      if (IO_Sen_out->KX155_T_token == TOKEN_ON)
         printf("        T\r");
      printf("BLANK\r");
   }
   else if (IO_Sen_out->KX155_mode == TEST)
   {
      printf("            ------                ------      ------\r");
      if (IO_Sen_out->KX155_T_token == TOKEN_ON)
         printf("        T\r");
      printf("------\r");
   }
   else
   {
      /* There is always a decimal point after the 1st 3 digits */
      tmp1 = IO_Sen_out->KX155_NAV_STBY_display / 100;
      tmp2 = IO_Sen_out->KX155_NAV_STBY_display % 100;
      printf("                                              %3d.%2d\r", tmp1, tmp2);

      tmp1 = IO_Sen_out->KX155_NAV_USE_display / 100;
      tmp2 = IO_Sen_out->KX155_NAV_USE_display % 100;
      printf("                                  %3d.%2d\r", tmp1, tmp2);

      tmp1 = IO_Sen_out->KX155_COMM_STBY_display / 100;
      tmp2 = IO_Sen_out->KX155_COMM_STBY_display % 100;
      printf("             %3d.%2d\r", tmp1, tmp2);

      if (IO_Sen_out->KX155_T_token == TOKEN_ON)
         printf("        T\r");

      tmp1 = IO_Sen_out->KX155_COMM_USE_display / 100;
      tmp2 = IO_Sen_out->KX155_COMM_USE_display % 100;
      printf("%3d.%2d\r", tmp1, tmp2);
   }

   printf("\n-------------------------------------------------------\n");
}

/*----------------------------------------------------------------------*/

void NDG_debug_com1_box()
{
   long tmp1, tmp2;

   printf("\n-------------------------------------------------------\n");

   if (IO_Sen_out->KY196_mode == BLANK)
      printf("                               BLANK \r");
   else
   {
      tmp1 = IO_Sen_out->KY196_STBY_display / 100;
      tmp2 = IO_Sen_out->KY196_STBY_display % 100;
      printf("                               %3d.%2d \r", tmp1, tmp2);
   }

   if (IO_Sen_out->KY196_T_token == TOKEN_ON)
      printf("                   T\r");

   if (IO_Sen_out->KY196_mode == BLANK)
      printf("BLANK \r");
   else
   {
      tmp1 = IO_Sen_out->KY196_USE_display / 100;
      tmp2 = IO_Sen_out->KY196_USE_display % 100;
      printf("%3d.%2d \r", tmp1, tmp2);
   }

   printf("\n-------------------------------------------------------\n");
}

/*----------------------------------------------------------------------*/
/*----------------------------------------------------------------------*/

void NDG_debug_nav2_dial()
{
   printf("\nIO_nav2_battr.valid_flag = %d", IO_nav2_battr.valid_flag);

   printf("\nnav2_nav_flag = %s", (IO_Sen_out->nav2_nav_flag == SHOW_FLAG) ? "SHOW": "HIDE");
   printf("\nnav2_GS_flag = %s", (IO_Sen_out->nav2_GS_flag == SHOW_FLAG) ? "SHOW": "HIDE");
   printf("\nnav2_to_from_flag = %.1f", IO_Sen_out->nav2_to_from_flag);
   printf("\nnav2_left_right_needle = %.1f", IO_Sen_out->nav2_left_right_needle);
   printf("\nnav2_GS_needle = %.1f", IO_Sen_out->nav2_GS_needle);
   
}

/*----------------------------------------------------------------------*/

void NDG_debug_pilot_hsi()
{
   printf("\nIO_nav1_battr.valid_flag = %d", IO_nav1_battr.valid_flag);

   printf("\npilot_hsi_compass_demand_position = %.1f", IO_Sen_out->pilot_hsi_compass_demand_position);
   printf("\npilot_hsi_GS_enable = %s", (IO_Sen_out->pilot_hsi_GS_enable == ENABLE) ? "ENABLE": "DISABLE");
   printf("\npilot_hsi_GS_needle = %.1f", IO_Sen_out->pilot_hsi_GS_needle);
   printf("\npilot_hsi_nav_flag = %s", (IO_Sen_out->pilot_hsi_nav_flag == SHOW_FLAG) ? "SHOW": "HIDE");
   printf("\npilot_hsi_hdg_flag = %s", (IO_Sen_out->pilot_hsi_hdg_flag == SHOW_FLAG) ? "SHOW": "HIDE");
   printf("\npilot_hsi_left_right_needle = %.1f", IO_Sen_out->pilot_hsi_left_right_needle);
   printf("\npilot_hsi_to_from_needle = %.1f", IO_Sen_out->pilot_hsi_to_from_needle);
}

/*----------------------------------------------------------------------*/

void NDG_debug_copilot_hsi()
{
   printf("\ncopilot_hsi_compass_demand_position = %.1f", IO_Sen_out->copilot_hsi_compass_demand_position);
   printf("\ncopilot_hsi_GS_enable = %s", (IO_Sen_out->copilot_hsi_GS_enable == ENABLE) ? "ENABLE": "DISABLE");
   printf("\ncopilot_hsi_GS_needle = %.1f", IO_Sen_out->copilot_hsi_GS_needle);
   printf("\ncopilot_hsi_nav_flag = %s", (IO_Sen_out->copilot_hsi_nav_flag == SHOW_FLAG) ? "SHOW": "HIDE");
   printf("\ncopilot_hsi_hdg_flag = %s", (IO_Sen_out->copilot_hsi_hdg_flag == SHOW_FLAG) ? "SHOW": "HIDE");
   printf("\ncopilot_hsi_left_right_needle = %.1f", IO_Sen_out->copilot_hsi_left_right_needle);
   printf("\ncopilot_hsi_to_from_needle = %.1f", IO_Sen_out->copilot_hsi_to_from_needle);
}

/*----------------------------------------------------------------------*/

void NDG_debug_ndb_dial()
{
   printf("\nRMI_demand_heading = %.1f", IO_Sen_out->RMI_demand_heading);
   printf("\nRMI_ADF_needle = %.1f", IO_Sen_out->RMI_ADF_needle);
   printf("\nRMI_VOR_needle = %.1f", IO_Sen_out->RMI_VOR_needle);
}

/*----------------------------------------------------------------------*/

static void Debug_vhf(vhf_beac_attr *battr)
{
   printf("\nvalid_flag = %d", battr->valid_flag);
   printf("\nfreq = %d", battr->freq);

   printf("\nvalid_vor_ils = %d", battr->valid_vor_ils);
   printf("\ntype = %s", (battr->type == VOR_TYPE)? "VOR": "ILS");
   printf("\nglide_tx = %d", battr->valid_glide_tx);
   printf("\ndme = %d", battr->valid_dme);
   printf("\nemm_rate = %d", battr->ident_emmision_rate);
   printf("\nvoice = %d", battr->voice_ident);
   printf("\nident = %c%c%c", battr->ident[0], battr->ident[1],
	                          battr->ident[2]);

   printf("\nlat = %f", battr->beac_lat);
   printf("\nlon = %f", battr->beac_lon);
   printf("\nelev = %f", battr->beac_elev);

   printf("\nglat = %f", battr->glide_tx_lat);
   printf("\nglon = %f", battr->glide_tx_lon);
   printf("\ngang = %f", battr->glideslope_ang);
   printf("\ngelev = %f", battr->glide_elev);
   printf("\napp_head = %f", battr->approach_head);

   printf("\nmag_var = %f", battr->magnetic_var);

   printf("\ndme_lat = %f", battr->dme_lat);
   printf("\ndme_lon = %f", battr->dme_lon);
   printf("\ndme_elev = %f", battr->dme_elev);

   printf("\nmerit_ind = %d", battr->valid_vor_ind);
}

/*----------------------------------------------------------------*/

static void Debug_ndb(ndb_beac_attr *battr)
{
   printf("\nvalid_flag = %d", battr->valid_flag);
   printf("\nfreq = %d", battr->freq);

   printf("\nvoice = %d", battr->voice_ident);
   printf("\nmod_freq = %d", battr->mod_freq_flag);
   printf("\npower = %d", battr->power);
   printf("\nemm_rate = %d", battr->ident_emmision_rate);
   printf("\nident = %c%c%c", battr->ident[0],
	                          battr->ident[1],
                              battr->ident[2]);
   printf("\nlat = %f", battr->beac_lat);
   printf("\nlon = %f", battr->beac_lon);
   printf("\nelev = %f", battr->beac_elev);
   printf("\nmag_var = %f", battr->magnetic_var);
}

/*----------------------------------------------------------------*/

static void Hard_code_ils(vhf_beac_attr *battr)
{
   battr->freq = 10890;
   battr->valid_vor_ils = TRUE;
   battr->type = ILS_TYPE;
   battr->valid_glide_tx = TRUE;
   battr->valid_dme = FALSE;        //For now
   battr->ident_emmision_rate = 4;
   battr->voice_ident = FALSE;
   battr->ident[0] = 'I';
   battr->ident[1] = 'J';
   battr->ident[2] = 'R';
   battr->ident[3] = 0;

   battr->beac_lat = 36.0 + (43.0/60.0) + (57.04/3600.0);
   battr->beac_lon = -(6.0 + (3.0/60.0) + (57.67/3600.0));
   battr->beac_elev = (float)(85.0 / FT_IN_METRE);

   battr->glide_tx_lat = 36.0 + (45.0/60.0) + (3.53/3600.0);
   battr->glide_tx_lon = -(6.0 + (3.0/60.0) + (20.11/3600.0));
   battr->glideslope_ang = 3.0;
   battr->glide_elev = (float)(85.0 / FT_IN_METRE);
   battr->approach_head = 205.0;

   battr->magnetic_var = 4.0;

   battr->dme_lat = 36.0 + (43.0/60.0) + (57.04/3600.0);
   battr->dme_lon = -(6.0 + (3.0/60.0) + (57.67/3600.0));
   battr->dme_elev = (float)(85.0 / FT_IN_METRE);

   battr->valid_vor_ind = 0;

   // Say search was successful 
   battr->valid_flag = TRUE;

}

/*----------------------------------------------------------------*/
