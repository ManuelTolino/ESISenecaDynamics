/*
File: beac_io.c
Prefix: BIO_

Module: Dynamics for Seneca

Description: This file sets up and receives the beacon part of the
             comms between dyn and ios.
*/

/***************************/
/*   RCS Markers           */
/***************************/

/*
$Header: C:/master/seneca/NAV/RCS/beac_io.cpp 1.4 2000/05/30 14:16:47 colinj Exp $
$Log: beac_io.cpp $
Revision 1.4  2000/05/30 14:16:47  colinj
Make magnetic variation active.

Revision 1.3  2000/04/18 15:34:30  juliae
Added support for requesting beacon info constantly.

Revision 1.2  2000/04/17 11:38:49  juliae
Handle ignoring magnetic variation.

Revision 1.1  2000/02/21 20:21:22  juliae
Initial revision

*/

/*----------------------------------------------------------------------*/

/*-------------------------*/
/* Includes                */
/*-------------------------*/

#include <stdio.h>
#include <math.h>
#ifdef WIN32
#include <windows.h>
#endif
#include "c:\bae-sims\seneca\nav/beacon.h"
#include "c:\bae-sims\seneca\nav/dyn_ios.h"
#include "c:\bae-sims\seneca\dat_stru/out_head.h"
#include "c:\bae-sims\seneca\define.h"

/*-------------------------*/
/* Defines                 */
/*-------------------------*/

#define TRUE  1
#define FALSE 0

#define NDB_TOL_HZ  500    /* ie. 0.5 KHz */

//#define IGNORE_MAG_VAR 1

#define GET_BEAC_EVERY_TIME 1

/*----------------------------*/
/* External Routines          */
/*----------------------------*/

extern long N1A_get_active_n1_freq();
extern void N1A_refresh_beacon();
extern long N1A_get_dme_hold_freq();

/*----------------------------*/
/* Externals                  */
/*----------------------------*/

extern int    N2A_new_nav2_bfreq;
extern int    N1A_new_nav1_bfreq;
extern int    N1A_new_dme_bfreq;
extern int    ADF_new_ndb_bfreq;

extern sen_out_struct *IO_Sen_out;
extern vhf_beac_attr IO_nav2_battr;
extern vhf_beac_attr IO_nav1_battr;
extern vhf_beac_attr IO_dme_battr;
extern ndb_beac_attr IO_ndb_battr;

/*----------------------------*/
/* Locals                     */
/*----------------------------*/

/*----------------------------*/
/* Forward References         */
/*----------------------------*/

/*----------------------------------------------------------------*/

/* Sets up the beacon search request part of the coms to IOS

Inputs: N2A_new_nav2_bfreq
           N1A_new_nav2_bfreq
           N1A_new_dme_bfreq
           ADF_new_ndb_bfreq
           KX155_NAV_USE_display
           active nav1 freq
           dme hold freq
           KR87_USE_display
*/

void BIO_setup_beacon_2_ios(dyn_to_ios *dyn2ios)
{
   if (N2A_new_nav2_bfreq)
   {
      /* NB. We do NOT reset the N2A_new_nav2_bfreq here. It is
         reset ONLY when we know the IOS has received the
         frequency request */
      dyn2ios->flags |= NAV2_BIT;
      dyn2ios->nav2_freq = IO_Sen_out->KX155_NAV_USE_display;
   }

   if (N1A_new_nav1_bfreq)
   {
      /* NB. We do NOT reset the N1A_new_nav1_bfreq here. It is
         reset ONLY when we know the IOS has received the
         frequency request */
      dyn2ios->flags |= NAV1_BIT;
      dyn2ios->nav1_freq = N1A_get_active_n1_freq();
#ifndef GET_BEAC_EVERY_TIME
printf("\nSetting NAV1_BIT in Dyn_to_ios with freq = %d\n", dyn2ios->nav1_freq);
#endif
   }

   if (N1A_new_dme_bfreq)
   {
      /* NB. We do NOT reset the N1A_new_dme_bfreq here. It is
         reset ONLY when we know the IOS has received the
         frequency request */
      dyn2ios->flags |= DME_BIT;
      dyn2ios->dme_freq = N1A_get_dme_hold_freq();
#ifndef GET_BEAC_EVERY_TIME
printf("\nSetting DME_BIT in Dyn_to_ios with freq = %d\n", dyn2ios->dme_freq);
#endif
   }

   if (ADF_new_ndb_bfreq)
   {
      /* NB. We do NOT reset the ADF_new_ndb_bfreq here. It is
         reset ONLY when we know the IOS has received the
         frequency request */
      dyn2ios->flags |= NDB_BIT;
      dyn2ios->ndb_freq = IO_Sen_out->KR87_USE_display * 10;
      dyn2ios->ndb_tol = NDB_TOL_HZ;
#ifndef GET_BEAC_EVERY_TIME
printf("\nSetting NDB_BIT in Dyn_to_ios with freq = %d\n", dyn2ios->ndb_freq);
#endif
   }
}

/*----------------------------------------------------------------*/

/* This examines the data from the IOS and modifies any beacon
   data accordingly.
   
   Outputs: N2A_new_nav2_bfreq
            N1A_new_nav1_bfreq
            N1A_new_dme_bfreq
            ADF_new_ndb_bfreq
            IO_nav2_battr
            IO_nav1_battr
            IO_dme_battr
            IO_ndb_battr
            
*/

void BIO_read_beacon_from_ios(ios_to_dyn *ios2dyn)
{
   if (N2A_new_nav2_bfreq)
   {
      if (ios2dyn->flags & NAV2_DONE)
      {
         /* Update the global Nav2 beacon attributes used by all the beacon
            dials */
         memcpy(&IO_nav2_battr, &(ios2dyn->nav2_battr), sizeof(vhf_beac_attr));

#ifdef IGNORE_MAG_VAR
         //Convert any ILS magnetic approach heading to true approach heading
         IO_nav2_battr.approach_head -= IO_nav2_battr.magnetic_var;
         IO_nav2_battr.magnetic_var = 0.0;
#endif

         /* Check this beacon is the desired one by comparing frequency.
            NB. May need to allow for the tolerance band */
         if (IO_nav2_battr.freq == IO_Sen_out->KX155_NAV_USE_display)
         {
#ifndef GET_BEAC_EVERY_TIME
            N2A_new_nav2_bfreq = FALSE;
#endif
            ;
         }
         else
            IO_nav2_battr.valid_flag = FALSE;
#ifndef GET_BEAC_EVERY_TIME
printf("\nIO_nav2_battr.valid_flag = %d", IO_nav2_battr.valid_flag);
#endif
      }
   }

   if (N1A_new_nav1_bfreq)
   {
      if (ios2dyn->flags & NAV1_DONE)
      {
         /* Update the global Nav1 beacon attributes used by all the beacon
            dials */
         memcpy(&IO_nav1_battr, &(ios2dyn->nav1_battr), sizeof(vhf_beac_attr));

#ifdef IGNORE_MAG_VAR
         //Convert any ILS magnetic approach heading to true approach heading
         IO_nav1_battr.approach_head -= IO_nav1_battr.magnetic_var;
         IO_nav1_battr.magnetic_var = 0.0;
#endif

         /* Check this beacon is the desired one by comparing frequency.
            NB. May need to allow for the tolerance band */
         if (IO_nav1_battr.freq == N1A_get_active_n1_freq())
         {
#ifndef GET_BEAC_EVERY_TIME
            N1A_new_nav1_bfreq = FALSE;
#endif
            N1A_refresh_beacon();
         }
         else
            IO_nav1_battr.valid_flag = FALSE;
#ifndef GET_BEAC_EVERY_TIME
printf("\nIO_nav1_battr.valid_flag = %d", IO_nav1_battr.valid_flag);
#endif
      }
   }

   if (N1A_new_dme_bfreq)
   {
      if (ios2dyn->flags & DME_DONE)
      {
         /* Update the global Dme beacon attributes used in dme hold mode */
         memcpy(&IO_dme_battr, &(ios2dyn->dme_battr), sizeof(vhf_beac_attr));

#ifdef IGNORE_MAG_VAR
         //Convert any ILS magnetic approach heading to true approach heading
         IO_dme_battr.approach_head -= IO_dme_battr.magnetic_var;
         IO_dme_battr.magnetic_var = 0.0;
#endif

         /* Check this beacon is the desired one by comparing frequency.
            NB. May need to allow for the tolerance band */
         if (IO_dme_battr.freq == N1A_get_dme_hold_freq())
         {
#ifndef GET_BEAC_EVERY_TIME
            N1A_new_dme_bfreq = FALSE;
#endif
            ;
         }
         else
            IO_dme_battr.valid_flag = FALSE;
#ifndef GET_BEAC_EVERY_TIME
printf("\nIO_dme_battr.valid_flag = %d", IO_dme_battr.valid_flag);
#endif
      }
   }

   if (ADF_new_ndb_bfreq)
   {
      if (ios2dyn->flags & NDB_DONE)
      {
         /* Update the global Ndb beacon attributes used by all the beacon
            dials */
         memcpy(&IO_ndb_battr, &(ios2dyn->ndb_battr), sizeof(ndb_beac_attr));

#ifdef IGNORE_MAG_VAR
         IO_ndb_battr.magnetic_var = 0.0;
#endif
         /* Check this beacon is the desired one by comparing frequency.
            NB. May need to allow for the tolerance band */
         if (IO_ndb_battr.freq == (IO_Sen_out->KR87_USE_display * 10))
         {
#ifndef GET_BEAC_EVERY_TIME
            ADF_new_ndb_bfreq = FALSE;
#endif
            ;
         }
         else
            IO_ndb_battr.valid_flag = FALSE;
#ifndef GET_BEAC_EVERY_TIME
printf("\nIO_ndb_battr.valid_flag = %d", IO_ndb_battr.valid_flag);
#endif
      }
   }
}

/*----------------------------------------------------------------*/
