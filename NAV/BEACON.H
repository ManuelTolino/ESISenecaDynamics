/*
File  : beacon.h

Module: Dynamics and IOS for Seneca

Description: This file contains the structures for beacon data.
*/

/*
Updates
Draft B: Added idents
         Added magnetic variation to ndb
         Added voice_ident flag to ndb
         Added mod_freq_flag to ndb
         Added power and ident_emmision_rate to ndb
         Added valid_vor_ils flag (since freq may be DME alone)

Draft C: Added beac_elev
         Renamed dme_alt to dme_elev

Draft D: Corrected byte alignment
*/

/***************************/
/*   RCS Markers           */
/***************************/

/*
$Header: C:/master/seneca/NAV/RCS/beacon.h 1.3 2000/05/09 08:58:08 juliae Exp $
$Log: beacon.h $
Revision 1.3  2000/05/09 08:58:08  juliae
Added glide_offset  (replaces a spare).

Revision 1.2  2000/05/09 07:34:18  juliae
Added dme_offset.

Revision 1.1  2000/04/17 11:24:03  colinj
Initial revision (Draft D)

 * Revision 0.2  2000/03/10  10:19:46  juliae
 * Draft C
 *
 * Revision 0.1  2000/02/22  04:01:33  juliae
 * DRAFT B.
 *
*/

/*----------------------------------------------------------------------*/

#ifndef BEACON_H
#define BEACON_H

/*----------------------------------------------------------------------*/

/* General purpose beacon attributes for mHz frequencies (VOR, ILS, DME).
   Note that the freq may correspond to either a VOR or an ILS, with DME
   aswell, or it may just correspond to a DME. The "valid_flag" is used
   to indicate whether ANY information has been found for the frequency.
   The other separate "valid" flags must be checked before using any
   specific information. */

#define VOR_TYPE 0
#define ILS_TYPE 1

typedef struct {

   long freq;                 /* In 1/100 MHz */
   char valid_flag;           /* TRUE if beacon matching requested freq has been found */
   char valid_vor_ils;        /* TRUE if freq matches a VOR or ILS beacon */
   char type;                 /* 0 = VOR, 1 = ILS */
   char valid_glide_tx;       /* TRUE if there is a glideslope transmitter (ILS only?) */
   char valid_dme;            /* TRUE if there is DME associated with this freq */
   char ident_emmision_rate;  /* Number of signals per minute */
   char voice_ident;          /* TRUE if a voice signal is given */
   char ils_back_signal;
   char ident[4];             /* Morse code. Blanks should be set to \0 */

   double beac_lat;           /* North is +ve, South is -ve */
   double beac_lon;           /* East is +ve, West is -ve */
   double glide_tx_lat;       /* Only used if valid_glide_tx is set */
   double glide_tx_lon;       /* "  " */
   float  beac_elev;          /* Height in metres above msl */
   float  glideslope_ang;     /* "  " */
   float  glide_elev;         /* Height in metres above msl */
   float  approach_head;      /* Only valid for ILS type */
   float  magnetic_var;       /* West is +ve */
   float  glide_offset;       /* In metres, perpendicular offset from runway centre-line */

   double dme_lat;            /* Only used if valid_dme is set */
   double dme_lon;            /* "  " */
   float  dme_elev;           /* Height in metres above msl */
   char   dme_ident[4];       /* Morse code. Blanks should be set to \0 */
   char   valid_vor_ind;      /* 0 to 3 Only valid for VOR type */
   char   dme_offset;         /* In tenths of a nm, ie. 12 = 1.2 nm */
   char   spare2[2];
   
} vhf_beac_attr;

/* General purpose beacon attributes for kHz frequencies (NDB) */

typedef struct {

   long freq;                 /* In 1/10 KHz */
   char valid_flag;           /* TRUE if beacon matching requested freq has been found */
   char voice_ident;          /* TRUE if a voice signal is given */
   char mod_freq_flag;        /* TRUE if in audible range, ie. TRUE for A2A, FALSE for A1A */
   char power;                /* 0: < 25, 1: (25,50), 2: (50,2000), 3: > 2000 watts */
   char ident_emmision_rate;  /* Number of signals per minute */
   char spare[3];
   char ident[4];             /* Morse code. Blanks should be set to \0 */
   /* Range indication?? */

   double beac_lat;
   double beac_lon;
   float  beac_elev;          /* Height in metres above msl */
   float  magnetic_var;       /* West is +ve */

} ndb_beac_attr;

/*----------------------------------------------------------------------*/
#endif
