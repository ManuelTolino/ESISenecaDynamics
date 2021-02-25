/*
File: b_dials.h

Module: Navigation (Dynamics) for Seneca

Description: This file contains structures for ILS and VOR
             instruments.
---------------------*/

/***************************/
/*   RCS Markers           */
/***************************/

/*
$Header: C:/SENECA/NAV/RCS/b_dials.h 1.1 2000/02/22 02:25:52 juliae Exp $
$Log: b_dials.h $
Revision 1.1  2000/02/22 02:25:52  juliae
Initial revision

*/

/*----------------------------------------------------------------------*/

/*----------------*/
/* Typedefs       */
/*----------------*/

/* ILS */

typedef struct {

   char  valid_loc;      /* False, if no active beacon */
   char  valid_glide;    /* False, if no glideslope info */
   char  spare[2];
   float lt_rt_dev;      /* For VOR this is radial deviation bar */
   float vert_dev;       /* For glideslope deviation */

} ils_inst_out_data;

/*----------------------------------------------------------------*/

/* VOR */

#define NO_FLAG   0
#define TO_FLAG   1
#define FROM_FLAG 2

typedef struct {

   char  valid;      /* False, if no active beacon */
   char  to_flag;    /* See defines */
   char  spare[2];
   float radial_dev; /* For L/R deviation bar */

} vor_inst_out_data;

/*----------------------------------------------------------------*/

/* HSI. This mimicks the entries in sen_out_struct.
   This is purely for convenience so you can use the same
   routine for both pilot and copilot and then copy to correct
   variable names afterwards */

typedef struct {

   unsigned char GS_enable;
   unsigned char nav_flag;
   unsigned char hdg_flag;
   unsigned char spare;
   float         compass_demand_position;
   float         GS_needle;
   float         left_right_needle;
   float         to_from_needle;

} hsi_out_data;

/*----------------------------------------------------------------*/

typedef struct {

   float demand_head;
   float ADF_needle;
   float VOR_needle;

} rmi_out_data;

/*----------------------------------------------------------------*/
