/*
File  : harness1.c
Prefix: HA_

Module: Seneca Dynamics
Created: 

Description: This file contains test harness code
*/

//**************************
//   RCS Markers           *
//**************************


//$Header: C:/master/seneca/RCS/harness1.cpp 1.1 2000/06/01 09:49:41 juliae Exp $
//$Log: harness1.cpp $
//Revision 1.1  2000/06/01 09:49:41  juliae
//Initial revision
//


//----------------------------------------------------------------------

//----------------
// Include Files
//----------------

#include <stdio.h>
#include <string.h>

#include "dat_stru\in_head.h"

//----------------
// Defines
//----------------

//----------------
// Typedefs
//----------------

//--------------------
// External Routines
//--------------------

//----------------
// Externals
//----------------

//----------------
// Globals
//----------------

sen_in_struct HA_sen_kbd_in;

//-----------------
// Local Variables
//-----------------

//--------------------
// Forward References
//--------------------

//----------------------------------------------------------------------

#ifdef HARNESS_INPUT
//****************************************************************************
//                TEST HARNESS CODE ONLY
//****************************************************************************

//Initialise just those elements of sen_in that we update with keyboard input

void HA_init_sen_kbd_in()
{
   sen_in_struct *kbd_sen_in;

   kbd_sen_in = &HA_sen_kbd_in;

   memset (kbd_sen_in,  0, sizeof(sen_in_struct));

   kbd_sen_in->KNS80_off_on_switch = SW_OFF;
   kbd_sen_in->KNS80_HOLD_button = NOT_PRESSED;
   kbd_sen_in->KNS80_USE_button = NOT_PRESSED;
   kbd_sen_in->KNS80_DATA_button = NOT_PRESSED;
   kbd_sen_in->KNS80_DSP_button = NOT_PRESSED;
   kbd_sen_in->KNS80_RNAV_button = NOT_PRESSED;
   kbd_sen_in->KNS80_VOR_button = NOT_PRESSED;

/*
   kbd_sen_in->pilot_KA51_slew_CW_switch = SW_OFF;
   kbd_sen_in->pilot_KA51_slew_CCW_switch = SW_OFF;
   kbd_sen_in->copilot_KA51_slew_CW_switch = SW_OFF;
   kbd_sen_in->copilot_KA51_slew_CCW_switch = SW_OFF;
*/

/*
   kbd_sen_in->KR87_SET_RST_button = NOT_PRESSED;
   kbd_sen_in->KR87_ADF_ANT_switch = KR87_ADF;
   kbd_sen_in->KR87_FRQ_button     = NOT_PRESSED;
   kbd_sen_in->KR87_FLT_ET_button  = NOT_PRESSED;
   kbd_sen_in->KR87_off_on_switch = SW_OFF;
*/
   kbd_sen_in->KX155_off_on_switch = SW_OFF;

   kbd_sen_in->KY196_off_on_switch = SW_OFF;
   kbd_sen_in->KY196_off_on_pull_switch = SW_OFF;
   kbd_sen_in->KY196_FRQ_button = NOT_PRESSED;

/*
   kbd_sen_in->comm_1_PTT = SW_OFF;
   kbd_sen_in->comm_2_PTT = SW_OFF;
*/

   kbd_sen_in->KT76_IDENT_button = NOT_PRESSED;
   kbd_sen_in->KT76_off_on = KT76_OFF_STBY;
}

//----------------------------------------------------------------------

/* Overwrites IO_Sen_in values for KNS80, KX155, KY196 and KT76 */

void HA_update_from_sen_kbd()
{
   sen_in_struct *kbd_sen_in;

   kbd_sen_in = &HA_sen_kbd_in;


/*
   IO_Sen_in->KNS80_off_on_switch = kbd_sen_in->KNS80_off_on_switch;
   IO_Sen_in->KNS80_HOLD_button   = kbd_sen_in->KNS80_HOLD_button;
   IO_Sen_in->KNS80_USE_button    = kbd_sen_in->KNS80_USE_button;
   IO_Sen_in->KNS80_DATA_button   = kbd_sen_in->KNS80_DATA_button;
   IO_Sen_in->KNS80_DSP_button    = kbd_sen_in->KNS80_DSP_button;
   IO_Sen_in->KNS80_RNAV_button   = kbd_sen_in->KNS80_RNAV_button;
   IO_Sen_in->KNS80_VOR_button    = kbd_sen_in->KNS80_VOR_button;
   IO_Sen_in->KNS80_delta_outer_encoder = kbd_sen_in->KNS80_delta_outer_encoder;
   IO_Sen_in->KNS80_delta_inner_encoder = kbd_sen_in->KNS80_delta_inner_encoder;
   IO_Sen_in->KNS80_delta_inner_encoder_pull = kbd_sen_in->KNS80_delta_inner_encoder_pull;
*/

/*
   IO_Sen_in->KX155_off_on_switch = kbd_sen_in->KX155_off_on_switch;
   IO_Sen_in->KX155_NAV_FRQ_button = kbd_sen_in->KX155_NAV_FRQ_button;
   IO_Sen_in->KX155_COMM_FRQ_button = kbd_sen_in->KX155_COMM_FRQ_button;
   IO_Sen_in->KX155_NAV_delta_outer_encoder = kbd_sen_in->KX155_NAV_delta_outer_encoder;
   IO_Sen_in->KX155_COMM_delta_outer_encoder = kbd_sen_in->KX155_COMM_delta_outer_encoder;
   IO_Sen_in->KX155_NAV_delta_inner_encoder = kbd_sen_in->KX155_NAV_delta_inner_encoder;
   IO_Sen_in->KX155_COMM_delta_inner_encoder = kbd_sen_in->KX155_COMM_delta_inner_encoder;
   IO_Sen_in->KX155_NAV_delta_inner_encoder_pull = kbd_sen_in->KX155_NAV_delta_inner_encoder_pull;
   IO_Sen_in->KX155_COMM_delta_inner_encoder_pull = kbd_sen_in->KX155_COMM_delta_inner_encoder_pull;
*/

/*
   IO_Sen_in->KY196_off_on_switch = kbd_sen_in->KY196_off_on_switch;
   IO_Sen_in->KY196_off_on_pull_switch = kbd_sen_in->KY196_off_on_pull_switch;
   IO_Sen_in->KY196_FRQ_button = kbd_sen_in->KY196_FRQ_button;
   IO_Sen_in->KY196_delta_outer_encoder = kbd_sen_in->KY196_delta_outer_encoder;
   IO_Sen_in->KY196_delta_inner_encoder = kbd_sen_in->KY196_delta_inner_encoder;
   IO_Sen_in->KY196_delta_inner_encoder_pull = kbd_sen_in->KY196_delta_inner_encoder_pull;
*/

/*
   IO_Sen_in->KT76_IDENT_button = kbd_sen_in->KT76_IDENT_button;
   IO_Sen_in->KT76_off_on = kbd_sen_in->KT76_off_on;
*/
}

//----------------------------------------------------------------------
#endif    //Of HARNESS_INPUT
