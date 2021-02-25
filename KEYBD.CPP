/*
File  : keybd.c
Prefix: KB_

Module: Dynamics for Seneca
Created: 

Description: This file handles keyboard input
*/

//**************************
//   RCS Markers           *
//**************************


//$Header: C:/master/seneca/RCS/keybd.cpp 1.1 2000/06/01 09:46:35 juliae Exp $
//$Log: keybd.cpp $
//Revision 1.1  2000/06/01 09:46:35  juliae
//Initial revision
//


//----------------------------------------------------------------------

//----------------
// Include Files
//----------------

#include <stdio.h>
#include <string.h>
#include <conio.h>

#include "define.h"
#include "nav\dyn_ios.h"
#include "faults.h"
#include "dat_stru\in_head.h"
#include "dat_stru\out_head.h"
#include "fltdy\fmgdefs.h"

//----------------
// Defines
//----------------

#define ESC          27   /* 0x1B */
#define RET          13   /* 0x0D */

#define GENERAL_PAGE 0
#define NAV1_PAGE    1
#define NAV2_PAGE    2
#define COMPASS_PAGE 3
#define ADF_PAGE     4
#define COM2_PAGE    5    /* Part of NAV2 */
#define COM1_PAGE    6
#define RADAR_PAGE   7
#define ORIGINAL_CASES

//----------------
// Typedefs
//----------------

//--------------------
// External Routines
//--------------------

extern void NDG_debug_adf_box();
extern void NDG_debug_nav1_box();
extern void NDG_debug_nav2_box();
extern void NDG_debug_nav2_dial();
extern void NDG_debug_com1_box();
extern void NDG_debug_pilot_hsi();

//----------------
// Externals
//----------------

extern char SE_exit_flag;
extern char PETE_HEAD;

extern sen_fault_struct IO_Sen_faults;
extern ios_to_dyn       *ET_ios_data_p;
extern OUTPUTS          outputs;
extern INPUTS           inputs;
extern STATES           states;
extern unsigned int     ios_secs;
extern sen_in_struct    HA_sen_kbd_in;

//----------------
// Globals
//----------------

int KB_turn_dcl_on = FALSE;
int KB_pitchforce_mod = 30;
int KB_rollforce_mod = 0;
int KB_nav1 = FALSE, KB_nav2 = FALSE;

//-----------------
// Local Variables
//-----------------

static int Page_type_mode  = FALSE;
static int Page_number     = GENERAL_PAGE;
static int Num_str_index   = 0;
static char Num_str[10];

//--------------------
// Forward References
//--------------------

static void General_input(char ch);
static void Avionics_input(char ch);

//----------------------------------------------------------------------

char KB_decode_kb()
{
   char  ch;

	ch = (char) getch();
	switch ( ch )
	{
      case ESC:
         SE_exit_flag = TRUE;
         break;

      default:
#ifdef ALLOW_KB_INPUT
#ifdef ORIGINAL_CASES
         General_input(ch);
#else
         Avionics_input(ch);
#endif
#endif
         break;

   }
   return (1);
}

//****************************************************************************

static void General_input(char key)
{
   switch (key)
   {
		case ' ' : 
         break;

		case 'e' : //  ET_ios_data_p->FlightFailures   &= ~ FM_L_CHT_HIGH; 
         break;
      case 'h' : PETE_HEAD = !PETE_HEAD;
         break;
		case 'E' :   //ET_ios_data_p->FlightFailures   |=  FM_L_CHT_HIGH; 
         break;

		case 'd' :   KB_turn_dcl_on = FALSE;
         break;
		case 'D' :   //ET_ios_data_p->FlightFailures   |=  CB_ELEC_FLAP; 
                   KB_turn_dcl_on = TRUE;
         break;

		case 'c' :   ET_ios_data_p->EngineInstruments &= ~LEFT_ENGI_FUEL_FLOW_IND_STUCK; 
         break;
		case 'C' :   ET_ios_data_p->EngineInstruments |= LEFT_ENGI_FUEL_FLOW_IND_STUCK; 
							 break;

      case 's' :
         break;

      case 'q' : IO_Sen_faults.FI.P1_HORZ_POS_IND = TRUE;
         break;

      case 'Q' : IO_Sen_faults.FI.P1_HORZ_POS_IND = FALSE;
         break;

      case 'r' :
         break;


      case '1' :   //SE_back_track = 0;//reset_dynamics();
                   //IO_Sen_faults.FF.l_prop_deice = !IO_Sen_faults.FF.l_prop_deice;
			          KB_pitchforce_mod = 80;
         break;

      case '2' :   KB_pitchforce_mod = 85;
         break;

      case '3' :   KB_pitchforce_mod = 90;
         break;

      case '4' :   KB_pitchforce_mod = 95;
         break;

      case '5' :   KB_pitchforce_mod = 100;
         break;

      case '6' :   KB_pitchforce_mod = 55;
         break;

      case '7' :  KB_pitchforce_mod = 60;
         break;

      case '8' :  KB_pitchforce_mod = 65;
         break;

      case '9' :  KB_pitchforce_mod = 70;
         break;

      case '0' :   KB_pitchforce_mod = 75;
         break;

      case 'i' :   ios_secs = 580;
         break;

      case 'o' :   ios_secs = 590;
         break;

      case 'p' :   ios_secs =  600;
         break;

      case 'z' :  KB_nav1 = !KB_nav1;
         break;

      case 'x' :  KB_nav2 = !KB_nav2;
         break;


      case 'v' :  
         break;

      case 'b' :
         memset(&inputs,0,sizeof(INPUTS));
         memset(&states,0,sizeof(STATES));
         memset(&outputs,0,sizeof(OUTPUTS));
         break;

      case 'n' :  //IO_Sen_faults.pitch_trim_mcb = !IO_Sen_faults.pitch_trim_mcb;
                  
         break;

      case 'm' :  
         break;

      default:
         break;
   }
}

//****************************************************************************

//Handles keyboard input when it is being used to control
//avionics

static void Avionics_input(char key)
{
   sen_in_struct *sen_in;
   int           value;

   sen_in = &HA_sen_kbd_in;

   switch (key)
   {
      case RET:
         Num_str[Num_str_index] = '\0';
         value = atoi(Num_str);
         if (Page_type_mode)
         {
            Page_number = value;
            Page_type_mode = FALSE;
            if (Page_number == GENERAL_PAGE)
               printf("\nPage is GENERAL_PAGE");
            else if (Page_number == NAV1_PAGE)
               printf("\nPage is NAV1_PAGE");
            else if (Page_number == NAV2_PAGE)
               printf("\nPage is NAV2_PAGE");
            else if (Page_number == COM2_PAGE)
               printf("\nPage is COM2_PAGE");
            else if (Page_number == COMPASS_PAGE)
               printf("\nPage is COMPASS_PAGE");
            else if (Page_number == ADF_PAGE)
               printf("\nPage is ADF_PAGE");
            else if (Page_number == COM1_PAGE)
               printf("\nPage is COM1_PAGE");
            else if (Page_number == RADAR_PAGE)
               printf("\nPage is RADAR_PAGE");
            else
               Page_number = GENERAL_PAGE;
         }
         Num_str_index = 0;
         break;

      case '0':
      case '1':
      case '2':
      case '3':
      case '4':
      case '5':
      case '6':
      case '7':
      case '8':
      case '9':
         {
            if (Num_str_index < 9)
               Num_str[Num_str_index++] = key;
         }
         break;

      case 'a':
         if (Page_number == ADF_PAGE)
            sen_in->KR87_ADF_ANT_switch = PRESSED;
         break;

      case 'A':
         if (Page_number == ADF_PAGE)
            sen_in->KR87_ADF_ANT_switch = NOT_PRESSED;
         break;

      case 'b':
         if (Page_number == ADF_PAGE)
            sen_in->KR87_BFO_switch = PRESSED;
         break;

      case 'B':
         if (Page_number == ADF_PAGE)
            sen_in->KR87_BFO_switch = NOT_PRESSED;
         break;

      case 'd':   /* Pilot clockwise slave meter change */
         if (Page_number == COMPASS_PAGE)
            sen_in->pilot_KA51_slew_CW_switch = PRESSED;
         break;

      case 'D':   /* Stop Pilot clockwise slave meter */
         if (Page_number == COMPASS_PAGE)
            sen_in->pilot_KA51_slew_CW_switch = NOT_PRESSED;
         break;

      case 'e':   /* Pilot c_clockwise slave meter change */
         if (Page_number == COMPASS_PAGE)
            sen_in->pilot_KA51_slew_CCW_switch = PRESSED;
         else if (Page_number == ADF_PAGE)
            sen_in->KR87_FLT_ET_button = PRESSED;
         break;

      case 'E':   /* Stop Pilot c_clockwise slave meter */
         if (Page_number == COMPASS_PAGE)
            sen_in->pilot_KA51_slew_CCW_switch = NOT_PRESSED;
         else if (Page_number == ADF_PAGE)
            sen_in->KR87_FLT_ET_button = NOT_PRESSED;
         break;

      case 'f':
         if (Page_number == ADF_PAGE)
         {
            sen_in->KR87_FRQ_button = PRESSED;
         }
         break;

      case 'F':
         if (Page_number == ADF_PAGE)
         {
            sen_in->KR87_FRQ_button = NOT_PRESSED;
         }
         break;

      case 'g':   /* Press gyro slave button */
         if (Page_number == COMPASS_PAGE)
            sen_in->pilot_KA51_mode_switch = PRESSED;
         break;

      case 'G':   /* Press out gyro slave button */
         if (Page_number == COMPASS_PAGE)
            sen_in->pilot_KA51_mode_switch = NOT_PRESSED;
         break;

      case 'h':
         if (Page_number == NAV1_PAGE)
            sen_in->KNS80_HOLD_button = PRESSED;
         break;

      case 'H':
         if (Page_number == NAV1_PAGE)
            sen_in->KNS80_HOLD_button = NOT_PRESSED;
         break;

      case 'k':
         if (Page_number == COM1_PAGE)
            sen_in->comm_1_PTT = SW_ON;
         else if (Page_number == COM2_PAGE)
            sen_in->comm_2_PTT = SW_ON;
         break;

      case 'K':
         if (Page_number == COM1_PAGE)
            sen_in->comm_1_PTT = SW_OFF;
         else if (Page_number == COM2_PAGE)
            sen_in->comm_2_PTT = SW_OFF;
         break;

      case 'n':   /* Press RNAV mode button */
         if (Page_number == NAV1_PAGE)
            sen_in->KNS80_RNAV_button = PRESSED;
         break;

      case 'N':   /* Release RNAV mode button */
         if (Page_number == NAV1_PAGE)
            sen_in->KNS80_RNAV_button = NOT_PRESSED;
         break;

      case 'o':
         if (Page_number == NAV1_PAGE)
            sen_in->KNS80_off_on_switch = SW_ON;
         else if ((Page_number == NAV2_PAGE) || (Page_number == COM2_PAGE))
            sen_in->KX155_off_on_switch = SW_ON;
         else if (Page_number == ADF_PAGE)
            sen_in->KR87_off_on_switch = SW_ON;
         else if (Page_number == COM1_PAGE)
            sen_in->KY196_off_on_switch = SW_ON;
         else if (Page_number == RADAR_PAGE)
         {
            if (sen_in->KT76_off_on == KT76_OFF_STBY)
               sen_in->KT76_off_on = KT76_ON_ALT;
            else
               sen_in->KT76_off_on = KT76_TEST;
         }
         break;

      case 'O':
         if (Page_number == NAV1_PAGE)
            sen_in->KNS80_off_on_switch = SW_OFF;
         else if ((Page_number == NAV2_PAGE) || (Page_number == COM2_PAGE))
            sen_in->KX155_off_on_switch = SW_OFF;
         else if (Page_number == ADF_PAGE)
            sen_in->KR87_off_on_switch = SW_OFF;
         else if (Page_number == COM1_PAGE)
            sen_in->KY196_off_on_switch = SW_OFF;
         else if (Page_number == RADAR_PAGE)
            sen_in->KT76_off_on = KT76_OFF_STBY;
         break;

      case 's':
         if (Page_number == NAV1_PAGE)
            sen_in->KNS80_DSP_button = PRESSED;
         else if (Page_number == ADF_PAGE)
            sen_in->KR87_SET_RST_button = PRESSED;
         break;

      case 'S':
         if (Page_number == NAV1_PAGE)
            sen_in->KNS80_DSP_button = NOT_PRESSED;
         else if (Page_number == ADF_PAGE)
            sen_in->KR87_SET_RST_button = NOT_PRESSED;
         break;

      case 't':
         if (Page_number == NAV1_PAGE)
            sen_in->KNS80_DATA_button = PRESSED;
         break;

      case 'T':
         if (Page_number == NAV1_PAGE)
            sen_in->KNS80_DATA_button = NOT_PRESSED;
         break;

      case 'u':
         if (Page_number == NAV1_PAGE)
            sen_in->KNS80_USE_button = PRESSED;
         else if (Page_number == NAV2_PAGE)
            sen_in->KX155_NAV_FRQ_button = PRESSED;
         else if (Page_number == COM2_PAGE)
            sen_in->KX155_COMM_FRQ_button = PRESSED;
         else if (Page_number == COM1_PAGE)
            sen_in->KY196_FRQ_button = PRESSED;
         break;

      case 'U':
         if (Page_number == NAV1_PAGE)
            sen_in->KNS80_USE_button = NOT_PRESSED;
         else if (Page_number == NAV2_PAGE)
            sen_in->KX155_NAV_FRQ_button = NOT_PRESSED;
         else if (Page_number == COM2_PAGE)
            sen_in->KX155_COMM_FRQ_button = NOT_PRESSED;
         else if (Page_number == COM1_PAGE)
            sen_in->KY196_FRQ_button = NOT_PRESSED;
         break;

      case 'v':   /* Press NAV1 VOR mode button */
         if (Page_number == NAV1_PAGE)
            sen_in->KNS80_VOR_button = PRESSED;
         break;

      case 'V':   /* Release NAV1 VOR mode button */
         if (Page_number == NAV1_PAGE)
            sen_in->KNS80_VOR_button = NOT_PRESSED;
         break;

      case 'w':   /* Set type of data keyboard input changes */
printf("\nPage type");
         Page_type_mode = TRUE;
         Num_str_index = 0;
         break;

      case 'x':   /* Increase outer dial counter */
         if (Page_number == NAV1_PAGE)
            sen_in->KNS80_delta_outer_encoder += 1;
         else if (Page_number == ADF_PAGE)
            sen_in->KR87_delta_outer_encoder += 1;
         else if (Page_number == NAV2_PAGE)
            sen_in->KX155_NAV_delta_outer_encoder += 1;
         else if (Page_number == COM2_PAGE)
            sen_in->KX155_COMM_delta_outer_encoder += 1;
         else if (Page_number == COM1_PAGE)
            sen_in->KY196_delta_outer_encoder += 1;
         break;

      case 'X':   /* Decrease outer dial counter */
         if (Page_number == NAV1_PAGE)
            sen_in->KNS80_delta_outer_encoder -= 1;
         else if (Page_number == ADF_PAGE)
            sen_in->KR87_delta_outer_encoder -= 1;
         else if (Page_number == NAV2_PAGE)
            sen_in->KX155_NAV_delta_outer_encoder -= 1;
         else if (Page_number == COM2_PAGE)
            sen_in->KX155_COMM_delta_outer_encoder -= 1;
         else if (Page_number == COM1_PAGE)
            sen_in->KY196_delta_outer_encoder -= 1;
         break;

      case 'y':   /* Increase inner dial counter */
         if (Page_number == NAV1_PAGE)
            sen_in->KNS80_delta_inner_encoder += 1;
         else if (Page_number == ADF_PAGE)
            sen_in->KR87_delta_inner_encoder += 1;
         else if (Page_number == NAV2_PAGE)
            sen_in->KX155_NAV_delta_inner_encoder += 1;
         else if (Page_number == COM2_PAGE)
            sen_in->KX155_COMM_delta_inner_encoder += 1;
         else if (Page_number == COM1_PAGE)
            sen_in->KY196_delta_inner_encoder += 1;
         break;

      case 'Y':   /* Decrease inner dial counter */
         if (Page_number == NAV1_PAGE)
            sen_in->KNS80_delta_inner_encoder -= 1;
         else if (Page_number == ADF_PAGE)
            sen_in->KR87_delta_inner_encoder -= 1;
         else if (Page_number == NAV2_PAGE)
            sen_in->KX155_NAV_delta_inner_encoder -= 1;
         else if (Page_number == COM2_PAGE)
            sen_in->KX155_COMM_delta_inner_encoder -= 1;
         else if (Page_number == COM1_PAGE)
            sen_in->KY196_delta_inner_encoder -= 1;
         break;

      case 'z':   /* Increase inner dial counter */
         if (Page_number == NAV1_PAGE)
            sen_in->KNS80_delta_inner_encoder_pull += 1;
         else if (Page_number == ADF_PAGE)
            sen_in->KR87_delta_inner_encoder_pull += 1;
         else if (Page_number == NAV2_PAGE)
            sen_in->KX155_NAV_delta_inner_encoder_pull += 1;
         else if (Page_number == COM2_PAGE)
            sen_in->KX155_COMM_delta_inner_encoder_pull += 1;
         else if (Page_number == COM1_PAGE)
            sen_in->KY196_delta_inner_encoder_pull += 1;
         break;

      case 'Z':   /* Decrease inner dial counter */
         if (Page_number == NAV1_PAGE)
            sen_in->KNS80_delta_inner_encoder_pull -= 1;
         else if (Page_number == ADF_PAGE)
            sen_in->KR87_delta_inner_encoder_pull -= 1;
         else if (Page_number == NAV2_PAGE)
            sen_in->KX155_NAV_delta_inner_encoder_pull -= 1;
         else if (Page_number == COM2_PAGE)
            sen_in->KX155_COMM_delta_inner_encoder_pull -= 1;
         else if (Page_number == COM1_PAGE)
            sen_in->KY196_delta_inner_encoder_pull -= 1;
         break;

      default:
         break;
   }
}

/*----------------------------------------------------------------------*/

void KB_debug_avionics()
{
   static int count = 0;

   count ++;
   if (count < 100)
      return;
   count = 0;

   if (Page_number == ADF_PAGE)
   {
      NDG_debug_adf_box();
//      NDG_debug_ndb_dial();
   }
   else if (Page_number == NAV1_PAGE)
   {
      NDG_debug_nav1_box();
      NDG_debug_pilot_hsi();
//      NDG_debug_copilot_hsi();
   }
   else if ((Page_number == NAV2_PAGE) || (Page_number == COM2_PAGE))
   {
      NDG_debug_nav2_box();
      NDG_debug_nav2_dial();
   }
   else if (Page_number == COM1_PAGE)
   {
      NDG_debug_com1_box();
   }
}

/*----------------------------------------------------------------------*/
