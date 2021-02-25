/*
IS: ice_sys.cpp
*/

/*--------------------- 
This file seneca ice protection systems
---------------------*/

/***************************/
/*   RCS Markers           */
/***************************/

/*
$Header: C:/Master/seneca/RCS/ice_sys.cpp 1.4 2000/06/16 07:59:35 ColinJ Exp $
$Log: ice_sys.cpp $
Revision 1.4  2000/06/16 07:59:35  ColinJ
Changed prop deice system to use two bladed model.
Added elec power test to wing deice system.

Revision 1.3  2000/05/25 14:50:28  colinj
low bus lamp no longer flashes and also depends on
MCB_alt_field_right/left_tripped.

Revision 1.2  2000/05/11 13:35:12  colinj
Added logic for de-ice test.

Revision 1.1  2000/04/14 16:16:51  colinj
Initial revision

*/

//----------------------------------------------------------------------

//----------------
// Include Files
//----------------
#include <stdio.h>

#include "dat_stru\in_head.h"
#include "dat_stru\out_head.h"
#include "define.h"
#include "elec\electric.h"
#include "fltdy\fmgdefs.h"
#include "fltdy\fms_defs.h"
#include "faults.h"
//----------------
// Defines
//----------------
//#define THREE_BLADE_DEICE //uncomment to use three blade prop deice system
#define TWO_BLADE_DEICE 1    //uncomment to use two blade prop deice system
#define DE_ICE_TIME        8.3f //time wing de-ice system on for
#define DEICE1             1.0f
#define DEICE2             8.0f
#define BUS_FLASH          0.5f

#ifdef THREE_BLADE_DEICE
//#define LEFT_DEICE_LOAD   20.0f
//#define RIGHT_DEICE_LOAD  23.0f
//#define LPROP_DEICE_TIME  90.0f// 5.0//
//#define RPROP_DEICE_TIME 180.0f//10.0//

#define RIGHT_OUTER 1
#define RIGHT_INNER 2
#define LEFT_OUTER  3
#define LEFT_INNER  4

#define RIGHT_OUTER_DEICE_LOAD  20.0f
#define RIGHT_INNER_DEICE_LOAD  20.0f
#define LEFT_OUTER_DEICE_LOAD   23.0f
#define LEFT_INNER_DEICE_LOAD   23.0f
#define FAILED_PROP_LOAD        15.0f

#define RPROP_OUTER_DEICE_TIME  90.0f //time right outer blade pad finishes
#define RPROP_DEICE_TIME        90.0
#define LPROP_DEICE_TIME       180.0 
#define LPROP_OUTER_DEICE_TIME 180.0 //time right outer blade pad finishes

#endif

#ifdef TWO_BLADE_DEICE
#define RIGHT_OUTER 1
#define RIGHT_INNER 2
#define LEFT_OUTER  3
#define LEFT_INNER  4

#define RIGHT_OUTER_DEICE_LOAD  20.0f
#define RIGHT_INNER_DEICE_LOAD  22.0f
#define LEFT_OUTER_DEICE_LOAD   20.0f
#define LEFT_INNER_DEICE_LOAD   23.0f
#define FAILED_PROP_LOAD        15.0f

#define RPROP_OUTER_DEICE_TIME  20.0 //time right outer blade pad finishes
#define RPROP_DEICE_TIME        40.0
#define LPROP_DEICE_TIME        80.0 
#define LPROP_OUTER_DEICE_TIME  60.0 //time right outer blade pad finishes
#endif
//#define DEBUG_ICE
//----------------
// Typedefs
//----------------
typedef struct
{
   unsigned char left_overboost_lamp;
   unsigned char right_overboost_lamp;
   unsigned char oil_press_lamp;
   unsigned char left_gyro_air_lamp;
   unsigned char right_gyro_air_lamp;
   unsigned char baggage_door_lamp;
   unsigned char alt_out_lamp;
   unsigned char gear_unsafe_lamp;
   unsigned char heater_overheat_lamp;
   unsigned char cabin_door_lamp;
   unsigned char left_auxiliary_fuel_lamp;
   unsigned char right_auxiliary_fuel_lamp;
}store_cwp;

//--------------------
// External Routines
//--------------------

//----------------
// Externals
//----------------

extern sen_out_struct   *IO_Sen_out;
extern sen_in_struct    *IO_Sen_in;
extern OUTPUTin			*OUTPUTinbox;//simulinkudp
extern EL_in            EL_input;
extern OUTPUTS          outputs;
extern INPUTS           inputs;
extern EL_out           EL_output;
extern sen_fault_struct IO_Sen_faults;

extern float delta_time;

//----------------
// Globals
//----------------
store_cwp cwp_lamp;

float dip_vac = 0.0;

int low_bus = FALSE;

char last_cwp_test = SW_OFF;
char IC_ice_load   = FALSE;
char wing_deice_flag = FALSE;
//-----------------
// Local Variables
//-----------------
static float wing_deice_ct    = 0.0;
static float bus_low_flash    = 0.0;
static float left_gen_spd     = 0.0;
static float right_gen_spd    = 0.0;
static float prop_deice_timer = 0.0;

static char left_alt_low    = FALSE , right_alt_low = FALSE;
static char left_off        = FALSE , right_off     = FALSE;
static char last_deiceSide = LEFT_INNER;
static char last_deiceSwitch = SW_OFF;
//--------------------
// Forward References
//--------------------

//----------------------------------------------------------------------
void IS_deice(void)
{
   IC_ice_load = FALSE;
// "SHORT DEICE TIMER IF SWITCH IS SWITCH ON THEN OFF
   if(EL_output.power == TRUE && IO_Sen_in->panel_propellor_deice_switch == SW_ON && 
      last_deiceSwitch == SW_OFF)
   {
      if(last_deiceSide == RIGHT_OUTER)
      {
         prop_deice_timer = RPROP_OUTER_DEICE_TIME;
      }
      else
      if(last_deiceSide == RIGHT_INNER)
      {
         prop_deice_timer = RPROP_DEICE_TIME;
      }
      else
      if(last_deiceSide == LEFT_OUTER)
      {
         prop_deice_timer = LPROP_OUTER_DEICE_TIME;
      }
      else
         prop_deice_timer = 0.0;
   }

   if(EL_output.power == TRUE && IO_Sen_in->panel_propellor_deice_switch == SW_ON)
   {
      prop_deice_timer += delta_time;

      if(prop_deice_timer < RPROP_DEICE_TIME)
      {
         if(prop_deice_timer < RPROP_OUTER_DEICE_TIME)
         {
            last_deiceSide = RIGHT_OUTER;
            if(IO_Sen_faults.FF.r_prop_deice == FALSE)
            {
               IO_Sen_out->deice_ammeter = RIGHT_OUTER_DEICE_LOAD;
               IC_ice_load = TRUE;
            }
            else
               IO_Sen_out->deice_ammeter = FAILED_PROP_LOAD;
         }
         else
         {
            last_deiceSide = RIGHT_INNER;
            IO_Sen_out->deice_ammeter = RIGHT_INNER_DEICE_LOAD;
            IC_ice_load = TRUE;
         }

      }
      else
      if(prop_deice_timer < LPROP_DEICE_TIME)
      {
         if(prop_deice_timer < LPROP_OUTER_DEICE_TIME)
         {
            last_deiceSide = LEFT_OUTER;
            if(IO_Sen_faults.FF.l_prop_deice == FALSE)
            {
               IO_Sen_out->deice_ammeter = LEFT_OUTER_DEICE_LOAD;
               IC_ice_load = TRUE;
            }
            else
               IO_Sen_out->deice_ammeter = FAILED_PROP_LOAD;
         }
         else
         {
            last_deiceSide = LEFT_INNER;
            IO_Sen_out->deice_ammeter = LEFT_INNER_DEICE_LOAD;
            IC_ice_load = TRUE;
         }

      }
      else
         prop_deice_timer = 0.0;
   }
   else
      IO_Sen_out->deice_ammeter = 0.0;


last_deiceSwitch = IO_Sen_in->panel_propellor_deice_switch;


if(EL_output.power == TRUE)
{
   if(IO_Sen_in->panel_surface_deice_switch == SW_ON && outputs.user[Y_A_VACUUM] > 4.5)
   {
      wing_deice_ct   = 0.0;
      wing_deice_flag = TRUE;
      dip_vac         = 0.5;
   }

   if(wing_deice_flag == TRUE)
   {
      //play inflate sound
      wing_deice_ct += delta_time;
      if(wing_deice_ct > DEICE1)
      {
         IO_Sen_out->panel_wing_tail_lamp = (unsigned char)(LIT * EL_output.power);
      }

      if(wing_deice_ct > DEICE2)
      {
         IO_Sen_out->panel_wing_tail_lamp = UNLIT;
      }

      if(wing_deice_ct > DE_ICE_TIME)
      {
         wing_deice_ct = 0.0;
         wing_deice_flag = FALSE;
         dip_vac = 0.0;
         //play pop sound
      }
   }

}
else
{
	wing_deice_ct = 0.0;
   wing_deice_flag = FALSE;
   dip_vac = 0.0;
}

   // low bus lamp this should be in elec but was not included

   left_gen_spd = (outputs.user[Y_A_L_RPM] * (!IO_Sen_in->elec_left_alternator_switch)* !IO_Sen_in->MCB_alt_field_left_tripped);
   right_gen_spd = (outputs.user[Y_A_R_RPM] * (!IO_Sen_in->elec_right_alternator_switch)* !IO_Sen_in->MCB_alt_field_right_tripped);

   left_alt_low = ((left_gen_spd < 1200) && (left_gen_spd > 1000));
   right_alt_low = ((right_gen_spd < 1200) && (right_gen_spd > 1000));

   left_off = (left_gen_spd < 900);
   right_off = (right_gen_spd < 900);
/*
   if((left_alt_low && (right_alt_low || right_off)) || (right_alt_low && (left_alt_low || left_off)))
   {
      bus_low_flash += delta_time;
      if(bus_low_flash > BUS_FLASH)
      {
         low_bus = !low_bus;
         bus_low_flash = 0.0;
      }
   }
   else
   */
      low_bus = (left_off && right_off);

}


void cwp_panel(void)
{
	if(IO_Sen_in->panel_annunciator_test_button == SW_ON && last_cwp_test == SW_OFF)
   {	//store values before setting all on
	   cwp_lamp.alt_out_lamp              = IO_Sen_out->panel_annunciator_alt_out_lamp;
	   cwp_lamp.baggage_door_lamp         = IO_Sen_out->panel_annunciator_baggage_door_lamp;
	   cwp_lamp.cabin_door_lamp           = IO_Sen_out->panel_annunciator_cabin_door_lamp;
	   cwp_lamp.gear_unsafe_lamp          = IO_Sen_out->panel_annunciator_gear_unsafe_lamp;
	   cwp_lamp.heater_overheat_lamp      = IO_Sen_out->panel_annunciator_heater_overheat_lamp;
	   cwp_lamp.left_auxiliary_fuel_lamp  = IO_Sen_out->panel_annunciator_left_auxiliary_fuel_lamp;
	   cwp_lamp.left_gyro_air_lamp        = IO_Sen_out->panel_annunciator_left_gyro_air_lamp;
	   cwp_lamp.left_overboost_lamp       = IO_Sen_out->panel_annunciator_left_overboost_lamp;
	   cwp_lamp.oil_press_lamp            = IO_Sen_out->panel_annunciator_oil_press_lamp;
	   cwp_lamp.right_auxiliary_fuel_lamp = IO_Sen_out->panel_annunciator_right_auxiliary_fuel_lamp;
	   cwp_lamp.right_gyro_air_lamp       = IO_Sen_out->panel_annunciator_right_gyro_air_lamp;
	   cwp_lamp.right_overboost_lamp      = IO_Sen_out->panel_annunciator_right_overboost_lamp;

      IO_Sen_out->panel_annunciator_left_overboost_lamp       = LIT;
      IO_Sen_out->panel_annunciator_right_overboost_lamp      = LIT;
      IO_Sen_out->panel_annunciator_oil_press_lamp            = LIT;
      IO_Sen_out->panel_annunciator_left_gyro_air_lamp        = LIT;
      IO_Sen_out->panel_annunciator_right_gyro_air_lamp       = LIT;
      IO_Sen_out->panel_annunciator_baggage_door_lamp         = UNLIT; //MODIFICADO POR MI ANTES LIT
      IO_Sen_out->panel_annunciator_alt_out_lamp              = LIT;
      IO_Sen_out->panel_annunciator_gear_unsafe_lamp          = LIT;
      IO_Sen_out->panel_annunciator_heater_overheat_lamp      = LIT;
      IO_Sen_out->panel_annunciator_cabin_door_lamp           = UNLIT; //MODIFICADO POR MI ANTES LIT;
      IO_Sen_out->panel_annunciator_left_auxiliary_fuel_lamp  = LIT;
      IO_Sen_out->panel_annunciator_right_auxiliary_fuel_lamp = LIT;

   }


   if(IO_Sen_in->panel_annunciator_test_button == SW_ON)
   {
       IO_Sen_out->panel_annunciator_left_overboost_lamp       = LIT;
       IO_Sen_out->panel_annunciator_right_overboost_lamp      = LIT;
       IO_Sen_out->panel_annunciator_oil_press_lamp            = LIT;
       IO_Sen_out->panel_annunciator_left_gyro_air_lamp        = LIT;
       IO_Sen_out->panel_annunciator_right_gyro_air_lamp       = LIT;
       IO_Sen_out->panel_annunciator_baggage_door_lamp         = UNLIT; //MODIFICADO POR MI ANTES LIT
       IO_Sen_out->panel_annunciator_alt_out_lamp              = LIT;
       IO_Sen_out->panel_annunciator_gear_unsafe_lamp          = LIT;
       IO_Sen_out->panel_annunciator_heater_overheat_lamp      = LIT;
       IO_Sen_out->panel_annunciator_cabin_door_lamp           = UNLIT; //MODIFICADO POR MI ANTES LIT
       IO_Sen_out->panel_annunciator_left_auxiliary_fuel_lamp  = LIT;
       IO_Sen_out->panel_annunciator_right_auxiliary_fuel_lamp = LIT;
   }



   if(IO_Sen_in->panel_annunciator_test_button == SW_OFF && last_cwp_test == SW_ON)
   {	   //restore values 
	   IO_Sen_out->panel_annunciator_alt_out_lamp = cwp_lamp.alt_out_lamp;
	   IO_Sen_out->panel_annunciator_baggage_door_lamp = cwp_lamp.baggage_door_lamp;
	   IO_Sen_out->panel_annunciator_cabin_door_lamp = cwp_lamp.cabin_door_lamp;
	   IO_Sen_out->panel_annunciator_gear_unsafe_lamp = cwp_lamp.gear_unsafe_lamp;
	   IO_Sen_out->panel_annunciator_heater_overheat_lamp = cwp_lamp.heater_overheat_lamp;
	   IO_Sen_out->panel_annunciator_left_auxiliary_fuel_lamp = cwp_lamp.left_auxiliary_fuel_lamp;
	   IO_Sen_out->panel_annunciator_left_gyro_air_lamp = cwp_lamp.left_gyro_air_lamp;
	   IO_Sen_out->panel_annunciator_left_overboost_lamp = cwp_lamp.left_overboost_lamp;
	   IO_Sen_out->panel_annunciator_oil_press_lamp = cwp_lamp.oil_press_lamp;
	   IO_Sen_out->panel_annunciator_right_auxiliary_fuel_lamp = cwp_lamp.right_auxiliary_fuel_lamp;
	   IO_Sen_out->panel_annunciator_right_gyro_air_lamp = cwp_lamp.right_gyro_air_lamp;
	   IO_Sen_out->panel_annunciator_right_overboost_lamp = cwp_lamp.right_overboost_lamp;
   }

last_cwp_test = IO_Sen_in->panel_annunciator_test_button;
}