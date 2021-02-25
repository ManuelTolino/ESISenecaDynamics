/*
File: faults.h
Description: header file containing fault defines for seneca.
             The order of bits in the structures corresponds
             to the order of bits from the IOS
*/

/***************************/
/*   RCS Markers           */
/***************************/

/*
$Header: C:/master/seneca/RCS/faults.h 1.2 2000/05/14 10:28:01 colinj Exp $
$Log: faults.h $
Revision 1.2  2000/05/14 10:28:01  colinj
Added landlight2 to load_struct.

Revision 1.1  2000/04/14 17:58:48  colinj
Initial revision

*/

//---------------------------------------------------------------------

//FOR INFORMATION ONLY
//Here we are relying on the first entry of a bit field
//structure corresponding to bit 0 of the corresponding long.


typedef struct
{
   unsigned P1_ATITUDE_IND     :1; //instrument failures
   unsigned P1_AIRSPEED_IND    :1; //instrument failures
   unsigned P1_ALTIMETER_1     :1; //instrument failures
   unsigned P1_ALTIMETER_2     :1; //instrument failures
   unsigned P1_TURN_IND        :1; //instrument failures
   unsigned P1_HORZ_POS_IND    :1; //instrument failures
   unsigned P1_DELTA_CLIMB     :1; //instrument failures
   unsigned P2_ATITUDE_IND     :1; //instrument failures
   unsigned P2_AIRSPEED_IND    :1; //instrument failures
   unsigned P2_ALTIMETER_1     :1; //instrument failures
   unsigned P2_TURN_IND        :1; //instrument failures
   unsigned P2_HORZ_POS_IND    :1; //instrument failures
   unsigned P2_DELTA_CLIMB     :1; //instrument failures
   unsigned PITOT_BLOCK        :1; //instrument failures
   unsigned STATIC_BLOCK       :1; //instrument failures
   unsigned Right_SUCTION_PUMP :1; //instrument failures
   unsigned Left_SUCTION_PUMP  :1; //instrument failures
   unsigned rmi_fail           :1;
   unsigned nav2_fail          :1;
}flight_inst;

typedef struct
{
unsigned l_fuel_starv :1; //fm failures
unsigned l_mag_l_fail :1; //fm failures
unsigned l_mag_r_fail :1; //fm failures
unsigned l_oil_temp   :1; //fm failures
unsigned l_oil_press  :1; //fm failures
unsigned l_cht_high   :1; //fm failures
unsigned l_prop_cntrl :1; //fm failures
unsigned l_prop_deice :1; //fm failures
unsigned r_fuel_starv :1; //fm failures
unsigned r_mag_l_fail :1; //fm failures
unsigned r_mag_r_fail :1; //fm failures
unsigned r_oil_temp   :1; //fm failures
unsigned r_oil_press  :1; //fm failures
unsigned r_cht_high   :1; //fm failures
unsigned r_prop_cntrl :1; //fm failures
unsigned r_prop_deice :1; //fm failures
unsigned l_mag_l_hot  :1;
unsigned l_mag_r_hot  :1;
unsigned l_fuel_pump  :1;
unsigned r_mag_l_hot  :1;
unsigned r_mag_r_hot  :1;
unsigned r_fuel_pump  :1;
}flight_fail;

typedef struct
{
unsigned l_alt_over   :1; //fm failures
unsigned l_alt_zero   :1; //fm failures
unsigned r_alt_over   :1; //fm failures
unsigned r_alt_zero   :1; //fm failures
unsigned l_alt_mcb                :1; //mcb failures
unsigned r_alt_mcb                :1; //mcb failures
unsigned landing_gear_pump_mcb    :1; //mcb failures
unsigned landing_gear_control_mcb :1; //mcb failures
unsigned electric_flaps_mcb       :1; //mcb failures
unsigned pitch_trim_mcb           :1; //mcb failures
unsigned autopilot_mcb            :1;
unsigned batt_flat                :1;
}elec_fail;

typedef struct
{
unsigned LEFT_MANIFOLD      :2;
unsigned LEFT_RPM_IND       :2;
unsigned LEFT_EGT_IND       :2;
unsigned LEFT_FUEL_FLOW_IND :2;
unsigned LEFT_FUEL_CONT_IND :2;
unsigned LEFT_OIL_PRESS_IND :2;
unsigned LEFT_OIL_TEMP_IND  :2;
unsigned LEFT_CYL_TEMP_IND  :2;

// Right Engine
unsigned RIGHT_MANIFOLD      :2;
unsigned RIGHT_RPM_IND       :2;
unsigned RIGHT_EGT_IND       :2;
unsigned RIGHT_FUEL_FLOW_IND :2;
unsigned RIGHT_FUEL_CONT_IND :2;
unsigned RIGHT_OIL_PRESS_IND :2;
unsigned RIGHT_OIL_TEMP_IND  :2;
unsigned RIGHT_CYL_TEMP_IND  :2;
}eng_inst;

typedef struct
{
   unsigned ammeter :2;
   unsigned suction :2;
}ammSuction;

typedef struct
{
   unsigned land_gear_ext    :1;
   unsigned land_gear_retrat :1;
   unsigned flaps            :1;
   unsigned airframe_icing   :1;
}airframe;

typedef struct
{
   unsigned com1         :1;
   unsigned com_nav2     :1;
   unsigned nav1         :1;
   unsigned radio_pwr    :1;
   unsigned adf          :1;
   unsigned radar        :1;
   unsigned auto_pilot   :1;
   unsigned standby_pwr  :1;
   unsigned left_gyro    :1;
   unsigned right_gyro   :1;
   unsigned trim_runaway :1;
   unsigned trim_fail    :1;
   unsigned spare        :20;

} av_fail;

typedef struct
{
   flight_inst FI;
   flight_fail FF;
   eng_inst    EI;
   elec_fail   EF;
   airframe    AF;
   av_fail     AVF;
   ammSuction  AS;
}
sen_fault_struct;

typedef struct
{
   unsigned left_alt    :1;
   unsigned right_alt   :1;
   unsigned batt_switch :1;
}switch_struct;

typedef struct
{
   unsigned ambient        :1;
   unsigned prop_deice     :1;
   unsigned pitot_heat     :1;
   unsigned gear_pump      :1;
   unsigned flap_motor     :1;
   unsigned vent_fan       :1;
   unsigned heater         :1;
   unsigned defrost_fan    :1;
   unsigned radios_tx      :1;
   unsigned landing_lights :1;
   unsigned recognition    :1;
   unsigned navigation     :1;
   unsigned strobes        :1;
   unsigned lstarter       :1;
   unsigned rstarter       :1;
   unsigned landlight2     :1;
}load_struct;