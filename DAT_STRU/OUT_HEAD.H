/*
File 				:	out_head.h

Version			:	1.6			26/5/00


Module			:	Cockpit Hardware Driver for Seneca III FNPT2


Reference		:

Description	:	The data is packed out to 32 bits and the
							structure is 432 bytes long.

Mod. Record	: Date		Notes																By
							====		=====																==
							26/1		Initial Release											RJW
							31/1		'int's removed											RJW
							1/2			driuve_pitch_trim_up &							RJW
											drive_pitch_trim_down added
											KNS80 MIN & RNV tokens added
											KNS80 display modes enhanced
							10/2		System control added								RJW
							17_2		Lo Bus added to Lamp PTT Enable			RJW
											KNS80 Use blank mode added
											KR87 STBY/TIME blank mode added
							28/4		Version 1.5 created									RJW
							26/5		dcl_mode added


*/


#define UNLIT 0
#define LIT 1



// Definitions for Avionics
#define BLANK 0
#define TEST 1
#define ACTIVE 2
#define KNS80_NM_DASH 3
#define KNS80_KT_DASH 4
#define KNS80_MIN_DASH 5
#define KNS80_FRQ 6
#define KNS80_RAD_DST 7
#define KNS80_DSP_BLANK 8
#define KR87_STBY_TIME_BLANK 9
#define KR87_STBY_LEADING_ZERO 10

#define DCL_ON 1
#define DCL_OFF 0
#define DCL_INIT 0
#define DCL_NORM 1

#define HW_INITIALISE 0
#define HW_PROCESS 1
#define HW_SHUTDOWN 2



// Main definitions of switches & lamps
typedef struct
{

unsigned char io_operating_mode;								// HW_INITIALISE, HW_PROCESS or HW_SHUTDOWN
unsigned char pack19[3];

// **********************************

// Avionics

// **********************************


// KY196
long KY196_USE_display;
long KY196_STBY_display;
unsigned char KY196_mode;
unsigned char KY196_T_token;
unsigned char pack20[2];

// KX155
long KX155_NAV_USE_display;
long KX155_NAV_STBY_display;
long KX155_COMM_USE_display;
long KX155_COMM_STBY_display;
unsigned char KX155_mode;
unsigned char KX155_T_token;

unsigned char pack21[1];

// KNS80
unsigned char KNS80_USE_display_mode;
unsigned char KNS80_DSP_display_mode;
unsigned char KNS80_NM_display_mode;
unsigned char KNS80_KT_display_mode;
unsigned char KNS80_MIN_display_mode;

unsigned char KNS80_FRQ_RAD_DST_display_mode;
unsigned char KNS80_NM_token;
unsigned char KNS80_KT_token;
unsigned char KNS80_MIN_token;

unsigned char KNS80_FREQ_token;
unsigned char KNS80_RAD_token;
unsigned char KNS80_DST_token;
unsigned char KNS80_PAR_token;

unsigned char KNS80_VOR_token;
unsigned char KNS80_ENR_token;
unsigned char KNS80_APR_token;
unsigned char KNS80_RNV_token;

unsigned char KNS80_HLD_token;
unsigned char KNS80_ILS_token;
unsigned char KNS80_USE_token;
unsigned char KNS80_DSP_token;

long KNS80_USE_display;
long KNS80_DSP_display;
long KNS80_MIN_display;
long KNS80_NM_display;
long KNS80_KT_display;
long KNS80_FREQ_RAD_DST_display;


// KFC150
unsigned char KFC150_YD_token;
unsigned char KFC150_FD_token;
unsigned char KFC150_ALT_token;
unsigned char KFC150_HDG_token;
unsigned char KFC150_GS_token;
unsigned char KFC150_NAV_token;
unsigned char KFC150_APR_token;
unsigned char KFC150_BC_token;
unsigned char KFC150_TRIM_token;
unsigned char KFC150_AP_token;


// KR21
unsigned char KR21_A_token;
unsigned char KR21_O_token;
unsigned char KR21_M_token;


// KT76
unsigned char KT76_IDENT_lamp;
unsigned char pack23[1];


// KR87 ADF
unsigned char KR87_USE_display_mode;
unsigned char KR87_STBY_TIME_display_mode;
unsigned char KR87_ANT_token;
unsigned char KR87_ADF_token;
unsigned char KR87_BFO_token;
unsigned char KR87_FRQ_token;
unsigned char KR87_FLT_token;
unsigned char KR87_ET_token;
unsigned char KR87_colon_token;
long KR87_USE_display;
long KR87_STBY_TIME_display;


// KA51's
float pilot_KA51_slaving_meter;
float copilot_KA51_slaving_meter;


// **********************************

// Cockpit Discretes

// **********************************
unsigned char MCB_alt_field_left_trigger;
unsigned char MCB_alt_field_right_trigger;
unsigned char MCB_landing_gear_pump_trigger;
unsigned char MCB_landing_gear_control_trigger;
unsigned char MCB_electric_flaps_trigger;
unsigned char MCB_pitch_trim_trigger;
unsigned char MCB_autopilot_trigger;					// ???

unsigned char panel_left_start_engage_lamp;
unsigned char panel_right_start_engage_lamp;

unsigned char panel_low_bus_lamp;

unsigned char panel_landing_gear_nose_lamp;
unsigned char panel_landing_gear_left_lamp;
unsigned char panel_landing_gear_right_lamp;

unsigned char throttle_prop_sync_lamp;

unsigned char panel_flap_transit_lamp;
unsigned char panel_wing_tail_lamp;

unsigned char panel_lo_bus_wing_tail_transit_lamp_PTT_enable;

unsigned char system_power_control;
unsigned char system_DCL_power_control;
unsigned char system_servo_motor_power_control;
unsigned char system_stepper_motor_power_control;
unsigned char system_lighting_power_control;
unsigned char system_KMA24H_power_control;

unsigned char panel_stall_horn;
unsigned char panel_gear_horn;
unsigned char panel_HOBBS_meter;

unsigned char panel_annunciator_left_overboost_lamp;
unsigned char panel_annunciator_right_overboost_lamp;
unsigned char panel_annunciator_oil_press_lamp;
unsigned char panel_annunciator_left_gyro_air_lamp;
unsigned char panel_annunciator_right_gyro_air_lamp;
unsigned char panel_annunciator_baggage_door_lamp;
unsigned char panel_annunciator_alt_out_lamp;
unsigned char panel_annunciator_gear_unsafe_lamp;
unsigned char panel_annunciator_heater_overheat_lamp;
unsigned char panel_annunciator_cabin_door_lamp;
unsigned char panel_annunciator_left_auxiliary_fuel_lamp;
unsigned char panel_annunciator_right_auxiliary_fuel_lamp;

unsigned char drive_pitch_trim_up;
unsigned char drive_pitch_trim_down;



// **********************************

// Cockpit Instruments

// **********************************
// Altimeters
float pilot_altimeter_1_demand_height;
float pilot_altimeter_2_demand_height;
float copilot_altimeter_demand_height;

// Nav2
float nav2_to_from_flag;
float nav2_left_right_needle;
float nav2_GS_needle;
unsigned char nav2_nav_flag;
unsigned char nav2_GS_flag;
unsigned char pack25[2];

// Pilot FCI
float pilot_FCI_pitch_demand_position;
float pilot_FCI_roll_demand_position;
unsigned long pilot_FCI_cmd_enable;
float pilot_FCI_cmd_pitch;
float pilot_FCI_cmd_roll;

// Copilot FCI
float copilot_FCI_pitch_demand_position;
float copilot_FCI_roll_demand_position;
unsigned long copilot_FCI_cmd_enable;
float copilot_FCI_cmd_pitch;
float copilot_FCI_cmd_roll;


// RMI
float RMI_demand_heading;
float RMI_ADF_needle;
float RMI_VOR_needle;


// Pilot HSI
float pilot_hsi_compass_demand_position;
float pilot_hsi_left_right_needle;
float pilot_hsi_to_from_needle;
float pilot_hsi_GS_needle;
unsigned char pilot_hsi_nav_flag;
unsigned char pilot_hsi_hdg_flag;
unsigned char pilot_hsi_GS_enable;
unsigned char pack26;

// Copilot HSI
float copilot_hsi_compass_demand_position;
float copilot_hsi_GS_needle;
float copilot_hsi_left_right_needle;
float copilot_hsi_to_from_needle;
unsigned char copilot_hsi_GS_enable;
unsigned char copilot_hsi_nav_flag;
unsigned char copilot_hsi_hdg_flag;
unsigned char pack27;

// ASI
float pilot_airspeed;
float copilot_airspeed;

// VSI
float pilot_rate_of_climb;
float copilot_rate_of_climb;

// turn & Slip
float pilot_turn;
float pilot_slip;

float copilot_turn;
float copilot_slip;


// Compass
float compass_heading;

// OAT
float outside_air_temperature;


// Engine Instruments
float left_manifold_pressure;
float right_manifold_pressure;

float left_fuel_flow;
float right_fuel_flow;

float left_tacho;
float right_tacho;

float left_EGT;
float right_EGT;

float left_fuel_quantity;
float right_fuel_quantity;

float left_oil_pressure;
float right_oil_pressure;

float left_oil_temperature;
float right_oil_temperature;

float left_cyclic_temperature;
float right_cyclic_temperature;

float ammeter;
float	suction;
float deice_ammeter;



// DCL
float pitch_demand_position;
float pitch_demand_force;

float roll_demand_position;
float roll_demand_force;

float rudder_demand_position;
float rudder_demand_force;

float pitch_trim_demand_position;   // These are reserved for use by
float pitch_trim_demand_force;			// proces_hardware()


//AÑADIDO POR MI SCREENS BEATERIO MODIFICADO


unsigned char dcl_mode;
unsigned char pack28[3];

float pilot_hsi_obs;
float pilot_hsi_atp;
float copilot_hsi_obs;
float copilot_hsi_atp;

float nav2_obs;

long alt1_cap_baro;
long alt2_cap_baro;
long alt3_fo_baro;

float pilot_ias_set;
float copilot_ias_set;

unsigned char battery_on;

long qtgid;

} sen_out_struct;


// Main definitions of SCREENS
typedef struct
{

unsigned char io_operating_mode;								// HW_INITIALISE, HW_PROCESS or HW_SHUTDOWN

// Altimeters
float pilot_altimeter_1_demand_height;
float pilot_altimeter_2_demand_height;
float copilot_altimeter_demand_height;

// Nav2
float nav2_to_from_flag;
float nav2_left_right_needle;
float nav2_GS_needle;
unsigned char nav2_nav_flag;
unsigned char nav2_GS_flag;
unsigned char pack25[2];

// Pilot FCI
float pilot_FCI_pitch_demand_position;
float pilot_FCI_roll_demand_position;
unsigned long pilot_FCI_cmd_enable;
float pilot_FCI_cmd_pitch;
float pilot_FCI_cmd_roll;

// Copilot FCI
float copilot_FCI_pitch_demand_position;
float copilot_FCI_roll_demand_position;
unsigned long copilot_FCI_cmd_enable;
float copilot_FCI_cmd_pitch;
float copilot_FCI_cmd_roll;


// RMI
float RMI_demand_heading;
float RMI_ADF_needle;
float RMI_VOR_needle;


// Pilot HSI
float pilot_hsi_compass_demand_position;
float pilot_hsi_left_right_needle;
float pilot_hsi_to_from_needle;
float pilot_hsi_GS_needle;
unsigned char pilot_hsi_nav_flag;
unsigned char pilot_hsi_hdg_flag;
unsigned char pilot_hsi_GS_enable;
unsigned char pack26;

// Copilot HSI
float copilot_hsi_compass_demand_position;
float copilot_hsi_GS_needle;
float copilot_hsi_left_right_needle;
float copilot_hsi_to_from_needle;
unsigned char copilot_hsi_GS_enable;
unsigned char copilot_hsi_nav_flag;
unsigned char copilot_hsi_hdg_flag;
unsigned char pack27;

// ASI
float pilot_airspeed;
float copilot_airspeed;

// VSI
float pilot_rate_of_climb;
float copilot_rate_of_climb;

// turn & Slip
float pilot_turn;
float pilot_slip;

float copilot_turn;
float copilot_slip;



// Engine Instruments
float left_manifold_pressure;
float right_manifold_pressure;

float left_fuel_flow;
float right_fuel_flow;

float left_tacho;
float right_tacho;

float left_EGT;
float right_EGT;

float left_fuel_quantity;
float right_fuel_quantity;

float left_oil_pressure;
float right_oil_pressure;

float left_oil_temperature;
float right_oil_temperature;

float left_cyclic_temperature;
float right_cyclic_temperature;

unsigned char pack28[3];

float pilot_hsi_obs;
float pilot_hsi_atp;
float copilot_hsi_obs;
float copilot_hsi_atp;

float nav2_obs;

long alt1_cap_baro;
long alt2_cap_baro;
long alt3_fo_baro;

float pilot_ias_set;
float copilot_ias_set;

unsigned char battery_on;

} sen_screens_struct;