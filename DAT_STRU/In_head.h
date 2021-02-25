/*
File 				:	in_head.h

Version			:	 1.5		28/4/00


Module			:	Cockpit Hardware Driver for Seneca III FNPT2


Reference		:	I

Description	:	The data is padded ourt to 32 bits and the
							structure is 340 bytes long.


Mod. Record	: Date		Notes																By
							====		=====																==
							26/1		Initial Release											RJW
							31/1		'int's removed											RJW
							1/2			Fuel selects changed from float to	RJW
											char.
											Roll trim changed to rudder trim
											Removed air_temp & air_intake
							10/2		QTG Validation support added				RJW
							17/2		KX155 Nav Ident Pull switch added		RJW
											Comm 1 PTT & Comm 2 PTT added				RJW
							6/4			Added FCI actual positions					RJW
							28/4		Version 1.5 created									RJW
							29/7		Version 1.6 created. added test sw's


*/


#define NOT_PRESSED 1
#define PRESSED 0
#define SW_ON 0
#define SW_OFF 1


// Definitions for special switch states

#define KR87_ANT 1   //ANT with switch out
#define KR87_ADF 0   //ADF with switch In

#define KR21_HI 0
#define KR21_LO 1
#define KR21_TEST 2

#define KT76_OFF_STBY 0
#define KT76_ON_ALT 1
#define KT76_TEST 2


#define PROP_OFF 0
#define PROP_1 1
#define PROP_2 2

#define FUEL_AUX 0
#define FUEL_LO 1
#define FUEL_HI 2
#define FUEL_OFF 3
#define FUEL_ON 4
#define FUEL_XFEED 5

#define FD_PILOT 0
#define FD_COPILOT 1

#define RMI_NAV1 0
#define RMI_NAV2 1


// Main definitions of switches & lamps
typedef struct
{

// For QTG Validation
float test_control_column_force;
float test_light_sensor;

// **********************************

// Avionics

// **********************************


// KY196
long KY196_delta_inner_encoder;
long KY196_delta_outer_encoder;
long KY196_delta_inner_encoder_pull;
unsigned char KY196_off_on_switch;
unsigned char KY196_off_on_pull_switch;
unsigned char KY196_FRQ_button;
unsigned char KY196_CHAN_button;


// KX155
unsigned char KX155_off_on_switch;
unsigned char KX155_off_on_pull_switch;
unsigned char KX155_COMM_FRQ_button;
unsigned char KX155_NAV_FRQ_button;
unsigned char KX155_NAV_ident_pull_switch;
unsigned char pack0[3];
long KX155_COMM_delta_outer_encoder;
long KX155_COMM_delta_inner_encoder;
long KX155_COMM_delta_inner_encoder_pull;
long KX155_NAV_delta_inner_encoder;
long KX155_NAV_delta_outer_encoder;
long KX155_NAV_delta_inner_encoder_pull;

// KNS80
long KNS80_delta_inner_encoder;
long KNS80_delta_outer_encoder;
long KNS80_delta_inner_encoder_pull;
unsigned char KNS80_VOR_button;
unsigned char KNS80_RNAV_button;
unsigned char KNS80_HOLD_button;
unsigned char KNS80_USE_button;

unsigned char KNS80_DSP_button;
unsigned char KNS80_DATA_button;
unsigned char KNS80_off_on_switch;
unsigned char KNS80_off_on_pull_switch;


// KFC150
unsigned char KFC150_FD_button;
unsigned char KFC150_ALT_button;
unsigned char KFC150_HDG_button;
unsigned char KFC150_NAV_button;

unsigned char KFC150_APR_button;
unsigned char KFC150_BC_button;
unsigned char KFC150_AP_ENG_button;
unsigned char KFC150_TEST_button;

unsigned char KFC150_UP_button;
unsigned char KFC150_DOWN_button;
unsigned char pack1[2];


// KR21
unsigned char KR21_beacon_sensitivity;


// KT76
unsigned char KT76_IDENT_button;
unsigned char KT76_off_on;


// KR87 ADF

unsigned char KR87_off_on_switch;
unsigned char KR87_ADF_ANT_switch;
unsigned char KR87_BFO_switch;
unsigned char KR87_FRQ_button;
unsigned char KR87_FLT_ET_button;

unsigned char KR87_SET_RST_button;
unsigned char pack2[3];


long KR87_delta_outer_encoder;
long KR87_delta_inner_encoder;
long KR87_delta_inner_encoder_pull;

// KA51's
unsigned char pilot_KA51_slew_CW_switch;
unsigned char pilot_KA51_slew_CCW_switch;
unsigned char pilot_KA51_mode_switch;

unsigned char copilot_KA51_slew_CW_switch;
unsigned char copilot_KA51_slew_CCW_switch;
unsigned char copilot_KA51_mode_switch;


// Audio
unsigned char comm_1_PTT;
unsigned char comm_2_PTT;


//

// **********************************

// Cockpit Discretes

// **********************************
unsigned char system_freeze_button;

unsigned char MCB_alt_field_left_tripped;
unsigned char MCB_alt_field_right_tripped;
unsigned char MCB_landing_gear_pump_tripped;

unsigned char MCB_landing_gear_control_tripped;
unsigned char MCB_electric_flaps_tripped;
unsigned char MCB_pitch_trim_tripped;
unsigned char MCB_autopilot_tripped;

unsigned char panel_annunciator_test_button;
unsigned char panel_RMI_switch;
unsigned char panel_propellor_deice_switch;
unsigned char panel_windscreen_deice_switch;

unsigned char panel_surface_deice_switch;
unsigned char panel_wing_deice_switch;

unsigned char yokes_CWS_switch;
unsigned char yokes_trim_up_button;

unsigned char yokes_trim_down_button;
unsigned char yokes_autopilot_disconnect_button;

unsigned char interseat_fan_heat_switch;
unsigned char interseat_on_defrost_switch;

unsigned char throttle_prop_sync_switch;

unsigned char elec_battery_switch;
unsigned char elec_left_alternator_switch;
unsigned char elec_right_alternator_switch;

unsigned char elec_left_start_switch;
unsigned char elec_right_start_switch;
unsigned char elec_left_magneto_1_switch;
unsigned char elec_right_magneto_1_switch;

unsigned char elec_left_magneto_2_switch;
unsigned char elec_right_magneto_2_switch;
unsigned char elec_recognition_lights_switch;
unsigned char elec_labding_lights_1_switch;

unsigned char elec_labding_lights_2_switch;
unsigned char elec_navigation_lights_switch;
unsigned char elec_anti_collision_lights_switch;
unsigned char elec_pitot_heater_switch;

unsigned char elec_left_fuel_switch;
unsigned char elec_right_fuel_switch;
unsigned char elec_left_primer_button;
unsigned char elec_right_primer_button;

unsigned char panel_parking_brake;
unsigned char panel_landing_gear_lever;
unsigned char panel_emergency_gear_switch;

unsigned char panel_left_ammeter_button;
unsigned char panel_right_ammeter_button;

unsigned char panel_avionisc_emergency_bus_switch;
unsigned char panel_avionisc_power_switch;

unsigned char panel_FD_pilot_copilot_switch;
unsigned char panel_RMI_nav1_nav2_switch;

unsigned char panel_left_manifold_gauge_drain_button;
unsigned char panel_right_manifold_gauge_drain_button;

unsigned char left_fuel_select;
unsigned char right_fuel_select;

unsigned char pack5[3];

// **********************************

// Cockpit Instruments

// **********************************
// Altimeters
float pilot_altimeter_1_current_height;

float pilot_altimeter_2_current_height;

float copilot_altimeter_current_height;

unsigned char pilot_altimeter_1_status;
unsigned char pilot_altimeter_2_status;
unsigned char copilot_altimeter_status;
unsigned char pack6;

// Nav2
float nav2_obs;

// Pilot FCI
float pilot_FCI_actual_pitch;
float pilot_FCI_actual_roll;

// Copilot FCI
float copilot_FCI_actual_pitch;
float copilot_FCI_actual_roll;

// Pilot HSI
float pilot_hsi_obs;
float pilot_hsi_compass;
float pilot_hsi_course_error;
float pilot_hsi_heading_error;

// Copilot HSI
float copilot_hsi_obs;
float copilot_hsi_compass;
float copilot_hsi_course_error;
float copilot_hsi_heading_error;



// **********************************

// Cockpit Analogue Inputs

// **********************************
// Range on these controls is 0-100 unless noted otherwise
float pilot_left_toebrake;
float pilot_right_toebrake;
float copilot_left_toebrake;
float copilot_right_toebrake;

float left_throttle;
float right_throttle;
float left_prop_rpm;
float right_prop_rpm;
float left_engine_mixture;
float right_engine_mixture;
float left_alternate_air;
float right_alternate_air;
float left_cowl_flap_control;
float right_cowl_flap_control;
float flap_control_lever;						// Note range is 0 to 40


float pitch_trim_wheel;            // Range is +/- 100
float rudder_trim_wheel;           // Range is +/- 100


float test_force;
float test_visuals;

unsigned char test_switch_1_up;
unsigned char test_switch_1_dn;
unsigned char test_switch_2_up;
unsigned char test_switch_2_dn;
unsigned char test_switch_3_up;
unsigned char test_switch_3_dn;
unsigned char test_switch_4_up;
unsigned char test_switch_4_dn;
 
float control_yoke_pitch_position; // Range is +/- 100
float control_yoke_roll_position;  // Range is +/- 100
float rudder_pedal_position;       // Range is +/- 100

// AÑADIDO para screens desde dinamico BEATERIO MODIFICADO
float pilot_hsi_atp;
float copilot_hsi_atp;


long alt1_cap_baro;
long alt2_cap_baro;
long alt3_fo_baro;

float pilot_ias_set;
float copilot_ias_set;



unsigned char pack25[2];
unsigned char io_operating_mode;



} sen_in_struct;

typedef struct {         //==========================ESTRUCTURA DE PRUEBA  //AÑADIDO POR MANOLO
	float a;
	float b;
	float c;
}OUTPUTin;           //==========================ESTRUCTURA DE PRUEBA  //AÑADIDO POR MANOLO

typedef struct
{
float qtg_pilot_roll;
float qtg_pilot_pitch;
float qtg_pilot_compass;

} sen_qtg_struct;
