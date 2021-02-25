head	1.5;
access;
symbols;
locks; strict;
comment	@ * @;


1.5
date	2000.06.13.14.57.10;	author COLINJ;	state Exp;
branches;
next	1.4;

1.4
date	2000.05.31.16.13.41;	author colinj;	state Exp;
branches;
next	1.3;

1.3
date	2000.05.24.15.42.15;	author colinj;	state Exp;
branches;
next	1.2;

1.2
date	2000.05.10.09.41.57;	author colinj;	state Exp;
branches;
next	1.1;

1.1
date	2000.04.17.11.12.22;	author colinj;	state Exp;
branches;
next	;


desc
@@


1.5
log
@changed pitch_trim_auth and map_dot values.
added new factors for mag drops so drops on each engine are different.
@
text
@/*****************************************************************************
 **                                                                         **
 ** File        : FMS_S3FM.H                                                **
 **                                                                         **
 ** Description : Seneca III flight model-specific include file             **
 **               Aircraft physical constants                               **
 **                                                                         **
 ** Created     : 31 Jan 2000, M G Kellett                                  **
 **                                                                         **
 **                                                                         **
 **                                          (c) Flight Dynamics Ltd, 2000  **
 *****************************************************************************/

/**
 ** Seneca III flight model physical constants
 **/

//**************************
//   RCS Markers           *
//**************************

//$Header: C:/MASTER/seneca/FLTDY/RCS/fms_s3fm.h 1.4 2000/05/31 16:13:41 colinj Exp COLINJ $
//$Log: fms_s3fm.h $
//Revision 1.4  2000/05/31 16:13:41  colinj
//Changed alpha_s_t and alpha_stall_warn.
//
//Revision 1.3  2000/05/24 15:42:15  colinj
//Updates from Martin Kellet.
//
//Revision 1.2  2000/05/10 09:41:57  colinj
//Updates from Martin K.
//
//Revision 1.1  2000/04/17 11:12:22  colinj
//Initial revision
//

/*---------------------------------------------------------------*/

// frig mountain addition
const float thrust_scale = (float) 0.70;//0.80;
const float drag_scale   = (float) 0.75;//0.75;
// asymmetric kick factors due to differential rate of change of MAP
const float k_N_delta_MAP_dot = 850.0;//CJ 2.0e3;
const float k_L_delta_MAP_dot = 1.0e3;

//const float k_dalpha_stall_T = (float)(-60.0/(57.3*20.0e3));
const float k_dalpha_stall_T = (float)0.0;//(-40.0/(57.3*20.0e3));
// Cubic stick shaping functions on aileron (xsi), rudder (zeta) and elevator (eta).
// f factors control how much effect is applied, 1.0=max effect, 0.0=no effect, i.e.
// normal linear operation
const float f1 = (float) 0.0;
const float f2 = (float) 0.0;
const float f3 = (float) 0.0;

// speed trimming parameters

const float CQ0   = (float) 0.001; //0.002;//0.0015;//1.0e-3;//0.0015; //0.0025; //0.005; //0.0025;
const float kCQ   = (float) 3.0;//3.0;//0.01;//3.0; //5.0;//7.5; //10.0; //25.0; //2.78;

const float Cd0   = (float)  0.035;//0.030;//0.025;//0.030; // from Roskam Cessna 310
const float k     = (float)  0.050; //0.055; // from Roskam Cessna 310

const float dCTdbeta  = (float) 0.06; //0.12;

// phugoid SAS
const float Ksas_z = (float) 0.0;
const float Ksas_x = (float) 0.0;
const float KCd_phug = (float) 0.0005;//0.001;
                           // Airframe icing
const float T_ice   = (float) 20.0; // Icing time constant
const float T_deice = (float) 0.5; // De-icing time constant
const float dCl_ice = (float) 0.0; // Ice effect on lift coefficient
const float dCd_ice = (float) 0.02; // Ice effect on drag coefficient
const float dW_ice  = (float) 100.0; // Ice effect on airframe weight

//  Prop icing
const float T_prop_ice   = (float) 20.0; // Icing time constant
const float T_prop_deice = (float) 0.5; // De-icing time constant
const float dCQ_ice      = (float) 0.001; // Ice effect on lift coefficient

// Control authorities
const float auth_pitch_trim_aero  = (float) 0.0; //0.10;
const float auth_rudd_trim_aero   = (float) 0.10;
const float auth_pitch_trim  = (float) 1.0;//CJ 0.80; //0.50;
const float auth_rudd_trim   = (float) 1.0; //0.20;

const float auth_pitch_servo = (float) 0.10;
const float auth_roll_servo  = (float) 0.25;//0.0;//0.25; //0.10;

// Autopilot parameters

const float pitch_servo_trim_limit = (float) 0.25; /// more servo movement than this
// will trigger a trim movement request
const float flash1_rate = (float) 1.0; // Hz
const float flash2_rate = (float) 2.0; // Hz

const float ap_Ktheta_p  = (float) -10.0;//-10.0; //-1.0;
const float ap_Ktheta_i  = (float) 0.0; //-2.0;
const float ap_Ktheta_d  = (float) 0.0;

const float ap_Kphi_p    = (float) 10.0; //1.0;
const float ap_Kphi_i    = (float) 0.5; //2.0;
const float ap_Kphi_d    = (float) 0.0;

const float ap_Kpsi_p    = (float) 4.0; //2.0; // 2.0;
const float ap_Kpsi_i    = (float) 0.0;
const float ap_Kpsi_d    = (float) 0.0;

const float ap_theta0  = (float) (4.0*D2R);

const float ap_Khdot_p  = (float) 0.03;

const float ap_Kalt_p  = (float) 0.005;
const float ap_Kalt_i  = (float) 0.01;
const float ap_Kalt_d  = (float) 0.0;

const float ap_phi_max   = (float)  (20.0*D2R);
const float ap_theta_max = (float) (+15.0*D2R);
const float ap_theta_min = (float) (-10.0*D2R);

// Engine parameters

const float eng_min_speed           = (float) 50.0; // rad/s - engines don't
// generate any power below this speed
const float eng_shutdown_rate_limit = (float) 100.0;//250.0; // rad/s per second
const float eng_shutdown_t          = (float)  0.5;
const float k_f_bhp_mix_rich   = (float) -0.2; //-1.0;
const float f_mix_bhp_rich_max = (float) 2.00; // allows 100% rich before poser reduction
const float k_egt_mix          = (float) -2000.0;
const float k_egt_bhp          = (float)(1600.0/220.0);
const float idle_passage  = (float) 0.13;//0.208;//0.25; //0.2; //0.06; //0.005; //0.01;
const float t_throt       = (float) 0.2; //1.0;
const float t_rpm         = (float) 0.666;//0.666; //3.33;
const float max_boost     = (float) 33.0;//32.0;//26.0; //1.28*17.0;
const float max_rpm = (float) (2800.0*RPM2RPS);
const float max_bhp = (float) 220.0;
const float t_boost       = (float) 0.2; //0.5;
const float max_MAP       = (float) 40.0; // max power developed here


const float primer_max_ff = (float)    8.0;
const float T_ff         = (float)     0.5; // fuel flow damping
const float K_mag_fail   = (float)     0.99;//0.98;//0.96; //0.92; // mag fail bhp scale factor
const float bhp_mag_fail = (float)     1.0; //1.0;//3.0; // bhp drop for mag fail
const float l_bhp_factor = (float)     0.001; // CJ scale factor for mag drop cases
const float r_bhp_factor = (float)     0.0015; // CJ ditto

const float t_EGT        = (float)     1.0; // EGT time constant
const float EGT_0        = (float) +1000.0;//+1100.0;//+900.0; // EGT nominal temp (deg F)
const float EGT_man_P    = (float)   +7.5;//5.0;//+10.0;//+15.0; // EGT/manifold pressure relation
const float EGT_Vu       = (float)    -1.0; // EGT/speed relation
const float dEGT_Vu_cowl = (float)    -1.0; // EGT/speed effect of cowl flap
const float d_Vu_EGT      = (float)   10.0; // EGT/speed nominal speed

const float t_CHT        = (float)     25.0; // CHT time constant
const float CHT_0        = (float)   +100.0; // CHT nominal temp (deg F)
const float CHT_man_P    = (float)    +10.0; // CHT/manifold pressure relation
const float CHT_Vu       = (float)     -1.0; // CHT/speed relation
const float dCHT_Vu_cowl = (float)     -1.0; // CHT/speed effect of cowl flap
const float d_Vu_CHT      = (float)    10.0; // CHT/speed nominal speed

const float t_OilT        = (float)     50.0; // OilT time constant
const float OilT_0        = (float)   +50.0; // OilT nominal temp (deg F)
const float OilT_man_P    = (float)     +5.0; // OilT/manifold pressure relation
const float OilT_Vu       = (float)     -0.5; // OilT/speed relation
const float dOilT_Vu_cowl = (float)     -0.5; // OilT/speed effect of cowl flap
const float d_Vu_OilT      = (float)    10.0; // OilT/speed nominal speed

const float t_OilP        = (float)     2.0;//5.0;//25.0; // OilP time constant
const float OilP_0        = (float)    +30.0;//+20.0; // OilP nominal pressure (psi)
const float OilP_man_P    = (float)     +1.0; // OilP/manifold pressure relation

const float man_P_min = (float) 5.0;
const float man_P_max = (float) 40.0;
const float Qstart    = (float) 50.0; //100.0; // starter motor torque
const float t_man_P   = (float) 0.5; // manifold pressure time constant
const float Keng      = (float) 25.61e-3; // engine power coefficient
const float beta_hi   = (float) (90.0*D2R);
const float beta_lo   = (float) (12.0*D2R);//(14.0*D2R);
const float rpm_hi    = (float) (2800.0 * (2*PI)/60.0);
const float rpm_lo    = (float) (1000.0 * (2*PI)/60.0);// (1500.0 * (2*PI)/60.0);
const float CTmax     = (float) 0.10;//0.03;
//const float CQ0 = 0.0005;
//const float kCQ = 0.265;



const float Qmax      = (float) 6000.0;
const float Jprop     = (float) 2.0; //5.0; //15.0; // kg.m^2 (estimate)
const float prop_rad  = (float) 0.965; // m
const float prop_area = (float) 2.93; // m^2

// Undercarriage
// NG = nose gear, RG = right gear, LG = left gear

const float K_nose_steer    = (float)  0.05; //0.10; // 0.25;
const float t_N_gear_1      = (float)  1.00;
const float t_L_gear_1      = (float)  1.10;
const float t_R_gear_1      = (float)  1.12;
const float max_gear_rate_1 = (float) 10.00;

const float t_N_gear_2      = (float)  0.50;
const float t_L_gear_2      = (float)  0.55;
const float t_R_gear_2      = (float)  0.57;
const float max_gear_rate_2 = (float) 10.00;

const float NG_k = (float) 160.0e3;//80.0e3; //40.0e3; //20.0e3;
const float RG_k = (float) 40.0e3; //20.0e3; //10.0e3;
const float LG_k = (float) 40.0e3; //20.0e3; //10.0e3;

const float NG_b = (float)  150.0e3;//80.0e3;//40.0e3;//20.0e3; //8.5e3;
const float RG_b = (float)  15.0e3; //7.0e3;
const float LG_b = (float)  15.0e3; //7.0e3;

const float NG_x = (float) +2.0;
const float RG_x = (float) -1.0; //-0.5;
const float LG_x = (float) -1.0; //-0.5;

const float NG_y = (float)  0.0;
const float RG_y = (float) +2.0;
const float LG_y = (float) -2.0;

const float NG_z = (float)  0.0;
const float RG_z = (float)  0.0;
const float LG_z = (float)  0.0;

const float NG_xl = (float) 2.10;//2.2;
const float RG_xl = (float) 2.0;
const float LG_xl = (float) 2.0;

const float NG_preload = (float) 0.0;
const float RG_preload = (float) 0.0;
const float LG_preload = (float) 0.0;

const float NG_RateLimit    = (float) (1/5.0); // Change this to slow down gear transit
const float RG_RateLimit    = (float) (1/6.0); // (this is reciprocal of travel time)
const float LG_RateLimit    = (float) (1/7.0); // i.e. (1/6.0) is 6 seconds transit

const float NG_TimeConstant = (float) 0.1; // Stops the bump at the end!
const float RG_TimeConstant = (float) 0.1; // (probably best to leave alone)
const float LG_TimeConstant = (float) 0.1;

// Electric pitch trim motor system
//const float PT_RateLimit    = (float) 1.0;
//const float PT_TimeConstant = (float) 0.1;

// Aerodynamics

// aircraft geometry

const float y_eng = (float) 3.0;//1.90;

const float kx = (float) 1.90; // X radius of gyration
const float ky = (float) 1.37; // Y radius of gyration
const float kz = (float) 2.25; // Z radius of gyration

const float Sw    = (float) 18.98;
const float b     = (float) 11.86;
const float cbar  = (float)  1.60;
const float St    = (float)  3.60;
const float lt    = (float)  5.22;
const float Sf    = (float)  2.19;
const float lf    = (float)  4.93;
const float h0    = (float)  0.50; // 0.25; // W/B aero centre position (Fraction of MAC)

const float dCd_cowl_flap = (float) 0.005;//0.01;//0.005;
const float dCd_gear = (float) 0.03;//0.02;
const float dCM_gear = (float) -0.005;
const float Cla_w = (float)  5.1;
const float CM0       = (float) -0.1;
const float alpha_0_w = (float) -0.035;//-0.05;
const float alpha_s_w = (float) 0.26;
const float deda      = (float) 0.3;
const float Cla_t     = (float) 4.5;
const float Cl_t_eta  = (float) 1.9976;
const float alpha_s_t = (float) -0.25;//-0.15;//-0.2; //0.0;
const float alpha_stall_0    = (float) (15.5 / 57.3); // clean stall alpha
//const float alpha_stall_0    = (float) (13.14 / 57.3); // clean stall alpha
//const float alpha_stall_0    = (float) (16.4 / 57.3); // clean stall alpha
const float alpha_stall_warn = (float) (13.0 / 57.3); // stall warning alpha
//const float alpha_stall_warn = (float) (14.0 / 57.3); // stall warning alpha
const float k_pitch_damp = (float) 1.5; // pitch damping multiplier
const float ail_max  = (float) 25.0;
const float elev_max = (float) 15.0; //25.0;
const float rudd_max = (float) 25.0;

// principal derivatives
const float Yv    = (float) -1.0;
const float Lv    = (float) -1.0;//-0.2;//-1.0;//-0.5;
const float Lp    = (float) -0.5;
const float Nv    = (float)  1.0;
const float Nr    = (float) -0.5;
// secondary derivatives
const float Yp    = (float)  0.0;
const float Yr    = (float)  0.0;
const float Lr    = (float)  0.1;
const float Np    = (float)  0.0;
// control derivatives
const float Yzeta = (float)  0.0;
const float Lzeta = (float)  0.0;
const float Nzeta = (float) -0.125;//-0.15;//-0.80;
const float Yxsi  = (float)  0.0;
const float Lxsi  = (float) -0.25;
const float Nxsi  = (float)  0.0;

/**
const float Yv    = (float) -0.958;
const float Yp    = (float) -0.0255;
const float Yr    = (float)  0.2275;
const float Lv    = (float) -0.5;
const float Lp    = (float) -0.5;
const float Lr    = (float)  0.0742;
const float Nv    = (float)  0.5; //0.1595;
const float Np    = (float) -0.01823;
const float Nr    = (float) -0.5; //-0.0866;
const float Yzeta = (float)  0.3414;
const float Lzeta = (float)  0.02863;
const float Nzeta = (float) -0.06; //-0.120;
const float Yxsi  = (float) -0.02988;
const float Lxsi  = (float) -0.2357;
const float Nxsi  = (float)  0.0;//0.0051;
**/

// JIT values
/*********
const float Yv    = -0.958;
const float Yp    = -0.0255;
const float Yr    =  0.2275;
const float Yzeta =  0.3414;
const float Yxsi  = -0.02988;
const float Lv    = -0.1036;
const float Lp    = -0.256;
const float Lr    =  0.0742;
const float Lzeta =  0.02863;
const float Lxsi  = -0.2357;
const float Nv    =  0.1595;
const float Np    = -0.01823;
const float Nr    = -0.0866;
const float Nzeta = -0.120;
const float Nxsi  =  0.0051;
*************/

int trim_model( int mode, double  height, double  North, double  East,
double lat, double lon,
float  speed,  float  psi, float h_dot,
INPUTS *u, STATES *x,  OUTPUTS *y, ATMOS *atmos, float *FM);

#ifdef FDL_HOME
void KFC150_exec_FDL( float theta,
#else
void KFC150_exec( float theta,
#endif
                  float phi,
                  float psi,
                  float alt,
                  float hdg_err,
                  float cdi,
                  float gs_dev,
                  float time,
                  float *pitch_servo,
                  float *roll_servo,
                  float *pitch_trim,
                  float *fd_pitch,
                  float *fd_roll,
                  int buttons[],
                  int lamps[],
                  int *fd_visible,
                  int *pitch_trim_req,
                  int power_switch );

@


1.4
log
@Changed alpha_s_t and alpha_stall_warn.
@
text
@d22 1
a22 1
//$Header: C:/master/seneca/FLTDY/RCS/fms_s3fm.h 1.3 2000/05/24 15:42:15 colinj Exp colinj $
d24 3
d43 1
a43 1
const float k_N_delta_MAP_dot = 2.0e3;
d84 1
a84 1
const float auth_pitch_trim  = (float) 0.80; //0.50;
d145 3
@


1.3
log
@Updates from Martin Kellet.
@
text
@d22 1
a22 1
//$Header: C:/master/seneca/FLTDY/RCS/fms_s3fm.h 1.2 2000/05/10 09:41:57 colinj Exp colinj $
d24 3
d270 1
a270 1
const float alpha_s_t = (float) -0.15;//-0.2; //0.0;
d274 2
a275 1
const float alpha_stall_warn = (float) (14.0 / 57.3); // stall warning alpha
@


1.2
log
@Updates from Martin K.
@
text
@d22 1
a22 1
//$Header: C:/MASTER/SENECA/FLTDY/RCS/fms_s3fm.h 1.1 2000/04/17 11:12:22 colinj Exp $
d24 3
d33 26
d62 2
a63 3
const float KCd_phug = (float) 0.001;

// Airframe icing
d80 1
d82 1
a82 1
const float auth_roll_servo  = (float) 0.25; //0.10;
d119 1
a119 1
const float eng_shutdown_rate_limit = (float) 250.0; // rad/s per second
d125 1
a125 1
const float idle_passage  = (float) 0.2; //0.06; //0.005; //0.01;
d128 1
a128 1
const float max_boost     = (float) 26.0; //1.28*17.0;
d140 2
a141 2
const float EGT_0        = (float) +1100.0;//+900.0; // EGT nominal temp (deg F)
const float EGT_man_P    = (float)   +10.0;//+15.0; // EGT/manifold pressure relation
d160 1
a160 1
const float t_OilP        = (float)     25.0; // OilP time constant
d170 1
a170 1
const float beta_lo   = (float) (12.0*D2R);
a172 1
const float dCTdbeta  = (float) 0.06; //0.12;
d176 3
a178 2
const float CQ0       = (float) 0.0015; //0.0025; //0.005; //0.0025;
const float kCQ       = (float) 3.0; //5.0;//7.5; //10.0; //25.0; //2.78;
d198 1
a198 1
const float NG_k = (float) 80.0e3; //40.0e3; //20.0e3;
d202 1
a202 1
const float NG_b = (float)  20.0e3; //8.5e3;
d218 1
a218 1
const float NG_xl = (float) 2.0;
d257 2
a258 2
const float dCd_cowl_flap = (float) 0.005;
const float dCd_gear = (float) 0.02;
a260 4
//const float Cd0   =  0.10; //0.040; //0.028;
//const float k     =  0.070; //0.04;
const float Cd0   = (float)  0.015;//0.020; //0.030; // from Roskam Cessna 310
const float k     = (float)  0.02; //0.040; // 0.055; // from Roskam Cessna 310
d262 1
a262 1
const float alpha_0_w = (float) -0.05;
d267 5
a271 2
const float alpha_s_t = (float) -0.2; //0.0;
const float alpha_stall = (float) (15.0 / 57.3);
d279 1
a279 1
const float Lv    = (float) -0.2;//-1.0;//-0.5;
d281 1
a281 1
const float Nv    = (float)  0.5;
d291 1
a291 1
const float Nzeta = (float) -0.06;
d338 3
d342 1
@


1.1
log
@Initial revision
@
text
@d22 5
a26 2
//$Header: $
//$Log: $
d30 5
d48 1
a48 1
const float auth_pitch_trim_aero  = (float) 0.10;
d50 2
a51 2
const float auth_pitch_trim  = (float) 0.50;
const float auth_rudd_trim   = (float) 0.20;
d70 1
a70 1
const float ap_Kpsi_p    = (float) 2.0; // 2.0;
d88 5
a92 1
const float k_f_bhp_mix_rich   = (float) -1.0;
d96 1
a96 1
const float idle_passage  = (float) 0.005; //0.01;
d98 1
a98 1
const float t_rpm         = (float) 0.666; //3.33;
d108 2
a109 1
const float K_mag_fail   = (float)     0.92; // mag fail bhp scale factor
d111 2
a112 2
const float EGT_0        = (float) +900.0; // EGT nominal temp (deg F)
const float EGT_man_P    = (float)   +15.0; // EGT/manifold pressure relation
d132 1
a132 1
const float OilP_0        = (float)    +20.0; // OilP nominal pressure (psi)
d137 1
a137 1
const float Qstart    = (float) 100.0; // starter motor torque
d141 1
a141 1
const float beta_lo   = (float) ( 5.0*D2R);
d143 1
a143 1
const float rpm_lo    = (float) (1500.0 * (2*PI)/60.0);
d145 1
d148 2
a149 2
const float CQ0       = (float) 0.0025; //0.005; //0.0025;
const float kCQ       = (float) 10.0; //25.0; //2.78;
d158 1
a158 1
const float K_nose_steer    = (float)  0.25;
d169 7
a175 7
const float NG_k = (float) 20.0e3;
const float RG_k = (float) 10.0e3;
const float LG_k = (float) 10.0e3;

const float NG_b = (float)  8.5e3;
const float RG_b = (float)  7.0e3;
const float LG_b = (float)  7.0e3;
d213 1
a213 1
const float y_eng = (float) 1.90;
d234 2
a235 2
const float Cd0   = (float)  0.030; // from Roskam Cessna 310
const float k     = (float)  0.055; // from Roskam Cessna 310
d237 1
a237 1
const float alpha_0_w = (float) -0.0436;
d244 1
a244 1

d249 20
a271 2
const float Yzeta = (float)  0.3414;
const float Yxsi  = (float) -0.02988;
a274 2
const float Lzeta = (float)  0.02863;
const float Lxsi  = (float) -0.2357;
d278 7
a284 2
const float Nzeta = (float) -0.120;
const float Nxsi  = (float)  0.0051;
d308 1
a308 1
INPUTS *u, STATES *x,  OUTPUTS *y, ATMOS *atmos);
d310 7
a316 3
void KFC150_exec( float theta, float phi, float psi, float alt,
                  float hdg_bug,
                  float loc_hdg, float loc_dev, float gld_dev,
d318 10
a327 2
                  float *pitch_servo, float *roll_servo, float *pitch_trim, float *fd_pitch, float *fd_roll,
                  int buttons[], int lamps[], int *fd_visible, int *pitch_trim_req);
@
