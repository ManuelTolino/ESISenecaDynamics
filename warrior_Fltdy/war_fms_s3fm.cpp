/*****************************************************************************
 **                                                                         **
 ** File        : FMS_S3FM.CPP                                              **
 **                                                                         **
 ** Description : Piper Seneca 3 model, model-specific functions            **
 **               required to integrate with generic flight model structure **
 **                                                                         **
 ** Created     : 29 Jan 2000, M G Kellett                                  **
 **                                                                         **
 **                                                                         **
 **                                          (c) Flight Dynamics Ltd, 2000  **
 *****************************************************************************/

//**************************
//   RCS Markers           *
//**************************

//$Header: C:/MASTER/seneca/FLTDY/RCS/fms_s3fm.cpp 1.5 2000/06/13 15:10:16 COLINJ Exp $
//$Log: fms_s3fm.cpp $
//Revision 1.5  2000/06/13 15:10:16  COLINJ
//added nose gear extension failure.
//resets use defines and not preset numbers.
//extra contact flags set.
//new tailplane angle included.
//new mag drop code.
//airspeed indicator problem fixed.
//
//Revision 1.4  2000/05/31 16:05:42  colinj
//Changed condition for setting Y_D_STALL to TRUE.
//
//Revision 1.3  2000/05/24 15:37:57  colinj
//Updates from Martin Kellet.
//
//Revision 1.2  2000/05/10 09:41:57  colinj
//Updates from Martin K.
//
//Revision 1.1  2000/04/17 10:53:37  colinj
//Initial revision
//

/*---------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <ctype.h>

#include "war_fmgdefs.h"
#include "war_fmg.h"
#include "war_fms_s3fm.h"
#include "war_fms_defs.h"

#define TARGET_POWER 0.8f
//temp made global

extern float delta_time;
extern float nose_press;

static float wind_factor2 = 0.0f;
static float wind_airspd[3]  = {0.0f,90.0f,200.0f};
static float wind_percent[3] = {0.0f, 1.0f,  1.0f};
static float nose_com[3] =  {100.0f,20.0f,4.0f};
static float wind_scale[3]= {10.0f,10.0f,12.0f};
static float h0    = 0.50f; // 0.25; // W/B aero centre position (Fraction of MAC)
static float egt_boost[3] = {0.0f,650.0f,0.0f};
static float rpm_egt[3]   = {1500.0f,2000.0f,2350.0f};
static float ail_auth[4]  = {0.0f, 7.5f, 35.5f, 45.5f};
static float power_boost[4]  = {1.0f, 1.0f, 5.5f, 15.5f};
static float ail_speed[4] = {0.0f,66.0f,140.0f,200.0f};//{200.0f,140.0f,66.0f,0.0f};
static float ail_max  = 8.5f; // NB POST CAA MOD
static float dem_k_dalpha[3] = {-40.0/(57.3*20.0e3),-40.0/(57.3*20.0e3),-20.0/(57.3*20.0e3)};
static float map_value[3]    = {0.0f,20.0f,40.0f};
static float k_dalpha_stall_T = 0.0f;//(float)(-40.0/(57.3*20.0e3));
static float max_flap_up_speed = 10.0f;
static float max_flap_dn_speed = 10.0f;
static float flap_uprate[3] = { 10.0f,  12.5f,  15.5f};
static float flap_dnrate[3] = { 10.0f,   7.5f,   5.5f};
static float flap_airspd[3] = {  0.0f, 100.0f, 200.0f};
static float extra_egt = 0.0f;
static float TpropR,QpropL,QengL;
static float EGTdem,CHTdem,OilTdem,OilPdem;
static float TpropL,QpropR,QengR,mixture,passage,f_bhp_mix,f_egt_mix;
static float max_alpha = (70.0f *D2R);
static float map_fix = 16.0f;//correct Very low map gague reading
//temp debug
static float tprop_scale = 1.0f;
static float y_eng = 0.0f;
static float lwlift = 0.0f,rwlift = 0.0f, tlift = 0.0f;
static float col_alpha = 0.0f;
static float col_al = 0.0f;
static float col_ar = 0.0f;
static float col_at = 0.0f;
//static float alpha_stall_0    = (float) (18.5 / 57.3);
static float alpha_stall_0    = (float) (60.5 / 57.3);
static float alpha_stall_warn = (float) (13.5 / 57.3);
static float pump_scale_l = 1.0f;
static float pump_scale_r = 1.0f;
static float max_MAPl = 40.0f; // max power developed here
static float max_MAPr = 40.0f; // max power developed here
static float wind_factor = 0.0f;
static float wind_factor1 = 0.0f;
//end 
// internal use only, so no need to declare anywhere else
void calc_war_instruments(STATES *x, OUTPUTS *y, float alt1, float alt2, float alt3);

//float flap_stages[4] = { 0.0, 10.0,   25.0,  40.0  };

static float flap_deflection[6] = { 0.0, 5.00,  10.00,  17.50,  25.00,   40.00  };
static float dCl_flap[6]        = { 0.0, 0.15,   0.15,   0.30,   0.30,    0.30  };
static float dCd_flap[6]        = { 0.0, 0.005,  0.005,  0.010,  0.010,   0.020 };
static float dCM_flap[6]        = { 0.0, 0.010, -0.020, -0.010, -0.040,  -0.040 };
static float dSt_flap[6]        = { 0.0,-0.25,   -0.5,   -0.75,   -1.0,    -1.25   }; // stall alpha change
//float dSt_flap[6]        = { 0.0,-1.0,   -2.0,   -3.0,   -4.0,    -5.0   }; // stall alpha change

//float dCd_flap[5]        = { 0.00, 0.01,  0.01,   0.02,  0.03 };
//float dCl_flap[5]        = { 0.00, 0.08,  0.08,   0.12,  0.15 };
//float flap_stages[4] = { 0.0, 10.0,   25.0,  40.0  };
//float dCl_flap[4]    = { 0.0,  0.08,   0.12,  0.15 };
//float dCd_flap[4]    = { 0.0,  0.01,   0.02,  0.05 };
//float dCM_flap[4]    = { 0.0, -0.05,  -0.10, -0.15 };


//float dCl_flap[4]    = { 0.0,  0.20,   0.40,  0.50  };
//float dCM_flap[4]    = { 0.0, -0.005, -0.005,-0.005 };

void calc_war_FM(INPUTS *u, STATES *x, OUTPUTS *y, ATMOS *atmos, float *FM, STATES *dx, MASS *mass)
{
float FM_aero[6],FM_eng[6],FM_gr[6];
float w_alpha = 0.0;
int i,contact;
GEAR gear;
float ftemp;
float Yaero,Laero,Naero;
float xsi,eta,zeta; /* aileron, elevator, rudder deflections (rad) */
float xsi_dem,eta_dem,zeta_dem;
float Cl_w_l, L_w_l, Cd_w_l, D_w_l;
float Cl_w_r, L_w_r, Cd_w_r, D_w_r;
float Cl_t, Cd_t, L_t, D_t, alpha_t;
float alpha_l,alpha_r;
float h;
float Vi,vi,vic,TL,TR,betaL,betaR;
float wN,wE,wD,Vu,Vv,Vw,Vtas;
float qbar;
float flap_time;
int flap_select;
float ffL,ffR,throt_scale,ff_dem,ff_prime;
float Cl_flap,Cd_flap,CM_flap;
float Cd_gear,CM_gear,Cd_phug;
float T,p,rho,amach,atm_theta,atm_delta,atm_sigma;
float CT,CQ,rpm_dem,man_P_dem,bhp,lambda;
float thrust_scale = 1.0f;
//float TpropR,QpropL,QengL;
//float EGTdem,CHTdem,OilTdem,OilPdem;
//float TpropL,QpropR,QengR,mixture,passage,f_bhp_mix,f_egt_mix;
int kfc150_buttons[16],kfc150_lamps[16];
float kfc150_pitch_servo,kfc150_roll_servo,kfc150_pitch_trim,kfc150_fd_pitch,kfc150_fd_roll;
int fd_visible,pitch_trim_req;
int n_mags;
int ngear_fail = FALSE;
float vap_ret_l,vap_ret_r;
float l_mix_lever,r_mix_lever;
float TpropW,QpropW;
float f_bhp_lo;
float Vt_l,Vt_r; // propwash velocities
float steer_dem;
float map_gauge_dem,map_gauge_dem1;
float alpha_stall_flap;
float Vinertial, flying_factor, ground_factor; // factor which is 1.0 when flying (<50 kn inertial), and
// washes out to 0.0 below that.
float dalpha_stall_T;

float vsi_gauge_dem;
float low_throttle_factor;


	for(i=0;i<N_USER_STATES;i++) dx->user[i] = 0.0;

    Vinertial = sqrt(x->u*x->u + x->v*x->v + x->w*x->w);
    flying_factor = Vinertial/(50.0*KN2MPS);
    if(flying_factor > 1.0) flying_factor = 1.0;
    ground_factor = 1.0-flying_factor;

    //scale aileron authority with airspeed
    ail_max = table1D_war(ail_speed,ail_auth,4,y->user[Y_A_P1_ASI]);
    //thrust_scale = table1D_war(ail_speed,power_boost,4,y->user[Y_A_P1_ASI]);
    thrust_scale = (thrust_scale * ( u->inps[U_A_L_MIX] * 30.0f + u->inps[U_A_R_MIX] * 30.0f));
    if(thrust_scale < 1.0f)
       thrust_scale = 1.0f;



    u->inps[U_A_R_MIX] = 1.0f;
    u->inps[U_A_L_MIX] = 1.0f;
/** Calculate mass & inertias to pass to rigid body dynamics **/
    mass->mass = u->inps[U_A_MASS] * LBS2KG + dW_ice*x->user[X_ICE];
    mass->Ixx  = kx*kx * mass->mass;
    mass->Iyy  = 6.0 * ky*ky * mass->mass;
//    mass->Iyy  = 3.0 * ky*ky * mass->mass;
    mass->Izz  = kz*kz * mass->mass;
    mass->Ixz  = 0.10 * mass->Izz;

// wind boundary layer factor
    wind_factor1 = (-x->D-atmos->ht-2.0)/20.0;
    if (wind_factor1 < 0.0) wind_factor1 = 0.0;
    if (wind_factor1 > 1.0) wind_factor1 = 1.0;

    wind_factor1 = wind_factor1*(2.0f/3.0f);

    //introduce wind as nose wheel goes light

//    wind_factor = table1D_war(nose_com,wind_scale,3,nose_press*100.0f);
//    wind_factor -= 10.0f;
    wind_factor = 0.0f - nose_press*5.0f;
    if (wind_factor < 0.0) wind_factor = 0.0;
    if (wind_factor > 1.0) wind_factor = 1.0;

    wind_factor = wind_factor*(1.0f/3.0f);

    wind_factor += wind_factor1;
    if (wind_factor < 0.0) wind_factor = 0.0;
    if (wind_factor > 1.0) wind_factor = 1.0;
//    wind_factor = 1.0f;

	Vu = x->u + wind_factor*atmos->wN * y->S[0][0] + wind_factor*atmos->wE * y->S[0][1] + wind_factor*atmos->wD * y->S[0][2];
	Vv = x->v + wind_factor*atmos->wN * y->S[1][0] + wind_factor*atmos->wE * y->S[1][1] + wind_factor*atmos->wD * y->S[1][2];
	Vw = x->w + wind_factor*atmos->wN * y->S[2][0] + wind_factor*atmos->wE * y->S[2][1] + wind_factor*atmos->wD * y->S[2][2];

   if(fabs(Vu) > 1.0e-4)
   {
      w_alpha = atan2(Vw,Vu);
   }
   else
   {
      if(Vw > 0.0)
      {
         w_alpha = +90.0 * D2R;
      }
      else
      {
         w_alpha = -90.0 * D2R;
      }
   }

	Vtas = sqrt(Vu*Vu + Vv*Vv + Vw*Vw);
	y->Vtas = Vtas;
    calc_war_ISA(atmos->delta_T,atmos->delta_p,atmos->delta_r,-x->D,
             &T,&p,&rho,&amach,&atm_theta,&atm_delta,&atm_sigma);

    qbar = 0.5 * rho * Vtas * Vtas;
// Make pressures available to instruments

// Normal conditions
    if(u->dinps[U_D_S1_BLOCKAGE]!=0)
	  dx->user[X_P_S1] = 0.0;
    else
	  dx->user[X_P_S1] = (p - x->user[X_P_S1]) / 1.0;

    if(u->dinps[U_D_S2_BLOCKAGE]!=0)
	  dx->user[X_P_S2] = 0.0;
    else
	  dx->user[X_P_S2] = (p - x->user[X_P_S2]) / 1.0;

// Copy rate of change of pressure to outputs
	y->user[Y_A_DS1DT] = dx->user[X_P_S1];
	y->user[Y_A_DS2DT] = dx->user[X_P_S2];

// Rate of climb (ft/min) Pressure version
	vsi_gauge_dem  =  - 60.0 * y->user[Y_A_DS1DT] * 3.28 / 11.95;
   dx->user[X_L_VSI_GAUGE]   = (vsi_gauge_dem-x->user[X_L_VSI_GAUGE])/1.5;
   dx->user[X_R_VSI_GAUGE]   = (vsi_gauge_dem-x->user[X_R_VSI_GAUGE])/1.5;
   y->user[Y_A_P1_VSI_GAUGE] = x->user[X_L_VSI_GAUGE];
   y->user[Y_A_P2_VSI_GAUGE] = x->user[X_R_VSI_GAUGE];
   //	vsi_gauge_dem  =  - 60.0 * y->user[Y_A_DS2DT] * 3.28 / 11.95;


// Update pitot pressures (if no blockages)
    if(u->dinps[U_D_P_BLOCKAGE]==FALSE)
      y->user[Y_A_PITOT] = p + qbar;  // P1 pitot pressure
    y->user[Y_A_QBAR]  = qbar/3840.0;
// Execute autopilot code
    kfc150_buttons[KFC150_B_FD]     =  u->dinps[U_D_KFC150_FD];
    kfc150_buttons[KFC150_B_ALT]    =  u->dinps[U_D_KFC150_ALT];
    kfc150_buttons[KFC150_B_HDG]    =  u->dinps[U_D_KFC150_HDG];
    kfc150_buttons[KFC150_B_NAV]    =  u->dinps[U_D_KFC150_NAV];
    kfc150_buttons[KFC150_B_APR]    =  u->dinps[U_D_KFC150_APR];
    kfc150_buttons[KFC150_B_BC]     =  u->dinps[U_D_KFC150_BC];
    kfc150_buttons[KFC150_B_TEST]   =  u->dinps[U_D_KFC150_TEST];
    kfc150_buttons[KFC150_B_AP_ENG] =  u->dinps[U_D_KFC150_AP_ENG];
    kfc150_buttons[KFC150_B_VT]     =  u->dinps[U_D_KFC150_VT];
    kfc150_buttons[KFC150_B_AP_DIS] =  u->dinps[U_D_AP_DISC];
    kfc150_buttons[KFC150_B_CWS]    =  u->dinps[U_D_CWS];

#ifdef FDL_HOME
    KFC150_exec_FDL(y->theta,
#else
    KFC150_exec(y->theta,
#endif
                y->phi,
                y->psi,
                -x->D,
                (float)u->inps[U_A_HDG_ERR]*D2R,
                u->inps[U_A_CDI],
                u->inps[U_A_GS_DEV],
                x->user[X_SIM_TIME],
                u->inps[U_A_HDG],
                &kfc150_pitch_servo,
                &kfc150_roll_servo,
                &kfc150_pitch_trim,
                &kfc150_fd_pitch,
                &kfc150_fd_roll,
                kfc150_buttons,
                kfc150_lamps,
                &fd_visible,
                &pitch_trim_req,
                u->dinps[U_D_KFC150_PWR]);
    y->duser[Y_D_FD_VISIBLE] = fd_visible;
    y->duser[Y_D_KFC150_FD]  = kfc150_lamps[KFC150_L_FD];
    y->duser[Y_D_KFC150_ALT] = kfc150_lamps[KFC150_L_ALT];
    y->duser[Y_D_KFC150_HDG] = kfc150_lamps[KFC150_L_HDG];
    y->duser[Y_D_KFC150_GS]  = kfc150_lamps[KFC150_L_GS];
    y->duser[Y_D_KFC150_NAV] = kfc150_lamps[KFC150_L_NAV];
    y->duser[Y_D_KFC150_APR] = kfc150_lamps[KFC150_L_APR];
    y->duser[Y_D_KFC150_BC]  = kfc150_lamps[KFC150_L_BC];
    y->duser[Y_D_KFC150_TRIM]= kfc150_lamps[KFC150_L_TRIM];
    y->duser[Y_D_KFC150_AP]  = kfc150_lamps[KFC150_L_AP];
    y->duser[Y_D_PITCH_TRIM_REQ] = pitch_trim_req;
    y->user[Y_A_FD_ROLL]     = kfc150_fd_roll;
    y->user[Y_A_FD_PITCH]    = kfc150_fd_pitch;
    y->user[Y_A_ROLL_SERVO]     = kfc150_roll_servo;
    y->user[Y_A_PITCH_SERVO]    = kfc150_pitch_servo;



/** Undercarriage forces & moments **/

	if (1)  //(x->user[X_N_GEAR] > 0.95)
	{
		gear.x[0] = NG_x; gear.y[0] = NG_y; gear.xl[0]      = NG_xl;
		gear.k[0] = NG_k; gear.b[0] = NG_b; gear.preload[0] = NG_preload;

		gear.x[2] = RG_x; gear.y[2] = RG_y; gear.xl[2]      = RG_xl;
		gear.k[2] = RG_k; gear.b[2] = RG_b; gear.preload[2] = RG_preload;

		gear.x[1] = LG_x; gear.y[1] = LG_y; gear.xl[1]      = LG_xl;
		gear.k[1] = LG_k; gear.b[1] = LG_b; gear.preload[1] = LG_preload;

        steer_dem = -K_nose_steer*u->inps[U_A_PED];
        dx->user[X_STEER] = (steer_dem - x->user[X_STEER])/0.50;

//CJ if nose gear fail is true and gear not down then set flag to lower 
//nose terrain ht in calc gear
      if(u->dinps[U_D_NOSE_GEAR_EXTEND_FAIL] == TRUE && x->user[X_N_GEAR] < 0.01)
      {
         ngear_fail = TRUE;
      }
      else
         ngear_fail = FALSE;

		contact = calc_war_gear(&gear,x,y,-x->D-atmos->ht,ground_factor*x->user[X_STEER],u->inps[U_A_L_BRAKE],u->inps[U_A_R_BRAKE],FM_gr);
//		y->duser[Y_D_N_CONTACT] = u->dinps[U_D_GEAR] && ((contact     )& 1);
//		y->duser[Y_D_R_CONTACT] = u->dinps[U_D_GEAR] && ((contact >> 2)& 1);
//		y->duser[Y_D_L_CONTACT] = u->dinps[U_D_GEAR] && ((contact >> 1)& 1);
//NEXT THREE LINES ADDED BY CJ TO DETECT "WHEELS UP" LANDING
		y->duser[Y_D_N_CONTACT_ACTUAL] =/* u->dinps[U_D_GEAR] &&*/ ((contact     )& 1);
		y->duser[Y_D_R_CONTACT_ACTUAL] =/* u->dinps[U_D_GEAR] && */((contact >> 2)& 1);
		y->duser[Y_D_L_CONTACT_ACTUAL] =/* u->dinps[U_D_GEAR] && */((contact >> 1)& 1);

		y->duser[Y_D_N_CONTACT] = (x->user[X_N_GEAR]>0.5) && ((contact     )& 1);
		y->duser[Y_D_R_CONTACT] = (x->user[X_R_GEAR]>0.5) && ((contact >> 2)& 1);
		y->duser[Y_D_L_CONTACT] = (x->user[X_L_GEAR]>0.5) && ((contact >> 1)& 1);
	}
	else
	{
        dx->user[X_STEER] = (0.0 - x->user[X_STEER])/1.0;
		FM_gr[0] = FM_gr[1] = FM_gr[2] =
		FM_gr[4] = FM_gr[5] = FM_gr[5] = 0.0;
		y->duser[Y_D_N_CONTACT] = 1;
		y->duser[Y_D_R_CONTACT] = 1;
		y->duser[Y_D_L_CONTACT] = 1;
	}

// Propulsion forces & moments

// Left Engine

    passage = idle_passage+(1.0-idle_passage)*u->inps[U_A_L_THROT];
//    passage = idle_passage+(1.0-idle_passage)*
//    (0.5*u->inps[U_A_L_THROT]*u->inps[U_A_L_THROT] + (1.0-0.5)*u->inps[U_A_L_THROT]);

    passage = (1.0 - cos(passage*PI/2.0));  // orifice is nonlinear

    dx->user[X_L_MAN_P] = passage*((p*PA2INHG+x->user[X_L_BOOST])-x->user[X_L_MAN_P])/(t_throt) -
    (x->user[X_L_RPM]/max_rpm)*x->user[X_L_MAN_P]/(t_rpm);

// no fuel left is same as full lean mixture
    max_MAPl = 40.0f; 

    if( ( (x->user[X_L_FUEL]*(u->dinps[U_D_L_XFEED]==0)) +
          (x->user[X_R_FUEL]*(u->dinps[U_D_L_XFEED]==2)) ) > 0.0)
    {
       if(u->dinps[U_D_LEFT_FUEL_PUMP_FAIL])
       {
          pump_scale_l = 0.0f;
          if(u->dinps[U_D_L_BOOST])
          {
             pump_scale_l = 1.0f;
             max_MAPl = 46.0f; 
          }
          else
          if(u->dinps[U_D_LOW_L_BOOST])
          {
             pump_scale_l = 1.0f;
             max_MAPl = 76.0f; 
          }
       }
       else
          pump_scale_l = 1.0f;

       l_mix_lever = u->inps[U_A_L_MIX] * pump_scale_l;
    }
    else
      l_mix_lever=0.0;

//    mixture = 1.50 * l_mix_lever   *
//              sqrt(passage) *
//              40.0/x->user[X_L_MAN_P];
    mixture = l_mix_lever * 1.20 * (1.0+(-x->D/10000.0));

    if(mixture<1.0)
      f_bhp_mix = 1.0 - (mixture-1.0)*(mixture-1.0);
    else if (mixture<f_mix_bhp_rich_max)
      f_bhp_mix = 1.0;
    else
      f_bhp_mix = 1.0+(mixture-f_mix_bhp_rich_max)*k_f_bhp_mix_rich;

    if (f_bhp_mix < 0.0) f_bhp_mix = 0.0;

    n_mags = u->dinps[U_D_L_MAG_L]*(u->dinps[U_D_FAIL_L_MAG_L]==FALSE) +
             u->dinps[U_D_L_MAG_R]*(u->dinps[U_D_FAIL_L_MAG_R]==FALSE);

// factor for low speed/MAP running (straight line bhp laws don't apply
// at low rpm or MAP)
    f_bhp_lo = (x->user[X_L_RPM]/(0.4*max_rpm))*(x->user[X_L_MAN_P]/(0.4*max_MAPl));
    if(f_bhp_lo > 1.0) f_bhp_lo = 1.0;
    if(f_bhp_lo < 0.1) f_bhp_lo = 0.1;
//f_bhp_lo=1.0;
    switch(n_mags)
    {
      case 0:
        bhp = 0.0;
        break;
      case 1:
        bhp = f_bhp_lo * f_bhp_mix * K_mag_fail * max_bhp * (x->user[X_L_MAN_P]/max_MAPl) * (x->user[X_L_RPM]/max_rpm);
        //bhp -= bhp_mag_fail;//make left and right mag drops different
        if(u->dinps[U_D_L_MAG_L]*(u->dinps[U_D_FAIL_L_MAG_L]==FALSE))
        {
		     bhp -= (bhp_mag_fail + ((60.0*x->user[X_L_RPM]/(2*PI)*l_bhp_factor)));
        }
        else
           bhp -= (bhp_mag_fail + ((60.0*x->user[X_L_RPM]/(2*PI)*r_bhp_factor)));
        break;
      case 2:
        bhp = f_bhp_lo * f_bhp_mix * max_bhp * (x->user[X_L_MAN_P]/max_MAPl) * (x->user[X_L_RPM]/max_rpm);
        break;

      default:
         bhp = 0.0f;
         break;

    }

    if (bhp > 1.1*max_bhp) bhp = 1.1*max_bhp;

    if (bhp > 1.0)
      y->duser[Y_D_L_ENGINE_LIT]=TRUE;
    else
      y->duser[Y_D_L_ENGINE_LIT]=FALSE;

    //bhp = bhp * pump_scale_l;

    dx->user[X_L_BOOST]=(bhp*(max_boost/max_bhp)-x->user[X_L_BOOST])/t_boost;

    y->user[Y_A_L_MIXTURE]=mixture;
    y->user[Y_A_L_BHP]=bhp;

    if(u->inps[U_A_L_RPM] > 0.1 && y->user[Y_A_P1_ASI] > 50.0f && (x->user[X_L_RPM]<100.0))// && y->duser[Y_D_L_ENGINE_LIT] == FALSE)
    {
       u->dinps[U_D_START]= -1;
    }

    QengL = 4745.7*bhp/(x->user[X_L_RPM]+1.0);
    if (x->user[X_L_RPM]<eng_min_speed) QengL = 0.0;
    if ( (u->dinps[U_D_START]==-1) && (x->user[X_L_RPM]<100.0) ) QengL += Qstart;



/**
    lambda=atan2(Vu,(x->user[X_L_RPM]+0.01)*prop_rad);
    CT = dCTdbeta*(x->user[X_L_BLADE]-Vu/(x->user[X_L_RPM]*prop_rad + 1.0));
    CQ = CQ0 + dCQ_ice*x->user[X_L_PROP_ICE] + kCQ*CT*CT;
    if (CT>CTmax) CT=CTmax-(CT-CTmax);
    if (CT<0.0) CT=0.0;
    TpropL = 0.5 * rho * x->user[X_L_RPM]*prop_rad*x->user[X_L_RPM]*prop_rad* prop_area*CT;
    QpropL = TpropL*sin(lambda)*prop_rad + 0.5 * rho * x->user[X_L_RPM]*prop_rad*x->user[X_L_RPM]*prop_rad* prop_area*prop_rad*CQ;
**/

    lambda = atan2(Vu, (x->user[X_L_RPM]+0.01)*prop_rad);
    CT = dCTdbeta*(x->user[X_L_BLADE]-lambda);
//    CQ = (CQ0 + dCQ_ice*x->user[X_L_PROP_ICE] + kCQ*CT*CT);
    CQ = (0.0015*ground_factor + CQ0 + dCQ_ice*x->user[X_L_PROP_ICE] + kCQ*CT*CT);
// simple blade stall model
    if (CT >  CTmax) CT =   CTmax-(CT-CTmax);
    if (CT < -CTmax) CT = -(CTmax-(-CT-CTmax));
    //if (CT<0.0) CT=0.0;
// wind axes
    TpropW = 0.5 * rho * x->user[X_L_RPM]*prop_rad*x->user[X_L_RPM]*prop_rad* prop_area*CT;
    QpropW = 0.5 * rho * x->user[X_L_RPM]*prop_rad*x->user[X_L_RPM]*prop_rad* prop_area*prop_rad*CQ;
// prop axes
    TpropL = TpropW*cos(lambda)-QpropW*sin(lambda)/prop_rad;
//    QpropL = QpropW*cos(lambda)+0.70*TpropW*sin(lambda)*prop_rad;
//    QpropL = QpropW*cos(lambda)+0.55*TpropW*sin(lambda)*prop_rad;
    QpropL = QpropW*cos(lambda)+0.60*TpropW*sin(lambda)*prop_rad;

    dx->user[X_L_RPM]= (QengL-QpropL)/Jprop;
    rpm_dem = rpm_lo + (rpm_hi-rpm_lo)*u->inps[U_A_L_RPM];

    if(u->inps[U_A_L_RPM]<0.1)
      dx->user[X_L_BLADE] = 0.25*(beta_hi-x->user[X_L_BLADE]);
    else
      dx->user[X_L_BLADE] = -0.05*(rpm_dem-x->user[X_L_RPM]) + 0.01*dx->user[X_L_RPM];

// if(x->user[X_L_RPM]<800.0*RPM2RPS) dx->user[X_L_BLADE]=0.0;

    if(x->user[X_L_BLADE]>(beta_hi-0.02))
    {
      if(dx->user[X_L_BLADE] > 0.5*(beta_hi-x->user[X_L_BLADE]))
        dx->user[X_L_BLADE] = 0.5*(beta_hi-x->user[X_L_BLADE]);
    }
    if(x->user[X_L_BLADE]<(beta_lo+0.02))
    {
      if(dx->user[X_L_BLADE] < -0.5*(-beta_lo+x->user[X_L_BLADE]))
        dx->user[X_L_BLADE] = -0.5*(-beta_lo+x->user[X_L_BLADE]);
    }

// prop control failure = runaway down to fine pitch
      if(u->dinps[U_D_FAIL_L_PROP])
        dx->user[X_L_BLADE] = -0.5*(-beta_lo+x->user[X_L_BLADE]);


//    f_egt_mix = k_egt_mix*fabs(mixture-1.0);
//    EGTdem   = 60.0 + k_egt_bhp*bhp + f_egt_mix;

//    EGTdem = 1500.0 - fabs(1.0-mixture)*500.0;
    //EGTdem = 1600.0 - fabs(1.0-mixture)*400.0 - 50.0*(n_mags==1);
      extra_egt = table1D_war(rpm_egt,egt_boost,3,y->user[Y_A_L_RPM]);

      EGTdem = 1550.0 - fabs(1.0-mixture)*400.0 + 50.0*(n_mags==1);
      //EGTdem += 650.0f;
      EGTdem += extra_egt;
    if (EGTdem<60.0) EGTdem = 60.0;
    if(bhp<100.0) EGTdem *= (bhp/100.0);
    dx->user[X_L_EGT] = (EGTdem - x->user[X_L_EGT])/t_EGT;

//    if(x->user[X_L_RPM]>(500.0/9.549))
//      CHTdem = CHT_0 + CHT_man_P*x->user[X_L_MAN_P] + (CHT_Vu+dCHT_Vu_cowl*u->inps[U_A_L_COWL])*(Vu+d_Vu_CHT);
//    else
//      CHTdem = 0.0;
    CHTdem=200.0+bhp*(1.25-0.5*u->inps[U_A_L_COWL]);
    dx->user[X_L_CHT] = (CHTdem - x->user[X_L_CHT])/t_CHT;

    if(x->user[X_L_RPM]>(500.0/9.549))
      OilTdem = OilT_0 + OilT_man_P*x->user[X_L_MAN_P] + (OilT_Vu+dOilT_Vu_cowl*u->inps[U_A_L_COWL])*(Vu+d_Vu_OilT);
    else
      OilTdem = 0.0;
    dx->user[X_L_OIL_T] = (OilTdem - x->user[X_L_OIL_T])/t_OilT;

    if(x->user[X_L_RPM]>(500.0/9.549))
      OilPdem = OilP_0 + OilP_man_P*x->user[X_L_MAN_P];
    else
      OilPdem = 0.0;
    dx->user[X_L_OIL_P] = (OilPdem - x->user[X_L_OIL_P])/t_OilP;

    if( (u->dinps[U_D_L_PRIME]==TRUE) &&
        ( ( (x->user[X_L_FUEL]*(u->dinps[U_D_L_XFEED]==0)) +
            (x->user[X_R_FUEL]*(u->dinps[U_D_L_XFEED]==2)) ) > 0.0) )
    {
      ff_prime = -(primer_max_ff + 0.5*u->inps[U_A_L_THROT]*u->inps[U_A_L_MIX])/3600.0;
    }
    else
      ff_prime = 0.0;

    ff_dem = - 0.75 * bhp*(720e-6*bhp + 0.478) * mixture / (6.0 * 3600.0);

    if (ff_dem < -35.0) ff_dem = -35.0/3600.0;

    if ((-ff_prime) > (-ff_dem)) ff_dem=ff_prime;

    dx->user[X_L_FF]   = (ff_dem-x->user[X_L_FF])/T_ff;

// last action - override all for engine shutdown if both mags off
// or mixture < 10% (& speed low), or rpm low with starter OFF
    if
    (
      ( ((n_mags==0) || (mixture<0.1))  && (Vu<10.0) && (u->dinps[U_D_START]!=-1) ) ||
      ( u->dinps[U_D_L_ENGINE_SEIZE] ) ||
      ( (x->user[X_L_RPM]<eng_min_speed) && (u->dinps[U_D_START]!=-1) )
    )
    dx->user[X_L_RPM] = -x->user[X_L_RPM]/eng_shutdown_t;
	if (dx->user[X_L_RPM] > +eng_shutdown_rate_limit) dx->user[X_L_RPM] = +eng_shutdown_rate_limit;
	if (dx->user[X_L_RPM] < -eng_shutdown_rate_limit) dx->user[X_L_RPM] = -eng_shutdown_rate_limit;

// Right Engine

    passage = idle_passage+(1.0-idle_passage)*u->inps[U_A_R_THROT];
//    passage = idle_passage+(1.0-idle_passage)*
//    (0.5*u->inps[U_A_R_THROT]*u->inps[U_A_R_THROT] + (1.0-0.5)*u->inps[U_A_R_THROT]);
    passage = (1.0 - cos(passage*PI/2.0));  // orifice is nonlinear

    dx->user[X_R_MAN_P] = passage*((p*PA2INHG+x->user[X_R_BOOST])-x->user[X_R_MAN_P])/t_throt -
    (x->user[X_R_RPM]/max_rpm)*x->user[X_R_MAN_P]/t_rpm;

// no fuel left is same as full lean mixture
    max_MAPr = 40.0f; 
    if( ( (x->user[X_R_FUEL]*(u->dinps[U_D_R_XFEED]==0)) +
          (x->user[X_L_FUEL]*(u->dinps[U_D_R_XFEED]==2)) ) > 0.0)
    {
       if(u->dinps[U_D_RIGHT_FUEL_PUMP_FAIL])
       {
          pump_scale_r = 0.0f;
          if(u->dinps[U_D_R_BOOST])
          {
             pump_scale_r = 1.0f;
             max_MAPr = 46.0f; 
          }
          else
          if(u->dinps[U_D_LOW_R_BOOST])
          {
             pump_scale_r = 1.0f;
             max_MAPr = 76.0f; 
          }
       }
       else
          pump_scale_r = 1.0f;

      r_mix_lever = u->inps[U_A_R_MIX] * pump_scale_r;
    }
    else
      r_mix_lever=0.0;

//    mixture = 1.50 * r_mix_lever   *
//              sqrt(passage) *
//              40.0/x->user[X_R_MAN_P];
    mixture = r_mix_lever * 1.20 * (1.0+(-x->D/10000.0));

//    f_bhp_mix = 1.0 - (mixture-1.0)*(mixture-1.0);
//    if (f_bhp_mix < 0.0) f_bhp_mix = 0.0;
    if(mixture<1.0)
      f_bhp_mix = 1.0 - (mixture-1.0)*(mixture-1.0);
    else if (mixture<f_mix_bhp_rich_max)
      f_bhp_mix = 1.0;
    else
      f_bhp_mix = 1.0+(mixture-f_mix_bhp_rich_max)*k_f_bhp_mix_rich;

    if (f_bhp_mix < 0.0) f_bhp_mix = 0.0;

    n_mags = u->dinps[U_D_R_MAG_L]*(u->dinps[U_D_FAIL_R_MAG_L]==FALSE) +
             u->dinps[U_D_R_MAG_R]*(u->dinps[U_D_FAIL_R_MAG_R]==FALSE);

// factor for low speed/MAP running (straight line bhp laws don't apply
// at low rpm or MAP)
    f_bhp_lo = ( x->user[X_R_RPM]   / (0.4*max_rpm) ) *
               ( x->user[X_R_MAN_P] / (0.4*max_MAPr) );
    if(f_bhp_lo > 1.0) f_bhp_lo = 1.0;
    if(f_bhp_lo < 0.1) f_bhp_lo = 0.1;

    switch(n_mags)
    {
      case 0:
        bhp = 0.0;
        break;
      case 1:
        bhp = f_bhp_lo * f_bhp_mix * K_mag_fail * max_bhp * (x->user[X_R_MAN_P]/max_MAPr) * (x->user[X_R_RPM]/max_rpm);
        if(u->dinps[U_D_R_MAG_L]*(u->dinps[U_D_FAIL_R_MAG_L]==FALSE))
        {
           bhp -= (bhp_mag_fail + ((60.0*x->user[X_R_RPM]/(2*PI)*l_bhp_factor)));
        }
        else
           bhp -= (bhp_mag_fail + ((60.0*x->user[X_R_RPM]/(2*PI)*r_bhp_factor)));
        break;
      case 2:
        bhp = f_bhp_lo * f_bhp_mix * max_bhp * (x->user[X_R_MAN_P]/max_MAPr) * (x->user[X_R_RPM]/max_rpm);
        break;
    }

    if (bhp > 1.1*max_bhp) bhp = 1.1*max_bhp;

    if (bhp > 1.0)
      y->duser[Y_D_R_ENGINE_LIT]=TRUE;
    else
      y->duser[Y_D_R_ENGINE_LIT]=FALSE;

    dx->user[X_R_BOOST]=(bhp*(max_boost/max_bhp)-x->user[X_R_BOOST])/t_boost;

    y->user[Y_A_R_MIXTURE]=mixture;
    y->user[Y_A_R_BHP]=bhp;
    QengR = 4745.7*bhp/(x->user[X_R_RPM]+1.0);

    if(u->inps[U_A_R_RPM] > 0.1 && y->user[Y_A_P1_ASI] > 50.0f && (x->user[X_R_RPM]<100.0))// && y->duser[Y_D_L_ENGINE_LIT] == FALSE)
    {
       u->dinps[U_D_START] = 1;
    }

    if (x->user[X_R_RPM]<eng_min_speed) QengR = 0.0;
    if ( (u->dinps[U_D_START]== +1) && (x->user[X_R_RPM]<100.0) ) QengR += Qstart;

    lambda = atan2(Vu, (x->user[X_R_RPM]+0.01)*prop_rad);
//    CT = dCTdbeta*(x->user[X_R_BLADE]-Vu/(x->user[X_R_RPM]*prop_rad + 1.0));
    CT = dCTdbeta*(x->user[X_R_BLADE]-lambda);
//    CQ = (CQ0 + dCQ_ice*x->user[X_R_PROP_ICE] + kCQ*CT*CT);
    CQ = (0.0015*ground_factor + CQ0 + dCQ_ice*x->user[X_R_PROP_ICE] + kCQ*CT*CT);
// simple blade stall model
    if (CT >  CTmax) CT=CTmax-(CT-CTmax);
    if (CT < -CTmax) CT=-(CTmax-(-CT-CTmax));

//    TpropR = 0.5 * rho * x->user[X_R_RPM]*prop_rad*x->user[X_R_RPM]*prop_rad* prop_area*CT;
//    QpropR = TpropR*sin(lambda)*prop_rad +
//    0.5 * rho * x->user[X_R_RPM]*prop_rad*x->user[X_R_RPM]*prop_rad* prop_area*prop_rad*CQ;
// wind axes
    TpropW = 0.5 * rho * x->user[X_R_RPM]*prop_rad*x->user[X_R_RPM]*prop_rad* prop_area*CT;
    QpropW = 0.5 * rho * x->user[X_R_RPM]*prop_rad*x->user[X_R_RPM]*prop_rad* prop_area*prop_rad*CQ;
// prop axes
    TpropR = TpropW*cos(lambda)-QpropW*sin(lambda)/prop_rad;
//    QpropR = QpropW*cos(lambda)+0.70*TpropW*sin(lambda)*prop_rad;
//    QpropR = QpropW*cos(lambda)+0.55*TpropW*sin(lambda)*prop_rad;
    QpropR = QpropW*cos(lambda)+0.60*TpropW*sin(lambda)*prop_rad;

    dx->user[X_R_RPM]= (QengR-QpropR)/Jprop;
    rpm_dem = rpm_lo + (rpm_hi-rpm_lo)*u->inps[U_A_R_RPM];

//    dx->user[X_R_BLADE] = -0.05*(rpm_dem-x->user[X_R_RPM]) + 0.01*dx->user[X_R_RPM];

    if(u->inps[U_A_R_RPM]<0.1)
      dx->user[X_R_BLADE] = 0.25*(beta_hi-x->user[X_R_BLADE]);
    else
      dx->user[X_R_BLADE] = -0.05*(rpm_dem-x->user[X_R_RPM]) + 0.01*dx->user[X_R_RPM];

    //if(x->user[X_R_RPM]<800.0*RPM2RPS) dx->user[X_R_BLADE]=0.0;


    if(x->user[X_R_BLADE]>(beta_hi-0.02))
    {
      if(dx->user[X_R_BLADE] > 0.5*(beta_hi-x->user[X_R_BLADE]))
        dx->user[X_R_BLADE] = 0.5*(beta_hi-x->user[X_R_BLADE]);
    }
    if(x->user[X_R_BLADE]<(beta_lo+0.02))
    {
      if(dx->user[X_R_BLADE] < -0.5*(-beta_lo+x->user[X_R_BLADE]))
        dx->user[X_R_BLADE] = -0.5*(-beta_lo+x->user[X_R_BLADE]);
    }

// prop control failure = runaway down to fine pitch
      if(u->dinps[U_D_FAIL_R_PROP])
        dx->user[X_R_BLADE] = -0.5*(-beta_lo+x->user[X_R_BLADE]);

//    f_egt_mix = k_egt_mix*fabs(mixture-1.0);
//    EGTdem   = 60.0 + k_egt_bhp*bhp + f_egt_mix;
//    if (EGTdem<60.0) EGTdem = 60.0;
//    dx->user[X_R_EGT] = (EGTdem - x->user[X_R_EGT])/t_EGT;

//    EGTdem = 1500.0 - fabs(1.0-mixture)*500.0;
    //EGTdem = 1600.0 - fabs(1.0-mixture)*500.0 + 50.0*(n_mags==1);
      extra_egt = table1D_war(rpm_egt,egt_boost,3,y->user[Y_A_R_RPM]);
      EGTdem = 1550.0 - fabs(1.0-mixture)*400.0 + 50.0*(n_mags==1);

      EGTdem += extra_egt;
    if (EGTdem<60.0) EGTdem = 60.0;
    if(bhp<100.0) EGTdem *= (bhp/100.0);
    dx->user[X_R_EGT] = (EGTdem - x->user[X_R_EGT])/t_EGT;


//    if(x->user[X_R_RPM]>(500.0/9.549))
//      CHTdem = CHT_0 + CHT_man_P*x->user[X_R_MAN_P] + (CHT_Vu+dCHT_Vu_cowl*u->inps[U_A_R_COWL])*(Vu+d_Vu_CHT);
//    else
//      CHTdem = 0.0;

    CHTdem=200.0+bhp*(1.25-0.5*u->inps[U_A_R_COWL]);
    dx->user[X_R_CHT] = (CHTdem - x->user[X_R_CHT])/t_CHT;

    if(x->user[X_R_RPM]>(500.0/9.549))
      OilTdem = OilT_0 + OilT_man_P*x->user[X_R_MAN_P] + (OilT_Vu+dOilT_Vu_cowl*u->inps[U_A_R_COWL])*(Vu+d_Vu_OilT);
    else
      OilTdem = 0.0;
    dx->user[X_R_OIL_T] = (OilTdem - x->user[X_R_OIL_T])/t_OilT;

    if(x->user[X_R_RPM]>(500.0/9.549))
      OilPdem = OilP_0 + OilP_man_P*x->user[X_R_MAN_P];
    else
      OilPdem = 0.0;
    dx->user[X_R_OIL_P] = (OilPdem - x->user[X_R_OIL_P])/t_OilP;

//    if(u->dinps[U_D_R_PRIME]==TRUE)
//      ff_dem = -primer_max_ff*u->inps[U_A_R_MIX]/3600.0;
//    else
//      ff_dem = -2.351e-3 * x->user[X_R_RPM] * x->user[X_R_MAN_P] * mixture / 3600.0;

//    if(u->dinps[U_D_R_PRIME]==TRUE)
    if( (u->dinps[U_D_R_PRIME]==TRUE) &&
        ( ( (x->user[X_R_FUEL]*(u->dinps[U_D_R_XFEED]==0)) +
            (x->user[X_L_FUEL]*(u->dinps[U_D_R_XFEED]==2)) ) > 0.0) )

    {
      ff_prime = -(primer_max_ff + 1.0*u->inps[U_A_R_THROT]*u->inps[U_A_R_MIX])/3600.0;
    }
    else
      ff_prime = 0.0;

    ff_dem = - 0.75 * bhp*(720e-6*bhp + 0.478) * mixture / (6.0 * 3600.0);
    if (ff_dem < -35.0) ff_dem = -35.0/3600.0;

    if ((-ff_prime) > (-ff_dem)) ff_dem=ff_prime;

    dx->user[X_R_FF]   = (ff_dem-x->user[X_R_FF])/T_ff;


// fuel tank contents depend on cross-feeds.  Include effect of
// vapour return lines

    vap_ret_l = x->user[X_L_FF]*0.1; // additional 10% for vapour return
    vap_ret_r = x->user[X_L_FF]*0.1;
//    dx->user[X_L_FUEL] = -vap_ret_l;
//    dx->user[X_R_FUEL] = -vap_ret_r;

    //if(u->dinps[U_D_L_XFEED]==0) dx->user[X_L_FUEL] += (x->user[X_L_FF]+vap_ret_l);
    //if(u->dinps[U_D_L_XFEED]==2) dx->user[X_R_FUEL] += (x->user[X_L_FF]+vap_ret_l);
    //if(u->dinps[U_D_R_XFEED]==0) dx->user[X_R_FUEL] += (x->user[X_R_FF]+vap_ret_r);
    //if(u->dinps[U_D_R_XFEED]==2) dx->user[X_L_FUEL] += (x->user[X_R_FF]+vap_ret_r);

// last action - override all for engine shutdown if both mags off
// or mixture < 10% (& speed low), or rpm low with starter OFF
//    if ((n_mags==0) || ((mixture<0.1)&&(Vu<10.0)) || ((x->user[X_R_RPM]<eng_min_speed)&&(u->dinps[U_D_START]!=+1)))
    if
    (
      ( ((n_mags==0) || (mixture<0.1))  && (Vu<10.0) && (u->dinps[U_D_START]!=+1) ) ||
      ( u->dinps[U_D_R_ENGINE_SEIZE] ) ||
      ( (x->user[X_R_RPM]<eng_min_speed) && (u->dinps[U_D_START]!=+1) )
    )
    dx->user[X_R_RPM] = -x->user[X_R_RPM]/eng_shutdown_t;

   if (dx->user[X_R_RPM] > +eng_shutdown_rate_limit) dx->user[X_R_RPM] = +eng_shutdown_rate_limit;
	if (dx->user[X_R_RPM] < -eng_shutdown_rate_limit) dx->user[X_R_RPM] = -eng_shutdown_rate_limit;

   low_throttle_factor = (1.0-u->inps[U_A_L_THROT]/0.05);
   if (low_throttle_factor < 0.0) low_throttle_factor = 0.0;
   if (low_throttle_factor > 1.0) low_throttle_factor = 1.0;

   low_throttle_factor = 0.0f;
   if(x->user[X_L_MAN_P]<16.0)//CJ1
   {
     map_gauge_dem1=x->user[X_L_MAN_P]+/*(1.0-0.25*low_throttle_factor)**/(map_fix-x->user[X_L_MAN_P])*0.75;
     //if(map_gauge_dem1 > 16.0)
    // map_gauge_dem1 = 16.0;
   }
   else
     map_gauge_dem1=x->user[X_L_MAN_P];

   //map_gauge_dem1=x->user[X_L_MAN_P];

   map_gauge_dem= (u->dinps[U_D_L_MAP_DRAIN]==FALSE)*map_gauge_dem1 +
                  (u->dinps[U_D_L_MAP_DRAIN]==TRUE)*(p*29.92/101325.0);
   if (map_gauge_dem > 42.0) map_gauge_dem = 42.0;
   dx->user[X_L_MAP_GAUGE]=(map_gauge_dem-x->user[X_L_MAP_GAUGE])/1.0;

   low_throttle_factor = (1.0-u->inps[U_A_R_THROT]/0.05);
   if (low_throttle_factor < 0.0) low_throttle_factor = 0.0;
   if (low_throttle_factor > 1.0) low_throttle_factor = 1.0;
   low_throttle_factor = 0.0f;
   if(x->user[X_R_MAN_P]<16.0)//CJ1
   {
     map_gauge_dem1=x->user[X_R_MAN_P]+/*(1.0-0.25*low_throttle_factor)**/(map_fix-x->user[X_R_MAN_P])*0.75;
//     if(map_gauge_dem1 > 16.0)
  //       map_gauge_dem1 = 16.0;
   }
     else
     map_gauge_dem1=x->user[X_R_MAN_P];
   
    //map_gauge_dem1=x->user[X_R_MAN_P];

   map_gauge_dem= (u->dinps[U_D_R_MAP_DRAIN]==FALSE)*map_gauge_dem1 +
                  (u->dinps[U_D_R_MAP_DRAIN]==TRUE)*(p*29.92/101325.0);
   if (map_gauge_dem > 42.0) map_gauge_dem = 42.0;
   dx->user[X_R_MAP_GAUGE]=(map_gauge_dem-x->user[X_R_MAP_GAUGE])/1.0;

   if(TpropL < 0.0f)
   {
      TpropL = TpropL * 2.5f;//tprop_scale;
   }

   if(TpropR < 0.0f)
   {
      TpropR = TpropR * 2.5f;//tprop_scale;
   }

   if((TpropL - TpropR) > 1000)
   {
      if(tprop_scale > TARGET_POWER)
      {
         tprop_scale -= 0.05 * delta_time;
      }

      TpropL = TpropL * tprop_scale;

   }
   else
   if((TpropR - TpropL) > 1000)
   {
      if(tprop_scale > TARGET_POWER)
      {
         tprop_scale -= 0.05 * delta_time;
      }

      TpropR = TpropR * tprop_scale;
   }
   else
   {
      if(tprop_scale < 1.0f)
      {
         tprop_scale += 0.05 * delta_time;
      }

   }

   FM_eng[0] = thrust_scale*(TpropL + TpropR);//*tprop_scale;
   FM_eng[1] = 0.0;
   FM_eng[2] = 0.0;
   FM_eng[3] = 0.0;
   FM_eng[4] = 0.0;
   if(u->dinps[U_D_PARK] == TRUE && y->duser[Y_D_L_CONTACT])
   {      
      FM_eng[5] = 0.0;
   }
   else
   {

      //FM_eng[5] = 5.0 * (TpropL-TpropR)*y_eng;
     FM_eng[5] = (1.0*flying_factor + 0.5*ground_factor) * 
                  ( (1.0-ground_factor*u->inps[U_A_L_BRAKE])*TpropL - 
                   (1.0-ground_factor*u->inps[U_A_R_BRAKE])*TpropR    ) * y_eng;
//      FM_eng[5] = (1.0-max(u->inps[U_A_L_BRAKE],u->inps[U_A_L_BRAKE])) * 5.0 * (TpropL-TpropR)*y_eng;

   }

//   if(u->dinps[U_D_GEAR_EMERG])
//     FM_eng[5] += k_delta_MAP_dot * (dx->user[X_L_MAN_P]-dx->user[X_R_MAN_P]);
//   if(u->dinps[U_D_GEAR_EMERG])
     //FM_eng[5] += flying_factor * k_N_delta_MAP_dot * (dx->user[X_L_MAP_GAUGE]-dx->user[X_R_MAP_GAUGE]);

// ??
// Asymmetric MAPdot effect on roll.  I thought we needed to keep this, but was tested
// and approved with Xfeed off.  Hence leave out unless otherwise required.
//   if(u->dinps[U_D_L_XFEED]==2)
//     FM_eng[3] += flying_factor * k_L_delta_MAP_dot * (dx->user[X_L_MAP_GAUGE]-dx->user[X_R_MAP_GAUGE]);

// Aerodynamic forces & moments

    xsi_dem  = ail_max  * D2R * (u->inps[U_A_LAT]  + auth_roll_servo      * kfc150_roll_servo);
    zeta_dem = rudd_max * D2R * (u->inps[U_A_PED]  + auth_rudd_trim_aero  * u->inps[U_A_RUD_TRIM]);
    eta_dem  = elev_max * D2R * (u->inps[U_A_LONG] + auth_pitch_trim_aero * u->inps[U_A_ELE_TRIM] + auth_pitch_servo*kfc150_pitch_servo );

// Cubic stick shaping functions on aileron (xsi), rudder (zeta) and elevator (eta).
// f factors control how much effect is applied, 1.0=max effect, 0.0=no effect, i.e.
// normal linear operation
    xsi  = (1.0-f1)* xsi_dem + f1* xsi_dem* xsi_dem* xsi_dem;
    zeta = (1.0-f2)*zeta_dem + f2*zeta_dem*zeta_dem*zeta_dem;
    eta  = (1.0-f3)* eta_dem + f3* eta_dem* eta_dem* eta_dem;

    Cl_flap          =     table1D_war(flap_deflection,dCl_flap,6,x->user[X_FLAP]);
    Cd_flap          =     table1D_war(flap_deflection,dCd_flap,6,x->user[X_FLAP]);
    CM_flap          =     table1D_war(flap_deflection,dCM_flap,6,x->user[X_FLAP]);
    alpha_stall_flap = D2R*table1D_war(flap_deflection,dSt_flap,6,x->user[X_FLAP]);

    Cd_gear = dCd_gear * x->user[X_N_GEAR];
    CM_gear = dCM_gear * x->user[X_N_GEAR];

    Cd_phug = KCd_phug * (Vu-80.0);

//    Vt_l = 0.01 * TpropL; // left propwash speed
//      alpha_l = atan2(Vw - x->p*b/2.0,Vu + Vt_l + x->r*b/2.0);
//    Vt_r = 0.01 * TpropR; // right propwash speed
//      alpha_r = atan2(Vw + x->p*b/2.0,Vu + Vt_r - x->r*b/2.0);
//    if(fabs(Vu+Vt_l)>0.1) alpha_l = atan2(Vw - x->p*b/2.0,Vu + x->r*b/2.0);
//    if(fabs(Vu+Vt_r)>0.1) alpha_r = atan2(Vw + x->p*b/2.0,Vu - x->r*b/2.0);

//    Vt_l = flying_factor * 0.01 * TpropL; // left propwash speed
//    Vt_r = flying_factor * 0.01 * TpropR; // right propwash speed
    Vt_l = flying_factor * 0.001 * TpropL; // left propwash speed
    Vt_r = flying_factor * 0.001 * TpropR; // right propwash speed

    if(fabs(Vu)>0.1) alpha_l = atan2(Vw - 0.1*x->p*b/2.0,Vu + Vt_l + x->r*b/2.0);
    else             alpha_l = 90.0*D2R;

    if(fabs(Vu)>0.1) alpha_r = atan2(Vw + 0.1*x->p*b/2.0,Vu + Vt_r - x->r*b/2.0);
    else             alpha_r = 90.0*D2R;

// stall warner is set on free stream incidence of left wing
//    if(alpha_l > (alpha_stall_warn+alpha_stall_flap))

    
    k_dalpha_stall_T = table1D_war(map_value,dem_k_dalpha,3,y->user[Y_A_L_MAN_P]);
    dalpha_stall_T = k_dalpha_stall_T*(TpropL+TpropR)/2.0;


    if( atan2(Vw,Vu) > ( alpha_stall_warn + 
                         0.5*alpha_stall_flap + 
                         dalpha_stall_T) )
      y->duser[Y_D_STALL] =  TRUE;
    else
      y->duser[Y_D_STALL] = FALSE;

    col_alpha = atan2(Vw,Vu);
    if(alpha_l > max_alpha)
       alpha_l = max_alpha;

    if(alpha_r > max_alpha)
       alpha_r = max_alpha;

    col_al    = alpha_l;
    col_ar    = alpha_r;


    calc_war_Cl_Cd( alpha_l,
                Cla_w,
                Cl_flap+dCl_ice*x->user[X_ICE],
                alpha_0_w,
                (alpha_stall_0+alpha_stall_flap+dalpha_stall_T),
                Cd0,
                k,
                &Cl_w_l,
                &Cd_w_l);

    calc_war_Cl_Cd( alpha_r,
                Cla_w,
                Cl_flap+dCl_ice*x->user[X_ICE],
                alpha_0_w,
                (alpha_stall_0+alpha_stall_flap+dalpha_stall_T),
                Cd0,
                k,
                &Cl_w_r,
                &Cd_w_r);

    //col_alpha = alpha_stall_0+alpha_stall_flap+dalpha_stall_T;
//    Cl_w = Cla_w * (y->alpha-alpha_0_w) + Cl_flap + dCl_ice*x->user[X_ICE];

//    Cd_w = Cd0+k*Cl_w*Cl_w + Cd_flap + Cd_gear + dCd_ice*x->user[X_ICE] +
//       dCd_cowl_flap*(u->inps[U_A_L_COWL]+u->inps[U_A_R_COWL])/2.0;

    Cd_w_l += Cd_flap +
              Cd_gear +
              dCd_ice*x->user[X_ICE] +
              dCd_cowl_flap*(u->inps[U_A_L_COWL]+u->inps[U_A_R_COWL])/2.0 +
              Cd_phug;

    Cd_w_r += Cd_flap +
              Cd_gear +
              dCd_ice*x->user[X_ICE] +
              dCd_cowl_flap*(u->inps[U_A_L_COWL]+u->inps[U_A_R_COWL])/2.0 +
              Cd_phug;

    if (Cd_w_l < Cd0) Cd_w_l = Cd0;
    if (Cd_w_r < Cd0) Cd_w_r = Cd0;

	L_w_l  = 0.5 * rho * (Vu+Vt_l) * (Vu+Vt_l) * 0.5*Sw * Cl_w_l;
	L_w_r  = 0.5 * rho * (Vu+Vt_r) * (Vu+Vt_r) * 0.5*Sw * Cl_w_r;


   lwlift = L_w_l;
   rwlift = L_w_r;
   
	D_w_l  = drag_scale * 0.5 * rho * (Vu+Vt_l) * (Vu+Vt_l) * 0.5*Sw * Cd_w_l;
	D_w_r  = drag_scale * 0.5 * rho * (Vu+Vt_r) * (Vu+Vt_r) * 0.5*Sw * Cd_w_r;

//    h0 = 0.25;
    h = (u->inps[U_A_CG] - 78.4 )*0.0254/cbar;

//    if (fabs(Vu+0.5*(Vt_l+Vt_r)) > 0.1) alpha_t = (1.0 - deda)*y->alpha + alpha_s_t + k_pitch_damp*x->q*lt/(Vu+0.5*(Vt_l+Vt_r)) + eta;
//    else                alpha_t = (1.0 - deda)*y->alpha + alpha_s_t;

    if (fabs(Vu+0.5*(Vt_l+Vt_r)) > 0.1) alpha_t = (1.0 - deda)*w_alpha + alpha_s_t + k_pitch_damp*x->q*lt/(Vu+0.5*(Vt_l+Vt_r)) + eta;
    else                alpha_t = (1.0 - deda)*w_alpha + alpha_s_t;


    if(alpha_t > max_alpha)
       alpha_t = max_alpha;
    col_at = alpha_t;
    calc_war_Cl_Cd( alpha_t,
                Cla_t,
                0.0,
                0.0,
                90.0*D2R, // magic no-stall tailplane!
                0.1,
                0.0,
                &Cl_t,
                &Cd_t);

//    calc_Cl_Cd( alpha_t,
//                Cla_t,
//                0.0,
//                0.0,
//                17.5*D2R+0.5*dalpha_stall_T,
//                0.1,
//                0.0,
//                &Cl_t,
//                &Cd_t);

    //Cl_t    = Cla_t*alpha_t; // all-moving tail! + Cl_t_eta*eta;

    L_t     = qbar * St * Cl_t;
    D_t     = qbar * St * Cd_t;

    tlift = L_t;
// Laterals are implemented as stability derivatives

    Yaero = 0.5 * rho * y->Vt * Sw     * (Yv*Vv + b*(Yp*x->p + Yr*x->r) + y->Vt*(Yzeta*zeta + Yxsi*xsi));
    Laero = 0.5 * rho * y->Vt * Sw * b * (Lv*Vv + b*(Lp*x->p + Lr*x->r) + y->Vt*(Lzeta*zeta + Lxsi*xsi)) +
            (L_w_l - L_w_r) * y_eng;
    Naero = 0.5 * rho * y->Vt * Sw * b * (Nv*Vv + b*(Np*x->p + Nr*x->r) + y->Vt*(Nzeta*zeta + Nxsi*xsi)) +
            (D_w_r - D_w_l)*y_eng;
/*
	FM_aero[0] =   (L_w_l+L_w_r)*sin(y->alpha) - (D_w_l+D_w_r)*cos(y->alpha);
	FM_aero[1] =  Yaero;
	FM_aero[2] = -(L_w_l+L_w_r+L_t)*cos(y->alpha) - (D_w_l+D_w_r)*sin(y->alpha);
	FM_aero[3] =  Laero;
	FM_aero[4] =  qbar*Sw*(CM0+CM_flap+CM_gear)*cbar + (L_w_l+L_w_r)*(h-h0)*cbar - L_t*lt*cos(y->alpha)-D_t*lt*sin(y->alpha);
	FM_aero[5] =  Naero;
*/
	FM_aero[0] =   (L_w_l+L_w_r)*sin(w_alpha) - (D_w_l+D_w_r)*cos(w_alpha);
	FM_aero[1] =  Yaero;
	FM_aero[2] = -(L_w_l+L_w_r+L_t)*cos(w_alpha) - (D_w_l+D_w_r)*sin(w_alpha);
	FM_aero[3] =  Laero;
	FM_aero[4] =  qbar*Sw*(CM0+CM_flap+CM_gear)*cbar + (L_w_l+L_w_r)*(h-h0)*cbar - L_t*lt*cos(w_alpha)-D_t*lt*sin(w_alpha);
	FM_aero[5] =  Naero;

;

/*** Force/Moment Summation ***/

	FM[0] = FM_aero[0] + FM_eng[0] + FM_gr[0];
	FM[1] = FM_aero[1] + FM_eng[1] + FM_gr[1];
	FM[2] = FM_aero[2] + FM_eng[2] + FM_gr[2];
	FM[3] = FM_aero[3] + FM_eng[3] + FM_gr[3];
	FM[4] = FM_aero[4] + FM_eng[4] + FM_gr[4];
	FM[5] = FM_aero[5] + FM_eng[5] + FM_gr[5];

        y->user[Y_A_TRIM_COST] =
	sqrt(
	  (FM[0] + mass->mass*9.81*y->S[0][2])*(FM[0] + mass->mass*9.81*y->S[0][2]) +
	  (FM[1] + mass->mass*9.81*y->S[1][2])*(FM[1] + mass->mass*9.81*y->S[1][2]) +
	  (FM[2] + mass->mass*9.81*y->S[2][2])*(FM[2] + mass->mass*9.81*y->S[2][2]) +
	  FM[3]*FM[3] +
	  FM[4]*FM[4] +
	  FM[5]*FM[5] );

/*** STATE DERIVATIVES ***/


    if(u->dinps[U_D_AIRFRAME_ICE]==TRUE)
      dx->user[X_ICE] = (1.0-x->user[X_ICE])/T_ice;
    else
      dx->user[X_ICE] = 0.0;

    if( (u->dinps[U_D_DEICE]==TRUE) && (u->dinps[U_D_FAIL_DEICE]==FALSE) )
      dx->user[X_ICE] = (0.0-x->user[X_ICE])/T_deice;

    if(u->dinps[U_D_PROP_ICE]==TRUE)
    {
      dx->user[X_L_PROP_ICE] = (1.0-x->user[X_L_PROP_ICE])/T_ice;
      dx->user[X_R_PROP_ICE] = (1.0-x->user[X_R_PROP_ICE])/T_ice;
    }
    else
    {
      dx->user[X_L_PROP_ICE] = 0.0;
      dx->user[X_R_PROP_ICE] = 0.0;
    }

    if( (u->dinps[U_D_DEICE]==TRUE) && (u->dinps[U_D_FAIL_DEICE]==FALSE) )
      dx->user[X_ICE] = (0.0-x->user[X_ICE])/T_deice;

    if( (u->dinps[U_D_PROP_DEICE]==TRUE) && (u->dinps[U_D_FAIL_L_DEICE]==FALSE) )
      dx->user[X_L_PROP_ICE] = (0.0-x->user[X_L_PROP_ICE])/T_deice;

    if( (u->dinps[U_D_PROP_DEICE]==TRUE) && (u->dinps[U_D_FAIL_R_DEICE]==FALSE) )
      dx->user[X_R_PROP_ICE] = (0.0-x->user[X_R_PROP_ICE])/T_deice;


    max_flap_up_speed = table1D_war(flap_airspd,flap_uprate,3,y->user[Y_A_P1_ASI]);
    max_flap_dn_speed = table1D_war(flap_airspd,flap_dnrate,3,y->user[Y_A_P1_ASI]);


    if(u->dinps[U_D_FAIL_FLAP_MOTOR]==FALSE)
    {
   	  //dx->user[X_FLAP] = ((float)(flap_stages[u->dinps[U_D_FLAP]]) - x->user[X_FLAP])/0.5;
       dx->user[X_FLAP] = ((float)(u->inps[U_A_FLAP]) - x->user[X_FLAP])/0.5;
	  if (dx->user[X_FLAP] > 0.5) dx->user[X_FLAP] = max_flap_dn_speed;
	  if (dx->user[X_FLAP] < -0.50) dx->user[X_FLAP] = -max_flap_up_speed;
    }
    else
   	  dx->user[X_FLAP] = 0.0;


   dx->user[X_N_GEAR] = 0.0;
   dx->user[X_L_GEAR] = 0.0;
   dx->user[X_R_GEAR] = 0.0;


   if (u->dinps[U_D_FAIL_GEAR_PUMP]==FALSE)
   {
      //CJ if flag is true and gear is not fully down then stop movment of nose leg
      if(ngear_fail == TRUE)
      {
         ///don't do anything
      }
      else
      {
         dx->user[X_N_GEAR] = ((float)(u->dinps[U_D_GEAR]) - x->user[X_N_GEAR])/t_N_gear_1;         
	      if (dx->user[X_N_GEAR] > +max_gear_rate_1) dx->user[X_N_GEAR] = +max_gear_rate_1;
	      if (dx->user[X_N_GEAR] < -max_gear_rate_1) dx->user[X_N_GEAR] = -max_gear_rate_1;
      }

   	   dx->user[X_L_GEAR] = ((float)(u->dinps[U_D_GEAR]) - x->user[X_L_GEAR])/t_L_gear_1;
	  if (dx->user[X_L_GEAR] > +max_gear_rate_1) dx->user[X_L_GEAR] = +max_gear_rate_1;
	  if (dx->user[X_L_GEAR] < -max_gear_rate_1) dx->user[X_L_GEAR] = -max_gear_rate_1;

     	   dx->user[X_R_GEAR] = ((float)(u->dinps[U_D_GEAR]) - x->user[X_R_GEAR])/t_R_gear_1;
	  if (dx->user[X_R_GEAR] > +max_gear_rate_1) dx->user[X_R_GEAR] = +max_gear_rate_1;
	  if (dx->user[X_R_GEAR] < -max_gear_rate_1) dx->user[X_R_GEAR] = -max_gear_rate_1;
   }

   if(u->dinps[U_D_GEAR_EMERG]==1)
   {
      //CJ if flag is true and gear is not fully down then stop movment of nose leg
      if(ngear_fail == TRUE)
      {
         ///don't do anything
      }
      else
      {
         dx->user[X_N_GEAR] = ((float)(u->dinps[U_D_GEAR_EMERG]) - x->user[X_N_GEAR])/t_N_gear_2;   	
	      if (dx->user[X_N_GEAR] > +max_gear_rate_2) dx->user[X_N_GEAR] = +max_gear_rate_2;
	      if (dx->user[X_N_GEAR] < -max_gear_rate_2) dx->user[X_N_GEAR] = -max_gear_rate_2;
      }

   	dx->user[X_L_GEAR] = ((float)(u->dinps[U_D_GEAR_EMERG]) - x->user[X_L_GEAR])/t_L_gear_2;
	if (dx->user[X_L_GEAR] > +max_gear_rate_2) dx->user[X_L_GEAR] = +max_gear_rate_2;
	if (dx->user[X_L_GEAR] < -max_gear_rate_2) dx->user[X_L_GEAR] = -max_gear_rate_2;

   	dx->user[X_R_GEAR] = ((float)(u->dinps[U_D_GEAR_EMERG]) - x->user[X_R_GEAR])/t_R_gear_2;
	if (dx->user[X_R_GEAR] > +max_gear_rate_2) dx->user[X_R_GEAR] = +max_gear_rate_2;
	if (dx->user[X_R_GEAR] < -max_gear_rate_2) dx->user[X_R_GEAR] = -max_gear_rate_2;
   }

    if ( ((x->user[X_N_GEAR] > 0.05)&(x->user[X_N_GEAR] < 0.95)) |
         ((x->user[X_L_GEAR] > 0.05)&(x->user[X_L_GEAR] < 0.95)) |
         ((x->user[X_R_GEAR] > 0.05)&(x->user[X_R_GEAR] < 0.95)) )
      y->duser[Y_D_GEAR] = TRUE;
    else
      y->duser[Y_D_GEAR] = FALSE;

    if(x->user[X_N_GEAR]>0.95) y->duser[Y_D_N_GEAR] = TRUE;
    else                       y->duser[Y_D_N_GEAR] = FALSE;
    if(x->user[X_L_GEAR]>0.95) y->duser[Y_D_L_GEAR] = TRUE;
    else                       y->duser[Y_D_L_GEAR] = FALSE;
    if(x->user[X_R_GEAR]>0.95) y->duser[Y_D_R_GEAR] = TRUE;
    else                       y->duser[Y_D_R_GEAR] = FALSE;

//    if(fabs(dx->user[X_FLAP])>0.10) y->duser[Y_D_FLAPS] = TRUE;
//    else                            y->duser[Y_D_FLAPS] = FALSE;

    if( fabs(x->user[X_FLAP] - u->inps[U_A_FLAP]) > 1.0 )
    //if( fabs(x->user[X_FLAP] - (float)flap_stages[u->dinps[U_D_FLAP]]) > 1.0 )
      y->duser[Y_D_FLAPS] = TRUE;
    else
      y->duser[Y_D_FLAPS] = FALSE;

//    if(y->alpha > (alpha_stall_warn+alpha_stall_flap)) y->duser[Y_D_STALL] = TRUE;
//    if(alpha_l > (alpha_stall_warn+alpha_stall_flap))
//      y->duser[Y_D_STALL] =  TRUE;
//    else
//      y->duser[Y_D_STALL] = FALSE;


//    dx->user[X_L_FUEL] = -2.6105e-5 * u->inps[U_A_L_THROT]*x->user[X_L_RPM]*u->inps[U_A_L_MIX];
//    dx->user[X_R_FUEL] = -2.6105e-5 * u->inps[U_A_R_THROT]*x->user[X_R_RPM]*u->inps[U_A_R_MIX];
//    y->user[Y_A_L_FF] = -dx->user[X_L_FUEL] * 3600.0;
//    y->user[Y_A_R_FF] = -dx->user[X_R_FUEL] * 3600.0;
    y->user[Y_A_L_FF] = -x->user[X_L_FF] * 3600.0;
    y->user[Y_A_R_FF] = -x->user[X_R_FF] * 3600.0;

    y->user[Y_A_XBAL]=FM[0] + mass->mass*9.81*y->S[0][2];
    y->user[Y_A_ZBAL]=FM[2] + mass->mass*9.81*y->S[2][2];
    y->user[Y_A_MBAL]=FM[4];

    dx->user[X_SIM_TIME] = 1.0;

    y->user[Y_A_L_MIXTURE]=dalpha_stall_T*57.3;
//    y->user[Y_A_R_MIXTURE]=ground_factor;
}

void init_war_model(char *filename)
{
// No initialisation is required since all the flight model data is
// hard-coded (i.e. no parameter file needs to be read)
}

void reset_war_FMS(INPUTS *u, STATES *x)
{
int i;

// Initialise ALL states to zero, for safety
	  for(i=0;i<N_USER_STATES;i++)
      {
        x->user[i] = 0.0;
      }

    x->user[X_L_VSI_GAUGE]=0.0;
    x->user[X_R_VSI_GAUGE]=0.0;

    x->user[X_L_MAP_GAUGE]=29.92;
    x->user[X_R_MAP_GAUGE]=29.92;

    x->user[X_N_GEAR]=(float)(u->dinps[U_D_GEAR]);
    x->user[X_L_GEAR]=(float)(u->dinps[U_D_GEAR]);
    x->user[X_R_GEAR]=(float)(u->dinps[U_D_GEAR]);
    x->user[X_FLAP] = (float) u->inps[U_A_FLAP];//(flap_stages[u->dinps[U_D_FLAP]]);//u->dinps[U_D_FLAP];//
    x->user[X_L_FUEL] = u->inps[U_A_L_INIT_FUEL];
    x->user[X_R_FUEL] = u->inps[U_A_R_INIT_FUEL];
    x->user[X_SIM_TIME]=0.0;
    x->user[X_ICE]=(float)(u->dinps[U_D_AIRFRAME_ICE]);
    x->user[X_L_PROP_ICE]=(float)(u->dinps[U_D_PROP_ICE]);
    x->user[X_R_PROP_ICE]=(float)(u->dinps[U_D_PROP_ICE]);


    x->user[X_P_S1] = 101325.0;
    x->user[X_P_S2] = 101325.0;
    x->user[X_L_BLADE] = 0.0;
    x->user[X_R_BLADE] = 0.0;
    x->user[X_L_MAN_P]=29.92;
    x->user[X_R_MAN_P]=29.92;

    if(u->dinps[U_D_LIVE_START]==TRUE)
    {
      x->user[X_L_RPM] = 0.0f;//rpm_lo + (rpm_hi-rpm_lo)*u->inps[U_A_L_RPM];
      x->user[X_R_RPM] = 0.0f;//rpm_lo + (rpm_hi-rpm_lo)*u->inps[U_A_R_RPM];
    }
    else
    {
      x->user[X_L_RPM] = 0.0;
      x->user[X_R_RPM] = 0.0;
    }

//X_L_COWL - no dynamics
//X_R_COWL - no dynamics
#define X_P_PITOT                6
#define X_L_RPM                  9
#define X_L_BLADE                10
#define X_L_MAN_P                11
#define X_L_CHT                  13
#define X_L_EGT                  14
#define X_R_RPM                  15
#define X_R_BLADE                16
#define X_R_MAN_P                17
#define X_R_CHT                  19
#define X_R_EGT                  20
#define X_R_FF       29
#define X_L_FF       30
#define X_L_BOOST 31
#define X_R_BOOST 32
#define X_L_OIL_T    21
#define X_R_OIL_T    22
#define X_L_OIL_P    23
#define X_R_OIL_P    24



}

void calc_war_OP_FMS(INPUTS *u, STATES *x, OUTPUTS *y)
{
int i;
static int pass=0;

// Initialise ALL outputs to zero, for safety (first pass only)
	if (pass==0)
	{
	  for(i=0;i<N_USER_A_OUTPUTS;i++)
      {
        y->user[i]  = 0.0;
        y->duser[i] = 0;
      }
	  pass=1;
	}
    calc_war_instruments(x, y, u->inps[U_A_ALT_P1A],
                           u->inps[U_A_ALT_P1B],
                           u->inps[U_A_ALT_P2]);

    y->user[Y_A_L_BLADE] = R2D*x->user[X_L_BLADE];
    y->user[Y_A_R_BLADE] = R2D*x->user[X_R_BLADE];

    y->user[Y_A_L_MAN_P] = x->user[X_L_MAP_GAUGE];
    y->user[Y_A_R_MAN_P] = x->user[X_R_MAP_GAUGE];

    y->user[Y_A_L_RPM  ] = 60.0*x->user[X_L_RPM]/(2*PI);
    y->user[Y_A_R_RPM  ] = 60.0*x->user[X_R_RPM]/(2*PI);

    y->user[Y_A_L_EGT  ] = x->user[X_L_EGT  ];
    y->user[Y_A_R_EGT  ] = x->user[X_R_EGT  ];

    y->user[Y_A_L_CHT  ] = x->user[X_L_CHT  ];
    y->user[Y_A_R_CHT  ] = x->user[X_R_CHT  ];

//  fuel flow rate output computed in calc_FM()
    y->user[Y_A_L_FUEL ] = x->user[X_L_FUEL ];
    y->user[Y_A_R_FUEL ] = x->user[X_R_FUEL ];

    y->user[Y_A_L_OIL_P] = 0.0;
    y->user[Y_A_R_OIL_P] = 0.0;

    y->user[Y_A_L_OIL_T] = x->user[X_L_OIL_T];
    y->user[Y_A_R_OIL_T] = x->user[X_R_OIL_T];

    y->user[Y_A_L_OIL_P] = x->user[X_L_OIL_P];
    y->user[Y_A_R_OIL_P] = x->user[X_R_OIL_P];

    y->user[Y_A_PITCH_NULL] = auth_pitch_trim * u->inps[U_A_ELE_TRIM];
    y->user[Y_A_PEDAL_NULL] = auth_rudd_trim * u->inps[U_A_RUD_TRIM];

    y->user[Y_A_VACUUM] =
      ( y->user[Y_A_L_RPM]*(u->dinps[U_D_FAIL_L_SUCTION]==false) +
        y->user[Y_A_R_RPM]*(u->dinps[U_D_FAIL_R_SUCTION]==false) )
        * 5.0/500.0; // 5.0 psi at 500 rpm from any one engine

    if (y->user[Y_A_VACUUM] > 5.0) y->user[Y_A_VACUUM]=5.0;

    if( sqrt(y->az*y->az + y->ay*y->ay + y->ax*y->ax) > (9.81*10.0) )
      y->duser[Y_D_CRASH]=1;
    else
      y->duser[Y_D_CRASH]=0;

    y->duser[Y_D_CRASH]=0;

}










void calc_war_instruments(STATES *x, OUTPUTS *y, float alt1, float alt2, float alt3)
{
static float T0 = 288.15;
static float rho0 = 1.2256;

// Rate of climb (ft/min) Pressure version
	y->user[Y_A_P1_ROC]  =  - 60.0 * y->user[Y_A_DS1DT] * 3.28 / 11.95;
	y->user[Y_A_P2_ROC]  =  - 60.0 * y->user[Y_A_DS2DT] * 3.28 / 11.95;

// Altimeters (ft)
	y->user[Y_A_P1A_ALT] = 3.28 * T0*(1.0-pow(0.01*x->user[X_P_S1]/alt1,0.190263107)) / 0.0065;
 	y->user[Y_A_P1B_ALT] = 3.28 * T0*(1.0-pow(0.01*x->user[X_P_S1]/alt2,0.190263107)) / 0.0065;
 	y->user[Y_A_P2_ALT]  = 3.28 * T0*(1.0-pow(0.01*x->user[X_P_S2]/alt3,0.190263107)) / 0.0065;

// Airspeed indicators (kn)
   //MOD BY COLIN STOP A STATIC BLOCK CAUSING AIRSPEED TO DROP AND THEN RISE AS PITOT PRESSURE GOES BELOW STATIC PRESSURE
   if((y->user[Y_A_PITOT] - x->user[X_P_S1]) > 0.0)
   {
       y->user[Y_A_P1_ASI] = sqrt( fabs(y->user[Y_A_PITOT] - x->user[X_P_S1]) / (0.5*rho0) ) * 1.9144;
       y->user[Y_A_P2_ASI] = sqrt( fabs(y->user[Y_A_PITOT] - x->user[X_P_S1]) / (0.5*rho0) ) * 1.9144;
   }
   else
   {
      y->user[Y_A_P1_ASI] = 0.0;
      y->user[Y_A_P2_ASI] = 0.0;
   }

// Turn & slip indicators
	y->user[Y_A_SLIP_ANGLE] = y->ay / 9.81; /* rad */
	y->user[Y_A_TURN_RATE]  = y->psi_dot * 60.0 * 57.3; /* deg/min */
}

#ifdef FDL_HOME
// Executable code for KFC150 autopilot

//void KFC150_exec( float theta, float phi, float psi, float alt,
//                  float hdg_bug,
//                  float loc_hdg, float loc_dev, float gld_dev,
//                  float time,
//                  float *pitch_servo, float *roll_servo, float *pitch_trim,
//                  float *fd_pitch, float *fd_roll,
//                  int buttons[], int lamps[] , int *fd_visible, int *pitch_trim_req)
void KFC150_exec_FDL( float theta,
                  float phi,
                  float psi,
                  float alt,
                  float hdg_err,
                  float cdi,
                  float gs_dev,
                  float time,
                  float hdg,
                  float *pitch_servo,
                  float *roll_servo,
                  float *pitch_trim,
                  float *fd_pitch,
                  float *fd_roll,
                  int buttons[],
                  int lamps[],
                  int *fd_visible,
                  int *pitch_trim_req,
                  int power_switch )
{
static int mode = KFC150_M_RESET;
static float timer_start=0.0,last_time;
float dt,psi_error,theta_error,phi_error,alt_error;
static bool fd_mode=FALSE, ap_mode=FALSE, hdg_mode=FALSE, alt_mode=FALSE;
static int button_state[16];
static float theta_dem,alt_dem;
static float last_psi_error,last_theta_error,last_phi_error,last_alt_error,last_alt;
static float i_psi_error,i_theta_error,i_phi_error,i_alt_error;
float d_psi_error,d_theta_error,d_phi_error,d_alt_error;
float dtime;
static float roll_servo_dem,pitch_servo_dem,hdot_dem;
bool flash1,flash2;
float hdot;

// All autopilot commands are zero unless specifically commanded otherwise
    *fd_pitch    = 0.0;
    *fd_roll     = 0.0;
    *pitch_servo = 0.0;
    *roll_servo  = 0.0;
    *pitch_trim  = 0.0;

    if(power_switch==false) // i.e. avionics switched off
    {
      lamps[KFC150_L_FD]   = FALSE; lamps[KFC150_L_ALT]  = FALSE;
      lamps[KFC150_L_HDG]  = FALSE; lamps[KFC150_L_GS]   = FALSE;
      lamps[KFC150_L_NAV]  = FALSE; lamps[KFC150_L_APR]  = FALSE;
      lamps[KFC150_L_BC]   = FALSE; lamps[KFC150_L_TRIM] = FALSE;
      lamps[KFC150_L_AP]   = FALSE;
      pitch_servo_dem = 0.0;
      mode=KFC150_M_RESET;
      return;
    }

    dtime = time-last_time;
    if (dtime<0.001) dtime=0.001;

// If in first 0.1 seconds then do power-up reset
    if(time<0.1)
    {
      roll_servo_dem  = 0.0;
      pitch_servo_dem = 0.0;
      mode = KFC150_M_RESET; // power-up reset!
      fd_mode = FALSE;
      button_state[KFC150_B_FD]     = FALSE;
      button_state[KFC150_B_AP_ENG] = FALSE;
      button_state[KFC150_B_HDG] = FALSE;
      button_state[KFC150_B_ALT] = FALSE;
      i_psi_error      = 0.0;
      i_theta_error    = 0.0;
      i_phi_error      = 0.0;
      last_psi_error   = 0.0;
      last_theta_error = 0.0;
      last_phi_error   = 0.0;
    }

// All autopilot commands are zero unless specifically commanded otherwise
    *fd_pitch    = 0.0;
    *fd_roll     = 0.0;
    *pitch_servo = 0.0;
    *roll_servo  = 0.0;
    *pitch_trim  = 0.0;

    if(buttons[KFC150_B_HDG]==TRUE) // button pressed?
      if (button_state[KFC150_B_HDG]==FALSE) // if was not pressed last frame
      {
        hdg_mode=!hdg_mode; // then change mode
        if(hdg_mode) fd_mode=TRUE;
      }
    button_state[KFC150_B_HDG]=buttons[KFC150_B_HDG];

    if(buttons[KFC150_B_ALT]==TRUE) // button pressed?
      if (button_state[KFC150_B_ALT]==FALSE) // if was not pressed last frame
      {
        alt_mode=!alt_mode; // then change mode
        if(alt_mode)
        {
          alt_dem = alt;
          fd_mode = TRUE;
          last_alt=alt;
        }
      }
    button_state[KFC150_B_ALT]=buttons[KFC150_B_ALT];

    if(buttons[KFC150_B_FD]==TRUE) // button pressed?
      if (button_state[KFC150_B_FD]==FALSE) // if was not pressed last frame
      {
        fd_mode=!fd_mode; // then change mode
        if(!fd_mode)
        {
          alt_mode = FALSE;
          hdg_mode = FALSE;
        }
      }
    button_state[KFC150_B_FD]=buttons[KFC150_B_FD];

    if(buttons[KFC150_B_AP_ENG]==TRUE) // button pressed?
      if (button_state[KFC150_B_AP_ENG]==FALSE) // if was not pressed last frame
      {
        ap_mode=!ap_mode; // then change mode
        if(ap_mode)
        {
          theta_dem = theta; // capture pitch attitude
          fd_mode = TRUE;
        }
        i_theta_error = 0.0;
        i_phi_error   = 0.0;
        i_psi_error   = 0.0;
        i_alt_error   = 0.0;
      }
    button_state[KFC150_B_AP_ENG]=buttons[KFC150_B_AP_ENG];

    if(buttons[KFC150_B_VT]==+1)
    {
      theta_dem += 0.9*D2R*dtime;
      alt_dem   += (500.0/60.0)*0.3048*dtime;
    }
    if(buttons[KFC150_B_VT]==-1)
    {
      theta_dem -= 0.9*D2R*dtime;
      alt_dem   -= (500.0/60.0)*0.3048*dtime;
    }


    psi_error = hdg_err; //hdg_bug - psi;
    if (psi_error < -PI) psi_error += PI;
    if (psi_error > +PI) psi_error -= PI;
    i_psi_error += psi_error*dtime;
    d_psi_error = (psi_error-last_psi_error)/dtime;
    last_psi_error = psi_error;

    if(hdg_mode)
      *fd_roll = ap_Kpsi_p * psi_error;
    else
      *fd_roll = 0.0;

    if (*fd_roll > +ap_phi_max) *fd_roll = +ap_phi_max;
    if (*fd_roll < -ap_phi_max) *fd_roll = -ap_phi_max;
    phi_error=phi-*fd_roll;
    i_phi_error+=phi_error*dtime;

    if(alt_mode)
    {
      alt_error = alt_dem - alt;
      hdot = (alt-last_alt)/dtime;
      i_alt_error += alt_error*dtime;
      d_alt_error = (alt_error-last_alt_error)/dtime;
      hdot_dem =  ap_Kalt_p * alt_error +
                  ap_Kalt_i * i_alt_error +
                  ap_Kalt_d * d_alt_error;
      theta_dem = ap_theta0 +
                  ap_Khdot_p * (hdot_dem - hdot);
      last_alt_error = alt_error;
      last_alt=alt;
    }
    *fd_pitch = theta_dem;
    theta_error=*fd_pitch-theta;
// Protect pitch integrator against windup
//    if((theta_error*ap_Ktheta_i)>0.0)
//      if((i_theta_error*ap_Ktheta_i) < +1.0) i_theta_error+=theta_error*dtime;
//    else
//      if((i_theta_error*ap_Ktheta_i) > -1.0) i_theta_error+=theta_error*dtime;
    i_theta_error+=theta_error*dtime;

    if (*fd_pitch > ap_theta_max) *fd_pitch = ap_theta_max;
    if (*fd_pitch < ap_theta_min) *fd_pitch = ap_theta_min;

    if(ap_mode==TRUE)
    {
      roll_servo_dem  = (ap_Kphi_p   *   phi_error)   +
                        (ap_Kphi_i   * i_phi_error);
      pitch_servo_dem = (ap_Ktheta_p *   theta_error) +
                        (ap_Ktheta_i * i_theta_error);
      if (roll_servo_dem  > +1.0) roll_servo_dem  = +1.0;
      if (roll_servo_dem  < -1.0) roll_servo_dem  = -1.0;
      if (pitch_servo_dem > +1.0) pitch_servo_dem = +1.0;
      if (pitch_servo_dem < -1.0) pitch_servo_dem = -1.0;
    }


    *roll_servo  = roll_servo_dem;
    *pitch_servo = pitch_servo_dem;

    *pitch_trim_req  = 0;
    if(ap_mode==TRUE)
    {
      if (*pitch_servo < -0.25) *pitch_trim_req = -1;
      if (*pitch_servo > +0.25) *pitch_trim_req = +1;
    }

    if(buttons[KFC150_B_TEST]==TRUE)
    {
      mode = KFC150_M_SELFTEST;
      timer_start = time;
    }

    dt = time-timer_start;
    flash1 = TRUE;
    if((int)(dt*2.0*flash1_rate)&0x01) flash1 = FALSE;
    flash2 = TRUE;
    if((int)(dt*2.0*flash2_rate)&0x01) flash2 = FALSE;

    switch(mode)
    {
      case KFC150_M_RESET:
        lamps[KFC150_L_FD]   = FALSE; lamps[KFC150_L_ALT]  = FALSE;
        lamps[KFC150_L_HDG]  = FALSE; lamps[KFC150_L_GS]   = FALSE;
        lamps[KFC150_L_NAV]  = FALSE; lamps[KFC150_L_APR]  = FALSE;
        lamps[KFC150_L_BC]   = FALSE; lamps[KFC150_L_TRIM] = TRUE;
        lamps[KFC150_L_AP]   = FALSE;
        break;

      case KFC150_M_SELFTEST:
        if(dt>15.0) mode = KFC150_M_IDLE;
        if(dt<5.0)
        {
          lamps[KFC150_L_FD]   = TRUE; lamps[KFC150_L_ALT]  = TRUE;
          lamps[KFC150_L_HDG]  = TRUE; lamps[KFC150_L_GS]   = TRUE;
          lamps[KFC150_L_NAV]  = TRUE; lamps[KFC150_L_APR]  = TRUE;
          lamps[KFC150_L_BC]   = TRUE; lamps[KFC150_L_TRIM] = flash1;
          lamps[KFC150_L_AP]   = TRUE;
        }
        else
        {
          lamps[KFC150_L_FD]   = FALSE; lamps[KFC150_L_ALT]  = FALSE;
          lamps[KFC150_L_HDG]  = FALSE; lamps[KFC150_L_GS]   = FALSE;
          lamps[KFC150_L_NAV]  = FALSE; lamps[KFC150_L_APR]  = FALSE;
          lamps[KFC150_L_BC]   = FALSE; lamps[KFC150_L_TRIM] = FALSE;
          lamps[KFC150_L_AP]   = FALSE;
          if( (dt > 5.0) && (dt <=15.0) ) lamps[KFC150_L_AP]   = flash1;
          else lamps[KFC150_L_AP]   = FALSE;
        }
        break;
    }

    if(fd_mode==TRUE)
    {
      lamps[KFC150_L_FD] = TRUE;
      *fd_visible=TRUE;
    }
    else
    {
      lamps[KFC150_L_FD] = FALSE;
      *fd_visible=FALSE;
    }

    if(ap_mode ==TRUE) lamps[KFC150_L_AP]  = TRUE;
    if(alt_mode==TRUE) lamps[KFC150_L_ALT] = TRUE;
    if(hdg_mode==TRUE) lamps[KFC150_L_HDG] = TRUE;

    last_time = time;

}
#endif // FDL_HOME

