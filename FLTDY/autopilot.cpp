//AP :autopilot.cpp


//--------------------- 
//This file controls seneca autopilot functions.
//autopilot routine is called from within MGK's flight model
//---------------------

//**************************
//   RCS Markers           *
//**************************


//$Header: C:/master/seneca/FLTDY/RCS/autopilot.cpp 1.1 2000/05/24 15:46:34 colinj Exp $
//$Log: autopilot.cpp $
//Revision 1.1  2000/05/24 15:46:34  colinj
//Initial revision
//


//----------------------------------------------------------------------

//----------------
// Include Files
//----------------

#include <stdio.h>

#include "fmgdefs.h"
#include "fms_s3fm.h"
#include "fms_defs.h"
#include "define.h"

#include "c:\bae-sims\seneca\dat_stru\in_head.h"
#include "c:\bae-sims\seneca\dat_stru\out_head.h"
//----------------
// Defines
//----------------
//#define apKcdi_dot  -50.0f
//#define apKcdi       -0.2f
//#define apKcdi_dot -250.0f
//#define apKcdi       -0.2f
//----------------
// Typedefs
//----------------

//--------------------
// External Routines
//--------------------

//----------------
// Externals
//----------------

extern sen_in_struct    *IO_Sen_in;
extern OUTPUTin			*OUTPUTinbox;//simulinkudp
extern sen_out_struct   *IO_Sen_out;

extern float delta_time;
extern float head_error;

extern int DY_init_ap;
//----------------
// Globals
//----------------
 
float auto_scale = 100.0f;
float debug_alt_error = 0.0f;
float gs_timer = 0.0f;
float apKcdi_dot = -65.0f;
float apKcdi     =  -0.03f;
float apKalt     =  -0.02f;
float debug_cdi_dot = 0.0f;
float debug_cdi_dot_dem = 0.0f;

int AP_beep = FALSE;
int nav_valid = FALSE;
int gs_valid = FALSE;
int last_gs_valid = FALSE;
char gs_couple = FALSE;
char gs_flash = FALSE;
char trim_warn = FALSE , last_trim_warn = FALSE;
char hdg_off = FALSE, last_hdg_off = FALSE;
char pitch_trim_on = FALSE;
char trim_caution = FALSE;
char fd_switch = FALSE;
bool ap_mode=FALSE;
//-----------------
// Local Variables
//-----------------
static float nav_hdg_err = 0.0f; //hdg_bug - psi;
static float timer_start=0.0,last_time;
//static 
static float theta_dem,alt_dem;
static float last_psi_error,last_theta_error,last_phi_error,last_alt_error,last_alt;

static float roll_servo_dem,pitch_servo_dem,hdot_dem;
//static 
static float last_cdi = 0.0;
static float last_gs_dev = 0.0;
static float init_nav_hdg = 0.0;
static float d_cdi = 0.0 ,last_d_cdi = 0.0;
//static float d_gs = 0.0 ,last_d_gs = 0.0;
static float i_psi_error,i_theta_error,i_phi_error,i_alt_error;
//static float nav_hdg_err = 0.0;
static float cdi_dot_dem = 0.0;
static float gs_dot_dem = 0.0;
static float alt_dot = 0.0f,alt_dot_dem = 0.0f;
static float alt1_error = 0.0f,last_alt1_error = 0.0f;
static int last_power_switch = FALSE;
int mode = KFC150_M_RESET;
static int button_state[16];
static int intercept_side = 2;
static char last_fd_switch;
static bool fd_mode=FALSE,  hdg_mode=FALSE, alt_mode=FALSE ,last_ap_mode = FALSE;
static bool flash1,flash2,dis_mode = FALSE;
static bool nav_mode = FALSE,nav_arm = FALSE,last_nav_mode = FALSE,apr_mode = FALSE,apr_arm = FALSE;
static bool bc_mode = FALSE,last_bc_mode = FALSE;


//--------------------
// Forward References
//--------------------

//----------------------------------------------------------------------

void KFC150_exec( float theta,
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

float dt,psi_error,theta_error,phi_error,alt_error;
float d_psi_error,d_alt_error;
float dtime;
float hdot,cdi_dot,gs_dot;
float roll_dem;

   
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
      last_power_switch = power_switch;
      fd_mode  = FALSE;
      alt_mode = FALSE;
      ap_mode  = FALSE;
      last_ap_mode  = FALSE;
      last_nav_mode = FALSE;
      hdg_mode = FALSE;
      nav_mode = FALSE;
      nav_arm = FALSE;
      apr_mode = FALSE;
      apr_arm = FALSE;
      bc_mode = FALSE;
      intercept_side = 2;
      fd_mode=FALSE;  hdg_mode=FALSE; alt_mode=FALSE ;last_ap_mode = FALSE;
      dis_mode = FALSE;
      nav_mode = FALSE;nav_arm = FALSE;last_nav_mode = FALSE;apr_mode = FALSE;apr_arm = FALSE;
      bc_mode = FALSE;last_bc_mode = FALSE;
      last_fd_switch = fd_switch;

      return;
    }
/*
    if(mode=KFC150_M_RESET)
    {
        fd_mode=FALSE;  hdg_mode=FALSE; alt_mode=FALSE ;last_ap_mode = FALSE;
        dis_mode = FALSE;
        nav_mode = FALSE;nav_arm = FALSE;last_nav_mode = FALSE;apr_mode = FALSE;apr_arm = FALSE;
        bc_mode = FALSE;last_bc_mode = FALSE;
    }
*/
    dtime = time-last_time;
    if (dtime<0.001) dtime=(float)0.001;

// If in first 0.1 seconds then do power-up reset
    //Whenever power is applied to autopilot and was not there before
    if(last_power_switch == FALSE && power_switch == TRUE)
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
      fd_mode  = FALSE;
      alt_mode = FALSE;
      ap_mode  = FALSE;
      hdg_mode = FALSE;
      nav_mode = FALSE;
      nav_arm = FALSE;
      intercept_side = 2;
    }

    if(DY_init_ap == TRUE)
    {
      roll_servo_dem  = 0.0;
      pitch_servo_dem = 0.0;
      mode = KFC150_M_IDLE; // power-up reset!
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
      fd_mode  = FALSE;
      alt_mode = FALSE;
      ap_mode  = FALSE;
      hdg_mode = FALSE;
      nav_mode = FALSE;
      nav_arm = FALSE;
      intercept_side = 2;
    }

   lamps[KFC150_L_GS] = FALSE;
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
        if(hdg_mode)
        {
           nav_arm  = FALSE;
           nav_mode = FALSE;
           apr_mode = FALSE;
           apr_arm  = FALSE;
        }
      }

    if(buttons[KFC150_B_NAV] == TRUE && button_state[KFC150_B_NAV] == FALSE /*&& nav == true*/)
    {
       if(nav_arm == TRUE || nav_mode == TRUE)
       {
          nav_arm  = FALSE;
          nav_mode = FALSE;
       }
       else
       {
          nav_arm = TRUE;
          apr_arm = FALSE;
          apr_mode = FALSE;
       }

//          nav_arm  = FALSE;
//          nav_mode = FALSE;

    }
    if(nav_arm == TRUE && nav_valid)
    {//nav mode goes from arm to on when deviation is less than 3 dots out
       if((cdi > - 0.60) && (cdi < 0.60))
       {
          nav_arm = FALSE;
          nav_mode = TRUE;
          hdg_mode = FALSE; // hdg mode goes off
       }
    }


    if(buttons[KFC150_B_APR] && !button_state[KFC150_B_APR])
    {
       if(apr_mode || apr_arm)
       {
          apr_mode = FALSE;
          apr_arm  = FALSE;
       }
       else
       {
          apr_arm = TRUE;
          gs_couple = TRUE;
          gs_timer = 0.0f;
          nav_arm  = FALSE;
          nav_mode = FALSE;
       }
          //apr_mode = !apr_mode;
    }
    if(apr_arm == TRUE  && nav_valid)
    {//nav mode goes from arm to on when deviation is less than 3 dots out
       if((cdi > - 0.60) && (cdi < 0.60))
       {
          apr_arm = FALSE;
          apr_mode = TRUE;
          hdg_mode = FALSE; // hdg mode goes off
       }
    }

    if(buttons[KFC150_B_BC] && !button_state[KFC150_B_BC])
    {
       if(bc_mode)
       {
          apr_mode = FALSE;
          apr_arm  = FALSE;
          bc_mode  = FALSE;
       }
       else
       {
          bc_mode = TRUE;
          apr_arm = TRUE;
          gs_couple = FALSE;
          gs_timer = 0.0f;
       }

    }

    if(buttons[KFC150_B_VT]== +1 || buttons[KFC150_B_VT]== -1 || gs_timer > 6.0f)
    {
       gs_couple = FALSE;
    }

    if(gs_couple == TRUE && last_gs_valid && !gs_valid)
    {
       gs_timer += delta_time;
    }
    else
    if(gs_timer > 0.0f)
       gs_timer += delta_time;

    if(gs_timer < 6.0f && gs_timer > 0.0f)
    {
       gs_flash = TRUE;
    }
    else
       gs_flash = FALSE;

    last_gs_valid = gs_valid;
    
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
    

    if(buttons[KFC150_B_CWS] == TRUE)// && button_state[KFC150_B_CWS] == FALSE)
    {
       fd_mode  = TRUE;
       if(alt_mode)
       {
          alt_dem = alt;
          last_alt=alt;
          last_alt_error = 0;
          theta_dem = theta;
          i_theta_error = 0.0;
          i_phi_error   = 0.0;
          i_psi_error   = 0.0;
          i_alt_error   = 0.0;

       }
       else
          theta_dem = theta;

       //alt_mode = FALSE;
    }

    if(buttons[KFC150_B_AP_DIS] == TRUE || (fd_switch != last_fd_switch) )
    {
       //fd_mode = FALSE;
       ap_mode = FALSE;
	   fd_mode = FALSE; //MODIFICADO POR MI Añadido FD,ALT,HDG
	   alt_mode = FALSE;
	   hdg_mode = FALSE;

    }


    if(buttons[KFC150_B_FD]==TRUE) // button pressed?
      if (button_state[KFC150_B_FD]==FALSE) // if was not pressed last frame
      {
        fd_mode=!fd_mode; // then change mode
        if(!fd_mode)
        {
          alt_mode = FALSE;
          hdg_mode = FALSE;
          nav_mode = FALSE;
          nav_arm = FALSE;
          apr_mode = FALSE;
          apr_arm = FALSE;
          bc_mode = FALSE;
        }
        else
        theta_dem = theta;
      }

    

    if(buttons[KFC150_B_AP_ENG]==TRUE && fd_mode == TRUE && mode == KFC150_M_IDLE) // button pressed?
      if (button_state[KFC150_B_AP_ENG]==FALSE) // if was not pressed last frame
      {
        ap_mode=!ap_mode; // then change mode

        i_theta_error = 0.0;
        i_phi_error   = 0.0;
        i_psi_error   = 0.0;
        i_alt_error   = 0.0;

      }
      if( dis_mode || fd_mode == FALSE || mode != KFC150_M_IDLE || (trim_warn && !last_trim_warn)|| (hdg_off && !last_hdg_off) || pitch_trim_on)
      {
         ap_mode = FALSE;
      }

    
      if(trim_warn && !last_trim_warn)
      {
         if(mode == KFC150_M_IDLE)
         trim_caution = TRUE;
      }



    last_trim_warn = trim_warn;
    last_hdg_off   = hdg_off;
    

    if(buttons[KFC150_B_VT]==+1)
    {
      theta_dem += (float)(0.9*D2R*dtime);
      alt_dem   += (float)((500.0/60.0)*0.3048*dtime);
    }
    if(buttons[KFC150_B_VT]==-1)
    {
      theta_dem -= (float)(0.9*D2R*dtime);
      alt_dem   -= (float)((500.0/60.0)*0.3048*dtime);
    }


    if(hdg_off)
    {

       hdg_mode = FALSE;
       nav_mode = FALSE;
       nav_arm  = FALSE;
       apr_mode = FALSE;
       apr_arm  = FALSE;
       bc_mode  = FALSE;
    }

    if(nav_mode || apr_mode)
    {
       //hdg_err = 0.0; //reset hdg_err

       if(last_nav_mode == FALSE) //first time so get heading
       {
          nav_hdg_err = (float)0.0;
          init_nav_hdg = (float)(psi * R2D);
          if(init_nav_hdg < 0.0) init_nav_hdg += 360.0;
          
          d_cdi = 0.0;
          last_d_cdi = 0.0;
          last_cdi = cdi;
          gs_dot     = 0.0f;
          gs_dot_dem = 0.0f;

          if(cdi > 0.0)
          {
             intercept_side = TRUE;
          }
          else
             intercept_side = FALSE;
       }

       if(bc_mode)
       {
          cdi = -cdi;
       }
 
       if(nav_valid)
       {
          cdi_dot     = (float) (((cdi - last_cdi)/dtime));
          cdi_dot_dem = (float) (apKcdi * cdi);
          if (cdi>0.0)
          {

            debug_cdi_dot = cdi_dot;
            debug_cdi_dot_dem = cdi_dot_dem;
            roll_dem    = (float) (+apKcdi_dot * (cdi_dot_dem - cdi_dot));
            if((cdi_dot < 0.05 && cdi_dot > -0.05) && (cdi > 0.20f || cdi < -0.20f))
               roll_dem = 0.0f;


          }
          else
          {
            debug_cdi_dot = cdi_dot;
            debug_cdi_dot_dem = cdi_dot_dem;

            roll_dem    = (float) (+apKcdi_dot * (cdi_dot_dem - cdi_dot));
            if((cdi_dot < 0.05 && cdi_dot > -0.05) && (cdi > 0.20f || cdi < -0.20f))
               roll_dem = 0.0f;
          }
       }
       else
          roll_dem = 0.0f;

       if(apr_mode && !bc_mode)
       {

          if(gs_valid &&(gs_dev < 25.0f && gs_dev > -25.0f)) //DO NOT CAPTURE GLIDESLOP UNTILL WITHIN 25% OF 0
          {
             gs_dot     = (float) (((gs_dev - last_gs_dev)/dtime));
             gs_dot_dem = (float) (apKcdi * gs_dev);
             if (gs_dev>0.0)
             {
                alt_error = (float) (+apKcdi_dot * (gs_dot_dem - gs_dot));
                alt_error = alt_error/2.0f;
                debug_alt_error = alt_error;
             }
             else
             {
                alt_error = (float) (+apKcdi_dot * (gs_dot_dem - gs_dot));
                alt_error = alt_error/2.0f;
                debug_alt_error = alt_error;
             }
             if(gs_dev < 25.0f && gs_dev > -25.0f)
             {
                lamps[KFC150_L_GS] = TRUE;
                if(!bc_mode)
                   gs_couple = TRUE;
                gs_timer = 0.0f;
             }

          }
          else
            if(gs_couple == FALSE)
             alt_error = 0.0f;

          if(alt_error > 150.0f)
             alt_error = 150.0f;
          if(alt_error < -150.0f)
             alt_error = -150.0f;
       }
       

    }

    last_nav_mode = nav_mode || apr_mode;
    last_cdi = cdi;
    last_gs_dev = gs_dev;

    if(nav_mode || hdg_mode || apr_mode)
    {/*
       if(nav_mode)
       {
          psi_error = nav_hdg_err;
       }
       else*/
          psi_error = hdg_err; //hdg_bug - psi;
    }
    
    if (psi_error < -PI) psi_error += (float)PI;
    if (psi_error > +PI) psi_error -= (float)PI;
    i_psi_error += psi_error*dtime;
    d_psi_error = (psi_error-last_psi_error)/dtime;
    last_psi_error = psi_error;

    if(hdg_mode)// || nav_mode)
       roll_dem = (ap_Kpsi_p * psi_error);
    else if (!nav_mode && !apr_mode)
       roll_dem = 0.0;
/*
    if (nav_mode || apr_mode)
    {
       if(head_error > 45.0f)
          roll_dem = 0.0f;

       
    }*/
//    else
//    if(nav_mode)
//    {
//       roll_dem = (apKcdi_dot * cdi_dot);
//    }
//    else
//       roll_dem = 0.0;

    if (roll_dem > +ap_phi_max) roll_dem = +ap_phi_max;
    if (roll_dem < -ap_phi_max) roll_dem = -ap_phi_max;
        
      phi_error = phi- roll_dem;


    i_phi_error+=phi_error*dtime;

    *fd_roll = phi_error;


    if(alt_mode)
    {
      if(!(apr_mode && (gs_dev < 25.0f && gs_dev > -25.0f)) || (gs_couple == TRUE && !gs_valid))
      {
         alt1_error = alt_dem - alt;

         alt_dot = (alt1_error-last_alt1_error)/dtime;
         alt_dot_dem = +apKcdi * alt1_error;

         alt_error = (+apKcdi_dot * (alt_dot_dem - alt_dot));
         alt_error = alt_error/2.0f;

         last_alt1_error = alt1_error;

         if(alt_error > 150.0f)
            alt_error = 150.0f;
         if(alt_error < -150.0f)
            alt_error = -150.0f;

      }
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
//    *fd_pitch = theta_dem;
//    theta_error=*fd_pitch-theta;
    if (theta_dem > ap_theta_max) theta_dem = ap_theta_max;
    if (theta_dem < ap_theta_min) theta_dem = ap_theta_min;

    theta_error= theta_dem-theta;
    *fd_pitch = theta_error; 
// Protect pitch integrator against windup
//    if((theta_error*ap_Ktheta_i)>0.0)
//      if((i_theta_error*ap_Ktheta_i) < +1.0) i_theta_error+=theta_error*dtime;
//    else
//      if((i_theta_error*ap_Ktheta_i) > -1.0) i_theta_error+=theta_error*dtime;
    i_theta_error+=theta_error*dtime;

    *pitch_trim_req  = 0;

    if(ap_mode==TRUE && buttons[KFC150_B_CWS] == FALSE)
    {
      roll_servo_dem  = (ap_Kphi_p   *   phi_error)   +
                        (ap_Kphi_i   * i_phi_error);
      pitch_servo_dem = (ap_Ktheta_p *   theta_error) +
                        (ap_Ktheta_i * i_theta_error);
      if (roll_servo_dem  > +1.0) roll_servo_dem  = +1.0;
      if (roll_servo_dem  < -1.0) roll_servo_dem  = -1.0;
      if (pitch_servo_dem > +1.0) pitch_servo_dem = +1.0;
      if (pitch_servo_dem < -1.0) pitch_servo_dem = -1.0;
      *roll_servo  = roll_servo_dem;
      *pitch_servo = pitch_servo_dem;
      if (*pitch_servo < -0.20) *pitch_trim_req = -1;
      if (*pitch_servo > +0.20) *pitch_trim_req = +1;

    }




/*
    if(ap_mode==TRUE)
    {
      if (*pitch_servo < -0.25) *pitch_trim_req = -1;
      if (*pitch_servo > +0.25) *pitch_trim_req = +1;

    }
*/
    if(buttons[KFC150_B_TEST]==TRUE)
    {
      trim_caution = FALSE;
      if(mode != KFC150_M_IDLE)
      {
         mode = KFC150_M_SELFTEST;
         timer_start = time;
      }
    }

    if(last_ap_mode == TRUE && ap_mode == FALSE)
    {
       dis_mode = TRUE;
       timer_start = time;
    }

    if(!nav_valid) //if no valid nav signal all nav modes drop out
    {
       nav_mode = FALSE;
       nav_arm = FALSE;
       apr_mode = FALSE;
       apr_arm = FALSE;
       bc_mode = FALSE;
    }


    dt = time-timer_start;
    flash1 = TRUE;
    if((int)(dt*2.0*flash1_rate)&0x01) flash1 = FALSE;
    flash2 = TRUE;
    if((int)(dt*2.0*flash2_rate)&0x01) flash2 = FALSE;


    if(nav_arm == TRUE)
    {
       lamps[KFC150_L_NAV] = flash1;
    }
    else
       lamps[KFC150_L_NAV] = nav_mode;

    if(apr_arm == TRUE)
    {
       lamps[KFC150_L_APR] = flash1;
    }
    else
       lamps[KFC150_L_APR] = apr_mode;

    if(gs_flash)
    {
       lamps[KFC150_L_GS] = flash1;
    }

    lamps[KFC150_L_FD]   = fd_mode;
    *fd_visible          = lamps[KFC150_L_FD];
    lamps[KFC150_L_AP]   = ap_mode; 
    lamps[KFC150_L_ALT]  = alt_mode;
    lamps[KFC150_L_HDG]  = hdg_mode;
    lamps[KFC150_L_TRIM] = trim_warn;
    lamps[KFC150_L_BC]   = bc_mode;

    //lamps[KFC150_L_APR] = apr_mode;
    

    last_time = time;
    last_ap_mode = ap_mode;
    last_power_switch = power_switch;

   button_state[KFC150_B_CWS]    = buttons[KFC150_B_CWS];
   button_state[KFC150_B_FD]     = buttons[KFC150_B_FD];
   button_state[KFC150_B_AP_ENG] = buttons[KFC150_B_AP_ENG];
   button_state[KFC150_B_ALT]    = buttons[KFC150_B_ALT];
   button_state[KFC150_B_HDG]    = buttons[KFC150_B_HDG];
   button_state[KFC150_B_NAV]    = buttons[KFC150_B_NAV];
   button_state[KFC150_B_APR]    = buttons[KFC150_B_APR];
   button_state[KFC150_B_BC]     = buttons[KFC150_B_BC];
    switch(mode)
    {
      case KFC150_M_RESET:
        lamps[KFC150_L_FD]   = FALSE; lamps[KFC150_L_ALT]  = FALSE;
        lamps[KFC150_L_HDG]  = FALSE; lamps[KFC150_L_GS]   = FALSE;
        lamps[KFC150_L_NAV]  = FALSE; lamps[KFC150_L_APR]  = FALSE;
        lamps[KFC150_L_BC]   = FALSE; lamps[KFC150_L_TRIM] = TRUE;
        lamps[KFC150_L_AP]   = FALSE; *fd_visible          = FALSE;
        break;

      case KFC150_M_SELFTEST:
        if(dt>11.0) mode = KFC150_M_IDLE;
        if(dt<5.0)
        {
          lamps[KFC150_L_FD]   = TRUE; lamps[KFC150_L_ALT]  = TRUE;
          lamps[KFC150_L_HDG]  = TRUE; lamps[KFC150_L_GS]   = TRUE;
          lamps[KFC150_L_NAV]  = TRUE; lamps[KFC150_L_APR]  = TRUE;
          lamps[KFC150_L_BC]   = TRUE; lamps[KFC150_L_TRIM] = flash1;
          lamps[KFC150_L_AP]   = TRUE; *fd_visible          = TRUE;
        }
        else
        {
          lamps[KFC150_L_FD]   = FALSE; lamps[KFC150_L_ALT]  = FALSE;
          lamps[KFC150_L_HDG]  = FALSE; lamps[KFC150_L_GS]   = FALSE;
          lamps[KFC150_L_NAV]  = FALSE; lamps[KFC150_L_APR]  = FALSE;
          lamps[KFC150_L_BC]   = FALSE; lamps[KFC150_L_TRIM] = FALSE;
          lamps[KFC150_L_AP]   = FALSE; *fd_visible          = FALSE;
          if( (dt > 5.0) && (dt <=11.0) )
          {
             lamps[KFC150_L_AP]   = flash2;
             if(dt < 7.5)
                AP_beep = TRUE;
             else
                AP_beep = FALSE;
          }
          else
          {
             lamps[KFC150_L_AP]   = FALSE;
             AP_beep = FALSE;
          }
        }
        break;
    }

    if(dis_mode)
    {
       if(dt < 5.0)
       {
          lamps[KFC150_L_AP] = flash2;
          if(dt < 2.5)
             AP_beep = flash2;
       }
       else
          dis_mode = FALSE;
    }

    last_fd_switch = fd_switch;

}
