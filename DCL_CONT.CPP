//DC: dcl_cont.cpp


//--------------------- 
//This file controls force loading on flying controls
//---------------------

//**************************
//   RCS Markers           *
//**************************


//$Header: C:/MASTER/seneca/RCS/dcl_cont.cpp 1.4 2000/06/13 13:08:07 pete Exp $
//$Log: dcl_cont.cpp $
//Revision 1.4  2000/06/13 13:08:07  pete
//Reduced roll control force.
//
//Revision 1.3  2000/05/25 14:25:21  colinj
//Change way dcl is handled.
//
//Revision 1.2  2000/05/14 08:36:36  colinj
//On going updates.
//
//Revision 1.1  2000/04/14 16:00:01  colinj
//Initial revision
//

//----------------------------------------------------------------------

//----------------
// Include Files
//----------------

#include <stdio.h>
#include <math.h>

#include "faults.h"
#include "const.h"
#include "define.h"

#include "dat_stru\in_head.h"
#include "dat_stru\out_head.h"

#include "fltdy\fms_defs.h"
#include "fltdy\fmgdefs.h"
#include "fltdy\fmg.h"
#include "qtgdefs.h"


//----------------
// Defines
//----------------

#define MAX_RUDDER     35.0f // 45% gives 60 kg of force
#define MAX_RUDDER1    30.0f // give max force to dcl at low speed washes out as true dynamic force comes in
#define MAX_PITCH      25.0f // 25% gives 20 Kg of force
#define MAX_PITCH1     20.0f // give max force to dcl at low speed washes out as true dynamic force comes in
#define MAX_ROLL       15.0f // 70% gives 20 Kg of force
#define MIN_RUDDER     10.0f // MIN Value for dcl control
#define MIN_PITCH       3.0f // MIN Value for dcl control
#define MIN_ROLL        3.0f // MIN Value for dcl control
#define MAX_ROLL_DELTA 10.0f //limit autopilot roll speed
//#define RUDD_SCALE      2.0f
//----------------
// Typedefs
//----------------

//--------------------
// External Routines
//--------------------

//----------------
// Externals
//----------------
extern OUTPUTS outputs;
extern INPUTS  inputs;
extern INPUTS  qtg_inputs;
extern sen_in_struct   *IO_Sen_in;
extern OUTPUTin			*OUTPUTinbox;//simulinkudp
extern sen_out_struct  *IO_Sen_out;
extern sen_fault_struct IO_Sen_faults;

extern float RC_rudder_scale;
extern float RC_roll_scale;
extern float RC_pitch_scale;

extern float pitch_force_scale;
extern float roll_force_scale;
extern float rudder_force_scale;
extern float delta_time;
extern float cg_change;
extern float col_al;

extern int IO_elec_pwr;
extern int IO_avionic_pwr;
extern int set_cntrl;
extern int IOS_simtype;

extern bool ap_mode;
//----------------
// Globals
//----------------

//-----------------
// Local Variables
//-----------------
QTG              qtg1;
int   buff_speed = 10;
float buff_amount = 2.0f;
float buff_force = 50.0f;
float roll_deflection[4] = { -100.0, -15.0, 15.0, 100.0}; //deflection
float roll_force_grad[4] = {    3.0,   3.0,  3.0,   3.0}; // force grade to bob
float rud_err = 0.0f;
//float pitch_deflection[3] = {  0.0f,100.0f, 200.0f};// 90.0, 160.0, 200.0}; 
//float pitch_force_grad[3] = { 20.0f,100.0f, 100.0f};//,  60.0, 100.0};
float pitch_deflection[4] = {  0.0, 90.0, 160.0, 200.0}; 
float pitch_force_grad[4] = {  30.0, 35.0,  60.0, 100.0};

//float rudder_position[5] = {-100.0f,-25.0f, 0.0f,25.0f,100.0f};
//float rudder_force[5]    = {  40.0f, 25.0f, 5.0f,25.0f, 40.0f};
float rudder_error[2]  = {0.0f, 100.0f};
float rudder_force[2]  = {15.0f,  40.0f};

static float roll_dyn_pressure = 0.0;
static float dyn_pressure    = 0.0;
static float real_null_pos   = 0.0;
static float low_dyn_press   = 0.0;
static float low_speed       = 0.0;
static float pitch_err = 0.0;

static float rud_gnd_force[2] = { 1.0f,0.0f};
static float rud_gnd_speed[2] = {0.0f,80.0f};
static float rud_air_force[2] = { 0.0f,  1.0f};
static float rud_air_speed[2] = {30.0f,170.0f};
//static float dem_buff[4] = {   0.0f, 0.0f, 0.50f, 1.0f};
static float dem_buff[4] = {   0.0f, 0.0f, 0.80f, 1.5f};
static float alpha[4]    = {-100.0f,12.5f,13.5f,18.0f};
static float pitch_buff = 0.0f;
static float dir = 1.0f;
static float roll_null = 0.0f;

float rud_force_scale  = 0.0f;
float rud_force_scale1 = 0.0f;
float rud_force_scale2 = 0.0f;

static int   timer = 0;
static char last_stall = FALSE;
//--------------------
// Forward References
//--------------------

//----------------------------------------------------------------------
void DC_dcl_control(void)
{
   if(set_cntrl)
   {
      IO_Sen_out->roll_demand_position = 0.0f;
      roll_null = 0.0f;
   }

   dyn_pressure = outputs.user[Y_A_QBAR];
   if(dyn_pressure < 0.0) dyn_pressure = 0.0;
   if(dyn_pressure > 1.0) dyn_pressure = 1.0;

   low_dyn_press = (float)((outputs.Vtas * MS_TO_KTS)/40.0);
   if(low_dyn_press > 1.0) low_dyn_press = 1.0;

   //Roll control no autopilot in yet
    

//   IO_Sen_out->roll_demand_position = (float)(outputs.user[Y_A_ROLL_SERVO] * -20.0);
if(!set_cntrl)
{
   if(outputs.user[Y_A_ROLL_SERVO] > 0.1f)
   {
      IO_Sen_out->roll_demand_position -= delta_time * MAX_ROLL_DELTA;
      roll_null -= delta_time * MAX_ROLL_DELTA;
   }
   else
   if(outputs.user[Y_A_ROLL_SERVO] < -0.1f)
   {
      IO_Sen_out->roll_demand_position += delta_time * MAX_ROLL_DELTA;
      roll_null += delta_time * MAX_ROLL_DELTA;
   }

   if(!ap_mode)
   {/*
      if(roll_null > delta_time * MAX_ROLL_DELTA)
      {
         roll_null -= delta_time * MAX_ROLL_DELTA;
      }
      else
      if(roll_null < delta_time * -MAX_ROLL_DELTA)
      {
         roll_null += delta_time * MAX_ROLL_DELTA;
      }
      else*/
         roll_null = 0.0f;
   }


   
}

   
if(outputs.user[Y_A_PEDAL_NULL] < 0.05 && outputs.user[Y_A_PEDAL_NULL] > -0.05)
{
//   printf("\n RUDDER TRIM %f RUDDER NULL %f",IO_Sen_in->rudder_trim_wheel,outputs.user[Y_A_PEDAL_NULL]);
   if(IO_Sen_in->rudder_pedal_position > 2.5 && IO_Sen_in->rudder_pedal_position < 10.5)
   {
      IO_Sen_out->rudder_demand_position = (float)(IO_Sen_in->rudder_pedal_position - 4.5);//((delta_time * (outputs.Vtas * MS_TO_KTS)) * 5.0));
   }
   else   
      if(IO_Sen_in->rudder_pedal_position < - 2.5 && IO_Sen_in->rudder_pedal_position > -10.5)
      {
         IO_Sen_out->rudder_demand_position = (float)(IO_Sen_in->rudder_pedal_position + 4.5);//((delta_time * (outputs.Vtas * MS_TO_KTS)) * 5.0));
      }
      else
         IO_Sen_out->rudder_demand_position = 0.0;
}
else
   IO_Sen_out->rudder_demand_position = (float)(low_dyn_press * (outputs.user[Y_A_PEDAL_NULL] * 100.0));
   //rudder demand force

/*      
        rud_gnd_force = (float) (MAX_RUDDER1 * (40.0 - (outputs.Vtas * MS_TO_KTS)) * 0.025);
		  if(rud_gnd_force < 0.0) rud_gnd_force = 0.0;
		  rud_air_force = (float) (MAX_RUDDER  * (dyn_pressure * (IO_Sen_in->rudder_pedal_position - IO_Sen_out->rudder_demand_position)));
        IO_Sen_out->rudder_demand_force = rud_gnd_force + rud_air_force;
//low_dyn_press;
      if(IO_Sen_out->rudder_demand_force < 0.0) IO_Sen_out->rudder_demand_force = (float)(IO_Sen_out->rudder_demand_force * -1.0);

      if(IO_Sen_out->rudder_demand_force < MIN_RUDDER) IO_Sen_out->rudder_demand_force = MIN_RUDDER;
      if(IO_Sen_out->rudder_demand_force > MAX_RUDDER1) IO_Sen_out->rudder_demand_force = MAX_RUDDER1;
*/

    rud_err = (float)fabs((outputs.user[Y_A_PEDAL_NULL] * 100.0f) - IO_Sen_in->rudder_pedal_position);

    
    IO_Sen_out->rudder_demand_force = table1D(rudder_error,rudder_force,2,rud_err);

    rud_force_scale1 = table1D(rud_air_speed,rud_air_force,2,(float)(outputs.Vtas * MS_TO_KTS));
    rud_force_scale2 = table1D(rud_gnd_speed,rud_gnd_force,2,(float)(outputs.Vtas * MS_TO_KTS));

    rud_force_scale = rud_force_scale1 + rud_force_scale2;

    if(rud_force_scale > 1.0f) rud_force_scale = 1.0f;
    if(rud_force_scale < 0.0f) rud_force_scale = 0.0f;

    IO_Sen_out->rudder_demand_force = IO_Sen_out->rudder_demand_force * rud_force_scale;




//    IO_Sen_out->rudder_demand_position = (outputs.user[Y_A_PEDAL_NULL] * 100.0);
//    rudder_error[2]  = {0.0f, 100.0f};
//    rudder_force[2]  = {5.0f,  40.0f};

    



   




   //pitch demand position
   IO_Sen_out->pitch_trim_demand_force = 0;
//stop trim wheel operation if no elec power or yoke wheel autopilot disconect button depresed

if(IO_Sen_in->MCB_pitch_trim_tripped == SW_ON && IO_avionic_pwr && IO_Sen_in->yokes_autopilot_disconnect_button == 
   SW_OFF && IO_Sen_in->MCB_autopilot_tripped == SW_ON || IO_Sen_faults.AVF.auto_pilot)
{

   if(IO_Sen_faults.AVF.trim_runaway)
   {
      IO_Sen_out->pitch_trim_demand_position += 3.75;//4.0;
      IO_Sen_out->pitch_trim_demand_force = 100;
   }
   else
   {
      if(outputs.duser[Y_D_PITCH_TRIM_REQ] == TRUE)
      {
         IO_Sen_out->pitch_trim_demand_position = IO_Sen_in->pitch_trim_wheel;
         IO_Sen_out->pitch_trim_demand_position -= 1.75;//2.0;
         IO_Sen_out->pitch_trim_demand_force = 100;
     // IO_Sen_out->pitch_trim_demand_force = 0;//100;
      }
      else
      if(outputs.duser[Y_D_PITCH_TRIM_REQ] == -1)
      {
            IO_Sen_out->pitch_trim_demand_position = IO_Sen_in->pitch_trim_wheel;
            IO_Sen_out->pitch_trim_demand_position += 1.75;//2.0;
            IO_Sen_out->pitch_trim_demand_force = 100;        
             //IO_Sen_out->pitch_trim_demand_force = 0;//100;
      }

      if(IO_Sen_in->yokes_trim_down_button == SW_ON)
      {
         IO_Sen_out->pitch_trim_demand_position = IO_Sen_in->pitch_trim_wheel;
         IO_Sen_out->pitch_trim_demand_position -= 3;//4.0;//MODIFICADO POR MI 10_6_08 valor real 3.75
         IO_Sen_out->pitch_trim_demand_force = 100;
      }
      else
         if(IO_Sen_in->yokes_trim_up_button == SW_ON)
         {
            IO_Sen_out->pitch_trim_demand_position = IO_Sen_in->pitch_trim_wheel;
            IO_Sen_out->pitch_trim_demand_position += 3;//4.0;//MODIFICADO POR MI 10_6_08 valor real 3.75
            IO_Sen_out->pitch_trim_demand_force = 100;
         }
   }
}

if(IO_Sen_out->pitch_trim_demand_position > 100.0 ) 
{
   IO_Sen_out->pitch_trim_demand_position = 100.0;
   //IO_Sen_out->pitch_trim_demand_force = 0;
}

if(IO_Sen_out->pitch_trim_demand_position < -100.0 )
{
   IO_Sen_out->pitch_trim_demand_position = -100.0;
   //IO_Sen_out->pitch_trim_demand_force = 0;
}

      //scale pitch null position to give 0 to -2
      real_null_pos = (float)((outputs.user[Y_A_PITCH_NULL] - 1.0)* 100.0);
      //take off dyn_pressure
      low_speed = (float)(outputs.Vtas * MS_TO_KTS);

      low_dyn_press = (float)((low_speed)/40.0);

      if(low_dyn_press > 1.0) low_dyn_press = 1.0;
      
      roll_dyn_pressure = (float)((outputs.Vtas * MS_TO_KTS)/170.0);
if (qtg1.testid == 11)
{
      IO_Sen_out->pitch_demand_position = 20;
}else{
      IO_Sen_out->pitch_demand_position = (float)(100.0 + (real_null_pos * low_dyn_press));
}
      if(IO_Sen_out->pitch_demand_position > 90.0) IO_Sen_out->pitch_demand_position = 90.0;
      IO_Sen_out->pitch_demand_force = table1D(pitch_deflection,pitch_force_grad,4,((float)(outputs.Vtas * MS_TO_KTS)));

      //((float)(outputs.Vtas * MS_TO_KTS))

//Pitch buffet
/*
      if(!last_stall && outputs.duser[Y_D_STALL])
      {
         pitch_buff = buff_amount;
      }
*/
      if(!set_cntrl)
      {
         timer ++;// delta_time;

         if(timer > buff_speed)
         {
            timer = 0;
            //dem_buff = -dem_buff;
            dir = -dir;            
         }
         pitch_buff = table1D(alpha,dem_buff,4,(col_al * R2D));
         pitch_buff = pitch_buff * dir;
         if(IOS_simtype == WARRIOR)
            pitch_buff = 0.0f;
      }
      else
      {
         pitch_buff = 0.0f;
         IO_Sen_out->roll_demand_position = roll_null;
      }

      if(pitch_buff  < 0.01f && pitch_buff > -0.01f)
      {
         IO_Sen_out->roll_demand_position = roll_null;
      }

/*
      if((outputs.Vtas * MS_TO_KTS) < 30.0f)
      pitch_buff = 0.0f;
            pitch_buff = 0.0f;
  */       
//      }
/*
      else
      {
         dem_buff = 0.0f;
         pitch_buff = 0.0f;
         timer = 0;
      }
*/
      //pitch_buff = 0.0f;
      last_stall = outputs.duser[Y_D_STALL];

      //IO_Sen_out->pitch_demand_position += (pitch_buff);//cg_change * 2.0f);


      
      if(roll_dyn_pressure > 1.0) roll_dyn_pressure = 1.0;
      IO_Sen_out->roll_demand_force = table1D(roll_deflection,roll_force_grad,4,IO_Sen_in->control_yoke_roll_position);
      IO_Sen_out->roll_demand_force = (float)(IO_Sen_out->roll_demand_force * roll_dyn_pressure);
      if(ap_mode)
      {
         IO_Sen_out->roll_demand_force = 100.0f;
         IO_Sen_out->pitch_demand_force = 100.0f;
      }
      else
      {
         IO_Sen_out->roll_demand_position += (pitch_buff);
      }
/*
      if(IO_Sen_in->rudder_pedal_position > 0.0f)
      {
         IO_Sen_out->rudder_demand_force = IO_Sen_out->rudder_demand_force * RC_rudder_scale;
      }
*/
      IO_Sen_out->pitch_demand_force  = IO_Sen_out->pitch_demand_force * RC_pitch_scale;
      IO_Sen_out->roll_demand_force   = IO_Sen_out->roll_demand_force * RC_roll_scale;
      IO_Sen_out->rudder_demand_force = IO_Sen_out->rudder_demand_force * RC_rudder_scale;

}

void DC_qtgdcl_control(void)
{

   //IO_Sen_out->roll_demand_position   = inputs.inps[U_A_LAT] * -100.0f;
   IO_Sen_out->roll_demand_position   = outputs.user[Y_A_ROLL_SERVO] * -100.0f; //Woody Mod
if (qtg1.testid == 11)
{
	IO_Sen_out->pitch_demand_position  = 20;  //inputs.inps[U_A_LONG] * 100.0f;//AQUI
}else{
   IO_Sen_out->pitch_demand_position  = outputs.user[Y_A_PITCH_NULL] * 100.0f;  //inputs.inps[U_A_LONG] * 100.0f;//AQUI
}
   //IO_Sen_out->rudder_demand_position = inputs.inps[U_A_PED] * 100.0f;
   IO_Sen_out->rudder_demand_position = outputs.user[Y_A_PEDAL_NULL] * 100.0f; //Woody Mod

   rud_err = (float)fabs(IO_Sen_out->rudder_demand_position); 
   //roll_dyn_pressure = (float)((outputs.Vtas * MS_TO_KTS)/230.0);
   roll_dyn_pressure = (float)((outputs.Vtas * MS_TO_KTS)/170.0);      
   if(roll_dyn_pressure > 1.0) roll_dyn_pressure = 1.0;
   IO_Sen_out->roll_demand_force = table1D(roll_deflection,roll_force_grad,4,IO_Sen_in->control_yoke_roll_position);
   IO_Sen_out->roll_demand_force = (float)(IO_Sen_out->roll_demand_force * roll_dyn_pressure);
   
//   IO_Sen_out->roll_demand_force = 40.0f;
//   IO_Sen_out->roll_demand_force = 50.0f;

 /*  //if(IO_Sen_out->pitch_demand_position > 90.0) IO_Sen_out->pitch_demand_position = 90.0;
      pitch_err = fabs((IO_Sen_out->pitch_demand_position - IO_Sen_in->control_yoke_pitch_position));
      IO_Sen_out->pitch_demand_force = table1D(pitch_deflection,pitch_force_grad,3,pitch_err);

   roll_dyn_pressure = (float)((outputs.Vtas * MS_TO_KTS)/230.0);
   

//   IO_Sen_out->pitch_demand_force = 100.0f;
   IO_Sen_out->pitch_demand_force = IO_Sen_out->pitch_demand_force * roll_dyn_pressure;

   *///IO_Sen_out->pitch_demand_force = table1D(pitch_deflection,pitch_force_grad,4,((float)(outputs.Vtas * MS_TO_KTS)));
   IO_Sen_out->rudder_demand_force = table1D(rudder_error,rudder_force,2,rud_err);
    rud_force_scale1 = table1D(rud_air_speed,rud_air_force,2,(float)(outputs.Vtas * MS_TO_KTS));
    rud_force_scale2 = table1D(rud_gnd_speed,rud_gnd_force,2,(float)(outputs.Vtas * MS_TO_KTS));

    rud_force_scale = rud_force_scale1 + rud_force_scale2;

    if(rud_force_scale > 1.0f) rud_force_scale = 1.0f;
    if(rud_force_scale < 0.0f) rud_force_scale = 0.0f;

    IO_Sen_out->rudder_demand_force = IO_Sen_out->rudder_demand_force * rud_force_scale;

//   IO_Sen_out->rudder_demand_force = 0.0f;
   
   //IO_Sen_out->pitch_demand_position = (float)(100.0 + (real_null_pos * low_dyn_press));
   //if(IO_Sen_out->pitch_demand_position > 90.0) IO_Sen_out->pitch_demand_position = 90.0;
   IO_Sen_out->pitch_demand_force = table1D(pitch_deflection,pitch_force_grad,4,((float)(outputs.Vtas * MS_TO_KTS)));

   IO_Sen_out->pitch_trim_demand_position = outputs.user[Y_A_PITCH_NULL] * -100.0f;
   IO_Sen_out->pitch_trim_demand_force = 100;

   if(IO_Sen_in->yokes_trim_down_button == SW_ON)
   {
      IO_Sen_out->pitch_trim_demand_position = IO_Sen_in->pitch_trim_wheel;
      IO_Sen_out->pitch_trim_demand_position -= 3.75;//4.0;
      IO_Sen_out->pitch_trim_demand_force = 100;
   }
   else
   if(IO_Sen_in->yokes_trim_up_button == SW_ON)
   {
      IO_Sen_out->pitch_trim_demand_position = IO_Sen_in->pitch_trim_wheel;
      IO_Sen_out->pitch_trim_demand_position += 3.75;//4.0;
      IO_Sen_out->pitch_trim_demand_force = 100;
   }

//   IO_Sen_out->pitch_demand_force = 0.0f;

      if(IO_Sen_in->rudder_pedal_position > 0.0f)
      {
         IO_Sen_out->rudder_demand_force = IO_Sen_out->rudder_demand_force * RC_rudder_scale;
      }

      IO_Sen_out->pitch_demand_force  = IO_Sen_out->pitch_demand_force * RC_pitch_scale;
      IO_Sen_out->roll_demand_force   = IO_Sen_out->roll_demand_force * RC_roll_scale;
      IO_Sen_out->rudder_demand_force = IO_Sen_out->rudder_demand_force * RC_rudder_scale;

}