/*
SE: Seneca.cpp
*/

/*--------------------- 
This file Seneca Dynamics top level file
---------------------*/

/***************************/
/*   RCS Markers           */
/***************************/

/*
$Header: C:/Master/seneca/RCS/seneca.cpp 1.7 2000/06/16 10:08:01 ColinJ Exp $
$Log: seneca.cpp $

Revision 4.04 05/11/2010 BEATERIO
						 -MODIFICADO MAGNETOS

Revision 4.03 Añadido el sonido al hardware
  
Revision 1.7  2000/06/16 10:08:01  ColinJ
activated shutdown signals and tidy up.

Revision 1.6  2000/06/01 09:42:06  colinj
Moved keyboard input and test harness code to separate files.
Reset time_counter if > 0.01
In close_down, shutdown avionics sound and visuals and set
altimeters to zero.
If IOS sends shutdown, set state to DYN_HALT.

Revision 1.5  2000/05/30 16:29:24  colinj
Call IO_handler after setting up delta_time.
Changed keyboard inputs.

Revision 1.4  2000/05/14 08:24:24  colinj
Disabled altimeter demand height resets in close_down.
Rearranged ethernet_coms: IOS data now only sent if ether_ct != 2
and added test on ios_secs for going in to TRIM
New flag SE_turn_on_dcl.

Revision 1.3  2000/05/11 13:28:29  colinj
Added support for special move mode (SE_freeze_on).

Revision 1.2  2000/05/03 17:19:56  juliae
In close_down, call some exit routines to support
save of avionic data to file.

Revision 1.1  2000/04/14 17:06:12  colinj
Initial revision

*/

//----------------------------------------------------------------------

//----------------
// Include Files
//----------------

#include <time.h>
#include <stdio.h>
#include <string.h>
#include <conio.h>
#include <windows.h>



#include "define.h"

#include "nav\dyn_ios.h"
#include "faults.h"
#include "dat_stru\in_head.h"
#include "dat_stru\out_head.h"
#include "config.h"
#include "dyn_snd.h"
#include "fltdy\fmgdefs.h"
#include "version.h"

//#include "dat_stru\out_head.h"
//#include "dyn_snd.h"
//#include "fltdy\fmgdefs.h"
#include "sound\dynsnd.h"
#include "record.h"
//----------------
// Defines
//----------------
//#define DO_CLOSE
//----------------
// Typedefs
//----------------

//--------------------
// External Routines
//--------------------
extern void IO_handler(int state); // 
extern void reset_dynamics(void);

extern void initialise_ethernet(void); // init ethernet socket
extern void close_ethernet(void);      // shut down ethernet socket

//establish links with specific computers
extern int ET_init_hardware();
extern int ET_init_visual();
extern int ET_init_IOS();
extern int ET_init_sound();
extern int ET_init_screens();
extern int ET_init_screens1();
extern int ET_init_qtg();
extern int ET_init_SIMULINKoutputs();  //AÑADIDO POR MANOLO
extern int ET_init_SIMULINKscr();  //AÑADIDO POR MANOLO_12_04
extern int ET_init_SIMULINKhdw();  //AÑADIDO POR MANOLO_12_04
//extern int ET_init_SENDERvisual(); //AÑADIDO POR MANOLO_14_08

//exchange data with specific computers
extern void ET_ex_hardware_data();
extern void ET_ex_visual_data();
extern void ET_ex_ios_data();
extern void ET_ex_sound_data();
extern void ET_ex_screens_data();
extern void ET_ex_screens_data1();
extern void ET_ex_qtg_data();
extern int ET_ex_SIMULINKoutputs();  //AÑADIDO POR MANOLO
extern void ET_ex_SIMULINKscr();   //AÑADIDO POR MANOLO_12_04
extern void ET_ex_SIMULINKhdw();   //AÑADIDO POR MANOLO_12_04

extern void RC_read_config (void);
extern void N1A_exit();
extern void N2A_exit();
extern void ADF_exit();
extern void CA_exit();
extern void DYS_dynsnd(int);
extern char KB_decode_kb();

#ifdef HARNESS_INPUT
extern void HA_init_sen_kbd_in();
#endif
  
//----------------
// Externals
//----------------
extern sen_out_struct   *IO_Sen_out;
extern sen_in_struct    *IO_Sen_in;
extern OUTPUTin			*OUTPUTinbox;//simulinkudp
extern dyn_to_snd       *ET_tx_snd;
extern ios_to_dyn       *ET_ios_data_p;
extern record_struct    IO_rec_sen_out[];
extern int avail_time;
extern int type;
extern int RC_closedown;
//----------------
// Globals
//----------------

sen_config    SE_config;
 //AÑADIDO POR MANOLO
//int testvalue1 = 1;
//int testvalue2 = 1;
//int testvalue3 = 1;


float delta_time     = 0.0;
float d_time         = 0.0;
float eth_delta_time = 0.0;
float SE_back_track  = 0.0;//18052.0;//18052;
float SE_dt          = 0.0;


long SE_seconds           ;

unsigned int ios_secs = 0;

int IO_save_state  = FALSE;
int dyn_state      = INIT;
int ios_state_flag = DYN_HALT;
int SE_hz           = 0;
int qtg_testNum    = 1;
int SE_freeze_on       = FALSE;

char SE_exit_flag = FALSE;
//-----------------
// Local Variables
//-----------------

static time_t last_time;      // System clock value for previous pass
static time_t now;            // System clock value for this pass


static float time_counter = 0.0;


static long  frame_count = 0;

static int save_dyn_state  = DYN_HALT; // last sim state for h/w pausing
static int last_qtg_state  = 1;
static int ether_ct        = 0;

static char last_freeze = SW_OFF; //last position of c/pit freeze switch

//--------------------
// Forward References
//--------------------

int   time_delay(int targethz);
void timing( void );
void  command_line_decode (int argc, char *argv[]);
void  setup();
void  close_down();

static void Print_version();
static void ethernet_comms(void);

//----------------------------------------------------------------------

void main (int argc, char *argv[])
{

	//OUTPUTinbox->a = testvalue1;
	//OUTPUTinbox->b = testvalue2;
	//OUTPUTinbox->c = testvalue3;
	

#ifdef HARNESS_INPUT
   HA_init_sen_kbd_in();
#endif

   memset(&SE_config,0,sizeof(sen_config));
   command_line_decode(argc,argv);
   Print_version();   

   dyn_state = INIT;
   IO_handler(dyn_state);
   timing();
   initialise_ethernet();
   RC_read_config ();

   if(SE_config.visual == TRUE)
   {
      ET_init_visual();
	  //ET_init_SENDERvisual();
      printf("\n VISUAL ");
   }
   if(SE_config.instructor == TRUE)
   {
      ET_init_IOS();
      printf("\n INSTRUCTOR ");
   }
   if(SE_config.hardware == TRUE)
   {
      IO_Sen_out->io_operating_mode = HW_PROCESS;
      ET_init_hardware();
      printf("\n HARDWARE ");
   }
   if(SE_config.sound == TRUE)
   {
      ET_init_sound();
      printf("\n EXTERNAL SOUND");
   }
   
   if(SE_config.screens  == TRUE)
   {
      ET_init_screens();
	  ET_init_screens1();
      printf("\n SCREENS");
   }
   if(SE_config.senderr  == TRUE)
   {
	  /*testing*/ 
	  ET_init_SIMULINKoutputs(); //AÑADIDO POR MANOLO
	  ET_init_SIMULINKscr(); //AÑADIDO POR MANOLO
	  ET_init_SIMULINKhdw(); //AÑADIDO POR MANOLO_12_04
	  /*testing*/ 
      printf("\n SENDER");
   }
   while(!SE_exit_flag)
   {
      //Sleep(10);
      
      timing();
	  
	 


      time_counter += d_time;
      if(time_counter > 0.01)
      {
         time_counter = 0.0000;
      }

      
      if(time_counter > 0.006)
      {
         if ( kbhit() )
            KB_decode_kb();
         
         ethernet_comms();

         if(dyn_state == SHUTDOWN) 
         {
            SE_exit_flag = 1;
         }


      /*
      if(IO_Sen_in->system_freeze_button == SW_ON && last_freeze == SW_OFF && SE_freeze_on == FALSE)
      {
         SE_freeze_on = TRUE;
      }

      if(IO_Sen_in->system_freeze_button == SW_OFF && last_freeze == SW_ON && SE_freeze_on == TRUE)
      {
         ios_secs = avail_time;
         SE_freeze_on = FALSE;
      }
      last_freeze = IO_Sen_in->system_freeze_button;
      */
      if(IO_Sen_in->system_freeze_button != last_freeze)
      {
         if(SE_freeze_on == TRUE)
         {
            ios_secs = avail_time;         
            SE_freeze_on = FALSE;
         }
         else
            SE_freeze_on = TRUE;
      }

      last_freeze = IO_Sen_in->system_freeze_button;

         delta_time = time_counter;//d_time;
         time_counter = 0.000000000000;
         IO_handler(dyn_state);

      }   //End of (time_counter > 0.006)
         
   }         //end of main loop

   close_down();

}     //end of main




//****************************************************************************
//
//	T I M I N G    M O D U L E
//	--------------------------
// This module runs at the start of each free-time simulation frame
// It increments the frame counter, watches the main system clock and
// every second calculates how many frames were executed In the last second.
// Using this rate, the delta time is calculated
//
//****************************************************************************
void timing( void )
{

	now = time(0);

	++frame_count;



	if (now != last_time)
	{
		last_time   = now;

		SE_seconds++;
		SE_dt =(int) 1.0 / (float)frame_count; 						//Calc. time step between passes
		d_time = SE_dt;
		SE_hz = frame_count;
		//printf("\nFRAMES = %f", &SE_hz);
		//printf("\d_time = %f", &d_time);
		//printf("\nFRAMES = %f", &SE_hz);
		frame_count = 0;
	}


}

//*********************************************************************
//
//
//
//*********************************************************************
void command_line_decode(int argc, char *argv[])
{


int i;

	for (i=1; i<argc; i++)
	{
		if ( argv[i][0]=='-' && argv[i][1]=='V')
		{
         strcpy(SE_config.Vname,&argv[i][2]);
         SE_config.visual = TRUE;
		}
		else
		if ( argv[i][0]=='-' && argv[i][1]=='S')
		{
         strcpy(SE_config.Sname,&argv[i][2]);
         SE_config.sound = TRUE;
		}
		else
		if ( argv[i][0]=='-' && argv[i][1]=='H')
		{
         strcpy(SE_config.Hname,&argv[i][2]);
         SE_config.hardware = TRUE;
		}
		else
		if ( argv[i][0]=='-' && argv[i][1]=='I')
		{
         strcpy(SE_config.Iname,&argv[i][2]);
         SE_config.instructor = TRUE;
		}
		if ( argv[i][0]=='-' && argv[i][1]=='C')
		{
         strcpy(SE_config.Cname,&argv[i][2]);
         SE_config.screens  = TRUE;
		}
		if ( argv[i][0]=='-' && argv[i][1]=='X')   //AÑADIDO POR MANOLO
		{
         strcpy(SE_config.Xname,&argv[i][2]);      //AÑADIDO POR MANOLO
         SE_config.senderr  = TRUE;                //AÑADIDO POR MANOLO
		}
	}	//End of check command line arguments
   
}


//*********************************************************************
//        close down and clean up routine
//
//
//*********************************************************************
void close_down(void)
{
// shutdown sound computer
//#ifdef DO_CLOSE
if(RC_closedown)
{
   ET_tx_snd->sound_sDown = TRUE;
   ET_ex_sound_data();

   //set visual close down flag
   ET_ex_visual_data();
//#endif
}
   //set all altimeters to zero height
   IO_Sen_out->copilot_altimeter_demand_height = 0.0;
   IO_Sen_out->pilot_altimeter_1_demand_height = 0.0;
   IO_Sen_out->pilot_altimeter_2_demand_height = 0.0;
//#ifdef DO_CLOSE
if(RC_closedown)
   IO_Sen_out->io_operating_mode = HW_SHUTDOWN;
//#endif
   IO_Sen_out->system_DCL_power_control = UNLIT;
   ET_ex_hardware_data();

   close_ethernet();
   DYS_dynsnd(-1);
   N1A_exit();
   N2A_exit();
   ADF_exit();
   CA_exit();
   free(IO_Sen_in);
   free(IO_Sen_out);
   free(ET_tx_snd);
}

//*********************************************************************

static void Print_version()
{
   printf("\n**************************************************************************");
   printf("\n");
   printf("\n     UNIVERSIDAD DE CÁDIZ - ESI: Version %s, %s, %s", VERSION_STR, VERSION_DATE, VERSION_TIME);
   printf("\n     FTE-112 Seneca Dynamics");
   printf("\n");
   printf("\n**************************************************************************\n");
}

//*********************************************************************

void ethernet_comms(void)
{
	if(SE_config.sound == TRUE)
   {
      ET_ex_sound_data();
         
   }
   if(SE_config.screens  == TRUE)
   {
      ET_ex_screens_data();
	  ET_ex_screens_data1();
         
   }
   if(SE_config.senderr  == TRUE)  //AÑADIDO POR MANOLO
   {
	  /*testing*/
	  ET_ex_SIMULINKoutputs();   //AÑADIDO POR MANOLO
	  ET_ex_SIMULINKscr();   //AÑADIDO POR MANOLO_12_04
	  ET_ex_SIMULINKhdw();   //AÑADIDO POR MANOLO_12_04
	  /*testing*/
         
   }
   if(SE_config.hardware   == TRUE)
   {
      ET_ex_hardware_data();
   }

   if(ether_ct == 2) 
   {
      if(SE_config.visual     == TRUE)
      {
         ET_ex_visual_data();
      }
      ether_ct = 0;
   }
   else
   if(SE_config.instructor == TRUE)
   {
      ET_ex_ios_data();
      if(ios_state_flag != last_qtg_state)
      {
         if(ios_state_flag == DYN_RUN && ios_secs == -1)
         {
            dyn_state = DYN_TRIM;
         }
         else
            dyn_state = ios_state_flag;

         if(!RC_closedown)
         {
            if(ios_state_flag == SHUTDOWN)
            {
               dyn_state = DYN_HALT;
            }
         }

      }
      last_qtg_state = ios_state_flag;
   }

   ether_ct ++;

}

//****************************************************************************
//****************************************************************************
