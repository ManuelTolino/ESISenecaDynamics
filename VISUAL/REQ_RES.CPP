/*
File: req_res.cpp
Prefix: 

Module: Dynamics for Seneca

Description: This file handles the setup and decode of information
             sent to and from the Visual Image generator.
*/

/***************************/
/*   Revision Record       */
/***************************/

/*
$Header: C:/Master/seneca/VISUAL/RCS/req_res.cpp 1.7 2000/06/16 11:37:44 ColinJ Exp $
$Log: req_res.cpp $
Revision 1.7  2000/06/16 11:37:44  ColinJ
Added yaw angle to eye offsets.

Revision 1.6  2000/05/31 16:27:11  juliae
Changed FOG_DIST_SCALE to 2.5.

Revision 1.5  2000/05/30 13:59:16  colinj
Support for showing crash. Set up Shutdown_code.
New general routine: Set_sph_obj_req.

Revision 1.4  2000/05/14 11:03:14  colinj
Apply a fog scale to fog ranges.
Changed eye height and pitch.

Revision 1.3  2000/05/09 08:43:08  juliae
Support for go_round obstacle on a switch.
Support for latency test objects (currently off).

Revision 1.2  2000/04/17 10:34:20  juliae
Allow for changing eyepoint offset.

Revision 1.1  2000/04/12 12:10:52  juliae
Initial revision

*/

/*---------------------------------------------------------------*/

//----------------
// Include Files
//----------------

#include <conio.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>

#include "c:\bae-sims\seneca\define.h"

#include "c:\bae-sims\seneca\fltdy\fmgdefs.h"
#include "c:\bae-sims\seneca\fltdy\fmg.h"
#include "c:\bae-sims\seneca\fltdy\fms_defs.h"

#include "c:\bae-sims\seneca\const.h"
#include "c:\bae-sims\seneca\visual\visual5.h"
#ifdef TEST_HARNESS_IOS
#include "c:\bae-sims\seneca\rw_data.h"
#endif

#include "c:\bae-sims\seneca\nav\dyn_ios.h"

//----------------
// Defines        
//----------------

#define SPH_COORD 1
#define DIRECT_RW_SETUP 1
//#define TEST_HARNESS_IOS 1

//Runways can be setup via the IOS or internally by QTG or to OFF
#define RW_OFF 0
#define RW_QTG 1
#define RW_IOS 2

#define HOST_ID 0
#define AIRCRAFT_OBJ_ID 2
#define OBSTACLE_SW_ID 20
#define LATENCY1_OBJ_ID 3   //Cross
#define LATENCY2_OBJ_ID 4   //Checkboard

//#define USE_TOD_VALUE    0x00   //Bits 0
//#define USE_TOD_SETTINGS 0x01
//#define TOD_NOON         0x00   //Bits 1 and 2
//#define TOD_DUSK         0x02
//#define TOD_NIGHT        0x04

#define NO_FOG_BITS      0x00   //Bits 0 and 1
#define THIN_FOG_BITS    0x01
#define MEDIUM_FOG_BITS  0x02
#define THICK_FOG_BITS   0x03
#define USE_FOG_VALUE    0x00   //Bit 2
#define USE_FOG_SETTINGS 0x04   //Bit 2
#define SW_ON_BIT        0x8000 //Bit 15
#define SW_ALL_BIT       0x4000 //Bit 14

#define FOG_DIST_TOL     0.01f  //Metres
#define FOG_DIST_SCALE   1.33f
#define CLD_VAL_TOL      0.01f  //Metres

#define DYN_DOING_FOG  1
#define DYN_DOING_CLD  1

#define DO_EYEPOINT    1

#define EYE_ABOVE_C_G  -0.1f
#define EYE_FWD_OF_C_G 1.5f
#define EYE_PITCH      4.0f
#define EYE_YAW        6.0f
#define DUMMY_EYE      100

#define DO_OBSTACLE 1
#define LATENCY_OBJ 1

#define DO_LT_INTENSITY 1

#define DO_WINDSOCK 1
#define WSOCK_ROT_ID 1     //id of local rotation for windsock in the database
#define MIN_WINDSOCK_LEVEL_SW 30
#define NUM_WINDSOCK_LEVELS   5
#define WIND_DIR_TOL          1.0f    //Dictates the wind change needed to update visual model (deg)
#define FULL_WIND_STRENGTH    30.0f   //Wind speed in knots for full strength of windsock

#define DO_SKY  1
#define NO_SKY  1
#define SKY_MODEL_HDIST    71611.183f     //In metres
#define SKY_MODEL_HT       28459.076f     //Height of peak   (vertical sides are 11319.836 high)

#define SAFE_NUM_REQ    20
#define WALL_SW_NO     502
#define DEBUG_RW 1
//#define DEBUG1  1
//--------------------
// Typedefs  
//--------------------

//--------------------
// External Routines  
//--------------------

extern void reset_dynamics(void);
extern void UT_ll_2_local_xy(double lat, double lon, double ref_lat, double ref_lon,
                             float *x, float *y);

//--------------------
// External Variables 
//--------------------

int set_screen=0;
int test_screen = FALSE;

extern char RTX_Data[];
extern char ET_ios_data_in[];

extern STATES states;
extern OUTPUTS outputs;
extern int dyn_state;
extern int QD_do_fog;
extern int QD_night;
extern float CS_magnetic_var;
extern float vis_runway_head;
extern float delta_time;
#ifdef TEST_HARNESS_IOS
extern rw_info RW_rway_info[];
extern int     RW_rw_index;
#endif

extern double QD_ref_lat;
extern double QD_ref_lon;

#ifdef DYN_DOING_FOG
extern float FC_vis;
#endif
#ifdef DYN_DOING_CLD
extern int FC_cld_on;
extern float FC_cld_base;
extern float FC_cld_thickness;
#endif

extern AtisData IOS_atis;
extern int IOS_vis_view;
extern int IOS_skybox;

extern float DY_Twind_dir;    //True heading of wind, in degrees
extern float DY_wind_speed;   //Knots

extern ATMOS atmosphere; //FGProtocol
extern AtisData IOS_atis; //FGProtocol
 
//----------------
// Globals
//----------------

// This needs to be reset each time we wish to change the
// visual runway
//TODO: ONLY use this when  dynamics wants to do this indep of IOS
int RR_runway_setup_ok = FALSE;

int RR_vis_db_loaded = FALSE;
int RR_tfb_ok = FALSE;

//Runways can be setup via the IOS or internally by QTG or to OFF
int RR_rw_setup_mode = RW_IOS;

float terrain_height = 0.0f;
float debug_angle = 4.0f;
float debug_angle1 = 3.0f;
//-----------------
// Local Variables    
//-----------------

unsigned char Shutdown_code = EXIT_GFX;

// We are using a single runway model each time, which
// gets repositioned. It has id 1 in the Visual database.
static int Rw_id = 1;
static int Rw_on_flag = FALSE;

static int Cframe_setup_ok = FALSE;

//Used to check the visual is acknowledging the right request
//Should change ONLY when we change content of request
static int Rw_ref_id = 10;
static int Cf_ref_id = 10;
static int Eye_ref_id = 10;
static int Wind_dir_ref_id = 10;

static int Tfb_vtx_set_ok = FALSE;
static int Num_host_tfb_vtx = 0;

int collision_detect= 0;
int screen_saver_on = 0;
int crash_on = 0;

static char *Buffer_ptr;  //Used with RTX_Data

#ifdef DYN_DOING_FOG
static float Last_vis = -10.0;
static int Vis_setup_ok = FALSE;
static int Vis_ext_send_count = 0;
#endif

#ifdef DYN_DOING_CLD
static int Last_cld_on = FALSE;
static float Last_cld_base = 0.0;
static float Last_cld_thickness = 0.0;
static int Cld_setup_ok = FALSE;
static int Cld_ext_send_count = 0;
#endif

#ifdef DYN_DOING_TOD
static Tod_setup_ok = FALSE;
float ColinosTODo = 12.00f;
#endif

static int Last_eyepoint = DUMMY_EYE;
static int Eye_setup_ok = FALSE;
static float Left_eye_xyz[3] = {-0.5f, EYE_FWD_OF_C_G, EYE_ABOVE_C_G};
static float Left_eye_hpr[3] = {EYE_YAW, EYE_PITCH, 0.0f};
static float Right_eye_xyz[3] = {0.5f, EYE_FWD_OF_C_G, EYE_ABOVE_C_G};
static float Right_eye_hpr[3] = {-EYE_YAW, EYE_PITCH, 0.0f};
static float Centre_eye_xyz[3] = {0.0f, EYE_FWD_OF_C_G, EYE_ABOVE_C_G};
static float Centre_eye_hpr[3] = {0.0f, EYE_PITCH, 0.0f};

static int Sw_setup_ok = FALSE;
#ifdef DO_OBSTACLE
static float Last_obstacle = FALSE;
static int Send_obstacle_count = 0;
#endif

#ifdef DO_SKY
static int Last_sky_id = NO_SKY;
static int Sky_setup_ok = FALSE;
static int Sky_ext_send_count = 0;
static float Sky_sc_width = 0.1f;
static float Sky_sc_dist = 0.1f;
static float Sky_sc_ht = 0.5f;

static float Sky_hrange = 30000.0;   //Metres   //TO COLIN: This is the variable to change to alter sky range
static float Sky_ht = 13000.0;       //Metres. Need max height here

int    IOS_sky_id;              //TO COLIN: TEMPORARY for test purposes. Should be replaced by extern variable
                                //eg. for cloudy backdrop, sunny backdrop, or none
#endif

static float Vis_rw_ht = 0.0f;
static int crash_count = 0;
static float wall_time = 0.0f;

static int Lt_intensity = 3;   //High intensity
#ifdef DO_LT_INTENSITY
static int Last_intensity = -1;
static int Int_setup_ok = FALSE;
#endif

#ifdef DO_WINDSOCK
static int Wind_strength = 2;
static int Last_wind_strength = 0;
static int Wind_level_setup_ok = FALSE;
static float Last_wind_dir = 0.0f;
static int Wind_dir_setup_ok = FALSE;
#endif
static float Rw_thead = 0.0f;   //Heading of current runway model

//--------------------
// Forward References 
//--------------------

FGProtocol FGOutput;  //FGPROTOCOL

int RR_encode_vis_buffer(void);
int RR_decode_vis_buffer(void);

static int Send_event( unsigned int ,unsigned char ,float );
static int Set_own_craft_req();
static int Set_sph_own_craft_req();
static int Set_cloud_req(int on, float base_ht, float thickness);
static int Set_time_of_day_req(unsigned char, float tod);
static int Set_fog_req(float vis);
static int Set_switch_req(int sw_id, int on, int all);
static void Terrain_feedback_response(RTX_tfb_res *);
static int Set_tfbv_req();
static int Set_srr_req();
static int Set_our_srr_req();
static int Set_test_runway_req();
static void Debug_vis_out();
static int EncodeIOSToVisualDirectly();
static int Set_sph_obj_off_req(int obj_id);
static int Set_obj_on_req(int obj_id);
static int Set_eye_req(float xyz[], float hpr[]);
static int Set_sky_req(int sky_id, float sc_width, float sc_dist, float sc_ht);
static int Set_intensity_req(int intensity);
static int Set_loc_rot_req(int id, int ref_id, float h, float p, float r);
static int Set_windsock_level_req(int wind_strength);
static int Get_wind_strength(float speed, float max_speed);
static void Print_rw_change(RTX_rw_setup_req *rw);

//----------------------------------------------------------------------

// format RTX_Data for send purposes. Return number bytes to send
//
// Inputs: RTX_Data

int RR_encode_vis_buffer(void)
{
   RTX_header head;
   int i;
   short t_length = 0;
   float delta;
   float rel_wind_dir;

//   Left_eye_hpr[0] = debug_angle;
//   Left_eye_hpr[1] = debug_angle1;

   Buffer_ptr = RTX_Data; // reset buffer ptr to base of buffer

   Buffer_ptr  += sizeof(RTX_header); // advance data buffer ptr past header section


#ifdef DIRECT_RW_SETUP   //Usually comes from IOS
#ifdef SPH_COORD
   /* We MUST check for appropriate response */
   if (!RR_runway_setup_ok && (RR_rw_setup_mode == RW_QTG))
   {
      t_length += Set_test_runway_req();
   }
#endif
#endif


#ifdef SPH_COORD
   t_length += Set_sph_own_craft_req();
#else
   t_length += Set_own_craft_req();
#endif


#ifdef DO_OBSTACLE
   //Since we do not currently get a response specifically for
   //switch requests, we send request several times.
   if (IOS_atis.valid && IOS_atis.GoRound)
   {
      if (!Last_obstacle)
         Sw_setup_ok = FALSE;
      Last_obstacle = TRUE;
   }
   else
   {
      if (Last_obstacle)
         Sw_setup_ok = FALSE;
      Last_obstacle = FALSE;
   }
   if (!Sw_setup_ok)
   {
      if (Last_obstacle)
      {
         t_length += Set_switch_req(OBSTACLE_SW_ID, TRUE, FALSE);
      }
      else
      {
         t_length += Set_switch_req(OBSTACLE_SW_ID, FALSE, FALSE);
      }

      Send_obstacle_count = SAFE_NUM_REQ;
   }
   else if (Send_obstacle_count)
   {
      if (Last_obstacle)
      {
         t_length += Set_switch_req(OBSTACLE_SW_ID, TRUE, FALSE);
      }
      else
      {
         t_length += Set_switch_req(OBSTACLE_SW_ID, FALSE, FALSE);
      }

      Send_obstacle_count--;
   }
#endif

#ifdef PLANE_MODEL
   //For now, turn off model of plane
   t_length += Set_sph_obj_off_req(AIRCRAFT_OBJ_ID);
#endif

#ifdef LATENCY_OBJ
   t_length += Set_sph_obj_off_req(LATENCY1_OBJ_ID);
   t_length += Set_sph_obj_off_req(LATENCY2_OBJ_ID);
#endif

//#ifdef LATENCY_OBJ
   if(test_screen)
   {
      Set_switch_req(WALL_SW_NO, TRUE,FALSE);
      wall_time = 0.0f;
   }
   else
      if(wall_time < 1.0f)
      {
         Set_switch_req(WALL_SW_NO, FALSE,FALSE);
         wall_time += delta_time;
      }
   /*
      t_length += Set_sph_obj_off_req(LATENCY1_OBJ_ID);
      t_length += Set_sph_obj_off_req(LATENCY2_OBJ_ID);
*/

//      t_length += Set_obj_on_req(LATENCY2_OBJ_ID);

//#endif

//ADDED BY PETE 24/08/00 - example of clound and fog setup follows TOD stuf
#ifdef DYN_DOING_TOD
   static float otod= -1.0f;
   // you may want a controlling variable set by dynamics or this may overide
   // normal operation conditions
   if (otod != ColinosTODo)
   {
      otod = ColinosTODo;
      Tod_setup_ok = FALSE;
   }
   if (Tod_setup_ok == FALSE) // is set to true on viz response
      t_length += Set_time_of_day_req(0 , ColinosTODo);// job done ! (thud)

#endif

   if(dyn_state == DYN_QTG)
   {
      if(QD_night == TRUE)
      {
         t_length += Set_time_of_day_req(0,1.0f);
      }
      else
      {
         t_length += Set_time_of_day_req(0 ,12.0f);
      }
   }


// the following code allows you to set cloud and fog attributes (set FC_vis,
// FC_cld_on, FC_cld_base, FC_cld_thickness - check with julia

#ifdef DYN_DOING_FOG
   delta = FC_vis - Last_vis;
   if (delta < 0.0)
      delta = -delta;
   if (delta > FOG_DIST_TOL)
      Vis_setup_ok = FALSE;
   if (!Vis_setup_ok)       //Includes previous changes not yet responded to
   {
      t_length += Set_fog_req(FC_vis);
      Last_vis = FC_vis;
      Vis_ext_send_count = 2;
   }
   else if (Vis_ext_send_count)
   {
      t_length += Set_fog_req(FC_vis);
      Last_vis = FC_vis;
      Vis_ext_send_count--;
   }
#endif

   if(dyn_state == DYN_QTG)
   {
      if(QD_do_fog)
      {
         t_length += Set_fog_req(365.85f);//1200.0f / FT_IN_METRE);//MODIFICADO POR MI ANTES 800
      }
      else
         t_length += Set_fog_req(50000.0f);
   }

#ifdef DYN_DOING_CLD
   if (Last_cld_on != FC_cld_on)
      Cld_setup_ok = FALSE;
   else
   {
      delta = FC_cld_base - Last_cld_base;
      if (delta < 0.0)
         delta = -delta;
      if (delta > CLD_VAL_TOL)
         Cld_setup_ok = FALSE;
      else
      {
         delta = FC_cld_thickness - Last_cld_thickness;
         if (delta < 0.0)
            delta = -delta;
         if (delta > CLD_VAL_TOL)
            Cld_setup_ok = FALSE;
      }
   }
   if (!Cld_setup_ok)       //Includes previous changes not yet responded to
   {
      t_length += Set_cloud_req(FC_cld_on, FC_cld_base, FC_cld_thickness);
      Last_cld_on = FC_cld_on;
      Last_cld_base = FC_cld_base;
      Last_cld_thickness = FC_cld_thickness;
      Cld_ext_send_count = 2;
   }
   else if (Cld_ext_send_count)
   {
      t_length += Set_cloud_req(FC_cld_on, FC_cld_base, FC_cld_thickness);
      Last_cld_on = FC_cld_on;
      Last_cld_base = FC_cld_base;
      Last_cld_thickness = FC_cld_thickness;
      Cld_ext_send_count--;
   }
#endif

#ifdef DO_EYEPOINT
   if (Last_eyepoint != IOS_vis_view)
   {
      Eye_setup_ok = FALSE;
      Eye_ref_id++;
      if (Eye_ref_id > 120)
         Eye_ref_id = 10;
   }
   if (!Eye_setup_ok)       //Includes previous changes not yet responded to
   {
      if (IOS_vis_view == PILOT1_VIEW)
         t_length += Set_eye_req(Left_eye_xyz, Left_eye_hpr);
      else if (IOS_vis_view == PILOT2_VIEW)
         t_length += Set_eye_req(Right_eye_xyz, Right_eye_hpr);
      else
         t_length += Set_eye_req(Centre_eye_xyz, Centre_eye_hpr);
      Last_eyepoint = IOS_vis_view;
   }
#endif

#ifdef DO_LT_INTENSITY
   if (Lt_intensity != Last_intensity)
   {
      Int_setup_ok = FALSE;
   }
   if (!Int_setup_ok)       //Includes previous changes not yet responded to
   {
      t_length += Set_intensity_req(Lt_intensity);
      Last_intensity = Lt_intensity;
   }
#endif

#ifdef DO_WINDSOCK
   /* There is a rotation part for the windsock and several switch levels for strength */
   /* Dont update direction is within tolerance of last one */
   /* Since the windsock is nested below the runway object (needs to be for positioning)
      and the runway object is rotated so that the y-axis is along the runway heading
      instead of north, need to compensate for runway heading when setting the rotation
      of the wind sock */
   rel_wind_dir = -(DY_Twind_dir - Rw_thead);

   if (fabs(rel_wind_dir - Last_wind_dir) > WIND_DIR_TOL)
   {
      Wind_dir_setup_ok = FALSE;
      Wind_dir_ref_id++;
      if (Wind_dir_ref_id > 120)
         Wind_dir_ref_id = 10;
   }
   if (!Wind_dir_setup_ok)
   {
      t_length += Set_loc_rot_req(WSOCK_ROT_ID, Wind_dir_ref_id, rel_wind_dir, 0.0f, 0.0f);
      Last_wind_dir = rel_wind_dir;
   }

   /* Convert wind speed in to a stength value */
   Wind_strength = Get_wind_strength(DY_wind_speed, FULL_WIND_STRENGTH);
   if (Wind_strength != Last_wind_strength)
   {
      Wind_level_setup_ok = FALSE;
   }
   if (!Wind_level_setup_ok)
   {
      t_length += Set_windsock_level_req(Wind_strength);
      Last_wind_strength = Wind_strength;
   }
#endif



#ifdef DO_SKY
   IOS_sky_id = IOS_skybox;      //TO COLIN: TEMPORARY assignment for test purposes

   if (Last_sky_id != IOS_sky_id)
   {
      Sky_setup_ok = FALSE;   //TO COLIN: Also set this to false to change scale only
   }
   if (!Sky_setup_ok)       //Includes previous changes not yet responded to
   {
      Sky_sc_width = Sky_hrange / SKY_MODEL_HDIST;
      Sky_sc_dist  = Sky_sc_width;
      Sky_sc_ht    = Sky_ht / SKY_MODEL_HT;

      t_length += Set_sky_req(IOS_sky_id, Sky_sc_width, Sky_sc_dist, Sky_sc_ht);
      Last_sky_id = IOS_sky_id;
      /* Set up an extra number of times to send this request
         after response has been received, since standard response test
         has the limitation of only checking the id.
         If the scales are never varied for the same sky id, this extra
         count can be set to 0 */
      Sky_ext_send_count = 5;
   }
   else if (Sky_ext_send_count)
   {
      t_length += Set_sky_req(IOS_sky_id, Sky_sc_width, Sky_sc_dist, Sky_sc_ht);
      Last_sky_id = IOS_sky_id;
      Sky_ext_send_count--;
   }
#endif

   if (set_screen == 1)
   {
      crash_on = 0;
      screen_saver_on = 1;
      collision_detect = 0;
   }

   if (set_screen == 0)
   {
      screen_saver_on = 0;
   }


//   if (!Tfb_vtx_set_ok)
   {
      t_length += Set_tfbv_req();
   }



   t_length += EncodeIOSToVisualDirectly();


// format general header
//-----------------------


   head.timestamp = 0.0f;// not used yet ?

   head.length    = t_length  ;// count of no DWORDS to follow header


   head.status    = 0;
   if (dyn_state == SHUTDOWN)
      head.status = Shutdown_code;

   if (screen_saver_on)
      head.status    |= SCREEN_SAVER_ON; // logic op here (set bit not all of status)

   if (outputs.duser[Y_D_CRASH] == TRUE)
   {
      crash_count = 200;
   }

   if (crash_count > 0)
   {
      crash_count --;
      head.status    |= CRASH_SEQUENCE_ON; // logic op here (set bit not all of status)
   }


   head.chksum    = 0;

   // calculate checksum
   Buffer_ptr   = RTX_Data+sizeof(RTX_header);     // set buffer ptr to base + header length

   for ( i = 0; i <  ( head.length * 4); ++i)
   {
      head.chksum += *Buffer_ptr++;
   }

   Buffer_ptr   = RTX_Data;       // reset buffer ptr to base of buffer
   memcpy( RTX_Data, &head , sizeof(RTX_header));

#ifdef DEBUG1
   Debug_vis_out();
#endif

   return (sizeof(RTX_header) + 4 * head.length); // num bytes to send
                                                  // either pass to function
                                                  // or recalculate from strored values
}

//----------------------------------------------------------------------

static void Debug_vis_out()
{
   char       *buffer;
   RTX_header *header;
   int        bytes_left;
   int        nbytes;

   printf("\n\nDebug_vis_out\n");
   header = (RTX_header * ) RTX_Data;

   // advance data buffer ptr past header section
   buffer = RTX_Data + sizeof(RTX_header);
   
   bytes_left = header->length * 4;

   while (bytes_left > 0)
   {
      nbytes = buffer[1] * 4;
      bytes_left = bytes_left - nbytes;
      if (bytes_left < 0)
         return;       /* Not enough bytes */

      switch ( (unsigned char) buffer[0] )
      {
         case OBJECT_CODE:
            printf("\nOBJECT_CODE, nwords = %d", buffer[1]);
            buffer += nbytes;
            break;

         case OBJECT_SPH_CODE:
            printf("\nOBJECT_SPH_CODE, nwords = %d", buffer[1]);
            buffer += nbytes;
            break;

         case RWAY_SETUP_CODE:
            printf("\nRWAY_SETUP_CODE, nwords = %d", buffer[1]);
            buffer += nbytes;
            break;

         case LOC_CFRAME_CODE:
            printf("\nLOC_CFRAME_CODE, nwords = %d", buffer[1]);
            buffer += nbytes;
            break;

         case EVENT_CODE:
            printf("\nEVENT_CODE, nwords = %d", buffer[1]);
            buffer += nbytes;
            break;

         case TFB_VTX_CODE:
            printf("\nTFB_VTX_CODE, nwords = %d", buffer[1]);
            buffer += nbytes;
            break;

         case SEARCH_RAY_CODE:
            printf("\nSEARCH_RAY_CODE, nwords = %d", buffer[1]);
            buffer += nbytes;
            break;

         case FOG_REQUEST:
            printf("\nFOG_REQUEST, nwords = %d", buffer[1]);
            buffer += nbytes;
            break;

         case CLOUD_LAYER_REQUEST:
            printf("\nCLOUD_LAYER_REQUEST, nwords = %d", buffer[1]);
            buffer += nbytes;
            break;

         case SWITCH_CODE:
            printf("\nSWITCH_CODE, nwords = %d", buffer[1]);
            buffer += nbytes;
            break;

         case GSWITCH_CODE:
            printf("\nGSWITCH_CODE, nwords = %d", buffer[1]);
            buffer += nbytes;
            break;

         case LOCAL_ROT_CODE:
            printf("\nLOCAL_ROT_CODE, nwords = %d", buffer[1]);
            buffer += nbytes;
            break;

         case GLOCAL_ROT_CODE:
            printf("\nGLOCAL_ROT_CODE, nwords = %d", buffer[1]);
            buffer += nbytes;
            break;

         default:
            printf("\nUNRECOGNISED opcode = %0x hex", buffer[0]);
            break;
      }
   }
}

//----------------------------------------------------------------------

// Return number of 32 bit words in request

static int Set_own_craft_req()
{
   RTX_object_req *obj;

   obj = (RTX_object_req*) Buffer_ptr;

   obj->opcode = OBJECT_CODE;         // unique id for an object request
   obj->length = 0x08;                // 8 DWORD
   obj->obj_id = HOST_ID;             // Own craft
   obj->spare  = 0;
   obj->flags  = ON_BIT;

   if (collision_detect)
      obj->flags |= COLL_BIT;

   //Turn on terrain feedback (the visual s/w is currently always
   //sending this anyway)

   obj->flags |= TFB_BIT;

   //Need to inform Visual when we reposition, so that Visual
   //knows to ignore changes from old to current position
   if ((dyn_state == DYN_HALT) || (dyn_state == DYN_TRIM))
      obj->flags |= REPOS_BIT;

   obj->x = (float)states.E;
   obj->y = (float)states.N;
   obj->z = (float)(-states.D);
   obj->p = (float)(outputs.theta*R2D);
   obj->r = (float)(outputs.phi*R2D);
   obj->h = (float)(outputs.psi*R2D);

   /*FGPROTOCOL*/
   //FGOutput.vis =(float)IOS_atis.Visibility ;
   FGOutput.theta =(float)outputs.theta * R2D ;
   FGOutput.psi =(float)outputs.psi * R2D ; 
   FGOutput.phi =(float)outputs.phi * R2D ;  
   //FGOutput.altitude=(double)-states.D * m2ft ;
   FGOutput.altitude=(double)-states.D * m2ft;
   FGOutput.latitude=(double)states.latitude * R2D ;  
   FGOutput.longitude=(double)states.longitude * R2D ; 
   /*FGPROTOCOL*/

   Buffer_ptr += sizeof(RTX_object_req); // advance data buffer ptr
   return ( obj->length );
}

//----------------------------------------------------------------------

// For specifying the own craft position in lat/lon to the Visuals.
// A coordinate frame reference MUST be sent to the Visuals
// before this (which gives a lat/lon to centre point of
// current database), either by a Local Coordinate frame
// request or a Runway Setup request.
// Return number of 32 bit words in request

static int Set_sph_own_craft_req()
{
   RTX_object_sph_req *sobj;

   sobj = (RTX_object_sph_req*) Buffer_ptr;

   sobj->opcode = OBJECT_SPH_CODE;
   sobj->length = 10;                  // 10 DWORD
   sobj->obj_id = HOST_ID;             // Own craft
   sobj->spare  = 0;
   sobj->flags  = ON_BIT;

   if (collision_detect)
      sobj->flags |= COLL_BIT;

   //Turn on terrain feedback (the visual s/w is currently always
   //sending this anyway)

   sobj->flags |= TFB_BIT;

   //Need to inform Visual when we reposition, so that Visual
   //knows to ignore changes from old to current position
   if ((dyn_state == DYN_HALT) || (dyn_state == DYN_TRIM))
      sobj->flags |= REPOS_BIT;

   sobj->lat = states.latitude * R2D;       //double
   sobj->lon = states.longitude * R2D;      //double
   sobj->ht  = (float) (-states.D);
   sobj->p   = (float) (outputs.theta * R2D);
   sobj->r   = (float) (outputs.phi * R2D);
   sobj->h   = (float) (outputs.psi * R2D);

   /*FGPROTOCOL*/
   //FGOutput.vis =(float)IOS_atis.Visibility ;
   FGOutput.theta =(float)outputs.theta * R2D ;
   FGOutput.psi =(float)outputs.psi * R2D ; 
   FGOutput.phi =(float)outputs.phi * R2D ;  
   //FGOutput.altitude=(double)-states.D * m2ft ;
   //FGOutput.altitude=(double)-states.D * m2ft;
   FGOutput.altitude=(double)-states.D;
   FGOutput.latitude=(double)states.latitude * R2D ;  
   FGOutput.longitude=(double)states.longitude * R2D ; 
   /*FGPROTOCOL*/

   Buffer_ptr += sizeof(RTX_object_sph_req); // advance data buffer ptr
   return ( sobj->length );
}

//----------------------------------------------------------------------

static int Set_sph_obj_req(int obj_id, int on_flag, double lat, double lon, float ht_asl,
                           float thead, float pitch, float roll)
{
   RTX_object_sph_req *sobj;

   sobj = (RTX_object_sph_req*) Buffer_ptr;

   sobj->opcode = OBJECT_SPH_CODE;
   sobj->length = 10;
   sobj->obj_id = obj_id;
   sobj->spare  = 0;
   if (on_flag)
      sobj->flags  = ON_BIT;
   else
      sobj->flags  = 0;

   sobj->lat = lat;     //Degrees
   sobj->lon = lon;
   sobj->ht  = ht_asl;  //Metres
   sobj->p   = pitch;   //Degrees
   sobj->r   = roll;
   sobj->h   = thead;


   Buffer_ptr += sizeof(RTX_object_sph_req); // advance data buffer ptr
   return ( sobj->length );
}

//----------------------------------------------------------------------

// For now turn off object 2

static int Set_sph_obj_off_req(int obj_id)
{
   RTX_object_sph_req *sobj;

   sobj = (RTX_object_sph_req*) Buffer_ptr;

   sobj->opcode = OBJECT_SPH_CODE;
   sobj->length = 10;
   sobj->obj_id = obj_id;
   sobj->spare  = 0;
   sobj->flags  = 0;

   sobj->lat = 0.0;
   sobj->lon = 0.0;
   sobj->ht  = -100.0;
   sobj->p   = 0.0;
   sobj->r   = 0.0;
   sobj->h   = 0.0;

   Buffer_ptr += sizeof(RTX_object_sph_req); // advance data buffer ptr
   return ( sobj->length );
}

//----------------------------------------------------------------------

// Turns an object on at Cartesian origin

static int Set_obj_on_req(int obj_id)
{
   RTX_object_req *obj;

   obj = (RTX_object_req*) Buffer_ptr;

   obj->opcode = OBJECT_CODE;
   obj->length = 8;
   obj->obj_id = obj_id;
   obj->spare  = 0;
   obj->flags  = ON_BIT;

   obj->x = 0.0;
   obj->y = 0.0;
   obj->z = 0.0;

   obj->p  += delta_time;
   obj->r   += delta_time;//0.0;
   obj->h  += delta_time;//0.0;

   if(obj->p > 360.0f)
      obj->p -= 360.0f;
   if(obj->r > 360.0f)
      obj->r -= 360.0f;
   if(obj->h > 360.0f)
      obj->h -= 360.0f;

   Buffer_ptr += sizeof(RTX_object_req); // advance data buffer ptr
   return ( obj->length );
}
//----------------------------------------------------------------------
#ifdef DIRECT_RW_SETUP

// Setup a Runway request.
// This is for use with spherical coordinate position updates.
// It also setups up a coordinate frame of reference.
// Inputs: RR_rw_setup_mode
//         RW_rw_index & RW_rway_info[] (only in TEST_HARNESS_IOS)

static int Set_test_runway_req()
{
   RTX_rw_setup_req *rw;
   int               rw_num = 20;
   float             rw_mhead = 204.0;
   float             rw_length = 2300.0;
   float             rw_width = 45.0;
   printf("\n setting TEST runway");

   rw = (RTX_rw_setup_req*) Buffer_ptr;

   //The reference id is used to check that the visual
   //has received the current request
   //It should be changed ONLY when the contents
   //of the runway setup change

   rw->opcode    = RWAY_SETUP_CODE;
   rw->length    = 10;
   // We are using a single runway model each time, which
   // gets repositioned. It has id 1 in the Visual database.
   rw->obj_id    = Rw_id;

   //Data specific to runway choice
   if (RR_rw_setup_mode == RW_QTG)
   {
      Rw_on_flag = TRUE;
      rw->on_off    = 1;
      rw->ref_id    = Rw_ref_id;
      //The runway number needs to correspond to MAGNETIC heading
      rw_num = (int) ((vis_runway_head + CS_magnetic_var)/ 10.0);
      rw->app_lt    = 2;
      rw->guid_lt   = 1;
      rw->rw_num    = rw_num;
      rw->lat       = QD_ref_lat;
      rw->lon       = QD_ref_lon;
      rw->ht        = 0.0;     // Ht above sea level
      rw->h         = vis_runway_head;   //TRUE heading
      rw->rw_length = rw_length;
      rw->rw_width  = rw_width;
   }
   else if (RR_rw_setup_mode == RW_OFF)
   {
      Rw_on_flag = FALSE;
      rw->on_off    = 0;   //Off
      rw->ref_id    = Rw_ref_id;
      rw->app_lt    = 0;
      rw->guid_lt   = 0;
      rw->rw_num    = 0;
      rw->lat       = 0.0;
      rw->lon       = 0.0;
      rw->ht        = 0.0;
      rw->h         = 0.0;
      rw->rw_length = 0.0;
      rw->rw_width  = 0.0;
   }
   else
   {
#ifdef TEST_HARNESS_IOS
      if (RW_rw_index == -1)
      {
         Rw_on_flag = FALSE;
         rw->on_off    = 0;   //Off
         rw->ref_id    = Rw_ref_id;
         rw->app_lt    = 0;
         rw->guid_lt   = 0;
         rw->rw_num    = 0;
         rw->lat       = 0.0;
         rw->lon       = 0.0;
         rw->ht        = 0.0;
         rw->h         = 0.0;
         rw->rw_length = 0.0;
         rw->rw_width  = 0.0;
      }
      else
      {
         Rw_on_flag = TRUE;
         rw->on_off    = 1;
         rw->ref_id    = Rw_ref_id;
         rw->app_lt    = RW_rway_info[RW_rw_index].app_lt;
         rw->guid_lt   = RW_rway_info[RW_rw_index].guid_lt;
         rw->rw_num    = RW_rway_info[RW_rw_index].rw_num;
         rw->lat       = RW_rway_info[RW_rw_index].rw_lat;
         rw->lon       = RW_rway_info[RW_rw_index].rw_lon;
         rw->ht        = RW_rway_info[RW_rw_index].rw_ht;
         rw->h         = RW_rway_info[RW_rw_index].rw_head;
         rw->rw_length = RW_rway_info[RW_rw_index].rw_length;
         rw->rw_width  = RW_rway_info[RW_rw_index].rw_width;
      }
#else
      return 0;
#endif
   }
   
   Buffer_ptr += sizeof(RTX_rw_setup_req); // advance data buffer ptr
   return ( rw->length );
}
#endif
//----------------------------------------------------------------------

static int Set_cloud_req(int on, float base_ht, float thickness)
{
   RTX_cloud_layer_req cl;

   cl.opcode = CLOUD_LAYER_REQUEST; // unique id for an object request
   cl.length = 0x02;                // 2 WORD (Generic ICD Rev 1.4)
   cl.spare  = 0;                   // set to zero anyway
   if (on)
      cl.flags  = 1;                // Always use cloud 1
   else
      cl.flags  = 0;
   cl.base_ht = (unsigned short) base_ht;
   cl.thickness = (unsigned short) thickness;

   memcpy( Buffer_ptr, &cl, sizeof(RTX_cloud_layer_req));

   Buffer_ptr += sizeof(RTX_cloud_layer_req);
   return cl.length;
}

//---------------------------------------------------------------

static int Set_time_of_day_req(unsigned char fl, float tod )
{
   RTX_time_of_day_req td;

   td.opcode = TIME_OF_DAY_REQUEST; // unique id for an object request
   td.length = 0x02;                // 2 WORDs
   td.spare  = 0;                   // set to zero anyway
   td.flags  = fl;                  // 0 to use tod value instead of pre-sets

   td.time   = tod;

   memcpy( Buffer_ptr, &td , sizeof(RTX_time_of_day_req));

   Buffer_ptr += sizeof(RTX_time_of_day_req);

   return td.length;
}

//---------------------------------------------------------------

static int Set_fog_req(float vis)
{
   RTX_fog_req *fog;

   fog = (RTX_fog_req*) Buffer_ptr;

   fog->opcode = FOG_REQUEST;
   fog->length = 2;                 // 2 WORD (Generic ICD Rev 1.4)
   fog->spare  = 0;                 // set to zero anyway
   fog->flags  = 1;                 // See ICD

   fog->vis    = vis * FOG_DIST_SCALE;

   Buffer_ptr += sizeof(RTX_fog_req);
   return fog->length;
}

//---------------------------------------------------------------

static int Set_switch_req(int sw_id, int on, int all)
{
   static i;
   RTX_switch_req sr;

   sr.opcode    = SWITCH_CODE;
   sr.length    = 0x01;           // 1 WORD
   sr.sw_id_state = sw_id;          // bits 0 - 14 is the switch id
                                  // bit 15 set/off means set switch on/off
                                  // bit 14 set means set ALL switches to the value of bit 15
   if (on)
      sr.sw_id_state |= SW_ON_BIT;
   if (all)
      sr.sw_id_state |= SW_ALL_BIT;
   memcpy( Buffer_ptr, &sr , sizeof(RTX_switch_req));

   Buffer_ptr += sizeof(RTX_switch_req);
   return sr.length;
}

//---------------------------------------------------------------

static int Send_event( unsigned int event_id, unsigned char flags, float value)
{
   // call a function called send event
   // request display of timers
   RTX_event_req event;

   event.opcode   = EVENT_CODE;
   event.length   = 3;    // 1 + 7*num_seg

   event.event_id = event_id; // unique id for the given event
                             // N.B. This is database dependent.
   event.flags    = flags ;    // 0 = off, 1 = on
   event.spare[0] = event.spare[1] = event.spare[2] = 0;
   event.value    = value;

   memcpy( Buffer_ptr, &event , sizeof(RTX_event_req));

   Buffer_ptr += sizeof(RTX_event_req);

   return event.length;
}

//---------------------------------------------------------------

// Note that this is only actually necessary ONCE, unless a
// change is required to the part of the own craft that you
// wish to get the height above terrain from. As soon as
// you receive terrain ht data back from the Visuals, you
// know it must have received this request.

static int Set_tfbv_req()
{
   RTX_tfb_vtx_req tfbv;

   tfbv.opcode   = TFB_VTX_CODE;
   tfbv.length   = 4;
   tfbv.obj_id   = HOST_ID;
   tfbv.num_vtx  = 1;
   Num_host_tfb_vtx = 1;   // Used as a check later

   tfbv.xyz[0].x = 0.0f;
   tfbv.xyz[0].y = 0.0f;
   tfbv.xyz[0].z = 0.0f;

   memcpy( Buffer_ptr, &tfbv , tfbv.length * 4 );
   Buffer_ptr  += tfbv.length * 4;

   return tfbv.length;
}

//----------------------------------------------------------------------

//Offsets are with respect to c of g of craft (the host object position)

static int Set_eye_req(float xyz[], float hpr[])
{
   RTX_eye_offset_req *eye;

   eye = (RTX_eye_offset_req*) Buffer_ptr;

   eye->opcode     = EYE_OFFSET_CODE;
   eye->length     = 8;
   eye->channel_id = ALL_CHANNELS;
   eye->ref_id     = Eye_ref_id;

   eye->spare      = 0;

   eye->x          = xyz[0];   //Right/left
   eye->y          = xyz[1];   //Forward/back
   eye->z          = xyz[2];   //Up/down
   eye->h          = hpr[0];   //Heading, deg, +ve clockwise
   eye->p          = hpr[1];   //Pitch, deg, +ve up
   eye->r          = hpr[2];   //Roll, deg

   Buffer_ptr += sizeof(RTX_eye_offset_req);
   return eye->length;
}

//----------------------------------------------------------------------

/* This "request" is an odd one out really. Intensity selection is
   done via switches. Selecting one intensity requires turning the
   other instensity switches off (other similar situations are handled
   by a visual config file knowing the exclusive nature of certain
   switches but this has not been done in this case).

   So this routine actually creates several opcode records and returns
   the total length.
   Runway light intensity switching requires use of "group switches"
   since there is one for each runway configuration in the database
   model.
   
   We assume the light intensity passes relates directly to the
   required group switch number in db. */

static int Set_intensity_req(int intensity)
{
   RTX_gswitch_req *gsw;
   int             i;
   int             num_int_sw = 3;

   /* There are currently 3 intensity levels. So turn on the one
      we want and turn the other 2 off. The following assumes
      the switches in the database are consecutive with no gaps. */

   for (i = 1; i <= num_int_sw; i++)
   {
      gsw = (RTX_gswitch_req*) Buffer_ptr;

      gsw->opcode      = GSWITCH_CODE;
      gsw->length      = 1;
      gsw->sw_id_state = i;
      if (i == intensity)
         gsw->sw_id_state |= SW_ON_BIT;

      Buffer_ptr += sizeof(RTX_gswitch_req);
   }
   return (gsw->length * num_int_sw);
}

//----------------------------------------------------------------------

/* Local rotations.
   Inputs: angles in degrees
*/

static int Set_loc_rot_req(int id, int ref_id, float h, float p, float r)
{
   RTX_loc_rot_req *lrot;

   lrot = (RTX_loc_rot_req*) Buffer_ptr;

   lrot->opcode     = LOCAL_ROT_CODE;
   lrot->length     = 8;
   lrot->id         = id;
   lrot->ref_id     = ref_id;

   lrot->h          = h;   //Heading, +ve clockwise
   lrot->p          = p;   //Pitch, +ve up
   lrot->r          = r;   //Roll

   Buffer_ptr += sizeof(RTX_loc_rot_req);
   return lrot->length;
}

//----------------------------------------------------------------------

/* This "request" is an odd one out really (like Set_intensity_req).
   The selection of "fullness" of windsock is done via switches.
   Selecting one requires turning the other choices off.

   So this routine actually creates several opcode records and returns
   the total length.
   
   Inputs: wind_strength, range 0 (min) to 5 (max)
*/

static int Set_windsock_level_req(int wind_strength)
{
   RTX_switch_req *sw;
   int             i;

   /* There are currently "NUM_WINDSOCK_LEVELS" state levels. So turn on
      the one we want and turn the other 4 off. The following assumes
      the switches in the database are consecutive with no gaps. */

   for (i = 0; i < NUM_WINDSOCK_LEVELS; i++)
   {
      sw = (RTX_switch_req*) Buffer_ptr;

      sw->opcode      = SWITCH_CODE;
      sw->length      = 1;
      sw->sw_id_state = i + MIN_WINDSOCK_LEVEL_SW;
      if (i == wind_strength)
         sw->sw_id_state |= SW_ON_BIT;

      Buffer_ptr += sizeof(RTX_switch_req);
   }
   return (sw->length * NUM_WINDSOCK_LEVELS);
}

//----------------------------------------------------------------------

//To setup a sky box model. Only need to do this when type/scale
//of model is changed. Once setup, the Visual software will
//do all the positional updates.

static int Set_sky_req(int sky_id, float sc_width, float sc_dist, float sc_ht)
{
   RTX_sky_req *sky;

   sky = (RTX_sky_req*) Buffer_ptr;

   sky->opcode      = SKY_CODE;
   sky->length      = sizeof(RTX_sky_req) / 4;

   //This is set, if we want the sky to be fixed with Host heading
   sky->move_with_h = 1;               //Current models are complete surrounds
   sky->sky_id      = sky_id;

   sky->scale_width = sc_width;
   sky->scale_dist  = sc_dist;
   sky->scale_ht    = sc_ht;

   Buffer_ptr += sizeof(RTX_sky_req);
   return sky->length;
}

//---------------------------------------------------------------
//---------------------------------------------------------------

// For visual data that has been sent from the IOS. Copy
// data to visual out buffer (pointed to by Buffer_ptr)
// Return number of 32 bit words
// Inputs:  ET_ios_data_in
//          Buffer_ptr
// Outputs: Buffer_ptr
//          contents of RTX_Data buffer

static int EncodeIOSToVisualDirectly()
{
   char       *ios_vis_ptr;
   int        dwords_left;
   int        num_dwords;
   int        nbytes;
   ios_to_dyn *spd;
   int        length = 0;   // Number of dwords added
   unsigned char opcode;
   RTX_rw_setup_req *rw;


   spd = (ios_to_dyn *) ET_ios_data_in;
   dwords_left = spd->vis_length;
   if (dwords_left == 0)   //No visual data
      return 0;

   ios_vis_ptr = ET_ios_data_in + sizeof(ios_to_dyn);

   // encode any IOS stuff not required by dynamics but directly required by visual


   while (dwords_left)
   {
      opcode = (unsigned char) ios_vis_ptr[0];
      num_dwords = ios_vis_ptr[1];
      dwords_left -= num_dwords;
      if (dwords_left < 0)
      {
         // Something wrong, so just use good data so far
         printf("\nERROR: Bad vis data length in IOS data");
         return length;
      }
      nbytes = num_dwords * 4;
      
      switch (opcode)
      {
         case RWAY_SETUP_CODE:
            RR_rw_setup_mode = RW_IOS;

            rw = (RTX_rw_setup_req*) ios_vis_ptr;

            //Setup our own ref_id
            rw->ref_id = Rw_ref_id;
            if (rw->on_off)
            {
               Vis_rw_ht = rw->ht;
               Rw_on_flag = TRUE;
               Rw_thead = rw->h;
#ifdef DEBUG_RW
               Print_rw_change(rw);
#endif
            }
            else
            {
               Vis_rw_ht = 0.0;
               Rw_on_flag = FALSE;
            }

            /* NEW: Light intensity. The IOS encodes the light intensity
               as the top 4 bits of the approach light value. The visual
               does not handle it like this, so remove this and send
               equivalent opcode control to visual. */
            Lt_intensity = ((rw->app_lt & 0xF0) >> 4) & 0x0F;
            rw->app_lt &= 0x0F;

            break;

/*
         case CLOUD_LAYER_REQUEST:
            printf("Cld req\n");
            break;

         case TIME_OF_DAY_REQUEST:
            printf("Tod  req\n");
            break;

         case FOG_REQUEST:
            printf("Fog req\n");
            break;
*/
         default:
            break;
      }


      memcpy( Buffer_ptr, ios_vis_ptr, nbytes);
         
      Buffer_ptr += nbytes;
      ios_vis_ptr += nbytes;
      length += num_dwords;
   }

   return length;
}

//---------------------------------------------------------------
//---------------------------------------------------------------

// Inputs: contents of RTX_Data

int RR_decode_vis_buffer()
{
   int             i,loop=0;
   unsigned short  length = 0;
   char            chksum,chksum2=0;
   RTX_header     *header;
   RTX_object_res         *obj;
   RTX_coll_res           *col_res;
   RTX_search_res         *ser_res;
   RTX_tfb_res            *tfb_res;
   RTX_Acknowledge_res    *ack;
   RTX_rw_setup_res       *rw;
   RTX_loc_cframe_res     *cf;
   RTX_eye_offset_res     *eye;
   RTX_gswitch_res        *gsw;
   RTX_switch_res         *sw;
   RTX_loc_rot_res        *lrot;
   RTX_sky_res            *sky;
   int                    id;

   Buffer_ptr     = RTX_Data;                // reset buffer ptr to base of buffer

   header         = (RTX_header * ) RTX_Data;

   chksum         = header->chksum;                 // used later to check data integrity
   header->length = header->length;

   Buffer_ptr     = (RTX_Data + sizeof(RTX_header));


   if (crash_on)
      --crash_on;


   while ( length < header->length )
   {
      ++loop;
//#ifdef DUMMY
      if ( (loop > MAX_PACKETSIZE) //a simple kbhit may cut short data processing
            && kbhit() )
      {
//       cprintf("Packet error ! ");
         //in case of corrupted data and loop out of control
         break;
      }
//#endif

      switch( (char)*Buffer_ptr )               // response object type
      {
         case OBJECT_CODE:
            // Usually this means an error in object request

            obj = (RTX_object_res *)  Buffer_ptr;

            if (obj->obj_id == HOST_ID)
            {
               if (obj->error == ERR_NO_TFB_VTX)
                  Tfb_vtx_set_ok = FALSE;
            }

            Buffer_ptr += obj->length*4;
            length     += obj->length;
            break;


         case COLL_RES_CODE:
            /*
               this will be recieved if gfx detects motion
               has transversed a polygon.
               when this is called do reset (if required)

               1. set collision_detect = 1
               2. set crash_on = 1
               3. screen_saver_on = 0
            */

            crash_on = 40;
            screen_saver_on = 0;
            collision_detect = 0;


            //  reset_dynamics();
            col_res = (RTX_coll_res *) Buffer_ptr;

            Buffer_ptr += col_res->length * 4;
            length     += col_res->length;
            dyn_state = DYN_HALT;


//          graphics_collision = TRUE;
            break;

         case TFB_RES_CODE :

            tfb_res = (RTX_tfb_res *) Buffer_ptr;

            Terrain_feedback_response(tfb_res);

            Buffer_ptr += tfb_res->length * 4;

            length     += tfb_res->length;
            break;


         case SEARCH_RES_CODE :

            ser_res = (RTX_search_res *) Buffer_ptr;

            Buffer_ptr += ser_res->length * 4;
            length     += ser_res->length;

            break;

         case ACKNOWLEDGE_RESPONSE :


            ack = (RTX_Acknowledge_res *) Buffer_ptr;

            Buffer_ptr += ack->length * 4;
            length     += ack->length;
            Tfb_vtx_set_ok = TRUE;   //Hmm...!
#ifdef DYN_DOING_FOG
            Vis_setup_ok   = TRUE;   //For now
#endif
#ifdef DYN_DOING_CLD
            Cld_setup_ok   = TRUE;   //For now
#endif
#ifdef DYN_DOING_TOD
            Tod_setup_ok   = TRUE;   //For now
#endif

            Sw_setup_ok   = TRUE;    //For now
            break;



         case RWAY_SETUP_CODE:

            // This acknowledges receipt of a runway setup request
            rw = (RTX_rw_setup_res *) Buffer_ptr;

            // If this response matches our last runway request
            // then stop sending request

            if (rw->err_ref_id == ERR_BAD_OBJECT)
               RR_runway_setup_ok = FALSE;
            else
            {
               if ((rw->obj_id == Rw_id) && (rw->err_ref_id == Rw_ref_id))
               {
                  RR_runway_setup_ok = TRUE;
                  if (Rw_on_flag)
                  {
                     //NB. This is only intended for use when we first go
                     //in to RUN mode, ie. we dont reset this
                     RR_vis_db_loaded = TRUE;

                     //The first valid TFB for this new runway will be
                     //received in the NEXT transmission, NOT THIS ONE,
                     //so make sure this case is called after TFB_RES_CODE

                  }
               }
            }
               

            Buffer_ptr += rw->length * 4;
            length     += rw->length;

            break;

         case LOC_CFRAME_CODE:

            // This acknowledges receipt of a local coord frame request
            cf = (RTX_loc_cframe_res *) Buffer_ptr;

            if (cf->ref_id == Cf_ref_id)
            {
               Cframe_setup_ok = TRUE;
               // Increment ref_id, ready for next time
               Cf_ref_id++;
               if (Cf_ref_id > 120)
                  Cf_ref_id = 100;
            }

            Buffer_ptr += cf->length * 4;
            length     += cf->length;

            break;

#ifdef DO_EYEPOINT
         case EYE_OFFSET_CODE:

            // This acknowledges receipt of an eyepoint request
            eye = (RTX_eye_offset_res *) Buffer_ptr;

            if (eye->ref_id == Eye_ref_id)
               Eye_setup_ok = TRUE;

            Buffer_ptr += eye->length * 4;
            length     += eye->length;

            break;
#endif

         case GSWITCH_CODE:

            // This acknowledges receipt of a group switch request
            gsw = (RTX_gswitch_res *) Buffer_ptr;

#ifdef DO_LT_INTENSITY
            if (gsw->sw_id_state & SW_ON_BIT)   //Only look at the active one
            {
               //Clear bits not relating to number
               id = gsw->sw_id_state & (0xFFFF - SW_ON_BIT - SW_ALL_BIT);
               if (id == Lt_intensity)
                  Int_setup_ok = TRUE;
            }
#endif

            Buffer_ptr += gsw->length * 4;
            length     += gsw->length;

            break;

         case SWITCH_CODE:

            // This acknowledges receipt of a switch request
            sw = (RTX_switch_res *) Buffer_ptr;

#ifdef DO_WINDSOCK
            if (sw->sw_id_state & SW_ON_BIT)   //Only look at the active one
            {
               //Clear bits not relating to number
               id = sw->sw_id_state & (0xFFFF - SW_ON_BIT - SW_ALL_BIT);
               if (id == Wind_strength + MIN_WINDSOCK_LEVEL_SW)
                  Wind_level_setup_ok = TRUE;
            }
#endif

            Buffer_ptr += sw->length * 4;
            length     += sw->length;

            break;

         case LOCAL_ROT_CODE:

            // This acknowledges receipt of a switch request
            lrot = (RTX_loc_rot_res *) Buffer_ptr;

#ifdef DO_WINDSOCK
            if (lrot->ref_id == Wind_dir_ref_id)
               Wind_dir_setup_ok = TRUE;
#endif

            
            //Buffer_ptr += sw->length * 4;
            Buffer_ptr += lrot->length * 4;
            //length     += sw->length;
            length     += lrot->length;

            break;

         case GLOCAL_ROT_CODE:
            length += Buffer_ptr[1];
            Buffer_ptr += Buffer_ptr[1] * 4;
            break;

#ifdef DO_SKY
         case SKY_CODE:

            // This acknowledges receipt of a sky request
            sky = (RTX_sky_res *) Buffer_ptr;

            //This test is no bullet proof because it does not
            //allow for changing the scale of the same id as before
            if (sky->sky_id == IOS_sky_id)
               Sky_setup_ok = TRUE;

            Buffer_ptr += sky->length * 4;
            length     += sky->length;

            break;
#endif

         default:

            printf("\nUnrecognised opcode %0x hex", Buffer_ptr[0]);

            // Update length even if we don't recognise opcode
            length += Buffer_ptr[1];
            Buffer_ptr += Buffer_ptr[1] * 4;

            break;
      }
   }


   Buffer_ptr   = (RTX_Data + sizeof(RTX_header)) ;                // reset buffer ptr to base of buffer

   for( i = 0; i <  4 * header->length; ++i)
      chksum2 += Buffer_ptr[i]; // calculate checksum

   //Should be checking chksum at start!

   return 0;
}

//---------------------------------------------------------------

static void Terrain_feedback_response(RTX_tfb_res *tfbr)
{
   if (tfbr->obj_id == HOST_ID)
   {
      terrain_height = tfbr->set[0].height;   //TEMP OVER-RIDE

      //Until I fix this properly
      terrain_height = Vis_rw_ht;
      if(RR_rw_setup_mode == RW_QTG)
         terrain_height = 0.0f;

      if (RR_vis_db_loaded)
         RR_tfb_ok = TRUE;

      // Since we have received terrain feedback
      // we know that the terrain feedback vertices
      // have been setup correctly (unless we have
      // changed them during the simulation in which
      // case we need a more careful check)

      if (tfbr->nb_vtx == Num_host_tfb_vtx)
         Tfb_vtx_set_ok = TRUE;
   }
}

//---------------------------------------------------------------

void RR_init_db_check()
{
   terrain_height = 0.0;
   RR_tfb_ok = FALSE;
   RR_vis_db_loaded = FALSE;
   //Change reference id so that we do not read an old response
   //as belonging to new request
   Rw_ref_id++;
   if (Rw_ref_id > 120)
      Rw_ref_id = 10;

   //Initialize Last rw values for when I
   //sort out only updating visual when
   //values have changed.
}

//---------------------------------------------------------------

static void Print_rw_change(RTX_rw_setup_req *rw)
{
   static double last_lat = 0.0f;
   static double last_lon = 0.0f;
   double delta_lat;
   double delta_lon;

   delta_lat = last_lat - rw->lat;
   if (delta_lat < 0.0)
      delta_lat = -delta_lat;
   delta_lon = last_lon - rw->lon;
   if (delta_lon < 0.0)
      delta_lon = -delta_lon;
   if (delta_lat > 0.01 || delta_lon > 0.01)
   {
      
      printf("\nNEW RW lat, lon = (%f, %f), head = %f app %d guide %d", rw->lat, rw->lon, rw->h,rw->app_lt,rw->guid_lt);
      fflush(stdout);
   }
   last_lat = rw->lat;
   last_lon = rw->lon;
}

//---------------------------------------------------------------

/* This calculates a wind strength based on NUM_WINDSOCK_LEVELS
   possible levels, for winds between 0 and max_speed.
   The speed is assumed to be positive. */

static int Get_wind_strength(float speed, float max_speed)
{
   float delta_range;
   float range;
   int   level;

   if (speed >= max_speed)
      return (NUM_WINDSOCK_LEVELS -1);

   level = 0;
   delta_range = max_speed / (float)(NUM_WINDSOCK_LEVELS -1);
   range = delta_range;

   while (level < (NUM_WINDSOCK_LEVELS -1))
   {
      if (speed < range)
         return level;

      range = range + delta_range;
      level++;
   }

   return (NUM_WINDSOCK_LEVELS -1);
}

//---------------------------------------------------------------

/* Some requests are only sent to the visual a few times, or until
   we receive confirmation that they were received. If the visual
   is shutdown and restarted, these need to be sent again. This
   routine will initialize flags so that all such data is resent. */
   
void RR_refresh_vis()
{
   Vis_setup_ok = FALSE;
   Cld_setup_ok = FALSE;
#ifdef DYN_DOING_TOD
   Tod_setup_ok = FALSE;
#endif
   Eye_setup_ok = FALSE;
   Sw_setup_ok = FALSE;
   Int_setup_ok = FALSE;
   Wind_level_setup_ok = FALSE;
   Wind_dir_setup_ok = FALSE;
#ifdef DO_SKY
   Sky_setup_ok = FALSE;
#endif
}

//----------------------------------------------------------------
