/*
File: visual5.h
Module: Dynamics for Seneca

Description: Interface structures for coms with Visual
             (Generic ICD 1.5 DRAFT)
*/

/***************************/
/*   Revision Record       */
/***************************/

/*
$Header: C:/master/seneca/VISUAL/RCS/visual5.h 1.2 2000/05/30 13:37:59 juliae Exp $
$Log: visual5.h $
Revision 1.2  2000/05/30 13:37:59  juliae
Added shutdown defines.

Revision 1.1  2000/04/17 10:37:25  juliae
Initial revision

*/

/*---------------------------------------------------------------*/



#define MAX_PACKETSIZE 1472
// these need to be set accurately


//=======================================================================
// General purpose data structures
//=======================================================================

typedef union
{
	 float float_val;
	 long   long_val;
} float_long;


typedef struct
{
	 float x;
	 float y;
	 float z;

} RTX_xyz_data;


typedef struct
{
 float		          timestamp;// not used at present // make a float union
 unsigned short int length;   // number of 32 bit words in following request
 unsigned char      status;   // Dynamics to Visual  .......t.b.d.
															// Visual to Dynamics, 21= visual ready and running
 char               chksum;   // Checksum of data in the following buffers
} RTX_header;


#define OBJECT_CODE             0x01
#define COLL_RES_CODE           0x02
#define TFB_VTX_CODE            0x03
#define TFB_RES_CODE            0x04
#define EVENT_CODE              0x05
#define SEARCH_RAY_CODE         0x06
#define SEARCH_RES_CODE         0x07

/* . . . . . */ // some other values defined (F1 car) these can be relocated later

#define CLOUD_LAYER_REQUEST     0x10
#define TIME_OF_DAY_REQUEST     0x11
#define FOG_REQUEST             0x12
#define SWITCH_CODE             0x13
#define GSWITCH_CODE            0x16
#define LOCAL_ROT_CODE          0x17
#define GLOCAL_ROT_CODE         0x18
#define ACKNOWLEDGE_RESPONSE    0x1e

#define MODIFY_AIR_OBJECT_CODE  0x20

#define LOC_CFRAME_CODE         0x24
#define OBJECT_SPH_CODE         0x25
#define RWAY_SETUP_CODE         0x26
#define EYE_OFFSET_CODE         0x27
#define SKY_CODE                0x2A


#define SCREEN_SAVER_ON         0x0001
#define CRASH_SEQUENCE_ON       0x0002
#define EXIT_GFX                0x007C   // bit 2  3  4 5 & 6 set

/*
               4 3 2
SHUTDOWN       0 0 1 (1<<2)
REBOOT         0 1 0 (2<<2)
FORCE          0 1 1 (3<<2)
POWEROFF       1 0 0 (4<<2)
FORCEIFHUNG    1 0 1 (5<<2)
LOGOFF         1 1 0 (6<<2)
EXIT           1 1 1 (7<<2)

*/

#define ETH_SHUTDOWN      0x0004
#define ETH_REBOOT        0x0008
#define ETH_FORCE         0x000c
#define ETH_POWEROFF      0x0010
#define ETH_FORCEIFHUNG   0x0014
#define ETH_LOGOFF        0x0018
#define ETH_EXIT          EXIT_GFX



#define ON_BIT      0x0001
#define COLL_BIT    0x0002
#define TFB_BIT     0x0004
#define GROUND_BIT  0x0008
#define RAY_BIT     0x0010
#define REPOS_BIT   0x0020

#define ALL_SWITCHES 0X4000
#define SWITCH_ON    0X8000



// error codes
#define ERR_OK         0x00
#define ERR_NO_TFB_VTX 0x01
#define ERR_BAD_OBJECT 0x02

typedef struct
{
   unsigned char opcode;   // (01) Unique id for this data structure
   unsigned char length;   // 8
   unsigned char obj_id;   // 0 = ownship, 1 to 255 for other objects under host control
   unsigned char spare ;   // 0  to pad out 32 bit words
   unsigned long flags ;   // 32 bit field for status etc

   float x;                // centre position in database in metres
   float y;                // centre position in database in metres
   float z;                // centre position (height) in database in metres
   float h;                // in degrees cw from North (y axis)
   float p;                // in degrees, positive upwards from horizontal
   float r;                // in degrees, positive clockwise long forward axis of host

} RTX_object_req;

#define MAX_NUM_TFB_VTX 12

typedef struct
{
   unsigned char opcode ;  // (03) unique id for this data structure
   unsigned char length ;  // 1 + 3 * num_vertices
   unsigned char obj_id ;  // 0 = own, 1 to 255 for others
   unsigned char num_vtx;  // max of 12 pts
   RTX_xyz_data  xyz[MAX_NUM_TFB_VTX];

} RTX_tfb_vtx_req;

#define MAX_NUM_SEARCH_RAYS 6

typedef struct
{
	 float x;
	 float y;
	 float z;
	 float len;
	 RTX_xyz_data dir;

} RTX_coll_seg_data;

typedef struct
{
	 unsigned char   opcode;    // 06 hex
	 unsigned char   length;    // 1 + 7*num_seg
	 unsigned char   obj_id;    // 0 for Host
	 unsigned char   nb_rays;   // Max of 6
	 RTX_coll_seg_data   seg[MAX_NUM_SEARCH_RAYS];

} RTX_search_ray_req;

typedef struct
{
   unsigned char opcode;   // (05) unique id for this data structure
   unsigned char length;   // 3
   unsigned short event_id; // unique id for the given event
                           // N.B. This is database dependent.
   unsigned char flags;    // 0 = off, 1 = on
   unsigned char spare[3]; // 0 to pad out to 32 bit word
   float         value;    // 0 to 100% control parameter for event

} RTX_event_req;

typedef struct
{
	unsigned char opcode;   // (MODIFY_AIR_OBJECT_CODE) unique id for this data structure
	unsigned char length;   // 4 so far
	unsigned char obj_id;
	unsigned char flags;

	unsigned char flaps;
	unsigned char slats;
	signed   char aileron;
	signed   char L_elevator;

	signed   char R_elevator;
	signed   char rudder;
	unsigned char undercariage;
	unsigned char airbrakes;

	unsigned char canopy;
	signed   char front_wheel_heading; // (relative to a/c)
	signed   char lights; //
	signed   char spare1;

} RTX_air_object_data;

#define CLD_TYPE_BITS   0x0003
#define CLD_FIXED_BIT   0x0004

typedef struct   /* Generic ICD Rev 1.4 */
{
   unsigned char   opcode;   // (CLOUD_LAYER_REQUEST) unique id for this data structure
   unsigned char   length;   // 2
   char            spare;
   unsigned char   flags;
   unsigned short  base_ht;
   unsigned short  thickness;

} RTX_cloud_layer_req;

typedef struct
{
   unsigned char opcode;   // (TIME_OF_DAY_REQUEST) unique id for this data structure
   unsigned char length;   // 2
   char          spare;
	unsigned char flags;
   float         time;     // 0 to 24, 24 hour clock

} RTX_time_of_day_req;

#define FOG_TYPE_BITS   0x0003
#define FOG_FIXED_BIT   0x0004

typedef struct   /* Generic ICD Rev 1.4 */
{
   unsigned char   opcode;   // (FOG_REQUEST) unique id for this data structure
   unsigned char   length;   // 2
   char            spare;
   unsigned char   flags;
   float           vis;

} RTX_fog_req;

typedef struct
{
   unsigned char  opcode;   // (SWITCH_CODE) unique id for this data structure
   unsigned char  length;   // 1
   signed short   sw_id_state;

} RTX_switch_req;

typedef struct
{
   unsigned char   opcode;    /* (GSWITCH_CODE) */
   unsigned char   length;    /* 1 */
   unsigned short  sw_id_state;
   
} RTX_gswitch_req;

typedef struct
{
   unsigned char   opcode;    /* (LOCAL_ROT_CODE) */
   unsigned char   length;    /* 4 */
   unsigned char   id;
   unsigned char   ref_id;
   float           h;         /* Radians. Positive clockwise */
   float           p;         /* Radians. Positive up */
   float           r;         /* Radians. Positive anticlockwise */
   
} RTX_loc_rot_req;

typedef struct
{
   unsigned char   opcode;    /* (GLOCAL_ROT_CODE) */
   unsigned char   length;    /* 4 */
   unsigned char   group_id;
   unsigned char   ref_id;
   float           h;         /* Radians. Positive clockwise */
   float           p;         /* Radians. Positive up */
   float           r;         /* Radians. Positive anticlockwise */
   
} RTX_gloc_rot_req;

typedef struct
{
   unsigned char opcode;    /* 24 hex */
   unsigned char length;
   unsigned char ref_id;
   unsigned char on_off;
   double        lat;       
   double        lon;

} RTX_loc_cframe_req;

/* An extended version of opcode 01hex */
typedef struct
{
   unsigned char opcode;    /* 25 hex */
   unsigned char length;
   unsigned char obj_id;
   unsigned char spare;
   unsigned long flags;
   double        lat;       
   double        lon;
   float         ht;
   float         h;
   float         p;
   float         r;

} RTX_object_sph_req;

typedef struct
{
   unsigned char opcode;    /* 26 hex */
   unsigned char length;
   unsigned char obj_id;
   unsigned char ref_id;
   unsigned char app_lt;
   unsigned char guid_lt;
   unsigned char rw_num;
   unsigned char on_off;
   double        lat;       
   double        lon;
   float         ht;
   float         h;
   float         rw_length;
   float         rw_width;

} RTX_rw_setup_req;

#define ALL_CHANNELS 255

typedef struct
{
   unsigned char opcode;    /* 27 hex */
   unsigned char length;
   unsigned char channel_id;
   unsigned char ref_id;
   long          spare;
   float         x;    //Metres
   float         y;
   float         z;
   float         h;    //Degrees
   float         p;
   float         r;

} RTX_eye_offset_req;

typedef struct
{
   unsigned char   opcode;       /* 2A hex */
   unsigned char   length;       /* 4 */
   unsigned char   move_with_h;  /* eg. if model is not a complete circle */
   unsigned char   sky_id;       /* 0 = no sky model */
   float           scale_width;
   float           scale_dist;
   float           scale_ht;
   
} RTX_sky_req;

//===========================================================================
// Responses from GRAPHICS to DYNAMICS computer
//===========================================================================

typedef struct
{
   unsigned char opcode;    // 01 hex
   unsigned char length;
   unsigned char obj_id;
   unsigned char error;

} RTX_object_res;

typedef struct
{
   unsigned char opcode;    // 02 hex
   unsigned char length;
   unsigned char obj_id;
   unsigned char spare1;
   unsigned char flags;           // Bit 7: set for valid hit_id
   unsigned char terrain_type;
   unsigned char hit_obj_id;
   unsigned char spare2;

} RTX_coll_res;

typedef struct
{
   unsigned char flags;
   unsigned char terrain_type;
   unsigned char hit_obj_id;
   unsigned char spare;
   float         distance;

} RTX_search_data;

typedef struct
{
   unsigned char opcode;    // 07 hex
   unsigned char length;
   unsigned char obj_id;
   unsigned char num_rays;
   RTX_search_data   set[MAX_NUM_SEARCH_RAYS];

} RTX_search_res;


typedef struct
{
	 unsigned char opcode;    //
	 unsigned char length;
	 unsigned char cmd  ;
	 unsigned char id;
} RTX_Acknowledge_res;


typedef struct
{
   unsigned char terrain_type;
   unsigned char spare[3];
   float         height;

} RTX_tfb_data;

typedef struct
{
   unsigned char opcode;    // 04 hex
   unsigned char length;    // 1 + 2*nb_vtx
   unsigned char obj_id;    // 0 for Host
   unsigned char nb_vtx;    // Max of 12
   RTX_tfb_data  set[MAX_NUM_TFB_VTX];

} RTX_tfb_res;

typedef struct
{
   unsigned char opcode;    /* 24 hex */
   unsigned char length;
   unsigned char ref_id;    /* Echo of ref_id in request */
   unsigned char spare1;

} RTX_loc_cframe_res;

typedef struct
{
   unsigned char opcode;    /* 26 hex */
   unsigned char length;
   unsigned char obj_id;
   unsigned char err_ref_id; /* If NOT ERR_BAD_OBJECT, then
                                echo of ref_id in request */

} RTX_rw_setup_res;

typedef struct
{
   unsigned char opcode;    /* 27 hex */
   unsigned char length;
   unsigned char channel_no;
   unsigned char ref_id;    /* Echo of ref_id in request */

} RTX_eye_offset_res;

typedef struct
{
   unsigned char   opcode;    /* SWITCH_CODE */
   unsigned char   length;
   unsigned short  sw_id_state;

} RTX_switch_res;

typedef struct
{
   unsigned char   opcode;    /* GSWITCH_CODE */
   unsigned char   length;
   unsigned short  sw_id_state;

} RTX_gswitch_res;

typedef struct
{
   unsigned char opcode;    /* LOCAL_ROT_CODE */
   unsigned char length;
   unsigned char ref_id;    /* Echo of ref_id in request */
   unsigned char spare;

} RTX_loc_rot_res;

typedef struct
{
   unsigned char opcode;    /* GLOCAL_ROT_CODE */
   unsigned char length;
   unsigned char ref_id;    /* Echo of ref_id in request */
   unsigned char spare;

} RTX_gloc_rot_res;

typedef struct
{
   unsigned char   opcode;    /* 2A hex */
   unsigned char   length;
   unsigned char   spare;
   unsigned char   sky_id;    /* Echo of sky_id in request */
   
} RTX_sky_res;
