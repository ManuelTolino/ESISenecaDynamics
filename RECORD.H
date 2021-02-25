#define MAX_REC      30000       //number of records for record facility

//#include "dat_stru\out_head.h"
//#include "dyn_snd.h"
//#include "fltdy\fmgdefs.h"
//#include "sound\dynsnd.h"

typedef struct
{
   sen_out_struct IO_record;     //hardware outputs
   dyn_to_snd     snd_rec;       //sound computer outputs
   dynsound       dynSndRec;     //"internal" sounds
   STATES         rec_states;
   float          sim_time;      //time sim has been running 
   unsigned int   record_index;  //index number of record
   float          delta_time;    //delta time for record
   float          pitch ,roll,pitch_trim,rudder; //hardware inputs stored to enable controls to be driven
   float          z,h,p,r;            //craft height,heading,pitch,roll for visual
   double         lat,lon;            //lat lon for visuals

}record_struct;
