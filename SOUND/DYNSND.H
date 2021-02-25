//File: dynsnd.h
//Module: Seneca Dynamics

/* Dynamics Sound

  To use this:
  Call dynsnd(1) at initialisation (first call)
  Call dynsnd(0) every frame
  Call dynsnd(-1) at exit

  Before each call to dynsnd(0), DYS_in must be valid.
  Dynsnd(1) will give this a valid initial state (no sound).
  Changes made to DYS_in will persist between frames.

*/

//**************************
//   RCS Markers           *
//**************************


//$Header: C:/master/seneca/sound/RCS/dynsnd.h 1.3 2000/05/25 09:48:38 colinj Exp $
//$Log: dynsnd.h $
//Revision 1.3  2000/05/25 09:48:38  colinj
//New const and changes to existing ones.
//
//Revision 1.2  2000/05/11 07:25:08  colinj
//New const and changes to current ones.
//
//Revision 1.1  2000/04/17 09:59:05  colinj
//Initial revision
//

//----------------------------------------------------------------------

typedef struct dynsound
{
   float engine_left;   // Engine RPM
   float engine_right;

   float prop_left; // Prop RPM
   float prop_right;

   float airspeed; // for wind noise

   long unsigned flags; // see defs below
} dynsound;

extern dynsound DYS_in;

extern void DYS_dynsnd(int);

/* flags. All flags are momentary - resetting the flag will not stop
the sound. De-ice and engine starting are looped - engines have a start sample (played only if
the engine was not already running) and de-ice is timed: repeated presses add to the time out.

A ground start of an engine includes setting flag_start_x, an air start is just done by
changing RPM.
*/ 
 
const int flag_deice         = 1;  // Surface de-ice
const int flag_geardown      = 2; // Gear is moving up/down
const int flag_gearoff       = 4;
const int flag_crash         = 8;  // oops
const int flag_skid          = 16;  // Set this at touchdown
const int flag_flapup        = 32;   // Flap is moving up/down
const int flag_flapoff       = 64;
const int flag_start_left    = 128; // Ground start
const int flag_start_right   = 256;
const int flag_clunk         = 512;
const int flag_primer_left   = 1024;    // primer pump
const int flag_restart_left  = 2048; // Engine has failed to start - loop starting sound
const int flag_restart_right = 4096; // (flag_start_left should also be set)
const int flag_on_left       = 8192;
const int flag_on_right      = 16384;
const int flag_strobe        = 32768; // beacon strobe
const int flag_autopilot     = 65536;
const int flag_manifold      = 131072;
const int flag_primer_right  = 262144;
const int flag_deice_off     = 524288;
const int flag_gyro          = 1048576;
const int flag_Lclunk        = 2097152;
const int flag_Rclunk        = 4194304;

/* Environmental (indicators etc)? samples! */


const float prop_hz = 10000.0; // Freq of prop WAV
const float prop_min = 100.0;   // Minimum prop rpm to produce sound
const float prop_std = 1000.0;  // Prop rpm to produce sound at prop_hz

/*
const float eng_hz = 44100.0; // Freq of starting/idling engine WAV (start.wav)
const float eng_min = 100.0;   // Minimum engine rpm to produce sound
const float eng_std = 1700.0;  // Engine rpm to produce sound at eng_hz (note modified eng_hz must never exceed 100kHz. NMF.)
const float eng_high = 1500.0; // Max rpm to produce 100% volume
const float eng_max  = 1700.0; // Max rpm to produce sound
const int eng_start = 814079*4; //  Offset (bytes) into the engine WAV of start of idle loop
const int eng_end = 3400000;   //  "       "        "  "     "     "  "  end   "  " (don't put this too near the end - or it will leave gaps or start from the beginning)

const float ceng_hz = 44100.0; // as ", for cruising engine loop wav (2600.wav)
const float ceng_min = 1300.0;   // Minimum engine rpm to produce sound
const float ceng_std = 2600.0;  // Engine rpm to produce sound at eng_hz (note modified eng_hz must never exceed 100kHz. NMF.)
const float ceng_low = 1600.0; // Min rpm to produce 100% volume
*/

const float eng_hz = 44100.0; // Freq of starting/idling engine WAV (start.wav)
const float eng_min = 100.0;   // Minimum engine rpm to produce sound
const float eng_std = 1200.0;  // Engine rpm to produce sound at eng_hz (note modified eng_hz must never exceed 100kHz. NMF.)
const float eng_high = 1500.0; // Max rpm to produce 100% volume
const float eng_max  = 1800.0; // Max rpm to produce sound
const int eng_start = 568320*4; //  Offset (bytes) into the engine WAV of start of idle loop
const int eng_end = 662528*4;   //  "       "        "  "     "     "  "  end   "  " (don't put this too near the end - or it will leave gaps or start from the beginning)
const float eng_fade = -300.0;  // constant fade to idle engine
const int eng_startloop =   35840*4;          // Offset bytes into the engine WAV of start of starting loop (for failed starts)
const int eng_startloopend = 69632*4; // "   "        "              "        end   "        "
const int eng_startrumble  = 590464;

const float ceng_hz = 44100.0; // as ", for cruising engine loop wav (2600.wav)
const float ceng_min = 1300.0;   // Minimum engine rpm to produce sound
const float ceng_std = 2200.0;  // Engine rpm to produce sound at eng_hz (note modified eng_hz must never exceed 100kHz. NMF.)
const float ceng_max = 2800.0;
const float ceng_low = 1500.0; // Min rpm to produce 100% volume
const float ceng_fade = -1000.0; // dB/100 fade from idle/cruise change to max 

const int primer_endloop = 222208;
const int primer_startloop = 157696;

const int manifold_endloop = 156928*2;
const int manifold_startloop = 13824*2;

const int gyro_endloop = 572416*4;
const int gyro_startloop = 97280*4;
const long gyro_volume = -1000;

/* Offset (bytes) into the deice WAV to reset position to when restarting (while deice is still playing).
   This should be immediately after any 'starting' sounds. */
const int deice_start = 254000*2;
const int end_deice   = 6061320;
const long deice_volume = 0; 
const long strobe_volume = -1000;
const long primer_volume = -100;
/* Offset (bytes) into the deice WAV. If playing has passed this point, the WAV is assumed to have finished.
   This should be immediately before any 'finishing' sounds. */
const int deice_end = 3010000*2;

const float airspeed_max = 120.0; // Maximum airspeed - at and above this speed wind noise will be full volume.
const float airspeed_range = 4000.0; // Range of volume change (ie attenuation at speed 0) in dB / 100.

