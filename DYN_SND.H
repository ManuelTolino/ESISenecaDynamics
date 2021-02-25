/*
File:   dyn_snd.h
Module: Dynamics and Sound PCs for Seneca

Description: This file contains the interface structure between
             the dynamics and sound PC
*/

/***************************/
/*   RCS Markers           */
/***************************/

/*
$Header: C:/master/seneca/RCS/dyn_snd.h 1.2 2000/05/25 15:25:15 juliae Exp $
$Log: dyn_snd.h $
Revision 1.2  2000/05/25 15:25:15  juliae
Added BFO_ON_MORSE_TYPE define.

Revision 1.1  2000/04/14 17:57:01  juliae
Initial revision

*/

/*----------------------------------------------------------------------*/

//NB These are 1 less than IOS marker values
#define INNER_MARKER  2   //Also know as airways marker
#define MIDDLE_MARKER 1
#define OUTER_MARKER  0

#define MORSE_TYPE        0   //Morse sound, at standard pitch for channel
#define BFO_TONE_TYPE     1   //Adf only: continuous tone when BFO on for audible sound
#define BFO_ON_MORSE_TYPE 2   //Adf only: morse at different pitch when BFO on for non-audible sound

typedef struct {

   unsigned char   on;            /* TRUE is sound is on */
   unsigned char   type;          /* 0 = morse, 1 = bfo tone */
   unsigned char   morse_rate;    /* Number of times per minute */
   unsigned char   signal_strength; /* 1 = min, 100 = max */
   unsigned char   morse_str[4];  /* char 5 is 1st char of string,
                                     char 6 is 2nd char of string, etc. */

} snd_info;

typedef struct {

   snd_info      nav1;
   snd_info      nav2;
   snd_info      dme;
   snd_info      adf;
   unsigned char marker_on;    /* TRUE is sound is on */
   unsigned char marker_type;  /* See defines */
   char          sound_sDown;  // set true to shut down sound computer
   char          spare;

} dyn_to_snd;
