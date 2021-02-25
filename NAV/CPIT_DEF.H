/* Defines for use with IO_Sen_in and IO_Sen_out */

/***************************/
/*   RCS Markers           */
/***************************/

/*
$Header: C:/SENECA/NAV/RCS/cpit_def.h 1.3 2000/03/02 17:03:28 juliae Exp $
$Log: cpit_def.h $
Revision 1.3  2000/03/02 17:03:28  juliae
Changed KA51_SLAVE value.

 * Revision 1.2  2000/03/01  15:23:31  juliae
 * Changed ENABLE and DISABLE.
 *
 * Revision 1.1  2000/02/22  02:40:57  juliae
 * Initial revision
 *
*/

/*----------------------------------------------------------------------*/

//For GS enable
#define DISABLE 0
#define ENABLE  1

#define SHOW_FLAG SW_ON
#define HIDE_FLAG SW_OFF

#define CENTRE_NEEDLE  0.0
#define TO_NEEDLE      100.0
#define FROM_NEEDLE   -100.0

#define RIGHT_NEEDLE   100.0
#define LEFT_NEEDLE   -100.0

#define KA51_SLAVE  NOT_PRESSED
