/* File: const.h
*/
/***************************/
/*   Revision Record       */
/***************************/

/*
$Header: C:/master/seneca/RCS/const.h 1.1 2000/04/14 17:45:53 colinj Exp $
$Log: const.h $
Revision 1.1  2000/04/14 17:45:53  colinj
Initial revision

*/

/*---------------------------------------------------------------*/

#define G          ( 9.80665 )                    /* metres/sec/sec   */
#define FRAME_RATE ( 30 )          /* For dynamics task anyway        */
#define M_TO_F     ( 3.28 )            /* Convert from metres to feet */
#define F_TO_M     ( 0.3049 )        /* Convert from feet to metres   */
#define MS_TO_MPH  ( M_TO_F * 60.0 * 60.0 / 5280.0) /* m/s to mph     */
#define KTS_TO_MPH ( 1.155 )                        /* kts to mph     */
#define MPH_TO_KTS ( 1/1.155 )                      /* mph to kts     */
#define KTS_TO_MS  ( 0.514 )                        /* kts to m/s     */
#define MS_TO_KTS  ( 1/0.514 )                      /* m/s to kts     */
#define TWO_PI     ( M_PI * 2.00 )                  /*                */
#define PI_BY_TWO  ( M_PI * 0.50 )                  /*                */
#define PI_BY_FOUR ( M_PI * 0.25 )                  /*                */
#define MS_FTM     (196.8)           /* convert m/sec to ft/min */
#define FTM_MS     (1/196.8)         /* convert ft/min to m/sec */