/*
File  : bc_range.c

Module: Navigation for Seneca
Created: 24 March 2000

Description: This file contains the values used for controlling
             some of the beacon ranges
*/

/***************************/
/*   RCS Markers           */
/***************************/

/*
$Header: C:/seneca2/NAV/RCS/bc_range.h 1.1 2000/03/27 09:19:48 juliae Exp $
$Log: bc_range.h $
Revision 1.1  2000/03/27 09:19:48  juliae
Initial revision

*/

/*----------------------------------------------------------------------*/

/*----------------*/
/* Defines        */
/*----------------*/

#define RANGE_INC 2.0f  //MODIFICADO POR MI ANTES 1.3f

//VOR
//---

#define CONE_CONFUSION_PITCH 60.0f

/* VOR ranges are controlled by Merit ranges given
   from Jeppeson. If the Merit range states
   "unlimited" the following range is used */

#define TRUE_MAX_VOR_RANGE  200.0f
//Make signal available outside true range
#define MAX_VOR_RANGE  (RANGE_INC * TRUE_MAX_VOR_RANGE)

//ILS - Only affected by RANGE_INC
//---

//DME
//---

#define TRUE_DME_RANGE_NM 199.0f
//Make signal available outside true range
#define DME_RANGE_NM (RANGE_INC * TRUE_DME_RANGE_NM)


//NDB
//---

// Range depends on power via
// range in nm = 3 * sqrt(power in watts)
// 0: < 25, 1: (25,50), 2: (50,2000), 3: > 2000 watts

#define TRUE_NDB_RANGE_0_NM 10.0f   // 12 watts
#define TRUE_NDB_RANGE_1_NM 18.0f   // 37 watts
#define TRUE_NDB_RANGE_2_NM 96.0f   // 1025 watts
#define TRUE_NDB_RANGE_3_NM 164.0f  // 3000 watts

//Make signals available outside true ranges
#define NDB_RANGE_0_NM  (RANGE_INC * TRUE_NDB_RANGE_0_NM)
#define NDB_RANGE_1_NM  (RANGE_INC * TRUE_NDB_RANGE_1_NM)
#define NDB_RANGE_2_NM  (RANGE_INC * TRUE_NDB_RANGE_2_NM)
#define NDB_RANGE_3_NM  (RANGE_INC * TRUE_NDB_RANGE_3_NM)
