head	1.4;
access;
symbols;
locks; strict;
comment	@ * @;


1.4
date	2000.06.13.14.51.41;	author COLINJ;	state Exp;
branches;
next	1.3;

1.3
date	2000.05.24.15.31.06;	author colinj;	state Exp;
branches;
next	1.2;

1.2
date	2000.05.10.09.39.33;	author colinj;	state Exp;
branches;
next	1.1;

1.1
date	2000.04.17.11.12.22;	author colinj;	state Exp;
branches;
next	;


desc
@@


1.4
log
@added new failure flag for nose gear fail to extend and second set
of contact flags which work irespective of gear position.
@
text
@/*****************************************************************************
 **                                                                         **
 ** File        : FMS_DEFS.H                                                **
 **                                                                         **
 ** Description : Seneca III flight model-specific include file             **
 **               Index definitions for array entires                       **
 **                                                                         **
 ** Created     : 31 Jan 2000, M G Kellett                                  **
 **                                                                         **
 **                                                                         **
 **                                          (c) Flight Dynamics Ltd, 2000  **
 *****************************************************************************/

//**************************
//   RCS Markers           *
//**************************

//$Header: C:/MASTER/seneca/FLTDY/RCS/fms_defs.h 1.3 2000/05/24 15:31:06 colinj Exp COLINJ $
//$Log: fms_defs.h $
//Revision 1.3  2000/05/24 15:31:06  colinj
//Added new defines.
//
//Revision 1.2  2000/05/10 09:39:33  colinj
//Updates from Martin K.
//
//Revision 1.1  2000/04/17 11:12:22  colinj
//Initial revision
//

/*---------------------------------------------------------------*/

 /**
 ** These #define's are the indexes to the array entries for the
 ** FMG user elements of the input, state and output arrays.
 **/

/*** Analogue flight model inputs ***/

#define U_A_LAT          0
#define U_A_LONG         1
#define U_A_PED          2
#define U_A_AIL_TRIM     3
#define U_A_ELE_TRIM     4
#define U_A_RUD_TRIM     5
#define U_A_L_THROT      6
#define U_A_L_RPM        7
#define U_A_L_MIX        8
#define U_A_R_THROT      9
#define U_A_R_RPM       10
#define U_A_R_MIX       11
#define U_A_L_BRAKE     12
#define U_A_R_BRAKE     13
#define U_A_ALT_P1A     14
#define U_A_ALT_P1B     15
#define U_A_ALT_P2      16
#define U_A_MASS        17
#define U_A_CG          18
#define U_A_L_INIT_FUEL 19
#define U_A_R_INIT_FUEL 20
#define U_A_L_COWL  21
#define U_A_R_COWL  22
#define U_A_HDG_ERR 23
#define U_A_CDI     24
#define U_A_GS_DEV  25
//
#define U_A_DEBUG_P1 27
#define U_A_DEBUG_P2 28
#define U_A_DEBUG_P3 29
#define U_A_DEBUG_P4 30

/*** Discrete flight model inputs ***/

#define U_D_GEAR               0
#define U_D_FLAP               1
#define U_D_START              2
#define U_D_L_MAG_L            3
#define U_D_L_MAG_R            4
#define U_D_R_MAG_L            5
#define U_D_R_MAG_R            6
#define U_D_L_PRIME  7
#define U_D_R_PRIME  8
#define U_D_L_XFEED            9
#define U_D_R_XFEED            10
#define U_D_L_BOOST            11
#define U_D_R_BOOST            12
#define U_D_S_SOURCE           13
#define U_D_PARK               14
#define U_D_AP_DISC            15
#define U_D_CWS                16
#define U_D_ELEC_TRIM          17
#define U_D_KFC150_FD          18
#define U_D_KFC150_ALT         19
#define U_D_KFC150_HDG         20
#define U_D_KFC150_NAV         21
#define U_D_KFC150_APR         22
#define U_D_KFC150_BC          23
#define U_D_KFC150_TEST        24
#define U_D_KFC150_AP_ENG      25
#define U_D_KFC150_VT          26
#define U_D_S1_BLOCKAGE        27
#define U_D_S2_BLOCKAGE        28
#define U_D_P_BLOCKAGE         29
#define U_D_FAIL_L_SUCTION     30
#define U_D_FAIL_L_FUEL        31
#define U_D_FAIL_L_MAG_L       32
#define U_D_FAIL_L_MAG_R       33
#define U_D_FAIL_L_OIL_T       34
#define U_D_FAIL_L_OIL_P       35
#define U_D_FAIL_L_CHT         36
#define U_D_FAIL_L_PROP        37
#define U_D_FAIL_L_DEICE       38
#define U_D_FAIL_L_ALT_OVLD    39
#define U_D_FAIL_L_ALT_ZERO    40
#define U_D_FAIL_L_ALT_FLD     41
#define U_D_FAIL_R_SUCTION     42
#define U_D_FAIL_R_FUEL        43
#define U_D_FAIL_R_MAG_L       44
#define U_D_FAIL_R_MAG_R       45
#define U_D_FAIL_R_OIL_T       46
#define U_D_FAIL_R_OIL_P       47
#define U_D_FAIL_R_CHT         48
#define U_D_FAIL_R_PROP        49
#define U_D_FAIL_R_DEICE       50
#define U_D_FAIL_R_ALT_OVLD    51
#define U_D_FAIL_R_ALT_ZERO    52
#define U_D_FAIL_R_ALT_FLD     53
#define U_D_FAIL_GEAR_PUMP     54
#define U_D_FAIL_PITCH_TRIM    55
#define U_D_FAIL_FLAP_MOTOR    56
#define U_D_GEAR_EMERG   57
#define U_D_AIRFRAME_ICE 58
#define U_D_DEICE        59
#define U_D_FAIL_DEICE   60
#define U_D_PROP_ICE     61
#define U_D_PROP_DEICE   62
#define U_D_KFC150_PWR   63
#define U_D_L_MAP_DRAIN  64
#define U_D_R_MAP_DRAIN  65
#define U_D_NAV_VALID    66
#define U_D_GS_VALID     67
#define U_D_L_ENGINE_SEIZE 68
#define U_D_R_ENGINE_SEIZE 69
#define U_D_LIVE_START 70
#define U_D_NOSE_GEAR_EXTEND_FAIL 71

/*** Additional flight model states ***/

#define X_N_GEAR                 0
#define X_L_GEAR                 1
#define X_R_GEAR                 2
#define X_FLAP                   3
#define X_L_COWL                 4
#define X_R_COWL                 5
#define X_P_PITOT                6
#define X_P_S1                   7
#define X_P_S2                   8
#define X_L_RPM                  9
#define X_L_BLADE                10
#define X_L_MAN_P                11
#define X_L_FUEL                 12
#define X_L_CHT                  13
#define X_L_EGT                  14
#define X_R_RPM                  15
#define X_R_BLADE                16
#define X_R_MAN_P                17
#define X_R_FUEL                 18
#define X_R_CHT                  19
#define X_R_EGT                  20
#define X_L_OIL_T    21
#define X_R_OIL_T    22
#define X_L_OIL_P    23
#define X_R_OIL_P    24
#define X_SIM_TIME   25
#define X_ICE        26
#define X_L_PROP_ICE 27
#define X_R_PROP_ICE 28
#define X_R_FF       29
#define X_L_FF       30
#define X_L_BOOST 31
#define X_R_BOOST 32
#define X_L_MAP_GAUGE 33
#define X_R_MAP_GAUGE 34
#define X_L_VSI_GAUGE 35
#define X_R_VSI_GAUGE 36
#define X_STEER 37

/*** Additional flight model outputs (analogue) ***/

#define Y_A_PITOT                  0
#define Y_A_TRIM_COST             1
#define Y_A_P1_ASI                2
#define Y_A_P1A_ALT               3
#define Y_A_P1B_ALT               4
#define Y_A_P1_ROC                5
#define Y_A_P2_ASI                6
#define Y_A_P2_ALT                7
#define Y_A_P2_ROC                8
#define Y_A_TURN_RATE             9
#define Y_A_SLIP_ANGLE            10
#define Y_A_FD_PITCH              11
#define Y_A_FD_ROLL               12
#define Y_A_L_MAN_P               13
#define Y_A_L_RPM                 14
#define Y_A_L_EGT                 15
#define Y_A_L_CHT                 16
#define Y_A_L_FF                  17
#define Y_A_L_FUEL                18
#define Y_A_L_OIL_T               19
#define Y_A_L_OIL_P               20
#define Y_A_R_MAN_P               21
#define Y_A_R_RPM                 22
#define Y_A_R_EGT                 23
#define Y_A_R_CHT                 24
#define Y_A_R_FF                  25
#define Y_A_R_FUEL                26
#define Y_A_R_OIL_T               27
#define Y_A_R_OIL_P               28
#define Y_A_VACUUM                29
#define Y_A_DS1DT                 30
#define Y_A_DS2DT                 31
#define Y_A_L_BLADE               32
#define Y_A_R_BLADE               33
#define Y_A_PITCH_SERVO           34
#define Y_A_ROLL_SERVO            35
#define Y_A_QBAR       36
#define Y_A_PITCH_NULL 37
#define Y_A_PEDAL_NULL 38
#define Y_A_L_MIXTURE 39
#define Y_A_R_MIXTURE 40
#define Y_A_L_BHP     41
#define Y_A_R_BHP     42
#define Y_A_XBAL 43
#define Y_A_MBAL 44
#define Y_A_ZBAL 45
#define Y_A_P1_VSI_GAUGE 46
#define Y_A_P2_VSI_GAUGE 47


/*** Additional flight model outputs (discrete) ***/

#define Y_D_N_CONTACT              0
#define Y_D_L_CONTACT              1
#define Y_D_R_CONTACT              2
#define Y_D_CRASH                  3
#define Y_D_F_SCRAPE               4
#define Y_D_A_SCRAPE               5
#define Y_D_L_SCRAPE               6
#define Y_D_R_SCRAPE               7
#define Y_D_N_GEAR                 8
#define Y_D_L_GEAR                 9
#define Y_D_R_GEAR                 10
#define Y_D_GEAR                   11
#define Y_D_KFC150_FD              12
#define Y_D_KFC150_ALT             13
#define Y_D_KFC150_HDG             14
#define Y_D_KFC150_GS              15
#define Y_D_KFC150_NAV             16
#define Y_D_KFC150_APR             17
#define Y_D_KFC150_BC              18
#define Y_D_KFC150_TRIM            19
#define Y_D_KFC150_AP              20
#define Y_D_FLAPS                  21
#define Y_D_STALL                  22
#define Y_D_FD_VISIBLE             23
#define Y_D_PITCH_TRIM_REQ         24
#define Y_D_L_ENGINE_LIT 25
#define Y_D_R_ENGINE_LIT 26
#define Y_D_N_CONTACT_ACTUAL 27 //added by CJ these will be actual contact even no gear 
#define Y_D_L_CONTACT_ACTUAL 28 //added by CJ these will be actual contact even no gear
#define Y_D_R_CONTACT_ACTUAL 29 //added by CJ these will be actual contact even no gear

// KFC150 buttons
#define KFC150_B_FD      0
#define KFC150_B_ALT     1
#define KFC150_B_HDG     2
#define KFC150_B_NAV     3
#define KFC150_B_APR     4
#define KFC150_B_BC      5
#define KFC150_B_TEST    6
#define KFC150_B_AP_ENG  7
#define KFC150_B_VT      8
#define KFC150_B_CWS     9
#define KFC150_B_AP_DIS 10

// KFC150 lamps
#define KFC150_L_FD      0
#define KFC150_L_ALT     1
#define KFC150_L_HDG     2
#define KFC150_L_GS      3
#define KFC150_L_NAV     4
#define KFC150_L_APR     5
#define KFC150_L_BC      6
#define KFC150_L_TRIM    7
#define KFC150_L_AP      8

// KFC150 modes
#define KFC150_M_RESET    0
#define KFC150_M_SELFTEST 1
#define KFC150_M_IDLE     2

@


1.3
log
@Added new defines.
@
text
@d18 1
a18 1
//$Header: C:/master/seneca/FLTDY/RCS/fms_defs.h 1.2 2000/05/10 09:39:33 colinj Exp $
d20 3
d144 1
a144 1

d268 3
@


1.2
log
@Updates from Martin K.
@
text
@d18 1
a18 1
//$Header: C:/MASTER/SENECA/FLTDY/RCS/fms_defs.h 1.1 2000/04/17 11:12:22 colinj Exp $
d20 3
d140 1
d232 2
d277 1
@


1.1
log
@Initial revision
@
text
@d18 5
a22 2
//$Header: $
//$Log: $
d54 6
a59 6
#define U_A_L_COWL             21
#define U_A_R_COWL             22
#define U_A_HDG_BUG 23
#define U_A_LOC_HDG 24
#define U_A_LOC_DEV 25
#define U_A_GLD_DEV 26
d131 8
d174 5
d225 3
d257 2
@
