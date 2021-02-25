head	1.3;
access;
symbols;
locks; strict;
comment	@ * @;


1.3
date	2000.06.13.14.48.27;	author COLINJ;	state Exp;
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


1.3
log
@Set number of user states to 41 instead of 128
@
text
@//**************************
//   RCS Markers           *
//**************************

//$Header: C:/MASTER/seneca/FLTDY/RCS/fmgdefs.h 1.2 2000/05/10 09:39:33 colinj Exp COLINJ $
//$Log: fmgdefs.h $
//Revision 1.2  2000/05/10 09:39:33  colinj
//Updates from Martin K.
//
//Revision 1.1  2000/04/17 11:12:22  colinj
//Initial revision
//

/*---------------------------------------------------------------*/

#define N_USER_STATES      41
#define N_USER_D_INPUTS   128
#define N_USER_A_INPUTS   128
#define N_USER_D_OUTPUTS  128
#define N_USER_A_OUTPUTS  128

#define PI 3.141592

#define MPS2KN 1.9427
#define KN2MPS (1.0/MPS2KN)

#define D2R (PI/180.0)
#define R2D (180.0/PI)

#define RPS2RPM  9.54929
#define RPM2RPS  0.10472

#define LBS2KG         0.45359
#define KG2LBS         2.20462

#define USGAL2UKGAL    0.83267
#define UKGAL2USGAL    1.2010

#define INHG2MB 33.8639
#define MB2INHG 2.953e-2
#define PA2INHG 2.953e-4

#define TRUE 1
#define FALSE 0

#define FREEZE_UVW         1  //FREEZE VELOCITYS
#define FREEZE_PQR         2  //ANGULAR VELOCITYS
#define FREEZE_ANGLES      4  //FREEZE ALL ANGULAR MOVEMENT
#define FREEZE_NED         8  //FREEZE POSITIONS (NORTH EAST DOWN)
#define FREEZE_LATLON     16  //FREEZE LAT LONG
#define FREEZE_USER       32  //FREEZE ALL ENGINE STATES

#define FREEZE_RB_ONLY  (FREEZE_UVW+FREEZE_PQR+FREEZE_ANGLES+FREEZE_NED+FREEZE_LATLON)
#define FREEZE_NONE


typedef struct {
  float u;
  float v;
  float w;
  float p;
  float q;
  float r;
  float e0;
  float e1;
  float e2;
  float e3;
  double N;
  double E;
  double D;
  double latitude;
  double longitude;
  float user[N_USER_STATES];
} STATES;

typedef struct {
  float mass;
  float Ixx;
  float Iyy;
  float Izz;
  float Ixz;
} MASS;

typedef struct {
  float wN;
  float wE;
  float wD;
  float delta_T;
  float delta_p;
  float delta_r;
  float ht;
} ATMOS;

typedef struct {
  float inps[N_USER_A_INPUTS];
  int dinps[N_USER_D_INPUTS];
} INPUTS;

typedef struct {
	float phi;
	float theta;
	float psi;
	float phi_dot;
	float theta_dot;
	float psi_dot;
	float p_dot;
	float q_dot;
	float r_dot;
	float alpha;
	float beta;
	float gamma;
	float ax;
	float ay;
	float az;
	float Vt;
	float Vtas;
	float Vg;
	float track;
	float amach;
	float temperature;
	float pressure;
	float density;
	float atm_theta;
	float atm_delta;
	float atm_sigma;
	float hdot;
	float S[3][3];
	float user[N_USER_A_OUTPUTS];
	int duser[N_USER_D_OUTPUTS];
} OUTPUTS;

typedef struct {
	float x[3];
	float y[3];
	float xl[3];
	float k[3];
	float b[3];
	float preload[3];
} GEAR;


@


1.2
log
@Updates from Martin K.
@
text
@d5 1
a5 1
//$Header: C:/MASTER/SENECA/FLTDY/RCS/fmgdefs.h 1.1 2000/04/17 11:12:22 colinj Exp $
d7 3
d16 1
a16 1
#define N_USER_STATES     128
@


1.1
log
@Initial revision
@
text
@d5 5
a9 2
//$Header: $
//$Log: $
d13 6
d21 3
d43 11
d70 1
a70 1
  float user[64];
d92 2
a93 2
  float inps[64];
  int dinps[64];
d97 6
a102 6
	float phi;    
	float theta;  
	float psi;    
	float phi_dot;  
	float theta_dot; 
	float psi_dot;   
d106 7
a112 7
	float alpha;  
	float beta;   
	float gamma;  
	float ax;     
	float ay;     
	float az;     
	float Vt;     
d125 2
a126 2
	float user[64];
	int duser[64];
@
