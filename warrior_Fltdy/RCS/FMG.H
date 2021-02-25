head	1.3;
access;
symbols;
locks
	COLINJ:1.3; strict;
comment	@ * @;


1.3
date	2000.05.24.15.27.05;	author colinj;	state Exp;
branches;
next	1.2;

1.2
date	2000.05.10.09.37.40;	author colinj;	state Exp;
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
@Pass ATMOS structure to reset_model and calc_OP.
@
text
@//**************************
//   RCS Markers           *
//**************************

//$Header: C:/master/seneca/FLTDY/RCS/fmg.h 1.2 2000/05/10 09:37:40 colinj Exp $
//$Log: fmg.h $
//Revision 1.2  2000/05/10 09:37:40  colinj
//Updates from Martin K.
//
//Revision 1.1  2000/04/17 11:12:22  colinj
//Initial revision
//

/*---------------------------------------------------------------*/

/***************************************************/
/*** Functions which comprise the user interface ***/
/***************************************************/

#ifdef C_PlusPlus
extern "C" void init_model(char *filename);

extern "C" void simulate_model(INPUTS *inputs, STATES *states, OUTPUTS *outputs, ATMOS *atmos, float dt, int freeze_flags );

extern "C" void reset_model( double  height, double  North,   double  East, 
                  double lat, double lon,
                  float  speed,  float  heading, float  theta, float gamma, float phi,
                  INPUTS *inputs, STATES *states,  OUTPUTS *outputs ,ATMOS *atmos);
#else
void init_model(char *filename);

void simulate_model(INPUTS *inputs, STATES *states, OUTPUTS *outputs, ATMOS *atmos, float dt, int freeze_flags );

void reset_model( double  height, double  North,   double  East,
                  double lat, double lon,
                  float  speed,  float  heading, float  theta, float gamma, float phi,
                  INPUTS *inputs, STATES *states,  OUTPUTS *outputs ,ATMOS *atmos);
#endif


/**********************************************/
/*** Functions generic to all flight models ***/
/**********************************************/

#ifdef C_PlusPlus
extern "C" void integrate( STATES *x, STATES *dx, float dt, int freeze_flags);
extern "C" void calc_RB( float *FM,      MASS *mass,    STATES *x,
              OUTPUTS *y, STATES *dx );
extern "C" void calc_OP( INPUTS *u, STATES *x, OUTPUTS *y, ATMOS *a);
extern "C" void calc_ISA( float  delta_t,  float  delta_p, float  delta_rho, float  h, 
               float *t,        float *p,       float *rho,       float *a, 
               float *omega,    float *delta,    float *sigma);
extern "C" int calc_gear(GEAR *gear, STATES *x, OUTPUTS *y, float ht, 
                float steer, float L_brake, float R_brake, float FM[6]);
extern "C" int calc_gear_leg(float x,  float y,  float xl,
                    float u,  float v,  float w, 
                    float p,  float q,  float r, 
                    float h, float theta, float phi,
                    float k,  float b,  float preload,
                    float brake, float steer, float FM[6]);
extern "C" void calc_Cl_Cd(float alpha, float Cla, float dCl, float alpha_0, float alpha_s, float Cd0, float Cdi, float *Cl, float *Cd);
extern "C" float table1D(float *xt, float *yt, int n, float x);
#else
void integrate( STATES *x, STATES *dx, float dt, int freeze_flags);

void calc_RB( float *FM,      MASS *mass,    STATES *x,
              OUTPUTS *y, STATES *dx );

void calc_OP( INPUTS *u, STATES *x, OUTPUTS *y, ATMOS *a);

void calc_ISA( float  delta_t,  float  delta_p, float  delta_rho, float  h, 
               float *t,        float *p,       float *rho,       float *a, 
               float *omega,    float *delta,    float *sigma);

int calc_gear(GEAR *gear, STATES *x, OUTPUTS *y, float ht, 
                float steer, float L_brake, float R_brake, float FM[6]);

int calc_gear_leg(float x,  float y,  float xl,
                    float u,  float v,  float w, 
                    float p,  float q,  float r, 
                    float h, float theta, float phi,
                    float k,  float b,  float preload,
                    float brake, float steer, float FM[6]);

void calc_Cl_Cd(float alpha, float Cla, float dCl, float alpha_0, float alpha_s, float Cd0, float Cdi, float *Cl, float *Cd);
float table1D(float *xt, float *yt, int n, float x);
#endif


/***********************************************************/
/*** Functions to be supplied by a specific flight model ***/
/***********************************************************/

#ifdef C_PlusPlus
extern "C" void calc_FM(INPUTS *inputs, STATES *states, OUTPUTS *outputs, ATMOS *atmos, float *FM, STATES *dstates, MASS *mass);

extern "C" void reset_FMS(INPUTS *inputs, STATES *states);

extern "C" void calc_OP_FMS(INPUTS *inputs, STATES *states, OUTPUTS *outputs);
#else
void calc_FM(INPUTS *inputs, STATES *states, OUTPUTS *outputs, ATMOS *atmos, float *FM, STATES *dstates, MASS *mass);

void reset_FMS(INPUTS *inputs, STATES *states);

void calc_OP_FMS(INPUTS *inputs, STATES *states, OUTPUTS *outputs);
#endif

@


1.2
log
@Updates from Martin K.
@
text
@d5 1
a5 1
//$Header: C:/MASTER/SENECA/FLTDY/RCS/fmg.h 1.1 2000/04/17 11:12:22 colinj Exp $
d7 3
d28 1
a28 1
                  INPUTS *inputs, STATES *states,  OUTPUTS *outputs );
d34 1
a34 1
void reset_model( double  height, double  North,   double  East, 
d37 1
a37 1
                  INPUTS *inputs, STATES *states,  OUTPUTS *outputs );
d49 1
a49 1
extern "C" void calc_OP( INPUTS *u, STATES *x, OUTPUTS *y);
d69 1
a69 1
void calc_OP( INPUTS *u, STATES *x, OUTPUTS *y);
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
d20 1
a20 1
extern "C" void simulate_model(INPUTS *inputs, STATES *states, OUTPUTS *outputs, ATMOS *atmos, float dt );
d29 1
a29 1
void simulate_model(INPUTS *inputs, STATES *states, OUTPUTS *outputs, ATMOS *atmos, float dt );
d43 1
a43 1
extern "C" void integrate( STATES *x, STATES *dx, float dt);
d61 1
a61 1
void integrate( STATES *x, STATES *dx, float dt);
@
