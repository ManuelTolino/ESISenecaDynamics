//**************************
//   RCS Markers           *
//**************************

//$Header: C:/MASTER/seneca/FLTDY/RCS/fmg.h 1.3 2000/05/24 15:27:05 colinj Exp $
//$Log: fmg.h $
//Revision 1.3  2000/05/24 15:27:05  colinj
//Pass ATMOS structure to reset_model and calc_OP.
//
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

