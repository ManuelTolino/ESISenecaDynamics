//**************************
//   RCS Markers           *
//**************************

//new fmg.cpp back in for Dyn_232
//$Header: C:/MASTER/seneca/FLTDY/RCS/fmg.cpp 1.4 2000/06/13 14:45:32 COLINJ Exp $
//$Log: fmg.cpp $
//Revision 1.4  2000/06/13 14:45:32  COLINJ
//Reset now uses a define instead of hardcoded number.
//
//Revision 1.3  2000/05/24 15:22:56  colinj
//Pass atmos structure to reset_model and other updates.
//
//Revision 1.2  2000/05/10 09:36:23  colinj
//simulate_model now passed freeze_flag.
//
//Revision 1.1  2000/04/17 10:51:32  colinj
//Initial revision
//

/*---------------------------------------------------------------*/

#include <math.h>
#include <stdio.h>

#ifdef C_PlusPlus
	#include <condit.h>
	#include <fmgdefs.h>
	#include <fmg.h>
#else
	#include "fmgdefs.h"
	#include "fmg.h"
#endif

float FM[6]; //made global so that can see within FM application
float brake_scale = 1.0f;
float nose_press  = 0.0f;
float left_press  = 0.0f;
float right_press = 0.0f;

int right_calc    = FALSE;
int left_calc     = FALSE;

/*****************************************************************************/

void simulate_model( INPUTS *inputs, STATES *states, 
                     OUTPUTS *outputs, ATMOS *atmos, float dt, int freeze_flags)
{
//float FM[6];  see above
STATES dstates;
MASS mass;
int i;

/**
 ** Initialise user state derivatives.  This should floatly go in reset_FMS, but
 ** this doesn't currently have state derivatives passed to it.
 **/
//	for(i=0;i<16;i++) dstates.user[i] = 0.0;

	calc_FM(inputs,states,outputs,atmos,FM,&dstates,&mass);
	calc_RB(FM,&mass,states,outputs,&dstates);
	integrate(states,&dstates,dt,freeze_flags);

	outputs->p_dot = dstates.p;
	outputs->q_dot = dstates.q;
	outputs->r_dot = dstates.r;
	outputs->ax = FM[0]/mass.mass;
	outputs->az = FM[2]/mass.mass;
	if (outputs->Vt<5.0)  // fade out ay between 2 and 5 m/s
	  if(outputs->Vt>2.0) outputs->ay = ((outputs->Vt-2.0)/3.0)*FM[1]/mass.mass;
	  else                outputs->ay = 0.0;
	else
	  outputs->ay = FM[1]/mass.mass;
	outputs->hdot=-dstates.D;
	calc_OP(inputs,states,outputs,atmos);

}

/*****************************************************************************/

void calc_RB( float *FM,      /* INPUT Forces & moments */
              MASS *m,    /* INPUT Mass & inertias */
              STATES *x,  /* INPUT Rigid body xates */
              OUTPUTS *op, /* INPUT Direction cosine matrix */
              STATES *dx) /* OUTPUT State derivatives */
{
float lambda;
static double R = 6378245.0; /* Earth radius (m) */

   dx->u = FM[0]/m->mass - (x->w*x->q-x->v*x->r) + 9.81*op->S[0][2];
   dx->v = FM[1]/m->mass - (x->u*x->r-x->w*x->p) + 9.81*op->S[1][2];
   dx->w = FM[2]/m->mass - (x->v*x->p-x->u*x->q) + 9.81*op->S[2][2];

/////////////////////////Woody Mod 24_01_07 not a prob if this is out ///////////

 // dx->N = x->u*op->S[0][0] + x->v*op->S[1][0] + x->w*op->S[2][0];
 // dx->E = x->u*op->S[0][1] + x->v*op->S[1][1] + x->w*op->S[2][1];
 // dx->D = x->u*op->S[0][2] + x->v*op->S[1][2] + x->w*op->S[2][2];

/////////////////////////Woody Mod 24-01-07 ///////////

   dx->p = ( FM[3] + (m->Iyy - m->Izz)*x->q*x->r + m->Ixz*(op->r_dot+x->p*x->q) )/m->Ixx;
   dx->q = ( FM[4] + (m->Izz - m->Ixx)*x->r*x->p + m->Ixz*(  x->r*x->r-x->p*x->p) )/m->Iyy;
   dx->r = ( FM[5] + (m->Ixx - m->Iyy)*x->p*x->q + m->Ixz*(op->p_dot-x->q*x->r) )/m->Izz;


   lambda = 1.0 - (x->e0*x->e0 + x->e1*x->e1 + x->e2*x->e2 + x->e3*x->e3);

   dx->e0 = -0.5*(             + x->p*x->e1 + x->q*x->e2 + x->r*x->e3 ) + 1.0 * lambda * x->e0;
   dx->e1 = -0.5*( -x->p*x->e0              - x->r*x->e2 + x->q*x->e3 ) + 1.0 * lambda * x->e1;
   dx->e2 = -0.5*( -x->q*x->e0 + x->r*x->e1              - x->p*x->e3 ) + 1.0 * lambda * x->e2;
   dx->e3 = -0.5*( -x->r*x->e0 - x->q*x->e1 + x->p*x->e2              ) + 1.0 * lambda * x->e3;

   dx->latitude  = dx->N / R;
   dx->longitude = dx->E / (R*cos(x->latitude));
}

/*****************************************************************************/

void reset_model( double  height, double  North,   double  East, double lat, double lon,
                  float  speed,  float  heading, float  theta, float gamma, float phi,
                  INPUTS *inputs, STATES *st,  OUTPUTS *outputs, ATMOS *atmos)

{


float c1,c2,c3;
float s1,s2,s3;
//float phi;

   st->N  = North;
   st->E  = East;
   st->D  = -height;

   st->latitude  = lat;
   st->longitude = lon;

   st->u = speed * cos(theta-gamma);
   st->v = 0.0;
   st->w = speed * sin(theta-gamma);
   st->p  = 0.0;
   st->q  = 0.0;
   st->r  = 0.0;

//   phi = 0.0; // phi now supplied as a parameter (MGK 01 Sep 99)
   c1 = cos(phi/2.0); c2 = cos(theta/2.0); c3 = cos(heading/2.0);
   s1 = sin(phi/2.0); s2 = sin(theta/2.0); s3 = sin(heading/2.0);

   st->e0 = c1*c2*c3 + s1*s2*s3;
   st->e1 = s1*c2*c3 - c1*s2*s3;
   st->e2 = c1*s2*c3 + s1*c2*s3;
   st->e3 = c1*c2*s3 - s1*s2*c3;

   outputs->az = 0.0; // make sure these aren't remembered from
   outputs->ay = 0.0; // any previous run
   outputs->az = 0.0;

   reset_FMS(inputs,st); /* Allow FMS states to be initialised */
   calc_OP(inputs,st,outputs,atmos);  /* Outputs need to be valid */

}

/*****************************************************************************/


void calc_OP(INPUTS *u, STATES *st, OUTPUTS *y, ATMOS *a)
{
float e00,e01,e02,e03,e11,e12,e13,e22,e23,e33;
float temp;

/////////////////////////Woody Mod 24_01_07 ///////////
float vN,vE,vD;
/////////////////////////Woody Mod 24_01_07 ///////////

	e00 = st->e0 * st->e0;
	e01 = st->e0 * st->e1;
	e02 = st->e0 * st->e2;
	e03 = st->e0 * st->e3;

	e11 = st->e1 * st->e1;
	e12 = st->e1 * st->e2;
	e13 = st->e1 * st->e3;

	e22 = st->e2 * st->e2;
	e23 = st->e2 * st->e3;

	e33 = st->e3 * st->e3;

	y->S[0][0] =       (e00 + e11 - e22 - e33);
	y->S[0][1] = 2.0 * (e12 + e03            );
	y->S[0][2] = 2.0 * (e13 - e02            );
	y->S[1][0] = 2.0 * (e12 - e03            );
	y->S[1][1] =       (e00 - e11 + e22 - e33);
	y->S[1][2] = 2.0 * (e23 + e01            );
	y->S[2][0] = 2.0 * (e02 + e13            );
	y->S[2][1] = 2.0 * (e23 - e01            );
	y->S[2][2] =       (e00 - e11 - e22 + e33);

	y->psi = atan2(y->S[0][1],y->S[0][0]);
	y->phi = atan2(y->S[1][2],y->S[2][2]);

	     if (y->S[0][2] >  1.0) y->theta = -90.0*D2R;
	else if (y->S[0][2] < -1.0) y->theta =  90.0*D2R;
	else                     y->theta = -asin(y->S[0][2]);

	y->Vt = sqrt(st->u*st->u + st->v*st->v + st->w*st->w);

	if (fabs(st->u) > 1.0e-4)
	  y->alpha = atan2(st->w,st->u);
	else
	{
	  if (st->w > 0.0)
	    y->alpha = +90.0 * D2R;
	  else
	    y->alpha = -90.0 * D2R;
	}

	if (y->Vt > 1.0e-4)
	  y->beta = atan2(st->v,y->Vt);
	else
	{
	  if (st->v > 0.0)
	    y->beta = +90.0 * D2R;
	  else
	    y->beta = -90.0 * D2R;
	}

   if(y->Vt < 0.1)
      y->Vt = 0.01;


	y->phi_dot = st->p + tan(y->theta)*(st->q*sin(y->phi) +st->r*cos(y->phi));
	y->theta_dot = st->q*cos(y->phi) - st->r*sin(y->phi);
	temp=cos(y->theta);
	if (fabs(temp)<0.01) y->psi_dot = 999.9;
	else y->psi_dot = (st->q*sin(y->phi)+st->r*cos(y->phi))/temp;

// Potter has removed code from here to FMS_3FM.cpp ////
/////////////////////////Woody Mod 24_01_07 ///////////
	//	y->Vtas = y->Vt; // Done in FMS, since wind info is not here

	vN = st->u*y->S[0][0] + st->v*y->S[1][0] + st->w*y->S[2][0];
	vE = st->u*y->S[0][1] + st->v*y->S[1][1] + st->w*y->S[2][1];
	vD = st->u*y->S[0][2] + st->v*y->S[1][2] + st->w*y->S[2][2];

	y->Vg    = sqrt(vN*vN + vE*vE);

//	y->gamma = y->theta - y->alpha; // Only works when level!

	if(fabs(y->Vg) > 0.1)
	  y->gamma = atan2(-vD,y->Vg);
	else
	  y->gamma = 0.0;

	if ( fabs(y->Vg) > 0.1)
	{
	  if (fabs(vN) > 0.1)
	    y->track = atan2(vE,vN); // normal case
	  else
	    if (vE > 0.0) // due E/W might cause divide by zero
	      y->track =  90.0*D2R; // due East
	    else
	      y->track = -90.0*D2R; // due West
	}
	else
	  y->track = 0.0; // zero speed case

/////////////////////////Woody Mod 24_01_07 ///////////

	calc_ISA(a->delta_T,a->delta_p,a->delta_r,-st->D,
	  &y->temperature,
	  &y->pressure,
	  &y->density,
	  &y->amach,
	  &y->atm_theta,
	  &y->atm_delta,
	  &y->atm_sigma);

//	calc_ISA(0.0,0.0,0.0,-st->D,
//	  &y->temperature,
//	  &y->pressure,
//	  &y->density,
//	  &y->amach,
//	  &y->atm_theta,
//	  &y->atm_delta,
//	  &y->atm_sigma);

	calc_OP_FMS(u,st,y);
}


void integrate(STATES *states, STATES *dstates, float dt, int freeze_flags)
{
int i;

    if(!(freeze_flags&FREEZE_UVW))
    {
      states->u += dstates->u * dt;
      states->v += dstates->v * dt;
      states->w += dstates->w * dt;
    }

    if(!(freeze_flags&FREEZE_PQR))
    {
      states->p += dstates->p * dt;
      states->q += dstates->q * dt;
      //states->r += dstates->r * dt;
    }

    if(!(freeze_flags&FREEZE_HEAD))
    {
       //states->p += dstates->p * dt;
      //states->q += dstates->q * dt;
      states->r += dstates->r * dt;

    }

    if(!(freeze_flags&FREEZE_ANGLES))
    {
      states->e0 += dstates->e0 * dt;
      states->e1 += dstates->e1 * dt;
      states->e2 += dstates->e2 * dt;
      states->e3 += dstates->e3 * dt;
    }

    if(!(freeze_flags&FREEZE_NED))
    {
      states->N += dstates->N * (double)dt;
      states->E += dstates->E * (double)dt;
      states->D += dstates->D * (double)dt;
    }

    if(!(freeze_flags&FREEZE_LATLON))
    {
      states->latitude  += dstates->latitude  * (double)dt;
      states->longitude += dstates->longitude * (double)dt;
    }

    if(!(freeze_flags&FREEZE_USER))
    {
      for (i=0;i<N_USER_STATES;i++)
        states->user[i] = states->user[i] + (dstates->user[i] * dt);
    }
}


/**
 **  Function to calculate aerodynamic parameters for the International
 **  Standard Atmosphere (ISA).
 **/


void calc_ISA( float delta_t,   /* INPUT temp deviation from ISA (K or C) */
               float delta_p,   /* INPUT press deviation from ISA (N/m^2) */
               float delta_rho, /* INPUT density deviation from ISA (kg/m^3) */
               float h,         /* INPUT physical height AMSL (m) */
               float *t,        /* OUTPUT temperature (K) */
               float *p,        /* OUTPUT pressure (N/m^2) */ 
               float *rho,      /* OUTPUT density (kg/m^3) */ 
               float *a,        /* OUTPUT sonic velocity (m/s) */
               float *omega,    /* OUTPUT temperature ratio */
               float *delta,    /* OUTPUT pressure ratio */
               float *sigma)    /* OUTPUT density ratio */
{
float   t_ISA = 288.15;   /* deg K */
float   p_ISA = 101325.0; /* N/m^2 */
float rho_ISA = 1.2256;    /* kg/m^3 */
float t0, p0, rho0;

   t0   =   t_ISA + delta_t;
   p0   =   p_ISA + delta_p;
   rho0 = rho_ISA + delta_rho;
   
   //Added by SAH 19/3/97 after code reviewed by fest
   //inserted to stop instability in atmosphere calculations
   if ( h < -100.0 ) h = -100.0;
   //end change
   
   *t     = t0 - 0.0065*h;
   *omega = *t/t0;
   *delta = pow((*t)/t0,5.2558797);
   *sigma = pow((*t)/t0,4.2558797);
   *p     =   p0 * (*delta);
   *rho   = rho0 * (*sigma);
   *a     = 20.047*sqrt(*t);
}


int calc_gear(GEAR *gear, STATES *x, OUTPUTS *y, float ht, 
                float steer, float L_brake, float R_brake, float FM[6])
{
float FM_N[6],FM_L[6],FM_R[6];
int cN,cL,cR;



   cN = calc_gear_leg(gear->x[0],gear->y[0],gear->xl[0],
                      x->u, x->v, x->w,
                      x->p, x->q, x->r,
                 ht, y->theta, y->phi,
                      gear->k[0],gear->b[0],gear->preload[0],
                      0.0, steer, FM_N);

   left_calc = TRUE;
   cL = calc_gear_leg(gear->x[1],gear->y[1],gear->xl[1],
                      x->u, x->v, x->w,
                      x->p, x->q, x->r,
                 ht, y->theta, y->phi,
                      gear->k[1],gear->b[1],gear->preload[1],
                      L_brake, 0.0, FM_L);
   left_calc = FALSE;
   right_calc = TRUE;
   cR = calc_gear_leg(gear->x[2],gear->y[2],gear->xl[2],
                      x->u, x->v, x->w,
                      x->p, x->q, x->r,
                 ht, y->theta, y->phi,
                      gear->k[2],gear->b[2],gear->preload[2],
                      R_brake, 0.0, FM_R);
   right_calc = FALSE;
   FM[0] = FM_N[0] + FM_L[0] + FM_R[0];
   FM[1] = FM_N[1] + FM_L[1] + FM_R[1];
   FM[2] = FM_N[2] + FM_L[2] + FM_R[2];
   FM[3] = FM_N[3] + FM_L[3] + FM_R[3];
   FM[4] = FM_N[4] + FM_L[4] + FM_R[4];
   FM[5] = FM_N[5] + FM_L[5] + FM_R[5];

   return(cN + (cL<<1) + (cR<<2));
}


int calc_gear_leg(float x,  float y,  float xl,
                    float u,  float v,  float w, 
                    float p,  float q,  float r, 
                    float h, float theta, float phi,
                    float k,  float b,  float preload,
                    float brake, float steer, float FM[6])
{
float compression, rate, R, Rk, Rb, transverse;
float X_tyre,Y_tyre,X_ac,Y_ac;
float kx, ky;
float mu_x,mu_y;
int contact;
float factor;

	compression = xl - h - theta*x + phi*y;

     if(left_calc == TRUE)
     {
        left_press = compression;
     }
     else
     if(right_calc == TRUE)
     {
        right_press = compression;
     }

     nose_press = (right_press + left_press)/2.0f;


	if(compression > -1.0)

//   if(compression > 0.0)
	{
	  contact = 1;
	  rate    = w - x*q + p*y;
//	  R       = preload + k*compression + b*rate;

      if (compression > 1.0) factor = 1.0;
      else                   factor = compression/1.0;

     //factor = 1.0;
	  Rk      = preload + k*compression;
	  Rb      = b*rate;
//	  if (Rb < 0.0) Rb = 0.0;
	  R = factor*(Rk + Rb);
	  if (R < 0.0) R = 0.0;

	  ky = 1.0;
//	  kx = 1.0 * (0.001 + 0.999*brake);
	  kx = 1.0; // friction "slope"

//	  mu_x = kx * (u + v*steer);
	  mu_x = kx * (u);
	  if (mu_x > +0.7) mu_x = +0.7;
	  if (mu_x < -0.7) mu_x = -0.7;

//	  if (mu_x < -0.1) mu_x = -0.1; // not as good going backwards!

//	  mu_x *= (0.01 + 0.99*brake); // scale by brakes
//	  mu_x *= (0.001 + 0.999*brake); // scale by brakes
	  mu_x *= (brake * brake_scale); // scale by brakes

	  transverse = v + r*x;
	  mu_y = ky * (transverse - u*steer);
	  if (mu_y > +0.7) mu_y = +0.7;
	  if (mu_y < -0.7) mu_y = -0.7;

	  X_tyre = -R * mu_x;
	  Y_tyre = -R * mu_y;

	  X_ac  = X_tyre - Y_tyre*steer;
	  Y_ac  = Y_tyre + X_tyre*steer;

     FM[0] = +X_ac + R*theta;
     FM[1] = +Y_ac;
     FM[2] = -R;
     FM[3] = -(Y_ac*(xl-compression) + R*y);
     FM[4] = +(X_ac*(xl-compression) + R*x);
     FM[5] = +Y_ac*x - X_ac*y;
   }
   else
   {
     contact = 0;
     FM[0] = 0.0; FM[1] = 0.0; FM[2] = 0.0;
     FM[3] = 0.0; FM[4] = 0.0; FM[5] = 0.0;
   }
   return(contact);
}

#define CDMAX 1.2

void calc_Cl_Cd(float alpha, float Cla, float dCl, float alpha_0, float alpha_s, float Cd0, float Cdi, float *Cl, float *Cd)
{
float alpha_1, alpha_2;
float Clmax;

/* First work out small perturbation values of Cl & Cd **/

   *Cl = Cla * (alpha - alpha_0) + dCl;
   *Cd = Cd0 + Cdi*(*Cl)*(*Cl);

/* Cl needs modifying if outside the stall limits */

   alpha_1 =   (-2*alpha_s - alpha_0);
   alpha_2 =   ( 2*alpha_s - alpha_0);

//   if (alpha >  alpha_s) *Cl = -Cla*(alpha-alpha_2);
   Clmax = Cla*(alpha_s-alpha_0);
   if (alpha >  alpha_s) *Cl = Clmax-0.5*Cla*(alpha-alpha_s);

   if (alpha < -alpha_s) *Cl = -Cla*(alpha-alpha_1);

   if (alpha > alpha_2) *Cl = 0.0;
   if (alpha < alpha_1) *Cl = 0.0;

/* Cd value just needs clipping if greater than CDMAX */

   if (*Cd > CDMAX) *Cd = CDMAX;
}

/** 1D Table Lookup function **/

float table1D(float *xt, float *yt, int n, float x)
{
int i;
float f;

   if(x <= xt[0])
     return (yt[0]);

   if(x >= xt[n-1])
     return (yt[n-1]);

   i=0;
   while(x>xt[i])
     i++;

   f = (x-xt[i-1])/(xt[i]-xt[i-1]);

   return(f*yt[i] + (1.0-f)*yt[i-1]);
}


                                              