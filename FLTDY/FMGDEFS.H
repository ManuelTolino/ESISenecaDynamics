//**************************
//   RCS Markers           *
//**************************

//$Header: C:/MASTER/seneca/FLTDY/RCS/fmgdefs.h 1.3 2000/06/13 14:48:27 COLINJ Exp $
//$Log: fmgdefs.h $
//Revision 1.3  2000/06/13 14:48:27  COLINJ
//Set number of user states to 41 instead of 128
//
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

#define PI 3.141592f

#define MPS2KN 1.9427f
#define KN2MPS (1.0f/MPS2KN)

#define D2R (PI/180.0f)
#define R2D (180.0f/PI)

#define RPS2RPM  9.54929f
#define RPM2RPS  0.10472f

#define LBS2KG         0.45359f
#define KG2LBS         2.20462f

#define USGAL2UKGAL    0.83267f
#define UKGAL2USGAL    1.2010f

#define INHG2MB 33.8639f
#define MB2INHG 2.953e-2
#define PA2INHG 2.953e-4

#define m2ft (1 / 0.3048)

#define TRUE 1
#define FALSE 0

#define FREEZE_UVW         1  //FREEZE VELOCITYS
#define FREEZE_PQR         2  //ANGULAR VELOCITYS
#define FREEZE_ANGLES      4  //FREEZE ALL ANGULAR MOVEMENT
#define FREEZE_NED         8  //FREEZE POSITIONS (NORTH EAST DOWN)
#define FREEZE_LATLON     16  //FREEZE LAT LONG
#define FREEZE_USER       32  //FREEZE ALL ENGINE STATES
#define FREEZE_HEAD       64  //freeze heading only

#define FREEZE_RB_ONLY  (FREEZE_UVW+FREEZE_PQR+FREEZE_ANGLES+FREEZE_NED+FREEZE_LATLON+FREEZE_HEAD)

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
	//OUTPUT
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
	//STATES
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
	//MASS
	float mass;
    float Ixx;
    float Iyy;
    float Izz;
    float Ixz;
	//ATMOS
	float wN;
	float wE;
	float wD;
	float delta_T;
	float delta_p;
	float delta_r;
	float ht;
} SIMULINKfloat; //SIMULINK


typedef struct {
	double N;
	double E;
	double D;
	double latitude;
	double longitude;
} SIMULINKdouble; //SIMULINK

typedef struct {
	double latitude;
	double longitude;
	double elev;
	float phi;
	float theta;
	float psi;
} XPythonPlugin; //XPython Plugin

typedef struct {
    char header[5];
	float pad;
	double latitude;
	double longitude;
	double elev;
	float psi;
	float theta;
	float phi;
} xplaneDREF;


typedef struct {
    char header[5];
	int p;
	double latitude;
	double longitude;
	double altitude;
	float psi;
	float theta;
	float phi;
} FGProtocol; //FGPROTOCOL

typedef struct {
	float x[3];
	float y[3];
	float xl[3];
	float k[3];
	float b[3];
	float preload[3];
} GEAR;
         //==========================ESTRUCTURA DE PRUEBA  //A�ADIDO POR MANOLO

