// File: Config.h

// Seneca dynamics config structure

//**************************
//   RCS Markers           *
//**************************

//$Header: C:/master/seneca/RCS/config.h 1.1 2000/04/14 18:01:36 colinj Exp $
//$Log: config.h $
//Revision 1.1  2000/04/14 18:01:36  colinj
//Initial revision
//

/*---------------------------------------------------------------*/

#define RC_LEFT 1
#define RC_RIGHT 2


typedef struct
{
   unsigned visual     :1;
   unsigned instructor :1;
   unsigned sound      :1;
   unsigned awe32      :1;
   unsigned hardware   :1;
   unsigned screens    :1; //ULTIMA MODIFICACION DE FTE
   unsigned senderr    :1;  //AÑADIDO POR MANOLO

   char         Vname[15];
   char         Iname[15];
   char         Sname[15];
   char         Hname[15];  
   char         Cname[15];
   char         Xname[15];  //AÑADIDO POR MANOLO
}sen_config;



typedef struct
{
	float throt_zero[2];
	float prop_zero[2];
	float mix_zero[2];
	float throt_100[2];
	float prop_100[2];
	float mix_100[2];
	float throt_dead[2];
	float prop_dead[2];
	float mix_dead[2];
   float throt_scale[2];
	float prop_scale[2];
	float mix_scale[2];

	float oil_temp_scale[2];
	float oil_press_scale[2];
	float cyl_scale[2];
	float egt_scale[2];
	float fuel_flow_scale[2];
	float fuel_scale[2];

	float suction_scale;
	float ammeter_scale;
}scale_struct;

