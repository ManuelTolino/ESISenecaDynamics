/*
File  : etherhost.cpp
Prefix: ET_

Module: Dynamics for Seneca

Description: This file contains ethernet function calls
             for the host computer
*/

/***************************/
/*   RCS Markers           */
/***************************/

/*

3_12_08 AÑADIDO SCREENS
$Header: C:/MASTER/seneca/RCS/etherhost.cpp 1.1 2000/04/14 16:52:18 colinj Exp $
$Log: etherhost.cpp $
Revision 1.1  2000/04/14 16:52:18  colinj
Initial revision

*/

/*----------------------------------------------------------------------*/

/*----------------*/
/* Include Files  */
/*----------------*/

#include <stdio.h>
#include <windows.h>
#include <string.h>
#include <fstream.h>
#include <winsock.h>
#include <memory.h>
#include <conio.h>


#include "dat_stru\in_head.h"
#include "dat_stru\out_head.h"

#include "FLTDY\Fmgdefs.h" //AÑADIDO 1/04

#include "config.h"
#include "nav\dyn_ios.h"
#include "faults.h"
#include "dyn_snd.h"
/*----------------*/
/* Defines        */
/*----------------*/
/*
#define NET_ID_BROADCAST    0xFFFFFFFFL
#define NET_MAX_PACKET_SIZE 512 //??????
#define	WINSOCK_VERSION	    0x0101
#define NET_BROADCAST_PORT  2700
*/

#define MAX_PACKET_SIZE 1472  // pc limit ?
#define XPLANE_PACKET_SIZE 128

 
#define MAJOR_VERSION   1
#define MINOR_VERSION   1
#define WSA_MAKEWORD(x,y)       ((y) * 256 + (x)) /* HI:Minor, LO:Major */

#define bcopy(s,d,l)  memcpy(d,s,l)

/* use the UDP ttytst source port for test */
#define SOCK_DISCARD  12000
#define SOCK_DISCARD1  13000 //MODIFICADO Beaterio

#define COMS_BREAK_TIME 2.0f

/*----------------*/
/* Typedefs       */
/*----------------*/

/*--------------------*/
/* External Routines  */
/*--------------------*/

extern int RR_encode_vis_buffer(void);
extern int RR_decode_vis_buffer(void);
extern void IOS_set_data_to_ios(dyn_to_ios *dyn2ios);
extern void IOS_get_data_from_ios(ios_to_dyn *ios2dyn);
extern void SC_setup_eth_snd_data(dyn_to_snd *dyn2snd);

#ifdef HARNESS_INPUT
extern void SE_update_from_sen_kbd();
#endif
/*--------------------*/
/* External Variables */
/*--------------------*/

extern dyn_to_ios IO_dyn_ios;
extern OUTPUTS outputs;
extern FGProtocol FGOutput;  //FGPROTOCOL
extern SIMULINKdouble simulinkdoubledata;  //SIMULINK
extern SIMULINKfloat SIMULINKfloatdata;  //SIMULINK

extern sen_in_struct *IO_Sen_in;
extern sen_qtg_struct *IO_Sen_qtg;
extern sen_out_struct *IO_Sen_out;
extern sen_screens_struct *IO_Sen_screens;
extern sen_config SE_config;
extern sen_fault_struct IO_Sen_faults;

extern OUTPUTin *OUTPUTinbox; //SIMULINK RECEIVE


extern float delta_time;
extern float testvalue;

#ifdef SND_TEST
extern int SE_nav1,SE_nav2;
#endif

/*--------------------*/
/* Globals            */
/*--------------------*/
dyn_to_snd *ET_tx_snd;
dyn_to_ios ET_dyn_data;
char       ET_ios_data_in[MAX_PACKET_SIZE]; // From IOS (larger than dyn_to_ios to allow for extra vis data)
//char       BUFFERinbox[MAX_PACKET_SIZE];
ios_to_dyn *ET_ios_data_p;

sen_in_struct recv_sen_in;
OUTPUTin BUFFERinbox;
sen_qtg_struct recv_sen_qtg;
//OUTPUTin BUFFERinbox; //bufferlocal

/*
int sendtest=5;         //==========================VALOR DE PRUEBA  //AÑADIDO POR MANOLO
*/

char RTX_Data[MAX_PACKET_SIZE]; // to/from graphics

/*--------------------*/
/* Locals             */
/*--------------------*/

SOCKET sd;

static sockaddr_in dest;
static sockaddr_in H_W_Dest;        /* Does this have to be global?? */
static sockaddr_in IOS_Dest;        /* Does this have to be global?? */
static sockaddr_in VIS_Dest;        /* Does this have to be global?? */
static sockaddr_in SND_Dest;        /* Does this have to be global?? */
static sockaddr_in SCR_Dest;
static sockaddr_in SCR_Dest1; 

static sockaddr_in SENDERR_Dest;    //AÑADIDO POR MANOLO
static sockaddr_in SENDERR_Dest2;    //AÑADIDO POR MANOLO_12_04
static sockaddr_in SENDERR_Dest3;    //AÑADIDO POR MANOLO_12_04

//static sockaddr_in SENDERR_Visual;    //FLIGHTGEAR_14_08_19 

static fd_set      Fdset;
static timeval     Timeout;
static hostent     *host_ptr;


static int socket_open = 0;
static int H_W_Socket_desc; // Connection socket descriptor for hardware
static int IOS_Socket_desc; // Connection socket descriptor for instructor
static int VIS_Socket_desc;  // Connection socket descriptor for Visuals
static int SND_Socket_desc;  // Connection socket descriptor for sound
static int SCR_Socket_desc; // Connection socket descriptor for hardware
static int SCR_Socket_desc1; // Connection socket descriptor for hardware
static int SENDER_Socket_desc;  //AÑADIDO POR MANOLO
static int SENDER_Socket_desc2;  //AÑADIDO POR MANOLO_12_04
static int SENDER_Socket_desc3;  //AÑADIDO POR MANOLO_12_04

//static int SENDER_Socket_descVis;  //AÑADIDO POR MANOLO_12_04 //FLIGHTGEAR_14_08_19 


/*--------------------*/
/* Forward References */
/*--------------------*/

void EtherCleanup();
int  close_sock(void);
int initialise_socket(void);

//static int ET_ex_SIMULINKoutputs(); 

/*----------------------------------------------------------------------*/

initialise_socket_fns()
{

   WORD wMajorVersion;
   WORD wMinorVersion;
   WORD VersionReqd;
   WSADATA wsocket_data;



	wMajorVersion = MAJOR_VERSION;
   wMinorVersion = MINOR_VERSION;
   VersionReqd   = (unsigned short)WSA_MAKEWORD(wMajorVersion, wMinorVersion);


   WSAStartup(VersionReqd, &wsocket_data );

   if (LOBYTE (wsocket_data.wVersion) != MINOR_VERSION ||
		 HIBYTE (wsocket_data.wVersion) != MAJOR_VERSION
		)
   {
		printf("Invalid version set\n");
   
   
      WSACleanup();
      return FALSE;
   }
   else
   {

   }

   socket_open = 1;

	return TRUE;
}
/*----------------------------------------------------------------------*/
int initialise_socket(void)
{
	unsigned long bl = 1;


	dest.sin_family = AF_INET;
	dest.sin_port   = htons(SOCK_DISCARD);
	sd = socket(AF_INET,SOCK_DGRAM, 0);
	bind(sd, (struct sockaddr *) &dest, sizeof(dest));

	ioctlsocket(sd, FIONBIO,  &bl );

	return 0;
}

/*----------------------------------------------------------------------*/ 
void initialise_ethernet(void)
{
	initialise_socket_fns();
   initialise_socket();

}
/*----------------------------------------------------------------------*/
void close_ethernet(void)
{
   EtherCleanup();
}
/*----------------------------------------------------------------------*/
int close_sock(void)
{
   if(closesocket(sd)==SOCKET_ERROR && socket_open)
   {
 
      return FALSE;
   }

   socket_open = 0;
   
   return TRUE;

}
/*----------------------------------------------------------------------*/
void EtherCleanup()
{
   close_sock();

   if(WSACleanup() == SOCKET_ERROR)
   {  
   } 
}

/*----------------------------------------------------------------------*/
int ET_init_visual()
{
	int  ret;
	char ptr = 0;

	unsigned long  non_blocking = 1l;

	host_ptr = gethostbyname(SE_config.Xname);
	//host_ptr = gethostbyname(SE_config.Xname); //CONFIGURADO PARA XSENDER
//host_ptr = gethostbyname(SE_config.Vname);

	VIS_Socket_desc = socket(PF_INET, SOCK_DGRAM, 0);

	if (setsockopt(VIS_Socket_desc, SOL_SOCKET, SO_BROADCAST, &ptr, sizeof(int)) == -1)
	{
		printf("setsockopt error");
		return 0;
	}

	VIS_Dest.sin_family = PF_INET;
	VIS_Dest.sin_port = htons( (int)SOCK_DISCARD );
	bcopy(host_ptr->h_addr, &VIS_Dest.sin_addr, host_ptr->h_length);

	if (connect(VIS_Socket_desc, (struct sockaddr *) &VIS_Dest, sizeof(VIS_Dest)) == -1)
	{
		printf("connect error");
		return 0;
	}


	ret = ioctlsocket(VIS_Socket_desc, FIONBIO, &non_blocking);

	if (ret == -1)
	{
		printf("ioctl error");
		return 0;
	}

	FD_ZERO(&Fdset);
	Timeout.tv_sec = 0;
	Timeout.tv_usec = 0;  /* select timeout values */

	return 1;
}
/*----------------------------------------------------------------------*/
/*----------------------------SEGUNDO VISUAL 14_08------------------------------------------*/


/********* PARA FLIGHT GEAR 

int ET_init_SENDERvisual()
{
	int  ret;
	char ptr = 0;

	unsigned long  non_blocking = 1l;

	host_ptr = gethostbyname(SE_config.Xname);
	//host_ptr = gethostbyname(SE_config.Xname);

	SENDER_Socket_descVis = socket(PF_INET, SOCK_DGRAM, 0);

	if (setsockopt(SENDER_Socket_descVis, SOL_SOCKET, SO_BROADCAST, &ptr, sizeof(int)) == -1)
	{
		printf("setsockopt error");
	}

	SENDERR_Visual.sin_family = PF_INET;
	SENDERR_Visual.sin_port = htons( (int)49009 ); //49000
	bcopy(host_ptr->h_addr, &SENDERR_Visual.sin_addr, host_ptr->h_length);

	

	if (connect(SENDER_Socket_descVis, (struct sockaddr *) &SENDERR_Visual, sizeof(SENDERR_Visual)) == -1)
	{
		printf("connect error");
	}


	ret = ioctlsocket(SENDER_Socket_descVis, FIONBIO, &non_blocking);

	if (ret == -1)
	{
		printf("ioctl error");
	}

	FD_ZERO(&Fdset);
	Timeout.tv_sec = 0;
	Timeout.tv_usec = 0;  // select timeout values

	return 1;
}

/********* PARA FLIGHT GEAR */



/*--------------------------------------SEGUNDO VISUAL 14_08--------------------------------*/

void ET_ex_visual_data()
{
	int bytes_sent;
	//int bytes_sent_tosender;
   int num_bytes;
	int rec;
	
	
   static float no_rec_time = 0.0;

	/* Need to make sure we do not send too much data, else
		receiving PC slows down */

   // format RTX_Data for send purposes. Return no bytes to send
   num_bytes = RR_encode_vis_buffer();


	bytes_sent = send(VIS_Socket_desc, (const char *) &RTX_Data, MAX_PACKET_SIZE, 0);
	//bytes_sent = send(VIS_Socket_desc, (const char *) &RTX_Data, MAX_PACKET_SIZE, 0);
	
    FGOutput.header[0] = 'V';
	FGOutput.header[1] = 'E';
	FGOutput.header[2] = 'H';
	FGOutput.header[3] = 'X';
	FGOutput.header[4] = '\0';

	FGOutput.p = 0;

	//bytes_sent_tosender = send(SENDER_Socket_descVis, (const char *) &FGOutput, sizeof(FGProtocol), 0); //MANOLO

	//printf("\n====================> VISUAL COMS FAIL");
	//printf("\naltitud terreno %.2f", &testvalue);

	rec = FALSE;

	
	while (( recv(VIS_Socket_desc,(char *) &RTX_Data, MAX_PACKET_SIZE, 0)) > 0)
		//while (( recv(VIS_Socket_desc,(char *) &RTX_Data, MAX_PACKET_SIZE, 0)) > 0)
   {
      rec = TRUE;
   }

   if (rec)
   {
      //Since both send and receive use the same buffer, we must NOT try
      //and decode RTX_Data if nothing was received (since that would
      //be decoding the output data!

      RR_decode_vis_buffer();
      no_rec_time = 0.0;
   }
   else
   {
      no_rec_time += delta_time;
      if (no_rec_time > COMS_BREAK_TIME)
      {
         printf("\n====================> VISUAL COMS FAIL");
         no_rec_time = 0.0;
      }
   }
}

//************************************************************************
int ET_init_IOS()
{
	int  ret;
	char ptr = 0;

	unsigned long  non_blocking = 1l;

	host_ptr = gethostbyname(SE_config.Xname);

	IOS_Socket_desc = socket(PF_INET, SOCK_DGRAM, 0);

	if (setsockopt(IOS_Socket_desc, SOL_SOCKET, SO_BROADCAST, &ptr, sizeof(int)) == -1)
	{
		printf("setsockopt error");
		return 0;
	}

	IOS_Dest.sin_family = PF_INET;
	IOS_Dest.sin_port = htons( (int)12005 ); //SOCK_DISCARD antes
	bcopy(host_ptr->h_addr, &IOS_Dest.sin_addr, host_ptr->h_length);

	if (connect(IOS_Socket_desc, (struct sockaddr *) &IOS_Dest, sizeof(IOS_Dest)) == -1)
	{
		printf("connect error");
		return 0;
	}


	ret = ioctlsocket(IOS_Socket_desc, FIONBIO, &non_blocking);

	if (ret == -1)
	{
		printf("ioctl error");
		return 0;
	}

	FD_ZERO(&Fdset);
	Timeout.tv_sec = 0;
	Timeout.tv_usec = 0;  /* select timeout values */

	return 1;
}
/*----------------------------------------------------------------------*/

void ET_ex_ios_data()
{

	int        bytes_sent;
	int        rec;
	ios_to_dyn *ios2dyn;
   

   IOS_set_data_to_ios(&IO_dyn_ios);
      IO_Sen_out->pack28[1]=IO_dyn_ios.dynamics_state ;

   //IO_dyn_ios.ios_ac.columnpos=(short)( IO_Sen_in->control_yoke_pitch_position);

  // printf("\n QTG_GO OFF %f %f %i",IO_dyn_ios.ios_ac.columnforce,IO_Sen_in->test_control_column_force,IO_dyn_ios.ios_ac.columnpos);
   bytes_sent = send(IOS_Socket_desc, (const char *) &IO_dyn_ios, sizeof(dyn_to_ios), 0);


 	rec = FALSE;
   
 	while((recv(IOS_Socket_desc,(char *) &ET_ios_data_in, MAX_PACKET_SIZE, 0)) > 0)
   {
      rec = TRUE;
   }


   if (rec)
   {
      ios2dyn = (ios_to_dyn*) &ET_ios_data_in;
      IOS_get_data_from_ios(ios2dyn);
   }
   else
   {
      //Make sure the visual data count is cleared to zero so we
      //dont try and send bad or old data
      ios2dyn = (ios_to_dyn*) &ET_ios_data_in;
      ios2dyn->vis_length = 0;
   }
   
}
/*----------------------------------------------------------------------*/

/*----------------------------------------------------------------------*/

/*----------------------------------------------------------------------*/

/*----------------------------------------------------------------------*/

int ET_init_hardware()
{
	int  ret;
	char ptr = 0;

	unsigned long  non_blocking = 1l;

	host_ptr = gethostbyname(SE_config.Hname);

	H_W_Socket_desc = socket(PF_INET, SOCK_DGRAM, 0);

	if (setsockopt(H_W_Socket_desc, SOL_SOCKET, SO_BROADCAST, &ptr, sizeof(int)) == -1)
	{
		printf("setsockopt error");
		return 0;
	}

	H_W_Dest.sin_family = PF_INET;
	H_W_Dest.sin_port = htons( (int)SOCK_DISCARD );
	bcopy(host_ptr->h_addr, &H_W_Dest.sin_addr, host_ptr->h_length);

	if (connect(H_W_Socket_desc, (struct sockaddr *) &H_W_Dest, sizeof(H_W_Dest)) == -1)
	{
		printf("connect error");
		return 0;
	}


	ret = ioctlsocket(H_W_Socket_desc, FIONBIO, &non_blocking);

	if (ret == -1)
	{
		printf("ioctl error");
		return 0;
	}
 
	FD_ZERO(&Fdset);
	Timeout.tv_sec = 0;
	Timeout.tv_usec = 0;  

	return 1;
}
/*----------------------------------------------------------------------*/

void ET_ex_hardware_data()
{
	int bytes_sent;
   int rec_flag;
   static float no_rec_time = 0.0;


  // IO_Sen_out->system_DCL_power_control = 1;
   //IO_Sen_out->pitch_demand_force       = 30.0;
   //IO_Sen_out->pitch_demand_position    = 80.0;




   bytes_sent = send(H_W_Socket_desc, (const char *) IO_Sen_out,sizeof(sen_out_struct), 0);
   
	rec_flag = FALSE;
   while(( recv(H_W_Socket_desc,(char *) &recv_sen_in,sizeof(sen_in_struct), 0)) > 0)
   {
      memcpy(IO_Sen_in,&recv_sen_in,sizeof(sen_in_struct));
      rec_flag = TRUE;
	  //PARA LAS SCREENS
	  	IO_Sen_out->alt1_cap_baro = IO_Sen_in->alt1_cap_baro ;
	IO_Sen_out->alt2_cap_baro = IO_Sen_in->alt2_cap_baro  ;
	IO_Sen_out->alt3_fo_baro  = IO_Sen_in->alt3_fo_baro  ;
IO_Sen_out->pilot_hsi_obs = IO_Sen_in->pilot_hsi_obs ;
IO_Sen_out->pilot_hsi_atp = IO_Sen_in->pilot_hsi_atp ;
IO_Sen_out->copilot_hsi_obs = IO_Sen_in->copilot_hsi_obs ;
IO_Sen_out->copilot_hsi_atp = IO_Sen_in->copilot_hsi_atp ;
IO_Sen_out->nav2_obs = IO_Sen_in->nav2_obs ;
IO_Sen_out->pilot_ias_set = IO_Sen_in->pilot_ias_set ;
IO_Sen_out->copilot_ias_set = IO_Sen_in->copilot_ias_set ;
IO_Sen_out->battery_on = IO_Sen_in->elec_battery_switch ;
IO_Sen_out->pack25[0] = IO_Sen_in->pack25[0];
      //IO_Sen_in->test_control_column_force = IO_Sen_in->test_control_column_force - 23.7f;
      //IO_Sen_in->test_control_column_force = IO_Sen_in->test_control_column_force * -2.0f;
   }

   if (!rec_flag)
   {
      no_rec_time += delta_time;
      if (no_rec_time > COMS_BREAK_TIME)
      {
         printf("\n====================> HARDWARE COMS FAIL");
         no_rec_time = 0.0;
      }
   }
   else
      no_rec_time = 0.0;

#ifdef HARNESS_INPUT
   SE_update_from_sen_kbd();
#endif

}

/*----------------------------------------------------------------------*/
/*
int ET_init_hardware()
{
	int  ret;
	char ptr = 0;

	unsigned long  non_blocking = 1l;

	host_ptr = gethostbyname(SE_config.Hname);

	H_W_Socket_desc = socket(PF_INET, SOCK_DGRAM, 0);


	if (setsockopt(H_W_Socket_desc, SOL_SOCKET, SO_BROADCAST, &ptr, sizeof(int)) == -1)
	{
		printf("setsockopt error");
		return 0;
	}



	H_W_Dest.sin_family = PF_INET;
	H_W_Dest.sin_port = htons( (int)SOCK_DISCARD);
	bcopy(host_ptr->h_addr, &H_W_Dest.sin_addr, host_ptr->h_length);

	if (connect(H_W_Socket_desc, (struct sockaddr *) &H_W_Dest, sizeof(H_W_Dest)) == -1)
	{
		printf("connect error");
		return 0;
	}




	ret = ioctlsocket(H_W_Socket_desc, FIONBIO, &non_blocking);

	if (ret == -1)
	{
		printf("ioctl error");
		return 0;
	}
 
	FD_ZERO(&Fdset);
	Timeout.tv_sec = 0;
	Timeout.tv_usec = 0;  

	return 1;
}*/
/*----------------------------------------------------------------------*/
/*
void ET_ex_hardware_data()
{
   int bytes_sent;
   int rec_flag;
   static float no_rec_time = 0.0;


   //IO_Sen_out->system_DCL_power_control = 1;
   //IO_Sen_out->pitch_demand_force       = 30.0;
   //IO_Sen_out->pitch_demand_position    = 80.0;
   //IO_Sen_out->roll_demand_position = 89;
   bytes_sent = send(H_W_Socket_desc, (const char *) IO_Sen_out,sizeof(sen_out_struct), 0);
   
	rec_flag = FALSE;
   while(( recv(H_W_Socket_desc,(char *) &recv_sen_in,sizeof(sen_in_struct), 0)) > 0)
   {
      memcpy(IO_Sen_in,&recv_sen_in,sizeof(sen_in_struct));
      rec_flag = TRUE;

	  // printf("pausa= %i",IO_Sen_in->KFC150_AP_ENG_button );

	  //printf("pausa= %i",IO_Sen_in->system_freeze_button );
      //IO_Sen_in->test_control_column_force = IO_Sen_in->test_control_column_force - 23.7f;
      //IO_Sen_in->test_control_column_force = IO_Sen_in->test_control_column_force * -2.0f;
   }

   if (!rec_flag)
   {
      no_rec_time += delta_time;
      if (no_rec_time > COMS_BREAK_TIME)
      {
         printf("\n====================> HARDWARE COMS FAIL");
         no_rec_time = 0.0;
      }
   }
   else
      no_rec_time = 0.0;

#ifdef HARNESS_INPUT
   SE_update_from_sen_kbd();
#endif

}*/
/*----------------------------------------------------------------------*/
/*----------------------------------------------------------------------*/
int ET_init_sound()
{
	int  ret;
	char ptr = 0;

	unsigned long  non_blocking = 1l;

	host_ptr = gethostbyname(SE_config.Sname);

	SND_Socket_desc = socket(PF_INET, SOCK_DGRAM, 0);

	if (setsockopt(SND_Socket_desc, SOL_SOCKET, SO_BROADCAST, &ptr, sizeof(int)) == -1)
	{
		printf("setsockopt error");
		return 0;
	}

	SND_Dest.sin_family = PF_INET;
	SND_Dest.sin_port = htons( (int)SOCK_DISCARD1 );//MODIFICADO Beaterio
	bcopy(host_ptr->h_addr, &SND_Dest.sin_addr, host_ptr->h_length);

	if (connect(SND_Socket_desc, (struct sockaddr *) &SND_Dest, sizeof(SND_Dest)) == -1)
	{
		printf("connect error");
		return 0;
	}


	ret = ioctlsocket(SND_Socket_desc, FIONBIO, &non_blocking);

	if (ret == -1)
	{
		printf("ioctl error");
		return 0;
	}
 
	FD_ZERO(&Fdset);
	Timeout.tv_sec = 0;
	Timeout.tv_usec = 0;  /* select timeout values */

	return 1;
}
/*----------------------------------------------------------------------*/

void ET_ex_sound_data()
{
	int bytes_sent;


   SC_setup_eth_snd_data(ET_tx_snd);

#ifdef SND_TEST
   unsigned char test[4] = "ABC";
   unsigned char test1[4] = "XYZ";
   memset(ET_tx_snd,0,sizeof(ET_tx_snd));
   if(SE_nav2 == TRUE)
   {
      ET_tx_snd->nav1.on = TRUE;
   }
   else
      ET_tx_snd->nav1.on = FALSE;

   ET_tx_snd->nav1.morse_rate = 15;
   if(SE_nav1 == TRUE)
   {
      strcpy((char *)ET_tx_snd->nav1.morse_str,(const char *)test);
   }
   else
      strcpy((char *)ET_tx_snd->nav1.morse_str,(const char *)test1);
#endif

   bytes_sent = send(SND_Socket_desc, (const char *) ET_tx_snd,sizeof(dyn_to_snd), 0);

}

/*----------------------------------------------------------------------*/

int ET_init_screens()
{
	int  ret;
	char ptr = 0;

	unsigned long  non_blocking = 1l;

	host_ptr = gethostbyname(SE_config.Cname);

	SCR_Socket_desc = socket(PF_INET, SOCK_DGRAM, 0);

	if (setsockopt(SCR_Socket_desc, SOL_SOCKET, SO_BROADCAST, &ptr, sizeof(int)) == -1)
	{
		printf("setsockopt error");
		return 0;
	}

	SCR_Dest.sin_family = PF_INET;
	SCR_Dest.sin_port = htons( (int)18000 );
	bcopy(host_ptr->h_addr, &SCR_Dest.sin_addr, host_ptr->h_length);

	if (connect(SCR_Socket_desc, (struct sockaddr *) &SCR_Dest, sizeof(SCR_Dest)) == -1)
	{
		printf("connect error");
		return 0;
	}


	ret = ioctlsocket(SCR_Socket_desc, FIONBIO, &non_blocking);

	if (ret == -1)
	{
		printf("ioctl error");
		return 0;
	}
 
	FD_ZERO(&Fdset);
	Timeout.tv_sec = 0;
	Timeout.tv_usec = 0;  

	return 1;
}
/*----------------------------------------------------------------------*/

void ET_ex_screens_data()
{
	int rec_flag;
	int bytes_sent;

IO_Sen_screens->alt1_cap_baro =IO_Sen_out->alt1_cap_baro ;
IO_Sen_screens->alt2_cap_baro  =IO_Sen_out->alt2_cap_baro;
IO_Sen_screens->alt3_fo_baro  =IO_Sen_out->alt3_fo_baro;
IO_Sen_screens->pack25[0]  =IO_Sen_out->pack25[0];
IO_Sen_screens->pack25[1]  =IO_Sen_out->pack25[1];
IO_Sen_screens->pack26 =IO_Sen_out->pack26;
IO_Sen_screens->pack27 =IO_Sen_out->pack27;
IO_Sen_screens->pack28[0] =IO_Sen_out->pack28[0];
IO_Sen_screens->pack28[1] =IO_Sen_out->pack28[1];
IO_Sen_screens->pack28[2] =IO_Sen_out->pack28[2];
IO_Sen_screens->io_operating_mode=IO_Sen_out->io_operating_mode;
IO_Sen_screens->pilot_altimeter_1_demand_height=IO_Sen_out->pilot_altimeter_1_demand_height; 
IO_Sen_screens->pilot_altimeter_2_demand_height=IO_Sen_out->pilot_altimeter_2_demand_height ;
IO_Sen_screens->copilot_altimeter_demand_height=IO_Sen_out->copilot_altimeter_demand_height; 
IO_Sen_screens->nav2_to_from_flag =IO_Sen_out->nav2_to_from_flag ;
IO_Sen_screens->nav2_left_right_needle =IO_Sen_out->nav2_left_right_needle ;
IO_Sen_screens->nav2_GS_needle =IO_Sen_out->nav2_GS_needle ;
IO_Sen_screens->nav2_nav_flag =IO_Sen_out->nav2_nav_flag ;
IO_Sen_screens->nav2_GS_flag =IO_Sen_out->nav2_GS_flag ;
IO_Sen_screens->pilot_FCI_pitch_demand_position =IO_Sen_out->pilot_FCI_pitch_demand_position ;
IO_Sen_screens->pilot_FCI_roll_demand_position =IO_Sen_out->pilot_FCI_roll_demand_position ;
//printf("POSICION ROLL: %f\r\n" ,IO_Sen_out->pilot_FCI_roll_demand_position);
IO_Sen_screens->pilot_FCI_cmd_enable =IO_Sen_out->pilot_FCI_cmd_enable ;
IO_Sen_screens->pilot_FCI_cmd_pitch =IO_Sen_out->pilot_FCI_cmd_pitch ;
IO_Sen_screens->pilot_FCI_cmd_roll =IO_Sen_out->pilot_FCI_cmd_roll ;
IO_Sen_screens->copilot_FCI_pitch_demand_position =IO_Sen_out->copilot_FCI_pitch_demand_position ;
IO_Sen_screens->copilot_FCI_roll_demand_position =IO_Sen_out->copilot_FCI_roll_demand_position ;
IO_Sen_screens->copilot_FCI_cmd_enable =IO_Sen_out->copilot_FCI_cmd_enable ;
IO_Sen_screens->copilot_FCI_cmd_pitch =IO_Sen_out->copilot_FCI_cmd_pitch ;
IO_Sen_screens->copilot_FCI_cmd_roll =IO_Sen_out->copilot_FCI_cmd_roll ;
IO_Sen_screens->RMI_demand_heading =IO_Sen_out->RMI_demand_heading;
IO_Sen_screens->RMI_ADF_needle =IO_Sen_out->RMI_ADF_needle ;
IO_Sen_screens->RMI_VOR_needle =IO_Sen_out->RMI_VOR_needle ;
IO_Sen_screens->pilot_hsi_compass_demand_position =IO_Sen_out->pilot_hsi_compass_demand_position;
IO_Sen_screens->pilot_hsi_left_right_needle =IO_Sen_out->pilot_hsi_left_right_needle ;
IO_Sen_screens->pilot_hsi_to_from_needle =IO_Sen_out->pilot_hsi_to_from_needle ;
IO_Sen_screens->pilot_hsi_GS_needle =IO_Sen_out->pilot_hsi_GS_needle ;
IO_Sen_screens->pilot_hsi_nav_flag =IO_Sen_out->pilot_hsi_nav_flag ;
IO_Sen_screens->pilot_hsi_hdg_flag =IO_Sen_out->pilot_hsi_hdg_flag ;
IO_Sen_screens->pilot_hsi_GS_enable =IO_Sen_out->pilot_hsi_GS_enable ;
IO_Sen_screens->copilot_hsi_compass_demand_position =IO_Sen_out->copilot_hsi_compass_demand_position;
IO_Sen_screens->copilot_hsi_left_right_needle =IO_Sen_out->copilot_hsi_left_right_needle ;
IO_Sen_screens->copilot_hsi_to_from_needle =IO_Sen_out->copilot_hsi_to_from_needle ;
IO_Sen_screens->copilot_hsi_GS_needle =IO_Sen_out->copilot_hsi_GS_needle ;
IO_Sen_screens->copilot_hsi_nav_flag =IO_Sen_out->copilot_hsi_nav_flag ;
IO_Sen_screens->copilot_hsi_hdg_flag =IO_Sen_out->copilot_hsi_hdg_flag ;
IO_Sen_screens->copilot_hsi_GS_enable =IO_Sen_out->copilot_hsi_GS_enable ;
IO_Sen_screens->pilot_airspeed =IO_Sen_out->pilot_airspeed ;
IO_Sen_screens->copilot_airspeed =IO_Sen_out->copilot_airspeed ;
IO_Sen_screens->pilot_rate_of_climb =IO_Sen_out->pilot_rate_of_climb ;
IO_Sen_screens->copilot_rate_of_climb =IO_Sen_out->copilot_rate_of_climb ;
IO_Sen_screens->pilot_turn =IO_Sen_out->pilot_turn ;
IO_Sen_screens->pilot_slip =IO_Sen_out->pilot_slip ;
IO_Sen_screens->copilot_turn =IO_Sen_out->copilot_turn ;
IO_Sen_screens->copilot_slip =IO_Sen_out->copilot_slip ;

IO_Sen_screens->left_manifold_pressure =IO_Sen_out->left_manifold_pressure;
IO_Sen_screens->right_manifold_pressure =IO_Sen_out->right_manifold_pressure ;
IO_Sen_screens->left_fuel_flow =IO_Sen_out->left_fuel_flow ;
IO_Sen_screens->right_fuel_flow =IO_Sen_out->right_fuel_flow ;
IO_Sen_screens->left_tacho =IO_Sen_out->left_tacho ;
IO_Sen_screens->right_tacho =IO_Sen_out->right_tacho ;
IO_Sen_screens->left_EGT =IO_Sen_out->left_EGT ;
IO_Sen_screens->right_EGT =IO_Sen_out->right_EGT ;
IO_Sen_screens->left_fuel_quantity =IO_Sen_out->left_fuel_quantity ;
IO_Sen_screens->right_fuel_quantity =IO_Sen_out->right_fuel_quantity ;
IO_Sen_screens->left_oil_pressure =IO_Sen_out->left_oil_pressure ;
IO_Sen_screens->right_oil_pressure =IO_Sen_out->right_oil_pressure ;
IO_Sen_screens->left_oil_temperature =IO_Sen_out->left_oil_temperature ;
IO_Sen_screens->right_oil_temperature =IO_Sen_out->right_oil_temperature ;
IO_Sen_screens->left_cyclic_temperature =IO_Sen_out->left_cyclic_temperature ;
IO_Sen_screens->right_cyclic_temperature =IO_Sen_out->right_cyclic_temperature ;
IO_Sen_screens->pilot_hsi_obs =IO_Sen_out->pilot_hsi_obs ;
IO_Sen_screens->pilot_hsi_atp =IO_Sen_out->pilot_hsi_atp ;
IO_Sen_screens->copilot_hsi_obs =IO_Sen_out->copilot_hsi_obs ;
IO_Sen_screens->copilot_hsi_atp =IO_Sen_out->copilot_hsi_atp ;
IO_Sen_screens->nav2_obs =IO_Sen_out->nav2_obs ;
IO_Sen_screens->pilot_ias_set =IO_Sen_out->pilot_ias_set ;
IO_Sen_screens->copilot_ias_set =IO_Sen_out->copilot_ias_set ;
IO_Sen_screens->battery_on =IO_Sen_in->elec_battery_switch ;


    bytes_sent = send(SCR_Socket_desc, (const char *) IO_Sen_screens,sizeof(sen_screens_struct), 0);
   
	rec_flag = FALSE;
   while(( recv(SCR_Socket_desc,(char *) &recv_sen_qtg,sizeof(sen_qtg_struct), 0)) > 0)
   {
      memcpy(IO_Sen_qtg,&recv_sen_qtg,sizeof(sen_qtg_struct));

      rec_flag = TRUE;
   }

	if (rec_flag)
	{
	//	printf("YAW %f \n",IO_Sen_qtg->qtg_pilot_compass  );
	}

}

/*----------------------------------------------------------------------*/

int ET_init_screens1()
{
	int  ret;
	char ptr = 0;

	unsigned long  non_blocking = 1l;

	host_ptr = gethostbyname(SE_config.Cname);

	SCR_Socket_desc1 = socket(PF_INET, SOCK_DGRAM, 0);

	if (setsockopt(SCR_Socket_desc1, SOL_SOCKET, SO_BROADCAST, &ptr, sizeof(int)) == -1)
	{
		printf("setsockopt error");
		return 0;
	}

	SCR_Dest1.sin_family = PF_INET;
	SCR_Dest1.sin_port = htons( (int)19000 );
	bcopy(host_ptr->h_addr, &SCR_Dest1.sin_addr, host_ptr->h_length);

	if (connect(SCR_Socket_desc1, (struct sockaddr *) &SCR_Dest1, sizeof(SCR_Dest1)) == -1)
	{
		printf("connect error");
		return 0;
	}


	ret = ioctlsocket(SCR_Socket_desc1, FIONBIO, &non_blocking);

	if (ret == -1)
	{
		printf("ioctl error");
		return 0;
	}
 
	FD_ZERO(&Fdset);
	Timeout.tv_sec = 0;
	Timeout.tv_usec = 0;  

	return 1;
}
/*----------------------------------------------------------------------*/

void ET_ex_screens_data1()
{
	int bytes_sent;

   bytes_sent = send(SCR_Socket_desc1, (const char *) IO_Sen_screens,sizeof(sen_screens_struct), 0);
   


}
/*----------------------------------------------------------------------*/
/*------TESTING_27_03_19--------------------*/



/*------TESTING_27_03_19--------------------*/
/*------TESTING_12_04_2019------------------*/


/*Creacion de un nuevo socket para flujo de datos de las pantallas.
 Se exporta la misma estructura que se envia al programa Screens*/

int ET_init_SIMULINKscr()  //AÑADIDO POR MANOLO_12_04
{
	int  ret;
	char ptr = 0;

	unsigned long  non_blocking = 1l;

	host_ptr = gethostbyname(SE_config.Xname);

	SENDER_Socket_desc2 = socket(PF_INET, SOCK_DGRAM, 0);

	if (setsockopt(SENDER_Socket_desc2, SOL_SOCKET, SO_BROADCAST, &ptr, sizeof(int)) == -1)
	{
		printf("setsockopt error");
		return 0;
	}

	SENDERR_Dest2.sin_family = PF_INET;
	SENDERR_Dest2.sin_port = htons( (int)19600 );
	bcopy(host_ptr->h_addr, &SENDERR_Dest2.sin_addr, host_ptr->h_length);

	if (connect(SENDER_Socket_desc2, (struct sockaddr *) &SENDERR_Dest2, sizeof(SENDERR_Dest2)) == -1)
	{
		printf("connect error");
		return 0;
	}


	ret = ioctlsocket(SENDER_Socket_desc2, FIONBIO, &non_blocking);

	if (ret == -1)
	{
		printf("ioctl error");
		return 0;
	}
 
	FD_ZERO(&Fdset);
	Timeout.tv_sec = 0;
	Timeout.tv_usec = 0;  

	return 1;
}
/*----------------------------------------------------------------------*/


void ET_ex_SIMULINKscr()  //AÑADIDO POR MANOLO_12_04
{
	int bytes_sent;

   /*bytes_sent = send(SENDER_Socket_desc, (const char *) &sendtest,sizeof(sendtest), 0);*/
   /*sendtest = sendtest + 1;*/  // Contador para pruebas

   bytes_sent = send(SENDER_Socket_desc2, (const char *) &SIMULINKfloatdata,sizeof(SIMULINKfloat), 0);


   /*
	OUTPUTinbox.a = OUTPUTinbox.a + 0.01;
	OUTPUTinbox.b = OUTPUTinbox.b + 0.01;
	OUTPUTinbox.c = OUTPUTinbox.c + 0.01;

	*/
}

/*Creacion de un nuevo socket para flujo de datos desde HARDWARE A DYNAMICS.
 Se exporta la misma estructura que se envia al programa Screens*/

int ET_init_SIMULINKhdw()  //AÑADIDO POR MANOLO_12_04
{
	int  ret;
	char ptr = 0;

	unsigned long  non_blocking = 1l;

	host_ptr = gethostbyname(SE_config.Xname);

	SENDER_Socket_desc3 = socket(PF_INET, SOCK_DGRAM, 0);

	if (setsockopt(SENDER_Socket_desc3, SOL_SOCKET, SO_BROADCAST, &ptr, sizeof(int)) == -1)
	{
		printf("setsockopt error");
		return 0;
	}

	SENDERR_Dest3.sin_family = PF_INET;
	SENDERR_Dest3.sin_port = htons( (int)19700 );
	bcopy(host_ptr->h_addr, &SENDERR_Dest3.sin_addr, host_ptr->h_length);

	if (connect(SENDER_Socket_desc3, (struct sockaddr *) &SENDERR_Dest3, sizeof(SENDERR_Dest3)) == -1)
	{
		printf("connect error");
		return 0;
	}


	ret = ioctlsocket(SENDER_Socket_desc3, FIONBIO, &non_blocking);

	if (ret == -1)
	{
		printf("ioctl error");
		return 0;
	}
 
	FD_ZERO(&Fdset);
	Timeout.tv_sec = 0;
	Timeout.tv_usec = 0;  

	return 1;
}
/*----------------------------------------------------------------------*/


void ET_ex_SIMULINKhdw()  //AÑADIDO POR MANOLO_12_04
{
	int bytes_sent;

   /*bytes_sent = send(SENDER_Socket_desc, (const char *) &sendtest,sizeof(sendtest), 0);*/
   /*sendtest = sendtest + 1;*/  // Contador para pruebas

   bytes_sent = send(SENDER_Socket_desc3, (const char *) &simulinkdoubledata,sizeof(SIMULINKdouble), 0);

}


/*
int ET_init_SIMULINKoutputs()  //AÑADIDO POR MANOLO
{
	int  ret;
	char ptr = 0;

	unsigned long  non_blocking = 1l;

	host_ptr = gethostbyname("192.128.134.189");

	SENDER_Socket_desc = socket(PF_INET, SOCK_DGRAM, 0);

	if (setsockopt(SENDER_Socket_desc, SOL_SOCKET, SO_BROADCAST, &ptr, sizeof(int)) == -1)
	{
		printf("setsockopt error");
		return 0;
	}

	SENDERR_Dest.sin_family = PF_INET;
	SENDERR_Dest.sin_port = htons( (int)19750);
	//SENDERR_Dest.sin_port = htons( (int)19500);
	bcopy(host_ptr->h_addr, &SENDERR_Dest.sin_addr, host_ptr->h_length);

	if (connect(SENDER_Socket_desc, (struct sockaddr *) &SENDERR_Dest, sizeof(SENDERR_Dest)) == -1)
	{
		printf("connect error");
		return 0;
	}


	ret = ioctlsocket(SENDER_Socket_desc, FIONBIO, &non_blocking);

	if (ret == -1)
	{
		printf("ioctl error");
		return 0;
	}
 
	FD_ZERO(&Fdset);
	Timeout.tv_sec = 0;
	Timeout.tv_usec = 0;  



	return 1;
}
*/
/*----------------------------------------------------------------------*/

/*
void ET_ex_SIMULINKoutputs()  //AÑADIDO POR MANOLO
{
	//int bytes_REC;
	int        rec;
	OUTPUTin *POINTERinbox;

	rec = FALSE;

   /*bytes_sent = send(SENDER_Socket_desc, (const char *) &sendtest,sizeof(sendtest), 0);*/
   /*sendtest = sendtest + 1;*/  // Contador para pruebas

   //bytes_REC = send(SENDER_Socket_desc, (const char *) &outputs,sizeof(outputs), 0);
   //bytes_REC = recv(SENDER_Socket_desc,(char *) &BUFFERinbox,sizeof(OUTPUTin), 0);

   //POINTERinbox = (OUTPUTin*) &BUFFERinbox;
   //memcpy(OUTPUTinbox,&BUFFERinbox,sizeof(OUTPUTin));

      /*while(( recv(H_W_Socket_desc,(char *) &recv_sen_in,sizeof(sen_in_struct), 0)) > 0)
   {
      memcpy(IO_Sen_in,&recv_sen_in,sizeof(sen_in_struct));
	  */

	//printf("\naltitud terreno %.2f", &POINTERinbox->a);
	//printf("\nBytes recibidos %d", &bytes_REC);
   /*
	OUTPUTinbox.a = OUTPUTinbox.a + 0.01;
	OUTPUTinbox.b = OUTPUTinbox.b + 0.01;
	OUTPUTinbox.c = OUTPUTinbox.c + 0.01;

	*/    /*

   
 	while((recv(SENDER_Socket_desc,(char *) &BUFFERinbox,MAX_PACKET_SIZE, 0)) > 0)
   {
      rec = TRUE;
   }


   if (rec)
   {
      POINTERinbox = (OUTPUTin*) &BUFFERinbox;
      	//printf("\naltitud terreno %.2f", &POINTERinbox->a);
	printf("\nBytes recibidos");
   }
   else
   {
	//printf("\nNO RECIBO NADA");
   }

	
}

          */
/******************TEST DE RECEPTOR UDP********************/ 

int ET_init_SIMULINKoutputs()
{
   

   unsigned long bl = 1;

   // bzero(&dest,sizeof(dest)); // set to zero
//   bcopy(host_ptr->h_addr,&dest.sin_addr,host_ptr->h_length); // defined above
   SENDERR_Dest.sin_family = AF_INET;
   SENDERR_Dest.sin_port   = htons((int)19750);
   SENDER_Socket_desc = socket(AF_INET,SOCK_DGRAM, 0);


   bind(SENDER_Socket_desc, (struct sockaddr *) &SENDERR_Dest, sizeof(SENDERR_Dest));

   // ioctl(sd, FIONBIO,&non_blocking);
   ioctlsocket(SENDER_Socket_desc, FIONBIO,  &bl );

return 1;
}
/**********************************************************/
int ET_ex_SIMULINKoutputs()
{

   int tmp;   
   int i;
   //OUTPUTin *POINTERinbox;
   static float no_rec_time = 0.0;
   tmp = sizeof(SENDERR_Dest);
/*
   if (readit)
   {
      fread(&ETH_RX_from_dyn, 200, 1,exout);
      return 1;
   }
*/

   if ( (i = recvfrom(SENDER_Socket_desc, (char * ) &BUFFERinbox, sizeof(OUTPUTin), 0, (struct sockaddr *) &SENDERR_Dest, &tmp)) > 0)
   {
	   //printf("\nBytes recibidos");
      while (recvfrom(SENDER_Socket_desc, (char * ) &BUFFERinbox, sizeof(OUTPUTin), 0, (struct sockaddr *) &SENDERR_Dest, &tmp) > 0) 
	  {
		  //printf("\nBytes recibidos");
		//POINTERinbox = (OUTPUTin*) &BUFFERinbox;
		  memcpy(OUTPUTinbox,&BUFFERinbox,sizeof(OUTPUTin));
	  no_rec_time += delta_time;
      if (no_rec_time > COMS_BREAK_TIME)
		{

         no_rec_time = 0.0;

		//OUTPUTinbox->a = POINTERinbox->a;
		//OUTPUTinbox->b = POINTERinbox->b;
		//OUTPUTinbox->c = POINTERinbox->c;

		////printf("\nPrimer dato %.3f", POINTERinbox->a);
		////printf("\nSegundo dato %.3f", POINTERinbox->b);
		////printf("\nTercer dato %.3f", POINTERinbox->c);

		//printf("\nPrimer dato %.3f", OUTPUTinbox->a);
		//printf("\nSegundo dato %.3f", OUTPUTinbox->b);
		//printf("\nTercer dato %.3f", OUTPUTinbox->c);

		}
	  }

/*
      if(record)
         fwrite(ETH_RX_from_dyn, 200, 1,exout);
      return TRUE;
*/
      return TRUE;
   }
   else
   return FALSE;
}


/******************TEST DE RECEPTOR UDP********************/
