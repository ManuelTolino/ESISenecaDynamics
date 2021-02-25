//RC: read_config.cpp


//--------------------- 
//This file reads in seneca "offset" configuration data
//---------------------

//**************************
//   RCS Markers           *
//**************************


//$Header: C:/Master/seneca/RCS/read_config.cpp 1.3 2000/06/16 09:49:12 ColinJ Exp $
//$Log: read_config.cpp $
//Revision 1.3  2000/06/16 09:49:12  ColinJ
//corrected a bug which stopped data being read in.
//
//Revision 1.2  2000/05/31 10:38:48  colinj
//Added 'w' to say that stall warning should turn lamp on.
//
//Revision 1.1  2000/04/14 17:31:48  colinj
//Initial revision
//


//----------------------------------------------------------------------

//----------------
// Include Files
//----------------
#include <stdio.h>
#include <mbstring.h>
#include <string.h>
#include <ctype.h>

#include "config.h"
#include "define.h"

//----------------
// Defines
//----------------

//----------------
// Typedefs
//----------------

//--------------------
// External Routines
//--------------------

//----------------
// Externals
//----------------

//----------------
// Globals
//----------------

scale_struct RC_scale;

float RC_rudder_scale = 1.0f;
float RC_roll_scale = 1.0f;
float RC_pitch_scale = 1.0f;

int RC_alt_stall_warner = FALSE;
int RC_closedown = FALSE;

//-----------------
// Local Variables
//-----------------
static FILE *stream;

static float store ;

static int ct  = 0;
static int pos = 0;

static char line[30][100]; //30 arrays of 100 chars ; 
static char first;

// decode test strings
static char throt[]   = "*throt";
static char prop[]    = "*prop ";
static char mixture[] = "*mixtu";
static char oilt[]    = "*oilte";
static char oilp[]    = "*oilpr";
static char cyl[]     = "*cylht";
static char egt[]     = "*egtem";
static char fflow[]   = "*fflow";
static char fcont[]   = "*fcont";
static char suction[] = "*sucti";
static char ammeter[] = "*ammet";

//--------------------
// Forward References
//--------------------

void RC_decode(int side);

//------------------------------------------------------------------------------------------


void RC_read_config (void)
{

   if( (stream  = fopen( "config.dat", "r" )) !=NULL ) //if file is there
   {
      while( !feof( stream ) ) //untill end of file read in each line
      {		  
		   if( fgets( line[ct], 100, stream ) == NULL)
         {
			   printf( "\nfgets error\n" );
         }
         else
		   //only fills an array element if first char is a letter
		   if((line[ct][0] >= 'a') && (line[ct][0] <= 'z') || (line[ct][0] >= 'A') && (line[ct][0] <= 'Z'))
         {
			   first = line[ct][0];
            //check if first char is a valid one 
			   switch (first)
            {
            case 'l' :
			   case 'L' : // it is a left control change the l to a * and decode input
				  _strnset( line[ct], '*', 1 );
				  RC_decode(RC_LEFT);
				  break;

			  case 'r' :
			  case 'R' : // it is a right control change the r to a * and decode input
              pos = strcspn( line[ct], "1234567890" );
              if(sscanf(&line[ct][pos],"%f",&store) != EOF)
              {
                 RC_rudder_scale = store;
              }
				  break;

           case 's' :
           case 'S' : // suction 
              pos = strcspn( line[ct], "1234567890" );
              if(sscanf(&line[ct][pos],"%f",&store) != EOF)
              {
                 RC_scale.suction_scale = store;
              }
              break;

           case 'a' :
           case 'A' :  //ammeter
              pos = strcspn( line[ct], "1234567890" );
              if(sscanf(&line[ct][pos],"%f",&store) != EOF)
              {
                 RC_scale.ammeter_scale = store;
              }
              break;

           case 'w' :
           case 'W' : //set the heater overheat lamp to come on with stall warner
              {
                 RC_alt_stall_warner = TRUE;
              }
              break;

           case 'c':
           case 'C':
              RC_closedown = TRUE;
              break;

           case 'p' :
           case 'P' : 
              pos = strcspn( line[ct], "1234567890" );
              if(sscanf(&line[ct][pos],"%f",&store) != EOF)
              {
                 RC_pitch_scale = store;
              }
              break;

           case 'y':
           case 'Y':

              pos = strcspn( line[ct], "1234567890" );
              if(sscanf(&line[ct][pos],"%f",&store) != EOF)
              {
                 RC_roll_scale = store;
              }

			  default :
				  break;
            }
            ct++;
		  }

      }
   }
}

         
void RC_decode(int side)
{
	if((line[ct][1] == 't') || (line[ct][1] == 'T'))
	{
	   if(_mbsnicmp ((const unsigned char *)throt ,(const unsigned char *) line[ct],6 ) == 0 )
	   {
         //find first digit
         pos = strcspn( line[ct], "1234567890" );         
         if(sscanf(&line[ct][pos],"%f",&store) != EOF)
         {
            RC_scale.throt_zero[side] = store;
            //fill previous up with *
            _strnset( line[ct], '*', pos+1 );
         }
         //find first space
         pos = strcspn( line[ct], " " );
         _strnset( line[ct], '*', pos+1 ); //then fill
         pos = strcspn( line[ct], "1234567890" ); //find next number

         if(sscanf(&line[ct][pos],"%f",&store) != EOF)
         {
            RC_scale.throt_100[side] = store;
            _strnset( line[ct], '*', pos+1 );
         }
         
         pos = strcspn( line[ct], " " );
         _strnset( line[ct], '*', pos+1 );
         pos = strcspn( line[ct], "1234567890" );                  
         if(sscanf(&line[ct][pos],"%f",&store) != EOF)
         {
            RC_scale.throt_dead[side] = store;
         }

         store = (RC_scale.throt_100[side] - RC_scale.throt_zero[side]);
         if( store > 0)
         {
            RC_scale.throt_scale[side] = (100/store);
         }

	   }
	}
	else

   if((line[ct][1] == 'p') || (line[ct][1] == 'P'))
   {
   	if(_mbsnicmp ((const unsigned char *)prop ,(const unsigned char *) line[ct],6 ) == 0 )
      {
         //find first digit
         pos = strcspn( line[ct], "1234567890" );         
         if(sscanf(&line[ct][pos],"%f",&store) != EOF)
         {
            RC_scale.prop_zero[side] = store;
            //fill previous up with *
            _strnset( line[ct], '*', pos+1 );
         }
         //find first space
         pos = strcspn( line[ct], " " );
         _strnset( line[ct], '*', pos+1 ); //then fill
         pos = strcspn( line[ct], "1234567890" ); //find next number

         if(sscanf(&line[ct][pos],"%f",&store) != EOF)
         {
            RC_scale.prop_100[side] = store;
            _strnset( line[ct], '*', pos+1 );
         }
         
         pos = strcspn( line[ct], " " );
         _strnset( line[ct], '*', pos+1 );
         pos = strcspn( line[ct], "1234567890" );                  
         if(sscanf(&line[ct][pos],"%f",&store) != EOF)
         {
            RC_scale.prop_dead[side] = store;
         }

         store = (RC_scale.prop_100[side] - RC_scale.prop_zero[side]);
         if( store > 0)
         {
            RC_scale.prop_scale[side] = (100/store);
         }


      }
   }
	else

   if((line[ct][1] == 'm') || (line[ct][1] == 'M'))
   {
	   if(_mbsnicmp ((const unsigned char *)mixture ,(const unsigned char *) line[ct],6 ) == 0 )
      {
         //find first digit
         pos = strcspn( line[ct], "1234567890" );         
         if(sscanf(&line[ct][pos],"%f",&store) != EOF)
         {
            RC_scale.mix_zero[side] = store;
            //fill previous up with *
            _strnset( line[ct], '*', pos+1 );
         }
         //find first space
         pos = strcspn( line[ct], " " );
         _strnset( line[ct], '*', pos+1 ); //then fill
         pos = strcspn( line[ct], "1234567890" ); //find next number

         if(sscanf(&line[ct][pos],"%f",&store) != EOF)
         {
            RC_scale.mix_100[side] = store;
            _strnset( line[ct], '*', pos+1 );
         }
         
         pos = strcspn( line[ct], " " );
         _strnset( line[ct], '*', pos+1 );
         pos = strcspn( line[ct], "1234567890" );                  
         if(sscanf(&line[ct][pos],"%f",&store) != EOF)
         {
            RC_scale.mix_dead[side] = store;
         }
         store = (RC_scale.mix_100[side] - RC_scale.mix_zero[side]);
         if( store > 0)
         {
            RC_scale.mix_scale[side] = (100/store);
         }

         
      }
   }
	else

   if((line[ct][1] == 'o') || (line[ct][1] == 'O'))
   {
	   if(_mbsnicmp ((const unsigned char *)oilt ,(const unsigned char *) line[ct],6 ) == 0 )
      {
         pos = strcspn( line[ct], "1234567890" );         
         if(sscanf(&line[ct][pos],"%f",&store) != EOF)
         {
            RC_scale.oil_press_scale[side] = store;
         }
      }
	   else
	   if(_mbsnicmp ((const unsigned char *)oilp ,(const unsigned char *) line[ct],6 ) == 0 )
      {
         pos = strcspn( line[ct], "1234567890" );         
         if(sscanf(&line[ct][pos],"%f",&store) != EOF)
         {
            RC_scale.oil_press_scale[side] = store;
         }
      }
   }
	else

   if((line[ct][1] == 'c') || (line[ct][1] == 'C'))
   {
	   if(_mbsnicmp ((const unsigned char *)cyl ,(const unsigned char *) line[ct],6 ) == 0 )
      {
         pos = strcspn( line[ct], "1234567890" );         
         if(sscanf(&line[ct][pos],"%f",&store) != EOF)
         {
            RC_scale.cyl_scale[side] = store;
         }

      }
   }


	else

   if((line[ct][1] == 'e') || (line[ct][1] == 'E'))
   {
	   if(_mbsnicmp ((const unsigned char *)egt ,(const unsigned char *) line[ct],6 ) == 0 )
      {
         pos = strcspn( line[ct], "1234567890" );         
         if(sscanf(&line[ct][pos],"%f",&store) != EOF)
         {
            RC_scale.egt_scale[side] = store;
         }

      }
   }
	else

   if((line[ct][1] == 'f') || (line[ct][1] == 'F'))
   {
	   if(_mbsnicmp ((const unsigned char *)fflow ,(const unsigned char *) line[ct],6 ) == 0 )
      {
         pos = strcspn( line[ct], "1234567890" );         
         if(sscanf(&line[ct][pos],"%f",&store) != EOF)
         {
            RC_scale.fuel_flow_scale[side] = store;
         }
      
      }
	   else
	   if(_mbsnicmp ((const unsigned char *)fcont ,(const unsigned char *) line[ct],6 ) == 0 )
      {
         pos = strcspn( line[ct], "1234567890" );         
         if(sscanf(&line[ct][pos],"%f",&store) != EOF)
         {
            RC_scale.cyl_scale[side] = store;
         }

      }
   }

}
