//File: dynsnd.cpp

//Module: Seneca Dynamics

//Description: This file ...

//**************************
//   RCS Markers           *
//**************************


//$Header: C:/Master/seneca/sound/RCS/dynsnd.cpp 1.4 2000/06/16 11:22:17 ColinJ Exp $
//$Log: dynsnd.cpp $
//Revision 1.4  2000/06/16 11:22:17  ColinJ
//Allow air start or if sound turned off in flight to restart without rumble.
//
//Revision 1.3  2000/05/25 09:56:37  colinj
//Added gyro noise, left and right primers not separate,
//new gear clunks, changed strobe, changed deice.
//
//Revision 1.2  2000/05/11 09:12:05  colinj
//Sound updates.
//
//Revision 1.1  2000/04/17 09:43:14  colinj
//Initial revision
//

//----------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <windows.h>
#include <windowsx.h>
#include <mmsystem.h>
#include <d3dtypes.h>
#include <dsound.h>
//#include "stdafx.h"
#include "dirsnd.h"
#include "dynsnd.h"



//#define TEST



char            szAppName[]     = "Dynamics Sound";



/*************************************************************************
*
* FUNCTION: DirSndTimerProc()
*
* DESCRIPTION: This function is called by the Windows timer, which is set
*              for all DirSnd objects created as STREAMING buffers.
*              DirSnd objects created as STATIC buffers do NOT use
*              this function.
*
*              When this function is called, UpdateStreamBuffer() is
*              called for the DirSnd object whose m_TimerID matches the
*              Timer_ID passed to this function.
*
*************************************************************************/
void CALLBACK DirSndTimerProc(HWND hwnd, UINT Msg, UINT Timer_ID,
							  DWORD Time)
{
#ifdef STREAMING_BUFFER
  if (gStreamBuf1.m_TimerID == Timer_ID)
  {
	gStreamBuf1.UpdateStreamBuffer();
  }
#endif


#ifdef SOUND_3D_STREAMING_SEC_BUFFER
  // Uses IDirect3DSoundBuffer API.
  if (g3DSecondaryStreamBuf.m_TimerID == Timer_ID)
  {
	g3DSecondaryStreamBuf.UpdateStreamBuffer();

  }
#endif

  return;
}


/* Interface struct */

dynsound DYS_in;
static dynsound last; 


static HINSTANCE hInst;

static HWND hw;

static DirSnd prop_left;
static DirSnd prop_right;
static DirSnd windnoise;
static DirSnd eng_left;
static DirSnd eng_right;
static DirSnd ceng_left;
static DirSnd ceng_right;
static DirSnd deice;
//static DirSnd gear_up;
static DirSnd gear_down;
static DirSnd flap_up;
//static DirSnd flap_down;
static DirSnd crash;
static DirSnd skid;
static DirSnd Nclunk;
static DirSnd Rclunk;
static DirSnd Lclunk;
static DirSnd primer_left;
static DirSnd primer_right;
static DirSnd strobe;
static DirSnd autopilot;
static DirSnd manifold;
static DirSnd gyro;

static char right_eng = FALSE, left_eng = FALSE;

/*************************************************************************
*
* FUNCTION: WndProc()
*
*************************************************************************/
LRESULT CALLBACK WndProc(HWND hwnd, UINT Message, WPARAM wParam,
						 LPARAM lParam)
{
  //switch(Message)
 // {
    //HANDLE_MSG(hwnd, WM_CREATE,      _OnCreate);
    //HANDLE_MSG(hwnd, WM_DESTROY,     _OnDestroy);
    //HANDLE_MSG(hwnd, WM_COMMAND,     _OnCommand);
    //HANDLE_MSG(hwnd, WM_PAINT,       _OnPaint);
	//HANDLE_MSG(hwnd, WM_LBUTTONDOWN, _OnLButtonDown);
	//HANDLE_MSG(hwnd, WM_RBUTTONDOWN, _OnRButtonDown);

   // default:
      return(DefWindowProc(hwnd, Message, wParam, lParam));
 // }
}

// load a wave file (with optional loop)
static DirSnd * NewWave(DirSnd *ds, char *wavename, int start=0, int end=0)
{
   int err;
   char buf[512];

 	err = ds->Create(DSBCAPS_STATIC | DSBCAPS_CTRLFREQUENCY | DSBCAPS_CTRLVOLUME | DSBCAPS_CTRLPAN, DSSCL_NORMAL, hw, NULL, NULL, NULL, wavename, start, end);

   if (err)
   {
      sprintf(buf, "NewWave FailCode %d (%s)", err, wavename);
      ErrBox(buf);
   }

   return ds;
}

static void NewWave(DirSnd &ds, char *wavename, int start=0, int end=0)
{
   NewWave(&ds, wavename, start, end);
}

  static WNDCLASS WndClass;
static int CmdShow;

static void Frame(void)
{
   //char buf[512];
   //int err;
   static frames = 0;
   float speed;
   float engine; 
   DWORD pos;
      DWORD dummy;

      frames++;

      /* Auto pilot prefilght check : 3000Hz tone whenever the flag is set. */
      if ((DYS_in.flags & flag_autopilot) && (!(last.flags & flag_autopilot)))
         autopilot.Play(1);
     
      if ((!(DYS_in.flags & flag_autopilot)) && (last.flags & flag_autopilot))
         autopilot.Stop();

      /* Beacon strobe - zap the laser regularly while this is set */

      if ((DYS_in.flags & flag_strobe) && (!(last.flags & flag_strobe)))
      {
         strobe.m_pDSBuffer->SetVolume((unsigned long)strobe_volume);
         strobe.Play(1);
      }
     
      if ((!(DYS_in.flags & flag_strobe)) && (last.flags & flag_strobe))
         strobe.Stop();

      /* Gyro. When flag is set, gyro is running. If flag was previously unset,
               reset the sample to the beginning. If flag is unset and was
               previously set, reset the sample to the end of the loop. Otherwise loop it. */

      if ((DYS_in.flags & flag_gyro) && (!(last.flags & flag_gyro)))
      {
			gyro.Stop();
         gyro.m_pDSBuffer->SetCurrentPosition(0);
         gyro.m_pDSBuffer->SetVolume(gyro_volume);
         gyro.Play(0);
      }
     
      if ((!(DYS_in.flags & flag_gyro)) && (last.flags & flag_gyro))
         gyro.m_pDSBuffer->SetCurrentPosition(gyro_endloop);
/*
      if (DYS_in.flags & flag_gyro)
      {
         eng_left.m_pDSBuffer->GetCurrentPosition(&pos, &dummy);
         if (pos > gyro_endloop)
            eng_left.m_pDSBuffer->SetCurrentPosition(gyro_startloop);
      }
*/
      if ((DYS_in.flags & flag_gyro) && (last.flags & flag_gyro))
      {
         gyro.m_pDSBuffer->GetCurrentPosition(&pos, &dummy);
         if (pos > gyro_endloop)
            gyro.m_pDSBuffer->SetCurrentPosition(gyro_startloop);
      }

   /* Props - change frequency from speed */
   if (DYS_in.prop_left < prop_min)
   {
      prop_left.Stop(); // off
   }
   else
   {
      prop_left.Play(1);
      prop_left.m_pDSBuffer->SetFrequency((unsigned long)((DYS_in.prop_left) * prop_hz / prop_std));
   }

   if (DYS_in.prop_right < prop_min)
   {
      prop_right.Stop(); // off
   }
   else
   {
      prop_right.Play(1);
      prop_right.m_pDSBuffer->SetFrequency((unsigned long)((DYS_in.prop_right) * prop_hz / prop_std));
   }

   /* Engines, including ground starts */

   // If this is a ground start, replay the full sample.
   if ((DYS_in.flags & flag_start_left) && (!(last.flags & flag_start_left)))
   {
      eng_left.m_pDSBuffer->SetCurrentPosition(0);
      eng_left.Play(0);
   }

   if ((DYS_in.flags & flag_start_right) && (!(last.flags & flag_start_right)))
   {
      eng_right.m_pDSBuffer->SetCurrentPosition(0);
      eng_right.Play(0);
   }

   /* Engines go to std RPM at start, real RPM in loop, mixed between - to avoid 'engine start' not playing as RPM will be too low at start. */

   eng_left.m_pDSBuffer->GetCurrentPosition(&pos, &dummy);
   engine = (float)pos;
   //printf("pos %d, start %d\n", pos, eng_start); 
   if (engine > eng_start) engine = eng_start; // loop: full
   engine /= eng_start; // 0..1 - proportion of actual RPM
   engine = (engine * DYS_in.engine_left) + ((1-engine) * eng_std);


   if ((DYS_in.engine_left <= eng_min) || (DYS_in.engine_left >= eng_max) || (!(DYS_in.flags & flag_on_left)))
   {
      eng_left.Stop(); // off
      left_eng = FALSE;
   }
   else
   {
      if(left_eng == FALSE)
      {
         left_eng = TRUE;
         eng_left.m_pDSBuffer->SetCurrentPosition(eng_start);
      }
      eng_left.Play(1);
      eng_left.m_pDSBuffer->SetFrequency((unsigned long)(engine * eng_hz / eng_std));
      if (DYS_in.engine_left >= eng_high)
      {
         eng_left.m_pDSBuffer->SetVolume((long)(eng_fade-(6 * ((DYS_in.engine_left)-eng_high)))); // -25.00dB fade
      }
      else
      {
         eng_left.m_pDSBuffer->SetVolume((long)eng_fade);
      }
   }

   if ((DYS_in.engine_left > ceng_min) && (DYS_in.flags & flag_on_left))
   {
      ceng_left.Play(1);
      if (engine <= ceng_std)
      {
         ceng_left.m_pDSBuffer->SetFrequency((unsigned long)(engine * ceng_hz / ceng_std));
      }
      else
      {
         float ne = engine - ((engine-ceng_std)/2);
         ceng_left.m_pDSBuffer->SetFrequency((unsigned long)(ne * ceng_hz / ceng_std));
      }
      if (DYS_in.engine_left <= ceng_low)
      {
         ceng_left.m_pDSBuffer->SetVolume((long)(ceng_fade-(6 * (ceng_low - (DYS_in.engine_left)))));
      }
      else
      {
         ceng_left.m_pDSBuffer->SetVolume((long)((float)ceng_fade + ((float)-ceng_fade * (float)(((float)engine-ceng_min)/(float)ceng_max))));
      }
   }
   else
   {
      ceng_left.Stop();
   }

   if (DYS_in.flags & flag_restart_left)
   {
      if (pos >= eng_startloopend)
      {
         eng_left.m_pDSBuffer->SetCurrentPosition(eng_startloop);
      }
   }

   if((!(DYS_in.flags & flag_restart_left)) && (last.flags & flag_restart_left))
   {
      eng_left.m_pDSBuffer->SetCurrentPosition(eng_startrumble);
   }

	if((DYS_in.flags & flag_on_left) && !(last.flags & flag_on_left) && DYS_in.engine_left > 600)
	{
		eng_left.Play(1);
		eng_left.m_pDSBuffer->SetCurrentPosition(eng_startloopend);
	}

   eng_right.m_pDSBuffer->GetCurrentPosition(&pos, &dummy);
   engine = (float)pos;
   if (engine > eng_start) engine = eng_start; // loop: full
   engine /= eng_start; // 0..1 - proportion of actual RPM
   engine = (engine * DYS_in.engine_right) + ((1-engine) * eng_std);

   engine *= (float)1.02; // avoid modulation

   if ((DYS_in.engine_right < eng_min) || (DYS_in.engine_right >= eng_max) || (!(DYS_in.flags & flag_on_right)))
   {
      eng_right.Stop(); // off
      right_eng = FALSE;
   }
   else
   {
      if(right_eng == FALSE)
      {
         right_eng = TRUE;
         eng_right.m_pDSBuffer->SetCurrentPosition(eng_start);
      }
      eng_right.Play(1);
      eng_right.m_pDSBuffer->SetFrequency((unsigned long)(engine * eng_hz / eng_std));
      if (DYS_in.engine_right >= eng_high)
      {
         eng_right.m_pDSBuffer->SetVolume((long)(eng_fade-(6 * ((DYS_in.engine_right)-eng_high))));
   
	  }
      else
      {
         eng_right.m_pDSBuffer->SetVolume((long)(eng_fade));
      }
   }

   if ((DYS_in.engine_right > ceng_min) && (DYS_in.flags & flag_on_right))
   {
      ceng_right.Play(1);
      if (engine <= ceng_std)
      {
         ceng_right.m_pDSBuffer->SetFrequency((unsigned long)(engine * ceng_hz / ceng_std));
      }
      else
      {
         float ne = engine - ((engine-ceng_std)/2);
         ceng_right.m_pDSBuffer->SetFrequency((unsigned long)(ne * ceng_hz / ceng_std));
      }
      if (DYS_in.engine_right <= ceng_low)
      {
         ceng_right.m_pDSBuffer->SetVolume((long)(ceng_fade-(6 * (ceng_low - (DYS_in.engine_right)))));
      }
      else
      {
         ceng_right.m_pDSBuffer->SetVolume((long)((float)ceng_fade + ((float)-ceng_fade * (float)(((float)engine-ceng_min)/(float)ceng_max))));
      }
   }
   else
   {
      ceng_right.Stop();
   }

   if (DYS_in.flags & flag_restart_right)
   {
      if (pos >= eng_startloopend)
      {
         eng_right.m_pDSBuffer->SetCurrentPosition(eng_startloop);
      }
   }

   if((!(DYS_in.flags & flag_restart_right)) && (last.flags & flag_restart_right))
   {
      eng_right.m_pDSBuffer->SetCurrentPosition(eng_startrumble);
   }


	if((DYS_in.flags & flag_on_right) && !(last.flags & flag_on_right) && DYS_in.engine_right > 600)
	{
		eng_right.Play(1);
		eng_right.m_pDSBuffer->SetCurrentPosition(eng_startloopend);
	}

   //last_right_restart = (DYS_in.flags & flag_restart_right);
   /* Wind noise: linearly proportional (but given in dB, so exp) to airspeed. */ 
   if (DYS_in.airspeed > airspeed_max) speed = airspeed_max;
   else speed = DYS_in.airspeed;
   windnoise.m_pDSBuffer->SetVolume(((unsigned long)(speed * airspeed_range / airspeed_max))-(unsigned long)airspeed_range);
   
   /* Gear up/down : make it momentary */
   /*if ((DYS_in.flags & flag_gearup) && (!(last.flags & flag_gearup)))
   {
      gear_up.m_pDSBuffer->SetCurrentPosition(0);
      gear_up.Play(0);
   }*/

   if ((DYS_in.flags & flag_geardown) && (!(last.flags & flag_geardown)))
   {
  //    gear_down.m_pDSBuffer->SetCurrentPosition(0);
      gear_down.Play(1);
   }

   /* Flaps : also momentary */

   if ((DYS_in.flags & flag_flapup) && (!(last.flags & flag_flapup)))
   {
//      flap_up.m_pDSBuffer->SetCurrentPosition(0);
      flap_up.Play(1);
   }

   /* Gear off */
   if ((DYS_in.flags & flag_gearoff) && (!(last.flags & flag_gearoff)))
   {
      gear_down.Stop();
   }

   /* Flaps off */
if ((DYS_in.flags & flag_flapoff) && (!(last.flags & flag_flapoff)))
   {
      flap_up.Stop();
   }

   /* Primer pump: looped */
primer_left.m_pDSBuffer->SetVolume(primer_volume);
   if ((DYS_in.flags & flag_primer_left))
   {
//      flap_up.m_pDSBuffer->SetCurrentPosition(0);
primer_left.m_pDSBuffer->GetCurrentPosition(&pos, &dummy);
  if (pos > primer_endloop) 
  {
     primer_left.m_pDSBuffer->SetCurrentPosition(primer_startloop);
     
  }
      primer_left.Play(0);

   }
  else
   {
     if (last.flags & flag_primer_left) primer_left.m_pDSBuffer->SetCurrentPosition(primer_endloop);
   }

primer_right.m_pDSBuffer->SetVolume(primer_volume);
   /* Primer pump: looped */
   if ((DYS_in.flags & flag_primer_right))
   {
//      flap_up.m_pDSBuffer->SetCurrentPosition(0);
primer_right.m_pDSBuffer->GetCurrentPosition(&pos, &dummy);
  if (pos > primer_endloop) 
  {
     primer_right.m_pDSBuffer->SetCurrentPosition(primer_startloop);
     
  }
      primer_right.Play(0);

   }
  else
   {
     if (last.flags & flag_primer_right) primer_right.m_pDSBuffer->SetCurrentPosition(primer_endloop);
   }


   /* Manifold : looped */
   if ((DYS_in.flags & flag_manifold))
   {
//      flap_up.m_pDSBuffer->SetCurrentPosition(0);
manifold.m_pDSBuffer->GetCurrentPosition(&pos, &dummy);
  if (pos > manifold_endloop) 
  {
     manifold.m_pDSBuffer->SetCurrentPosition(manifold_startloop);
  }
      manifold.Play(0);

   }
  else
   {
     if (last.flags & flag_manifold) manifold.m_pDSBuffer->SetCurrentPosition(manifold_endloop);
   }

   /*if ((DYS_in.flags & flag_flapdown) && (!(last.flags & flag_flapdown)))
   {
      flap_down.m_pDSBuffer->SetCurrentPosition(0);
      flap_down.Play(0);
   }*/

   /* Crashing : momentary */

   if ((DYS_in.flags & flag_crash) && (!(last.flags & flag_crash)))
   {
      //crash.m_pDSBuffer->SetCurrentPosition(0);
      crash.Play(0);
   }

   /* Gear Clunk Noise : momentary (?) */
   
   if ((DYS_in.flags & flag_clunk) && (!(last.flags & flag_clunk)))
   {
    //  elt.m_pDSBuffer->SetCurrentPosition(0);
      Nclunk.Play(0);
   }

   if ((DYS_in.flags & flag_Lclunk) && (!(last.flags & flag_Lclunk)))
   {
    //  elt.m_pDSBuffer->SetCurrentPosition(0);
      Lclunk.Play(0);
   }

   if ((DYS_in.flags & flag_Rclunk) && (!(last.flags & flag_Rclunk)))
   {
    //  elt.m_pDSBuffer->SetCurrentPosition(0);
      Rclunk.Play(0);
   }

   /* Wheel touchdown : momentary */

   if ((DYS_in.flags & flag_skid) && (!(last.flags & flag_skid)))
   {
    //  skid.m_pDSBuffer->SetCurrentPosition(0);
      skid.Play(0);
   }

   /* Deice : momentary, but when already running returns to start + a bit.*/
   
   if ((DYS_in.flags & flag_deice) && (!(last.flags & flag_deice)))
   {
     
      deice.m_pDSBuffer->GetCurrentPosition(&pos, &dummy);

      if ((pos > deice_end) || (pos < deice_start)) // debounce
      {
         pos = 0;
      }
      else
      {
         pos = deice_start;
      }

      deice.m_pDSBuffer->SetCurrentPosition(pos);
      deice.m_pDSBuffer->SetVolume(deice_volume);
      deice.Play(0);
   }

   if(DYS_in.flags & flag_deice_off)
   {
      deice.m_pDSBuffer->SetCurrentPosition(end_deice);
   }


   
   /* Looping buffer events */
   eng_left.Frame();
   eng_right.Frame();



         
   last = DYS_in;
   
}

static void DYSExit(void)
{
   prop_left.Destroy();
//   prop_right.Destroy();
   windnoise.Destroy();
   eng_left.Destroy();
//   eng_right.Destroy();
   ceng_left.Destroy();
//ceng_right.Destroy();
   deice.Destroy();
//   gear_up.Destroy();
   gear_down.Destroy();
   flap_up.Destroy();
  // flap_down.Destroy();
   crash.Destroy();
   skid.Destroy();
   Nclunk.Destroy();
   Rclunk.Destroy();
   primer_left.Destroy();
   strobe.Destroy();
   gyro.Destroy();
}



static void Init(void)
{
   int err;
   char buf[512];
   /* Create and display window needed to play sounds */
  /***** REGISTER THE CLASS *********************************/
  //if (!hPrevInstance)
  //{
    // Register main window.
    WndClass.style         = CS_HREDRAW | CS_VREDRAW;
    WndClass.lpfnWndProc   = WndProc;
    WndClass.cbClsExtra    = 0;
    WndClass.cbWndExtra    = 0;
    WndClass.hInstance     = hInst;
    WndClass.hIcon         = LoadIcon(hInst, IDI_APPLICATION);
    WndClass.hCursor       = LoadCursor(NULL, IDC_ARROW);
    WndClass.hbrBackground = (HBRUSH) (COLOR_WINDOW + 1);
    WndClass.lpszMenuName  = NULL;
    WndClass.lpszClassName = szAppName;

    if (RegisterClass(&WndClass) == 0)
    {
      Box(NULL, "FAILURE", "RegisterClass() FAILS!");
    }
  //}

   hw = CreateWindow(szAppName, szAppName,
                      WS_OVERLAPPEDWINDOW,
                      CW_USEDEFAULT, CW_USEDEFAULT,
                      CW_USEDEFAULT, CW_USEDEFAULT,
                      NULL, NULL, hInst, NULL);

  if (hw == NULL)
  {
    ErrBox("FAILURE! CreateWindow() FAILS!");
  }

  ShowWindow(hw, CmdShow);
  UpdateWindow(hw);

  /* Init DS */

  if (err = DirSnd::Init(hw))
  {
     sprintf(buf, "Unable to initialise DirectSound, error %d", err);
     WarnBox(buf);
  }

   /* Init DSB */

  NewWave(&prop_left, "C:\\wav\\Prop.wav");
 // NewWave(prop_right, "C:\\wav\\Prop.wav");
  prop_left.Duplicate(prop_right);
  //prop_right = NewWave(new DirSnd, "C:\\wav\\Prop.wav"); // XXX dup!
  NewWave(&windnoise, "C:\\wav\\60hznoise.wav");
  //NewWave(&windnoise,"C:\\wav\\Jet.wav");
  NewWave(&deice, "C:\\wav\\deice-1.wav");
  NewWave(&eng_left, "C:\\wav\\start.wav", eng_start, eng_end); // new
  eng_left.Duplicate(eng_right);
  //NewWave(&ceng_left, "C:\\wav\\seneca\\2600-s4-l150-200-h-5k-7k.wav");
  NewWave(&ceng_left, "C:\\wav\\2600-mono.wav"); 
  //NewWave(&ceng_right, "C:\\wav\\seneca\\2600-right.wav");
  ceng_left.Duplicate(ceng_right);
  //NewWave(&gear_up, "C:\\wav\\Gear.wav");
  NewWave(&gear_down, "C:\\wav\\Gear.wav"); //new
  NewWave(&flap_up, "C:\\wav\\newflap.wav");
  //NewWave(&flap_down, "C:\\wav\\Flaps.wav");  //new
  NewWave(&crash, "C:\\wav\\Crash.wav");
  NewWave(&skid, "C:\\wav\\Wheels.wav"); // new
  //NewWave(&clunk, "C:\\wav\\Clunk.wav"); // new
  NewWave(&Nclunk, "C:\\wav\\nosegearclunk.wav");//"C:\\wav\\cmtouch6.wav"); // new
  NewWave(&Rclunk, "C:\\wav\\cmtouch6.wav"); // new
  Rclunk.Duplicate(Lclunk);

   NewWave(&primer_left, "C:\\wav\\newprimer.wav"); //new
   primer_left.Duplicate(primer_right); //new
   NewWave(&strobe, "C:\\wav\\strobe.wav"); // new
  NewWave(&autopilot, "C:\\wav\\3kHz-square.wav"); // new
  NewWave(&manifold, "C:\\wav\\manifold.wav"); // new
  NewWave(&gyro, "C:\\wav\\gyro.wav"); // new


   
  /* Set stereo positions */
  prop_left.m_pDSBuffer->SetPan(DSBPAN_LEFT); //MODIFICADO POR MI ANTES DSBPAN_LEFT Y DSBPAN_RIGHT
  prop_right.m_pDSBuffer->SetPan(DSBPAN_LEFT);
  primer_left.m_pDSBuffer->SetPan(DSBPAN_LEFT);
  primer_right.m_pDSBuffer->SetPan(DSBPAN_LEFT);
  eng_left.m_pDSBuffer->SetPan(DSBPAN_LEFT);
  eng_right.m_pDSBuffer->SetPan(DSBPAN_LEFT);
  ceng_left.m_pDSBuffer->SetPan(DSBPAN_LEFT);
  ceng_right.m_pDSBuffer->SetPan(DSBPAN_LEFT);
  Rclunk.m_pDSBuffer->SetPan(DSBPAN_LEFT);
  Lclunk.m_pDSBuffer->SetPan(DSBPAN_LEFT);

  /* Init always looping sounds */

  windnoise.Play(1);

   /* Init control */
   /* set up struct */

   last.prop_left = last.prop_right = 0;
   last.engine_left = last.engine_right = 0;
   last.airspeed = 0;
   last.flags = 0;
   
   DYS_in.prop_left = DYS_in.prop_right = 0;
   DYS_in.engine_left = DYS_in.engine_right = 0;
   DYS_in.airspeed = 0;
   DYS_in.flags = 0;

}


// 0=frame update, 1=init, -1=exit
void DYS_dynsnd(int code)
{
   //return;
   switch(code)
   {
   case 0:
      {
         Frame();
         break;
      }
   case 1:
      {
         Init();
         break;
      }
   case -1:
      {
         DYSExit();
         break;
      }
   default:
      {
         ErrBox("Bad DYS_dynsnd code");
      }
   }
}


#ifdef TEST


int APIENTRY WinMain(HINSTANCE hInstance,
                     HINSTANCE hPrevInstance,
                     LPSTR     lpCmdLine,
                     int       nCmdShow)
{
   MSG Msg;
   int frames = 0;

      hInst = hInstance;
      CmdShow = nCmdShow;


      Init();

      
   DYS_in.prop_left = DYS_in.prop_right = 0;
   DYS_in.engine_left = DYS_in.engine_right = 0;
   DYS_in.airspeed = 0;
   DYS_in.flags = 0;

   //DYS_in.prop_right = 0;
      
  // DYS_in.flags |= flag_deice;

   //DYSExit();
   //exit(0);

  while (1)
  {
     Frame();
     Sleep(10);
     
     //printf("Frames %d\n", frames);
     
     frames++;
     if (frames == 5)
     {
//        DYS_in.flags |= flag_deice;
        //frames--;
     }

     if (frames == 10)
     {
        DYS_in.flags |= flag_flapup;
        //DYS_in.flags |= flag_deice;
      
        //DYS_in.engine_left = 1200;
     }
     if ((frames == 1010))
     {
        DYS_in.flags = 0;
     }
     if ((frames > 20) && (frames < 600))
     {
        ////DYS_in.flags |= flag_manifold;
     }
     else
//DYS_in.flags &= ~flag_manifold;

     if (frames == 1000)
     {
        //DYS_in.flags |= flag_start_right;
        //DYS_in.engine_right = 1200;
     }
     if ((frames >= 1500) && (frames < 1500+2800-1200))
     {
        DYS_in.engine_left+=1;
     }
     if ((frames >= 2400) && (frames < 2400+2800-1200))
     {
        //DYS_in.engine_right+=1;
     }
     if (frames > 2400+2800+400-1200)
     {
        if (DYS_in.engine_left > 1200) DYS_in.engine_left--;
        if (DYS_in.engine_right > 1200) DYS_in.engine_right--;
     }
      //DYS_in.flags |= flag_restart_right;
      //DYS_in.flags |= flag_on_left + flag_on_right;
    /* if (frames == 5)
        DYS_in.flags |= flag_primer;
     else
        DYS_in.flags &= ~flag_primer;
*/
      //DYS_in.flags |= flag_strobe;
      //if (frames & 128) DYS_flags ^= flag_autopilot;
      //if (frames == 108) DYS_in.flags |= flag_manifold;
     DYS_in.airspeed = 30;
    //((DYS_in.engine_left + DYS_in.engine_right)-2400)/20;

      DYS_in.flags |= flag_on_left;
      DYS_in.flags |= flag_on_right;

    if (PeekMessage(&Msg, hw, 0, 0, PM_REMOVE))
    {

       TranslateMessage(&Msg);
       DispatchMessage(&Msg);
    }
  }

  DYSExit();
  exit(0);


	return 0;
}

#endif


