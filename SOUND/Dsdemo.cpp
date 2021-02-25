/*************************************************************************
*
* PROGRAM: DSDEMO.CPP
*
* DESCRIPTION: Direct Sound demonstration program for Windows 95/NT.
*
* AUTHOR: Tom Bouril, Creative Labs, Inc.  (February 1997)
*
* NOTE Make certain to read README.TXT for details about the DirSnd C++
*      class.
*
* TO COMPILE: Put all files into a single directory, insert RESOURCE.RC
*             into your project along with this file (DSDEMO.CPP).
*             Link with the Microsoft libraries WINMM.LIB and DSOUND.LIB.
*             You will also need the Microsoft Direct Sound file
*             DSOUND.H in your "include" path.  Comment out or leave
*             #defined the #defines at the top of this source code to
*             determine which type of Direct Sound buffers are used.
*
* DANGER!:    Search for all ".wav" strings in this file and replace
*             them with file/path names that match your own wave files.
*
* DESCRIPTION: This program demonstrates how to use the DirSnd C++
*              class defined in the dirsnd.cpp file.  The DirSnd
*              C++ class does NOT support primary buffers--the DirSnd
*              C++ class supports only SECONDARY buffers--both STATIC
*              and STREAMING secondary buffers.  A 3D listener PRIMARY
*              BUFFER, which supports the Direct Sound 3D listener
*              functions, is supported by the 3DListener C++ class in
*              the DS3DLSTR.CPP file.  (This program DOES show how to
*              use the 3DListener C++ class.)
*
*              The DirSnd C++ class supports the following Direct
*              Sound API functons:
*
*              1) Functions belonging to the IDirectSound class.
*              2) Functions belonging to the IDirectSoundBuffer class.
*              3) Functions belonging to the IDirect3DSoundBuffer class.
*
*              Functions belonging to the IDirect3DListener Direct Sound
*              class are supported by the 3DListener C++ class located
*              in DS3DLSTR.CPP.
*
*
* HOW TO USE THE DirSnd C++ CLASS:
*
*     1) Declare some GLOBAL DirSnd classes--one global DirSnd class
*        for each wave file you want to play.
*
*     2) Before the Direct Sound API can be called, a GUID must be
*        obtained.  A GUID identifies which Direct Sound device
*        (audio card) will be used.  The GUID is then passed to
*        DirSnd::Create(), which creates a DirSnd object.  To use
*        the default Direct Sound device, use a GUID of NULL.
*
*        To obtain the GUID of a specific Direct Sound device (sound
*        card), call DirectSoundEnumerate().  DirectSoundEnumerate()
*        invokes the callback function DSEnumProc(), where the
*        available GUIDs are enumerated.  Alter DSEnumProc() to obtain
*        the GUID for the desired sound device.  See DSEnumProc() in
*        dsenum.cpp for details.
*
*        DirectSoundEnumerate() is called in the _OnCreate() function
*        below.
*
*     3) Call the DirSnd::Create() member function to initialize the
*        DirSnd object.  For details, see the _OnCreate() function below
*        and read README.TXT.
*
*     4) Call the Direct Sound API functions via the appropriate DirSnd
*        class member pointer:
*
*        * IDirectSound functions are called via the
*          m_pDS member pointer.
*
*        * IDirectSoundBuffer functions are called via the
*          m_pDSBuffer member pointer.
*
*        * IDirectSoundBuffer functions are called via the
*          m_pDS3DBuffer member pointer.
*
*        IDirect3DListener Direct Sound functions are called via
*        the m_p3DListener, which is part of the Listener3D C++
*        class located in DS3DLSTR.CPP.
*        
*
*        NOTE: The Microsoft Direct Sound IDirectSoundBuffer functions
*              listed below should NOT be used with the DirSnd class!!!
*              Instead, use the corresponding DirSnd member functions
*              intheir place.
*
*        DO NOT CALL THESE FUNCTIONS.         | INSTEAD, CALL...
*        ----------------------------------------------------------
*        IDirectSound::DuplicateSoundBuffer() | DirSnd::Duplicate()
*        ----------------------------------------------------------
*        IDirectSoundBuffer::Play()           | DirSnd::Play()
*        ----------------------------------------------------------
*        IDirectSoundBuffer::Stop()           | DirSnd::Stop()
*        ----------------------------------------------------------
*
*     5) For each STREAMING buffer created, you must #define a UNIQUE
*        Windows Timer ID.  This Timer ID is then passed as one the
*        parameters to the DirSnd::Create() function.
*
*     6) For each STREAMING buffer created, the DirSndTimerProc()
*        callback procedure must call DirSnd::UpdateStreamBuffer()
*        when the DirSnd's member variable m_TimerID matches the
*        Timer_ID passed to DirSndTimerProc().  See DirSndTimerProc()
*        below for details.
*
*     7) Every DirSnd class that calls DirSnd::Create() MUST call
*        DirSnd::Destroy() before the program terminates--this
*        releases any resources allocated.  See the _OnDestroy()
*        function below for details.
*
*************************************************************************/
/**************************************************
*  To test the following types of buffers,
*  #define the one(s) you want to test, comment
*  out the ones you do NOT want to test.
*  NOTE: Files played as STATIC buffers should
*  be fairly short--no more than a few seconds.
**************************************************/
#define STATIC_BUFFER
//#define STATIC_DUPLICATE_BUFFER
//#define STREAMING_BUFFER
//#define SOUND_3D_STATIC_SEC_BUFFER
//#define SOUND_3D_STREAMING_SEC_BUFFER


/***** #defines and #includes ***************/
,
		_OnRButtonDown(HWND, BOOL, int, int, UINT),
        _OnPaint(HWND),
        Alter3DListenerSettings(void),
        Alter3DSecondaryBufSettings(DirSnd *);


/***** GLOBAL VARIABLES *****************************/
static char       szAppName[] = "Direct Sound Demo";
static HINSTANCE  ghInstance;

GUID       *gpGuid = NULL;
DirSnd      gStaticBuf1,#define STRICT
#define WIN32_LEAN_AND_MEAN
#define TIMERID_1 1  // #define one timer ID for every streaming buffer.
#define TIMERID_2 2


#include <windows.h>
#include <windowsx.h>
#include "resource.h"
#include "dirsnd.cpp"
#include "ds3dlstr.cpp"

/***** STRUCTURES ***********************************/
struct DrvrInfo
{
  GUID     *pGuid;
  char      Descrip[100];
  DrvrInfo *Next;
};

#include "dsenum.cpp"


/***** FUNCTION PROTOTYPES **************************/
BOOL CALLBACK DialogProc(HWND, UINT, WPARAM, LPARAM);
LRESULT CALLBACK WndProc(HWND, UINT, WPARAM, LPARAM);
BOOL    _OnCreate(HWND, CREATESTRUCT FAR*);
void    _OnDestroy(HWND),
        _OnCommand(HWND, int, HWND, UINT),
		_OnLButtonDown(HWND, BOOL, int, int, UINT)
            gStreamBuf1,
		    g3DSecondaryStaticBuf,
			g3DSecondaryStreamBuf,
           *gpStaticBufDuplicate;
Listener3D  g3DListener;  // NEVER more than one Listener3D object!

DrvrInfo *gpDrivers     = NULL,
         *gpDriversHead = NULL;

/*************************************************************************
*
* FUNCTION: WinMain()
*
*************************************************************************/
int WINAPI WinMain(HINSTANCE hInst, HINSTANCE hPrevInstance,
                   LPSTR lpszCmdParam, int nCmdShow)
{
  HWND     hwnd;
  MSG      Msg;
  WNDCLASS WndClass;

  /***** REGISTER THE CLASS *********************************/
  if (!hPrevInstance)
  {
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
      return(FALSE);
    }
  }

  /***** CREATE THE WINDOW ************************************/
  ghInstance = hInst;
  hwnd = (szAppName, szAppName,
                      WS_OVERLAPPEDWINDOW,
                      CW_USEDEFAULT, CW_USEDEFAULT,
                      CW_USEDEFAULT, CW_USEDEFAULT,
                      NULL, NULL, hInst, NULL);

  if (hwnd == NULL)
  {
    Box(NULL, "FAILURE!", "CreateWindow() FAILS!");
    return(FALSE);
  }

  ShowWindow(hwnd, nCmdShow);
  UpdateWindow(hwnd);

  while (GetMessage(&Msg, NULL, 0, 0))
  {
    TranslateMessage(&Msg);
    DispatchMessage(&Msg);
  }

  return(Msg.wParam);
}


/*************************************************************************
*
* FUNCTION: WndProc()
*
*************************************************************************/
LRESULT CALLBACK WndProc(HWND hwnd, UINT Message, WPARAM wParam,
						 LPARAM lParam)
{
  switch(Message)
  {
    HANDLE_MSG(hwnd, WM_CREATE,      _OnCreate);
    HANDLE_MSG(hwnd, WM_DESTROY,     _OnDestroy);
    HANDLE_MSG(hwnd, WM_COMMAND,     _OnCommand);
    HANDLE_MSG(hwnd, WM_PAINT,       _OnPaint);
	HANDLE_MSG(hwnd, WM_LBUTTONDOWN, _OnLButtonDown);
	HANDLE_MSG(hwnd, WM_RBUTTONDOWN, _OnRButtonDown);

    default:
      return(DefWindowProc(hwnd, Message, wParam, lParam));
  }
}


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


/*************************************************************************
*
* FUNCTION: _OnCreate()
*
* DESCRIPTION: Initialize DirSnd object(s) and begin playing the
*              DirSnd object's associated wave file.  The #defines
*              at the top of this file determine which type of
*              buffers are created.
*
*************************************************************************/
BOOL _OnCreate(HWND hwnd, CREATESTRUCT FAR* lpCreateStruct)
{
  // Initialize gpGuid by calling DirectSoundEnumerate().
  // DirectSoundEnumerate() calls the DSEumProc() callback function.
  // You must alter DSEnumProc() so the desired GUID (sound card) is
  // selected.
  DirectSoundEnumerate((LPDSENUMCALLBACK) DSEnumProc, (LPVOID) NULL);

  // Pop up a dialog box to select Direct Sound driver.
  DialogBox(ghInstance,  MAKEINTRESOURCE(IDD_DIALOG1), hwnd, DialogProc);
  

#ifdef STATIC_DUPLICATE_BUFFER
  // NOTE: A static buffer must be created before a duplicate static
  //       buffer can be created.  #define STATIC_BUFFER here to
  //       ensure that a static buffer gets created.
  #define STATIC_BUFFER
#endif

/******************************/

#ifdef STATIC_BUFFER
  if (!gStaticBuf1.Create(DSBCAPS_CTRLDEFAULT | DSBCAPS_STATIC,
	                      DSSCL_NORMAL, hwnd, gpGuid, NULL, NULL,
						  "d:\\wave\\5.WAV"))
  {
    gStaticBuf1.m_pDSBuffer->SetVolume(0);
    gStaticBuf1.Play(1); // 1 = Play with looping, 0 = play once.
  }
  else
  {
    Box(NULL, "ERROR", "DirSnd::Create() FAILS");
  }
#endif

/******************************/

#ifdef STATIC_DUPLICATE_BUFFER
  gpStaticBufDuplicate = gStaticBuf1.Duplicate();
  if (gpStaticBufDuplicate == NULL)
  {
    Box(NULL, "ERROR", "DirSnd::Duplicate() FAILS");
  }
  else
  {
	gpStaticBufDuplicate->m_pDSBuffer->SetVolume(0);
	gpStaticBufDuplicate->Play(1); // 1 = Play with looping, 0 = play once.
  }
#endif

/******************************/

#ifdef STREAMING_BUFFER
  if (!gStreamBuf1.Create(DSBCAPS_CTRLDEFAULT, DSSCL_NORMAL, hwnd, gpGuid,
	                      NULL, TIMERID_1, "d:\\wave\\8.WAV"))
  {
    gStreamBuf1.m_pDSBuffer->SetVolume(0);
    gStreamBuf1.Play(1); // 1 = Play with looping, 0 = play once.
  }
  else
  {
    Box(NULL, "ERROR", "DirSnd::Create() FAILS");
  }
//PrintPCMData(gStreamBuf1.m_DSBufDesc.lpwfxFormat,
//               gStreamBuf1.m_DSBufDesc.dwBufferBytes);
#endif


/******************************/

#ifdef SOUND_3D_STREAMING_SEC_BUFFER
  // Create STREAMING SECONDARY 3D BUFFER.
  if (!g3DSecondaryStreamBuf.Create(DSBCAPS_CTRL3D, DSSCL_NORMAL, hwnd,
	                                gpGuid, NULL, TIMERID_2,
									"e:\\22k_wav\\acoustic\\04_acous.wav"))
  {
    g3DSecondaryStreamBuf.m_pDSBuffer->SetVolume(0);
    g3DSecondaryStreamBuf.Play(1); // 1 = Play with looping, 0 = play once.
  }
  else
  {
    Box(NULL, "ERROR", "DirSnd::Create() FAILS");
  }

  #define PRIMARY_3D_BUFFER  // Create ONLY ONE PRIMARY 3D BUFFER below!
#endif

/******************************/

#ifdef SOUND_3D_STATIC_SEC_BUFFER
  // Create STATIC SECONDARY 3D BUFFER.
  if (!g3DSecondaryStaticBuf.Create(DSBCAPS_CTRL3D | DSBCAPS_STATIC,
	                                DSSCL_NORMAL, hwnd, gpGuid, NULL,
							        TIMERID_2,
									"e:\\22k_wav\\classic\\07_class.wav"))
  {
    g3DSecondaryStaticBuf.m_pDSBuffer->SetVolume(0);
    g3DSecondaryStaticBuf.Play(1); // 1 = Play with looping, 0 = play once.
  }
  else
  {
    Box(NULL, "ERROR", "DirSnd::Create() FAILS");
  }

  #define PRIMARY_3D_BUFFER  // Create ONLY ONE PRIMARY 3D BUFFER below!
#endif


#ifdef PRIMARY_3D_BUFFER
  // CREATE PRIMARY 3D BUFFER
  if (g3DListener.Create(gpGuid, hwnd, DSSCL_EXCLUSIVE) != DS_OK)
  {
    Box(NULL, "ERROR", "Listener3D::Create() FAILS");
  }
#endif

  return(TRUE);
}


/*************************************************************************
*
* FUNCTION: _OnLButtonDown()
*
* DESCRIPTION: Each time the LEFT mouse button is pressed, the state
*              of a DirSnd object toggles between two modes.
*
*************************************************************************/
void _OnLButtonDown(HWND hwnd, BOOL DblClick, int x, int y, UINT KeyFlags)
{
  DWORD BufStatus;

#ifdef STATIC_BUFFER
  gStaticBuf1.m_pDSBuffer->GetStatus(&BufStatus);
  if (BufStatus & DSBSTATUS_PLAYING)
  {
    gStaticBuf1.Stop();
  }
  else
  {
	gStaticBuf1.Play(1); // 1 = Play with looping, 0 = play once.
  }

#endif

/******************************/

#ifdef STATIC_BUFFER_DUPLICATE
  gpStaticBufDuplicate->m_pDSBuffer->GetStatus(&BufStatus);
  if (BufStatus & DSBSTATUS_PLAYING)
  {
    gpStaticBufDuplicate->Stop();
  }
  else
  {
    gpStaticBufDuplicate->Play(1); // 1 = Play with looping, 0 = play once.
  }
#endif

/******************************/

#ifdef STREAMING_BUFFER
  gStreamBuf1.m_pDSBuffer->GetStatus(&BufStatus);
  if (BufStatus & DSBSTATUS_PLAYING)
  {
    gStreamBuf1.Stop();
  }
  else
  {
	gStreamBuf1.Play(1); // 1 = Play with looping, 0 = play once.
  }
#endif


/******************************/

#ifdef SOUND_3D_STREAMING_SEC_BUFFER
  g3DSecondaryStreamBuf.m_pDSBuffer->GetStatus(&BufStatus);
  
  if (BufStatus & DSBSTATUS_PLAYING)
  {
	g3DSecondaryStreamBuf.Stop();
  }
  else
  {
	g3DSecondaryStreamBuf.Play(1);
  }
  Alter3DListenerSettings();
  Alter3DSecondaryBufSettings(&g3DSecondaryStreamBuf);
#endif

/******************************/

#ifdef SOUND_3D_STATIC_SEC_BUFFER
  g3DSecondaryStaticBuf.m_pDSBuffer->GetStatus(&BufStatus);
  
  if (BufStatus & DSBSTATUS_PLAYING)
  {
	g3DSecondaryStaticBuf.Stop();
  }
  else
  {
	g3DSecondaryStaticBuf.Play(1);
  }
  Alter3DListenerSettings();
  Alter3DSecondaryBufSettings(&g3DSecondaryStaticBuf);
#endif

  return;
}


/*************************************************************************
*
* FUNCTION: _OnRButtonDown()
*
* DESCRIPTION: Make certain that you #define STATIC_BUFFER and
*              #define STREAMING_BUFFER at the top of this source
*              code.
*
*              Each time the LEFT mouse button is pressed, the
*              playback frequency (speed) of both buffers will
*              be altered.
*
*************************************************************************/
void _OnRButtonDown(HWND hwnd, BOOL DblClick, int x, int y, UINT KeyFlags)
{
  static int Toggle = TRUE;

  if (Toggle)
  {
    #ifdef STATIC_BUFFER
      gStaticBuf1.m_pDSBuffer->SetFrequency(10000);
    #endif

    #ifdef STREAMING_BUFFER
      gStreamBuf1.m_pDSBuffer->SetFrequency(40000);
    #endif

    Toggle = FALSE;
  }
  else
  {
    #ifdef STATIC_BUFFER
      gStaticBuf1.m_pDSBuffer->SetFrequency(40000);
    #endif

    #ifdef STREAMING_BUFFER
      gStreamBuf1.m_pDSBuffer->SetFrequency(10000);
    #endif

    Toggle = TRUE;
  }

  return;
}


/*************************************************************************
*
* FUNCTION: _OnDestroy()
*
* DESCRIPTION: Each DirSnd object created MUST be destroyed before
*              the program terminates!  Even if the DirSnd object
*              failed in DirSnd::Create, call its Destroy() function
*              so it can perform any necessary clean-up.
*
*************************************************************************/
void _OnDestroy(HWND hwnd)
{

#ifdef STATIC_BUFFER
  gStaticBuf1.Destroy();
#endif


#ifdef STATIC_BUFFER_DUPLICATE
  if (gpStaticBufDuplicate)
  {
	  // NOTE: DirSnd objects created via DirSnd::Duplicate()
	  //       are destroyed using the C++ keyword "delete,"
	  //       they DO NOT call DirSnd::Destroy()!!!
	delete(gpStaticBufDuplicate);
  }
#endif


#ifdef STREAMING_BUFFER
  gStreamBuf1.Destroy();
#endif


#ifdef SOUND_3D_STREAMING_SEC_BUFFER
  g3DSecondaryStreamBuf.Destroy();
  if (g3DListener.Destroy())
  {
    Box(NULL, "FAILURE!", "Listener3D::Destroy() FAILS!");
  }
#endif


#ifdef SOUND_3D_STATIC_SEC_BUFFER
  g3DSecondaryStaticBuf.Destroy();

#ifndef SOUND_3D_STREAMING_SEC_BUFFER
  // If streaming buffer existed, g3DListener already destroyed.
  if (g3DListener.Destroy())
  {
    Box(NULL, "FAILURE!", "Listener3D::Destroy() FAILS!");
  }
#endif

#endif

  // Free space allocated for GUID struct in DialogProc().
  if (gpGuid)
  {
	delete(gpGuid);
  }

  PostQuitMessage(0);
  return;
}


/*************************************************************************
*
* FUNCTION: _OnCommand()
*
*************************************************************************/
void _OnCommand(HWND hwnd, int id, HWND hwndCtl, UINT CodeNotify)
{
  return;
}



/*************************************************************************
*
* FUNCTION: _OnPaint()
*
*************************************************************************/
void _OnPaint(HWND hwnd)
{
  char        String[100];
  PAINTSTRUCT Ps;
  RECT        Rect;
  HDC         PaintDC = BeginPaint(hwnd, &Ps);

  GetClientRect(hwnd, &Rect);

  strcpy(String, "Press LEFT mouse button to toggle playing state.");
  TextOut(PaintDC, 0, 0, String, strlen(String));

  strcpy(String, "Press RIGHT mouse button to alter playing frequency.");
  TextOut(PaintDC, 0, 20, String, strlen(String));

  EndPaint(hwnd, &Ps);

  return;
}


/*************************************************************************
*
* FUNCTION: DialogProc()
*
* DESCRIPTION: Initialize the Combo Box contained in this Dialog Box
*              with all the Direct Sound driver descriptions found in
*              DSEnumProc().
*
*              When the user presses the OK button or closes the dialog
*              box, the driver GUID is initialized to the value that
*              corresponds to the Combo Box's selected driver description.
*
*************************************************************************/
BOOL CALLBACK DialogProc(HWND hDlg, UINT uMsg, WPARAM wP, LPARAM lP)
{
  DrvrInfo *Temp = gpDriversHead;  // Point to head of linked list.
  HWND      hComboBox;
  char      String[100];
  int       Index = 0;

  switch(uMsg)
  {
    case WM_INITDIALOG:
	  hComboBox = GetDlgItem(hDlg, IDC_COMBO);
	  while (Temp)
	  {
		// Traverse through linked list and add Driver Description
		// strings to the Combo Box.
        SendMessage(hComboBox, CB_ADDSTRING, 0, (LPARAM) Temp->Descrip);
		Temp = Temp->Next;
		Index++;  // Track how many entries added to Combo Box.
	  }

	  // Loop through entries in Combo Box looking for a string match.
	  while (Index > 0)
	  {
		// If string in Combo Box matches first entry in linked list,
		// select that string.
	    SendMessage(hComboBox, CB_SELECTSTRING, Index,
			        (LPARAM) gpDriversHead->Descrip);
	    Index--;
      }

	return(TRUE);  // End: WM_INITDIALOG


	case WM_COMMAND:
	  switch(LOWORD(wP))
	  {
	    case IDOK:      // OK button pressed.
		case IDCANCEL:  // Dialog Box closed.
		  // Get index of selected item.
		  hComboBox = GetDlgItem(hDlg, IDC_COMBO);
		  Index = SendMessage(hComboBox, CB_GETCURSEL, 0, 0);

		  // Get string of selected item.
		  SendMessage(hComboBox, CB_GETLBTEXT, Index, (LPARAM) String);

		  while (Temp)
		  {
		    if (!strcmp(Temp->Descrip, String))
			{
			  // String in combo box matches string in linked list.
			  gpGuid = Temp->pGuid;
			  if (gpGuid)
			  {
				// If GUID is not default device (gpGuid not NULL),
				// allocate space for GUID struct.
			    gpGuid = new(GUID);
			  }
			  break;  // Exit while() loop.
			}

			Temp = Temp->Next;  // Traverse to next link.
		  }

		  EndDialog(hDlg, 0);  // Kill Dialog Box.

		  // Deallocate linked list.
		  if (gpDriversHead)
          {
			DrvrInfo *Temp;

	        gpDrivers = gpDriversHead;  // Point to 1st entry in list.
	        while (gpDrivers)
	        {
              Temp = gpDrivers->Next;
              delete(gpDrivers);
	          gpDrivers = Temp;
            }
          }

	    return(TRUE);  // End: IDOK, IDCANCEL
	  }

	break;  // End: WM_COMMAND
  }

  return(FALSE);
}



/*************************************************************************
*
* FUNCTION: Alter3DListenerSettings()
*
* DESCRIPTION: Alter 3D sound of the PRIMARY buffer by calling
*              the Direct Sound IDirect3DListener API,
*
*************************************************************************/
void Alter3DListenerSettings()
{
  static D3DVALUE DopFact = (D3DVALUE) 0.0;
  static D3DVALUE      x1 = (D3DVALUE) 0.0;
  static D3DVALUE      y1 = (D3DVALUE) 0.0;
  static D3DVALUE      z1 = (D3DVALUE) 0.0;

  // SET DISTANCE FACTOR
  if (g3DListener.m_p3DListener->SetDistanceFactor((D3DVALUE) 2.0,
	                                               DS3D_DEFERRED) != DS_OK)
  {
    Box(NULL, "ERROR", "SetDistanceFactor() FAILS!");
  }

  // SET DOPPLER FACTOR
  if (g3DListener.m_p3DListener->SetDopplerFactor(DopFact,
	                                              DS3D_DEFERRED) != DS_OK)
  {
    Box(NULL, "ERROR", "SetDopplerFactor() FAILS!");
  }

  DopFact += (D3DVALUE) 1.0;
  if (DopFact > (D3DVALUE) 10.0)
  {
	DopFact = (D3DVALUE) 1.0;
  }

  // SET LISTENER ORIENTATION
  if (g3DListener.m_p3DListener->SetOrientation((D3DVALUE) 0.0,
	  (D3DVALUE) 0.0, (D3DVALUE) 1.0, (D3DVALUE) 0.0, (D3DVALUE) 1.0,
	  (D3DVALUE) 0.0, DS3D_DEFERRED) != DS_OK)
  {
    Box(NULL, "ERROR", "SetOrientation() FAILS!");
  }


  // SET LISTENER POSITION
  if (g3DListener.m_p3DListener->SetPosition(x1, y1, z1, DS3D_DEFERRED)
	  != DS_OK)
  {
    Box(NULL, "ERROR", "SetPosition() FAILS!");
  }
  x1 += (D3DVALUE) 5.0;
  y1 += (D3DVALUE) 5.0;
  z1 += (D3DVALUE) 5.0;


  // COMMIT DEFERRED SETTINGS.
  if (g3DListener.m_p3DListener->CommitDeferredSettings() != DS_OK)
  {
    Box(NULL, "ERROR", "CommitDeferedSettings() #1 FAILS!");
  }

  return;
}


/*************************************************************************
*
* FUNCTION: DirSnd::Alter3DSecondaryBufSettings()
*
* DESCRIPTION: Alter 3D sound of the SECONDARY buffer by calling
*              the Direct Sound IDirect3DSoundBuffer API,
*
*************************************************************************/
void Alter3DSecondaryBufSettings(DirSnd *pSecBuf3D)
{
  D3DVALUE x1 = (D3DVALUE) 0.0;
  D3DVALUE y1 = (D3DVALUE) 0.0;
  D3DVALUE z1 = (D3DVALUE) 2.0;

  if (pSecBuf3D->m_pDS3DBuffer->SetConeAngles(90, 180, DS3D_DEFERRED)
	                            != DS_OK)
  {
	Box(NULL, "ERROR!", "SetConeAngles() FAILS!");
  }

  if (pSecBuf3D->m_pDS3DBuffer->SetConeOrientation(x1, y1, z1,
	                                               DS3D_DEFERRED) != DS_OK)
  {
	Box(NULL, "ERROR!", "SetConeOrientation() FAILS!");
  }


  if (pSecBuf3D->m_pDS3DBuffer->SetConeOutsideVolume(0L, DS3D_DEFERRED)
	                            != DS_OK)
  {
	Box(NULL, "ERROR!", "SetConeOutsideVolume() FAILS!");
  }


  if (pSecBuf3D->m_pDS3DBuffer->SetMaxDistance((D3DVALUE) 1000.0,
	  DS3D_DEFERRED) != DS_OK)
  {
	Box(NULL, "ERROR!", "SetMaxDistance() FAILS!");
  }


  if (pSecBuf3D->m_pDS3DBuffer->SetMinDistance((D3DVALUE) 2.0,
	                                           DS3D_DEFERRED) != DS_OK)
  {
	Box(NULL, "ERROR!", "SetMinDistance() FAILS!");
  }


  if (pSecBuf3D->m_pDS3DBuffer->SetMode(DS3DMODE_NORMAL, DS3D_DEFERRED)
	                            != DS_OK)
  {
	Box(NULL, "ERROR!", "SetMode() FAILS!");
  }


  x1 = (D3DVALUE)  23.0;
  y1 = (D3DVALUE) -23.0;
  z1 = (D3DVALUE)  46.6;
  if (pSecBuf3D->m_pDS3DBuffer->SetPosition(x1, y1, z1, DS3D_DEFERRED)
	                            != DS_OK)
  {
	Box(NULL, "ERROR!", "SetPosition() FAILS!");
  }


  if (pSecBuf3D->m_pDS3DBuffer->SetVelocity(x1, y1, z1, DS3D_DEFERRED)
	                            != DS_OK)
  {
	Box(NULL, "ERROR!", "SetVelocity() FAILS!");
  }


  if (pSecBuf3D->m_pDS3DBuffer->SetPosition(x1, y1, z1, DS3D_DEFERRED)
	                            != DS_OK)
  {
	Box(NULL, "ERROR!", "SetPosition() FAILS!");
  }

  // COMMIT DEFERRED SETTINGS VIA LISTENER PRIMARY BUFFER.
  if (g3DListener.m_p3DListener->CommitDeferredSettings() != DS_OK)
  {
    Box(NULL, "ERROR!", "CommitDeferedSettings() #2 FAILS!");
  }

  return;
}