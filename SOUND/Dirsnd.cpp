#include "dirsnd.h"
#include <stdio.h>
#include <stdlib.h>


static    LPDIRECTSOUND          g_pDS;
static   DSBUFFERDESC            g_DSBufDesc;
static    LPDIRECTSOUNDBUFFER    g_pDSBuffer;
static	LPDIRECTSOUND3DBUFFER   g_pDS3DBuffer;

/*************************************************************************
*
* FUNCTION: DirSnd::Play()
*
* DESCRIPTION: Begin playing the file.
*
* PARAMETERS: If the parameter passed is FALSE (zero), the file will
*             be played only ONE TIME.
*
*             If the parameter passed is TRUE (non-zero), the file
*             will be played in a continuous loop.
*
*************************************************************************/
void DirSnd::Play(BOOL Looping_Status)
{
  DWORD BytesToLock = 0,
	    CurrentPlayCursor,
		CurrentWriteCursor;
  UINT  TimerLength;
  LONG  Volume;

  // m_LoopingStatus exists so other class functions can
  // determiine if the file is looping or not.
  m_LoopingStatus = Looping_Status;

  m_pDSBuffer->GetVolume(&Volume);  // Save volume setting.
  m_pDSBuffer->SetVolume(-10000);   // Set volume to minimum.

  if (m_DSBufDesc.dwFlags & DSBCAPS_STATIC)
  {
	// Buffer is a STATIC buffer.
	if (m_LoopingStatus)
	{
	  m_pDSBuffer->Play(0, 0, DSBPLAY_LOOPING);
	}
	else
	{
	  m_pDSBuffer->Play(0, 0, 0);
	}
  }
  else
  {
	// Buffer is a STREAMING buffer.
	if (m_WaitForEnd)
	{
	  // Buffer is NOT in looping mode AND the end of file has been
	  // loaded into buffer when Stop() was called.  Calculate
	  // TimerLength so the timer will be triggered precisely after
	  // the file is done playing.
	  m_pDSBuffer->GetCurrentPosition(&CurrentPlayCursor,
		                              &CurrentWriteCursor);
      if (CurrentPlayCursor > m_PreviousPlayCursor)
      {
        BytesToLock = (CurrentPlayCursor - m_PreviousPlayCursor);
      }
      else if (CurrentPlayCursor < m_PreviousPlayCursor)
      {
	    BytesToLock = m_DSBufDesc.dwBufferBytes -
		              (m_PreviousPlayCursor - CurrentPlayCursor);
      }

	  m_TotalBytesPlayed += BytesToLock;
      TimerLength = (DWORD) (float(m_FileInfo.DataChunkSize -
		                     m_TotalBytesPlayed) /
						     float(m_DSBufDesc.lpwfxFormat->nAvgBytesPerSec)
						     * 1000.0);
      if (TimerLength < 121)
	  {
 	    // File is < 121 mSec from done playing.  Terminate playing
	    // and reload the buffer so it's ready to play again.
	    InitialBufferLoad();
		m_pDSBuffer->SetVolume(Volume);   // Restore volume setting.
		return;
	  }
	}
	else
	{
      // Set TimerLength so timer triggers after 1/2 a buffer is played.
      TimerLength = (UINT)  (((float(m_DSBufDesc.dwBufferBytes) /
		                       float(m_DSBufDesc.lpwfxFormat->nAvgBytesPerSec))
		                     * 1000.0) / 2.0);
	}

	SetTimer(m_hwnd, m_TimerID, TimerLength, DirSndTimerProc);
	m_pDSBuffer->Play(0, 0, DSBPLAY_LOOPING);
  }

  m_pDSBuffer->SetVolume(Volume);   // Restore volume setting.

  return;
}

/* Set up primary buffer with format */
int DirSnd::Init(HWND hwnd)
{
WAVEFORMATEX fmt = {WAVE_FORMAT_PCM, 2, 44100, 44100*2*2, 4, 16, 0};

if (DirectSoundCreate(NULL, &g_pDS, NULL) == DS_OK)
  {
    /* SET COOP LEVEL */
    if (g_pDS->SetCooperativeLevel(hwnd, DSSCL_PRIORITY) != DS_OK)
    {
	  return(2);
    }
  }
  else
  {
	return(1);
  }

  g_DSBufDesc.dwSize = sizeof(DSBUFFERDESC);
  g_DSBufDesc.dwFlags = DSBCAPS_PRIMARYBUFFER;
  g_DSBufDesc.dwBufferBytes = 0;
  g_DSBufDesc.lpwfxFormat = NULL; // 1ary: set format later
  
     /* Create primary Direct Sound Buffer. */
  if ((g_pDS->CreateSoundBuffer(&g_DSBufDesc, &g_pDSBuffer, NULL)) != DS_OK)
  {
    // sprintf(buf, "CreateSoundBuffer failed, code %d (%x)\nBuffer %d,%d,%d", err, err, m_DSBufDesc.dwSize, m_DSBufDesc.dwFlags, m_DSBufDesc.dwBufferBytes);
     //ErrBox(buf);
	return(4);
  }


  /* Set format */
  if ((g_pDSBuffer->SetFormat(&fmt)) != DS_OK) return 5;

   // OK
  return 0;

}





/*************************************************************************
*
* FUNCTION: DS_Create()
*
* DESCRIPTION: Creates a Direct Sound object and SECONDARY buffer based
*              upon the parameters passed to this function.  The buffer
*              is then filled with WAVE data from the WAVE file.  Do
*              NOT use this function to create a PRIMARY buffer!
*
*              To create a PRIMARY buffer, #include the file
*              "ds3dlstr.cpp", declare a single (only one)
*              "Listener3D" object, and call the Listener3D::Create()
*              method.
*
* DANGER:      The dwFlags parameter passed to this function MUST NOT
*              include the DSBCAPS_PRIMARYBUFFER flag!  Setting the
*              dwFlags parameter to DSBCAPS_PRIMARYBUFFER will cause
*              problems!
*
* RETURN VALUE: 0 = NO ERRORS.
*               1 = DirectSoundCreate() fails.
*               2 = SetCooperativeLevel() fails.
*               3 = OpenWaveFile() fails.
*               4 = CreateSoundBuffer() fails.
*               5 = QueryInterface() fails, m_pDS3DBuffer not created.
*
*************************************************************************/
int DirSnd::Create(DWORD dwFlags, DWORD CoopLevel, HWND hwnd,
				   GUID *PtrGuid, UINT SizeOfBuf, UINT Timer_ID,
				   char *NameOfFile, int start, int end)
{
  /* INITIALIZE VARIABLES. */
  m_DSBufDesc.dwFlags  = dwFlags;
  //DwCoopLevel        = CoopLevel;
  m_hwnd               = hwnd;
  m_pDS3DBuffer        = 0;
  strcpy(m_Filename, NameOfFile);
  //PGuid              = PtrGuid;
  //PreviousPlayCursor = 0;
  //DS_TimerID            = Timer_ID;
  //TotalBytesPlayed   = 0;
  //WaitForEnd         = FALSE;
  //strcpy(Filename, NameOfFile);
  WAVEFORMATEX fmt = {WAVE_FORMAT_PCM, 2, 44100, 44100*2*2, 4, 16, 0};
  char buf[512];
int err;


  //sprintf(buf,"Using PGUID %p\n",PtrGuid);
  //WarnBox(buf);
  
  /* CREATE DIRECT SOUND OBJECT. */
  if (DirectSoundCreate(NULL, &m_pDS, NULL) == DS_OK)
  {
    /* SET COOP LEVEL */
    if (m_pDS->SetCooperativeLevel(hwnd, CoopLevel) != DS_OK)
    {
	  return(2);
    }
  }
  else
  {
	return(1);
  }

  //WarnBox("Created Direct Sound object");

  if (err = OpenWaveFile())
  {
     sprintf(buf, "Unable to open WAV file: error code %d", err);
     ErrBox(buf);
	/* OpenWaveFile FAILS! */
	//fprintf(stderr,"Direct Sound Failure\nOpenWaveFile()  Fails\n\n");
	//exit(2);
    return(3);
  }

  //WarnBox("Opened WAV file");
  
  if (!(m_DSBufDesc.dwFlags & DSBCAPS_STATIC))
  {
	/* Buffer is STREAMING buffer.
	 Determine proper size of the STREAMING buffer.  The Buffer
	 size--whether static or streaming--is stored in dwBufferBytes.
    
     NOTE: dwBufferBytes is first initialized in OpenWaveFile(),
	       where it is initialized to the size of the file's
	       data chunk.
		   */
    if (SizeOfBuf == 0)
    {
	  /* By default, make buffer 2 seconds long. */
      m_DSBufDesc.dwBufferBytes = 2 * m_DSBufDesc.lpwfxFormat->nAvgBytesPerSec;
    }
    else
    {
	  /* Buffer size is requested by caller. */
	  m_DSBufDesc.dwBufferBytes = SizeOfBuf;
    }

    if (m_DSBufDesc.dwBufferBytes > m_FileInfo.DataChunkSize)
    {
      /* The buffer is larger than the data chunk to be played.
	     So set dwBufferBytes to size of data chunk. */
      m_DSBufDesc.dwBufferBytes = m_FileInfo.DataChunkSize;
    }
  }


  /* Create Direct Sound Buffer. */
  if ((err = m_pDS->CreateSoundBuffer(&m_DSBufDesc, &m_pDSBuffer, NULL)) != DS_OK)
  {
     sprintf(buf, "CreateSoundBuffer failed, code %d (%x)\nBuffer %d,%d,%d", err, err, m_DSBufDesc.dwSize, m_DSBufDesc.dwFlags, m_DSBufDesc.dwBufferBytes);
     ErrBox(buf);
	return(4);
  }

//WarnBox("Created DS buffer");

#ifdef undef
  if (dwFlags & DSBCAPS_CTRL3D)
  {
	/* Initialize m_pDS3DBuffer so IDirect3DSoundBuffer
	   API functions can be called via m_pDS3DBuffer. */
	DS_PDSBuffer->lpVtbl->QueryInterface(DS_PDSBuffer, IID_IDirectSound3DBuffer, (LPVOID *) &DS_PDS3DBuffer);
	if (DS_PDS3DBuffer == NULL)
    {
	  return(5);
    }
  }
#endif

  /* Load the entire buffer with data. */
  InitialBufferLoad();

  //WarnBox("Loaded buffer");

  /* NOTE: If buffer is a streaming buffer, leave the WAVE file
           open--it will be closed in the Destroy() function. */
  if (m_DSBufDesc.dwFlags & DSBCAPS_STATIC)
  {
	// Buffer is static.  All wave data has been loaded into
	// the buffer, so close the wave file.
	CloseWaveFile();
  }

  //WarnBox("Closed WAV file");

   
  /* Set up looping (by notification) */
  /*if ((start > 0) || (end > 0))
  {
     HRESULT hr;
     DSBPOSITIONNOTIFY posn;
     SECURITY_ATTRIBUTES sec_attr = {sizeof(SECURITY_ATTRIBUTES), NULL, FALSE};

     // notify event
     if ((m_posnEvent = CreateEvent(&sec_attr, FALSE, FALSE, "loop_notify_event")) == NULL) return 6;

     // DS notify interface
     //hr = (m_pDSBuffer->QueryInterface(IID_IDirectSoundNotify, (void **)(&m_pDirectSoundNotify) ) );
     hr = (m_pDSBuffer->QueryInterface(IID_IDirectSoundNotify, (void **)(&m_pDirectSoundNotify) ) );
     if (hr != S_OK)
     {
        char buf[256];
        sprintf(buf, "%s: QueryInterface returned %d (%x)",NameOfFile, hr, hr);
        WarnBox(buf);
        return 7;
     }
     if (!(m_pDirectSoundNotify)) return 9;

     // set event trigger
     posn.dwOffset = end;
     posn.hEventNotify = m_posnEvent;
     if (!m_pDirectSoundNotify->SetNotificationPositions(1, &posn)) return 8;

     // set return position
     m_loop = start;

  }*/
  m_loop = start; m_loopend = end;
  // move forward to nearest here? Without sample-precision useless.
  

  return(0);
}



void DirSnd::Frame(void)
{
   DWORD play;
   DWORD nowt;
   HRESULT hr;

   if ((m_loopend > 0) || (m_loop > 0))
   {
   // End of loop?
   hr = m_pDSBuffer->GetCurrentPosition(&play, &nowt);
   if (hr != DS_OK)
   {
      char buf[256];
      sprintf(buf, "GetCurrentPosition failed, code 0x%x", hr);
      WarnBox(buf);
   }

   if ((int)play >= m_loopend)
   {
     hr = m_pDSBuffer->SetCurrentPosition(m_loop);
 if (hr != DS_OK)
   {
      char buf[256];
      sprintf(buf, "SetCurrentPosition failed, code 0x%x", hr);
      WarnBox(buf);
   }
   }
   
   }
}



/*************************************************************************
*
* FUNCTION: DirSnd::Destroy()
*
* DESCRIPTION: This function is the complimentary function to
*              DirSnd::Create().  For each DirSnd object created,
*              DirSnd::Destroy() MUST be called before the program
*              terminates to perform cleanup operations.
*
*              Which cleanup operations need to be perfomed depends
*              upon the value of m_CreateErrorCode, which was assigned
*              a value in DirSnd::Create().
*
*************************************************************************/
void DirSnd::Destroy()
{
  if (!(m_DSBufDesc.dwFlags & DSBCAPS_STATIC))
  {
	// Buffer is a STREAMING buffer.
	if (m_CreateErrorCode == 0)
	{
	  KillTimer(m_hwnd, m_TimerID);
	}

	if (m_CreateErrorCode == 0 || m_CreateErrorCode > 3)
	{
	  // Close wave file ONLY for streaming buffers.
	  CloseWaveFile();
	}
  }

  if (m_CreateErrorCode == 0 || m_CreateErrorCode > 3)
  {
	// Space for m_DSBufDesc.lpwfxFormat is allocated in OpenWaveFile().
	delete(m_DSBufDesc.lpwfxFormat);  // Deallocate WAVEFORMATEX space.
  }


  // RELEASE DIRECT SOUND BUFFER(S) AND DIRECT SOUND OBJECT.
  if (m_pDS3DBuffer)
  {
    // Release the IDirect3DSoundBuffer, if it was created.
    m_pDS3DBuffer->Release();
  }

  if (m_CreateErrorCode == 0 || m_CreateErrorCode > 4)
  {
	// Release the IDirectSoundBuffer.
    m_pDSBuffer->Release();
  }
	
  if (m_CreateErrorCode != 1)
  {
	// Release the Direct Sound object.
    m_pDS->Release();
  }

  return;
}


/*************************************************************************
*
* FUNCTION: DirSnd::Duplicate()
*
* DESCRIPTION: This function will create a DirSnd object that uses the
*              same Direct Sound object and STATIC buffer as the object
*              that calls this function.
*
*              NOTE: Duplicate() will NOT work for streaming buffers
*                    because the contents of streaming buffers
*                    continually change, making it impossible for
*                    a duplicate buffer to play the end of a long
*                    file while the original buffer plays the beginning
*                    of that file.
*
* RETURN VALUE: If successful, this function returns a pointer to
*               a newly allocated and initialized DirSnd object.
*               If this function fails, it returns NULL.
*
*************************************************************************/
void DirSnd::Duplicate(DirSnd &out)
{
  //DirSnd *pDirSnd;

  if (!(m_DSBufDesc.dwFlags & DSBCAPS_STATIC))
  {
	// Buffer is NOT static.
	// Duplicate() works ONLY for STATIC buffers.
	return;
  }

  // Allocate a new DirSnd object.
  //pDirSnd = new(DirSnd);

  //if (pDirSnd == NULL)
  //{
	//return(NULL);
  //}

  // Copy class variables from this object into the new DirSnd object.
  memcpy((void *)&out, this, sizeof(DirSnd));

  // Create the duplicate Direct Sound buffer.
  if (m_pDS->DuplicateSoundBuffer(m_pDSBuffer, &(out.m_pDSBuffer))
	  == DS_OK)
  {
    return;
  }
  
  // IDirectSound::DuplicateSoundBuffer() FAILED.
  //delete(pDirSnd);

  ErrBox("Unable to duplicate buffer");

  return;
}


/*************************************************************************
*
* FUNCTION: DirSnd::Stop()
*
* DESCRIPTION: Stops the file from playing.
*
*************************************************************************/
void DirSnd::Stop()
{
  if (!(m_DSBufDesc.dwFlags & DSBCAPS_STATIC))
  {
	// Buffer is STREAMING buffer.
    KillTimer(m_hwnd, m_TimerID);
  }
  
  m_pDSBuffer->Stop();
  return;
}


/*************************************************************************
*
* FUNCTION: DirSnd::InitialBufferLoad()
*
* DESCRIPTION: This function loads the entire buffer with data from the
*              beginning of the file.  This function must be called
*              before a file can be played.
*
*              This function is called under 2 circumstances:
*
*              1) This function is called whenever a buffer is
*                 created (when a DirSnd class is declared),
*                 whether the buffer is STATIC or STREAMING.
*
*              2) This function is called whenever a STREAMING buffer
*                 is used AND the entire file finishes playing AND the
*                 m_LoopingStatus flag is TRUE.  This ensures that
*                 the STREAMING buffer is ready to be played again from
*                 the start of the file the next time Play() is called.
*
*************************************************************************/
void DirSnd::InitialBufferLoad()
{
  DWORD  Length1,
         Length2;
  LPVOID pData1,
         pData2;
  LONG   Volume;

  m_pDSBuffer->GetVolume(&Volume);  // Save volume setting.
  m_pDSBuffer->SetVolume(-10000);   // Set volume to minimum.
  m_pDSBuffer->Stop();
  m_pDSBuffer->SetCurrentPosition(0);
  m_pDSBuffer->SetVolume(Volume);   // Restore volume setting.
  ResetFile();

  if (!(m_DSBufDesc.dwFlags & DSBCAPS_STATIC))
  {
	// Buffer is a STREAMING buffer.  Reset some class variables.
	KillTimer(m_hwnd, m_TimerID);
    m_PreviousPlayCursor = 0;
    m_TotalBytesPlayed   = 0;
    m_WaitForEnd         = FALSE;
  }

  // LOCK THE DIRECT SOUND BUFFER BEFORE LOADING IT WITH DATA.
  if (m_pDSBuffer->Lock(0, m_DSBufDesc.dwBufferBytes, &pData1, &Length1,
		                &pData2, &Length2, NULL) != DS_OK)
  {
    Box(NULL, "FAILURE", "Lock() FAILS");
  }

  // Load Direct Sound buffer with WAVE data from file.
  if (mmioRead(m_hMmioFile, (char *) pData1, m_DSBufDesc.dwBufferBytes) == -1)
  {
    Box(NULL, "FAILURE", "mmioRead() FAILS.");
  }


  // UNLOCK DIRECT SOUND BUFFER IMMEDIATELY AFTER WRITING TO IT.
  if (m_pDSBuffer->Unlock(pData1, Length1, pData2, Length2) != DS_OK)
  {
    Box(NULL, "FAILURE", "Lock() FAILS");
  }

  m_FileInfo.BytesToEndOfFile = m_FileInfo.DataChunkSize -
	                            m_DSBufDesc.dwBufferBytes;

  return;
}


/*************************************************************************
*
* FUNCTION: DirSnd::UpdateStreamBuffer()
*
* DESCRIPTION: This function is called ONLY for STREAMING buffers!
*
*              NEVER call this function directly!  This function is
*              called ONLY by DirSndTimerProc(), which gets called
*              automatically when the Windows timer is triggered.
*
*              Whenever the Windows timer is triggered via a DirSnd
*              object, this function will be called and data will be
*              loaded from the file into the buffer.  For details,
*              see commented code below.
*
*************************************************************************/
void DirSnd::UpdateStreamBuffer()
{
  char  *pData1,
        *pData2;		 
  DWORD  BytesToLock,
	     BytesToRead,
		 CurrentPlayCursor,
		 CurrentWriteCursor,
	     Length1,
         Length2;

  if (m_WaitForEnd)
  {
    // The ENTIRE non-looping file has JUST FINISHED PLAYING!
	// Terminate playing and reload the buffer so it's ready to
	// play again.
    InitialBufferLoad();
    return;
  }


  // Determine how many buffer bytes are available for writing.
  m_pDSBuffer->GetCurrentPosition(&CurrentPlayCursor, &CurrentWriteCursor);

  if (CurrentPlayCursor > m_PreviousPlayCursor)
  {
    BytesToLock = (CurrentPlayCursor - m_PreviousPlayCursor);
  }
  else if (CurrentPlayCursor < m_PreviousPlayCursor)
  {
	BytesToLock = m_DSBufDesc.dwBufferBytes -
		          (m_PreviousPlayCursor - CurrentPlayCursor);
  }
  else
  {
	// CurrentPlayCursor == m_PreviousPlayCursor
	return;
  }


  // LOCK THE DIRECT SOUND BUFFER BEFORE LOADING IT WITH DATA.
  if (m_pDSBuffer->Lock(m_PreviousPlayCursor, BytesToLock, (void **)(&pData1), &Length1,
	                    (void **)&pData2, &Length2, 0) != DS_OK)
  {
    Box(NULL, "FAILURE", "Lock() FAILS");
  }

  // Update m_PreviousPlayCursor
  m_PreviousPlayCursor = CurrentPlayCursor;


  /***** Update pData1/Length1 part of buffer. **************************/
  /**********************************************************************/
  if (Length1 <= m_FileInfo.BytesToEndOfFile)
  {
    BytesToRead = Length1;
    m_FileInfo.BytesToEndOfFile -= Length1;
  }
  else
  {
    BytesToRead = m_FileInfo.BytesToEndOfFile;
    m_FileInfo.BytesToEndOfFile = 0;
  }


  // Read data from file into buffer.
  if (mmioRead(m_hMmioFile, pData1, BytesToRead) == -1)
  {
    Box(NULL, "FAILURE", "mmioRead() FAILS.");
  }


  if (!m_LoopingStatus)
  {
	// Keep track of how many bytes have been played so it can be
	// determined when the end of the file is near.  m_TotalBytesPlayed
	// will be used to reprogram the timer so the timer will next
	// trigger precisely when the file has finished playing.
    m_TotalBytesPlayed += BytesToLock;

    if (m_FileInfo.BytesToEndOfFile == 0)
    {
      // All bytes have been loaded into buffer.  Now we
	  // must wait for the end of the file to be played.
      m_WaitForEnd = TRUE;
    }
  }

  
  if (m_LoopingStatus && (Length1 - BytesToRead) > 0)
  {
	// File will repeat playing AND the end of file has been reached.
	// Seek to beginning of file and continue filling the buffer.
    ResetFile();

	if (mmioRead(m_hMmioFile, pData1 + BytesToRead, Length1 - BytesToRead)
		== -1)
    {
      Box(NULL, "FAILURE", "mmioRead() FAILS.");
    }
    m_FileInfo.BytesToEndOfFile = m_FileInfo.DataChunkSize -
		                          (Length1 - BytesToRead);
  }

  if (m_FileInfo.BytesToEndOfFile == 0)
  {
    m_FileInfo.BytesToEndOfFile = m_FileInfo.DataChunkSize;
    ResetFile();  // Seek to beginning of data chunk.
  }


  /***** If necessary, update pData2/Length2 part of buffer. ************/
  /**********************************************************************/
  if (pData2)
  {
    if (Length2 <= m_FileInfo.BytesToEndOfFile)
    {
	  BytesToRead = Length2;
  	  m_FileInfo.BytesToEndOfFile -= Length2;
    }
    else
    {
      BytesToRead = m_FileInfo.BytesToEndOfFile;
	  m_FileInfo.BytesToEndOfFile = 0;
    }


    if (mmioRead(m_hMmioFile, pData2, BytesToRead) == -1)
    {
      Box(NULL, "FAILURE", "mmioRead() FAILS.");
    }


	if (!m_LoopingStatus && (m_FileInfo.BytesToEndOfFile == 0))
    {
      // All bytes have been loaded into buffer.  Now we
      // must wait for the end of the file to be played.
      m_WaitForEnd = TRUE;
    }

	
    if (m_LoopingStatus && (Length2 - BytesToRead) > 0)
    {
	  // File will repeat playing AND the end of file has been reached.
	  // Seek to beginning of file and continue filling the buffer.
      ResetFile();

	  if (mmioRead(m_hMmioFile, pData2 + BytesToRead, Length2 - BytesToRead)
		  == -1)
      {
        Box(NULL, "FAILURE", "mmioRead() FAILS.");
      }
	  m_FileInfo.BytesToEndOfFile = m_FileInfo.DataChunkSize -
		                            (Length2 - BytesToRead);
    }
  }  // End: if (pData2)


  // UNLOCK DIRECT SOUND BUFFER IMMEDIATELY AFTER WRITING TO IT.
  if (m_pDSBuffer->Unlock(pData1, Length1, pData2, Length2) != DS_OK)
  {
    Box(NULL, "FAILURE", "Lock() FAILS");
  }


  if (m_FileInfo.BytesToEndOfFile == 0)
  {
    m_FileInfo.BytesToEndOfFile = m_FileInfo.DataChunkSize;
    ResetFile();  // Seek to beginning of data chunk.
  }


  /**********************************************************************/
  /**********************************************************************/
  if (m_WaitForEnd)
  {
	// To get here, ALL the file bytes have been loaded into the buffer
	// AND the m_LoopingStatus flag is FALSE.
	UINT TimerLength;
    
    // Get the latest m_TotalBytesPlayed count.
    m_pDSBuffer->GetCurrentPosition(&CurrentPlayCursor,
		                            &CurrentWriteCursor);
    if (CurrentPlayCursor > m_PreviousPlayCursor)
    {
      m_TotalBytesPlayed += (CurrentPlayCursor - m_PreviousPlayCursor);
    }
    else if (CurrentPlayCursor < m_PreviousPlayCursor)
    {
	  m_TotalBytesPlayed += (m_DSBufDesc.dwBufferBytes -
		                     (m_PreviousPlayCursor - CurrentPlayCursor));
    }

	// Reprogram the timer so it triggers precisely
	// when the file is done playing.
    TimerLength = (DWORD) (float(m_FileInfo.DataChunkSize -
		                   m_TotalBytesPlayed) /
						   float(m_DSBufDesc.lpwfxFormat->nAvgBytesPerSec)
						   * 1000.0);
	if (TimerLength > 120)
	{
	  // Reprogram timer only if there is > 120ms left to play.
	  // Playing of the file will be terminated at the top of this
	  // function next time the timer is triggered.
	  KillTimer(m_hwnd, m_TimerID);
	  SetTimer(m_hwnd, m_TimerID, TimerLength, DirSndTimerProc);
	}
    else
	{
	  // File is virtually done playing.  Terminate playing and
	  // reload the buffer so it's ready to play again.
	  InitialBufferLoad();
	}
  }  // End: if (m_WaitForEnd)

  return;
}


/*************************************************************************
*
* FUNCTION: DirSnd::OpenWaveFile()
*
* DESCRIPTION: Opens a WAVE file and initializes some m_DSBufDesc fields
*              from information stored in the WAVE file's header.
*
* RETURN VALUE: 0 = NO ERRORS.
*               1 = Cannot find/open DirSnd.m_Filename
*               2 = Can't find "WAVE" chunk.
*               3 = Can't find "fmt " chunk.
*               4 = mmioRead() fails.
*               5 = Can't find "data" chunk.
*
*************************************************************************/
int DirSnd::OpenWaveFile()
{
  MMCKINFO ParentChunk,
           SubChunk;
  unsigned int err;
  char buf[512];

  // Open the WAVE file.
  m_hMmioFile = mmioOpen(m_Filename, NULL, MMIO_READ);
  if(!m_hMmioFile)
  {
	return(1);
  }

  // Verify that file is a WAVE file.
  ParentChunk.fccType = mmioFOURCC('W', 'A', 'V', 'E');
  if (mmioDescend(m_hMmioFile, &ParentChunk, NULL, MMIO_FINDRIFF))
  {
	CloseWaveFile();
	return(2);
  }

  // Find the "fmt " chunk.
  SubChunk.ckid = mmioFOURCC('f', 'm', 't', ' ');
  if (mmioDescend(m_hMmioFile, &SubChunk, &ParentChunk, MMIO_FINDCHUNK))
  {
	CloseWaveFile();
	return(3);
  }
    
  // Allocate memory so m_DSBufDesc.lpwfxFormat points to WAVEFORMATEX struct.
  m_DSBufDesc.lpwfxFormat = new(WAVEFORMATEX);

  // Read PCM wave format data from file into WAVEFORMATEX struct.
  // NOTE: Read ONLY sizeof(PCMWAVEFORMAT) into m_DSBufDesc.lpwfxFormat!!!
  if (err = mmioRead(m_hMmioFile, (char *) m_DSBufDesc.lpwfxFormat,
	  sizeof(PCMWAVEFORMAT)) == -1)
  {
	delete(m_DSBufDesc.lpwfxFormat);
	CloseWaveFile();
   sprintf(buf, "mmioRead in OpenWaveFile failed: code %d (%x)\n",err,err); 
   ErrBox(buf);
	return(4);
  }

  // Get size of data chunk--returned in SubChunk.cksize.
  SubChunk.ckid = mmioFOURCC('d', 'a', 't', 'a');
  if (mmioDescend(m_hMmioFile, &SubChunk, &SubChunk, MMIO_FINDCHUNK))
  {
	delete(m_DSBufDesc.lpwfxFormat);
	CloseWaveFile();
	return(5);
  }

  // Initialize DirSnd class variables..
  m_DSBufDesc.dwSize         = sizeof(DSBUFFERDESC);
  m_DSBufDesc.dwBufferBytes  = SubChunk.cksize; // Size of DS Buffer.
  m_FileInfo.DataChunkSize   = SubChunk.cksize; // Size of "data" chunk.
  m_FileInfo.DataChunkOffset = SubChunk.dwDataOffset;
  m_DSBufDesc.lpwfxFormat->cbSize = 0; // Init. last WAVEFORMATEX field.

  return(0);
}


/*************************************************************************
*
* FUNCTION: DirSnd::CloseWaveFile()
*
*************************************************************************/
void DirSnd::CloseWaveFile()
{
  mmioClose(m_hMmioFile, NULL);
  return;
}


/*************************************************************************
*
* FUNCTION: DirSnd::ResetFile()
*
*************************************************************************/
void DirSnd::ResetFile()
{
  if (mmioSeek(m_hMmioFile, m_FileInfo.DataChunkOffset, SEEK_SET) == -1)
  {
    Box(NULL, "FAILURE", "mmioSeek() FAILS.");
  }
  return;
}