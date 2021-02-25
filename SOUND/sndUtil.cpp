/***** FUNCTION PROTOTYPES ******************************/
#include <stdio.h>
//#include "dirsnd.h"
#include <windows.h>

/*************************************************************************
*
* FUNCTION: Box()
*
*************************************************************************/
void Box(const HWND hwnd, const char* String1, const char* String2)
{
  MessageBox(hwnd, String2, String1, MB_ICONEXCLAMATION | MB_OK);
  return;
}

// Display an error box and exit
void ErrBox(char *msg)
{
   MessageBox(NULL, msg, "Dynamics Sound Error", MB_OK | MB_ICONERROR | MB_TASKMODAL);
   exit(2);
}


// Display an warning box
void WarnBox(char *msg)
{
   MessageBox(NULL, msg, "Dynamics Sound Warning", MB_OK | MB_ICONWARNING | MB_TASKMODAL);
}


/*************************************************************************
*
* FUNCTION: PrintPCMData()
*
*************************************************************************/
void PrintPCMData(WAVEFORMATEX *WaveFormatEx, unsigned long BufferSize)
{
  char Buffer[200];

  sprintf(Buffer,
          "Format Tag = %d\n"
          "Channels = %d\n"
          "SampPerSec = %ld\n"
          "Block Align = %ld\n"
          "Bits Per Sample = %d\n"
          "Avg. Bytes Per Sec. = %d\n"
          "Buffer Size = %lu",
          WaveFormatEx->wFormatTag, WaveFormatEx->nChannels,
          WaveFormatEx->nSamplesPerSec, WaveFormatEx->nBlockAlign,
          WaveFormatEx->wBitsPerSample, WaveFormatEx->nAvgBytesPerSec,
          BufferSize);

  Box(NULL, "PCM WAVE DATA", Buffer);
  return;
}