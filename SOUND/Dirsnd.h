#define INITGUID  // THIS LINE REQUIRED FOR QueryInterface() CALL!

#include <windows.h>
#include <windowsx.h>
#include <mmsystem.h>
#include <dsound.h>


void Box(const HWND, const char*, const char*);
void PrintPCMData(WAVEFORMATEX *, unsigned long);
void ErrBox(char *msg);
void WarnBox(char *msg);

void CALLBACK DirSndTimerProc(HWND, UINT, UINT, DWORD);

/***** TYPEDEFs ***********************************************/
typedef struct _FileInfo
{
  UINT BytesToEndOfFile;
  UINT DataChunkSize;
  UINT DataChunkOffset;
} FILEINFO;


/***** C++ CLASSES ********************************************/
class DirSnd
{
  private:
	// PRIVATE VARIABLES
	BOOL     m_LoopingStatus,  // 0 = play once, 1 = infinite play loop.
		     m_WaitForEnd;
	char     m_Filename[_MAX_PATH];
	DWORD    m_dwCoopLevel,
		     m_PreviousPlayCursor,
			 m_TotalBytesPlayed;
	FILEINFO m_FileInfo;
	GUID    *m_pGuid;
	HMMIO    m_hMmioFile;
	int      m_CreateErrorCode;
   HANDLE               m_posnEvent;

	// PRIVATE METHODS
	void CloseWaveFile(void);
	void InitialBufferLoad(void);
	int  OpenWaveFile(void);
	void ResetFile(void);

  public:
	// PUBLIC VARIABLES
    DSBUFFERDESC          m_DSBufDesc;
	HWND                  m_hwnd;
    LPDIRECTSOUND         m_pDS;
    LPDIRECTSOUNDBUFFER   m_pDSBuffer;
	LPDIRECTSOUND3DBUFFER m_pDS3DBuffer;
   LPDIRECTSOUNDNOTIFY   m_pDirectSoundNotify;
    UINT                  m_TimerID;
   int      m_loop, m_loopend;

    // PUBLIC METHODS
	void Duplicate(DirSnd &);
    int     Create(DWORD, DWORD, HWND, GUID*, UINT, UINT, char*, int, int);
    void    Destroy(void);
       static int Init(HWND);
	void    Play(BOOL);
   void    Frame(void);
	void    Stop(void);
    void    UpdateStreamBuffer(void);
};


