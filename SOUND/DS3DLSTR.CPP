#define INITGUID  // THIS LINE REQUIRED FOR QueryInterface() CALL!
#include "dsound.h"

class Listener3D
{
  private:
	LPDIRECTSOUND        m_p3DListenObj;
	LPDIRECTSOUNDBUFFER  m_pPrimBuf;

  public:
    LPDIRECTSOUND3DLISTENER m_p3DListener;

	int Create(GUID *, HWND, DWORD);
	int Destroy();
};


/*************************************************************************
*
* FUNCTION: Listener3D::Create()
*
* DESCRIPTION: Creates a PRIMARY buffer capable of implementing the
*              IDirect3DListener Direct Sound class methods.
*              3D listener functions will be accessed via the
*              LPDIRECTSOUND3DLISTENER parameter passed to this function.
*
* NOTE: Because there can be only one primary buffer, there should
*       be ONLY ONE OBJECT of type "Listener3D" and Listener::Create()
*       should be called only once.
*
*       Before your program terminates, make certain to call the
*       Listener::Destroy() method for this Listener3D object.
*
* RETURN:   0 - NO ERRORS
*           1 - DirectSoundCreate() FAILS
*           2 - SetCooperativeLevel() FAILS
*           3 - CreateSoundBuffer() FAILS
*           4 - QueryInterface() FAILS
*
*************************************************************************/
int Listener3D::Create(GUID *pGuid, HWND hwnd, DWORD CoopLevel)
{
  DSBUFFERDESC DSBufDesc;

  DSBufDesc.dwSize        = sizeof(DSBUFFERDESC);
  DSBufDesc.dwFlags       = DSBCAPS_CTRL3D | DSBCAPS_PRIMARYBUFFER;
  DSBufDesc.dwBufferBytes = 0;
  DSBufDesc.dwReserved    = 0;
  DSBufDesc.lpwfxFormat   = 0;


  // Create a Direct Sound object.
  if (DirectSoundCreate(pGuid, &m_p3DListenObj, NULL) == DS_OK)
  {
    // Set cooperative level.
    if (m_p3DListenObj->SetCooperativeLevel(hwnd, CoopLevel) != DS_OK)
    {
	  return(2);
    }
  }
  else
  {
	return(1);
  }

    // Create Direct Sound primary buffer.
  if (m_p3DListenObj->CreateSoundBuffer(&DSBufDesc, &m_pPrimBuf, NULL)
	  != DS_OK)
  {
	return(3);
  }

  // Initialize m_p3DListener pointer so it can access
  // IID_DirectSound3DListener metods.
  if (m_pPrimBuf->QueryInterface(IID_IDirectSound3DListener,
	  (void **) &m_p3DListener) != DS_OK)
  {
	return(4);
  }

  return(0);
}


/*************************************************************************
*
* FUNCTION: Listener3D::Destroy()
*
* DESCRIPTION: Call Release() for the 3D listener pointer,
*              the 3D listener primary buffer, and the 3D
*              listener Direct Sound object.
*
* RETURN: 0 - NO ERRORS
*         1 - Release() for 3D listener pointer fails.
*         2 - Release() for 3D listener primary buffer fails.
*         3 - Release() for Direct Sound object fails.
*
*************************************************************************/
int Listener3D::Destroy()
{
  if (m_p3DListener->Release() != DS_OK)
  {
    return(1);
  }

  if (m_pPrimBuf->Release() != DS_OK)
  {
    return(2);
  }
  
  if (m_p3DListenObj->Release() != DS_OK)
  {
    return(3);
  }

  return(0);
}