/***** FUNCTION PROTOTYPE *******************************/
BOOL CALLBACK DSEnumProc(GUID *, LPSTR, LPSTR, LPVOID);

/***** GLOBALS ******************************************/
extern GUID     *gpGuid;        // Declared in dirsnd.h
extern DrvrInfo *gpDrivers;     // Declared in dsdemo.h
extern DrvrInfo *gpDriversHead; // Declared in dsdemo.h

/*************************************************************************
*
* FUNCTION: DSEnumProc()
*
* DESCRIPTION: This function will be called via DirectSoundEnumerate().
*              When DirectSoundEnumerate() is called, this function
*              will be called once for each Direct Sound device
*              installed in the system.  This function is then passed
*              information about each Direct Sound driver installed.
*
*              As designed, this function stores the GUID pointer
*              and its description in a GLOBAL linked list.  These
*              GUID descriptions are then loaded into a Combo Box
*              so the user can select the desired GUID (Direct Sound
*              device).  See DialogProc() in dsdemo.cpp.
*
* RETURN: TRUE  - Continues enumerating Direct Sound drivers.
*
*         FALSE - Stops enumerating Direct Sound drivers.  This
*                 is what to return when you have found the driver
*                 you're looking for.
*
*************************************************************************/
BOOL CALLBACK DSEnumProc(GUID *ptrGuid, LPSTR lpstrDescription,
				         LPSTR lpstrModule, LPVOID lpContext)
{
  // Store GUID information in global linked list.
  if (gpDrivers == NULL)
  {
	// Allocate space for first entry in linked list.
	gpDrivers = new(DrvrInfo);
	gpDriversHead = gpDrivers;  // Save 1st entry in list.
  }
  else
  {
    // gpDrivers points to last entry in list.
	gpDrivers->Next = new(DrvrInfo);  // Add new link to list.
    gpDrivers = gpDrivers->Next;      // Point to new entry.
  }

  // Add GUID and Driver Description to linked list.
  gpDrivers->pGuid = ptrGuid;
  strcpy(gpDrivers->Descrip, lpstrDescription);
  gpDrivers->Next = NULL;  // Mark end of linked list.

  /*
  // THIS IS USED FOR DEBUGGING
  char String[200];
  sprintf(String, "Device = %s\n"
	              "Module = %s\n"
	              "gpGuid = 0x%p", gpDrivers->Descrip, lpstrModule,
				                   gpDrivers->pGuid);
  Box(NULL, "Values obtained via DSEnumProc()", String);
  */

  return(TRUE);  // Return TRUE to continue enumerating devices.
}