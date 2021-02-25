//**************************
//   RCS Markers           *
//**************************


//$Header: C:/master/seneca/RCS/FastLogger.h 1.1 2000/05/10 15:13:12 darren Exp $
//$Log: FastLogger.h $
//Revision 1.1  2000/05/10 15:13:12  darren
//Initial revision
//

//----------------------------------------------------------------------
//
#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// FastLogger.h : header file
//

//#include <afxwin.h>
#include <windows.h>

#define LOGCOMPLETE 90001
#define NOCOUNTER 90050
#define FRAMEOVERRUN 90051

#define ARRAYSIZE 240
#define LOGFREQ 100 // a 100Hz timer

/////////////////////////////////////////////////////////////////////////////
// CFastLogger Class

class CFastLogger
{

// Construction
public:
	CFastLogger();
	~CFastLogger();
	int Logger(float, float, float);
	float arControl[ARRAYSIZE]; 
    float arLightLevel[ARRAYSIZE];  
    float arPotPos[ARRAYSIZE]; 
    int i;
// Implementation
protected:
	LARGE_INTEGER PerformanceFreq;
	LARGE_INTEGER LastBigNumber;
	double FrameTime;
	__int64 Trigger;
	
};

/////////////////////////////////////////////////////////////////////////////
// End of file FastLogger.h