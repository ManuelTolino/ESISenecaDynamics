// QtgDlg.h : header file
//

#if !defined(AFX_QTGDLG_H__CF1A0BE6_DE35_11D3_8CB8_00C0F0060D5B__INCLUDED_)
#define AFX_QTGDLG_H__CF1A0BE6_DE35_11D3_8CB8_00C0F0060D5B__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "Calibration.h" // Control calibration screen dialog header

// Setup values
#define MAXTESTDURATION 120
#define UPDATEFREQ 2 // two updates per second (2Hz)
#define CCSUPDATEFREQ 5 //  Five times a second (5Hz)
#define DATASIZE ( UPDATEFREQ * MAXTESTDURATION )
#define IDT_TIMER1 1
#define IDT_TIMER2 2

#define TOPOFFSET 40  // Offset of top of graphs from top of page
#define ZEROXAXIS 70  // Zero X Axis value
#define ZEROYAXIS (height - 70) // Zero Y Axis value

// QTG Test ID's
#define TEST1AI		1
#define TEST1AIIA	2
#define TEST1AIIB	3
#define TEST1BIA	4
#define TEST1BIB	5
#define TEST1BIC	6
#define TEST1CIA    7
#define TEST1CIB    8
#define TEST1CII	9
#define TEST2AIA    10
#define TEST2AIB    11
#define TEST2AIIA   12
#define TEST2AIIB   13
#define TEST2AIIIA  14
#define TEST2AIIIB  15
#define TEST2AIIIC  115
#define TEST2AIV    17
#define TEST2AV     18
#define TEST2BIA    19
#define TEST2BIB    20
#define TEST2BIC    21
#define TEST2BID    22
#define TEST2BIIA   23
#define TEST2BIIB   24
#define TEST2BIIC   25
#define TEST2BIID   26
#define TEST2BIVA   27
#define TEST2BIVB   28
#define TEST2BIVC   29
#define TEST2BIVD   30
#define TEST2BVA    31
#define TEST2BVB    32
#define TEST2BVC    33
#define TEST2BVD    34
#define TEST2BVE    35
#define TEST2BVF    36
#define TEST2BVG    37
#define TEST2BVIA   38
#define TEST2BVIB   39
#define TEST2BVIIA  40
#define TEST2BVIIB  41
#define TEST2BVIIC  42
#define TEST2BVIII  43
#define TEST2BIX    44
#define TEST2BX     45
#define TEST2CI     46
#define TEST2CIIA   47
#define TEST2CIIB   48
#define TEST2CIII   49
#define TEST2CIV	50
#define TEST2CVA	51
#define TEST2CVB	52
#define TEST2CVI	53
#define TEST2CVIIA	54
#define TEST2CVIIB	55
#define TEST2CVIII	56
#define TEST3AA		57
#define TEST3AB		58
#define TEST3AC		59
#define TEST3BA		60
#define TEST3BB		61
#define TEST3BC		62
#define TEST3BD		63
#define TEST3BE		64
#define TEST3BF		65
#define TEST3BG		66
#define TEST3BH		67
#define TEST3BI		68
#define TEST4A		69
#define TEST4BI		70
#define TEST4BII	71
#define TEST4BIII	72
#define TEST4C		73
#define TEST5A		74
#define TEST5B		75

// Misc defines
#define PRINT 1
#define NOPRINT 0
#define COLUMN 0 
#define WHEEL 1
#define RUDDER 2
#define TRIM 3
#define FLAP 4
#define GEAR 5

#define DEBUG

// Flight Idle Manifold pressure - CHECK THIS AGAINST A/C
#define FLIGHTIDLEMAP 14

/////////////////////////////////////////////////////////////////////////////
// CQtgDlg dialog

class CQtgDlg : public CDialog
{
// Construction
public:
	CQtgDlg(CWnd* pParent = NULL);	// standard constructor
    CQtgDlg(int testID);	// program constructor

	// Distance from runway (for use in visual feature Recognition test4b)
    float fDistFromRw;

	// Structures
    struct _CCS // Control Calibration Screen data structure
	{
		short elevator;		// Elevator position
		short aileron;		// Aileron position
		short rudder;		// Rudder position
		short thrLeft;		// Left throttle position
		short thrRight;		// Right throttle position
		short propLeft;		// Left Prop pitch
		short propRight;	// Right prop pitch
		short mixtureLeft;	// Left engine mixture
		short mixtureRight;	// Right engine mixture
		short eleTrim;		// Elevator trim
		short rudTrim;		// Rudder trim
		short p1Lbrake;		// P1 Left brake position
		short p1Rbrake;		// P1 Right brake position
		short p2Lbrake;		// P2 Right brake position
		short p2Rbrake;		// P2 Left brake position
	} ccs;

	struct _QTGinput  // Inputs to QTG pages
	{
		short count;       // Count incremented by IOS to tell me if data changed
		short altitude;    // QNE altitude (pressure 1013mb) (ft)
		short roc;         // Rate of climb (Fpm)
		short airspeed;    // Airspeed (knots)
		short stallwarner; // Stall warner discrete (Off=0/On=1) 
		short leftmap;     // Left engine manifold pressure (Hg)
		short rightmap;    // Right engine minifold pressure (Hg)
		short leftrpm;	   // Left engine RPM
		short rightrpm;	   // Right engine RPM
		short bankangle;   // Bank angle (deg)
        short pitchangle;  // Pitch Angle (deg)
		short yawangle;    // Yaw Angle (deg)
		short heading;     // Aircraft Heading (deg)
		short columnpos;   // Column position (mm)
		short wheelpos;	   // Wheel position (mm)
		short pedalpos;	   // Pedal position (mm)
		short trimpos;	   // Trim position (mm)
		short flappos;	   // Flap position (deg)
		short gearpos;     // Landing gear position (UP=0/HALF=50/DOWN=100)
		short sideslip;	   // Sideslip angle (deg)
		float columnforce; // Column force (being applied)
		float wheelforce;  // Wheel force (being applied)
		float pedalforce;  // Pedal force (being applied)
		float trimforce;   // Trim force (being applied)
		float transportdelay;// Transport delay for relevent test
	} QTGinput;

	// Dialog Data
	//{{AFX_DATA(CQtgDlg)
	enum { IDD = IDD_QTG_DIALOG };
	//}}AFX_DATA

	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CQtgDlg)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:

    struct _dataAircraft
	{
		int   altitude[DATASIZE];      // QNE altitude (pressure 1013mb) (ft)
		int   roc[DATASIZE];           // Rate of climb (Fpm)
		int   airspeed[DATASIZE];      // Airspeed (knots)
		int   stallwarner[DATASIZE];   // Stall warner discrete (Off=0/On=1) 
		int   leftmap[DATASIZE];       // Left engine manifold pressure (Hg)
		int   rightmap[DATASIZE];      // Right engine minifold pressure (Hg)
		int   thrleft[DATASIZE];       // Left throttle position
		int   thrright[DATASIZE];      // Right throttle position
		int   propleft[DATASIZE];      // Left propeller lever position
		int   propright[DATASIZE];     // Right propeller lever position
		int   leftrpm[DATASIZE];	   // Left engine RPM
		int   rightrpm[DATASIZE];	   // Right engine RPM
		int   bankangle[DATASIZE];     // Bank angle (deg)
        int   pitchangle[DATASIZE];    // Pitch Angle (deg)
		int   yawangle[DATASIZE];      // Yaw Angle (deg)
		int   heading[DATASIZE];       // Aircraft Heading (deg)
		int   columnpos[DATASIZE];	   // Column position (mm)
		float columnforce[DATASIZE];   // Column force (being applied)
		int   wheelpos[DATASIZE];	   // Wheel position (mm)
		float wheelforce[DATASIZE];	   // Wheel force (being applied)
		int   pedalpos[DATASIZE];	   // Pedal position (mm)
		float pedalforce[DATASIZE];	   // Pedal force (being applied)
		int   trimpos[DATASIZE];	   // Trim position (mm)
		float trimforce[DATASIZE];	   // Trim force (being applied)
		int   flappos[DATASIZE];	   // Flap position (deg)
		int   gearpos[DATASIZE];       // Landing gear position (UP=0/HALF=0.5/DOWN=1)
		int   sideslip[DATASIZE];	   // Sideslip angle (deg)
        int   aileronpos[DATASIZE];    // Aileron Position
	} Aircraft;

    struct _dataSim
	{
		int   altitude[DATASIZE];      // QNE altitude (pressure 1013mb) (ft)
		int   roc[DATASIZE];           // Rate of climb (Fpm)
		int   airspeed[DATASIZE];      // Airspeed (knots)
		int   stallwarner[DATASIZE];   // Stall warner discrete (Off=0/On=1) 
		int   leftmap[DATASIZE];       // Left engine manifold pressure (Hg)
		int   rightmap[DATASIZE];      // Right engine minifold pressure (Hg)
		int   thrleft[DATASIZE];       // Left throttle position
		int   thrright[DATASIZE];      // Right throttle position
		int   propleft[DATASIZE];      // Left propeller lever position
		int   propright[DATASIZE];     // Right propeller lever position
		int   leftrpm[DATASIZE];	   // Left engine RPM
		int   rightrpm[DATASIZE];	   // Right engine RPM
		int   bankangle[DATASIZE];     // Bank angle (deg)
        int   pitchangle[DATASIZE];    // Pitch Angle (deg)
		int   yawangle[DATASIZE];      // Yaw Angle (deg)
		int   heading[DATASIZE];       // Aircraft Heading (deg)
		int   columnpos[DATASIZE];	   // Column position (mm)
		float columnforce[DATASIZE];   // Column force (being applied)
		int   wheelpos[DATASIZE];	   // Wheel position (mm)
		float wheelforce[DATASIZE];	   // Wheel force (being applied)
		int   pedalpos[DATASIZE];	   // Pedal position (mm)
		float pedalforce[DATASIZE];	   // Pedal force (being applied)
		int   trimpos[DATASIZE];	   // Trim position (mm)
		float trimforce[DATASIZE];	   // Trim force (being applied)
		int   flappos[DATASIZE];	   // Flap position (deg)
		int   gearpos[DATASIZE];       // Landing gear position (UP=0/HALF=0.5/DOWN=1)
		int   sideslip[DATASIZE];	   // Sideslip angle (deg)
		int   aileronpos[DATASIZE];    // Aileron Position
		float transportdelay;          // Transport delay for relevent test
		float visuallatency;           // Visual latency timing
	} Simulator;

	// Typedefs
    typedef struct Point { int x, y; };
    typedef struct GraphSection
	{  
		Point TopPoint, BottomPoint;
		int PixDiff;
		float PerUnit;
	};
	
	// Function Prototypes
	void MessagePage(CDC *, char [], int);
	void Performance_1ai(CDC *, int);
	void Performance_1bi(CDC *, int );
	void Performance_1ci(CDC *, int );
	void Handling_2ai(CDC *, int, int );
	void Handling_2av(CDC *, int );
	void Handling_2bi(CDC *, int );
	void Handling_2bva(CDC *, int, int );
	void Handling_2bvi(CDC *, int );
	void Handling_2bix(CDC *, int );
	void Handling_2ci(CDC *, int );
	void Handling_2cii(CDC *, int );
	void Handling_2ciii(CDC *, int );
	void Handling_2civ(CDC *, int );
    void Handling_2cv(CDC *, int );
	void Handling_2cviii(CDC *, int );
	void Visual_4b(CDC *, int );
	void Transport(CDC *, int );
    void LoadAircraftData( void );
		
	// Global variables used by the dialog
	CRect m_rWinRect;
	CCalibration cal;
	RECT rect;
	int m_testID, timer_id1, timer_id2, ccsflag, time_index;

	// Generated message map functions
	//{{AFX_MSG(CQtgDlg)
	virtual BOOL OnInitDialog();
	afx_msg void OnPaint();
	afx_msg void OnSize(UINT nType, int cx, int cy);
	afx_msg void OnTimer(UINT nIDEvent);
	afx_msg void OnGotoCCS();
	afx_msg void OnPrint();
	afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
	afx_msg void OnHScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar);
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_QTGDLG_H__CF1A0BE6_DE35_11D3_8CB8_00C0F0060D5B__INCLUDED_)
