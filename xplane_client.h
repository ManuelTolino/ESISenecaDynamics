// XPlaneC.cpp : Defines the entry point for the console application.

// References
// http://www.nuclearprojects.com/xplane/xplaneref.html
// http://www.nuclearprojects.com/xplane/info.shtml
// https://docs.microsoft.com/en-us/windows/desktop/winsock/windows-sockets-error-codes-2
// https://stackoverflow.com/questions/32471004/udp-client-not-receiving-udp-server-message

#ifndef XPLANE_CLIENT
#define XPLANE_CLIENT

#include <string.h>
#include <windows.h>      // Needed for all Winsock stuff

#pragma comment(lib,"ws2_32.lib") // Winsock Library


/*----------------------------------------------
Defines
----------------------------------------------*/
#define  PORT_NUM             49000  // Port number used
#define  IP_ADDR        "127.0.0.1"  // IP address of server1 (*** HARDWIRED ***)

#define XCHR char       // (character, in local byte - order for the machine you are on)
#define XINT int        // (4 - byte int, in local byte - order for the machine you are on)
#define XFLT float      // (4 - byte ints and floats, in local byte - order for the machine you are on)
#define XDOB double     // (double - precision float, in local byte - order for the machine you are on)
#define strDIM 500
#define vehDIM 10
#define net_SIZE_buff 1024

#define DEFAULT_UPDATE_RATE 20

/*----------------------------------------------
Data Types
----------------------------------------------*/
typedef struct sim_data_type {
    float latitude_deg;         // 0: sim/flightmodel/position/latitude
    float longitude_deg;        // 1: sim/flightmodel/position/longitude
    float altitude_m_msl;       // 2: sim/flightmodel/position/elevation
    float altitude_m_agl;       // 3: sim/flightmodel/position/y_agl
    float ground_speed_mps;     // 4: sim/flightmodel/position/groundspeed
    float true_airspeed_mps;    // 5: sim/flightmodel/position/true_airspeed
    float vertical_speed_mps;   // 6: sim/flightmodel/position/vh_ind
    float true_heading_deg;     // 7: sim/flightmodel/position/true_psi
    float mag_heading_deg;      // 8: sim/flightmodel/position/mag_psi
    float true_heading_rate;    // 9: sim/flightmodel/position/R
    float track_deg;            // 10: sim/flightmodel/hpath
    float pitch_deg;            // 11: sim/flightmodel/position/true_theta
    float roll_deg;             // 12: sim/flightmodel/position/true_phi
    float slip_skid;            // 13: sim/cockpit2/gauges/indicators/slip_deg
    float mag_variation_deg;    // 14: sim/flightmodel/position/magnetic_variation
} sim_data_type;

enum {
    XPLN_LATITUDE_DEG,

};

#pragma pack(push,1)
typedef struct rref_request_type {
    XCHR cmd[5];
    XINT freq;
    XINT idx;
    XCHR str[400];
} rref_request_type;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct dref_int_struct {
    XCHR cmd[5];
    XINT var;
    XCHR dref_path[500];
} dref_int_struct;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct dref_float_struct {
    XCHR cmd[5];
    XFLT var;
    XCHR dref_path[500];
} dref_float_struct;
#pragma pack(pop)

typedef struct rref_data_type {
    XINT idx;
    XFLT val;
} rref_data_type;

typedef struct sockaddr_in sockaddr_in;
typedef struct sockaddr sockaddr;


/*----------------------------------------------
API Functions
----------------------------------------------*/
void xplane_client_open();
void xplane_client_poll();
void xplane_client_close();

sim_data_type *get_current_sim_state();

void set_float_dataref(char *ref, float val);

#endif