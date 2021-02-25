#include "xplane_client.h"

/*----------------------------------------------
Statics
----------------------------------------------*/
static int client_s;
static sockaddr_in server_addr;
static sim_data_type current_state;
static char is_socket_open;

static void NET_request_dataref(int client, int idx, int freq, char *str)
{
    rref_request_type tosend;

    memset(&tosend, 0, sizeof(tosend));
    strncpy(&tosend.cmd, "RREF", 4);
    tosend.cmd[4] = 0;
    strncpy(&tosend.str, str, strlen(str) + 1);

    tosend.freq = freq;
    tosend.idx = idx;

    size_t s = sizeof(tosend);
    if ( s != 413 )
        return;

    XINT result = sendto(client, &tosend, sizeof(tosend), 0, (const struct sockaddr*)&server_addr, sizeof(rref_request_type));
    if (result < 0)
    {
        int err = WSAGetLastError();
        is_socket_open = 0;
        //print("sendto() failed with error code : %d\n", err);
        return;
    }
}

static void NET_set_int_dataref(int client, char *str, int val)
{
    dref_int_struct tosend;

    memset(&tosend, 0, sizeof(tosend));
    strncpy(&tosend.cmd, "DREF", 4);
    strncpy(&tosend.dref_path, str, strlen(str) + 1);
    tosend.var = val;

    size_t s = sizeof(tosend);
    if ( s != 509 )
        return;

    XINT result = sendto(client, &tosend, sizeof(tosend), 0, (const struct sockaddr*)&server_addr, sizeof(rref_request_type));
    if (result < 0)
    {
        int err = WSAGetLastError();
        is_socket_open = 0;
        //print("sendto() failed with error code : %d\n", err);
        return;
    }
}

static void NET_set_float_dataref(int client, char *str, float val)
{
    dref_float_struct tosend;

    memset(&tosend, 0, sizeof(tosend));
    strncpy(&tosend.cmd, "DREF", 4);
    strncpy(&tosend.dref_path, str, strlen(str) + 1);
    tosend.var = val;

    size_t s = sizeof(tosend);
    if ( s != 509 )
        return;

    XINT result = sendto(client, &tosend, sizeof(tosend), 0, (const struct sockaddr*)&server_addr, sizeof(rref_request_type));
    if (result < 0)
    {
        int err = WSAGetLastError();
        is_socket_open = 0;
        //print("sendto() failed with error code : %d\n", err);
        return;
    }
}


static void NET_recv(int client, sockaddr_in server_addr)
{
    XCHR data_out[net_SIZE_buff];
    int i, num_structs;
    int retcode;
    i = 0;

    do {
        int addr_len = sizeof(server_addr);
        retcode = recvfrom(client, data_out, net_SIZE_buff, 0, (struct sockaddr *)&server_addr, &addr_len);
        if (retcode < 0)
        {
            int err = WSAGetLastError();
            //printf("recvfrom() failed with error code : %d\n", err);
            return;
        }

        if (strncmp(data_out, "RREF", 4) == 0)
        {
            num_structs = (retcode - 5) / sizeof(rref_data_type);
            rref_data_type *f = data_out + 5;

            for (i = 0; i < num_structs; i += 1)
            {
                ((float *) &current_state)[ f[i].idx ] = f[i].val;
            }
        }
        i++;
    } while (retcode > 0); // Repeat until the socket no longer has any data
}

static int setup_socket(int *client_s, sockaddr_in *server_addr)
{
    struct timeval read_timeout;
    int opt;
    u_long iMode = 1; // Non-blocking mode (https://docs.microsoft.com/en-us/windows/desktop/api/winsock/nf-winsock-ioctlsocket)
    opt = 1;

    read_timeout.tv_sec = 0;
    read_timeout.tv_usec = 10;

    *client_s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (client_s < 0)
    {
        //printf("*** ERROR - socket() failed \n");
        is_socket_open = 0;
        WSACleanup();
        return 1;
    }

    opt = 1;
    if (setsockopt(*client_s, SOL_SOCKET, SO_REUSEADDR, (char *)&opt, sizeof(opt)) == SOCKET_ERROR)
    {
        //printf("\nERROR in setting SO_REUSEADDR : %d \n", WSAGetLastError());
        is_socket_open = 0;
        closesocket(*client_s);
        WSACleanup();
        return 1;
    }

    if (ioctlsocket(*client_s, FIONBIO, &iMode) != NO_ERROR)
    {
        is_socket_open = 0;
        closesocket(*client_s);
        WSACleanup();
        return 1;
    }

    // UDP Timeout
    if (setsockopt(*client_s, SOL_SOCKET, SO_RCVTIMEO, &read_timeout, sizeof(read_timeout)) == SOCKET_ERROR)
    {
        //printf("\nERROR in broadcasting ERROR CODE : %d \n", WSAGetLastError());
        is_socket_open = 0;
        closesocket(*client_s);
        WSACleanup();
        return 1;
    }

    // UDP SOCKET Binding
    //if (bind(*client_s, (sockaddr *)server_addr, sizeof(sockaddr_in)) == SOCKET_ERROR)
    //{
    //    int err = WSAGetLastError();
    //    //printf("\nUDP socket binding failed ERROR CODE : %d\n", err);
    //    is_socket_open = 0;
    //    closesocket(*client_s);
    //    WSACleanup();
    //    return 1;
    //}

    is_socket_open = 1;
    return 0;
}

void xplane_client_open()
{
    WORD wVersionRequested = MAKEWORD(1, 1);       // Stuff for WSA functions
    WSADATA wsaData;                               // Stuff for WSA functions

    if (is_socket_open)
        return;

    WSAStartup(wVersionRequested, &wsaData);
    server_addr.sin_family = AF_INET;                 // Address family to use
    server_addr.sin_port = htons(PORT_NUM);           // Port num to use
    server_addr.sin_addr.S_un.S_addr = inet_addr(IP_ADDR);

    setup_socket(&client_s, &server_addr);

    NET_request_dataref(client_s, 0,  DEFAULT_UPDATE_RATE, "sim/flightmodel/position/latitude");
    NET_request_dataref(client_s, 1,  DEFAULT_UPDATE_RATE, "sim/flightmodel/position/longitude");
    NET_request_dataref(client_s, 2,  DEFAULT_UPDATE_RATE, "sim/flightmodel/position/elevation");
    NET_request_dataref(client_s, 3,  DEFAULT_UPDATE_RATE, "sim/flightmodel/position/y_agl");
    NET_request_dataref(client_s, 4,  DEFAULT_UPDATE_RATE, "sim/flightmodel/position/groundspeed");
    NET_request_dataref(client_s, 5,  DEFAULT_UPDATE_RATE, "sim/flightmodel/position/true_airspeed");
    NET_request_dataref(client_s, 6,  DEFAULT_UPDATE_RATE, "sim/flightmodel/position/vh_ind");
    NET_request_dataref(client_s, 7,  DEFAULT_UPDATE_RATE, "sim/flightmodel/position/true_psi");
    NET_request_dataref(client_s, 8,  DEFAULT_UPDATE_RATE, "sim/flightmodel/position/mag_psi");
    NET_request_dataref(client_s, 9,  DEFAULT_UPDATE_RATE, "sim/flightmodel/position/R");
    NET_request_dataref(client_s, 10, DEFAULT_UPDATE_RATE, "sim/flightmodel/position/hpath");
    NET_request_dataref(client_s, 11, DEFAULT_UPDATE_RATE, "sim/flightmodel/position/true_theta");
    NET_request_dataref(client_s, 12, DEFAULT_UPDATE_RATE, "sim/flightmodel/position/true_phi");
    NET_request_dataref(client_s, 13, DEFAULT_UPDATE_RATE, "sim/cockpit2/gauges/indicators/slip_deg");
    NET_request_dataref(client_s, 14, DEFAULT_UPDATE_RATE, "sim/flightmodel/position/magnetic_variation");
}

void xplane_client_poll()
{
    int i;

    if (!is_socket_open)
    {
        return;
    }

    NET_recv(client_s, server_addr);
}

void xplane_client_close()
{
    if (closesocket(client_s) < 0)
    {
        //printf("*** ERROR - closesocket() failed \n");
        is_socket_open = 0;
        return;
    }

    WSACleanup();
}

sim_data_type *get_current_sim_state()
{
    return &current_state;
}

void set_float_dataref(char *ref, float val)
{
    NET_set_float_dataref(client_s, ref, val);
}