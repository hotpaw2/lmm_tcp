//
//  lmm_tcp.c
//
//  An rtl_tcp server for the LimeSDR Mini USB peripheral
//  Serves SDR IQ data using the rtl_tcp protocol over iPv6
//
#define VERSION "v.1.0.101" 	//   2019-05-03  rhn
//
//   pi :    cc -std=c99 -lm  -lLimeSuite -O2 -o lmm_tcp lmm_tcp.c
//   requires first building and installing LimeSuite
//       https://github.com/myriadrf/LimeSuite
//
//   Copyright 2019 Ronald H Nicholson Jr. All Rights Reserved.
//   re-distribution permitted under the Mozilla Public License version 1.1
//       https://www.mozilla.org/en-US/MPL/1.1/
//
// This program is distributed in the hope that it might be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE whatsoever.
//

#define SAMPLE_BITS     ( 8)            // default to match rtl_tcp
// #define SAMPLE_BITS  (16)            // default to match rsp_tcp
// #define SAMPLE_BITS 	(32)    // LimeSuite capable of float32 IQ data
#define GAIN_SCALE      (16.0)          // gain scale factor
#define PORT            (1234)      	// default port

#define _POSIX_C_SOURCE 200112L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <inttypes.h>
#include <unistd.h>
#include <math.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <errno.h>

#include "lime/LimeSuite.h"

int sendcallback(float *p, int n);
static void sighandler(int signum);
int error(void);
int set_sample_rate(float sr);
int set_frequency(float_type f0);
void stop_running_device();
int decode_5cmd(int n, char *cbuffer) ;

lms_stream_t    streamId;
lms_device_t*   device          =  NULL;

int             sendErrorFlag   =  0;
int             sampleBits      =  SAMPLE_BITS;
static long int totalSamples    =  0;
long            sampRate        =  768000;
long            previousSRate   = -1;
float           gain0           =  GAIN_SCALE * 20.0;
int             client_sockfd   = -1;
int             running         =  0;

#define IQBUF_SIZE    (16384)
float iqbuffer[2 + 2 * IQBUF_SIZE]; // must hold I+Q values of each sample

static int     	        listen_sockfd;
struct sigaction        sigact, sigign;
float         acc_r             =  0.0;    // accumulated rounding
float         sMax              =  0.0;    // for debug
float         sMin              =  0.0;

char UsageString[]
	= "Usage:    [-p listen port (default: 1234)]\n          [-b 16]";

int main(int argc, char *argv[]) {
    
    struct sockaddr_in6 serv_addr ;
    char   client_addr_ipv6[100];
    char   cbuffer[1024];
    int    portno       =  PORT;    //
    char  *ipaddr       =  NULL;    // "127.0.0.1"
    int    timeout;
    struct pollfd pfds[200];
    int    nfds = 0;
    int n;
    
    if (argc > 1) {
        if ((argc % 2) != 1) {
            printf("%s\n", UsageString);
            exit(0);
        }
        for (int arg=3; arg<=argc; arg+=2) {
            if (strcmp(argv[arg-2], "-p")==0) {
                portno = atoi(argv[arg-1]);
                if (portno == 0) {
                    printf("invalid port number entry %s\n", argv[arg-1]);
                    exit(0);
                }
            } else if (strcmp(argv[arg-2], "-b")==0) {
                if (strcmp(argv[arg-1],"16")==0) {
                    sampleBits = 16;
                } else if (strcmp(argv[arg-1],"8")==0) {
                    sampleBits =  8;
                } else {
                    printf("%s\n", UsageString);
                    exit(0);
                }
            } else if (strcmp(argv[arg-2], "-a")==0) {
                ipaddr = argv[arg-1];        // unused
            } else {
                printf("%s\n", UsageString);
                exit(0);
            }
        }
    }
    
    printf("\nlmm_tcp Version %s\n\n", VERSION);
    printf("Serving %d-bit samples on port %d\n", sampleBits, portno);
    
    if ((n = LMS_GetDeviceList(NULL)) < 0) { error(); }
    printf("Devices found: %d \n" , n );
    if (n < 1) { return -1; }
    
    sigact.sa_handler = sighandler;
    sigemptyset(&sigact.sa_mask);
    sigact.sa_flags = 0;
    sigign.sa_handler = SIG_IGN;
    sigaction(SIGINT, &sigact, NULL);
    sigaction(SIGTERM, &sigact, NULL);
    sigaction(SIGQUIT, &sigact, NULL);
    sigaction(SIGPIPE, &sigign, NULL);
    
    lms_info_str_t *list;
    list = (lms_info_str_t *)malloc(256);
    if (LMS_GetDeviceList(list) < 0) { error(); }
    printf("%s \n", list);
    
    if ( LMS_Open(&device, list[0], NULL) != 0) { error(); }
    
    if (LMS_Init(device) != 0) {
    }
    if (LMS_EnableChannel(device, LMS_CH_RX, 0, true) != 0) {
        error();
    }
    
    float sampRate = 1.6 * 1.0e6;
    set_sample_rate(sampRate) ;
    
    // Set center frequency
    float_type f0 = 162400000.0;
    set_frequency(f0);
    
    // Configure LPF, bandwidth 8 MHz
    if (LMS_SetLPFBW(device, LMS_CH_RX, 0, 2e6) != 0) { error(); }
    // Set RX gain
    float rxg = 0.7;
    if (LMS_SetNormalizedGain(device, LMS_CH_RX, 0, rxg) != 0) { error(); }
    float_type gain; // normalized gain
    if (LMS_GetNormalizedGain(device, LMS_CH_RX, 0, &gain) != 0) { error(); }
    printf("Normalized RX Gain: %f \n", gain);
    unsigned int gaindB = 0.0; // gain in dB
    if (gain > 0.0) {
        if (LMS_GetGaindB(device, LMS_CH_RX, 0, &gaindB) != 0) { error(); }
        printf("RX Gain: %f dB \n" , gaindB );
    }
    if (LMS_Calibrate(device, LMS_CH_RX, 0, 2e6, 0) != 0) { error(); }
    streamId.channel             =     0;          // channel number
    streamId.fifoSize            =  1024 * 1024;    // fifo size in samples
    streamId.throughputVsLatency =     1.0;     // optimize for max throughput
    streamId.isTx                =  false;      // RX channel
    // streamId.dataFmt          =  LMS_FMT_I16;
    streamId.dataFmt             =  LMS_FMT_F32;
    
    printf("\nlmm_tcp IPv6 server starting on port %d\n", portno);
    
    listen_sockfd = socket(AF_INET6, SOCK_STREAM, 0);
    if (listen_sockfd < 0) {
        printf("ERROR opening socket");
        return(-1);
    }
    
#ifdef __APPLE__
    int val = 1;
    int r = setsockopt(listen_sockfd, SOL_SOCKET, SO_NOSIGPIPE,
                       (void*)&val, sizeof(val));
    printf("setsockopt status = %d \n", r);
#endif
    
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin6_flowinfo = 0;
    serv_addr.sin6_family = AF_INET6;
    serv_addr.sin6_addr = in6addr_any;
    serv_addr.sin6_port = htons(portno);
    
    struct linger ling = {1,0};
    int rr = 1;
    setsockopt(listen_sockfd, SOL_SOCKET, SO_REUSEADDR,
               (char *)&rr, sizeof(int));
    setsockopt(listen_sockfd, SOL_SOCKET, SO_LINGER,
               (char *)&ling, sizeof(ling));
    
    // Sockets Layer Call: bind()
    if (bind( listen_sockfd, (struct sockaddr *)&serv_addr,
             sizeof(serv_addr) ) < 0) {
        printf("ERROR on bind to listen\n");
        return(-1);
    }
    
    listen(listen_sockfd, 5);
    
    int run_listen = 1;
    while (run_listen) {
        
        fprintf(stdout, "listening for socket connection \n");
        // accept a connection
        struct sockaddr_in6 cli_addr;
        socklen_t claddrlen = sizeof(cli_addr);
        client_sockfd = accept( listen_sockfd,
                               (struct sockaddr *) &cli_addr,
                               &claddrlen );
        if (client_sockfd < 0) {
            printf("ERROR on accept\n");
            error();
            // break;
        }
        
        pfds[0].fd      = client_sockfd;
        pfds[0].events  = POLLIN;
        nfds = 1;
        
        inet_ntop(AF_INET6, &(cli_addr.sin6_addr), client_addr_ipv6, 100);
        printf("\nConnected to client with IP address: %s\n",
               client_addr_ipv6);
        
        if (LMS_SetupStream(device, &streamId) != 0) { error(); }
        int m = LMS_StartStream(&streamId);
        printf("LMS start status = %d\n", m);
        running        =  1;
        sendErrorFlag  =  0;
        long int nn    =  0;
        
        while ( 1 ) {
            int samplesRead;
            unsigned int wait_mS = 1000;
            samplesRead = LMS_RecvStream(&streamId,
                                         iqbuffer, IQBUF_SIZE, 
					 NULL, wait_mS);
            int n = samplesRead;
            if (n > 0) {
                float *p = (float *)iqbuffer;
                int status = sendcallback(p, n);
                if (status < 0) {
                    stop_running_device();
                    break;
                }
            }
            //
            if (n > 0 && running) {
                poll(pfds, 1, 2);
                if (pfds[0].revents & POLLIN) {
                    memset(cbuffer,0, 256);
                    int m = recv(pfds[0].fd, cbuffer, 255, 0);
                    if (m > 0) {
                        decode_5cmd(m, cbuffer);
                    }
                }
            }
        }
    }
    if (running == 1) {
        stop_running_device();
    }
    LMS_Close(device);
    
    fflush(stdout);
    return 0;
}  //  main

int error()
{
    if (device != NULL) {
        LMS_Close(device);
        device = NULL;
    }
    exit(-1);
}

void stop_running_device()
{
    if (running) {
        int status = LMS_StopStream(&streamId);
        printf("\nstop status = %d\n", status);
        LMS_DestroyStream(device, &streamId);
        running = 0;
    }
}

static void sighandler(int signum)
{
    fprintf(stderr, "Signal caught, exiting!\n");
    fflush(stderr);
    close(listen_sockfd);
    if (client_sockfd != 0) {
        close(client_sockfd);
        client_sockfd = -1;
    }
    if (device != NULL) {
        LMS_Close(device);
        device = NULL;
    }
    exit(-1);
}

void send_prefix() {     // send 16 or 12-byte rtl_tcp header
    int sz = 16;
    int n  =  0;
    if (sampleBits == 8) { sz = 12; }
    // LMM0 16
    char header[16] = { 0x4c,0x4d,0x4d,0x30, 0,0,0,sampleBits,
        0,0,0,1,             0,0,0,2 };
#ifdef __APPLE__
    n = send(client_sockfd, header, sz, 0);
#else
    n = send(client_sockfd, header, sz, MSG_NOSIGNAL);
#endif
}

int set_sample_rate(float sr)
{
    int n;
    float sampRate = 2400000.0;
    sampRate = sr;
    // Set sample rate , preferred oversampling in RF 8x
    if (LMS_SetSampleRate(device, sampRate, 8) != 0) {
        error();
    }
    previousSRate = sampRate;
    
    float_type rate, rf_rate;
    if (LMS_GetSampleRate(device, LMS_CH_RX, 0, &rate, &rf_rate) != 0)  {
        error();
    }
    printf( "USB sample rate: %f MHz\nRF ADC sample rate: %f MHz\n",
           rate/1.0e6 , rf_rate/1.0e6 );
    return(0);
}

int set_frequency(float_type f0)
{
    // Set center frequency
    if (LMS_SetLOFrequency(device, LMS_CH_RX, 0, f0) != 0) { error(); }
    float_type f1;
    if (LMS_GetLOFrequency(device, LMS_CH_RX, 0, &f1) != 0) { error(); }
    printf("\nCenter frequency: %f MHz\n", f1/1.0e6);
    return(0);
}

int decode_5cmd(int n, char *cbuffer)
{
    if ( (n <= 0) || (sendErrorFlag != 0) ) {
        if (running) {
            stop_running_device();
        }
        if (client_sockfd > 0) {
            close(client_sockfd);
        }
        client_sockfd = -1;
        return(-1);
    }
    int msg1 = cbuffer[0];
    if (msg1 != 4) {
        for (int i=0; i < n; i++) {
            fprintf(stdout, "%02x ", (0x00ff & cbuffer[i]));
        }
        if (n > 0) { fprintf(stdout, "\n"); }
    }
    for (int i=0; i < n; i+=5) {
        // decode 5 byte rtl_tcp command messages
        int msg  = cbuffer[i];
        int data = 0;
        for (int j=1;j<5;j++) {
            data = 256 * data + (0x00ff & cbuffer[i+j]);
        }
        if (msg == 1) {    // set frequency
            int f0 = data;
            fprintf(stdout, "setting frequency to: %d\n", f0);
            int m = set_frequency(f0);
            printf("set frequency status = %d\n", m);
        }
        if (msg == 2) {    // set sample rate
            int r = data;
            if (r != previousSRate) {
                fprintf(stdout, "setting samplerate to: %d\n", r);
                sampRate = r;
                int m = set_sample_rate(sampRate);
                printf("set samplerate status = %d\n", m);
                previousSRate = r;
            }
        }
        if (msg == 3) {    // other
            fprintf(stdout, "message = %d, data = %d\n", msg, data);
        }
        if (msg == 4) {             // gain
            if (   (sampleBits ==  8)
                || (sampleBits == 16) ) {
                // set gain ?
                float g1 = data; // data : in 10th dB's
                float g2 = 0.1 * (float)(data); // undo 10ths
                fprintf(stdout, "setting gain to: %f dB\n", g2);
                float g4 = g2 - 12.0; // ad hoc offset
                float g5 = pow(10.0, 0.1 * g4); // convert from dB
                gain0 = GAIN_SCALE * g5;        // 64.0 = nominal
                msg1 = msg;
                float  g8  =    0.25 * gain0; // GAIN_SCALE;
                fprintf(stdout, "8b  gain multiplier = %f\n", g8);
                float  g16 =    4.0  * gain0; // GAIN16;
                fprintf(stdout, "16b gain multiplier = %f\n", g16);
            }
        }
        if (msg > 4) {    // other
            fprintf(stdout, "message = %d, data = %d\n", msg, data);
        }
    }
}

int sendblockcount = 0;
uint8_t tmpBuf[8 * IQBUF_SIZE];

typedef union
{
    uint32_t i;
    float    f;
} Float32_t;

float rand_float_co()
{
    Float32_t x;
    x.i = 0x3f800000 | (rand() & 0x007fffff);
    return(x.f - 1.0f);
}

int sendcallback(float *p, int n)
{
    // float  *p =  NULL; // (float *)(context->samples);
    // int    n  =  0; // context->sample_count;
    int       sz ;
    
    if ((sendblockcount % 1000) == 0) {
        fprintf(stdout,"+"); fflush(stdout);
    }
    //
    if (p != NULL && n > 0) {
        // fwrite(p, 8, n, file);
        char      *dataBuffer     =  (char *)p;
        int     k         =  0;
        if (sampleBits ==  8) {
            float  g8  =  0.25 * gain0; // GAIN_SCALE;
            float rnd0A = rand_float_co();
            float rnd0B = rand_float_co();
            if (1) {            // new rounding
                for (int i=0; i<2*n; i++) {
                    float y = g8 * p[i];
                    // add triangular noise
                    // for noise filtered rounding
                    float rnd1 = rand_float_co(); // noise with pdf [0..1)
                    float r = rnd1 - (((i&1)==1) ? rnd0A : rnd0B);
                    y = y + r;
                    float ry = roundf(y);
                    acc_r += (y - ry);      // for future noise filter
                    k = ry + 128;
                    tmpBuf[i] = k;
                    if ((i&1)==1) {         // round I
                        rnd0A = rnd1;       // save for next iteration
                    } else {        // round Q
                        rnd0B = rnd1;       // save for next iteration
                    }
                }
            } else {    // previous rounding
                for (int i=0; i<2*n; i++) {
                    float x = g8 * p[i];
                    int   k = (int)roundf(x);
                    tmpBuf[i] = k + 128;   // 8-bit unsigned DC offset
                }
            }
            dataBuffer = (char *)(&tmpBuf[0]);
            sz = 2 * n;
        } else if (sampleBits == 16) {
            int16_t *tmp16ptr = (int16_t *)&tmpBuf[0];
            float  g16  =    4.0 * gain0; // GAIN16;
            for (int i=0; i<2*n; i++) {
                float x = g16 * p[i];
                int   k = (int)roundf(x);
                tmp16ptr[i] = k;
            }
            dataBuffer = (char *)(&tmpBuf[0]);
            sz = 4 * n;
        } else {
            sz = 8 * n;    // two 32-bit floats for IQ == 8 bytes
        }
        int send_sockfd = client_sockfd ;
#ifdef __APPLE__
        k = send(send_sockfd, dataBuffer, sz, 0);
#else
        k = send(send_sockfd, dataBuffer, sz, MSG_NOSIGNAL);
#endif
        if (k <= 0) { sendErrorFlag = -1; }
        if (k <  0) { return(-1); }
        totalSamples += n;
    }
    sendblockcount += 1;
    return(0);
}

// eof
