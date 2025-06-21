/*
 * Copyright 2018-2022 The MathWorks, Inc.
 *
 * File: MW_PX4_rtiostream_serial.c
 *
 * Abstract:
 *  Serial communications channel for PX4 Nuttx machine
 */

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <limits.h>
#include <assert.h>
#include "systemlib/err.h"

#include <termios.h>   /* POSIX terminal control definition*/
#include <unistd.h>    /*UNIX standard function definitions*/
#include <fcntl.h>     /* File control definition*/
#include <errno.h>     /*To get a description of error */
#include <sys/ioctl.h> /*For IOCTL Functions*/

#include "MW_target_hardware_resources.h"

#ifndef _rtiostream
extern "C" { /* sbcheck:ok:extern_c needed because of C++ compatibility for rtiostream.h*/
#include "rtiostream.h"
}
#define _rtiostream
#endif

#ifdef USE_MEXPRINTF
#include "mex.h"
#define printf mexPrintf
#endif

/***************** DEFINES ****************************************************/

#define N_SERIAL_PORTS 10    /* Number of com ports allowed to open */
#define MAX_COM_LEN 30       /* length of com port name */
#define MAX_COM_PREFIX_LEN 7 /* Full prefix is \\.\COM  */
#define TMP_BUF_SIZ (40)

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

#define COM_PORT_PREFIX "/dev/"
#define PORTNAME_DEFAULT_VAL "/dev/ttyS0"

#define BAUD_DEFAULT_VAL 57600
#define RTS_DTR_ENABLE_DEFAULT_VAL 0

#define MW_EXTSERIALPORT_STR(x) MW_EXTSERIALPORT_STR_EXPANDED(x)
#define MW_EXTSERIALPORT_STR_EXPANDED(x) #x

/* define a set of verbosity levels:
 *
 * 0: no verbose output
 * 1: basic verbose output
 * 2: extra verbose output */
typedef enum { VERBOSITY_LEVEL_0 = 0, VERBOSITY_LEVEL_1, VERBOSITY_LEVEL_2 } VerbosityLevel;
/* default verbosity */
#define DEFAULT_VERBOSITY VERBOSITY_LEVEL_0

#define READ_FILE_TIMEOUT 10000 /* 10 seconds */
#define WRITE_FILE_TIMEOUT                  \
    2000 /* 2 seconds. This should be large \
          * enough to send 512 bytes at the \
          * lowest possible baud rate. See  \
          * comments in serialDataSet for   \
          * more details                    \
          */

/***************** TYPEDEFS **************************************************/

typedef struct SerialCommsData_tag {
    int isInUse; /* flag to indicate whether an instance is in use */
    char port[MAX_COM_LEN];
    int serialHandle;
    uint32_T baud;
    VerbosityLevel verbosity;
    uint8_T rtsDtrEnable;
} SerialCommsData;

/**************** LOCAL DATA *************************************************/

/* All local data resides in the per instance SerialCommsData structures to
 * make sure each connection is completely independent.
 *
 * The static array will be deallocated when the shared library is unloaded.
 *
 * Using an array rather than a linked list allows us to have fast direct lookup
 * of SerialCommsData from streamID during calls to rtIOStreamSend/Recv */
static SerialCommsData SerialData[N_SERIAL_PORTS];
static bool dataInitialised = false;

/*
 * Refer to geck 700573 for resources used
 * All the parameters defined for Termios structure which
 * stores anything in relation to port attributes
 */
static struct termios sTermios;

/************** LOCAL FUNCTION PROTOTYPES ************************************/

static int serialDataSet(SerialCommsData* sd, const void* src, const size_t size, size_t* sizeSent);

static int serialDataGet(SerialCommsData* sd, char* dst, const size_t size, size_t* sizeRecvd);

static int serialDataPending(SerialCommsData* sd, int* outPending);

static void initSerialCommsDataStructure(void);

static SerialCommsData* getSerialData(int streamID);

/*************** LOCAL FUNCTIONS **********************************************/

/* Function: initSerialCommsDataStructure ==========================================================
 * Abstract: Initializes the data
 * structures to zero.
 */
static void initSerialCommsDataStructure(void) {
    if (!dataInitialised) {
        memset(&SerialData, 0, sizeof(SerialData));
        dataInitialised = true;
    }
}

/* Function: initTermios ===========================================================================
 * Abstract:
 *  Initializes the control settings for a UNIX serial communications device.
 */
static int initTermios(int fd, uint32_T baud) {
    speed_t baudC = B0;

    if (tcgetattr(fd, &sTermios) != 0) { /*Get the attributes set by default*/
        printf("ERROR: Failed to get existing serial port configuration\n");
        return RTIOSTREAM_ERROR;
    }

    /* ---------- Serial Port Config ------- */
    sTermios.c_cflag |= (CLOCAL     /*To not become port 'owner'*/
                         | CREAD);  /*Allow reading of incoming data*/
    sTermios.c_cflag &= ~(CSTOPB    /*To have one stop bit */
                          | CSIZE); /*Mask Character size bites */
    sTermios.c_cflag &= ~(PARENB);  /*Disable Parity Check*/
    sTermios.c_cflag |= CS8;        /*Character size at 8 bits */

    sTermios.c_lflag &= ~(ICANON     /*To have raw output*/
                          | ECHO     /*Disable input character echo*/
                          | ECHONL   /*Disable echoing NL character */
                          | ISIG     /*Disable SIGINTR, SIGSUSP, SIGDSUSP, and SIGQUIT signals*/
                          | IEXTEN); /* implementation-defined input processing */

    sTermios.c_cc[VMIN] = 0;  /*To not have read operation dependent
                               * on number of characters received*/
    sTermios.c_cc[VTIME] = 0; /* Does not have a timeout*/

    sTermios.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL |
                          IXON); /*Disable Software Flow Control*/

    sTermios.c_oflag &= ~OPOST; /*Disable Post processing of output*/

    /*Determine Actual Baud Value to setup port*/
    switch (baud) {
    case 50:
        baudC = B50;
        break;
    case 75:
        baudC = B75;
        break;
    case 110:
        baudC = B110;
        break;
    case 134:
        baudC = B134;
        break;
    case 150:
        baudC = B150;
        break;
    case 200:
        baudC = B200;
        break;
    case 300:
        baudC = B300;
        break;
    case 600:
        baudC = B600;
        break;
    case 1200:
        baudC = B1200;
        break;
    case 1800:
        baudC = B1800;
        break;
    case 2400:
        baudC = B2400;
        break;
    case 4800:
        baudC = B4800;
        break;
    case 9600:
        baudC = B9600;
        break;
    case 19200:
        baudC = B19200;
        break;
    case 38400:
        baudC = B38400;
        break;
    case 57600:
        baudC = B57600;
        break;
    case 115200:
        baudC = B115200;
        break;
    case 230400:
        baudC = B230400;
        break;
    case 460800:
        baudC = B460800;
        break;
    case 500000:
        baudC = B500000;
        break;
    case 576000:
        baudC = B576000;
        break;
    case 921600:
        baudC = B921600;
        break;
    case 1000000:
        baudC = B1000000;
        break;
    case 1152000:
        baudC = B1152000;
        break;
    case 1500000:
        baudC = B1500000;
        break;
    case 2000000:
        baudC = B2000000;
        break;
    case 2500000:
        baudC = B2500000;
        break;
    case 3000000:
        baudC = B3000000;
        break;
    default: {
        printf("ERROR: Invalid baud rate supplied (%d)\n", baud);
        return RTIOSTREAM_ERROR;
    }
    }

    if (cfsetispeed(&sTermios, baudC) != 0) { /*Set incoming Baud Rate*/
        printf("ERROR: Failed to set input baud rate to %d\n", baud);
        return RTIOSTREAM_ERROR;
    }

    if (cfsetospeed(&sTermios, baudC) != 0) { /*Set outgoing Baud Rate*/
        printf("ERROR: Failed to set output baud rate to %d\n", baud);
        return RTIOSTREAM_ERROR;
    }

    /************Didn't Find Equivalent for following***************/
    /* Write timeout not available for UNIX*/
    /* No equivalent for XonLim and XoffLim or EvtChar*/
    /***************************************************************/

    /* -------- Serial Port Config END---- */

    if (tcsetattr(fd, TCSANOW, &sTermios) != 0) {
        printf("ERROR: Failed to configure serial port\n");
        return RTIOSTREAM_ERROR;
    }
    return RTIOSTREAM_NO_ERROR;
}

/* Function: serialDataPending =====================================================================
 * Abstract:
 *  Returns true, via the 'pending' arg, if data is pending on the comm line.
 *  Returns false otherwise.
 *
 *  RTIOSTREAM_NO_ERROR is returned on success, RTIOSTREAM_ERROR on failure.
 */
static int serialDataPending(SerialCommsData* sd, int* outPending) {
    int retVal = RTIOSTREAM_NO_ERROR;

    int temp = 0;

    assert(sd->serialHandle >= 0);

    /* Find out how much data is available. */
    temp = ioctl(sd->serialHandle, FIONREAD, (unsigned long)outPending);
    if (temp < 0) {
        retVal = RTIOSTREAM_ERROR;
        *outPending = 0;
    }

    return retVal;
} /* end serialDataPending */


/* Function: serialDataGet =========================================================================
 * Abstract:
 *  Attempts to gets the specified number of bytes from the specified serial.
 *  The number of bytes read is returned via the 'sizeRecvd' parameter.
 *  RTIOSTREAM_NO_ERROR is returned on success, RTIOSTREAM_ERROR is returned on
 *  failure.
 *
 * NOTES:
 *  o it is not an error for 'sizeRecvd' to be returned as 0
 *  o this function waits for at most READ_FILE_TIMEOUT
 */
static int serialDataGet(SerialCommsData* sd, char* dst, const size_t size, size_t* sizeRecvd) {
    static const char* fnName = "serialDataGet:";
    int retVal = RTIOSTREAM_NO_ERROR;
    int avail = 0;

    ssize_t sizeRecvdTemp = 0;
    assert(sd->serialHandle >= 0);

    *sizeRecvd = 0;
    if (size == 0) {
        /* return immediately if caller requested to read 0 bytes */
        return retVal;
    }
    int retPoll = 1;
#if defined(PX4_CONNECTEDIO) || defined(PIL)
    struct pollfd fds[1] = {};
    fds[0].fd = sd->serialHandle;
    fds[0].events = POLLIN;
    const int timeout = 100;// in ms
    retPoll = poll(&fds[0], 1, timeout);
#endif
    if (retPoll>0) {
        retVal = serialDataPending(sd, &avail);
        if (retVal == RTIOSTREAM_ERROR || avail == 0) {
            if (retVal == RTIOSTREAM_ERROR) {
                printf("%s Pending returned RTIOSTREAM_ERROR\n", fnName);
            }
            return retVal;
        }

        sizeRecvdTemp = read(sd->serialHandle, dst, size);
        if (sizeRecvdTemp < 0) /*Error Condition check*/
    
        {
            printf("%s ReadFile returned ERROR\n", fnName);
            retVal = RTIOSTREAM_ERROR;
            return retVal;
        }
        
        *sizeRecvd = (size_t)sizeRecvdTemp;
    } else {
        retVal = RTIOSTREAM_ERROR;
    }

    return retVal;
} /* end serialDataGet */


/* Function: serialDataSet =========================================================================
 * Abstract:
 *  Utility function to send data via the specified serial
 */
static int serialDataSet(SerialCommsData* sd,
                         const void* src,
                         const size_t size,
                         size_t* sizeSent) {
    static const char* fnName = "serialDataSet:";
    int retVal = RTIOSTREAM_NO_ERROR;

    /* G861098: workaround an issue for some serial drivers (i.e.
     * Arduino and C2000 USB based serial drivers). When the send operation
     * times out for these drivers, they return that 0 bytes were sent.
     * However, we know this is not true (i.e. we know they have actually
     * sent non-zero bytes). The workaround is to limit the max size that
     * one can send so it can be successfully sent with the
     * WRITE_FILE_TIMEOUT of 2 seconds. The client code (PIL/ExtMode)
     * is responsible to loop to send the whole amount of data.
     */
    const size_t maxAllowedTransferSize = 512; /* in bytes */
    const size_t transferAmount = MIN(maxAllowedTransferSize, size);

    ssize_t sizeSentTemp = 0;
    *sizeSent = 0;

    if (size == 0) {
        /* return immediately if caller requested to write 0 bytes */
        return retVal;
    }

    sizeSentTemp = write(sd->serialHandle, src, transferAmount);

    /*G807280 Check the error number. If resource is "Temporarily Unavailable"
     * don't error, but let caller application deal with zero data*/
    if ((sizeSentTemp < 0) && ((errno == EAGAIN) || (errno == EWOULDBLOCK))) {
        sizeSentTemp = 0;
    }

    if (sizeSentTemp < 0) {
        printf("%s WriteFile returned ERROR\n", fnName);
        retVal = RTIOSTREAM_ERROR;
    }

    *sizeSent = (size_t)sizeSentTemp;

    return retVal;
}

/* Function: serialDataFlush =======================================================================
 * Abstract:
 *  Utility function to flush a port
 */
static int serialDataFlush(SerialCommsData* sd) {
    char tmpBuf[TMP_BUF_SIZ];
    size_t numRecvd = 0;
    int pending = 0;
    int error;
    static const char* fnName = "serialDataFlush:";

    do {
        error = serialDataPending(sd, &pending);
        if ((pending > 0) && (error == RTIOSTREAM_NO_ERROR)) {
            if (sd->verbosity) {
                printf("serialDataFlush: pending = %d\n", pending);
            }
            error = serialDataGet(sd, tmpBuf, sizeof(tmpBuf), &numRecvd);
            if (sd->verbosity) {
                size_t currElement;
                printf("serialDataFlush: sizeRecvd = %lu: ", (unsigned long)numRecvd);
                for (currElement = 0; currElement < numRecvd; currElement++) {
                    printf("%u ", (unsigned char)tmpBuf[currElement]);
                }
                putchar('\n'); // printf("\n");
            }
        }
    } while ((pending > 0) && (error == RTIOSTREAM_NO_ERROR) && numRecvd > 0);
    if (error == RTIOSTREAM_ERROR) {
        printf("%s Flushing returned RTIOSTREAM_ERROR\n", fnName);
    }
    return error;
}

static int serialPortOpenInitialize(SerialCommsData* sd,
                                    const int streamID,
                                    int argc,
                                    void* argv[]) {
    int errorCode = RTIOSTREAM_NO_ERROR;
    int serialHandle = -1;
    int closeFile = false;
#ifndef PIL    
    int result;
#endif    
    char fullPortName[MAX_COM_LEN] = "";

    /* Setting PX4 baud rate and port */
    sd->baud = (uint32_T)MW_PX4_EXTMODE_BAUD_RATE;
    strcat(fullPortName, MW_PX4_EXTMODE_HWPORT);
    strcpy(sd->port, fullPortName);
    sd->verbosity = DEFAULT_VERBOSITY;
    sd->rtsDtrEnable = RTS_DTR_ENABLE_DEFAULT_VAL;
    printf("External mode : Serial port baud rate: %d\n", sd->baud);
    printf("External mode : Serial port Number: %s\n", sd->port);
    fflush(stdout);

    if (sd->verbosity) {
        printf("rtIOStreamOpen (connection id %d): port %s, baud %lu\n", streamID, sd->port,
               (unsigned long)sd->baud);
    }

    PX4_INFO("Opening serial port '%s' at baud %lu \n", sd->port, (unsigned long)sd->baud);
    int retries = 0;
    while (retries < 15) {

        serialHandle = open(sd->port, O_RDWR             /*Enable Read and Write Operations*/
                                          | O_NOCTTY     /*Not controlling flag specified*/
                                          | O_NONBLOCK); /*Make the port non -blocking*/

        if (serialHandle != -1) {
            break;
        }
        // sleep a bit and retry. There seems to be a NuttX bug
        // that can cause ttyACM0 to not be available immediately,
        // but a small delay can fix it
        usleep(50000);
        retries++;
    }
    PX4_INFO("Opened serial port '%s' RETURN:  %d \n", sd->port, serialHandle);
    if (serialHandle == -1) { /*-1 returned on error by OPEN*/
        printf("Failed to open serial port '%s'. The reported error is %d: %s. ", sd->port, errno,
               strerror(errno));

        printf(
            "Possible causes "
            "are:\n    a) The target is not powered on. \n    "
            "b) The specified port number is not correct. \n    "
            "c) The specified bit rate is not "
            "supported by the host machine.\n    d) The previous "
            "connection was not closed correctly. "
            "If this is the case, you may need to re-start MATLAB.\n");
        fflush(stdout);

        errorCode = RTIOSTREAM_ERROR;
        goto EXIT_POINT;
    }

    sd->serialHandle = serialHandle;

    errorCode = initTermios(sd->serialHandle, sd->baud);
    if (errorCode == RTIOSTREAM_ERROR) {
        closeFile = true;
        goto EXIT_POINT;
    }
    /*UNIX has this done with setting of termios data
     * structure in initTermios*/

    /* g2099552 - to avoid PIL start packet getting flushed
     * when MAVLink is enabled in start up and disabled in px4_simulink_app
     * the serial open is blocked till a response from host (this need to be investigated) and hence
     * the PIL start packet([ 1 5 0 0 0 0 0]) is getting flushed with the following code. The
     * external mode takes more time to send the connect packet, hence not impacted */
#ifndef PIL
    result = serialDataFlush(sd); /* clear out old data on the port */

    if (result == RTIOSTREAM_ERROR) {
        printf("serialDataFlush failed\n");
        errorCode = RTIOSTREAM_ERROR;
        closeFile = true;
        goto EXIT_POINT;
    }
#endif



EXIT_POINT:
    if (closeFile) {
        close(serialHandle);
    }
    return errorCode;
}

/* Function: getSerialData =========================================================================
 * Abstract:
 *  Retrieves a SerialCommsData instance given its streamID.
 *
 * NOTE: An invalid streamID will lead to a NULL pointer being returned
 */
static SerialCommsData* getSerialData(int streamID) {
    /* return NULL for invalid or uninitialized streamIDs */
    SerialCommsData* sd = NULL;
    if ((streamID >= 0) && (streamID < N_SERIAL_PORTS)) {
        if (SerialData[streamID].isInUse) {
            sd = &SerialData[streamID];
        }
    }
    return sd;
}

/***************** VISIBLE FUNCTIONS ******************************************/

/* Function: rtIOStreamOpen ========================================================================
 * Abstract:
 *  Open the connection with the target.
 */
int rtIOStreamOpen(int argc, void* argv[]) {
    int streamID;
    int errorCode;
    SerialCommsData* sd;

    initSerialCommsDataStructure();

    /* Find any free ports */
    for (streamID = 0; streamID < N_SERIAL_PORTS; streamID++) {
        if (!SerialData[streamID].isInUse) {
            break;
        }
    }
    if (streamID == N_SERIAL_PORTS) {
        printf("All %d elements of the SerialCommsData structure are already in use.",
               N_SERIAL_PORTS);
        return RTIOSTREAM_ERROR;
    }

    sd = &SerialData[streamID];

    errorCode = serialPortOpenInitialize(sd, streamID, argc, argv);

    if (errorCode == RTIOSTREAM_NO_ERROR) {
        /* flag instance as in use */
        SerialData[streamID].isInUse = 1;
        return streamID;
    } else {
        /* flag error */
        return RTIOSTREAM_ERROR;
    }
}



/* Function: rtIOStreamSend ========================================================================
 * Abstract:
 *  Sends the specified number of bytes on the comm line. Returns the number of
 *  bytes sent (if successful) or a negative value if an error occurred. As long
 *  as an error does not occur, this function is guaranteed to set the requested
 *  number of bytes; the function blocks if the TCP/IP send buffer doesn't have
 *  room for all of the data to be sent
 */
int rtIOStreamSend(int streamID, const void* const src, size_t size, size_t* sizeSent) {
    int retVal;
    SerialCommsData* sd = getSerialData(streamID);
    if (sd == NULL) {
        retVal = RTIOSTREAM_ERROR;
        return retVal;
    }
    retVal = serialDataSet(sd, src, size, sizeSent);
    if (sd->verbosity) {
        printf("rtIOStreamSend (connection id %d): size = %lu, sizeSent = %lu", streamID,
               (unsigned long)size, (unsigned long)*sizeSent);
        if (sd->verbosity >= VERBOSITY_LEVEL_2) {
            size_t currElement;
            printf(": ");
            for (currElement = 0; currElement < *sizeSent; currElement++) {
                printf("%u ", ((const unsigned char*)src)[currElement]);
            }
        }
        putchar('\n'); // printf("\n");
    }
    return retVal;
}


/* Function: rtIOStreamRecv ========================================================================
 * Abstract: receive data
 *
 */
int rtIOStreamRecv(int streamID, void* const dst, size_t size, size_t* sizeRecvd) {
    int retVal = RTIOSTREAM_NO_ERROR;
    SerialCommsData* sd = getSerialData(streamID);
    if (sd == NULL) {
        retVal = RTIOSTREAM_ERROR;
        return retVal;
    }
    retVal = serialDataGet(sd, (char*)dst, size, sizeRecvd);
    if (sd->verbosity) {
        printf("rtIOStreamRecv (connection id %d): size = %lu, sizeRecvd = %lu", streamID,
               (unsigned long)size, (unsigned long)*sizeRecvd);
        if (sd->verbosity >= VERBOSITY_LEVEL_2) {
            size_t currElement;
            printf(": ");
            for (currElement = 0; currElement < *sizeRecvd; currElement++) {
                printf("%u ", ((unsigned char*)dst)[currElement]);
            }
        }
        putchar('\n'); // printf("\n");
    }
    return retVal;
}


/* Function: rtIOStreamClose =======================================================================
 * Abstract: close the connection.
 *
 */
int rtIOStreamClose(int streamID) {
    int retVal = RTIOSTREAM_NO_ERROR;
    SerialCommsData* sd = getSerialData(streamID);
    if (sd == NULL) {
        retVal = RTIOSTREAM_ERROR;
        return retVal;
    }
    retVal = serialDataFlush(sd);
    close(sd->serialHandle);
    sd->serialHandle = -1;
    /*because unlike Windows which uses a pointer it uses an
     * int File descriptor which can be 0*/
    /* clear in use flag */
    sd->isInUse = 0;

    if (sd->verbosity) {
        printf("rtIOStreamClose (connection id %d)\n", streamID);
    }

    PX4_INFO("rtIOStreamClose (connection id %d)\n", streamID);


    return retVal;
}
