/* Copyright 2020-2023 The MathWorks, Inc. */

#include "MW_PX4_SCI.h"
#include "MW_target_hardware_resources.h"

#if (defined(MATLAB_MEX_FILE) || defined(RSIM_PARAMETER_LOADING) || defined(RSIM_WITH_SL_SOLVER))

MW_Handle_Type MW_SCI_Open(void* SCIModule, uint8_T isString, uint32_T RxPin, uint32_T TxPin) {
    return 0;
}

MW_SCI_Status_Type MW_SCI_ConfigureHardwareFlowControl(
    MW_Handle_Type SCIModuleHandle,
    MW_SCI_HardwareFlowControl_Type HardwareFlowControl,
    uint32_T RtsDtrPin,
    uint32_T CtsDtsPin) {
    return MW_SCI_SUCCESS;
}

MW_SCI_Status_Type MW_SCI_SetBaudrate(MW_Handle_Type SCIModuleHandle, uint32_T Baudrate) {
    return MW_SCI_SUCCESS;
}

MW_SCI_Status_Type MW_SCI_SetFrameFormat(MW_Handle_Type SCIModuleHandle,
                                         uint8_T DataBitsLength,
                                         MW_SCI_Parity_Type Parity,
                                         MW_SCI_StopBits_Type StopBits) {
    return MW_SCI_SUCCESS;
}

MW_SCI_Status_Type MW_SCI_Receive(MW_Handle_Type SCIModuleHandle,
                                  uint8_T* RxDataPtr,
                                  uint32_T RxDataLength) {
    return MW_SCI_SUCCESS;
}

MW_SCI_Status_Type MW_SCI_Transmit(MW_Handle_Type SCIModuleHandle,
                                   uint8_T* TxDataPtr,
                                   uint32_T TxDataLength) {
    return MW_SCI_SUCCESS;
}

MW_SCI_Status_Type MW_SCI_GetStatus(MW_Handle_Type SCIModuleHandle) {
    return MW_SCI_SUCCESS;
}

MW_SCI_Status_Type MW_SCI_SendBreak(MW_Handle_Type SCIModuleHandle) {
    return MW_SCI_SUCCESS;
}

void MW_SCI_Close(MW_Handle_Type SCIModuleHandle) {}

MW_SCI_Status_Type MW_SCI_GetDataBytesAvailable(MW_Handle_Type SCIModuleHandle,
                                                bool blockingMode,
                                                void* size,
                                                int32_T timeout) {
    return 0;
}
#else

#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <stdbool.h>
#include <stdlib.h>
#include <time.h>

// time-out using select( )
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <sys/time.h>

#ifndef MW_TTYACM0_BAUDRATE
#define MW_TTYACM0_BAUDRATE 0
#endif

#ifndef MW_TTYS0_BAUDRATE
#define MW_TTYS0_BAUDRATE 0
#endif

#ifndef MW_TTYS1_BAUDRATE
#define MW_TTYS1_BAUDRATE 0
#endif

#ifndef MW_TTYS2_BAUDRATE
#define MW_TTYS2_BAUDRATE 0
#endif

#ifndef MW_TTYS3_BAUDRATE
#define MW_TTYS3_BAUDRATE 0
#endif

#ifndef MW_TTYS4_BAUDRATE
#define MW_TTYS4_BAUDRATE 0
#endif

#ifndef MW_TTYS5_BAUDRATE
#define MW_TTYS5_BAUDRATE 0
#endif

#ifndef MW_TTYS6_BAUDRATE
#define MW_TTYS6_BAUDRATE 0
#endif

#ifndef MW_TTYS7_BAUDRATE
#define MW_TTYS7_BAUDRATE 0
#endif

#ifndef MW_TTYACM0_PARITY
#define MW_TTYACM0_PARITY 0
#endif

#ifndef MW_TTYS0_PARITY
#define MW_TTYS0_PARITY 0
#endif

#ifndef MW_TTYS1_PARITY
#define MW_TTYS1_PARITY 0
#endif

#ifndef MW_TTYS2_PARITY
#define MW_TTYS2_PARITY 0
#endif

#ifndef MW_TTYS3_PARITY
#define MW_TTYS3_PARITY 0
#endif

#ifndef MW_TTYS4_PARITY
#define MW_TTYS4_PARITY 0
#endif

#ifndef MW_TTYS5_PARITY
#define MW_TTYS5_PARITY 0
#endif

#ifndef MW_TTYS6_PARITY
#define MW_TTYS6_PARITY 0
#endif

#ifndef MW_TTYACM0_STOPBIT
#define MW_TTYACM0_STOPBIT 0
#endif

#ifndef MW_TTYS0_STOPBIT
#define MW_TTYS0_STOPBIT 0
#endif

#ifndef MW_TTYS1_STOPBIT
#define MW_TTYS1_STOPBIT 0
#endif

#ifndef MW_TTYS2_STOPBIT
#define MW_TTYS2_STOPBIT 0
#endif

#ifndef MW_TTYS3_STOPBIT
#define MW_TTYS3_STOPBIT 0
#endif

#ifndef MW_TTYS4_STOPBIT
#define MW_TTYS4_STOPBIT 0
#endif

#ifndef MW_TTYS5_STOPBIT
#define MW_TTYS5_STOPBIT 0
#endif

#ifndef MW_TTYS6_STOPBIT
#define MW_TTYS6_STOPBIT 0
#endif

#ifndef MW_TTYACM0HWFLOWCONTROL_CHECKBOX
#define MW_TTYACM0HWFLOWCONTROL_CHECKBOX 0
#endif

#ifndef MW_TTYS0HWFLOWCONTROL_CHECKBOX
#define MW_TTYS0HWFLOWCONTROL_CHECKBOX 0
#endif

#ifndef MW_TTYS1HWFLOWCONTROL_CHECKBOX
#define MW_TTYS1HWFLOWCONTROL_CHECKBOX 0
#endif

#ifndef MW_TTYS2HWFLOWCONTROL_CHECKBOX
#define MW_TTYS2HWFLOWCONTROL_CHECKBOX 0
#endif

#ifndef MW_TTYS3HWFLOWCONTROL_CHECKBOX
#define MW_TTYS3HWFLOWCONTROL_CHECKBOX 0
#endif

#ifndef MW_TTYS4HWFLOWCONTROL_CHECKBOX
#define MW_TTYS4HWFLOWCONTROL_CHECKBOX 0
#endif

#ifndef MW_TTYS5HWFLOWCONTROL_CHECKBOX
#define MW_TTYS5HWFLOWCONTROL_CHECKBOX 0
#endif

#ifndef MW_TTYS6HWFLOWCONTROL_CHECKBOX
#define MW_TTYS6HWFLOWCONTROL_CHECKBOX 0
#endif


/* Local defines */
#define NUM_MAX_SCI_DEV (10) /*a random choice - no limit on serial devices*/
#define MAX_DEV_NAME (64)
#define MICROS 1000000L
#define NANOS 1000000000L

#define DEBUG 0
struct SCI_dev_t {
    int fd;
    char* portname;
    uint32_T baudrate;
    uint8_T databits;
    MW_SCI_Parity_Type parity;
    MW_SCI_StopBits_Type stopbits;
    uint8_T busy;
    SCI_dev_t()
        : fd(-1)
        , portname(NULL)
        , baudrate(9600)
        , databits(8)
        , parity(MW_SCI_PARITY_NONE)
        , stopbits(MW_SCI_STOPBITS_1)
        , busy(0) {}
};

static SCI_dev_t sciDev[NUM_MAX_SCI_DEV];

static uint32_T AllBaudrates[] = {50,     75,      110,     134,     150,     200,     300,
                                  600,    1200,    1800,    2400,    4800,    9600,    19200,
                                  38400,  57600,   115200,  230400,  460800,  500000,  576000,
                                  921600, 1000000, 1152000, 1500000, 2000000, 2500000, 3000000};

/* Open SERIAL channel*/
static int SERIAL_open(const char* port) {
    int fd;
    int retries = 0;
    /* O_NDELAY: disregard DCD signal line state
     * O_NOCTTY: we don't want to be the controlling terminal */
    fflush(stdout);
    while (retries < 15) {
        fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd != -1) {
            break;
        }
        // sleep a bit and retry. There seems to be a NuttX bug
        // that can cause ttyACM0 to not be available immediately,
        // but a small delay can fix it
        usleep(50000);
        retries++;
    }
    if (fd == -1) {
        perror("SERIAL_open/open");
    }

    /*flush both data received but not read, and data written but not transmitted.*/
    tcflush(fd, TCIOFLUSH);
    return fd;
}

/* Return device ID given serial port name */
int getCurrSciDev(const char* name) {
    int i;

    for (i = NUM_MAX_SCI_DEV - 1; i > -1; i--) {
        if ((name != NULL) && (sciDev[i].portname != NULL) &&
            (strncmp(name, sciDev[i].portname, MAX_DEV_NAME) == 0)) {
            break;
        }
    }

    return i;
}
/* Allocate device*/
int Serial_alloc(const char* name) {
    int i;

    for (i = 0; i < NUM_MAX_SCI_DEV; i++) {
        if (sciDev[i].portname == NULL) {
            break;
        }
    }
    if (i >= NUM_MAX_SCI_DEV) {
        fprintf(stderr, "Cannot allocate a new device for %s: [%d]\n", name, i);
        return -1;
    }
    sciDev[i].portname = strndup(name, MAX_DEV_NAME);

    return i;
}

/* Initialize a SCI */
MW_Handle_Type MW_SCI_Open(void* SCIModule,
                           uint8_T isString,
                           uint32_T RxPin, /* Not used */
                           uint32_T TxPin) /*Not Used*/
{
    MW_Handle_Type SCIHandle = (MW_Handle_Type)NULL;
    char* port;
    int currSciDev;

    /* Check parameters */
    if (0 == isString) {
        fprintf(stderr, "Only string as SCI Module name is supported.\n");
        exit(-1);
    } else {
        /* Initialize the SCI Module*/
        port = (char*)SCIModule;
#ifdef PX4_CONNECTEDIO
        /* For IO, the port name will be sent from host PC. IO wrapper does not use the length of
         * port name and pass it as the last part in the packet. While reading port name in target,
         * everything remaining in the buffer is read as port name. Because of this, sometimes extra
         * junk bytes get added to the port name. Since all the supported serial ports are of length
         * 10 (eg: /dev/ttyS1,/dev/ttyS6) manually ending the port name after 10 characters. note:
         * /dev/ttyACM0 is not supported for serial blocks as it is conflicting with Connected I/O.
         */
        port[10] = '\0';
#endif
#if DEBUG
        printf("In MW_SCI_Open. Port = %s\n", port);
#endif
        currSciDev = getCurrSciDev(port);
        fprintf(stdout, "INIT: sciDevNo = %d\n", currSciDev);
        if (currSciDev == -1) {
            currSciDev = Serial_alloc(port);
            fprintf(stdout, "ALLOC: devNo = %d\n", currSciDev);
            if (currSciDev == -1) {
                fprintf(stderr, "Error opening Serial bus (SERIAL_init/Alloc).\n");
                exit(-1);
            }
        }

        if (sciDev[currSciDev].fd < 0) {
            sciDev[currSciDev].fd = SERIAL_open(port);
            if (sciDev[currSciDev].fd < 0) {
                fprintf(stderr, "Error opening Serial bus (SERIAL_init/Open).\n");
                exit(-1);
            }
        }
        SCIHandle = (MW_Handle_Type)&sciDev[currSciDev];
    }
    return SCIHandle;
}

/* Set the SCI bus speed */
MW_SCI_Status_Type MW_SCI_SetBaudrate(MW_Handle_Type SCIModuleHandle, uint32_T Baudrate) {
    SCI_dev_t* sci;
    struct termios options;
    speed_t optBaud;

    if (NULL != (void*)SCIModuleHandle) {
        sci = (SCI_dev_t*)SCIModuleHandle;

#ifndef PX4_CONNECTEDIO
        if (strcmp(sci->portname, "/dev/ttyACM0") == 0) {
            Baudrate = AllBaudrates[MW_TTYACM0_BAUDRATE];
#if DEBUG
            PX4_INFO("Inside MW_SCI_SetBaudrate strcmp. Port name = %s.Baudrate = %u\n.",
                     sci->portname, Baudrate);
#endif
        } else if (strcmp(sci->portname, "/dev/ttyS0") == 0) {
            Baudrate = AllBaudrates[MW_TTYS0_BAUDRATE];
        } else if (strcmp(sci->portname, "/dev/ttyS1") == 0) {
            Baudrate = AllBaudrates[MW_TTYS1_BAUDRATE];
        } else if (strcmp(sci->portname, "/dev/ttyS2") == 0) {
            Baudrate = AllBaudrates[MW_TTYS2_BAUDRATE];
#if DEBUG
            PX4_INFO("Inside MW_SCI_SetBaudrate strcmp. Port name = %s.Baudrate = %u\n.",
                     sci->portname, Baudrate);
#endif
        } else if (strcmp(sci->portname, "/dev/ttyS3") == 0) {
            Baudrate = AllBaudrates[MW_TTYS3_BAUDRATE];
        } else if (strcmp(sci->portname, "/dev/ttyS4") == 0) {
            Baudrate = AllBaudrates[MW_TTYS4_BAUDRATE];
        } else if (strcmp(sci->portname, "/dev/ttyS5") == 0) {
            Baudrate = AllBaudrates[MW_TTYS5_BAUDRATE];
        } else if (strcmp(sci->portname, "/dev/ttyS6") == 0) {
            Baudrate = AllBaudrates[MW_TTYS6_BAUDRATE];
        } else if (strcmp(sci->portname, "/dev/ttyS7") == 0) {
            Baudrate = AllBaudrates[MW_TTYS7_BAUDRATE];
#if DEBUG
            PX4_INFO("Inside MW_SCI_SetBaudrate strcmp. Port name = %s.Baudrate = %u\n.",
                     sci->portname, Baudrate);
#endif
        } else {
            perror("SERIAL_SetBaudrate");
            return MW_SCI_BUS_ERROR;
        }
#endif

#if DEBUG
        printf("Inside MW_SCI_SetBaudrate. Port name = %s. Baudrate = %u\n.", sci->portname,
               Baudrate);
#endif
        /* Set parameters of the serial connection*/
        //         optBaud = Baudrate;
        switch (Baudrate) {
        case 50:
            optBaud = B50;
            break;
        case 75:
            optBaud = B75;
            break;
        case 110:
            optBaud = B110;
            break;
        case 134:
            optBaud = B134;
            break;
        case 150:
            optBaud = B150;
            break;
        case 200:
            optBaud = B200;
            break;
        case 300:
            optBaud = B300;
            break;
        case 600:
            optBaud = B600;
            break;
        case 1200:
            optBaud = B1200;
            break;
        case 1800:
            optBaud = B1800;
            break;
        case 2400:
            optBaud = B2400;
            break;
        case 4800:
            optBaud = B4800;
            break;
        case 9600:
            optBaud = B9600;
            break;
        case 19200:
            optBaud = B19200;
            break;
        case 38400:
            optBaud = B38400;
            break;
        case 57600:
            optBaud = B57600;
            break;
        case 115200:
            optBaud = B115200;
            break;
        case 230400:
            optBaud = B230400;
            break;
        case 460800:
            optBaud = B460800;
            break;
        case 500000:
            optBaud = B500000;
            break;
        case 576000:
            optBaud = B576000;
            break;
        case 921600:
            optBaud = B921600;
            break;
        case 1000000:
            optBaud = B1000000;
            break;
        case 1152000:
            optBaud = B1152000;
            break;
        case 1500000:
            optBaud = B1500000;
            break;
        case 2000000:
            optBaud = B2000000;
            break;
        case 2500000:
            optBaud = B2500000;
            break;
        case 3000000:
            optBaud = B3000000;
            break;
        default:
            perror("SERIAL_SetBaudrate");
            return MW_SCI_BUS_ERROR;
        }

        if (tcgetattr(sci->fd, &options) < 0) {
            perror("SERIAL_GetAttribute");
            return MW_SCI_BUS_ERROR;
        }

        /* Enable the receiver and set local mode*/
        options.c_cflag |= (CLOCAL    /*To not become port 'owner'*/
                            | CREAD); /*Allow reading of incoming data*/
        options.c_iflag &= ~(IXON     /*Disable Software Flow Control*/
                             | IXOFF | IXANY);

        options.c_lflag &= ~(ICANON   /*To have raw output*/
                             | ECHO   /*Disable input character echo*/
                             | ECHOE  /*Disable echo character erase*/
                             | ISIG); /*Disable SIGINTR, SIGSUSP, SIGDSUSP,
                                       * and SIGQUIT signals*/

        /* Set character read options*/
        options.c_cc[VMIN] = 0;
        options.c_cc[VTIME] = 0; /*This is a completely non-blocking read -*/
        /* Local options. Configure for RAW input*/

        options.c_oflag &= ~OPOST; /*Disable Post processing of output*/

        /* Set baud rate*/
        if (cfsetispeed(&options, optBaud) < 0) {
            perror("SERIAL_Set_ip_Speed");
            return MW_SCI_BUS_ERROR;
        }
        if (cfsetospeed(&options, optBaud) < 0) {
            perror("SERIAL_Set_op_Speed");
            return MW_SCI_BUS_ERROR;
        }

        /* Set attributes*/
        if (tcsetattr(sci->fd, TCSANOW, &options) < 0) {
            perror("SERIAL_SetAttribute");
            return MW_SCI_BUS_ERROR;
        } else {
            printf("Serial Port Opened. fd = %d. BaudRate = %lu\n", sci->fd, optBaud);
        }
        /*flush both data received but not read, and data written but not transmitted.*/
        tcflush(sci->fd, TCIOFLUSH);
    }

    return MW_SCI_SUCCESS;
}

/* Set SCI frame format */
MW_SCI_Status_Type MW_SCI_SetFrameFormat(MW_Handle_Type SCIModuleHandle,
                                         uint8_T DataBitsLength,
                                         MW_SCI_Parity_Type Parity,
                                         MW_SCI_StopBits_Type StopBits) {
    SCI_dev_t* sci;
    struct termios options;

    if (NULL != (void*)SCIModuleHandle) {

        sci = (SCI_dev_t*)SCIModuleHandle;
        if (tcgetattr(sci->fd, &options) < 0) {
            perror("SERIAL_GetAttribute");
            return MW_SCI_BUS_ERROR;
        }

        /* Set data bits*/
        options.c_cflag &= ~CSIZE;

        switch (DataBitsLength) {
        case 5:
            options.c_cflag |= CS5;
            break;
        case 6:
            options.c_cflag |= CS6;
            break;
        case 7:
            options.c_cflag |= CS7;
            break;
        case 8:
            options.c_cflag |= CS8;
            break;
        default:
            perror("SERIAL_SetFrameFormat/DataBitsLength");
            return MW_SCI_BUS_ERROR;
        }

#ifndef PX4_CONNECTEDIO
        if (strcmp(sci->portname, "/dev/ttyACM0") == 0) {
            Parity = static_cast<MW_SCI_Parity_Type>(MW_TTYACM0_PARITY);
            StopBits = static_cast<MW_SCI_StopBits_Type>((2 * MW_TTYACM0_STOPBIT) + 1);
#if DEBUG
            PX4_INFO(
                "Inside MW_SCI_SetFrameFormat strcmp. Port name = %s.Parity = %u. StopBits = %u\n.",
                sci->portname, Parity, StopBits);
#endif
        } else if (strcmp(sci->portname, "/dev/ttyS0") == 0) {
            Parity = static_cast<MW_SCI_Parity_Type>(MW_TTYS0_PARITY);
            StopBits = static_cast<MW_SCI_StopBits_Type>((2 * MW_TTYS0_STOPBIT) + 1);
        } else if (strcmp(sci->portname, "/dev/ttyS1") == 0) {
            Parity = static_cast<MW_SCI_Parity_Type>(MW_TTYS1_PARITY);
            StopBits = static_cast<MW_SCI_StopBits_Type>((2 * MW_TTYS1_STOPBIT) + 1);
        } else if (strcmp(sci->portname, "/dev/ttyS2") == 0) {
            Parity = static_cast<MW_SCI_Parity_Type>(MW_TTYS2_PARITY);
            StopBits = static_cast<MW_SCI_StopBits_Type>((2 * MW_TTYS2_STOPBIT) + 1);
#if DEBUG
            PX4_INFO(
                "Inside MW_SCI_SetFrameFormat strcmp. Port name = %s.Parity = %u. StopBits = %u\n.",
                sci->portname, Parity, StopBits);
#endif
        } else if (strcmp(sci->portname, "/dev/ttyS3") == 0) {
            Parity = static_cast<MW_SCI_Parity_Type>(MW_TTYS3_PARITY);
            StopBits = static_cast<MW_SCI_StopBits_Type>((2 * MW_TTYS3_STOPBIT) + 1);
        } else if (strcmp(sci->portname, "/dev/ttyS4") == 0) {
            Parity = static_cast<MW_SCI_Parity_Type>(MW_TTYS4_PARITY);
            StopBits = static_cast<MW_SCI_StopBits_Type>((2 * MW_TTYS4_STOPBIT) + 1);
        } else if (strcmp(sci->portname, "/dev/ttyS5") == 0) {
            Parity = static_cast<MW_SCI_Parity_Type>(MW_TTYS5_PARITY);
            StopBits = static_cast<MW_SCI_StopBits_Type>((2 * MW_TTYS5_STOPBIT) + 1);
        } else if (strcmp(sci->portname, "/dev/ttyS6") == 0) {
            Parity = static_cast<MW_SCI_Parity_Type>(MW_TTYS6_PARITY);
            StopBits = static_cast<MW_SCI_StopBits_Type>((2 * MW_TTYS6_STOPBIT) + 1);
#if DEBUG
            PX4_INFO(
                "Inside MW_SCI_SetFrameFormat strcmp. Port name = %s.Parity = %u. StopBits = %u\n.",
                sci->portname, Parity, StopBits);
#endif
        } else {
            perror("SERIAL_SetFrameFormat/Parity");
            return MW_SCI_BUS_ERROR;
        }
#endif

#if DEBUG
        printf("Inside MW_SCI_SetFrameFormat. Port name = %s. Parity = %u. StopBits = %u\n.",
               sci->portname, Parity, StopBits);
#endif
        /* Set parity*/
        switch (Parity) {
        case MW_SCI_PARITY_NONE:
            options.c_cflag &= ~PARENB;
            break;
        case MW_SCI_PARITY_EVEN:
            options.c_cflag |= PARENB;
            options.c_cflag &= ~PARODD;
            break;
        case MW_SCI_PARITY_ODD:
            options.c_cflag |= PARENB;
            options.c_cflag |= PARODD;
            break;
        default:
            perror("SERIAL_SetFrameFormat/Parity");
            return MW_SCI_BUS_ERROR;
        }

        /* Set stop bits (1 or 2)*/
        switch (StopBits) {
        case MW_SCI_STOPBITS_1:
            options.c_cflag &= ~CSTOPB;
            break;
        case MW_SCI_STOPBITS_2:
            options.c_cflag |= CSTOPB;
            break;
        default:
            perror("SERIAL_SetFrameFormat/StopBits");
            return MW_SCI_BUS_ERROR;
        }

        /* Set attributes*/
        if (tcsetattr(sci->fd, TCSANOW, &options) < 0) {
            perror("SERIAL_SetAttribute");
            return MW_SCI_BUS_ERROR;
        }
        /*flush both data received but not read, and data written but not transmitted.*/
        tcflush(sci->fd, TCIOFLUSH);
    }

    return MW_SCI_SUCCESS;
}

MW_SCI_Status_Type MW_SCI_GetDataBytesAvailable(MW_Handle_Type SCIModuleHandle,
                                                bool blockingMode,
                                                void* size,
                                                int32_T timeout) {
    int out;
    SCI_dev_t* sci;
    int* databytesAvailable = (int*)size;
    int error_counter = 0;
    if (NULL != (void*)SCIModuleHandle) {
        sci = (SCI_dev_t*)SCIModuleHandle;
#if DEBUG
        int currSciDev;
        currSciDev = getCurrSciDev(sci->portname);
        printf("Inside GetDataBytesAvailable. SCIModule = %d. | ", currSciDev);
        printf("port name = %s | fd = %d \n", sci->portname, sci->fd);
#endif

        if (blockingMode) {
            // Blocking mode
            px4_pollfd_struct_t fds[1];
            fds[0].fd = sci->fd;
            fds[0].events = POLLIN;

#if DEBUG
            printf("Inside MW_SCI_GetDataBytesAvailable. Before Poll \n");
#endif
            timeout = timeout * 1000;
            int poll_ret = px4_poll(fds, 1, timeout);
#if DEBUG
            printf("Inside MW_SCI_GetDataBytesAvailable. After Poll \n");
#endif
            /* handle the poll result */
            if (poll_ret == 0) {
                /* this means none of our providers is giving us data */
                PX4_ERR("Got no data within the timeout specified");
                return MW_SCI_DATA_NOT_AVAILABLE;
            } else if (poll_ret < 0) {
                /* this is seriously bad - should be an emergency */
                if (error_counter < 10 || error_counter % 50 == 0) {
                    /* use a counter to prevent flooding (and slowing us down) */
                    PX4_ERR("ERROR return value from poll(): %d", poll_ret);
                    return MW_SCI_BUS_ERROR;
                }
            } else {
                if (fds[0].revents & POLLIN) {
                    sci->busy = 1; /*Set the busy flag*/
                    out = ioctl(sci->fd, FIONREAD, (unsigned long)databytesAvailable);
                    sci->busy = 0; /*Reset the busy flag*/
                    if (out < 0) {
                        perror("MW_SCI_GetDataBytesAvailable");
                        return MW_SCI_BUS_ERROR;
                    }
                }
            }
        } // End of Blocking mode
        else
        // Non-Blocking mode
        {
#if DEBUG
            printf("Inside MW_SCI_GetDataBytesAvailable. Non-blocking mode \n");
#endif
            sci->busy = 1; /*Set the busy flag*/
            out = ioctl(sci->fd, FIONREAD, (unsigned long)databytesAvailable);
            sci->busy = 0; /*Reset the busy flag*/
            if (out < 0) {
                perror("MW_SCI_GetDataBytesAvailable");
                return MW_SCI_BUS_ERROR;
            }
        } // End of Non-Blocking mode
          //         ptr_size = (uint8_T*)&databytesAvailable;
#if DEBUG
        printf("Debug Print in MW_SCI_GetDataBytesAvailable: available %lu bytes : %u bytes.\n ",
               *databytesAvailable, *(char*)size);
#endif
    }
    return MW_SCI_SUCCESS;
}

/* Receive the data over SCI */
MW_SCI_Status_Type MW_SCI_Receive(MW_Handle_Type SCIModuleHandle,
                                  uint8_T* RxDataPtr,
                                  uint32_T RxDataLength) {
    SCI_dev_t* sci;
    ssize_t bytes_read = 0;

    if (NULL != (void*)SCIModuleHandle) {
        sci = (SCI_dev_t*)SCIModuleHandle;
#if DEBUG
        int currSciDev;
        currSciDev = getCurrSciDev(sci->portname);
        printf("Inside Receive. SCIModule = %d. | ", currSciDev);
        printf("port name = %s | fd = %d \n", sci->portname, sci->fd);
#endif
        if (0 == sci->busy) {
            sci->busy = 1; /*Set the busy flag*/
            bytes_read = read(sci->fd, RxDataPtr, RxDataLength);
            sci->busy = 0; /*Reset the busy flag*/
        } else {
            printf("Serial Port busy. Unable to perform read operation.\n");
            return MW_SCI_RX_BUSY;
        }
        usleep(1000);
        //---------------(For Debug)-----------Working----------------------------------------
        if (bytes_read > 0) // for debugging only.. remove as needed
        {
#if DEBUG
            uint32_T i;
            printf("Debug Print: received %d bytes. return = %u \n ", bytes_read, MW_SCI_SUCCESS);
            for (i = 0; i < RxDataLength; i++) {
                printf("[%d]", RxDataPtr[i]);
            }
            printf("\n");
            fflush(stdout);
#endif
        } else {
#if DEBUG
            printf("received %d bytes. err no : %d : %s.\n", bytes_read, errno, strerror(errno));
            fflush(stdout);
#endif
            return MW_SCI_DATA_NOT_AVAILABLE;
        }
        //--------------------------Working----------------------------------------
    }
    return MW_SCI_SUCCESS;
}

/* Transmit the data over SCI */
MW_SCI_Status_Type MW_SCI_Transmit(MW_Handle_Type SCIModuleHandle,
                                   uint8_T* TxDataPtr,
                                   uint32_T TxDataLength) {
    SCI_dev_t* sci;
    int ret;
    if (NULL != (void*)SCIModuleHandle) {

        sci = (SCI_dev_t*)SCIModuleHandle;
#if DEBUG
        int currSciDev;
        currSciDev = getCurrSciDev(sci->portname);
        printf("Inside Transmit. SCIModule = %d. | ", currSciDev);
        printf("port name = %s | fd = %d | ", sci->portname, sci->fd);
        printf("data : ");
        uint32_T i;
        for (i = 0; i < TxDataLength; i++) {
            printf("[%d]", TxDataPtr[i]);
        }
        printf("\n");
#endif
        if (0 == sci->busy) {
            sci->busy = 2; /*Set the busy flag*/
            ret = write(sci->fd, TxDataPtr, TxDataLength);
            sci->busy = 0; /*Reset the busy flag*/
        } else {
            printf("Serial Port busy. Unable to perform write operation.\n");
            return MW_SCI_TX_BUSY;
        }
        if (ret < 0) {
            perror("SERIAL_write/write");
            return MW_SCI_BUS_ERROR;
        }
    }
    return MW_SCI_SUCCESS;
}

struct timespec diff(struct timespec start, struct timespec end) {
    struct timespec temp;
    if ((end.tv_nsec - start.tv_nsec) < 0) {
        temp.tv_sec = end.tv_sec - start.tv_sec - 1;
        temp.tv_nsec = 1000000000 + end.tv_nsec - start.tv_nsec;
    } else {
        temp.tv_sec = end.tv_sec - start.tv_sec;
        temp.tv_nsec = end.tv_nsec - start.tv_nsec;
    }
    return temp;
}

/* Close SERIAL channel*/
static void SERIAL_close(int fd) {
    int ret;

    ret = close(fd);
    if (ret < 0) {
        /* EBADF, EINTR, EIO: In all cases, descriptor is torn down*/
        perror("SERIAL_close/close");
    }
}

/* Release SCI module */
void MW_SCI_Close(MW_Handle_Type SCIModuleHandle) {
    SCI_dev_t* sci;
    if (NULL != (void*)SCIModuleHandle) {
        sci = (SCI_dev_t*)SCIModuleHandle;
        if (sci->fd > 0) {
#if DEBUG
            printf("Inside SCI_Close. port name = %s | fd = %d \n", sci->portname, sci->fd);
#endif
            SERIAL_close(sci->fd);
            sci->fd = -1;
        }
    }
}

MW_SCI_Status_Type MW_SCI_ConfigureHardwareFlowControl(
    MW_Handle_Type SCIModuleHandle,
    MW_SCI_HardwareFlowControl_Type HardwareFlowControl,
    uint32_T RtsDtrPin,
    uint32_T CtsDtsPin) {
    uint8_T hardwareFCEnable = 0;
    SCI_dev_t* sci;
    struct termios options;
    if (NULL != (void*)SCIModuleHandle) {
        sci = (SCI_dev_t*)SCIModuleHandle;
#if DEBUG
        int currSciDev;
        currSciDev = getCurrSciDev(sci->portname);
        printf("Inside ConfigureHardwareFlowControl. SCIModule = %d. | ", currSciDev);
        printf("port name = %s | fd = %d \n", sci->portname, sci->fd);
#endif
        if (tcgetattr(sci->fd, &options) < 0) {
            perror("SERIAL_GetAttribute");
            return MW_SCI_BUS_ERROR;
        }

#ifndef PX4_CONNECTEDIO
        if (strcmp(sci->portname, "/dev/ttyACM0") == 0) {
            hardwareFCEnable = MW_TTYACM0HWFLOWCONTROL_CHECKBOX;
#if DEBUG
            PX4_INFO(
                "Inside MW_SCI_SetFrameFormat strcmp. Port name = %s.hardwareFCEnable = %u. \n.",
                sci->portname, hardwareFCEnable);
#endif
        } else if (strcmp(sci->portname, "/dev/ttyS0") == 0) {
            hardwareFCEnable = MW_TTYS0HWFLOWCONTROL_CHECKBOX;
        } else if (strcmp(sci->portname, "/dev/ttyS1") == 0) {
            hardwareFCEnable = MW_TTYS1HWFLOWCONTROL_CHECKBOX;
        } else if (strcmp(sci->portname, "/dev/ttyS2") == 0) {
            hardwareFCEnable = MW_TTYS2HWFLOWCONTROL_CHECKBOX;
#if DEBUG
            PX4_INFO(
                "Inside MW_SCI_SetFrameFormat strcmp. Port name = %s.hardwareFCEnable = %u. \n.",
                sci->portname, hardwareFCEnable);
#endif
        } else if (strcmp(sci->portname, "/dev/ttyS3") == 0) {
            hardwareFCEnable = MW_TTYS3HWFLOWCONTROL_CHECKBOX;
        } else if (strcmp(sci->portname, "/dev/ttyS4") == 0) {
            hardwareFCEnable = MW_TTYS4HWFLOWCONTROL_CHECKBOX;
        } else if (strcmp(sci->portname, "/dev/ttyS5") == 0) {
            hardwareFCEnable = MW_TTYS5HWFLOWCONTROL_CHECKBOX;
        } else if (strcmp(sci->portname, "/dev/ttyS6") == 0) {
            hardwareFCEnable = MW_TTYS6HWFLOWCONTROL_CHECKBOX;
#if DEBUG
            PX4_INFO("Inside MW_SCI_SetFrameFormat strcmp. Port name = %s.hardwareFCEnable = %u.",
                     sci->portname, hardwareFCEnable);
#endif
        } else {
            perror("SERIAL_SetFrameFormat/Parity");
            return MW_SCI_BUS_ERROR;
        }
#endif

        /* Set data bits for Hardware Flow Control*/
        if (1 == hardwareFCEnable) {
            options.c_cflag |= CRTSCTS;
        } else if (0 == hardwareFCEnable) {
            options.c_cflag &= ~CRTSCTS;
        }
    }
    return MW_SCI_SUCCESS;
}

MW_SCI_Status_Type MW_SCI_GetStatus(MW_Handle_Type SCIModuleHandle) {
    return MW_SCI_SUCCESS;
}

MW_SCI_Status_Type MW_SCI_SendBreak(MW_Handle_Type SCIModuleHandle) {
    return MW_SCI_SUCCESS;
}

#endif
