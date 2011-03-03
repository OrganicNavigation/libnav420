/***************************************************************************
                        nav420.h  -  description
                            -------------------
  begin                : December 2005
  copyright            : (C) 2005 by Pierre Lamon
  email                : pierre.lamon@epfl.ch

  description          : define the private structures and macros for the
                         interface with the NAV420 unit. Many macros are
                         defined to minimize function calls and, therefore,
                         speed up the execution. This header should not be
                         included in the user code. Use nav420_lib.h instead!
 ***************************************************************************/

#ifndef NAV420_H
#define NAV420_H

#include <sys/types.h>

#include <math.h>

#include <libelrob/Etypes.h>
#include <libelrob/Etime.h>

#include <libelrob/gps.h>

#ifndef DEG2RAD
#define DEG2RAD(deg)           ((deg)*M_PI/180.0)
#endif

#ifndef RAD2DEG
#define RAD2DEG(rad)           ((rad)*180.0/M_PI)
#endif

#ifndef CELSIUS_TO_KELVIN
#define CELSIUS_TO_KELVIN(cel) ((cel) + 273.15)
#endif

#ifndef KELVIN_TO_CELSIUS
#define KELVIN_TO_CELSIUS(kel) ((kel) - 273.15)
#endif

#define DMU_INT16(raw) ((int16_t)  ((raw)[0] << 8)  | ((raw)[1]))
#define DMU_UINT16(raw)((u_int16_t)((raw)[0] << 8)  | ((raw)[1]))
#define DMU_INT32(raw) ((int32_t)  ((raw)[0] << 24) | ((raw)[1] << 16) | ((raw)[2] << 8) | ((raw)[3]) )

#define DMU_NUM_MODES   3
#define DMU_SEM         3456

/* Communication */
#define DMU_CMD_GP      "GP"
#define DMU_CMD_PING    "PK"
#define DMU_CMD_WF      "WF"
#define DMU_CMD_SF      "SF"
#define DMU_CMD_RF      "RF"
#define DMU_CMD_WC      "WC"
#define DMU_CMD_FILL_HEADER(cmd) {cmd[0]=DMU_HEADER_CHAR; cmd[1]=cmd[0];}

#define DMU_CMD_START_CALIB 8
#define DMU_RS_START_CALIB  6

#define DMU_CMD_COMMIT_CALIB 8
#define DMU_CMD_CHANGE_BRATE 11
#define DMU_CMD_CHANGE_OUTPUT_RATE  11
#define DMU_CMD_CHANGE_PACKET_TYPE  11
#define DMU_CMD_REQUEST_DATA  7


#define DMU_RS_CALIB_PARAMS  17

#define DMU_CMD_SERIAL 7
#define DMU_RS_SERIAL  34

/* Conversion */
/* the NAV420 uses little endian numbers */
#define DMU_RAW_TO_ACCEL(raw) ((DMU_REAL)(DMU_INT16(raw)*10.0*DMU_GRAVITY\
                              /(1<<15)))

#define DMU_RAW_TO_RATE(raw)  DEG2RAD((DMU_REAL)(DMU_INT16(raw)*630.0\
                              /(1<<15)))

#define DMU_RAW_TO_MAG(raw)   ((DMU_REAL)(DMU_INT16(raw)*1.0/(1<<15)))


#define DMU_RAW_TO_ANGLE(raw) ((DMU_REAL)(DMU_INT16(raw)*M_PI/(1<<15)))

#define DMU_RAW_TO_TEMP(raw)  ((DMU_REAL)CELSIUS_TO_KELVIN(DMU_INT16(raw)*100.0\
                              /(1<<15)))

#define DMU_RAW_TO_VELOCITY(raw) ((DMU_REAL)(DMU_INT16(raw)*256.0/(1<<15)))
#define DMU_RAW_TO_LONG_LAT(raw) ((DMU_REAL)(DMU_INT32(raw)*M_PI/(1<<31)))

#define DMU_FILL_ACCEL(axis,raw) { \
  axis.x = DMU_RAW_TO_ACCEL(raw);  raw += 2;\
  axis.y = DMU_RAW_TO_ACCEL(raw);  raw += 2;\
  axis.z = DMU_RAW_TO_ACCEL(raw);  raw += 2;\
}

#define DMU_FILL_RATE(axis,raw) { \
  axis.x  = DMU_RAW_TO_RATE(raw);  raw += 2;\
  axis.y  = DMU_RAW_TO_RATE(raw);  raw += 2;\
  axis.z  = DMU_RAW_TO_RATE(raw);  raw += 2;\
}

#define DMU_FILL_MAG(axis,raw) { \
  axis.x = DMU_RAW_TO_MAG(raw);  raw += 2;\
  axis.y = DMU_RAW_TO_MAG(raw);  raw += 2;\
  axis.z = DMU_RAW_TO_MAG(raw);  raw += 2;\
}

#define DMU_FILL_TEMP(axis,raw) { \
  axis.x = DMU_RAW_TO_TEMP(raw);  raw += 2;\
  axis.y = DMU_RAW_TO_TEMP(raw);  raw += 2;\
  axis.z = DMU_RAW_TO_TEMP(raw);  raw += 2;\
}

#define DMU_FILL_VELOCITY(axis,raw) { \
  axis.x = DMU_RAW_TO_VELOCITY(raw);  raw += 2;\
  axis.y = DMU_RAW_TO_VELOCITY(raw);  raw += 2;\
  axis.z = DMU_RAW_TO_VELOCITY(raw);  raw += 2;\
}

#define DMU_FILL_ANGLE(axis,raw) { \
  axis.x = DMU_RAW_TO_ANGLE(raw);  raw += 2;\
  axis.y = DMU_RAW_TO_ANGLE(raw);  raw += 2;\
  axis.z = DMU_RAW_TO_ANGLE(raw);  raw += 2;\
}

#define DMU_FILL_VOID(axis) { \
  axis.x = DMU_FIELD_VOID;\
  axis.y = DMU_FIELD_VOID-1;\
  axis.z = DMU_FIELD_VOID-2;\
}

#define DMU_FILL_VAL(axis,val) { \
  axis.x = (val);\
  axis.y = (val);\
  axis.z = (val);\
}

#define DMU_FILL_VOID_GPS(gps) { \
  gps.latitude = DMU_FIELD_VOID;\
  gps.longitude = DMU_FIELD_VOID-1;\
  gps.altitude = DMU_FIELD_VOID-2;\
}

#ifndef DMU_FIELD_VOID
#define DMU_FIELD_VOID   9999.9  /* used for uninitialized float */
#endif

#ifndef DMU_GRAVITY
#define DMU_GRAVITY      9.81
#endif

#ifndef DMU_MAX_REC_SIZE
#define DMU_MAX_REC_SIZE 37
#endif

/* Exported structures */
typedef double DMU_REAL;

typedef unsigned short  u_int16_t;
typedef unsigned char   u_int8_t;

/** DMU_SCALED_MODE -> raw data, DMU_ANGLE_MODE -> run Kalman filter to
estimates the angles, DMU_NAV_MODE -> 6DOF's with GPS*/
typedef enum _DMU_MODE_TYPE {DMU_SCALED_MODE = 0, DMU_ANGLE_MODE, DMU_NAV_MODE}
DMU_MODE_TYPE;
typedef enum _DMU_RET_TYPE {DMU_RET_OK = -1,DMU_RET_BAD_CHKSUM =
0,DMU_RET_BAD_ARG,\
  DMU_RET_WRONG_PACKET,DMU_RET_BAD_HEADER,DMU_RET_BAD_BUFSIZE,\

DMU_RET_COMM_ERR,DMU_RET_WRITE_ERROR,DMU_RET_COMM_BUSY,DMU_RET_COMM_WAITING}
DMU_RET_TYPE;

typedef enum _DMU_RATE_TYPE
{DMU_RATE_QUIET=0,DMU_RATE_100=1,DMU_RATE_50=2,DMU_RATE_25=4,\
  DMU_RATE_20=5,DMU_RATE_10=10,DMU_RATE_2=50 } DMU_RATE_TYPE;

typedef enum _DMU_ACCU_TYPE
{DMU_ACCU_FULL,DMU_ACCU_LOW_HIGH,DMU_ACCU_LOW,DMU_ACCU_INIT} DMU_ACCU_TYPE;
typedef enum _DMU_DEV_TYPE {DMU_DEV_NAV, DMU_DEV_GPS} DMU_DEV_TYPE;

typedef struct _DMU_Shared_Mem
{
  TIMEVAL         stamp;
  unsigned long   meas_id;
  u_int8_t        rec_buf[DMU_MAX_REC_SIZE];
}DMU_Shared_Mem;

typedef struct _DMU_Calib_Result
{
  DMU_RET_TYPE  retval;
  DMU_REAL      bias_x;
  DMU_REAL      bias_y;
  DMU_REAL      scale;
}DMU_Calib_Result;

/** The structure containing the initial coordinate of the local map */
typedef struct _DMU_PoseZero_Struct{
  GPS_Struct      gps;             /* initial GPS coordinates */
  EPOINT3D        rpy;             /* initial roll pitch and heading (no
declination considered) */
  DMU_REAL        declination;     /* depends on lat long altitude and time (use
DMU_comp_geomag_now function to init) */
  DMU_REAL        radius;          /* do not modify by hand, use
DMU_SetLocalOrigin instead*/
}DMU_PoseZero_Struct;

/** The structure containing a 'Scaled' packet */
typedef struct _DMU_Scaled_Struct{
  EPOINT3D  accel;
  EPOINT3D  rate;
  EPOINT3D  mag;
  EPOINT3D  temp;
  DMU_REAL  cpu_temp;
  TIMEVAL   gps_itow;
  u_int16_t bit;
}DMU_Scaled_Struct;

/** The structure containing a 'Angle' packet. The heading angle corresponds to
the magnetic north */
typedef struct _DMU_Angle_Struct{
  EPOINT3D  angle;
  EPOINT3D  rate;
  EPOINT3D  accel;
  EPOINT3D  mag;
  DMU_REAL  temp;
  TIMEVAL   gps_itow;
  u_int16_t bit;
}DMU_Angle_Struct;

/** The structure containing a 'Angle' packet. The heading angle corresponds to
the true north */
typedef struct _DMU_Nav_Struct{
  EPOINT3D  angle;
  EPOINT3D  rate;
  EPOINT3D  velocity;
  GPS_Struct       coord;
  TIMEVAL          gps_itow;
  u_int16_t        bit;
}DMU_Nav_Struct;

void DMU_PRINT(const char *format, ...);
void DMU_DBG(const char *format, ...);

/* Exported functions */
/** Return the bits per second */
int DMU_BaudrateToBitPerSec(int baudrate);

/** Return the transmission time in micro-seconds of a packet of "mode" type */
unsigned int DMU_ComputeTransTime(DMU_MODE_TYPE mode, int baudrate);

/** Open the serial port */
int DMU_OpenSerial (DMU_DEV_TYPE port, char *device, int baudrate);

/** Close the serial port */
int DMU_CloseSerial(DMU_DEV_TYPE port);

/** Set the default output stream \arg ostream : a file descriptor (e.g. stdout,
stderr, etc.)*/
void DMU_SetPrintStream(FILE *ostream);

/** Print a string corresponding to a DMU_RET_TYPE */
void DMU_PrintRetType(DMU_RET_TYPE val);

/** Print the raw packet */
void DMU_PrintRawPacket(u_int8_t *buf, DMU_MODE_TYPE mode);

/** Print the content of a DMU_Scaled_Struct */
void DMU_PrintScaledStruct(DMU_Scaled_Struct *strct);

/** Print the content of a DMU_Angle_Struct */
void DMU_PrintAngleStruct(DMU_Angle_Struct *strct);

/** Print the content of a DMU_Nav_Struct */
void DMU_PrintNavStruct(DMU_Nav_Struct *strct);

/** Print the content of the BIT field */
void DMU_PrintBIT(u_int16_t bit);

/** Print the content of an axis struct */
void DMU_PrintAxisStruct(EPOINT3D *strct);

/** Print the serial number and the firmware version of the NAV420 */
void DMU_PrintSerialAndVersion();

/** Print the internal calibration structure */
void DMU_PrintCalibResult();

/** Init a DMU_Scaled_Struct with default values */
void DMU_ScaledInitVoid(DMU_Scaled_Struct *strct);

/** Init a DMU_Scaled_Struct with 0 */
void DMU_ScaledClear(DMU_Scaled_Struct *strct);

/** Init a DMU_Angle_Struct with 0 */
void DMU_AngleClear(DMU_Angle_Struct *strct);

/** Init a DMU_Nav_Struct with 0 */
void DMU_NavClear(DMU_Nav_Struct *strct);

/** Init a DMU_Angle_Struct with default values */
void DMU_AngleInitVoid(DMU_Angle_Struct *strct);

/** Init a DMU_Nav_Struct with default values */
void DMU_NavInitVoid(DMU_Nav_Struct *strct);

/** Init a EPOINT3D with void values */
void DMU_AxisInitVoid(EPOINT3D *strct);

/** Read the serial number and the firmware version of the NAV420 unit
*\arg serial:  the address of the variable to be filled with the serial number
*\arg version: an already allocated string (min size 30), can be NULL
*/
DMU_RET_TYPE DMU_QuerySerialAndVersion(u_int32_t *serial, char *version);

/** Read a 'Scaled' packet from the NAV420. Use this function when the NAV420 is
in streaming mode. */
DMU_RET_TYPE DMU_ReadScaledData(DMU_Scaled_Struct *strct);

/** Read a 'Angle' packet from the NAV420. Use this function when the NAV420 is
in streaming mode. */
DMU_RET_TYPE DMU_ReadAngleData(DMU_Angle_Struct *strct);

/** Read a 'Nav' packet from the NAV420. Use this function when the NAV420 is in
streaming mode. */
DMU_RET_TYPE DMU_ReadNavData(DMU_Nav_Struct *strct);

/** Allows to resync on a valid header */
DMU_RET_TYPE DMU_Resync();

/** Read a 'Scaled' packet from the NAV420 in polled mode */
DMU_RET_TYPE DMU_RequestScaledPacket(DMU_Scaled_Struct *strct);

/** Read a 'Angle' packet from the NAV420 in polled mode */
DMU_RET_TYPE DMU_RequestAnglePacket(DMU_Angle_Struct *strct);

/** Read a 'Nav' packet from the NAV420 in polled mode */
DMU_RET_TYPE DMU_RequestNavPacket(DMU_Nav_Struct *strct);

/** Ping the unit */
DMU_RET_TYPE DMU_Ping();

/** Change baud rate
*\arg rate: one of B9600, B19200, B38400, B57600
*/
DMU_RET_TYPE DMU_ChangeBaudRate(int rate);

/** Change the packet output rate
*\arg rate : one of DMU_RATE_TYPE constant
*\arg store: 0 -> does not save setting in the EEPROM else save in EEPROM
*/
DMU_RET_TYPE DMU_ChangeOutputRate(DMU_RATE_TYPE rate, int store);

/** Change the packet type
 *\arg mode : one of DMU_MODE_TYPE constant
 *\arg store: 0 -> does not save setting in the EEPROM else save in EEPROM
 */
DMU_RET_TYPE DMU_ChangePacketType(DMU_MODE_TYPE mode, int store);

/** Do soft and hard iron calibration */
DMU_RET_TYPE DMU_InteractIronCalibration();

/** Launch a thread that reads data continuously from the nav420 */
DMU_RET_TYPE DMU_LaunchNavContThread(DMU_MODE_TYPE mode, int root, void
(*pCallback) (DMU_Shared_Mem *));

/** Launch a thread that reads data continuously from the GPS */
DMU_RET_TYPE DMU_LaunchGPSContThread(int root, void (*pCallback) (GPS_gga *,
GPS_rmc *));

/** Launch a thread waiting for the calibration to finish */
DMU_RET_TYPE DMU_LaunchIronCalibration();

/** Return 1 if the calibration is done */
int DMU_IsCalibrationDone();

/** Return the calibration results stored during the iron calibration */
DMU_Calib_Result DMU_GetCalibrationResult();

/** Store the calibration parameters in the EEPROM */
DMU_RET_TYPE DMU_CommitCalibration();

/** Kills the thread */
void DMU_KillThread(DMU_DEV_TYPE dev);

/** Read the packet that has been read by the thread */
void DMU_GetThreadPacket(u_int8_t *buf, TIMEVAL *tv, unsigned long *meas_id);

/** Wait until a new packet is available(avoid reading twice the same packet),
 * the function returns when the new packet stamp differs from *tv
 * *tv is then updated with the new stamp
 */
void DMU_GetThreadNewPacket(u_int8_t *buf, TIMEVAL *tv, unsigned long *meas_id);

/** Get the received packet type */
DMU_MODE_TYPE DMU_GetPacketType(u_int8_t *buf);

/** Tells if the thread is running */
int DMU_ThreadRunning(DMU_DEV_TYPE dev);

/** Expand raw packet into understandable structures */
void DMU_FillScaledStruct(u_int8_t *buf, DMU_Scaled_Struct *strct);
void DMU_FillAngleStruct(u_int8_t *buf, DMU_Angle_Struct *strct);
void DMU_FillNavStruct(u_int8_t *buf, DMU_Nav_Struct *strct);

int DMU_CorrGPSTimestamp(TIMEVAL curr_stamp, TIMEVAL curr_itow, TIMEVAL
*old_itow);
void DMU_comp_geomag(double decim_date, GPS_Struct gps, double *myDeclination);
void DMU_comp_geomagTV(TIMEVAL time, GPS_Struct gps, double *myDeclination);
void DMU_comp_geomag_now(GPS_Struct gps, double *myDeclinaison);
void DMU_PrintLocalOrigin(DMU_PoseZero_Struct *strct);

GPS_gga DMU_GGA_GetCopy();
GPS_rmc DMU_RMC_GetCopy();

int DMU_WaitPacket(DMU_Shared_Mem *shm);
int DMU_StoreITOWChangeTime(TIMEVAL curr_itow, TIMEVAL curr_stamp);

#endif
