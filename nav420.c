/***************************************************************************
                        nav420.c  -  description
                            -------------------
  begin                : December 2005
  copyright            : (C) 2005 by Pierre Lamon
  email                : pierre.lamon@epfl.ch

  description          : implement the functions to communicate with the nav420
 ***************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <string.h>
#include <stdarg.h>
#include <pthread.h>
#include <unistd.h>

#include "nav420.h"
#include <elrob/libgps.h>
#include <elrob/gps-nmea.h>
#include "nav420_lib.h"
#include <elrob/serial_easy.h>
#include <elrob/Etime.h>
#include <elrob/Emacros.h>

typedef struct _DMU_ThreadParams{
  int baudrate_nav420;
  int baudrate_gps;
  int run_nav;
  int run_gps;
}DMU_ThreadParams;

/* Define the generic read and write functions */
#ifdef _USE_LIBSERIAL

/* FIXME use libserial stuff here */

#else

#endif

#define DMU_CHK_PACK_TYPE(byte,mode) (*(byte) == DMU_MODE_CHAR[(mode)])
#define DMU_CHK_VALID(mode)          ((mode) < DMU_NUM_MODES)

/* Constant arrays */
const u_int8_t DMU_RAW_PK_SIZE[DMU_NUM_MODES] = {35,35,37};
const char     DMU_MODE_CHAR[DMU_NUM_MODES]   = {'S','A','N'};
const char     DMU_RET_STRING[10][50]={"DMU_RET_OK","DMU_RET_BAD_CHKSUM",\
                                      "DMU_RET_BAD_ARG","DMU_RET_WRONG_PACKET",\
                                      "DMU_RET_BAD_HEADER","DMU_RET_BAD_BUFSIZE",\
                                      "DMU_RET_COMM_ERR","DMU_RET_WRITE_ERROR","DMU_RET_COMM_BUSY","DMU_RET_COMM_WAITING"};

const char DMU_MSG_DISCONNECT_UNIT[] = "Disconnect power from the NAV420CA and wait 10s at least !\n";
const char DMU_MSG_REPEAT_CALIB[] = "Repeat the process to check consistency:\nThe parameters should not deviate more than 0.01 between calibrations!\n";
const char DMU_MSG_CALIB_DONE[] = "Calibration done!\n";
const char DMU_MSG_PLEASE_ROTATE[] = "Please rotate the unit through a 380 deg of roation (around the z axis) (10-20 deg/s is ideal)\n";

/* Global variables for the library */
static int                  ttyNAV = -1;                         /* file descriptor for serial communication with the nav420 */
static int                  ttyGPS = -1;                         /* file descriptor for serial communication with the GPS*/
static FILE*                ostrm = NULL;                        /* file for debug print */
static pthread_mutex_t      m_mutex_nav = PTHREAD_MUTEX_INITIALIZER; /* mutex for continuous capture */
static pthread_mutex_t      m_mutex_gps = PTHREAD_MUTEX_INITIALIZER; /* mutex for continuous capture */
static int                  m_gga_received = 0;
static pthread_t            m_tty_thread_nav;
static pthread_t            m_tty_thread_gps;
static pthread_t            m_tty_thread_calib;
static pthread_attr_t       m_thread_attr_nav;
static pthread_attr_t       m_thread_attr_gps;
static void (*m_pNAVCallback)(DMU_Shared_Mem *) = NULL;
static void (*m_pGPSCallback)(GPS_gga *, GPS_rmc *) = NULL;
static DMU_MODE_TYPE        m_mode = DMU_SCALED_MODE;
static DMU_RATE_TYPE        m_output_rate = DMU_RATE_QUIET;
static DMU_ThreadParams     m_params = {0,0,0,0};
static DMU_Shared_Mem       m_shared_mem;
static DMU_Calib_Result     m_calib_result = {DMU_RET_COMM_BUSY, DMU_FIELD_VOID, DMU_FIELD_VOID-1.0, DMU_FIELD_VOID-2.0};
static unsigned int         m_delay_us = 0;
static TIMEVAL              m_time_itow_changed = {0,0};
static TIMEVAL              m_old_itow = {0,0};

/* Prototypes */
void DMU_PrintRawBuffer(u_int8_t *buf, size_t size);

/* --- Debug and print function */

void DMU_DBG(const char *format, ...)
{
  va_list ap;
  char p[512];
  char fname[512];
  char *ptr;

  va_start(ap, format);
  vsprintf(p, format, ap);
  va_end(ap);

  strcpy(fname, __FILE__);
  ptr = strrchr(fname, '/');
  if(ptr != NULL)
    fprintf(stdout,"%s :%i: ", ptr+1, __LINE__);
  else
    fprintf(stdout,":%i: ", __LINE__);
  fprintf(stdout,p);
}

void DMU_PRINT(const char *format, ...)
{
  va_list ap;
  char p[512];

  /* Create string to append */
  va_start(ap, format);
  vsprintf(p, format, ap);
  va_end(ap);

  if(ostrm != NULL) fprintf(ostrm,p);
}

/* ---  Setter and getter functions */
int DMU_SetNavFileDescr(int fd){
  if(ttyNAV != -1)
    if(DMU_CloseSerial(ttyNAV) != 0) {
      fprintf(stderr,"Could not close ttyNAV (%i)!\n", ttyNAV);
      return 0;
    }
  ttyNAV = fd;
  return 1;
}

int DMU_GetFileDescr(){
  return ttyNAV;
}

void DMU_SetPrintStream(FILE *oostrm) {
  ostrm = oostrm;
}

/* ---  Serial port functions */

/* Return the transmission time in micro-seconds */
unsigned int DMU_ComputeTransTime(DMU_MODE_TYPE mode, int baudrate)
{
  return ComputeTransTime(DMU_RAW_PK_SIZE[mode], baudrate);
}

int DMU_OpenSerial(DMU_DEV_TYPE port, char *device, int baudrate)
{
  if(port == DMU_DEV_NAV){
    ttyNAV = OpenSerial(device, baudrate, 50000, 37);
    m_params.baudrate_nav420 = baudrate;
    return ttyNAV;
  }

  if(port == DMU_DEV_GPS){
    ttyGPS = OpenSerial(device, baudrate, 50000,37);
    m_params.baudrate_gps = baudrate;
    return ttyGPS;
  }

  return 0;
}

int DMU_CloseSerial(DMU_DEV_TYPE port)
{
  if(port == DMU_DEV_NAV) return CloseSerial(ttyNAV);
  if(port == DMU_DEV_GPS) return CloseSerial(ttyGPS);
  return 0;
}

/* ---  Init functions */
void DMU_ScaledInitVoid(DMU_Scaled_Struct *strct)
{
  DMU_FILL_VOID(strct->accel);
  DMU_FILL_VOID(strct->rate);
  DMU_FILL_VOID(strct->mag);
  DMU_FILL_VOID(strct->temp);
  strct->cpu_temp = DMU_FIELD_VOID;
  Time_InitVoid(&strct->gps_itow);
  strct->bit = 0xFFFF;
}

void DMU_AngleInitVoid(DMU_Angle_Struct *strct)
{
  DMU_FILL_VOID(strct->angle);
  DMU_FILL_VOID(strct->rate);
  DMU_FILL_VOID(strct->accel);
  DMU_FILL_VOID(strct->mag);
  strct->temp = DMU_FIELD_VOID;
  Time_InitVoid(&strct->gps_itow);
  strct->bit = 0xFFFF;
}

void DMU_NavInitVoid(DMU_Nav_Struct *strct)
{
  DMU_FILL_VOID(strct->angle);
  DMU_FILL_VOID(strct->rate);
  DMU_FILL_VOID(strct->velocity);
  DMU_FILL_VOID_GPS(strct->coord);
  Time_InitVoid(&strct->gps_itow);
  strct->bit = 0xFFFF;
}

void DMU_AxisInitVoid(EPOINT3D *strct)
{
  strct->x = DMU_FIELD_VOID;
  strct->y = DMU_FIELD_VOID-1.0;
  strct->z = DMU_FIELD_VOID-2.0;
}

void DMU_ScaledClear(DMU_Scaled_Struct *strct)
{
  DMU_FILL_VAL(strct->accel,0.0);
  DMU_FILL_VAL(strct->rate,0.0);
  DMU_FILL_VAL(strct->mag,0.0);
  DMU_FILL_VAL(strct->temp,0.0);
  strct->cpu_temp = 0.0;
  strct->gps_itow.tv_sec  = 0;
  strct->gps_itow.tv_usec = 0;
  strct->bit = 0x0;
}

void DMU_AngleClear(DMU_Angle_Struct *strct)
{
  DMU_FILL_VAL(strct->angle,0.0);
  DMU_FILL_VAL(strct->rate,0.0);
  DMU_FILL_VAL(strct->accel,0.0);
  DMU_FILL_VAL(strct->mag,0.0);
  strct->temp = 0.0;
  strct->gps_itow.tv_sec  = 0;
  strct->gps_itow.tv_usec = 0;
  strct->bit = 0x0;
}

void DMU_NavClear(DMU_Nav_Struct *strct)
{
  DMU_FILL_VAL(strct->angle,0.0);
  DMU_FILL_VAL(strct->rate,0.0);
  DMU_FILL_VAL(strct->velocity,0.0);
  strct->coord.longitude = 0.0;
  strct->coord.latitude  = 0.0;
  strct->coord.altitude  = 0.0;
  strct->gps_itow.tv_sec  = 0;
  strct->gps_itow.tv_usec = 0;
  strct->bit = 0x0;
}

/* ---  Check functions */
inline u_int16_t DMU_CompChecksum(u_int8_t *buf, size_t size)
{
  u_int32_t  sum = 0;
  size_t     i;

  for (i = 0; i < size; i++) sum += buf[i];
  return (u_int16_t) (sum % (1 << 16));
}

DMU_RET_TYPE DMU_CheckChecksum(u_int8_t *buf, size_t size)
{
  if (DMU_CompChecksum(buf+2,size-4) == DMU_UINT16(&buf[size-2]))
    return DMU_RET_OK;
  else
    return DMU_RET_BAD_CHKSUM;
}

inline DMU_RET_TYPE  DMU_CheckValidPacket(u_int8_t *buf, int size, DMU_MODE_TYPE mode)
{
  DMU_RET_TYPE res;
  u_int16_t    bit;

  /* Check argument */
  if(!DMU_CHK_VALID(mode)) return DMU_RET_BAD_ARG;

  /* Check packet size */
  if(size != DMU_RAW_PK_SIZE[mode]) return DMU_RET_BAD_BUFSIZE;

  /* Check header */
  if(!DMU_CHK_HEAD(buf)) return DMU_RET_BAD_HEADER;

  /* Check if the received packet type correspond to the current mode */
  if(!DMU_CHK_PACK_TYPE(buf+2,mode)) return DMU_RET_WRONG_PACKET;

  /* Check the checksum */
  res = DMU_CheckChecksum(buf, size);
  if(res != DMU_RET_OK) return res;

  /* Check if there is a communication problem in the BIT
     the Crossbow doc is not clear (see bit 4 and 12 of the BIT)
     my guess is that at this point we should check only bit 4*/
  bit = DMU_UINT16(buf+size-4);
  if(DMU_MSK_TX_ERR & bit) return DMU_RET_COMM_ERR;

  return DMU_RET_OK;
}

/* ---  Tool functions */
inline void DMU_ITOW_to_Timeval(u_int16_t itow_ms, TIMEVAL *tv)
{
  tv->tv_sec  = itow_ms / 1000;
  tv->tv_usec = (itow_ms % 1000)*1000;
}

void DMU_FillCmd(u_int8_t *cmd, const char* cmd_str)
{
  size_t i;
  DMU_CMD_FILL_HEADER(cmd);
  for(i = 0; i < 2; i++) cmd[i+2] = cmd_str[i];
}

DMU_RET_TYPE DMU_WriteCmd(u_int8_t *cmd, size_t size)
{
  int n = SERIAL_WRITE(ttyNAV, cmd, size);
  if(n != size) return DMU_RET_COMM_ERR;
  return DMU_RET_OK;
}

DMU_RET_TYPE DMU_ReadAnswer(u_int8_t *buf, size_t size)
{
  int n = SERIAL_READ(ttyNAV,buf, size);
  if(n != size) return DMU_RET_COMM_ERR;
  return DMU_RET_OK;
}

DMU_RET_TYPE DMU_ProcessCmd(u_int8_t *cmd, size_t cmd_size\
    ,u_int8_t *buf, size_t buf_size)
{
  DMU_RET_TYPE res_write, res_read;

  res_write = DMU_WriteCmd(cmd, cmd_size);
  res_read  = DMU_ReadAnswer(buf, buf_size);
  if(res_write == DMU_RET_COMM_ERR || res_read == DMU_RET_COMM_ERR)
    return DMU_RET_COMM_ERR;

  return DMU_RET_OK;
}

inline void DMU_AddCmdCheckSum(u_int8_t *cmd, size_t size)
{
  u_int16_t    checksum;
  checksum = DMU_CompChecksum(cmd+2, size-4);

  cmd[size-2] = (checksum & 0xFF00) >> 8;
  cmd[size-1] = checksum & 0xFF;
}


DMU_REAL DMU_RawToFloat(u_int8_t *buf)
{
  return (DMU_REAL) (DMU_INT32(buf))/(1<<27);
}

void DMU_FillCalibStruct(u_int8_t *rec_buf, DMU_Calib_Result *calib)
{
  /* 4 bytes fixed point values */
  calib->bias_x = DMU_RawToFloat(rec_buf+3);
  calib->bias_y = DMU_RawToFloat(rec_buf+7);
  calib->scale  = DMU_RawToFloat(rec_buf+11);
}

void DMU_FillScaledStruct(u_int8_t *buf, DMU_Scaled_Struct *strct)
{
  u_int8_t *pByte = (u_int8_t *) (buf + 3);

  DMU_FILL_ACCEL(strct->accel,pByte);
  DMU_FILL_RATE(strct->rate,pByte);
  DMU_FILL_MAG(strct->mag,pByte);

  DMU_FILL_TEMP(strct->temp,pByte);
  strct->cpu_temp = DMU_RAW_TO_TEMP(pByte);
  pByte += 2;

  DMU_ITOW_to_Timeval(DMU_UINT16(pByte), &(strct->gps_itow));
  pByte += 2;
  strct->bit = DMU_UINT16(pByte);
}

void DMU_FillAngleStruct(u_int8_t *buf, DMU_Angle_Struct *strct)
{
  u_int8_t     *pByte = (u_int8_t *) (buf + 3);

  DMU_FILL_ANGLE(strct->angle,pByte);
  DMU_FILL_RATE(strct->rate,pByte);
  DMU_FILL_ACCEL(strct->accel,pByte);
  DMU_FILL_MAG(strct->mag,pByte);

  strct->temp = DMU_RAW_TO_TEMP(pByte);
  pByte += 2;

  DMU_ITOW_to_Timeval(DMU_UINT16(pByte), &(strct->gps_itow));
  pByte += 2;
  strct->bit = DMU_UINT16(pByte);

  if(strct->angle.z > M_PI)   strct->angle.z = M_PI;
  if(strct->angle.z <= -M_PI) strct->angle.z = M_PI;
}

void DMU_FillNavStruct(u_int8_t *buf, DMU_Nav_Struct *strct)
{
  int16_t      altitude;
  u_int8_t     *pByte = (u_int8_t *) (buf + 3);

  DMU_FILL_ANGLE(strct->angle,pByte);
  DMU_FILL_RATE(strct->rate,pByte);
  DMU_FILL_VELOCITY(strct->velocity,pByte);

  strct->coord.longitude = DMU_RAW_TO_LONG_LAT(pByte); pByte += 4;
  strct->coord.latitude  = DMU_RAW_TO_LONG_LAT(pByte); pByte += 4;

  altitude = DMU_INT16(pByte);
  if(altitude > -400 && altitude < 0)
    strct->coord.altitude = (DMU_REAL)(altitude * 8192.0/(1<<15));
  else
    strct->coord.altitude = (DMU_REAL)(((DMU_UINT16(pByte)))\
                                  * 8192.0/(1<<15));

  pByte += 2;
  DMU_ITOW_to_Timeval(DMU_UINT16(pByte), &(strct->gps_itow));
  pByte += 2;
  strct->bit = DMU_UINT16(pByte);

  if(strct->angle.z > M_PI)   strct->angle.z = M_PI;
  if(strct->angle.z <= -M_PI) strct->angle.z = M_PI;
}

DMU_RET_TYPE DMU_Resync()
{
  int      n, level = 0, mode;
  char     c;
  u_int8_t rec_buf[DMU_MAX_REC_SIZE];

  SERIAL_FLUSH_RX(ttyNAV);

  while(level != 3)
  {
    n = SERIAL_READ(ttyNAV,&c,1);
    if(n != 1) return DMU_RET_COMM_ERR;
    if(c == DMU_HEADER_CHAR || c == DMU_MODE_CHAR[0] ||\
       c == DMU_MODE_CHAR[1]|| c == DMU_MODE_CHAR[2]) level += 1;
    else level = 0;
  }

  /* At that point c contains the type of message being received */
  mode = 0;
  while (mode < 3 && DMU_MODE_CHAR[mode] != c) mode++;

  if(mode > 2) return DMU_RET_COMM_ERR;
  n = SERIAL_READ(ttyNAV,rec_buf, DMU_RAW_PK_SIZE[mode] - 3);

  if(n != DMU_RAW_PK_SIZE[mode] - 3) return DMU_RET_COMM_ERR;

  return DMU_RET_OK;
}

/* NAV420 commands */
DMU_RET_TYPE DMU_ChangeBaudRate(int rate)
{
  const size_t CS = DMU_CMD_CHANGE_BRATE;
  u_int8_t     cmd[DMU_CMD_CHANGE_BRATE];

  if(rate != B9600 && rate && B19200 && rate != B38400 && rate != B57600) return DMU_RET_BAD_ARG;

  DMU_FillCmd(cmd, DMU_CMD_WF);
  cmd[4] = 0x01; cmd[5] = 0x00; cmd[6] = 0x02; cmd[7] = 0x00;

  switch(rate) {
    case B9600:  cmd[8] = 0x00; break;
    case B19200: cmd[8] = 0x01; break;
    case B38400: cmd[8] = 0x02; break;
    case B57600: cmd[8] = 0x03; break;
  }

  DMU_AddCmdCheckSum(cmd,CS);
  return DMU_WriteCmd(cmd,CS);
}

DMU_RET_TYPE DMU_ChangeOutputRate(DMU_RATE_TYPE rate, int store)
{
  const size_t CS = DMU_CMD_CHANGE_OUTPUT_RATE;
  u_int8_t     cmd[DMU_CMD_CHANGE_OUTPUT_RATE];
  DMU_RET_TYPE res;

  if(store) DMU_FillCmd(cmd, DMU_CMD_WF);
  else DMU_FillCmd(cmd, DMU_CMD_SF);

  cmd[4] = 0x01;  cmd[5] = 0x00;  cmd[6] = 0x01;
  cmd[7] = 0x00;  cmd[8] = rate;

  DMU_AddCmdCheckSum(cmd,CS);

  res = DMU_WriteCmd(cmd,CS);
  usleep(100000);
  m_output_rate = rate;
  if(m_output_rate == DMU_RATE_QUIET)
    SERIAL_FLUSH_RX(ttyNAV);
  return res;
}

DMU_RET_TYPE DMU_ChangePacketType(DMU_MODE_TYPE mode, int store)
{
  const size_t CS = DMU_CMD_CHANGE_PACKET_TYPE;
  u_int8_t     cmd[DMU_CMD_CHANGE_PACKET_TYPE];
  DMU_RET_TYPE ret;

  if(store) DMU_FillCmd(cmd, DMU_CMD_WF);
  else DMU_FillCmd(cmd, DMU_CMD_SF);

  cmd[4] = 0x01;  cmd[5] = 0x00;  cmd[6] = 0x03;
  cmd[7] = 0x00;  cmd[8] = DMU_MODE_CHAR[mode];

  DMU_AddCmdCheckSum(cmd,CS);

  ret = DMU_WriteCmd(cmd,CS);
  if(ret == DMU_RET_OK) m_mode = mode;

  return ret;
}

DMU_RET_TYPE DMU_Ping()
{
  u_int8_t     cmd[4];
  DMU_RET_TYPE res;
  u_int8_t     rec_buf[DMU_MAX_REC_SIZE];

  DMU_FillCmd(cmd, DMU_CMD_PING);
  SERIAL_FLUSH_RX(ttyNAV);

  res = DMU_ProcessCmd(cmd,4,rec_buf,3);

  rec_buf[3] = '\0';

  if(res != DMU_RET_OK || rec_buf[0] != DMU_HEADER_CHAR ||\
            rec_buf[1] != DMU_HEADER_CHAR ||  rec_buf[2] != 'P')
    return DMU_RET_COMM_ERR;

  DMU_PRINT("Ping received '%s'\n", rec_buf);
  return DMU_RET_OK;
}

DMU_RET_TYPE DMU_RequestData(DMU_MODE_TYPE mode)
{
  const size_t CS = DMU_CMD_REQUEST_DATA;
  u_int8_t     cmd[DMU_CMD_REQUEST_DATA];

  if(!DMU_CHK_VALID(mode)) return DMU_RET_BAD_ARG;

  DMU_FillCmd(cmd, DMU_CMD_GP);
  cmd[4] = DMU_MODE_CHAR[mode];
  DMU_AddCmdCheckSum(cmd,CS);

  return DMU_WriteCmd(cmd,CS);
}

DMU_RET_TYPE DMU_RequestScaledPacket(DMU_Scaled_Struct *strct)
{
  DMU_RET_TYPE res;

  res = DMU_RequestData(DMU_SCALED_MODE);
  if(res != DMU_RET_OK) return res;

  return DMU_ReadScaledData(strct);
}

DMU_RET_TYPE DMU_RequestAnglePacket(DMU_Angle_Struct *strct)
{
  DMU_RET_TYPE res;

  res = DMU_RequestData(DMU_ANGLE_MODE);
  if(res != DMU_RET_OK) return res;

  return DMU_ReadAngleData(strct);
}

DMU_RET_TYPE DMU_RequestNavPacket(DMU_Nav_Struct *strct)
{
  DMU_RET_TYPE res;

  res = DMU_RequestData(DMU_NAV_MODE);
  if(res != DMU_RET_OK) return res;

  return DMU_ReadNavData(strct);
}

DMU_RET_TYPE DMU_ReadScaledData(DMU_Scaled_Struct *strct)
{
  int rec_size;
  DMU_RET_TYPE res;
  u_int8_t     rec_buf[DMU_MAX_REC_SIZE];

  rec_size = SERIAL_READ(ttyNAV, rec_buf, DMU_RAW_PK_SIZE[DMU_SCALED_MODE]);
  res = DMU_CheckValidPacket(rec_buf, rec_size, DMU_SCALED_MODE);
  if(res != DMU_RET_OK) return res;
  DMU_FillScaledStruct(rec_buf, strct);

  return DMU_RET_OK;
}

DMU_RET_TYPE DMU_ReadAngleData(DMU_Angle_Struct *strct)
{
  int rec_size;
  DMU_RET_TYPE res;
  u_int8_t     rec_buf[DMU_MAX_REC_SIZE];

  rec_size = SERIAL_READ(ttyNAV,rec_buf, DMU_RAW_PK_SIZE[DMU_ANGLE_MODE]);
  res = DMU_CheckValidPacket(rec_buf, rec_size, DMU_ANGLE_MODE);
  if(res != DMU_RET_OK) return res;
  DMU_FillAngleStruct(rec_buf, strct);

  return DMU_RET_OK;
}

DMU_RET_TYPE DMU_ReadNavData(DMU_Nav_Struct *strct)
{
  int rec_size;
  DMU_RET_TYPE res;
  u_int8_t     rec_buf[DMU_MAX_REC_SIZE];

  rec_size = SERIAL_READ(ttyNAV, rec_buf, DMU_RAW_PK_SIZE[DMU_NAV_MODE]);
  res = DMU_CheckValidPacket(rec_buf, rec_size, DMU_NAV_MODE);
  if(res != DMU_RET_OK) return res;
  DMU_FillNavStruct(rec_buf, strct);

  return DMU_RET_OK;
}

DMU_RET_TYPE DMU_QuerySerialAndVersion(u_int32_t *serial, char *version)
{
  const size_t CS = DMU_CMD_SERIAL;
  const size_t RS = DMU_RS_SERIAL;

  size_t       i;
  u_int8_t     cmd[DMU_CMD_SERIAL];
  DMU_RET_TYPE res;
  u_int8_t     rec_buf[DMU_MAX_REC_SIZE];

  DMU_FillCmd(cmd, DMU_CMD_GP);
  cmd[4] = 'D';

  DMU_AddCmdCheckSum(cmd,CS);

  SERIAL_FLUSH_RX(ttyNAV);
  res = DMU_ProcessCmd(cmd,CS,rec_buf,RS);
  if(res != DMU_RET_OK) return res;

  /* Check header */
  if(!DMU_CHK_HEAD(rec_buf)) return DMU_RET_BAD_HEADER;

  /* Check reply type */
  if(rec_buf[2] != 'D') return DMU_RET_WRONG_PACKET;

  /* Checksum */
  res = DMU_CheckChecksum(rec_buf, RS);
  if(res != DMU_RET_OK) return res;

  /* Fill outputs */
  *serial = rec_buf[3] << 8 | rec_buf[4] | rec_buf[5] << 24 | rec_buf[6] << 16;
  if(version != NULL) {
    for (i = 7; i < RS-2; i++ ) version[i-7] = rec_buf[i];
    version[i-7] = '\0';
  }
  return DMU_RET_OK;
}

DMU_RET_TYPE DMU_StartIronCalibration()
{
  const size_t CS = DMU_CMD_START_CALIB;
  const size_t RS = DMU_RS_START_CALIB;

  u_int8_t     cmd[DMU_CMD_START_CALIB];
  DMU_RET_TYPE res;
  u_int8_t     rec_buf[DMU_MAX_REC_SIZE];

  DMU_FillCmd(cmd, DMU_CMD_WC);
  cmd[4] = 0x00;  cmd[5] = 0x0C;

  DMU_AddCmdCheckSum(cmd,CS);

  res = DMU_ProcessCmd(cmd,CS,rec_buf,RS);
  if(res != DMU_RET_OK) return res;

  /* Check header and packet type */
  if(!DMU_CHK_HEAD(rec_buf)) return DMU_RET_BAD_HEADER;
  if(rec_buf[2] != 'W' || rec_buf[3] != 0x0C) return DMU_RET_WRONG_PACKET;

  /* Checksum */
  res = DMU_CheckChecksum(rec_buf, RS);
  if(res != DMU_RET_OK) return res;

  return DMU_RET_OK;
}

DMU_RET_TYPE DMU_CommitCalibration()
{
  const size_t CS = DMU_CMD_COMMIT_CALIB;
  u_int8_t     cmd[DMU_CMD_COMMIT_CALIB];

  DMU_FillCmd(cmd, DMU_CMD_WC);
  cmd[4] = 0x00;  cmd[5] = 0x0E;

  DMU_AddCmdCheckSum(cmd,CS);
  DMU_PrintRawBuffer(cmd,CS);
  DMU_PRINT("This should be equal to 0x55555743000E00A8 (size is %i)\n",CS);

  return DMU_WriteCmd(cmd, CS);
}

/* Return 0 if the answer is no else 1 if yes*/
static int DMU_AskUser(char *msg)
{
  char c;

  while(1)
  {
    DMU_PRINT(msg);
    DMU_PRINT("(y/n)");
    c = getchar();
    getchar();
    if(c == 'y' || c == 'Y')
      return 1;
    else
      if(c == 'n' || c == 'N')
      return 0;
    else DMU_PRINT("Anwer with y or n!\n");
    }
}

DMU_RET_TYPE DMU_InteractIronCalibration()
{
  const size_t RS = DMU_RS_CALIB_PARAMS;
  DMU_RET_TYPE res;
  u_int8_t     rec_buf[DMU_MAX_REC_SIZE];

  res = DMU_ChangeOutputRate(DMU_RATE_QUIET, 0);
  if(res != DMU_RET_OK) return res;

  if(!DMU_AskUser("Are you sure that the unit has been started 60s ago ?"))
  {
    DMU_PRINT("Please wait for 60s...\n");
    sleep(60);
    DMU_PRINT("Done!\n");
  }

  res = DMU_StartIronCalibration();
  if(res != DMU_RET_OK) return res;

  DMU_PRINT(DMU_MSG_PLEASE_ROTATE);

  res  = DMU_ReadAnswer(rec_buf, RS);
  if(res != DMU_RET_OK) return res;

  DMU_PrintRawBuffer(rec_buf, RS);

  /* Check everything */
  if(!DMU_CHK_HEAD(rec_buf)) return DMU_RET_BAD_HEADER;
  if(rec_buf[2] != 'C') return DMU_RET_WRONG_PACKET;

  /* Checksum */
  res = DMU_CheckChecksum(rec_buf, RS);
  if(res != DMU_RET_OK) return res;

  DMU_FillCalibStruct(rec_buf, &m_calib_result);
  DMU_PrintCalibResult();

  if(DMU_AskUser("Do you want to commit to the unit's EEPROM ?"))
  {
    res = DMU_CommitCalibration();
    if(res != DMU_RET_OK) return res;
    DMU_PRINT("The calibration factors have been commited to the EEPROM\n");
  }
  else
    DMU_PRINT("The calibration factors have **NOT** been commited to the EEPROM\n");

  sleep(1);
  DMU_PRINT(DMU_MSG_DISCONNECT_UNIT);
  sleep(1);
  DMU_PRINT(DMU_MSG_REPEAT_CALIB);
  sleep(1);
  DMU_PRINT(DMU_MSG_CALIB_DONE);
  return DMU_RET_OK;
}

DMU_Calib_Result DMU_GetCalibrationResult()
{
  return m_calib_result;
}

inline DMU_MODE_TYPE DMU_GetPacketType(u_int8_t *buf)
{
  char c = buf[2];

  if(c == DMU_MODE_CHAR[DMU_SCALED_MODE]) return DMU_SCALED_MODE;
  if(c == DMU_MODE_CHAR[DMU_ANGLE_MODE])  return DMU_ANGLE_MODE;
  if(c == DMU_MODE_CHAR[DMU_NAV_MODE])    return DMU_NAV_MODE;

  return -1;
}

/* ---  Reading thread functions */
inline void Sem_p(DMU_DEV_TYPE device)
{
  int res = 1;

  switch(device)
  {
    case DMU_DEV_NAV:
      res = pthread_mutex_lock(&m_mutex_nav);
      break;

    case DMU_DEV_GPS:
      res = pthread_mutex_lock(&m_mutex_gps);
      break;
    default:
      fprintf(stderr, "Sem_p -> Internal error!\n");
      exit(EXIT_FAILURE);
      break;
  }

  if(res != 0)
    DMU_PRINT("Sem_p failed\n");
}

inline void Sem_v(DMU_DEV_TYPE device)
{
  int res = 1;

  switch(device)
  {
    case DMU_DEV_NAV:
      res = pthread_mutex_unlock(&m_mutex_nav);
      break;

    case DMU_DEV_GPS:
      res = pthread_mutex_unlock(&m_mutex_gps);
      break;
    default:
      fprintf(stderr, "Sem_v -> Internal error!\n");
      exit(EXIT_FAILURE);
      break;
  }

  if (res != 0)
    DMU_PRINT("Sem_v failed\n");
}

/* delay_us must be smaller than 1000000us */
static void CorrectStamp(TIMEVAL *tv, unsigned int delay_us)
{
  if(tv->tv_usec >= delay_us)
    tv->tv_usec -= delay_us;
  else{
    tv->tv_sec -= 1;
    tv->tv_usec = 1000000 - (delay_us - tv->tv_usec);
  }
}

inline int DMU_ThreadRunning(DMU_DEV_TYPE dev)
{
  if(dev == DMU_DEV_NAV) return m_params.run_nav;
  if(dev == DMU_DEV_GPS) return m_params.run_gps;
  return -1;
}

inline TIMEVAL DMU_GetTimeItowChanged()
{
  TIMEVAL tv;
  Sem_p(DMU_DEV_GPS);
  tv = m_time_itow_changed;
  Sem_v(DMU_DEV_GPS);
  return tv;
}

/* if itow changed return 1 else 0 */
inline int DMU_StoreITOWChangeTime(TIMEVAL curr_itow, TIMEVAL curr_stamp)
{
  /* Detect if itow changed */
  if(Time_Compare(curr_itow, m_old_itow) != TIME_COMP_EQUAL)
  {
    Sem_p(DMU_DEV_GPS);
    m_time_itow_changed = curr_stamp;
    Sem_v(DMU_DEV_GPS);

    m_old_itow = curr_itow;
    return 1;
  }

  return 0;
}

/* Check if ITOW changed and correct timestamp if needed
 * return 1 if correction, -1 if not GPS old, 0 if nothing to do.
 * curr_stamp is the current timestamp of the NAV data, curr_itow: the itow of the NAV data
   old_itow a pointer on the old itow of the NAV data */
int DMU_CorrGPSTimestamp(TIMEVAL curr_stamp, TIMEVAL curr_itow, TIMEVAL *old_itow)
{
  typedef  enum {WAIT_NO, WAIT_RMC, WAIT_GGA} WAIT_TYPE;
  static   int wait = WAIT_NO;
  static   int gps_id = 0;
  static   TIMEVAL true_stamp;
  DMU_REAL rmc_stamp_dbl;
  DMU_REAL gga_stamp_dbl;
  DMU_REAL rmc_utc_dbl;
  DMU_REAL gga_utc_dbl;
  DMU_REAL curr_stamp_dbl;
  int      ret;

  ret = 0;

  if(wait == WAIT_RMC)
  {
    Sem_p(DMU_DEV_GPS);
    if(gps_id != GPS_GetIDRMC()) /* Wait for a new GPS data */
    {
      if(m_gga_received)
      {
        /* RMC and GGA finally arrived, we can assign the correct time stamp */
        GPS_SetMeasTimeStampRMC(true_stamp);
        GPS_SetMeasTimeStampGGA(true_stamp);
        Sem_v(DMU_DEV_GPS);
        wait = WAIT_NO;
        /*DMU_PRINT("Corr WAIT_RMC\n");*/
        return 1;
      }
      else {
        wait = WAIT_GGA; /* RMC has bee received but not GGA -> wait GGA */
        /*DMU_PRINT("Set WAIT_GGA in WAIT_RMC\n");*/
      }
    }
    Sem_v(DMU_DEV_GPS);
  }
  else
  if(wait == WAIT_GGA)
  {
    if(m_gga_received)
    {
      /* GGA finally arrived */
      Sem_p(DMU_DEV_GPS);
      GPS_SetMeasTimeStampRMC(true_stamp);
      GPS_SetMeasTimeStampGGA(true_stamp);
      Sem_v(DMU_DEV_GPS);
      wait = WAIT_NO;
      /*Time_Print(stdout,true_stamp,"stmp");
      DMU_DBG("Corr WAIT_GGA\n");*/
      return 1;
    }
  }

  /* Check if ITOW changed */
  if(Time_Compare(curr_itow, *old_itow) != TIME_COMP_EQUAL)
  {
    true_stamp = curr_stamp; /* Store in case GGA and/or RMC have not yet been received */
    curr_stamp_dbl = Time_FromTimeval(curr_stamp);

    Sem_p(DMU_DEV_GPS);
    rmc_stamp_dbl = Time_FromTimeval(GPS_GetAvailStampRMC());
    gga_stamp_dbl = Time_FromTimeval(GPS_GetAvailStampGGA());
    rmc_utc_dbl   = Time_FromTimeval(GPS_GetUTCRMC());
    gga_utc_dbl   = Time_FromTimeval(GPS_GetUTCGGA());

    /*DMU_DBG("Stamp diff: %f\n", rmc_stamp_dbl - gga_stamp_dbl);*/
    /*DMU_DBG("UTC diff: %f\n", rmc_utc_dbl - gga_utc_dbl);*/

    /* Check if the GPS data is fresh */
    if(Time_CheckCloseDbl(rmc_stamp_dbl, curr_stamp_dbl, 0.11))
    {
      if(m_gga_received)
      {
        GPS_SetMeasTimeStampRMC(curr_stamp);
        GPS_SetMeasTimeStampGGA(curr_stamp);
        /*DMU_DBG("Corr direct\n");*/
        ret = 1;
      }
      else /* GGA not yet received */
      {
        wait = WAIT_GGA;
        /*DMU_DBG("Set WAIT_GGA\n");*/
        ret = -1;
      }
    }
    else
    {
      /* We have to wait until new GPS data is available */
      gps_id = GPS_GetIDRMC();
      wait = WAIT_RMC;
      /*DMU_DBG("Set WAIT_RMC\n");*/
      ret = -1;
    }

    Sem_v(DMU_DEV_GPS);

    *old_itow = curr_itow;
  }

  return ret;
}

static void *ttyThreadWaitCalib (void *params)
{
  const size_t RS = DMU_RS_CALIB_PARAMS;
  u_int8_t rec_buf[DMU_MAX_REC_SIZE];

  DMU_Calib_Result *calib = (DMU_Calib_Result*) params;

  DMU_PRINT(DMU_MSG_PLEASE_ROTATE);
  DMU_DBG("Waiting for the parameters ...\n");

  /* Block until RS char have been received */
  calib->retval = DMU_ReadAnswer(rec_buf, RS);
  if( calib->retval != DMU_RET_OK) return NULL;

  DMU_PrintRawBuffer(rec_buf, RS);

  /* Check everything */
  if(!DMU_CHK_HEAD(rec_buf)) {
    calib->retval = DMU_RET_BAD_HEADER;
    return NULL;
  }
  if(rec_buf[2] != 'C') {
    calib->retval = DMU_RET_WRONG_PACKET;
    return NULL;
  }

  /* Checksum */
  calib->retval = DMU_CheckChecksum(rec_buf, RS);
  if(calib->retval != DMU_RET_OK) return NULL;

  /* 4 bytes fixed point values */
  DMU_FillCalibStruct(rec_buf, calib);
  calib->retval = DMU_RET_OK;

  DMU_PRINT(DMU_MSG_CALIB_DONE);

  return NULL;
}

/* Make sure the unit is quiet before calling this function */
DMU_RET_TYPE DMU_LaunchIronCalibration()
{
  DMU_RET_TYPE res;

  if(DMU_ThreadRunning(DMU_DEV_NAV))
    return DMU_RET_COMM_BUSY;

  DMU_PRINT("Make sure that the unit has been started 60s ago !\n");

  res = DMU_StartIronCalibration();
  if(res != DMU_RET_OK) return res;

  m_calib_result.retval = DMU_RET_COMM_WAITING;

  res = pthread_create(&m_tty_thread_calib, NULL, ttyThreadWaitCalib, (void*) &m_calib_result);
  if (res != 0)
  {
    perror("DMU_LaunchContThread -> pthread_create error!\n");
    exit(EXIT_FAILURE);
  }

  return DMU_RET_OK;
}

/* Call this function to check if the calibration is finished */
int DMU_IsCalibrationDone()
{
  if(m_calib_result.retval == DMU_RET_OK)
    return 1;

  return 0;
}

GPS_gga DMU_GGA_GetCopy()
{
  GPS_gga gga;
  GPS_gga *gga_ptr = GPS_GetGgaPtr();

  Sem_p(DMU_DEV_GPS);
  gga = *gga_ptr;
  Sem_v(DMU_DEV_GPS);

  return gga;
}

GPS_rmc DMU_RMC_GetCopy()
{
  GPS_rmc rmc;
  GPS_rmc *rmc_ptr = GPS_GetRmcPtr();

  Sem_p(DMU_DEV_GPS);
  rmc = *rmc_ptr;
  Sem_v(DMU_DEV_GPS);

  return rmc;
}

/* GPRMC arrives first then GPGGA */
static void *ttyThreadGPSCapture (void *params)
{
  DMU_ThreadParams *parameters = (DMU_ThreadParams *) params;
  unsigned int   delay_us_gga;
  unsigned int   delay_us_rmc;
  char           rec_buf[2*GGA_STR_LEN];
  char           *ptr;
  int            start,res;
  TIMEVAL        stamp;

  /* Correction for transmission time 8N1 = 10 bits*/
  delay_us_gga = ComputeTransTime(GGA_STR_LEN, parameters->baudrate_gps);
  DMU_DBG("Transmission time (GGA): %u us\n", delay_us_gga);

  delay_us_rmc = ComputeTransTime(RMC_STR_LEN, parameters->baudrate_gps);
  DMU_DBG("Transmission time (RMC): %u us\n", delay_us_rmc);

  parameters->run_gps = 1;
  start = 0;
  ptr = rec_buf;
  GPS_ClearGPSid();

  while(parameters->run_gps)
  {
    SERIAL_READ(ttyGPS,ptr,1);

    if(*ptr == '$') {
      rec_buf[0] = '$';
      ptr = (char *) (rec_buf+1);
      start = 1;  /* we are in sync now */
    }
    else
    if(*ptr == 0xA)
    {
      if(start)
      {
        Time_gettimeofday(&stamp);  /* stamp here (same policy as NAV stamping) */
        *ptr = '\0';
        /*DMU_PRINT("%s\n",rec_buf);*/

        if(!strncmp("$GPGGA", rec_buf, 6))
        {
          CorrectStamp(&stamp, delay_us_gga);
          Sem_p(DMU_DEV_GPS);
          res = GPS_gps_parse_gga(rec_buf, stamp);
          m_gga_received = 1;
          Sem_v(DMU_DEV_GPS);
        }
        else
        if(!strncmp("$GPRMC", rec_buf, 6))
        {
          CorrectStamp(&stamp, delay_us_rmc);
          Sem_p(DMU_DEV_GPS);
          res = GPS_gps_parse_rmc(rec_buf, stamp);
          m_gga_received = 0;
          Sem_v(DMU_DEV_GPS);
        }
        else
        if(!strncmp("$GPTXT", rec_buf, 6))
        {
          DMU_DBG("GPTXT received\n");
        }
        else
        {
          res = 0;
        }

        if(m_pGPSCallback != NULL) (*m_pGPSCallback)(GPS_GetGgaPtr(), GPS_GetRmcPtr());

      }
      start = 0; /* from now wait until '$' is received */
    }
    else
      ptr++;

#ifndef NDEBUG
    if(ptr > rec_buf + 2*GGA_STR_LEN ) {
      DMU_DBG("ttyThreadGPSCapture: Buffer Overrun! (is the NAV420 gps connected to the serial port?)\n");
      exit(EXIT_FAILURE);
    }
#endif
  }

  return NULL;
}

void DMU_InitTransTime()
{
  m_delay_us = DMU_ComputeTransTime(m_mode, m_params.baudrate_nav420);
  DMU_DBG("Transmission time: %u us\n", m_delay_us);
}

/* return 0 if error else ok */
int DMU_WaitPacket(DMU_Shared_Mem *shm)
{
  static DMU_MODE_TYPE  old_mode;
  static TIMEVAL        old_stamp = {0,0};
  static int            first = 1;

  int                   n, rSize;
  DMU_RET_TYPE          ret;
  TIMEVAL               stamp;
  DMU_REAL              h = 0.0, period;

  if(first)
  {
    old_mode = m_mode;
    first = 0;
  }

  rSize = DMU_RAW_PK_SIZE[m_mode];
  n = SERIAL_READ(ttyNAV, shm->rec_buf, rSize);

  Time_gettimeofday(&stamp);

  /*CorrectStamp(&stamp, m_delay_us);*/    /* !!! finally we don't remove the comm time remove the communication time */
  h = Time_DiffDbl(stamp,old_stamp);

  period = m_output_rate / 100.0;
  if(h > period && old_stamp.tv_sec != 0)
  {
    stamp = Time_Add(old_stamp, Time_FromDbl(period));
  }

  if(old_mode != m_mode)
  {
    DMU_Resync();
    old_mode = m_mode;
    old_stamp.tv_sec  = 0;
    old_stamp.tv_usec = 0;
    return 0;
  }

  old_stamp = stamp;   /* update the time and the mode for the next function call */
  old_mode  = m_mode;

  ret = DMU_CheckValidPacket(shm->rec_buf, n, m_mode);
  if(ret != DMU_RET_OK)
  {
    if(ret == DMU_RET_BAD_HEADER) DMU_Resync();
    /* DMU_DBG("Stamp: %f, Returned error: ", Time_FromTimeval(stamp));
       DMU_PrintRetType(ret);*/
    old_stamp.tv_sec = 0;
    old_stamp.tv_usec = 0;
    return 0;
  }
  else /* everything is ok */
  {
    /*shm->stamp.tv_sec  = stamp.tv_sec;
    shm->stamp.tv_usec = stamp.tv_usec;*/
    shm->stamp = stamp;
    /*    printf("sec: %li\n", shm->stamp.tv_sec);*/
    shm->meas_id = shm->meas_id + 1;
  }

  return 1;
}


static void *ttyThreadNavCapture (void *params)
{
  int            n, rSize;
  u_int8_t       rec_buf[DMU_MAX_REC_SIZE];
  DMU_MODE_TYPE  old_mode = m_mode;
  DMU_RET_TYPE   ret;
  TIMEVAL        stamp, old_stamp = {0,0};
  DMU_REAL       h = 0.0, period;
  DMU_ThreadParams *parameters = (DMU_ThreadParams *) params;

  DMU_Resync();

  /* Correction for transmission time 8N1 = 10 bits*/
  DMU_InitTransTime();

  Sem_p(DMU_DEV_NAV);
  m_shared_mem.meas_id = 0;
  Sem_v(DMU_DEV_NAV);

  parameters->run_nav = 1;

  while(parameters->run_nav)
  {
    rSize = DMU_RAW_PK_SIZE[m_mode];

    n = SERIAL_READ(ttyNAV, rec_buf, rSize);

    Time_gettimeofday(&stamp);
    CorrectStamp(&stamp, m_delay_us);    /* remove the communication time */
    h = Time_DiffDbl(stamp,old_stamp);

    period = m_output_rate / 100.0;
    if(h > period && old_stamp.tv_sec != 0)
    {
      stamp = Time_Add(old_stamp, Time_FromDbl(period));  /* Add period to old_stamp */
    }

    old_stamp = stamp;

    if(old_mode != m_mode) {
      DMU_Resync();
      old_mode = m_mode;
      old_stamp.tv_sec  = 0;
      old_stamp.tv_usec = 0;
      continue;
    }

    old_mode  = m_mode;

    ret = DMU_CheckValidPacket(rec_buf, n, m_mode);
    if(ret != DMU_RET_OK)
    {
      if(ret == DMU_RET_BAD_HEADER) DMU_Resync();
      DMU_DBG("Stamp: %f, Returned error: ", Time_FromTimeval(stamp));
      DMU_PrintRetType(ret);
    }
    else
    {
      Sem_p(DMU_DEV_NAV);
      memcpy(m_shared_mem.rec_buf, rec_buf, rSize);
      m_shared_mem.stamp.tv_sec  = stamp.tv_sec;
      m_shared_mem.stamp.tv_usec = stamp.tv_usec;
      m_shared_mem.meas_id += 1;
      Sem_v(DMU_DEV_NAV);

      if (m_pNAVCallback != NULL) (*m_pNAVCallback)(&m_shared_mem);
    }
  }

  return NULL;
}

inline void DMU_GetThreadLatestPacket(u_int8_t *buf, TIMEVAL *tv, unsigned long *meas_id)
{
  Sem_p(DMU_DEV_NAV);
  memcpy(buf, m_shared_mem.rec_buf, DMU_RAW_PK_SIZE[m_mode]);
  tv->tv_sec  = m_shared_mem.stamp.tv_sec;
  tv->tv_usec = m_shared_mem.stamp.tv_usec;
  *meas_id    = m_shared_mem.meas_id;
  Sem_v(DMU_DEV_NAV);
}

inline void DMU_GetThreadNewPacket(u_int8_t *buf, TIMEVAL *tv, unsigned long *meas_id)
{
  unsigned long new_id;

  DMU_GetThreadLatestPacket(buf, tv, &new_id);

  while(*meas_id == new_id) {
    usleep(1000); /* cpu friendly */
    DMU_GetThreadLatestPacket(buf, tv, &new_id);
  }
  *meas_id = new_id;
}

void InitThreadAttr(pthread_attr_t *attr, int root)
{
 int            res;
 struct sched_param    sched;

 /* Increase the priority of the thread (for better time stamping) */
 res = pthread_attr_init(attr);
 if (res != 0)
  {
    perror("InitThreadAttr -> pthread_attr_init error!\n");
    exit(EXIT_FAILURE);
  }

  /* set the priority; others are unchanged */
  if(root)
  {
    res = pthread_attr_setinheritsched(attr, PTHREAD_EXPLICIT_SCHED);
    if (res != 0)
    {
      perror("InitThreadAttr -> pthread_attr_setinheritsched error!\n");
      exit(EXIT_FAILURE);
    }

    res = pthread_attr_setschedpolicy(attr, SCHED_FIFO);
    if (res != 0)
    {
      perror("InitThreadAttr -> pthread_attr_setschedpolicy error!\n");
      exit(EXIT_FAILURE);
    }

    sched.sched_priority = sched_get_priority_max(SCHED_FIFO);

    if(sched.sched_priority == -1)
    {
      perror("InitThreadAttr -> sched_get_priority_max failed!\n");
      exit(EXIT_FAILURE);
    }

    DMU_DBG("InitThreadAttr -> Set sched priority to %i\n", sched.sched_priority);

    /* set the new scheduling param */
    res = pthread_attr_setschedparam (attr, &sched);
    if (res != 0)
    {
      perror("InitThreadAttr -> pthread_attr_setschedparam error!\n");
      exit(EXIT_FAILURE);
    }
  }
}

/* if root == 1 increase priority for the thread */
DMU_RET_TYPE DMU_LaunchNavContThread(DMU_MODE_TYPE mode, int root, void (*pCallback) (DMU_Shared_Mem *))
{
  int            res;
  DMU_RET_TYPE   ret;

  if(ttyNAV == -1) {
    fprintf(stderr, "DMU_LaunchNavThread -> Port not open\n");
    exit(EXIT_FAILURE);
  }

  ret = DMU_ChangePacketType(mode, 0);
  if(ret != DMU_RET_OK) return ret;

  m_mode = mode;

  /* Finally create the thread */
  m_pNAVCallback = pCallback;

  InitThreadAttr(&m_thread_attr_nav, root);

  res = pthread_create(&m_tty_thread_nav, &m_thread_attr_nav, ttyThreadNavCapture, (void*) &m_params);
  if (res != 0)
  {
    perror("DMU_LaunchNavThread -> pthread_create error!\n");
    DMU_PRINT("Are you root ?\n");
    exit(EXIT_FAILURE);
  }

  return DMU_RET_OK;
}

/* if root != 0 increase priority for the thread */
DMU_RET_TYPE DMU_LaunchGPSContThread(int root, void (*pCallback) (GPS_gga *, GPS_rmc *))
{
  int res;

  if(ttyGPS == -1) {
    fprintf(stderr, "DMU_LaunchGPSContThread -> Port not open\n");
    exit(EXIT_FAILURE);
  }

  GPS_gps_alloc();

  /* Finally create the thread */
  m_pGPSCallback = pCallback;

  InitThreadAttr(&m_thread_attr_gps, root);
  res = pthread_create(&m_tty_thread_gps, &m_thread_attr_gps, ttyThreadGPSCapture, (void*) &m_params);
  if (res != 0)
  {
    perror("DMU_LaunchGPSContThread -> pthread_create error!\n");
    DMU_PRINT("Are you root ?\n");
    exit(EXIT_FAILURE);
  }

  return DMU_RET_OK;
}

void KillThread(pthread_t *thread, pthread_attr_t *attr)
{
  int  res;
  void *value_ptr;

  DMU_DBG("Killing thread ...\n");
  res = pthread_join(*thread, &value_ptr);

  if(res != 0) {
    perror("DMU_KillThread -> could not join thread!");
    exit (EXIT_FAILURE);
  }

  res = pthread_attr_destroy(attr);
  if(res != 0) {
    perror("DMU_KillThread -> destroy thread attr!");
    exit (EXIT_FAILURE);
  }
}

void DMU_KillThread(DMU_DEV_TYPE dev)
{
  if(dev == DMU_DEV_NAV)
  {
    m_params.run_nav = 0;
    KillThread(&m_tty_thread_nav, &m_thread_attr_nav);
    DMU_DBG("Nav Thread killed\n");
  }

  if(dev == DMU_DEV_GPS)
  {
    m_params.run_gps = 0;
    KillThread(&m_tty_thread_gps, &m_thread_attr_gps);
    DMU_DBG("GPS Thread killed\n");
  }
}

/* --- Print functions */
void DMU_PrintRawBuffer(u_int8_t *buf, size_t size)
{
  size_t  i;
  DMU_PRINT("(%i) ", size);
  for(i = 0; i < size-1; i++) DMU_PRINT( "%.2X, ", buf[i]);
  DMU_PRINT("%.2X\n", buf[i]);
}

void DMU_PrintRawPacket(u_int8_t *buf, DMU_MODE_TYPE mode)
{
  DMU_PrintRawBuffer(buf, DMU_RAW_PK_SIZE[mode]);
}

void DMU_PrintRetType(DMU_RET_TYPE val)
{
  DMU_PRINT(DMU_RET_STRING[val+1]);
  DMU_PRINT("\n");
}

void DMU_PrintAccelerations(EPOINT3D *acc)
{
  DMU_PRINT("X-Axis Acceleration:\t%.3f [m/s^2]\n", acc->x);
  DMU_PRINT("Y-Axis Acceleration:\t%.3f [m/s^2]\n", acc->y);
  DMU_PRINT("Z-Axis Acceleration:\t%.3f [m/s^2]\n", acc->z);
}

void DMU_PrintAxisStruct(EPOINT3D *strct)
{
  DMU_PRINT("X-Axis:\t%.3f \n", strct->x);
  DMU_PRINT("Y-Axis:\t%.3f \n", strct->y);
  DMU_PRINT("Z-Axis:\t%.3f \n", strct->z);
}

void DMU_PrintRates(EPOINT3D *rate)
{
  DMU_PRINT("X-Axis Angular Rate:\t%.3f [deg/s]\n", RAD2DEG(rate->x));
  DMU_PRINT("Y-Axis Angular Rate:\t%.3f [deg/s]\n", RAD2DEG(rate->y));
  DMU_PRINT("Z-Axis Angular Rate:\t%.3f [deg/s]\n", RAD2DEG(rate->z));
}

void DMU_PrintMagneticFields(EPOINT3D *mag)
{
  DMU_PRINT("X-Axis Magnetic Field:\t%.3f [Gauss]\n", mag->x);
  DMU_PRINT("Y-Axis Magnetic Field:\t%.3f [Gauss]\n", mag->y);
  DMU_PRINT("Z-Axis Magnetic Field:\t%.3f [Gauss]\n", mag->z);
}

void DMU_PrintTemperatures(EPOINT3D *temp)
{
  DMU_PRINT("X-Axis Temperature:\t%.2f [Celsius]\n",
            KELVIN_TO_CELSIUS(temp->x));
  DMU_PRINT("Y-Axis Temperature:\t%.2f [Celsius]\n",
            KELVIN_TO_CELSIUS(temp->y));
  DMU_PRINT("Z-Axis Temperature:\t%.2f [Celsius]\n",
            KELVIN_TO_CELSIUS(temp->z));
}

void DMU_PrintAngles(EPOINT3D *angle, DMU_MODE_TYPE mode)
{
  DMU_PRINT("Roll (x):\t\t%.3f [deg] (%f rad)\n", RAD2DEG(angle->x), angle->x);
  DMU_PRINT("Pitch (y):\t\t%.3f [deg] (%f rad)\n", RAD2DEG(angle->y), angle->y);
  if(mode == DMU_ANGLE_MODE)
    DMU_PRINT("Yaw (mag north):\t%.3f [deg] (%f rad)\n", RAD2DEG(angle->z), angle->z);
  else
    DMU_PRINT("Yaw (true north):\t%.3f [deg] (%f rad)\n", RAD2DEG(angle->z), angle->z);
}

void DMU_PrintVelocity(EPOINT3D *vel)
{
  DMU_PRINT("X-Axis Velocity:\t%.3f [m/s]\n", vel->x);
  DMU_PRINT("Y-Axis Velocity:\t%.3f [m/s]\n", vel->y);
  DMU_PRINT("Z-Axis Velocity:\t%.3f [m/s]\n", vel->z);
}

void DMU_PrintITOW(TIMEVAL *itow)
{
  DMU_PRINT("GPS ITOW:\t\t%.6f [s]\n", itow->tv_sec +1e-6*itow->tv_usec);
}

void DMU_PrintCPUTemp(DMU_REAL temp)
{
  DMU_PRINT("CPU Board Temperature:\t%.2f [Celsius]\n",
            KELVIN_TO_CELSIUS(temp));
}

void DMU_PrintBIT(u_int16_t bit)
{
  DMU_PRINT("BIT:\t\t\t0x%.4X (%d)\n--\n", bit, bit);
  DMU_PRINT("  Turn detect:\t\t%s", (bit & DMU_MSK_TURN_DETECT) ?\
      "turning":"NOT turning" );

  DMU_PRINT("\n  Comm transmit error:\t%s", (bit & DMU_MSK_TX_ERR) ?\
      "ERROR":"ok" );

  DMU_PRINT("\n  GPS status:\t\t%s", (bit & DMU_MSK_GPS_STATUS) ?\
      "NOT valid":"valid" );

  DMU_PRINT("\n  Algorithm initializ.:\t%s",(bit & DMU_MSK_ALGO_INIT)?\
      "NOT ready":"ready" );

  DMU_PRINT("\n  1 PPS Signal:\t\t%s", (bit & DMU_MSK_PPS_LOCK) ?\
      "NOT locked":"locked" );

  DMU_PRINT("\n  EEPROM integrity:\t%s", (bit & DMU_MSK_EEPROM_INT) ?\
      "CORRUPTED":"ok" );

  DMU_PRINT("\n  Magnet. calib. valid:\t%s",(bit & DMU_MSK_MAG_CAL)?\
      "NOT valid":"valid" );

  DMU_PRINT("\n  User port receive err:%s",(bit & DMU_MSK_USR_PORT)?\
      "ERROR":"ok" );

  DMU_PRINT("\n  Algo. accuracy:\t");

  switch (DMU_GET_ALGO_ACCURACY(bit))
  {
    case DMU_ACCU_FULL:
      DMU_PRINT("Full accuracy (GPS available)");
      break;
    case DMU_ACCU_LOW_HIGH:
      DMU_PRINT("low accuracy NAV/high accuracy AHRS");
      break;
    case DMU_ACCU_LOW:
      DMU_PRINT("low accuracy AHRS");
      break;
    case DMU_ACCU_INIT:
      DMU_PRINT("AHRS initialization");
      break;
    default:
      DMU_PRINT("AHRS initialization");
      break;
  }
  DMU_PRINT("\n\n");
}

void DMU_PrintScaledStruct(DMU_Scaled_Struct *strct)
{
  DMU_PRINT("** DMU_Scaled_Struct ('%c') **\n",\
      DMU_MODE_CHAR[DMU_SCALED_MODE]);
  DMU_PrintAccelerations(&strct->accel);
  DMU_PrintRates(&strct->rate);
  DMU_PrintMagneticFields(&strct->mag);
  DMU_PrintTemperatures(&strct->temp);
  DMU_PRINT("CPU temperature:\t%.2f [Celsius]\n",\
      KELVIN_TO_CELSIUS(strct->cpu_temp));
  DMU_PrintITOW(&strct->gps_itow);
  DMU_PrintBIT(strct->bit);
}

void DMU_PrintAngleStruct(DMU_Angle_Struct *strct)
{
  DMU_PRINT("** DMU_Angle_Struct ('%c') **\n",\
      DMU_MODE_CHAR[DMU_ANGLE_MODE]);
  DMU_PrintAngles(&strct->angle, DMU_ANGLE_MODE);
  DMU_PrintRates(&strct->rate);
  DMU_PrintAccelerations(&strct->accel);
  DMU_PrintMagneticFields(&strct->mag);
  DMU_PRINT("Temperature:\t\t%.2f [Celsius]\n",\
      KELVIN_TO_CELSIUS(strct->temp));

  DMU_PrintITOW(&strct->gps_itow);
  DMU_PrintBIT(strct->bit);
}

void DMU_PrintNavStruct(DMU_Nav_Struct *strct)
{
  DMU_PRINT("** DMU_Nav_Struct ('%c') **\n", DMU_MODE_CHAR[DMU_NAV_MODE]);
  DMU_PrintAngles(&strct->angle, DMU_NAV_MODE);
  DMU_PrintRates(&strct->rate);
  DMU_PrintVelocity(&strct->velocity);
  GPS_PrintGPS(&strct->coord);

  DMU_PrintITOW(&strct->gps_itow);
  DMU_PrintBIT(strct->bit);
}

void DMU_PrintSerialAndVersion()
{
  char      version[30];
  u_int32_t serial = 0;
  DMU_RET_TYPE res;

  res = DMU_QuerySerialAndVersion(&serial,version);

  if(res != DMU_RET_OK) {
    DMU_PRINT("An error occured in DMU_PrintSerialAndVersion\n");
    DMU_PrintRetType(res);
  }
  else {
    DMU_PRINT("The serial number is: %i\n", serial);
    DMU_PRINT("The version of the firmware is: %s\n", version);
  }
}

void DMU_PrintCalibResult()
{
  DMU_PRINT("Bias x: %.6f, Bias y: %.6f, Scale ratio: %.6f \n", m_calib_result.bias_x, m_calib_result.bias_y, m_calib_result.scale);
  DMU_PRINT("Retval: ");
  DMU_PrintRetType(m_calib_result.retval);
}

void DMU_PrintLocalOrigin(DMU_PoseZero_Struct *strct)
{
  GPS_PrintGPS(&strct->gps);
  DMU_PRINT("Roll (x):\t\t%.3f [deg] (%f rad)\n", RAD2DEG(strct->rpy.x), strct->rpy.x);
  DMU_PRINT("Pitch (y):\t\t%.3f [deg] (%f rad)\n", RAD2DEG(strct->rpy.y), strct->rpy.y);
  DMU_PRINT("Heading:\t%.3f [deg] (%f rad)\n", RAD2DEG(strct->rpy.z), strct->rpy.z);

  DMU_PRINT("Declination: %f\n", strct->declination);
  DMU_PRINT("Earth radius: %f\n", strct->radius);
}

void DMU_comp_geomagTV(TIMEVAL time, GPS_Struct gps, double *myDeclination)
{
  DMU_comp_geomag(Time_GeomagDate(time), gps, myDeclination);
}
