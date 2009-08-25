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


void DMU_PRINT(const char *format, ...);
void DMU_DBG(const char *format, ...);

#endif
