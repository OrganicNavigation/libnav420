/***************************************************************************
                        nav420_struct.h  -  description
                            -------------------
  begin                : December 2005
  copyright            : (C) 2005 by Pierre Lamon
  email                : pierre.lamon@epfl.ch

  description          : define the main structures and types for the nav420
 ***************************************************************************/
#ifndef NAV420_STRUCT_LIB_H
#define NAV420_STRUCT_LIB_H

#ifndef DMU_FIELD_VOID
#define DMU_FIELD_VOID   9999.9  /* used for uninitialized float */
#endif

#ifndef DMU_GRAVITY
#define DMU_GRAVITY      9.81
#endif

#ifndef DMU_MAX_REC_SIZE
#define DMU_MAX_REC_SIZE 37
#endif

#include <elrob/Etypes.h>
#include <elrob/libgps_struct.h>

/* Exported structures */
typedef double DMU_REAL;

typedef unsigned short  u_int16_t;
typedef unsigned char   u_int8_t;

/** DMU_SCALED_MODE -> raw data, DMU_ANGLE_MODE -> run Kalman filter to estimates the angles, DMU_NAV_MODE -> 6DOF's with GPS*/
typedef enum _DMU_MODE_TYPE {DMU_SCALED_MODE = 0, DMU_ANGLE_MODE, DMU_NAV_MODE} DMU_MODE_TYPE;
typedef enum _DMU_RET_TYPE {DMU_RET_OK = -1,DMU_RET_BAD_CHKSUM = 0,DMU_RET_BAD_ARG,\
  DMU_RET_WRONG_PACKET,DMU_RET_BAD_HEADER,DMU_RET_BAD_BUFSIZE,\
      DMU_RET_COMM_ERR,DMU_RET_WRITE_ERROR,DMU_RET_COMM_BUSY,DMU_RET_COMM_WAITING} DMU_RET_TYPE;

typedef enum _DMU_RATE_TYPE {DMU_RATE_QUIET=0,DMU_RATE_100=1,DMU_RATE_50=2,DMU_RATE_25=4,\
  DMU_RATE_20=5,DMU_RATE_10=10,DMU_RATE_2=50 } DMU_RATE_TYPE;

typedef enum _DMU_ACCU_TYPE {DMU_ACCU_FULL,DMU_ACCU_LOW_HIGH,DMU_ACCU_LOW,DMU_ACCU_INIT} DMU_ACCU_TYPE;
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
  EPOINT3D        rpy;             /* initial roll pitch and heading (no declination considered) */
  DMU_REAL        declination;     /* depends on lat long altitude and time (use DMU_comp_geomag_now function to init) */
  DMU_REAL        radius;          /* do not modify by hand, use DMU_SetLocalOrigin instead*/
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

/** The structure containing a 'Angle' packet. The heading angle corresponds to the magnetic north */
typedef struct _DMU_Angle_Struct{
  EPOINT3D  angle;
  EPOINT3D  rate;
  EPOINT3D  accel;
  EPOINT3D  mag;
  DMU_REAL  temp;
  TIMEVAL   gps_itow;
  u_int16_t bit;
}DMU_Angle_Struct;

/** The structure containing a 'Angle' packet. The heading angle corresponds to the true north */
typedef struct _DMU_Nav_Struct{
  EPOINT3D  angle;
  EPOINT3D  rate;
  EPOINT3D  velocity;
  GPS_Struct       coord;
  TIMEVAL          gps_itow;
  u_int16_t        bit;
}DMU_Nav_Struct;

#endif
