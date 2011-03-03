/***************************************************************************
                        nav420_tools.h  -  description
                            -------------------
  begin                : December 2005
  copyright            : (C) 2005 by Pierre Lamon
  email                : pierre.lamon@epfl.ch

  description          : tools for converting GPS into xyz coordinates
 ***************************************************************************/

#ifndef NAV420_TOOLS_H
#define NAV420_TOOLS_H

#include <linux/types.h>

#include <libelrob/gps-nmea.h>

#include "nav420.h"

#define DMU_HEADER_CHAR 'U'
#define DMU_CHK_HEAD(raw)     (((raw)[0] == DMU_HEADER_CHAR) && \
                              ((raw)[1] == DMU_HEADER_CHAR))

/* BIT flags */
#define DMU_MSK_TURN_DETECT (1<<3)
#define DMU_MSK_TX_ERR      (1<<4)
#define DMU_MSK_GPS_STATUS  (1<<6)
#define DMU_MSK_ALGO_INIT   (1<<8)
#define DMU_MSK_PPS_LOCK    (1<<9)
#define DMU_MSK_EEPROM_INT  (1<<10)
#define DMU_MSK_MAG_CAL     (1<<11)
#define DMU_MSK_USR_PORT    (1<<12)

#define DMU_GET_ALGO_ACCURACY(bit_) (((bit_)>>14))
#define DMU_IS_ACCU_FULL(bit_)      (DMU_GET_ALGO_ACCURACY(bit_) == DMU_ACCU_FULL)
#define DMU_IS_3DGPS_VALID(bit_)    (((bit_) & DMU_MSK_GPS_STATUS) == DMU_MSK_GPS_STATUS ? (0) : (1))

#define DMU_SEP                   ('\t')
#define DMU_DUMP_AXIS(strm,axis)  fprintf((strm), "%.8f%c%.8f%c%.8f%c", (axis).x, DMU_SEP, (axis).y, DMU_SEP, (axis).z, DMU_SEP)
#define DMU_DUMP_GPS(strm,gps)    fprintf((strm), "%.8f%c%.8f%c%.8f%c", (gps).latitude, DMU_SEP, (gps).longitude, DMU_SEP, (gps).altitude, DMU_SEP)
#define DMU_DUMP_REAL(strm,real)  fprintf((strm), "%.8f%c", (real), DMU_SEP)
#define DMU_DUMP_TIMEVAL(strm,tv) fprintf((strm), "%.6f%c", (tv).tv_sec + 1e-6*(tv).tv_usec, DMU_SEP)
#define DMU_DUMP_INT(strm,val)    fprintf((strm), "%i%c", (val), DMU_SEP)

/** Convert GPS coordinates into local coordinates.
 *  the local frame is defined as follows: x points towards true north
 *  y points west and z points up (// to the gravity field). */
EPOINT3D DMU_GPSToLocalXYZ(GPS_Struct *gps);

/** Set the origin of the local map */
void DMU_SetLocalOrigin(GPS_Struct *orig, EPOINT3D *rpy, DMU_REAL declination);

/** Get the origin of the local map */
DMU_PoseZero_Struct DMU_GetLocalOrigin();

/** One step mean update for a EPOINT3D */
void DMU_UpdateAxisMean(unsigned long n, EPOINT3D *new_data, EPOINT3D *old_data);

void DMU_UpdateScaledMean(unsigned long n, DMU_Scaled_Struct *new_data, DMU_Scaled_Struct *old_data);
void DMU_UpdateAngleMean(unsigned long n, DMU_Angle_Struct *new_data, DMU_Angle_Struct *old_data);
void DMU_UpdateNavMean(unsigned long n, DMU_Nav_Struct *new_data, DMU_Nav_Struct *old_data);

void DMU_UpdateAxisVariance(unsigned long n,
                            EPOINT3D *new_axis,
                            EPOINT3D *axis_avg,
                            EPOINT3D *old_avg,
                            EPOINT3D *axis_var_upd);

void DMU_UpdateScaledVariance(unsigned long n,
                              DMU_Scaled_Struct *new_data,
                              DMU_Scaled_Struct *new_avg,
                              DMU_Scaled_Struct *old_avg,
                              DMU_Scaled_Struct *var_upd);

void DMU_UpdateAngleVariance(unsigned long n,
                             DMU_Angle_Struct *new_data,
                             DMU_Angle_Struct *new_avg,
                             DMU_Angle_Struct *old_avg,
                             DMU_Angle_Struct *var_upd);

void DMU_UpdateNavVariance(unsigned long n,
                           DMU_Nav_Struct *new_data,
                           DMU_Nav_Struct *new_avg,
                           DMU_Nav_Struct *old_avg,
                           DMU_Nav_Struct *var_upd);

long DMU_SkipHeader(FILE *input);
int DMU_ReadScaledFromFile(FILE *input, DMU_Scaled_Struct *strct, TIMEVAL *tv);

/** Dump header for the logfile. Return the number of column per row */
unsigned int DMU_DumpHeader(FILE *strm, DMU_MODE_TYPE mode, char *one_line_comment, char *hostname);

/** Dump a Scaled struct in strm */
DMU_RET_TYPE DMU_DumpScaled(FILE *strm, DMU_Scaled_Struct *strct, TIMEVAL *tv);

/** Dump a Angle struct in strm */
DMU_RET_TYPE DMU_DumpAngle(FILE *strm, DMU_Angle_Struct *strct, TIMEVAL *tv);

/** Dump a Nav struct in strm */
DMU_RET_TYPE DMU_DumpNav(FILE *strm, DMU_Nav_Struct *strct, TIMEVAL *tv);

/** Read the data type from a file row (first column) */
int DMU_ReadDataType(FILE *input);

/** Read a Scaled record from input */
int DMU_ReadScaledFromFile(FILE *input, DMU_Scaled_Struct *strct, TIMEVAL *tv);

/** Read an Angle record from input */
int DMU_ReadAngleFromFile(FILE *input, DMU_Angle_Struct *strct, TIMEVAL *tv);

/** Read a Nav record from input */
int DMU_ReadNavFromFile(FILE *input, DMU_Nav_Struct *strct, TIMEVAL *tv);

/** Remove the gravity field using roll and pitch. Only the accelerations are affected */
void DMU_RemoveAngleGravity(DMU_Angle_Struct *strct);

/** Remove the gravity field using roll and pitch. Only the accelerations are affected */
void DMU_RemoveNavGravity(DMU_Nav_Struct *strct);

void DMU_FillGPSWithGGA(GPS_Struct *gps, GPS_gga *gga);
void DMU_FillGPSWithRMC(GPS_Struct *gps, GPS_rmc *rmc);


#endif
