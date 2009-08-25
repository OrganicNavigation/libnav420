/***************************************************************************
                        nav420_lib.h  -  description
                            -------------------
  begin                : December 2005
  copyright            : (C) 2005 by Pierre Lamon
  email                : pierre.lamon@epfl.ch

  description          : export the functions to communicate with the
                         NAV420 unit. This is the header file to include
                         in the user code of the library
 ***************************************************************************/

#ifndef NAV420_LIB_H
#define NAV420_LIB_H

/*[cp] fixed for Suse 10
 #include <linux/types.h>*/
#include <sys/types.h>

#include <stdio.h>
#include "nav420_struct.h"
#include "nav420_tools.h"
#include <elrob/gps-nmea.h>

/* Exported functions */
/** Return the bits per second */
int DMU_BaudrateToBitPerSec(int baudrate);

/** Return the transmission time in micro-seconds of a packet of "mode" type */
unsigned int DMU_ComputeTransTime(DMU_MODE_TYPE mode, int baudrate);

/** Open the serial port */
int DMU_OpenSerial (DMU_DEV_TYPE port, char *device, int baudrate);

/** Close the serial port */
int DMU_CloseSerial(DMU_DEV_TYPE port);

/** Set the default output stream \arg ostream : a file descriptor (e.g. stdout, stderr, etc.)*/
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

/** Read a 'Scaled' packet from the NAV420. Use this function when the NAV420 is in streaming mode. */
DMU_RET_TYPE DMU_ReadScaledData(DMU_Scaled_Struct *strct);

/** Read a 'Angle' packet from the NAV420. Use this function when the NAV420 is in streaming mode. */
DMU_RET_TYPE DMU_ReadAngleData(DMU_Angle_Struct *strct);

/** Read a 'Nav' packet from the NAV420. Use this function when the NAV420 is in streaming mode. */
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
DMU_RET_TYPE DMU_LaunchNavContThread(DMU_MODE_TYPE mode, int root, void (*pCallback) (DMU_Shared_Mem *));

/** Launch a thread that reads data continuously from the GPS */
DMU_RET_TYPE DMU_LaunchGPSContThread(int root, void (*pCallback) (GPS_gga *, GPS_rmc *));

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

int DMU_CorrGPSTimestamp(TIMEVAL curr_stamp, TIMEVAL curr_itow, TIMEVAL *old_itow);
void DMU_comp_geomag(double decim_date, GPS_Struct gps, double *myDeclination);
void DMU_comp_geomagTV(TIMEVAL time, GPS_Struct gps, double *myDeclination);
void DMU_comp_geomag_now(GPS_Struct gps, double *myDeclinaison);
void DMU_PrintLocalOrigin(DMU_PoseZero_Struct *strct);

GPS_gga DMU_GGA_GetCopy();
GPS_rmc DMU_RMC_GetCopy();

int DMU_WaitPacket(DMU_Shared_Mem *shm);
int DMU_StoreITOWChangeTime(TIMEVAL curr_itow, TIMEVAL curr_stamp);



#endif
