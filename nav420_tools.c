/***************************************************************************
                        nav420_tools.c  -  description
                            -------------------
  begin                : December 2005
  copyright            : (C) 2005 by Pierre Lamon
  email                : pierre.lamon@epfl.ch

  description          : implement the conversion functions
 ***************************************************************************/

#include <math.h>
#include <stdio.h>
#include <string.h>

#include <elrob/Emacros.h>
#include "nav420.h"
#include "nav420_tools.h"
#include <elrob/gps-nmea.h>
#include <elrob/libgps.h>
#include <elrob/Etime.h>
#include <elrob/Etools.h>

#ifndef SQR
#define SQR(a) ((a)*(a))
#endif

/* Global variables */
static DMU_PoseZero_Struct m_origin = {{0,0,0},{0,0,0},0,0};
static char DUMP_FMT_SCALED[256] = "#";
static char DUMP_FMT_ANGLE[256]  = "#";
static char DUMP_FMT_NAV[256]    = "#";

const char DMU_AXIS_FMT[] = "%lf\t%lf\t%lf\t";
const char DMU_REAL_FMT[] = "%lf\t";
const char DMU_INT_FMT[]  = "%i\t";
const char DMU_CHAR_FMT[] = "%c\t";

/* --- Coordinate conversion */
void DMU_SetLocalOrigin(GPS_Struct *orig, EPOINT3D *rpy, DMU_REAL declination)
{
  m_origin.gps    = *orig;
  if(rpy != NULL)
    m_origin.rpy = *rpy;
  else
    DMU_FILL_VAL(m_origin.rpy,0.0);

  m_origin.declination = declination;
  m_origin.radius = GPS_ComputeEarthRadius(sin(m_origin.gps.latitude)) + m_origin.gps.altitude;
}

DMU_PoseZero_Struct DMU_GetLocalOrigin()
{
  return m_origin;
}

EPOINT3D DMU_GPSToLocalXYZ(GPS_Struct *gps)
{
  return GPS_GPSToLocalXYZ(gps, &m_origin.gps, m_origin.radius);
}

void DMU_GPSToEarthXYZ(EPOINT3D *xyz, DMU_Nav_Struct *strct)
{
  *xyz = GPS_ConvertToEarthXYZ(&strct->coord);
}

void DMU_RemoveAngleGravity(DMU_Angle_Struct *strct)
{
  double cp = cos(strct->angle.y);

  strct->accel.x -= DMU_GRAVITY * sin (strct->angle.y);
  strct->accel.y += DMU_GRAVITY * sin (strct->angle.x) * cp;
  strct->accel.z += DMU_GRAVITY * cos (strct->angle.x) * cp;
}

void DMU_RemoveNavGravity(DMU_Nav_Struct *strct)
{

}

void DMU_UpdateAxisMean(unsigned long n, EPOINT3D *new_data, EPOINT3D *old_data)
{
  old_data->x = UPDATE_AVERAGE(n, new_data->x, old_data->x);
  old_data->y = UPDATE_AVERAGE(n, new_data->y, old_data->y);
  old_data->z = UPDATE_AVERAGE(n, new_data->z, old_data->z);
}

void DMU_UpdateScaledMean(unsigned long n, DMU_Scaled_Struct *new_data, DMU_Scaled_Struct *old_data)
{
  DMU_UpdateAxisMean(n, &new_data->accel, &old_data->accel);
  DMU_UpdateAxisMean(n, &new_data->rate, &old_data->rate);
  DMU_UpdateAxisMean(n, &new_data->mag, &old_data->mag);
  DMU_UpdateAxisMean(n, &new_data->temp, &old_data->temp);
  old_data->cpu_temp = UPDATE_AVERAGE(n, new_data->cpu_temp, old_data->cpu_temp);
}

void DMU_UpdateAngleMean(unsigned long n, DMU_Angle_Struct *new_data, DMU_Angle_Struct *old_data)
{
  DMU_UpdateAxisMean(n, &new_data->angle, &old_data->angle);
  DMU_UpdateAxisMean(n, &new_data->rate, &old_data->rate);
  DMU_UpdateAxisMean(n, &new_data->accel, &old_data->accel);
  DMU_UpdateAxisMean(n, &new_data->mag, &old_data->mag);
  old_data->temp = UPDATE_AVERAGE(n, new_data->temp, old_data->temp);
}

void DMU_UpdateNavMean(unsigned long n, DMU_Nav_Struct *new_data, DMU_Nav_Struct *old_data)
{
  DMU_UpdateAxisMean(n, &new_data->angle, &old_data->angle);
  DMU_UpdateAxisMean(n, &new_data->rate, &old_data->rate);
  DMU_UpdateAxisMean(n, &new_data->velocity, &old_data->velocity);
  GPS_UpdateGPSMean(n, &new_data->coord, &old_data->coord);
}

void DMU_UpdateAxisVariance(unsigned long n,
                            EPOINT3D *new_axis,
                            EPOINT3D *axis_avg,
                            EPOINT3D *old_avg,
                            EPOINT3D *axis_var_upd)
{
  axis_var_upd->x = UPDATE_VARIANCE(n, new_axis->x, axis_avg->x, old_avg->x, axis_var_upd->x);
  axis_var_upd->y = UPDATE_VARIANCE(n, new_axis->y, axis_avg->y, old_avg->y, axis_var_upd->y);
  axis_var_upd->z = UPDATE_VARIANCE(n, new_axis->z, axis_avg->z, old_avg->z, axis_var_upd->z);
}

void DMU_UpdateScaledVariance(unsigned long n,
                              DMU_Scaled_Struct *new_data,
                              DMU_Scaled_Struct *new_avg,
                              DMU_Scaled_Struct *old_avg,
                              DMU_Scaled_Struct *var_upd)
{
  DMU_UpdateAxisVariance(n, &new_data->accel, &new_avg->accel, &old_avg->accel, &var_upd->accel);
  DMU_UpdateAxisVariance(n, &new_data->rate,  &new_avg->rate,  &old_avg->rate,  &var_upd->rate);
  DMU_UpdateAxisVariance(n, &new_data->mag,   &new_avg->mag,   &old_avg->mag,   &var_upd->mag);
  DMU_UpdateAxisVariance(n, &new_data->temp,  &new_avg->temp,  &old_avg->temp,  &var_upd->temp);
  var_upd->cpu_temp = UPDATE_VARIANCE(n, new_data->cpu_temp, new_avg->cpu_temp, old_avg->cpu_temp, var_upd->cpu_temp);
}

void DMU_UpdateAngleVariance(unsigned long n,
                             DMU_Angle_Struct *new_data,
                             DMU_Angle_Struct *new_avg,
                             DMU_Angle_Struct *old_avg,
                             DMU_Angle_Struct *var_upd)
{
  DMU_UpdateAxisVariance(n, &new_data->angle, &new_avg->angle, &old_avg->angle, &var_upd->angle);
  DMU_UpdateAxisVariance(n, &new_data->rate,  &new_avg->rate,  &old_avg->rate,  &var_upd->rate);
  DMU_UpdateAxisVariance(n, &new_data->accel, &new_avg->accel, &old_avg->accel, &var_upd->accel);
  DMU_UpdateAxisVariance(n, &new_data->mag,   &new_avg->mag,   &old_avg->mag,   &var_upd->mag);
  var_upd->temp = UPDATE_VARIANCE(n, new_data->temp, new_avg->temp, old_avg->temp, var_upd->temp);
}

void DMU_UpdateNavVariance(unsigned long n,
                           DMU_Nav_Struct *new_data,
                           DMU_Nav_Struct *new_avg,
                           DMU_Nav_Struct *old_avg,
                           DMU_Nav_Struct *var_upd)
{
  DMU_UpdateAxisVariance(n, &new_data->angle,    &new_avg->angle,    &old_avg->angle,    &var_upd->angle);
  DMU_UpdateAxisVariance(n, &new_data->rate,     &new_avg->rate,     &old_avg->rate,     &var_upd->rate);
  DMU_UpdateAxisVariance(n, &new_data->velocity, &new_avg->velocity, &old_avg->velocity, &var_upd->velocity);
  GPS_UpdateGPSVariance (n, &new_data->coord,    &new_avg->coord,    &old_avg->coord,    &var_upd->coord);
}

/* --- Dump functions (for logging) */
unsigned int DMU_DumpHeader(FILE *strm, DMU_MODE_TYPE mode, char *one_line_comment, char *hostname)
{
  unsigned int column;

  if(hostname != NULL)
    DUMP_HOSTNAME(strm,hostname);

  switch(mode)
  {
    case DMU_SCALED_MODE: {
      fprintf(strm,"# Header for SCALED mode\n");
      fprintf(strm,"# The data is expressed in the unit frame\n");
      if(one_line_comment != NULL) fprintf(strm,"# %s", one_line_comment);
      fprintf(strm,"# 1: data row number\n# 2: stamp [s]\n# 3: mode(0)\n# 4: gps itow [s]\n# 5: BIT (flags)\n");
      fprintf(strm,"# 6,7,8: acceleration [m/s^2]\n# 9,10,11: rates [rad/s] |\n");
      fprintf(strm,"# 12,13,14: magnetic field [Gauss]\n# 15,16,17: axis temperatures [K]\n# 18: cpu temperature [K]\n");
      column = 19;
    }
    break;

    case DMU_ANGLE_MODE: {
      fprintf(strm,"# Header for ANGLE mode\n");
      fprintf(strm,"# The data is expressed in the unit frame\n");
      if(one_line_comment != NULL) fprintf(strm,"# %s", one_line_comment);
      fprintf(strm,"# 1: data row number\n# 2: stamp [s]\n# 3: mode(1)\n# 4: gps itow [s]\n# 5: BIT (flags)\n");
      fprintf(strm,"# 6,7,8: angle [rad]\n# 9,10,11: rates [rad/s]\n");
      fprintf(strm,"# 12,13,14: acceleration [m/s^2]\n# 15,16,17: magnetic field [Gauss]\n# 18: sensor temperature [K]\n");
      column = 19;
    }
    break;

    case DMU_NAV_MODE: {
      fprintf(strm,"# Header for NAV mode\n");
      fprintf(strm,"# The data is expressed in the unit frame\n");
      if(one_line_comment != NULL) fprintf(strm,"# %s", one_line_comment);
      fprintf(strm,"# 1: data row number\n# 2: stamp [s]\n# 3: mode(2)\n# 4: gps itow [s]\n# 5: BIT (flags)\n");
      fprintf(strm,"# 6,7,8: angle [rad]\n# 9,10,11: rates [rad/s]\n# 12,13,14: velocity [m/s]\n");
      fprintf(strm,"# 15: latitude [rad]\n# 16: longitude [rad]\n# 17: altitude [m]\n");
      column = 18;
    }
    break;

    default: return 0;
  }

  return column;
}

void DMU_DumpRMC(FILE *strm)
{
  GPS_DumpRMC(strm, GPS_GetRmcPtr());
}

void DMU_DumpGGA(FILE *strm)
{
  GPS_DumpGGA(strm, GPS_GetGgaPtr());
}

DMU_RET_TYPE DMU_DumpScaled(FILE *strm, DMU_Scaled_Struct *strct, TIMEVAL *tv)
{
  struct timeval tv_tmp;

  fprintf(strm,"%i%c",
    MODULE_NAME_NAV420_SCALED,
    DMU_SEP);

  if(tv != NULL)
    DMU_DUMP_TIMEVAL(strm,*tv);
  else {
    gettimeofday(&tv_tmp,0);
    DMU_DUMP_TIMEVAL(strm,tv_tmp);
  }

  DMU_DUMP_INT(strm,DMU_SCALED_MODE);
  DMU_DUMP_TIMEVAL(strm,strct->gps_itow);
  DMU_DUMP_INT(strm,strct->bit);
  DMU_DUMP_AXIS(strm,strct->accel);
  DMU_DUMP_AXIS(strm,strct->rate);
  DMU_DUMP_AXIS(strm,strct->mag);
  DMU_DUMP_AXIS(strm,strct->temp);
  DMU_DUMP_REAL(strm,strct->cpu_temp);

  return DMU_RET_OK;
}

DMU_RET_TYPE DMU_DumpAngle(FILE *strm, DMU_Angle_Struct *strct, TIMEVAL *tv)
{
  struct timeval tv_tmp;

  fprintf(strm,"%i%c",
          MODULE_NAME_NAV420_ANGLE,
          DMU_SEP);

  if(tv != NULL)
    DMU_DUMP_TIMEVAL(strm,*tv);
  else {
    gettimeofday(&tv_tmp, 0);
    DMU_DUMP_TIMEVAL(strm,tv_tmp);
  }

  DMU_DUMP_INT(strm,DMU_ANGLE_MODE);
  DMU_DUMP_TIMEVAL(strm,strct->gps_itow);
  DMU_DUMP_INT(strm,strct->bit);
  DMU_DUMP_AXIS(strm,strct->angle);
  DMU_DUMP_AXIS(strm,strct->rate);
  DMU_DUMP_AXIS(strm,strct->accel);
  DMU_DUMP_AXIS(strm,strct->mag);
  DMU_DUMP_REAL(strm,strct->temp);

  return DMU_RET_OK;
}

DMU_RET_TYPE DMU_DumpNav(FILE *strm, DMU_Nav_Struct *strct, TIMEVAL *tv)
{
  struct timeval tv_tmp;

  fprintf(strm,"%i%c",
          MODULE_NAME_NAV420_NAV,
          DMU_SEP);

  if(tv != NULL)
    DMU_DUMP_TIMEVAL(strm,*tv);
  else {
    gettimeofday(&tv_tmp, 0);
    DMU_DUMP_TIMEVAL(strm,tv_tmp);
  }

  DMU_DUMP_INT(strm,DMU_NAV_MODE);
  DMU_DUMP_TIMEVAL(strm,strct->gps_itow);
  DMU_DUMP_INT(strm,strct->bit);
  DMU_DUMP_AXIS(strm,strct->angle);
  DMU_DUMP_AXIS(strm,strct->rate);
  DMU_DUMP_AXIS(strm,strct->velocity);
  DMU_DUMP_GPS(strm,strct->coord);

  return DMU_RET_OK;
}

int DMU_ReadAxis(FILE *input, EPOINT3D *axis)
{
  char c;
  return fscanf(input, "%lf%c%lf%c%lf%c", &axis->x, &c, &axis->y, &c, &axis->z, &c);
}

int DMU_ReadReal(FILE *input, DMU_REAL *real)
{
  char c;
  return fscanf(input, "%lf%c", real, &c);
}

long DMU_SkipHeader(FILE *input)
{
  char  buf[512];
  long  offset = 0;
  char  *ptr = NULL;

  rewind(input);

  do
  {
    ptr = fgets(buf, 512, input);

    if(buf[0] != '#') {
      fseek(input,offset,SEEK_SET);
      return offset;
    }
    offset = ftell(input);
  }
  while(!feof(input));

  return 0;
}

/* Read the same format as the Dump functions. Return 0 if failure */
int DMU_ReadScaledFromFile(FILE *input, DMU_Scaled_Struct *strct, TIMEVAL *tv)
{
  double dbl_tv, dbl_itow;
  char   buf[512];
  int    items = 0;
  int    mode;
  int    bit;
  int    msg_type;

  if(feof(input) != 0)
    return 0;

  if(DUMP_FMT_SCALED[0] == '#')
  {
    strcpy(DUMP_FMT_SCALED,DMU_INT_FMT);
    strcat(DUMP_FMT_SCALED,DMU_REAL_FMT);
    strcat(DUMP_FMT_SCALED,DMU_INT_FMT);
    strcat(DUMP_FMT_SCALED,DMU_REAL_FMT);
    strcat(DUMP_FMT_SCALED,DMU_INT_FMT);
    strcat(DUMP_FMT_SCALED,DMU_AXIS_FMT);
    strcat(DUMP_FMT_SCALED,DMU_AXIS_FMT);
    strcat(DUMP_FMT_SCALED,DMU_AXIS_FMT);
    strcat(DUMP_FMT_SCALED,DMU_AXIS_FMT);
    strcat(DUMP_FMT_SCALED,DMU_REAL_FMT);
    strcat(DUMP_FMT_SCALED,"\n");
  }

  fgets(buf,512,input);
  if(feof(input) != 0)
    return 0;

  items = sscanf(buf,DUMP_FMT_SCALED,
            &msg_type,
            &dbl_tv,
            &mode,
            &dbl_itow,
            &bit,
            &strct->accel.x,
            &strct->accel.y,
            &strct->accel.z,
            &strct->rate.x,
            &strct->rate.y,
            &strct->rate.z,
            &strct->mag.x,
            &strct->mag.y,
            &strct->mag.z,
            &strct->temp.x,
            &strct->temp.y,
            &strct->temp.z,
            &strct->cpu_temp);

  if(items != 18)
    return 0;

  strct->bit = (u_int16_t) bit;
  strct->gps_itow = Time_FromDbl(dbl_itow);
  *tv = Time_FromDbl(dbl_tv);

  return DMU_RET_OK;
}

int DMU_ReadAngleFromFile(FILE *input, DMU_Angle_Struct *strct, TIMEVAL *tv)
{
  double dbl_tv, dbl_itow;
  char   buf[512];
  int    items = 0;
  int    mode;
  int    bit;
  int    msg_type;

  if(feof(input) != 0) {
    fprintf(stderr,"DMU_ReadAngleFromFile -> end of file (1)!\n");
    return 0;
  }

  if(DUMP_FMT_ANGLE[0] == '#')
  {
    strcpy(DUMP_FMT_ANGLE,DMU_INT_FMT);
    strcat(DUMP_FMT_ANGLE,DMU_REAL_FMT);
    strcat(DUMP_FMT_ANGLE,DMU_INT_FMT);
    strcat(DUMP_FMT_ANGLE,DMU_REAL_FMT);
    strcat(DUMP_FMT_ANGLE,DMU_INT_FMT);
    strcat(DUMP_FMT_ANGLE,DMU_AXIS_FMT);
    strcat(DUMP_FMT_ANGLE,DMU_AXIS_FMT);
    strcat(DUMP_FMT_ANGLE,DMU_AXIS_FMT);
    strcat(DUMP_FMT_ANGLE,DMU_AXIS_FMT);
    strcat(DUMP_FMT_ANGLE,DMU_REAL_FMT);
    strcat(DUMP_FMT_ANGLE,"\n");
  }

  fgets(buf,512,input);
  if(feof(input) != 0){
    fprintf(stderr,"DMU_ReadAngleFromFile -> end of file (2)!\n");
    return 0;
  }

  items = sscanf(buf,DUMP_FMT_ANGLE,
                 &msg_type,
                 &dbl_tv,
                 &mode,
                 &dbl_itow,
                 &bit,
                 &strct->angle.x,
                 &strct->angle.y,
                 &strct->angle.z,
                 &strct->rate.x,
                 &strct->rate.y,
                 &strct->rate.z,
                 &strct->accel.x,
                 &strct->accel.y,
                 &strct->accel.z,
                 &strct->mag.x,
                 &strct->mag.y,
                 &strct->mag.z,
                 &strct->temp);

  if(items != 18){
    fprintf(stderr,"DMU_ReadAngleFromFile -> Could not read 18 items\n");
    return 0;
  }

  strct->bit = (u_int16_t) bit;
  strct->gps_itow = Time_FromDbl(dbl_itow);
  /*printf("Dbl tv: %.10f, angle: %.10f\n",dbl_tv,strct->angle.z);*/

  if(strct->angle.z > M_PI) {
    strct->angle.z = M_PI;
    DMU_DBG("DMU_ReadAngleFromFile -> strct->angle.z > M_PI: correcting");
  }
  if(strct->angle.z < -M_PI) {
    strct->angle.z = -M_PI;
    DMU_DBG("DMU_ReadAngleFromFile -> strct->angle.z < -M_PI: correcting");
  }

  *tv = Time_FromDbl(dbl_tv);
  return DMU_RET_OK;
}

int DMU_ReadNavFromFile(FILE *input, DMU_Nav_Struct *strct, TIMEVAL *tv)
{
  double dbl_tv, dbl_itow;
  char   buf[512];
  int    items = 0;
  int    mode;
  int    bit;
  int    msg_type;

  if(feof(input)) return 0;

  if(DUMP_FMT_NAV[0] == '#')
  {
    strcpy(DUMP_FMT_NAV,DMU_INT_FMT);
    strcat(DUMP_FMT_NAV,DMU_REAL_FMT);
    strcat(DUMP_FMT_NAV,DMU_INT_FMT);
    strcat(DUMP_FMT_NAV,DMU_REAL_FMT);
    strcat(DUMP_FMT_NAV,DMU_INT_FMT);
    strcat(DUMP_FMT_NAV,DMU_AXIS_FMT);
    strcat(DUMP_FMT_NAV,DMU_AXIS_FMT);
    strcat(DUMP_FMT_NAV,DMU_AXIS_FMT);
    strcat(DUMP_FMT_NAV,DMU_AXIS_FMT);
    strcat(DUMP_FMT_NAV,"\n");
  }

  fgets(buf,512,input);
  if(feof(input) != 0)
    return 0;

  items = sscanf(buf,DUMP_FMT_NAV,
                 &msg_type,
                 &dbl_tv,
                 &mode,
                 &dbl_itow,
                 &bit,
                 &strct->angle.x,
                 &strct->angle.y,
                 &strct->angle.z,
                 &strct->rate.x,
                 &strct->rate.y,
                 &strct->rate.z,
                 &strct->velocity.x,
                 &strct->velocity.y,
                 &strct->velocity.z,
                 &strct->coord.latitude,
                 &strct->coord.longitude,
                 &strct->coord.altitude);

  if(items != 17)
    return 0;

  if(strct->angle.z > M_PI) {
    strct->angle.z = M_PI;
    DMU_DBG("DMU_ReadNavFromFile -> strct->angle.z > M_PI: correcting");
  }
  if(strct->angle.z < -M_PI) {
    strct->angle.z = -M_PI;
    DMU_DBG("DMU_ReadNavFromFile -> strct->angle.z < -M_PI: correcting");
  }

  strct->bit = (u_int16_t) bit;
  strct->gps_itow = Time_FromDbl(dbl_itow);
  *tv = Time_FromDbl(dbl_tv);

  return DMU_RET_OK;
}

/* return -1 if error else the type of data (see Etypes.h) */
int DMU_ReadDataType(FILE *input)
{
  int     type, items;
  long    pos;
  char    type_str[256] = "";
  int     p = 0;

  if(feof(input))
      return -1;

  /* Store current position */
  pos = ftell(input);

  /* Read the file until we get the first DMU_SEP*/
  while(!feof(input) && fread(&type_str[p],1,1,input) == 1 && type_str[p] != DMU_SEP) {
    p += 1;
  }

  /* check if anything went wrong */
  if(p == 0){
    fseek(input, pos, SEEK_SET);
    return -1;
  }

  /* Convert char to int */
  type_str[p+1] = '\n';
  items = sscanf(type_str, DMU_INT_FMT, &type);

  if(items != 1)
  {
    fseek(input, pos, SEEK_SET);
    return -1;
  }

  /* Put the file pointer back in place */
  fseek(input, pos, SEEK_SET);
  return type;
}

void DMU_FillGPSWithGGA(GPS_Struct *gps, GPS_gga *gga)
{
  gps->latitude  = gga->latitude;
  gps->longitude = gga->longitude;
  gps->altitude  = gga->altitude;
}

/** Warning the altitude is not available in rmc! Set to zero by the function */
void DMU_FillGPSWithRMC(GPS_Struct *gps, GPS_rmc *rmc)
{
  gps->latitude  = rmc->latitude;
  gps->longitude = rmc->longitude;
  gps->altitude  = 0.0;
}




