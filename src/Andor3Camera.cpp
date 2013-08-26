/* andor3 plugin camera class
 * Copyright (C) 2013 IPANEMA USR3461, CNRS/MCC.
 * Written by Serge Cohen <serge.cohen@synchrotron-soleil.fr>
 *
 * This file is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 3 of
 * the License, or (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this file. If not, see <http://www.gnu.org/licenses/>.
 */

// System headers :
#include <sstream>
#include <iostream>
#include <string>
#include <math.h>

// Camera SDK headers :

// LImA headers :

// Andor3 plugin headers :
#include "Andor3Camera.h"


lima::Andor3::Camera::Camera(const std::string& bitflow_path,int camera_number)
{
  
}

lima::Andor3::Camera::~Camera()
{
  
}

void
lima::Andor3::Camera::prepareAcq()
{
}

void
lima::Andor3::Camera::startAcq()
{
}

void
lima::Andor3::Camera::stopAcq()
{
}

// -- detector info object
void
lima::Andor3::Camera::getImageType(ImageType& type)
{
}

void
lima::Andor3::Camera::setImageType(ImageType type)
{
}

void
lima::Andor3::Camera::getDetectorType(std::string& type)
{
}

void
lima::Andor3::Camera::getDetectorModel(std::string& model)
{
}

void
lima::Andor3::Camera::getDetectorImageSize(Size& size)
{
}

// -- Buffer control object
lima::HwBufferCtrlObj*
lima::Andor3::Camera::getBufferCtrlObj()
{
}

//-- Synch control object
void
lima::Andor3::Camera::setTrigMode(TrigMode  mode)
{
}

void
lima::Andor3::Camera::getTrigMode(TrigMode& mode)
{
}

void
lima::Andor3::Camera::setExpTime(double  exp_time)
{
}

void
lima::Andor3::Camera::getExpTime(double& exp_time)
{
}

void
lima::Andor3::Camera::setLatTime(double  lat_time)
{
}
void
lima::Andor3::Camera::getLatTime(double& lat_time)
{
}

void
lima::Andor3::Camera::getExposureTimeRange(double& min_expo, double& max_expo) const
{
}
void
lima::Andor3::Camera::getLatTimeRange(double& min_lat, double& max_lat) const
{
}

void
lima::Andor3::Camera::setNbFrames(int  nb_frames)
{
}
void
lima::Andor3::Camera::getNbFrames(int& nb_frames)
{
}
void
lima::Andor3::Camera::getNbHwAcquiredFrames(int &nb_acq_frames)
{
}

void
lima::Andor3::Camera::checkRoi(const Roi& set_roi, Roi& hw_roi)
{
}
void
lima::Andor3::Camera::setRoi(const Roi& set_roi)
{
}
void
lima::Andor3::Camera::getRoi(Roi& hw_roi)
{
}

bool
lima::Andor3::Camera::isBinningAvailable()
{
}
void
lima::Andor3::Camera::checkBin(Bin&)
{
}
void
lima::Andor3::Camera::setBin(const Bin&)
{
}
void
lima::Andor3::Camera::getBin(Bin&)
{
}

void
lima::Andor3::Camera::setShutterMode(ShutterMode mode)
{
}
void
lima::Andor3::Camera::getShutterMode(ShutterMode& mode)
{
}

void
lima::Andor3::Camera::setShutter(bool flag)
{
}
void
lima::Andor3::Camera::getShutter(bool& flag)
{
}

void
lima::Andor3::Camera::getPixelSize(double& sizex, double& sizey)
{
}

void
lima::Andor3::Camera::getStatus(Camera::Status& status)
{
}

// --- Acquisition interface
void
lima::Andor3::Camera::reset()
{
}
int
lima::Andor3::Camera::getNbHwAcquiredFrames()
{
}

// -- andor3 specific, LIMA don't worry about it !
void
lima::Andor3::Camera::initialiseController()
{
}

void
lima::Andor3::Camera::setAdcGain(A3_Gain iGain)
{
}
void
lima::Andor3::Camera::getAdcGain(A3_Gain &oGain)
{
}
void
lima::Andor3::Camera::setAdcRate(A3_ReadOutRate iRate)
{
}
void
lima::Andor3::Camera::getAdcRate(A3_ReadOutRate &oRate)
{
}
void
lima::Andor3::Camera::setElectronicShutterMode(A3_ShutterMode iMode)
{
}
void
lima::Andor3::Camera::getElectronicShutterMode(A3_ShutterMode &oMode)
{
}



//-----------------------------------------------------
// @brief	set the temperature set-point // DONE
// @param	temperature in centigrade
//
//-----------------------------------------------------
void
lima::Andor3::Camera::setTemperatureSP(double temp)
{
  DEB_MEMBER_FUNCT();
  if ( AT_SUCCESS != setFloat(L"TargetSensorTemperature", temp) ) {
    DEB_ERROR() << "Failed to set temperature set-point" <<" : error code = " << m_camera_error_str;
    THROW_HW_ERROR(Error) << "Failed to set temperature set-point";
  }
  // As advised by Andor SDK 3 doc : proof-reading after setting :
  if ( AT_SUCCESS != getFloat(L"TargetSensorTemperature", &m_temperature_sp) ) {
    DEB_ERROR() << "Failed to proof-read temperature set-point" <<" : error code = " << m_camera_error_str;
    THROW_HW_ERROR(Error) << "Failed to proof-read temperature set-point";
  }
}

//-----------------------------------------------------
// @brief	return the temperature set-point // DONE (trusting the cached value)
// @param	temperature in centigrade
//
//-----------------------------------------------------
void
lima::Andor3::Camera::getTemperatureSP(double& temp)
{
  DEB_MEMBER_FUNCT();
  temp = m_temperature_sp;
}


//-----------------------------------------------------
// @brief	Gets the real temperature of the detector sensor // DONE
// @param	temp temperature in centigrade
//
//-----------------------------------------------------
void
lima::Andor3::Camera::getTemperature(double& temp)
{
  DEB_MEMBER_FUNCT();
  if ( AT_SUCCESS != getFloat(L"SensorTemperature", &temp) ) {
    DEB_ERROR() << "Failed to read temperature" <<" : error code = " << m_camera_error_str;
    THROW_HW_ERROR(Error) << "Failed to read temperature";
  }
}

//-----------------------------------------------------
// @brief	start or Stop the cooler    // DONE
// @param	flag true-on, false-off
//
//-----------------------------------------------------
void
lima::Andor3::Camera::setCooler(bool flag)
{
  DEB_MEMBER_FUNCT();
  if ( AT_SUCCESS != setBool(L"SensorCooling", flag) ) {
    DEB_ERROR() << "Failed to set cooler" <<" : error code = " << m_camera_error_str;
    THROW_HW_ERROR(Error) << "Failed to set cooler";
  }
  // As advised by Andor SDK 3 doc : proof-reading after setting :
  if ( AT_SUCCESS != getBool(L"SensorCooling", &m_cooler) ) {
    DEB_ERROR() << "Failed to proof-read cooler" <<" : error code = " << m_camera_error_str;
    THROW_HW_ERROR(Error) << "Failed to proof-read cooler";
  }
  // And finally check that it corresponds to the requested action :
  if ( flag != m_cooler ) {
    DEB_ERROR() << "Failed to properly set the cooler : requeste " << flag << ", but got " << m_cooler;
    THROW_HW_ERROR(Error) << "Failed to properly set the cooler";
  }
}

//-----------------------------------------------------
// @brief	Get the Cooler status     // DONE (trusting the cached value)
// @param	flag true-on, false-off
//
//-----------------------------------------------------
void
lima::Andor3::Camera::getCooler(bool& flag)
{
  DEB_MEMBER_FUNCT();
  flag = m_cooler;
}

//-----------------------------------------------------
// @brief	Gets cooling status
// @param	status status as a string
//
//-----------------------------------------------------
void
lima::Andor3::Camera::getCoolingStatus(std::string& status)
{
  DEB_MEMBER_FUNCT();
  
  wchar_t		wcs_status_string[1024];
  if ( AT_SUCCESS != getEnumString(L"TemperatureStatus", wcs_status_string, 1023) ) {
    DEB_ERROR() << "Failed to read cooling status" <<" : error code = " << m_camera_error_str;
    THROW_HW_ERROR(Error) << "Failed to cooling status";
  }
  status = WStringToString(wcs_status_string);
}

//-----------------------------------------------------
// @brief handle the andor3 error codes
//-----------------------------------------------------
bool
lima::Andor3::Camera::andor3Error(int code) const
{
  m_camera_error = code;
  //  m_camera_error_str = m_andor3_error_maps[code]; // Not const !
  m_camera_error_str = m_andor3_error_maps.at(code);
  
  return ( AT_SUCCESS != code );
}

//-----------------------------------------------------
// @brief just build a map of error codes
//-----------------------------------------------------
void
lima::Andor3::Camera::_mapAndor3Error()
{
  m_andor3_error_maps[AT_SUCCESS] = "'AT_SUCCESS' : Function call has been successful";
  m_andor3_error_maps[AT_ERR_NOTINITIALISED] = "'AT_ERR_NOTINITIALISED' : Function called with an uninitialised handle";
  m_andor3_error_maps[AT_ERR_NOTIMPLEMENTED] = "'AT_ERR_NOTIMPLEMENTED' : Feature has not been implemented for the chosen camera";
  m_andor3_error_maps[AT_ERR_READONLY] = "'AT_ERR_READONLY' : Feature is read only";
  m_andor3_error_maps[AT_ERR_NOTREADABLE] = "'AT_ERR_NOTREADABLE' : Feature is currently not readable";
  m_andor3_error_maps[AT_ERR_NOTWRITABLE] = "'AT_ERR_NOTWRITABLE' : Feature is currently not writable";
  m_andor3_error_maps[AT_ERR_OUTOFRANGE] = "'AT_ERR_OUTOFRANGE' : Value is outside the maximum and minimum limits";
  m_andor3_error_maps[AT_ERR_INDEXNOTAVAILABLE] = "'AT_ERR_INDEXNOTAVAILABLE' : Index is currently not available";
  m_andor3_error_maps[AT_ERR_INDEXNOTIMPLEMENTED] = "'AT_ERR_INDEXNOTIMPLEMENTED' : Index is not implemented for the chosen camera";
  m_andor3_error_maps[AT_ERR_EXCEEDEDMAXSTRINGLENGTH] = "'AT_ERR_EXCEEDEDMAXSTRINGLENGTH' : String value provided exceeds the maximum allowed length";
  m_andor3_error_maps[AT_ERR_CONNECTION] = "'AT_ERR_CONNECTION' : Error connecting to or disconnecting from hardware";
  m_andor3_error_maps[AT_ERR_NODATA] = "AT_ERR_NODATA";
  m_andor3_error_maps[AT_ERR_INVALIDHANDLE] = "AT_ERR_INVALIDHANDLE";
  m_andor3_error_maps[AT_ERR_TIMEDOUT] = "'AT_ERR_TIMEDOUT' : The AT_WaitBuffer function timed out while waiting for data arrive in output queue";
  m_andor3_error_maps[AT_ERR_BUFFERFULL] = "'AT_ERR_BUFFERFULL' : The input queue has reached its capacity";
  m_andor3_error_maps[AT_ERR_INVALIDSIZE] = "'AT_ERR_INVALIDSIZE' : The size of a queued buffer did not match the frame size";
  m_andor3_error_maps[AT_ERR_INVALIDALIGNMENT] = "'AT_ERR_INVALIDALIGNMENT' : A queued buffer was not aligned on an 8-byte boundary";
  m_andor3_error_maps[AT_ERR_COMM] = "'AT_ERR_COMM' : An error has occurred while communicating with hardware";
  m_andor3_error_maps[AT_ERR_STRINGNOTAVAILABLE] = "'AT_ERR_STRINGNOTAVAILABLE' : Index / String is not available";
  m_andor3_error_maps[AT_ERR_STRINGNOTIMPLEMENTED] = "'AT_ERR_STRINGNOTIMPLEMENTED' : Index / String is not implemented for the chosen camera";
  m_andor3_error_maps[AT_ERR_NULL_FEATURE] = "AT_ERR_NULL_FEATURE";
  m_andor3_error_maps[AT_ERR_NULL_HANDLE] = "AT_ERR_NULL_HANDLE";
  m_andor3_error_maps[AT_ERR_NULL_IMPLEMENTED_VAR] = "AT_ERR_NULL_IMPLEMENTED_VAR";
  m_andor3_error_maps[AT_ERR_NULL_READABLE_VAR] = "AT_ERR_NULL_READABLE_VAR";
  m_andor3_error_maps[AT_ERR_NULL_READONLY_VAR] = "AT_ERR_NULL_READONLY_VAR";
  m_andor3_error_maps[AT_ERR_NULL_WRITABLE_VAR] = "AT_ERR_NULL_WRITABLE_VAR";
  m_andor3_error_maps[AT_ERR_NULL_MINVALUE] = "AT_ERR_NULL_MINVALUE";
  m_andor3_error_maps[AT_ERR_NULL_MAXVALUE] = "AT_ERR_NULL_MAXVALUE";
  m_andor3_error_maps[AT_ERR_NULL_VALUE] = "AT_ERR_NULL_VALUE";
  m_andor3_error_maps[AT_ERR_NULL_STRING] = "AT_ERR_NULL_STRING";
  m_andor3_error_maps[AT_ERR_NULL_COUNT_VAR] = "AT_ERR_NULL_COUNT_VAR";
  m_andor3_error_maps[AT_ERR_NULL_ISAVAILABLE_VAR] = "AT_ERR_NULL_ISAVAILABLE_VAR";
  m_andor3_error_maps[AT_ERR_NULL_MAXSTRINGLENGTH] = "AT_ERR_NULL_MAXSTRINGLENGTH";
  m_andor3_error_maps[AT_ERR_NULL_EVCALLBACK] = "AT_ERR_NULL_EVCALLBACK";
  m_andor3_error_maps[AT_ERR_NULL_QUEUE_PTR] = "AT_ERR_NULL_QUEUE_PTR";
  m_andor3_error_maps[AT_ERR_NULL_WAIT_PTR] = "AT_ERR_NULL_WAIT_PTR";
  m_andor3_error_maps[AT_ERR_NULL_PTRSIZE] = "AT_ERR_NULL_PTRSIZE";
  m_andor3_error_maps[AT_ERR_NOMEMORY] = "AT_ERR_NOMEMORY";
  m_andor3_error_maps[AT_ERR_DEVICEINUSE] = "AT_ERR_DEVICEINUSE";
  m_andor3_error_maps[AT_ERR_HARDWARE_OVERFLOW] = "AT_ERR_HARDWARE_OVERFLOW";
}

int
lima::Andor3::Camera::printInfoForProp(const AT_WC * iPropName, A3_TypeInfo iPropType)
{
  DEB_MEMBER_FUNCT();
  
  int i_err = 0;
  
  AT_BOOL b_exists;
  AT_BOOL b_readonly;
  AT_BOOL b_writable;
  AT_BOOL b_readable;
  
  DEB_TRACE() << "Retrieving information on property named \"" << WStringToString(iPropName) << "\".\n";
  
  // Implemented
  if ( AT_SUCCESS != andor3Error(AT_IsImplemented(m_camera_handle, iPropName, &b_exists)) ) {
    DEB_TRACE() << "Error in printInfoForProp : " << m_camera_error_str;
    return i_err;
  }
  DEB_TRACE() << "\tIsImplemented = " << atBoolToString(b_exists);
  
  if ( ! b_exists ) {
    DEB_TRACE() << "No more information to query, since the feature does not \"exists\" for this camera/driver/SDK.";
    return i_err;
  }
  
  // ReadOnly
  andor3Error(AT_IsReadOnly(m_camera_handle, iPropName, &b_readonly));
  DEB_TRACE() << "\tIsReadOnly = " << atBoolToString(b_readonly);
  
  // Writable
  andor3Error(AT_IsWritable(m_camera_handle, iPropName, &b_writable));
  DEB_TRACE() << "\tIsWritable = " << atBoolToString(b_writable);
  
  // Readable
  andor3Error(AT_IsReadable(m_camera_handle, iPropName, &b_readable));
  DEB_TRACE() << "\tIsReadable = " << atBoolToString(b_readable);
  
  if ( ! b_readable ) {
    DEB_TRACE() << "Since the property is not readable at this time, we will stop here.";
    return i_err;
  }
  
  // Now getting the value itself : we absolutely need now to know the type of the feature :
  if ( Camera::Unknown == iPropType ) {
    DEB_TRACE() << "Could not retrieve information on a property of unknown type !!\n"
    << "Returning error code!!";
    return -1;
  }
  
  AT_64		i_Value;
  double	d_Value;
  AT_BOOL	b_Value;
  int		enum_Value;
  int		enum_Count;
  AT_WC		s_Value[1024];
  int		s_MaxLen;
  
  switch (iPropType) {
    case Camera::Int:
      DEB_TRACE() << "\tFeature of type INTEGER";
      
      if ( AT_SUCCESS == andor3Error(AT_GetInt(m_camera_handle, iPropName, &i_Value)) )
        DEB_TRACE() << "\tValue = " << i_Value;
      else
        DEB_TRACE() << "\tError message : " << m_camera_error_str;
      
      if ( AT_SUCCESS == andor3Error(AT_GetIntMax(m_camera_handle, iPropName, &i_Value)) )
        DEB_TRACE() << "\tMax value = " << i_Value;
      else
        DEB_TRACE() << "\tError message : " << m_camera_error_str;
      
      if ( AT_SUCCESS == andor3Error(AT_GetIntMin(m_camera_handle, iPropName, &i_Value)) )
        DEB_TRACE() << "\tMin value = " << i_Value;
      else
        DEB_TRACE() << "\tError message : " << m_camera_error_str;
      break;
      
    case Camera::Float:
      DEB_TRACE() << "\tFeature of type FLOAT";
      if ( AT_SUCCESS == andor3Error(AT_GetFloat(m_camera_handle, iPropName, &d_Value)) )
        DEB_TRACE() << "\tValue = " << d_Value;
      else
        DEB_TRACE() << "\tError message : " << m_camera_error_str;
      if ( AT_SUCCESS == andor3Error(AT_GetFloatMax(m_camera_handle, iPropName, &d_Value)) )
        DEB_TRACE() << "\tMax value = " << d_Value;
      else
        DEB_TRACE() << "\tError message : " << m_camera_error_str;
      if ( AT_SUCCESS == andor3Error(AT_GetFloatMin(m_camera_handle, iPropName, &d_Value)) )
        DEB_TRACE() << "\tMin value = " << d_Value;
      else
        DEB_TRACE() << "\tError message : " << m_camera_error_str;
      break;
      
    case Camera::Bool:
      DEB_TRACE() << "\tFeature of type BOOLEAN";
      if ( AT_SUCCESS == andor3Error(AT_GetBool(m_camera_handle, iPropName, &b_Value)) )
        DEB_TRACE() << "\tValue = " << atBoolToString(b_Value);
      else
        DEB_TRACE() << "\tError message : " << m_camera_error_str;
      break;
      
    case Camera::Enum:
      DEB_TRACE() << "\tFeature of type ENUM";
      if ( AT_SUCCESS == andor3Error(AT_GetEnumIndex(m_camera_handle, iPropName, &enum_Value)) ) {
        DEB_TRACE() << "\tValue = (" << enum_Value << ")";
        if ( AT_SUCCESS == andor3Error(AT_GetEnumStringByIndex(m_camera_handle, iPropName, enum_Value, s_Value, 1024)) )
          DEB_TRACE() << " \"" << WStringToString(s_Value) << "\"";
        else
          DEB_TRACE() << "\tError message : " << m_camera_error_str;
        if ( AT_SUCCESS == andor3Error(AT_IsEnumIndexImplemented(m_camera_handle, iPropName, enum_Value, &b_Value)) )
          DEB_TRACE() << "; implemented = " << atBoolToString(b_Value);
        else
          DEB_TRACE() << "\tError message : " << m_camera_error_str;
        if ( AT_SUCCESS == andor3Error(AT_IsEnumIndexAvailable(m_camera_handle, iPropName, enum_Value, &b_Value)) )
          DEB_TRACE() << "; available = " << atBoolToString(b_Value);
        else
          DEB_TRACE() << "\tError message : " << m_camera_error_str;
        DEB_TRACE() << ".";
      }
      else
        DEB_TRACE() << "\tError message : " << m_camera_error_str;
      
      if ( AT_SUCCESS == andor3Error(AT_GetEnumCount(m_camera_handle, iPropName, &enum_Count)) ) {
        DEB_TRACE() << "\tAvailable choices are (" << enum_Count << ") :";
        for ( int i=0; enum_Count != i; ++i ) {
          DEB_TRACE() << "\t\t(" << i << ")";
          if ( AT_SUCCESS == andor3Error(AT_GetEnumStringByIndex(m_camera_handle, iPropName, i, s_Value, 1024)) )
            DEB_TRACE() << " \"" << WStringToString(s_Value) << "\"";
          else
            DEB_TRACE() << "\tError message : " << m_camera_error_str;
          if ( AT_SUCCESS == andor3Error(AT_IsEnumIndexImplemented(m_camera_handle, iPropName, i, &b_Value)) )
            DEB_TRACE() << "; implemented = " << atBoolToString(b_Value);
          else
            DEB_TRACE() << "\tError message : " << m_camera_error_str;
          if ( AT_SUCCESS == andor3Error(AT_IsEnumIndexAvailable(m_camera_handle, iPropName, i, &b_Value)) )
            DEB_TRACE() << "; available = " << atBoolToString(b_Value);
          else
            DEB_TRACE() << "\tError message : " << m_camera_error_str;
          DEB_TRACE() << ".";
        }
      }
      else
        DEB_TRACE() << "\tError message : " << m_camera_error_str;
      
      break;
      
    case Camera::String :
      DEB_TRACE() << "\tFeature of type STRING";
      if ( AT_SUCCESS == andor3Error(AT_GetString(m_camera_handle, iPropName, s_Value, 1024)) )
        DEB_TRACE() << "\tValue = \"" <<  WStringToString(s_Value) << "\"";
      else
        DEB_TRACE() << "\tError message : " << m_camera_error_str;
      
      if ( AT_SUCCESS == andor3Error(AT_GetStringMaxLength(m_camera_handle, iPropName, &s_MaxLen)) ) {
        DEB_TRACE() << "\tMaximum length of this property's string = " << s_MaxLen << ".";
      }
      else
        DEB_TRACE() << "\tError message : " << m_camera_error_str;
      break;
      
    default:
      DEB_TRACE() << "\tNot TREATED case so far !!!";
      break;
  }
  return i_err;
}

int
lima::Andor3::Camera::setInt(const AT_WC* Feature, AT_64 Value)
{
  DEB_MEMBER_FUNCT();
  return andor3Error(AT_SetInt(m_camera_handle, Feature, Value));
}

int
lima::Andor3::Camera::getInt(const AT_WC* Feature, AT_64* Value) const
{
  DEB_MEMBER_FUNCT();
  return andor3Error(AT_GetInt(m_camera_handle, Feature, Value));
}

int
lima::Andor3::Camera::getIntMax(const AT_WC* Feature, AT_64* MaxValue) const
{
  DEB_MEMBER_FUNCT();
  return andor3Error(AT_GetIntMax(m_camera_handle, Feature, MaxValue));
}

int
lima::Andor3::Camera::getIntMin(const AT_WC* Feature, AT_64* MinValue) const
{
  DEB_MEMBER_FUNCT();
  return andor3Error(AT_GetIntMin(m_camera_handle, Feature, MinValue));
}

int
lima::Andor3::Camera::setFloat(const AT_WC* Feature, double Value)
{
  DEB_MEMBER_FUNCT();
  return andor3Error(AT_SetFloat(m_camera_handle, Feature, Value));
}

int
lima::Andor3::Camera::getFloat(const AT_WC* Feature, double* Value) const
{
  DEB_MEMBER_FUNCT();
  return andor3Error(AT_GetFloat(m_camera_handle, Feature, Value));
}

int
lima::Andor3::Camera::getFloatMax(const AT_WC* Feature, double* MaxValue) const
{
  DEB_MEMBER_FUNCT();
  return andor3Error(AT_GetFloatMax(m_camera_handle, Feature, MaxValue));
}

int
lima::Andor3::Camera::getFloatMin(const AT_WC* Feature, double* MinValue) const
{
  DEB_MEMBER_FUNCT();
  return andor3Error(AT_GetFloatMin(m_camera_handle, Feature, MinValue));
}

int
lima::Andor3::Camera::setBool(const AT_WC* Feature, bool Value)
{
  DEB_MEMBER_FUNCT();
  AT_BOOL newBool = Value;
  return andor3Error(AT_SetBool(m_camera_handle, Feature, newBool));
}

int
lima::Andor3::Camera::getBool(const AT_WC* Feature, bool* Value) const
{
  DEB_MEMBER_FUNCT();
  AT_BOOL	newBool;
  int i_Err = andor3Error(AT_GetBool(m_camera_handle, Feature, &newBool));
  *Value = newBool;
  return i_Err;
}

int
lima::Andor3::Camera::setEnumIndex(const AT_WC* Feature, int Value)
{
  DEB_MEMBER_FUNCT();
  return andor3Error(AT_SetEnumIndex(m_camera_handle, Feature, Value));
}

int
lima::Andor3::Camera::setEnumString(const AT_WC* Feature, const AT_WC* String)
{
  DEB_MEMBER_FUNCT();
  return andor3Error(AT_SetEnumString(m_camera_handle, Feature, String));
}

int
lima::Andor3::Camera::getEnumIndex(const AT_WC* Feature, int* Value) const
{
  DEB_MEMBER_FUNCT();
  return andor3Error(AT_GetEnumIndex(m_camera_handle, Feature, Value));
}

int
lima::Andor3::Camera::getEnumString(const AT_WC* Feature, AT_WC* String, int StringLength) const
{
  DEB_MEMBER_FUNCT();
  int Value;
  int i_Err = andor3Error(AT_GetEnumIndex(m_camera_handle, Feature, &Value));
  if ( AT_SUCCESS != i_Err )
    return i_Err;
  return andor3Error(AT_GetEnumStringByIndex(m_camera_handle, Feature, Value, String, StringLength));
}

int
lima::Andor3::Camera::getEnumCount(AT_H m_camera_handle,const  AT_WC* Feature, int* Count) const
{
  DEB_MEMBER_FUNCT();
  return andor3Error(AT_GetEnumCount(m_camera_handle, Feature, Count));
}

int
lima::Andor3::Camera::isEnumIndexAvailable(const AT_WC* Feature, int Index, bool* Available) const
{
  DEB_MEMBER_FUNCT();
  AT_BOOL  isAvailable;
  int i_Err = andor3Error(AT_IsEnumIndexAvailable(m_camera_handle, Feature, Index, &isAvailable));
  *Available = isAvailable;
  return i_Err;
}
int
lima::Andor3::Camera::isEnumIndexImplemented(const AT_WC* Feature, int Index, bool* Implemented) const
{
  DEB_MEMBER_FUNCT();
  AT_BOOL  isImplemented;
  int i_Err = andor3Error(AT_IsEnumIndexAvailable(m_camera_handle, Feature, Index, &isImplemented));
  *Implemented = isImplemented;
  return i_Err;
}

int
lima::Andor3::Camera::getEnumStringByIndex(const AT_WC* Feature, int Index, AT_WC* String, int StringLength) const
{
  DEB_MEMBER_FUNCT();
  return andor3Error(AT_GetEnumStringByIndex(m_camera_handle, Feature, Index, String, StringLength));
}

int
lima::Andor3::Camera::getEnumIndexByString(const AT_WC* Feature, AT_WC* String, int *Index) const
{
  DEB_MEMBER_FUNCT();
  
  int       i_Err;
  int   		i_enumCount;
  int     	i_enumIndex;
  const int i_maxStringLen = 1024;
  AT_WC   	wcs_enumString[i_maxStringLen + 5];
  
  if ( AT_SUCCESS != (i_Err = getEnumCount(m_camera_handle, Feature, &i_enumCount)) ) {
    DEB_ERROR() << "Failed to get Enum Count" << " : error code = " << m_camera_error_str;
    return i_Err;
  }
  for (i_enumIndex = 0; i_enumCount != i_enumIndex; ++i_enumIndex) {
    if ( AT_SUCCESS != getEnumStringByIndex(Feature, i_enumIndex, wcs_enumString, i_maxStringLen) ) {
      DEB_ERROR() << "Failed to get Enum String" << " : error code = " << m_camera_error_str;
      return i_Err;
    }
    if ( ! wcscmp(wcs_enumString, String) ) {
      break;
    }
  }
  if ( i_enumCount == i_enumIndex ) {
    DEB_ERROR() << "Unable to find index of enum string '" << WStringToString(String) << "' in '" << WStringToString(Feature) << "' : no such choice.";
    *Index = -1;
    i_Err = AT_ERR_INDEXNOTAVAILABLE;
  }
  else {
    *Index = i_enumIndex;
    i_Err = AT_SUCCESS;
  }
  return i_Err;
}

int
lima::Andor3::Camera::setString(const AT_WC* Feature, const AT_WC* String)
{
  DEB_MEMBER_FUNCT();
  return andor3Error(AT_SetString(m_camera_handle, Feature, String));
}

int
lima::Andor3::Camera::getString(const AT_WC* Feature, AT_WC* String, int StringLength) const
{
  DEB_MEMBER_FUNCT();
  return andor3Error(AT_GetString(m_camera_handle, Feature, String, StringLength));
}

int
lima::Andor3::Camera::sendCommand(const AT_WC* Feature)
{
  DEB_MEMBER_FUNCT();
  return andor3Error(AT_Command(m_camera_handle, Feature));
}

/*
 Local Variables:
 c-file-style: "gnu"
 End:
 */
