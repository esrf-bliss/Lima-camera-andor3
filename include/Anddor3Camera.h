#ifndef ANDOR3CAMERA_H
#define ANDOR3CAMERA_H

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

#if defined (__GNUC__) && (__GNUC__ == 3) && defined (__ELF__)
#   define GENAPI_DECL __attribute__((visibility("default")))
#   define GENAPI_DECL_ABSTRACT __attribute__((visibility("default")))
#endif

// System headers :
#include <stdlib.h>
#include <limits>
#include <ostream>

// Camera SDK headers :
#include <atcore.h>

// LImA headers :
#include "HwMaxImageSizeCallback.h"
#include "HwBufferMgr.h"

// Andor3 plugin headers :

namespace lima
{
  namespace Andor3
  {
    /*******************************************************************
     * \class Camera
     * \brief object controlling the andor3 camera via andor3 SDK driver
     *******************************************************************/
    class Camera
    {
      DEB_CLASS_NAMESPC(DebModCamera, "Camera", "Andor3");
    public:
      
      enum Status { Ready, Acquisition, Fault };
      enum A3_TypeInfo { Unknown, Int, Float, Bool, Enum, String };

      //! @TODO : later on should do a map (string to int and vice-versa) from parsed enum info for the next 3 :
      // In the same order/index as "PreAmpGainControl"
      enum A3_Gain { Gain1 = 0, Gain2 = 1, Gain3 = 2, Gain4 = 3, Gain1_Gain3 = 4, Gain1_Gain4 = 5, Gain2_Gain3 = 6, Gain2_Gain4 = 7 };
      // In the same order/index as "ElectronicShutteringMode"
      enum A3_ShutterMode { Rolling = 0, Global = 1 };
      // In the same order/index as "PixelReadoutRate"
      enum A3_ReadOutRate { MHz10 = 0, MHz100 = 1, MHz200 = 2, MHz280 = 3 };

      
      Camera(const std::string& bitflow_path,int camera_number=0);
      ~Camera();

      void prepareAcq();
      void startAcq();
      void stopAcq();

      // -- detector info object
      void getImageType(ImageType& type);
      void setImageType(ImageType type);
      
      void getDetectorType(std::string& type);
      void getDetectorModel(std::string& model);
      void getDetectorImageSize(Size& size);

      // -- Buffer control object
      HwBufferCtrlObj* getBufferCtrlObj();
      
      //-- Synch control object
      void setTrigMode(TrigMode  mode);
      void getTrigMode(TrigMode& mode);
      
      void setExpTime(double  exp_time);
      void getExpTime(double& exp_time);
      
      void setLatTime(double  lat_time);
      void getLatTime(double& lat_time);
      
      void getExposureTimeRange(double& min_expo, double& max_expo) const;
      void getLatTimeRange(double& min_lat, double& max_lat) const;
      
      void setNbFrames(int  nb_frames);
      void getNbFrames(int& nb_frames);
      void getNbHwAcquiredFrames(int &nb_acq_frames);
      
      void checkRoi(const Roi& set_roi, Roi& hw_roi);
      void setRoi(const Roi& set_roi);
      void getRoi(Roi& hw_roi);
      
      bool isBinningAvailable();
      void checkBin(Bin&);
      void setBin(const Bin&);
      void getBin(Bin&);
      
      void setShutterMode(ShutterMode mode);
      void getShutterMode(ShutterMode& mode);
      
      void setShutter(bool flag);
      void getShutter(bool& flag);
      
      void getPixelSize(double& sizex, double& sizey);

      void getStatus(Camera::Status& status);
      
      // --- Acquisition interface
      void reset();
      int getNbHwAcquiredFrames();

      // -- andor3 specific, LIMA don't worry about it !
      void initialiseController();

      void setAdcGain(A3_Gain iGain);
      void getAdcGain(A3_Gain &oGain);
      void setAdcRate(A3_ReadOutRate iRate);
      void getAdcRate(A3_ReadOutRate &oRate);
      void setElectronicShutterMode(A3_ShutterMode iMode);
      void getElectronicShutterMode(A3_ShutterMode &oMode);
      void setTemperatureSP(double temp);
      void getTemperatureSP(double& temp);
      void getTemperature(double& temp);
      void setCooler(bool flag);
      void getCooler(bool& flag);
      void getCoolingStatus(std::string& status);
      
      // -- andor3 Lower level functions
      bool andor3Error(int code) const;
      void _mapAndor3Error();
      
      int printInfoForProp(AT_H iCamHandle, const AT_WC * iPropName, A3_TypeInfo iPropType);
      
      int setInt(AT_H Hndl, const AT_WC* Feature, AT_64 Value);
      int getInt(AT_H Hndl, const AT_WC* Feature, AT_64* Value) const;
      int getIntMax(AT_H Hndl, const AT_WC* Feature, AT_64* MaxValue) const;
      int getIntMin(AT_H Hndl, const AT_WC* Feature, AT_64* MinValue) const;
      
      int setFloat(AT_H Hndl, const AT_WC* Feature, double Value);
      int getFloat(AT_H Hndl, const AT_WC* Feature, double* Value) const;
      int getFloatMax(AT_H Hndl, const AT_WC* Feature, double* MaxValue) const;
      int getFloatMin(AT_H Hndl, const AT_WC* Feature, double* MinValue) const;
      
      int setBool(AT_H Hndl, const AT_WC* Feature, bool Value);
      int getBool(AT_H Hndl, const AT_WC* Feature, bool* Value) const;
      
      int setEnumIndex(AT_H Hndl, const AT_WC* Feature, int Value);
      int setEnumString(AT_H Hndl, const AT_WC* Feature, const AT_WC* String);
      int getEnumIndex(AT_H Hndl, const AT_WC* Feature, int* Value) const;
      int getEnumString(AT_H Hndl, const AT_WC* Feature, AT_WC* String, int StringLength) const;
      int getEnumCount(AT_H Hndl,const  AT_WC* Feature, int* Count) const;
      int isEnumIndexAvailable(AT_H Hndl, const AT_WC* Feature, int Index, bool* Available) const;
      int isEnumIndexImplemented(AT_H Hndl, const AT_WC* Feature, int Index, bool* Implemented) const;
      int getEnumStringByIndex(AT_H Hndl, const AT_WC* Feature, int Index, AT_WC* String, int StringLength) const;
      int getEnumIndexByString(AT_H Hndl, const AT_WC* Feature, AT_WC* String, int *Index) const;
      
      int setString(AT_H Hndl, const AT_WC* Feature, const AT_WC* String);
      int getString(AT_H Hndl, const AT_WC* Feature, AT_WC* String, int StringLength) const;
      
      int sendCommand(AT_H Hndl, const AT_WC* Feature);

      
    private:
      class _AcqThread;
      friend class _AcqThread;

      // -- Members
      _AcqThread*                 m_acq_thread;
      Cond                        m_cond;

      string                      m_detector_model;
      string                      m_detector_type;
      string											m_detector_serial;

      // -- andor3 SDK stuff
      string                      m_bitflow_path;
      int                         m_camera_number;
      AT_H                        m_camera_handle;
      mutable string              m_camera_error_str;
      mutable int                 m_camera_error;
      A3_Gain											m_adc_gain;
      A3_ReadOutRate							m_adc_rate;
      A3_ShutterMode							m_electronic_shutter_mode;
      map<TrigMode, int>          m_trig_mode_maps;
      map<int, string>            m_andor3_error_maps;

      static bool						sAndorSDK3Initted;
    };
  } // namespace Andor3
} // namespace lima


#endif  /* ANDOR3CAMERA_H */

/*
 Local Variables:
 c-file-style: "gnu"
 End:
 */
