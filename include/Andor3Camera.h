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
#include <atutility.h>

// LImA headers :
#include "lima/HwMaxImageSizeCallback.h"
#include "lima/HwBufferMgr.h"

// Andor3 plugin headers :

namespace lima
{
  namespace Andor3
  {
    /*******************************************************************
     * \class Camera
     * \brief object controlling the andor3 camera via andor3 SDK driver
     *******************************************************************/
    class Camera : public HwMaxImageSizeCallbackGen
    {
      friend class Interface;

      DEB_CLASS_NAMESPC(DebModCamera, "Camera", "Andor3");

    public:
      
      enum Status { Ready, Exposure, Readout, Latency, Fault };
      enum A3_TypeInfo { Unknown, Int, Float, Bool, Enum, String };

      //! @TODO : later on should do a map (string to int and vice-versa) from parsed enum info for the next 3 :
      // In the same order/index as "PreAmpGainControl"
      enum A3_Gain { Gain1 = 0, Gain2 = 1, Gain3 = 2, Gain4 = 3, Gain1_Gain3 = 4, Gain1_Gain4 = 5, Gain2_Gain3 = 6, Gain2_Gain4 = 7 };
      // The "simple" version :
      enum A3_SimpleGain { b11_hi_gain=0, b11_low_gain=1, b16_lh_gain=2, none=31};
      // In the same order/index as "ElectronicShutteringMode"
      enum A3_ShutterMode { Rolling = 0, Global = 1 };
      // In the same order/index as "PixelReadoutRate"
      enum A3_ReadOutRate { MHz10 = 0, MHz100 = 1, MHz200 = 2, MHz280 = 3 };
      // In the same order/index as 'BitDepth'
      enum A3_BitDepth { b11 = 0, b16= 1 };
      // The camera trigger mode (in the enum order) :
      enum A3_TriggerMode { Internal = 0, ExternalLevelTransition = 1, ExternalStart = 2, ExternalExposure = 3, Software = 4, Advanced = 5, External = 6 };
      // The binning system of andor3 :
      enum A3_Binning { B1x1=0, B2x2=1, B3x3=2, B4x4=3, B8x8=4};
      // The fan speed
      enum A3_FanSpeed { Off=0, Low=1, On=2};
      enum A3_PixelEncoding {Mono12=0, Mono12Packed = 1, Mono16=2, Mono32=3};
      
      struct SdkFrameDim {       
	AT_64 width;
	AT_64 height;
	AT_64 stride;
	double bytes_per_pixel;
	AT_64 size;
	A3_PixelEncoding encoding;
	const AT_WC* input_encoding;
	const AT_WC* output_encoding;
	bool is_encoded;
	bool is_strided;
      };
      
      int getHwBitDepth(int *bit_depth);
      
      Camera(const std::string& bitflow_path, int camera_number=0);
      ~Camera();

      // Preparing the camera's SDK to acquire frames
      void prepareAcq();
      // Launches the SDK's acquisition and the m_acq_thread to retrieve frame buffers as they are ready
      void startAcq();
      // Stops the acquisition, as soon as the m_acq_thread is retrieving frame buffers.
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
      bool checkTrigMode(TrigMode mode);
      void setTrigMode(TrigMode  mode);
      void getTrigMode(TrigMode& mode);
      
      void setExpTime(double  exp_time);
      void getExpTime(double& exp_time);
      
      void getExposureTimeRange(double& min_expo, double& max_expo) const;
      void getLatTimeRange(double& min_lat, double& max_lat) const;
            
      void setNbFrames(int  nb_frames);
      void getNbFrames(int& nb_frames);
      void getNbHwAcquiredFrames(int &nb_acq_frames);
      
      void checkRoi(const Roi& set_roi, Roi& hw_roi);
      void setRoi(const Roi& set_roi);
      void getRoi(Roi& hw_roi);
      
      bool isBinningAvailable();
      void checkBin(Bin& ioBin);
      void setBin(const Bin& iBin);
      void getBin(Bin& oBin);
      
      void setShutterMode(ShutterMode mode);
      void getShutterMode(ShutterMode& mode);
      
      void getPixelSize(double& sizex, double& sizey);

      void getStatus(Camera::Status& status);
      
      // --- Acquisition interface
      void reset();
      int getNbHwAcquiredFrames();

      // -- andor3 specific, LIMA don't worry about it !
      void initialiseController();

      // AdcGain is deprecated, should use SimpleGain (Zyla does not have AdcGain feature).
      void setAdcGain(A3_Gain iGain);					
      void getAdcGain(A3_Gain &oGain) const;
      void getAdcGainString(std::string &oGainString) const;

      void setAdcRate(A3_ReadOutRate iRate);  // à exporter (avec le get)
      void getAdcRate(A3_ReadOutRate &oRate) const;
      void getAdcRateString(std::string &oRateString) const;
      void setElectronicShutterMode(A3_ShutterMode iMode);  // à exporter (avec le get)
      void getElectronicShutterMode(A3_ShutterMode &oMode) const;
      void getElectronicShutterModeString(std::string &oModeString) const;
      void setBitDepth(A3_BitDepth iMode);
      void getBitDepth(A3_BitDepth &oMode) const;
      void getBitDepthString(std::string &oDepthString) const;
      void getPxEncoding(A3_PixelEncoding &oPxEncoding) const;
      void getPxEncodingString(std::string &oPxEncoding) const;
      void setTriggerMode(A3_TriggerMode iMode);
      void getTriggerMode(A3_TriggerMode &oMode) const;
      void getTriggerModeString(std::string &oModeString) const;
      void setTemperatureSP(double temp);  // à exporter (avec le get)
      void getTemperatureSP(double& temp) const;
      void getTemperature(double& temp) const;   // à exporter (read-only)
      void setCooler(bool flag);					 // à exporter (avec le get)
      void getCooler(bool& flag) const;
      void getCoolingStatus(std::string& status) const;  // à exporter (read-only)
      
      void setBufferOverflow(bool i_overflow);
      void getBufferOverflow(bool &o_overflow) const;
      void setFanSpeed(A3_FanSpeed iFS);
      void getFanSpeed(A3_FanSpeed &oFS) const;
      void getFanSpeedString(std::string &oFSString) const;
      void setOverlap(bool i_overlap);
      void getOverlap(bool &o_overlap) const;
      void setSimpleGain(A3_SimpleGain i_gain);
      void getSimpleGain(A3_SimpleGain &o_gain) const;
      void getSimpleGainString(std::string &o_gainString) const;
      void setSpuriousNoiseFilter(bool i_filter);
      void getSpuriousNoiseFilter(bool &o_filter) const;
      void setSyncTriggering(bool i_sync);
      void getSyncTriggering(bool &o_sync) const;
      
      // -- some readonly attributes :
      void getBytesPerPixel(double &o_value) const;
      void getFirmwareVersion(std::string &o_fwv) const;
      void setFrameRate(double i_frame_rate);
      void getFrameRate(double &o_frame_rate) const;
      void getFrameRateRange(double& o_min_lat, double& o_max_lat) const;
      void getFullRoiControl(bool &o_fullROIcontrol) const;
      void getImageSize(int &o_frame_size) const;
      void getMaxFrameRateTransfer(double &o_max_transfer_rate) const;
      void getReadoutTime(double &o_time) const;
      void getSerialNumber(std::string &o_sn) const;
      int getPixelStride() const;
      void getSdkFrameDim(SdkFrameDim &frame_dim,bool last);
      HwBufferCtrlObj* getTempBufferCtrlObj();


    protected:
      virtual void setMaxImageSizeCallbackActive(bool cb_active);

    private:
       
      // -- some internals :
      // Stopping an acquisition, iForce : without waiting the end of frame buffer retrieval by m_acq_thread
      void _stopAcq(bool iImmediate);
      // Setting the status in a thread safe manner :
      void _setStatus(Camera::Status iStatus, bool iForce);
      
    private:
      // -- andor3 Lower level functions
      int printInfoForProp(const AT_WC * iPropName, A3_TypeInfo iPropType) const;
      bool propImplemented(const AT_WC * iPropName) const;
      
      static int getIntSystem(const AT_WC* Feature, AT_64* Value);
      //      static int bufferOverflowCallback(AT_H i_handle, const AT_WC* i_feature, void* i_info);
      
      int setInt(const AT_WC* Feature, AT_64 Value);
      int getInt(const AT_WC* Feature, AT_64* Value) const;
      int getIntMax(const AT_WC* Feature, AT_64* MaxValue) const;
      int getIntMin(const AT_WC* Feature, AT_64* MinValue) const;
      
      int setFloat(const AT_WC* Feature, double Value);
      int getFloat(const AT_WC* Feature, double* Value) const;
      int getFloatMax(const AT_WC* Feature, double* MaxValue) const;
      int getFloatMin(const AT_WC* Feature, double* MinValue) const;
      
      int setBool(const AT_WC* Feature, bool Value);
      int getBool(const AT_WC* Feature, bool* Value) const;
      
      int setEnumIndex(const AT_WC* Feature, int Value);
      int setEnumString(const AT_WC* Feature, const AT_WC* String);
      int getEnumIndex(const AT_WC* Feature, int* Value) const;
      int getEnumString(const AT_WC* Feature, AT_WC* String, int StringLength) const;
      int getEnumCount(AT_H Hndl,const  AT_WC* Feature, int* Count) const;
      int isEnumIndexAvailable(const AT_WC* Feature, int Index, bool* Available) const;
      int isEnumIndexImplemented(const AT_WC* Feature, int Index, bool* Implemented) const;
      int getEnumStringByIndex(const AT_WC* Feature, int Index, AT_WC* String, int StringLength) const;
      int getEnumIndexByString(const AT_WC* Feature, AT_WC* String, int *Index) const;
      
      int setString(const AT_WC* Feature, const AT_WC* String);
      int getString(const AT_WC* Feature, AT_WC* String, int StringLength) const;
      
      int sendCommand(const AT_WC* Feature);

    private:
      class _AcqThread;
      class _BufferCtrlObj;
      friend class _AcqThread;

      // -- Members
      // LIMA / Acquisition (thread) related :
      SoftBufferCtrlObj*          m_buffer_ctrl_obj;
      SoftBufferCtrlObj*          m_temp_buffer_ctrl_obj;
      // Pure thread and signals :
      _AcqThread*                 m_acq_thread;		   // The thread retieving frame buffers from the SDK
      Cond                        m_cond;		   // Waiting condition for inter thread signaling
      volatile bool               m_acq_thread_waiting;    // The m_acq_thread is waiting (main uses it to tell it to stop waiting)
      volatile bool               m_acq_thread_running;	   // The m_acq_thread is running (main uses it to accept stopAcq)
      volatile bool	          m_acq_thread_should_quit;// The main thread signals to m_acq_thread that it should quit.

      // A bit more general :
      size_t                      m_nb_frames_to_collect;  // The number of frames to collect in current sequence
      size_t                      m_image_index;	   // The index in the current sequence of the next image to retrieve
      bool                        m_buffer_ringing;	   // Should the buffer be considered as a ring buffer rather than a single use buffer.
      Status                      m_status;	           // The current status of the camera
      
      // LIMA / Not directly acquisition related :
      bool                        m_real_camera;	   // Set to false for CameraModel == "SIMCAM CMOS"
      std::string                 m_detector_model;
      std::string                 m_detector_type;
      std::string		  m_detector_serial;
      Size			  m_detector_size;
      double		          m_exp_time;

      // -- andor3 SDK stuff
      std::string                 m_bitflow_path;
      int                         m_camera_number;
      AT_H                        m_camera_handle;
      A3_Gain                     m_adc_gain;
      A3_SimpleGain               m_simple_gain;
      A3_ReadOutRate	          m_adc_rate;
      A3_ShutterMode		  m_electronic_shutter_mode;
      A3_BitDepth                 m_bit_depth;
      A3_TriggerMode              m_trig_mode;
      bool                        m_cooler;
      double                      m_temperature_sp;
      bool                        m_temperature_control_available;

      // std::map<TrigMode, int>     m_trig_mode_maps;

      static int                  sAndorSDK3InittedCounter;
    
      bool  m_maximage_size_cb_active;
      Camera::SdkFrameDim m_sdk_frame_dim;
    };
    
    // Some inline utility functions; used all-over in Andor3 plugin :
    inline std::wstring StringToWString(const std::string & iStr)
    {
      wchar_t		tmpWStringBuf[1024];
      
      mbstowcs(tmpWStringBuf, iStr.c_str(), 1023);
      return std::wstring(tmpWStringBuf);
      /*std::wostringstream		theWSStream;
       theWSStream << iStr.c_str();
       return theWSStream.str();
       */
    }
    
    inline std::string WStringToString(const std::wstring & iStr)
    {
      // Should use wcstombs
      char			tmpStringBuf[1024];
      
      bzero(tmpStringBuf, 1024);
      wcstombs(tmpStringBuf, iStr.c_str(), 1023);
      return std::string(tmpStringBuf);
      /*    std::ostringstream			theSStream;
       theSStream << iStr.c_str();
       return theSStream.str();
       */
    }
    
    inline std::string atBoolToString(AT_BOOL iBool)
    {
      return (iBool) ? std::string("true") : std::string("false");
    }

  } // namespace Andor3
} // namespace lima


#endif  /* ANDOR3CAMERA_H */

/*
 Local Variables:
 c-file-style: "gnu"
 End:
 */
