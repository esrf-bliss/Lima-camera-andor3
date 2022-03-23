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

//static methods predefinition
static const char* error_code(int error_code);

#define THROW_IF_NOT_SUCCESS(command,error_prefix)			\
{									\
  int ret_code = command;						\
  if ( AT_SUCCESS != ret_code )						\
    THROW_HW_ERROR(Error) << error_prefix << DEB_VAR1(error_code(ret_code)); \
}

// Defining the parameter names of the andor3 SDK :
namespace lima {
  namespace andor3 {
    static const AT_WC* AcquisitionStart = L"AcquisitionStart";
    static const AT_WC* AcquisitionStop = L"AcquisitionStop";
    static const AT_WC* AOIBinning = L"AOIBinning";
    static const AT_WC* AOIHBin = L"AOIHBin";
    static const AT_WC* AOIHeight = L"AOIHeight";
    static const AT_WC* AOILeft = L"AOILeft";
    static const AT_WC* AOIStride = L"AOIStride";
    static const AT_WC* AOITop = L"AOITop";
    static const AT_WC* AOIVBin = L"AOIVBin";
    
    static const AT_WC* AOIWidth = L"AOIWidth";
    static const AT_WC* BitDepth = L"BitDepth";
    static const AT_WC* BufferOverflowEvent = L"BufferOverflowEvent";
    static const AT_WC* BytesPerPixel = L"BytesPerPixel";
    static const AT_WC* CameraAcquiring = L"CameraAcquiring";
    static const AT_WC* CameraModel = L"CameraModel";
    static const AT_WC* CycleMode = L"CycleMode";
    static const AT_WC* DeviceCount = L"DeviceCount";

    static const AT_WC* ElectronicShutteringMode = L"ElectronicShutteringMode";
    static const AT_WC* ExposureTime = L"ExposureTime";
    static const AT_WC* FanSpeed = L"FanSpeed";
    static const AT_WC* FirmwareVersion = L"FirmwareVersion";
    static const AT_WC* FrameRate = L"FrameRate";
    static const AT_WC* FullAOIControl = L"FullAOIControl";
    static const AT_WC* ImageSizeBytes = L"ImageSizeBytes";
    static const AT_WC* MaxInterfaceTransferRate = L"MaxInterfaceTransferRate";
    
    static const AT_WC* Overlap = L"Overlap";
    static const AT_WC* PixelEncoding = L"PixelEncoding";
    static const AT_WC* PixelReadoutRate = L"PixelReadoutRate";
    static const AT_WC* PixelHeight = L"PixelHeight";
    static const AT_WC* PixelWidth = L"PixelWidth";
    static const AT_WC* PreAmpGainControl = L"PreAmpGainControl";
    static const AT_WC* ReadoutTime = L"ReadoutTime";
    static const AT_WC* SensorCooling = L"SensorCooling";
    static const AT_WC* SensorHeight = L"SensorHeight";
    static const AT_WC* SensorTemperature = L"SensorTemperature";
    static const AT_WC* SensorWidth = L"SensorWidth";
    static const AT_WC* SerialNumber = L"SerialNumber";
    static const AT_WC* SimplePreAmpGainControl = L"SimplePreAmpGainControl";
    static const AT_WC* SoftwareTrigger = L"SoftwareTrigger";

    static const AT_WC* SpuriousNoiseFilter = L"SpuriousNoiseFilter";
    static const AT_WC* SynchronousTriggering = L"SynchronousTriggering";
    static const AT_WC* TargetSensorTemperature = L"TagetSensorTemperature";
    static const AT_WC* TemperatureStatus = L"TemperatureStatus";
    static const AT_WC* TriggerMode = L"TriggerMode";
    static const AT_WC* FrameCount = L"FrameCount";

    static const AT_WC* IOSelector = L"IOSelector";
    static const AT_WC* IOInvert = L"IOInvert";
    static const AT_WC* AuxiliaryOutSource = L"AuxiliaryOutSource";

    // For future !!
    //    static const AT_WC* AccumulateCount = L"AccumulateCount";
    //    static const AT_WC* BaselineLevel = L"BaselineLevel";
    //    static const AT_WC* CameraDump = L"CameraDump";
    //    static const AT_WC* ControllerID = L"ControllerID";
    //    static const AT_WC* LUTIndex = L"LUTIndex";
    //    static const AT_WC* LUTValue = L"LUTValue";
    //    static const AT_WC* MetadataEnable = L"MetadataEnable";
    //    static const AT_WC* MetadataFrame = L"MetadataFrame";
    //    static const AT_WC* MetadataTimestamp = L"MetadataTimestamp";
    //    static const AT_WC* PixelCorrection = L"PixelCorrection";
    //    static const AT_WC* PreAmpGain = L"PreAmpGain"; // Deprecated
    //    static const AT_WC* PreAmpGainChannel = L"PreAmpGainChannel"; // Deprecated
    //    static const AT_WC* PreAmpGainSelector = L"PreAmpGainSelector"; // Deprecated
    //    static const AT_WC* RowNExposureEndEvent = L"RowNExposureEndEvent";
    //    static const AT_WC* RowNExposureStartEvent = L"RowNExposureStartEvent";
    //    static const AT_WC* SoftwareVersion = L"SoftwareVersion";
        static const AT_WC* TemperatureControl = L"TemperatureControl";
    //    static const AT_WC* TimestampClock = L"TimestampClock";
    //    static const AT_WC* TimestampClockFrequency = L"TimestampClockFrequency";
    //    static const AT_WC* TimestampClockReset = L"TimestampClockReset";
  }
}

//---------------------------
//- utility variables
//---------------------------
int lima::Andor3::Camera::sAndorSDK3InittedCounter = 0;
pthread_mutex_t sdkInitMutex = PTHREAD_MUTEX_INITIALIZER;

//---------------------------
//- utility thread
//---------------------------
namespace lima {
  namespace Andor3 {
    class Camera::_AcqThread : public Thread
    {
      DEB_CLASS_NAMESPC(DebModCamera, "Camera", "_AcqThread");
    public:
      _AcqThread(Camera &aCam);
      virtual ~_AcqThread();
      
    protected:
      virtual void threadFunction();
      
    private:
      Camera&    m_cam;
    };
  }
}


lima::Andor3::Camera::Camera(const std::string& bitflow_path, int camera_number) :
m_acq_thread(NULL),
m_cond(),
m_acq_thread_waiting(true),
m_acq_thread_running(false),
m_acq_thread_should_quit(false),
m_nb_frames_to_collect(1),
m_image_index(0),
m_buffer_ringing(false),
m_status(Ready),
m_real_camera(false),
m_detector_model("un-inited"),
m_detector_type("un-inited"),
m_detector_serial("un-inited"),
m_detector_size(1,1),
m_exp_time(1.0),
m_bitflow_path(bitflow_path),
m_camera_number(camera_number),
m_camera_handle(AT_HANDLE_UNINITIALISED),
m_adc_gain(Gain1_Gain4),
m_simple_gain(b16_lh_gain),
m_adc_rate(MHz100),
m_electronic_shutter_mode(Rolling),
m_bit_depth(b16),
m_trig_mode(IntTrig),
m_cooler(true),
m_temperature_sp(999.99),
m_has_temperature_sp(false),
m_has_temperature_control(false),
m_maximage_size_cb_active(false)
{
  DEB_CONSTRUCTOR();

  // Create the camera buffer
  m_buffer_ctrl_obj = new SoftBufferCtrlObj();
  // temporary buffer when the reconstruction task is active for decoding and destriding
  m_temp_buffer_ctrl_obj = new SoftBufferCtrlObj();

  // Initialisation of the atcore library :
  pthread_mutex_lock(&sdkInitMutex);
  if ( ! sAndorSDK3InittedCounter ) {
    if ( m_bitflow_path != "" ) {
      setenv("BITFLOW_INSTALL_DIRS", m_bitflow_path.c_str(), true);
    }
    else {
      setenv("BITFLOW_INSTALL_DIRS", "/usr/share/andor-3-sdk", false);
    }
    try
      {
	THROW_IF_NOT_SUCCESS(AT_InitialiseLibrary(),
			     "Library initialization failed, check the bitflow path");
	
	THROW_IF_NOT_SUCCESS(AT_InitialiseUtilityLibrary(),
			     "Library Utility initialization failed, check the bitflow path");
      }
    catch(...)
      {
	pthread_mutex_unlock(&sdkInitMutex);
	throw;
      }
    ++sAndorSDK3InittedCounter;
  }
  pthread_mutex_unlock(&sdkInitMutex);

  // --- Get available cameras and select the choosen one.
  AT_64 numCameras;
  DEB_TRACE() << "Get all attached cameras";
  THROW_IF_NOT_SUCCESS(getIntSystem(andor3::DeviceCount, &numCameras),
		       "No camera present!");

  DEB_TRACE() << "Found "<< numCameras << " camera" << ((numCameras>1)? "s": "");
  DEB_TRACE() << "Try to set current camera to number " << m_camera_number;
  
  if ( m_camera_number < numCameras && m_camera_number >=0 ) {
    // Getting the m_camera_handle WITH some error checking :
    THROW_IF_NOT_SUCCESS(AT_Open(m_camera_number, &m_camera_handle),
			 "Cannot get camera handle");
  }
  else {
    THROW_HW_ERROR(InvalidValue) << "Invalid camera number " 
				 << m_camera_number << ", there is "
				 << numCameras << " available";
  }
  
  // --- Get Camera model (and all other parameters which are not changing during camera setup and usage )
  AT_WC	model[1024];
  THROW_IF_NOT_SUCCESS(getString(andor3::CameraModel, model, 1024),
		       "Cannot get camera model");

  m_detector_model = WStringToString(std::wstring(model));
  
  // Adding some extra information on the model (more human readable) :
  // DC-152 -> Neo
  if ( ! m_detector_model.compare(0, 6, "DC-152 ")) {
    m_detector_model += "/Neo";
    m_detector_type = std::string("Neo");
  }
  // DG-152 -> Zyla-5.5
  if ( ! m_detector_model.compare(0, 6, "DG-152 ")) {
    m_detector_model += "/Zyla-5.5";
    m_detector_type = std::string("Zyla-5.5");
  }
  // ZYLA-4.2 -> Zyla-4.2
  if ( ! m_detector_model.compare(0, 6, "ZYLA-4.2 ")) {
    m_detector_model += "/Zyla-4.2";
    m_detector_type = std::string("Zyla-4.2");
  }
  // With latest (2014) firmware model is more human readable !!
  if (! m_detector_model.compare(0, 7, "ZYLA5.5")) {
    m_detector_type = std::string("Zyla-5.5");    
  }
  if (! m_detector_model.compare(0, 12, "MARANA-4BV11")) {
    m_detector_type = std::string("Marana-4.2");
  }
  
  if ( m_detector_model != "SIMCAM CMOS" ) {
    std::string		the_serial, the_fw;

    getSerialNumber(the_serial);
    getFirmwareVersion(the_fw);
    m_real_camera = true;
    m_detector_model += " (SN : " + the_serial + ", firmware " + the_fw + ")";
    
    DEB_ALWAYS() << "Camera is ready: type " << m_detector_type << ", model "<< m_detector_model;
  }
  else {
    m_real_camera = false;
    m_detector_type = std::string("Simulator");
    
    DEB_TRACE() << "The camera is indeed the SIMULATED camera, all exception for invalid parameter name will be ignored!!!";
    DEB_ALWAYS() << "BE VERY CAREFULL : The andor SDK3 camera that you are connected to is a SIMULATED CAMERA !!!";
  }

  // --- Get Camera Serial number
  AT_WC	serial[1024];
  THROW_IF_NOT_SUCCESS(getString(andor3::SerialNumber, serial, 1024),
		       "Cannot get camera serial number");

  m_detector_serial = WStringToString(std::wstring(serial));

  // --- Get Camera maximum image size 
  AT_64 xmax, ymax;
  // --- Get the max image size of the detector
  THROW_IF_NOT_SUCCESS(getInt(andor3::SensorWidth, &xmax),
		       "Cannot get detector X size");

  THROW_IF_NOT_SUCCESS(getInt(andor3::SensorHeight, &ymax),
		       "Cannot get detector Y size");

  m_detector_size= Size(static_cast<int>(xmax), static_cast<int>(ymax));

  // --- init trigger modes
  initTrigMode();

// --- to investiagte cam features
//  printInfoForProp(andor3::ElectronicShutteringMode, Enum);
//  printInfoForProp(andor3::PixelReadoutRate, Enum);
//  printInfoForProp(andor3::PixelEncoding, Enum);
//  printInfoForProp(andor3::SimplePreAmpGainControl, Enum);
//  printInfoForProp(andor3::FanSpeed, Enum);
//  printInfoForProp(andor3::BitDepth, Enum);
//  printInfoForProp(andor3::TemperatureControl, Enum);
//  printInfoForProp(andor3::AuxiliaryOutSource, Enum);

  // --- Initialise deeper parameters of the controller
  initialiseController();
  
}

lima::Andor3::Camera::~Camera()
{
  DEB_DESTRUCTOR();

  // Stop Acq thread
  _stopAcq(true);
  delete m_acq_thread;
  m_acq_thread = NULL;
 
  // Close camera
  if (m_cooler && m_has_temperature_sp) {
    DEB_ERROR() <<"Please stop the cooling before shuting dowm the camera\n"
    << "brutale heating could damage the sensor.\n"
    << "And wait until temperature rises above 5 deg, before shuting down.";

    DEB_ALWAYS() << "The cooler of the camera is ON!!!\n"
    << "we are now waiting the camera to warm-up slowly, "
    << "setting the temperature to 10C and waiting for it "
    << "to rise above 5C before continuing the shutdown.";
    //    setTemperatureSP(6.0);
    double			the_sensor_temperature;
    
    DEB_TRACE() << "While leaving the camera, the temperature provided by the cooler is " << the_sensor_temperature;
    getTemperature(the_sensor_temperature);
    // while ( the_sensor_temperature < 5.1 ) {
    //   sleep(1);
    //   ++the_sensor_temp_wait;
    //   getTemperature(the_sensor_temperature);
    //   DEB_TRACE() << "The temperature provided by the cooler is " << the_sensor_temperature << "(waited approx. " << the_sensor_temp_wait << "s)";
    // }
    // setCooler(false);
    
    //    THROW_HW_ERROR(Error)<<"Please stop the cooling before shuting dowm the camera\n"
    //    << "brutale heating could damage the sensor.\n"
    //    << "And wait until temperature rises above 5 deg, before shuting down.";
  }
  
  DEB_TRACE() << "Shutdown camera";

  THROW_IF_NOT_SUCCESS(AT_Close(m_camera_handle),"Cannot close the camera");

  m_camera_handle = AT_HANDLE_UNINITIALISED;

  pthread_mutex_lock(&sdkInitMutex);
  if(!--sAndorSDK3InittedCounter)
    {
      try
	{
	  THROW_IF_NOT_SUCCESS(AT_FinaliseUtilityLibrary(),
			       "Cannot finalise Andor SDK 3 Utility library");
	  
	  THROW_IF_NOT_SUCCESS(AT_FinaliseLibrary(),
			       "Cannot finalise Andor SDK 3 library");
	}
      catch(...)
	{
	  pthread_mutex_unlock(&sdkInitMutex);
	  throw;
	}
    }
  pthread_mutex_unlock(&sdkInitMutex);

  delete m_buffer_ctrl_obj;
  delete m_temp_buffer_ctrl_obj;
}


/*! 
 @brief Preparing the object and memory for an acquisition sequence
 
 This method is called just before the acquisition is started (by startAcq).
 It is responsible for setting both the internals of the object and the
 internals of the camera's SDK for the planned acquisition sequence.
 Most of the work consists in the setting of the frame buffers.
 */
/*!
 Maybe later : write a specific SoftBufferAllocMgr or SoftBufferCtrlObj
 subclass that better handles :
   * the stride of the image
   * the image size, the best being to get the value from the SDK
      itself though ImageSizeBytes
   * possibility to handle the metadata (in particular the 
      frame timestamp generated on the FPGA clock)
 */
void
lima::Andor3::Camera::prepareAcq()
{
  DEB_MEMBER_FUNCT();
  
  if ( ! m_real_camera ) {
    DEB_ALWAYS() << "BE VERY CAREFULL : The andor SDK3 camera that you are connected to is a SIMULATED CAMERA !!!";
  }
  
  // Setting the next image index to 0 (since we are starting at 0):
  m_image_index = 0;
  
  
  // Get from SDK the real frame size with stride (padding columns)
  std::string	the_bit_depth;
  FrameDim      the_frame_mem_dim;
  int           the_frame_mem_size;
  int           the_alloc_frames;
  SdkFrameDim   the_sdk_frame_dim;

  getSdkFrameDim(the_sdk_frame_dim, false);
    
  SoftBufferCtrlObj* the_buffer_ctrl_obj;

  if (the_sdk_frame_dim.is_encoded || the_sdk_frame_dim.is_strided) 
    {
      // prepare the temporary buffers 
      // just ajust the width to get a buffer size closed to the sdk frame size
      int the_temp_width = ceil(the_sdk_frame_dim.size/the_sdk_frame_dim.height/the_sdk_frame_dim.bytes_per_pixel)+1;
      
      the_frame_mem_dim.setImageType(Bpp16);
      the_frame_mem_dim.setSize(Size(the_temp_width, the_sdk_frame_dim.height));
      m_temp_buffer_ctrl_obj->setFrameDim(the_frame_mem_dim);
      the_alloc_frames = 16;
      m_temp_buffer_ctrl_obj->setNbBuffers(the_alloc_frames);

      the_frame_mem_size = the_sdk_frame_dim.size;
      the_buffer_ctrl_obj = m_temp_buffer_ctrl_obj;
     }
  else
    {
      m_buffer_ctrl_obj->getFrameDim(the_frame_mem_dim);
      the_frame_mem_size = the_frame_mem_dim.getMemSize();
      m_buffer_ctrl_obj->getNbBuffers(the_alloc_frames);      
      the_buffer_ctrl_obj = m_buffer_ctrl_obj;
    }

  // ringing flag in case of short buffer or continuous acquistion (0 frame)
  m_buffer_ringing = (m_nb_frames_to_collect == 0) || (int(m_nb_frames_to_collect) > the_alloc_frames );
  StdBufferCbMgr &the_buffer_mgr = the_buffer_ctrl_obj->getBuffer();


  // Handing the frame buffers to the SDK :
  // As proposed by the SDK, we make sure that we start from an empty queue :
  DEB_TRACE() << "Flushing the queue of the framegrabber";
  AT_Flush(m_camera_handle);
  
  // Then queue all the buffers allocated by StdBufferCbMgr
  
  DEB_TRACE() << "Pushing all the frame buffers to the frame grabber SDK";
  for ( int i=0; the_alloc_frames != i; ++i) {
    void*		the_buffer_ptr = the_buffer_mgr.getFrameBufferPtr(i);
    THROW_IF_NOT_SUCCESS(AT_QueueBuffer(m_camera_handle, static_cast<AT_U8*>(the_buffer_ptr), the_frame_mem_size),
			 "Cannot Queue the frame buffer");
    DEB_TRACE() << "Queueing the frame buffer " << i;
  }
  DEB_TRACE() << "Finished queueing " << the_alloc_frames << " frame buffers to andor's SDK3";
 
  // Better to use continuous mode, smaller ring-buffer allocated by SDK. L.Claustre
  // excepted if the trigger mode is IntTrigMult (e.i Software)
  if (m_trig_mode == IntTrigMult)
    {
      DEB_TRACE()<< "Software trigger ON, set CycleMode to Fixed mode";
      AT_64  nb_frames = static_cast<AT_64>(m_nb_frames_to_collect);
      setInt(andor3::FrameCount, nb_frames);
      setEnumString(andor3::CycleMode, L"Fixed");    
    }
  else
    {
      setEnumString(andor3::CycleMode, L"Continuous");    
    }
  
  AT_WC          the_string[256];
  getEnumString(andor3::CycleMode, the_string, 255); 
  DEB_TRACE() << "At the end of prepareAcq, the CycleMode is " << WStringToString(std::wstring(the_string));
}


/*! 
 @brief Launching an acquisition sequence, including the frame retrieving thread.
 
 This method is called at the launch of the acquisition sequence, after the
 prepareAcq method but before any frame can be shot.
 
 This method is responsible to perform the following actions :
   * Tell the SDK to launch the acquisition sequence
   * Signal to the m_acq_thread that it should resume running
       because some frames will soon be available to retrieve
 */
void
lima::Andor3::Camera::startAcq()
{
  DEB_MEMBER_FUNCT();
  
  if ( ! m_real_camera ) {
    DEB_ALWAYS() << "BE VERY CAREFULL : The andor SDK3 camera that you are connected to is a SIMULATED CAMERA !!!";
  }

  DEB_TRACE() << "Starting the acquisition by the camera (or triggering when in software trigger mode)";

  if ( 0 == m_image_index ) {
    // Setting the start timestamp of the buffer :
    m_buffer_ctrl_obj->getBuffer().setStartTimestamp(Timestamp::now());
    // Sending the start command to the SDK :
    sendCommand(andor3::AcquisitionStart);
  }
  
  if ( IntTrigMult == m_trig_mode ) {
    // If we are in software trigger mode, the call to startAcq serves as the trigger :
    sendCommand(andor3::SoftwareTrigger);
  }
  
  DEB_TRACE() << "Resuming the action of the acquisition thread";
  AutoMutex    the_lock(m_cond.mutex());
  m_acq_thread_waiting = false;
  m_cond.broadcast();
  
  DEB_TRACE() << "Done, the acquisition is now started and the frame retrieving should take place in parallel in a second thread";

}

void
lima::Andor3::Camera::stopAcq()
{
  DEB_MEMBER_FUNCT();
  if ( ! m_real_camera ) {
    DEB_ALWAYS() << "BE VERY CAREFULL : The andor SDK3 camera that you are connected to is a SIMULATED CAMERA !!!";
  }
  _stopAcq(false);
}

// -- detector info object
void
lima::Andor3::Camera::getImageType(ImageType& type)
{
  DEB_MEMBER_FUNCT();
  A3_BitDepth			the_bit_depth;
  getBitDepth(the_bit_depth);
  
  switch (the_bit_depth) {
    case b11:
      type = Bpp12;
      break;
      
    case b16:
      type = Bpp16;
      break;

    default:
      type = Bpp16;
      THROW_HW_ERROR(Error) << "Unknown image type for the SDK " << type
      << "\n\tReturning 16b since all output is made on this pixel depth";
      break;
  }
}

void
lima::Andor3::Camera::setImageType(ImageType type)
{
  DEB_MEMBER_FUNCT();
  A3_BitDepth			the_bit_depth;
  switch (type) {
    case Bpp8:
    case Bpp8S:
    case Bpp10:
    case Bpp10S:
    case Bpp12:
    case Bpp12S:
      the_bit_depth = b11;
      break;
    case Bpp14:
    case Bpp14S:
    case Bpp16:
    case Bpp16S:
      the_bit_depth = b16;
      break;
    case Bpp32:
    case Bpp32S:
      the_bit_depth = b16;
      THROW_HW_ERROR(Error) << "Unsupported type " << type
      << ", setting to the maximum : " << the_bit_depth << ".";
    default:
      the_bit_depth = b16;
      THROW_HW_ERROR(Error) << "Unknown type " << type
      << ", setting to the maximum : " << the_bit_depth << ".";
      break;
  }
  setBitDepth(the_bit_depth);
}

void
lima::Andor3::Camera::getDetectorType(std::string& type)
{
  DEB_MEMBER_FUNCT();
  type = m_detector_type;
}

void
lima::Andor3::Camera::getDetectorModel(std::string& model)
{
  DEB_MEMBER_FUNCT();
  model = m_detector_model;
}

/*!
 @brief get the max image size of the detector (the chip)
 */
void
lima::Andor3::Camera::getDetectorImageSize(Size& size)
{
  DEB_MEMBER_FUNCT();
    size = m_detector_size;
}

// -- Buffer control object
lima::HwBufferCtrlObj*
lima::Andor3::Camera::getBufferCtrlObj()
{
  DEB_MEMBER_FUNCT();
  return m_buffer_ctrl_obj;
}

//-- Synch control object

void
lima::Andor3::Camera::initTrigMode()
{
  DEB_MEMBER_FUNCT();
  AT_BOOL	b_value;
  int		enum_count;
  AT_WC		s_value[1024];
  std::string   s_mode;

  AT_GetEnumCount(m_camera_handle, andor3::TriggerMode, &enum_count);

  for (int idx=0; enum_count != idx; ++idx) {
    AT_GetEnumStringByIndex(m_camera_handle, andor3::TriggerMode, idx, s_value, 1024);
    AT_IsEnumIndexImplemented(m_camera_handle, andor3::TriggerMode, idx, &b_value);
    if (b_value) {
      s_mode = WStringToString(s_value);
      if ( s_mode == "Internal" ) {
        m_trig_mode_map.emplace(IntTrig, idx);
        DEB_TRACE() << "Found IntTrig mode" ;
      } else if ( s_mode == "Software" ) {
        m_trig_mode_map.emplace(IntTrigMult, idx);
        DEB_TRACE() << "Found IntTrigMult mode" ;
      } else if ( s_mode == "External" ) {
        m_trig_mode_map.emplace(ExtTrigMult, idx);
        DEB_TRACE() << "Found ExtTrigMult mode" ;
      } else if ( s_mode == "External Start" ) {
        m_trig_mode_map.emplace(ExtTrigSingle, idx);
        DEB_TRACE() << "Found ExtTrigSingle mode" ;
      } else if ( s_mode == "External Exposure" ) {
        m_trig_mode_map.emplace(ExtGate, idx);
        DEB_TRACE() << "Found ExtGate mode" ;
      }
    }
  }
}
     
bool
lima::Andor3::Camera::checkTrigMode(TrigMode mode)
{
  std::map<TrigMode,int>::iterator it;

  it = m_trig_mode_map.find(mode);
  if (it != m_trig_mode_map.end())
    return true;
  else
    return false;
}

void
lima::Andor3::Camera::setTrigMode(TrigMode mode)
{
  DEB_MEMBER_FUNCT();
  std::map<TrigMode,int>::iterator it;
  int i_setmode, i_getmode;

  if (m_real_camera) {
    it = m_trig_mode_map.find(mode);
    if (it == m_trig_mode_map.end())
        THROW_HW_ERROR(Error) << "The triggering mode " << mode
        << " is NOT implemented in this SDK";

    i_setmode = it->second;
    setEnumIndex(andor3::TriggerMode, i_setmode);

    getEnumIndex(andor3::TriggerMode, &i_getmode);
    if ( i_setmode != i_getmode ) {
        DEB_ERROR() << "Proof-reading the trigger mode :"
        << "\n\tGot " << i_getmode << ", while requesting " << i_setmode;
        for (it=m_trig_mode_map.begin(); it!=m_trig_mode_map.end(); ++it) {
          if (i_getmode == it->second) {
            m_trig_mode = it->first;
          }
        }
    } else {
        m_trig_mode = mode;
    }
  }
  else {
    setEnumString(andor3::TriggerMode, L"Advanced");
    DEB_TRACE() << "SIMCAM - forcing trigger mode to Advanced";
    m_trig_mode = mode;
  }
  DEB_TRACE() << "Trigger Mode is now : " << m_trig_mode;
}
  
void
lima::Andor3::Camera::getTrigMode(TrigMode& mode)
{
  DEB_MEMBER_FUNCT();
  mode = m_trig_mode;
}

void
lima::Andor3::Camera::setExpTime(double  exp_time)
{
  DEB_MEMBER_FUNCT();

  setFloat(andor3::ExposureTime, exp_time);
}

void
lima::Andor3::Camera::getExpTime(double& exp_time)
{
  DEB_MEMBER_FUNCT();
  double		the_exp_time;
  getFloat(andor3::ExposureTime, &the_exp_time);
  exp_time = the_exp_time;
}


void
lima::Andor3::Camera::getExposureTimeRange(double& min_expo, double& max_expo) const
{
  DEB_MEMBER_FUNCT();
  double			the_min, the_max;
  
  getFloatMin(andor3::ExposureTime, &the_min);
  getFloatMax(andor3::ExposureTime, &the_max);
  
  min_expo = the_min;
  max_expo = the_max;
}


void
lima::Andor3::Camera::setNbFrames(int nb_frames)
{
  DEB_MEMBER_FUNCT();
  DEB_PARAM() << DEB_VAR1(nb_frames);
  m_nb_frames_to_collect = static_cast<size_t>(nb_frames);
  DEB_TRACE() << "Setting the number of frames to collect to " << m_nb_frames_to_collect;
}

void
lima::Andor3::Camera::getNbFrames(int& nb_frames)
{
  DEB_MEMBER_FUNCT();
  nb_frames = static_cast<int>(m_nb_frames_to_collect);
  DEB_RETURN() << DEB_VAR1(nb_frames);
}

void
lima::Andor3::Camera::getNbHwAcquiredFrames(int &nb_acq_frames)
{
  DEB_MEMBER_FUNCT();
  nb_acq_frames = getNbHwAcquiredFrames();
}

void
lima::Andor3::Camera::checkRoi(const Roi& set_roi, Roi& hw_roi)
{
  DEB_MEMBER_FUNCT();
  DEB_PARAM() << DEB_VAR1(set_roi);
  bool 			the_fullaoi_control;
  Bin			the_binning;
  
  // For andor3 SDK : the height and width are in super-pixels (eg. width is the number of data point by line at acquisition)
  // Conversely the top and left are set in physical pixels
  
  // For LIMA : the binning is set BEFORE the ROI, hence part of the roi might be in super-pixel as units
  // Currently, from the documentaiton and source code I will consider that both top/left and size are un super-pixels.
  
  getBin(the_binning);
  if ( m_real_camera ) {
    getBool(andor3::FullAOIControl, &the_fullaoi_control);
  }
  else {
    the_fullaoi_control = false;
  }
 
  DEB_TRACE() << "Is Full AOI control available? " << the_fullaoi_control;
 
  // From now on, we are performing all tests in physical pixels (and we will convert back to super pixel just before returning).
  int				the_bin_nb = the_binning.getX();
  Roi				the_phys_set_roi;
  Roi				the_phys_test_roi;
  Roi				the_phys_hw_roi;
  
  the_phys_set_roi = set_roi.getUnbinned(Bin(the_bin_nb, the_bin_nb));
  Point phy_ori = the_phys_set_roi.getTopLeft();
  phy_ori += 1;
  the_phys_set_roi = Roi(phy_ori,the_phys_set_roi.getSize());
  the_phys_test_roi = Roi(Point(1, 1), m_detector_size);
  
  DEB_TRACE() << "Requested ROI is : " << set_roi << ", corresponding to " << the_phys_set_roi << " in term of physical pixel";
  // First : check that we are smaller than the maximum AOI for the current binning
  if ( ! the_phys_test_roi.containsRoi(the_phys_set_roi) ) {
    the_phys_set_roi = the_phys_test_roi;
  }

  DEB_TRACE() << "After testing if this is included in the max area of the detector, got " << the_phys_set_roi << " in term of physical pixel";
  if (!the_fullaoi_control ) {
    
    DEB_TRACE() << "Testing a roi while the camera has only limited support for ROI (FullAOIControl==false)";
    DEB_TRACE() << "We will now scan the possible camera ROIs, selecting the smallest one that include the requested one (lower number of lines)";
    
    // If there is no FullAOIControl, then we should resort to one of the proposition of § 3.5 of the SDK manual (page 42) :
    struct andor3_roi {
      int width;
      int height;
      int top;
      int left_12b;
      int left_16b;
      int center_12b;
      int center_16b;
    };

    // The idea is to select the ROI that enables the fastest possible frame rate but without cropping the requested ROI
    // For the rest of the algorithm the available rois are ordered by decreasing number of lines (most important aspect in
    //  frame rate), then the decrease in the number width.
    struct andor3_roi		the_rois[] = {
      {2592, 2160,    1,    1,    1, 1297, 1297},
      {2544, 2160,    1,   17,   25, 1289, 1297},
      {2064, 2048,   57,  257,  265, 1289, 1297},
      {1776, 1760,  201,  401,  409, 1289, 1297},
      {1920, 1080,  537,  337,  337, 1297, 1297},
      {1392, 1040,  561,  593,  601, 1289, 1297},
      { 528,  512,  825, 1025, 1033, 1289, 1297},
      {2592,  304,  929,    1,    1, 1297, 1297},
      { 240,  256,  953, 1169, 1177, 1289, 1297},
      { 144,  128, 1017, 1217, 1225, 1289, 1297},
    //  NULL
    };
    
    // Trying to take the smallest enclosing possible ROI (depending on the bit-depth of the image) :
    struct andor3_roi		*the_here_roi;
    if ( b11 == m_bit_depth ) {
      the_here_roi = the_rois;
      while ( the_here_roi ) {
        the_phys_test_roi = Roi(Point(the_here_roi->left_12b, the_here_roi->top), Size(the_here_roi->width, the_here_roi->height));
        if ( the_phys_test_roi.containsRoi(the_phys_set_roi)) {
          the_phys_hw_roi = the_phys_test_roi;
        }
        else {
          break;
        }
        ++the_here_roi;
      }
    }
    else { // b16 == m_bit_depth
      the_here_roi = the_rois;
      while ( the_here_roi ) {
        the_phys_test_roi = Roi(Point(the_here_roi->left_16b, the_here_roi->top), Size(the_here_roi->width, the_here_roi->height));
        if ( the_phys_test_roi.containsRoi(the_phys_set_roi)) {
          the_phys_hw_roi = the_phys_test_roi;
        }
        else {
          break;
        }
        ++the_here_roi;
      }
    }
    
    DEB_TRACE() << "The selected hardware ROI is now : " << the_phys_hw_roi << " in term of physical pixel";

    // Lima Roi starts at <0,0> Andor3 starts at <1,1>, so -1 for topleft
    Point topLeft = the_phys_hw_roi.getTopLeft();
    topLeft+=1;
    the_phys_hw_roi.setTopLeft(topLeft);
    hw_roi = the_phys_hw_roi.getBinned(Bin(the_bin_nb, the_bin_nb));
  }
  else { // the full AOI control accepted    
    the_phys_hw_roi = the_phys_set_roi;
    // Now need to apply the Bin AND Roi in order to get the stride width and return a correct hw Roi.

      
    setRoi(set_roi);

    hw_roi = set_roi;

    // Pb with Odd height Zyla does not support, set nearest even height size (+1)
    // then Lima will apply a good SoftRoi (-1)
    if (hw_roi.getSize().getHeight() & 1) {
      DEB_TRACE() << "Odd number of raw ("<< hw_roi.getSize().getHeight() << ") not supported, increase the size by +1";
      Size hw_size = hw_roi.getSize();

      hw_roi.setSize(Size(hw_size.getWidth(),hw_size.getHeight()+1));
    }    
  }  
  
  DEB_TRACE() << "hw_roi = " << hw_roi;
}

void
lima::Andor3::Camera::setRoi(const Roi& set_roi)
{
  DEB_MEMBER_FUNCT();
  Bin		the_binning;
  AT_64		the_left, the_width, the_top, the_height;
  AT_64		the_bin_nb;
  Roi           hw_roi, max_roi;
  Size          max_size;

  DEB_TRACE() << "set_roi = " << set_roi;
  getBin(the_binning);
  the_bin_nb = the_binning.getX();

  // full size Roi with Binning of course
  max_roi = Roi(0,0,m_detector_size.getWidth(), m_detector_size.getHeight()).getBinned(the_binning);

  if (set_roi.isActive() && set_roi !=max_roi) {
    // --- a real roi requested
    hw_roi = set_roi;
    
  } else {
    // --- either no roi or max size
    hw_roi = max_roi;
  }

  the_left = static_cast<AT_64>(hw_roi.getTopLeft().x) * the_bin_nb;
  the_width = static_cast<AT_64>(hw_roi.getSize().getWidth());
  the_top = static_cast<AT_64>(hw_roi.getTopLeft().y) * the_bin_nb;
  the_height = static_cast<AT_64>(hw_roi.getSize().getHeight());
  // Pb with Odd height Zyla does not support, set nearest even height size (+1)
  if (the_height & 1) the_height++;

  // Performing the settings in the order prescribed by the SDK's documentation:
  // Binning, width, left, heigh, top :  
  setBin(the_binning);
  // Lima Roi starts at <0,0> Andor3 starts at <1,1>
  setInt(andor3::AOIWidth, the_width);
  setInt(andor3::AOILeft, the_left+1);
  setInt(andor3::AOIHeight, the_height);
  setInt(andor3::AOITop, the_top+1);
}

void
lima::Andor3::Camera::getRoi(Roi& hw_roi)
{
  DEB_MEMBER_FUNCT();
  
  AT_64		the_left, the_width, the_top, the_height;
  Bin the_binning;
  getBin(the_binning);

  getInt(andor3::AOIWidth, &the_width);
  getInt(andor3::AOILeft, &the_left);
  the_left -=1;
  the_left/=the_binning.getX();
  getInt(andor3::AOIHeight, &the_height);
  getInt(andor3::AOITop, &the_top);
  the_top -=1;
  the_top /= the_binning.getY();

  // Lima Roi starts at <0,0> Andor3 starts at <1,1>
  Roi the_roi;
  the_roi.setTopLeft(Point(static_cast<int>(the_left), static_cast<int>(the_top)));
  the_roi.setSize(Size(static_cast<int>(the_width), static_cast<int>(the_height)));
		     

  hw_roi = the_roi;
  DEB_TRACE() << "hw_roi = " << hw_roi;
}

bool
lima::Andor3::Camera::isBinningAvailable()
{
  DEB_MEMBER_FUNCT();
  bool 			the_fullaoi_control;
  
  if ( m_real_camera ) {
    getBool(andor3::FullAOIControl, &the_fullaoi_control);
  }
  else {
    the_fullaoi_control = false;
  }
  
  return the_fullaoi_control;
}

void
lima::Andor3::Camera::checkBin(Bin& ioBin)
{
  DEB_MEMBER_FUNCT();
  int			the_bin_x, the_bin_y;
  
  the_bin_x = ioBin.getX();
  the_bin_y = ioBin.getY();
  
  if ( the_bin_x != the_bin_y) {
    if ( the_bin_x < the_bin_y)
      the_bin_y = the_bin_x;
    else
      the_bin_x = the_bin_y;
    THROW_HW_ERROR(NotSupported) << "For andor SDK v3 the binning should be a square power of two, ie. 1x1, 2x2, 4x4 or 8x8, while you tried to set " << the_bin_x << "x" << the_bin_y;
  }
  
  if ( 1 == the_bin_x ) {
    ioBin = Bin(1, 1);
    return;
  }
  if ( 2 == the_bin_x ) {
    ioBin = Bin(2, 2);
    return;
  }
  if ( 4 > the_bin_x ) {
    ioBin = Bin(2, 2);
    THROW_HW_ERROR(NotSupported) << "You have asked for a " << the_bin_x << "x" << the_bin_y << " binning which is not available."
    << "\n\tProviding you with the next smaller binning available : 2x2";
    return;
  }
  if ( 4 == the_bin_x ) {
    ioBin = Bin(4, 4);
    return;
  }
  if ( 8 > the_bin_x ) {
    ioBin = Bin(4, 4);
    THROW_HW_ERROR(NotSupported) << "You have asked for a " << the_bin_x << "x" << the_bin_y << " binning which is not available."
    << "\n\tProviding you with the next smaller binning available : 4x4";
    return;
  }
  if ( 8 == the_bin_x ) {
    ioBin = Bin(8, 8);
    return;
  }
  ioBin = Bin(8, 8);
  THROW_HW_ERROR(NotSupported) << "You have asked for a " << the_bin_x << "x" << the_bin_y << " binning which is not available (too big)."
  << "\n\tProviding you with the next smaller binning available : 8x8";
  return;
}

void
lima::Andor3::Camera::setBin(const Bin& iBin)
{
  DEB_MEMBER_FUNCT();
  
  if ( m_real_camera ) {
    A3_Binning		the_bin;
    switch (iBin.getX()) {
      case 1:
        the_bin = B1x1;
        break;
      case 2:
        the_bin = B2x2;
        break;
      case 4:
        the_bin = B4x4;
        break;
      case 8:
        the_bin = B8x8;
        break;
        
      default:
        the_bin = B1x1;
        break;
    }
    setEnumIndex(andor3::AOIBinning, static_cast<int>(the_bin));
  }
  else { // We are using a SIMCAM : setting through AOIHBin and AOIVBin :
    AT_64			the_x_bin, the_y_bin;
    the_x_bin = static_cast<AT_64>(iBin.getX());
    the_y_bin = static_cast<AT_64>(iBin.getY());
    setInt(andor3::AOIHBin, the_x_bin);
    setInt(andor3::AOIVBin, the_y_bin);
  }
}

void
lima::Andor3::Camera::getBin(Bin& oBin)
{
  DEB_MEMBER_FUNCT();
  if ( m_real_camera ) {
    A3_Binning		the_bin;
    int						the_bin_int;
    getEnumIndex(andor3::AOIBinning, &the_bin_int);
    the_bin = static_cast<A3_Binning>(the_bin_int);
    switch (the_bin) {
      case B1x1:
        oBin = Bin(1, 1);
        break;
      case B2x2:
        oBin = Bin(2, 2);
        break;
      case B4x4:
        oBin = Bin(4, 4);
        break;
      case B8x8:
        oBin = Bin(8, 8);
        break;
        
      default:
        oBin = Bin(1, 1);
        THROW_HW_ERROR(NotSupported) << "Unknown binning returned by andor SDK3" << the_bin_int;
        break;
    }
  }
  else { // Taking care of the SIMCAM case :
    AT_64			the_x_bin, the_y_bin;
    getInt(andor3::AOIHBin, &the_x_bin);
    getInt(andor3::AOIVBin, &the_y_bin);
    oBin = Size(static_cast<int>(the_x_bin), static_cast<int>(the_y_bin));
  }
}

void
lima::Andor3::Camera::setShutterMode(ShutterMode mode)
{
  DEB_MEMBER_FUNCT();
  DEB_PARAM() << DEB_VAR1(mode);
}

void
lima::Andor3::Camera::getShutterMode(ShutterMode& mode)
{
  DEB_MEMBER_FUNCT();
  mode = ShutterManual;
}

void
lima::Andor3::Camera::getPixelSize(double& sizex, double& sizey)
{
  DEB_MEMBER_FUNCT();
  getFloat(andor3::PixelWidth, &sizex);
  getFloat(andor3::PixelHeight, &sizey);
}

void
lima::Andor3::Camera::getStatus(Camera::Status& status)
{
  DEB_MEMBER_FUNCT();
  AutoMutex aLock(m_cond.mutex());
  status = m_status;
  //Check if the camera is not waiting for soft. trigger
  if (status == Camera::Readout && 
      m_trig_mode == IntTrigMult)
    {
      status = Camera::Ready;
    }
  DEB_RETURN() << DEB_VAR1(DEB_HEX(status));
}

// --- Acquisition interface
void
lima::Andor3::Camera::reset()
{
  DEB_MEMBER_FUNCT();
  _stopAcq(false); // We wait for the current frame buffer retrieval to be finished.
  initialiseController();
}

int
lima::Andor3::Camera::getNbHwAcquiredFrames()
{
  DEB_MEMBER_FUNCT();
  return static_cast<int>(m_image_index);
}

// -- andor3 specific, LIMA don't worry about it !
void
lima::Andor3::Camera::initialiseController()
{
  DEB_MEMBER_FUNCT();
  A3_BitDepth		the_bd = m_bit_depth;
  A3_SimpleGain 	the_simple_gain = m_simple_gain;
  A3_ReadOutRate	the_rate = m_adc_rate;
  
  // Carefully crafting the order, since some are affecting others...
  setElectronicShutterMode(m_electronic_shutter_mode);
  setTrigMode(m_trig_mode);
  setSimpleGain(the_simple_gain);
  setAdcRate(the_rate);
  setBitDepth(the_bd);
  setCooler(m_cooler);
  initTemperature();
  setExpTime(m_exp_time);
  
  setSpuriousNoiseFilter(false);
  
  AT_64			the_chip_width, the_chip_height;
  getInt(andor3::SensorWidth, &the_chip_width);
  getInt(andor3::SensorHeight, &the_chip_height);
  
  m_detector_size = Size(static_cast<int>(the_chip_width), static_cast<int>(the_chip_height));

  // Setting the ROI to the max :
  Roi aRoi = Roi(0, 0, m_detector_size.getWidth(), m_detector_size.getHeight());
  DEB_TRACE() << "Set the ROI to full frame: "<< aRoi;
  setRoi(aRoi);

  if ( m_real_camera ) {
    // Making sure the «spurious noise filter» is OFF (maybe later use the higher level setSpuriousNoiseFilter call rather than this low level one) :
    int  the_err_code = setBool(andor3::SpuriousNoiseFilter, false);
    if ( AT_SUCCESS != the_err_code ) {
      DEB_WARNING() << "Cannot set SpuriousNoiseFilter to false" << DEB_VAR1(error_code(the_err_code));
      if ( AT_ERR_NOTIMPLEMENTED == the_err_code ) {
	DEB_WARNING() << "It seems that the feature 'SpuriousNoiseFilter' is not implemented on this hardware, we will not throw an exception this time.";
      }
      else {
	DEB_ERROR() << "Indeed the 'SpuriousNoiseFilter' is implemented, so we are throwing an error since we can not choose the state of this filter.";
	THROW_HW_ERROR(Error) << "Cannot set SpuriousNoiseFilter to false";
      }
    }
  }
  else {
    DEB_TRACE() << "Since the camera is SIMULATED, it is not possible to set it noise filter (" << "SpuriousNoiseFilter" << ") to OFF.";
  }

  if ( NULL == m_acq_thread ) {
    m_acq_thread = new _AcqThread(*this);
    m_acq_thread->start();
  }
  
  //  AT_RegisterFeatureCallback(m_camera_handle, andor3::BufferOverflowEvent, &lima::Andor3::Camera::bufferOverflowCallback, static_cast<void*>(this));
}

void
lima::Andor3::Camera::setAdcGain(A3_Gain iGain)
{
  DEB_MEMBER_FUNCT();
  if ( m_real_camera ) {
    int the_gain;
    // This can automatically update the PixelEncoding and BitDepth to values corresponding to the gain
    // Hence we proof-read the new values and update the cache accordingly :
    int	the_bit_depth; // Pixel encoding will be reset appropriately in the setBitDepth method
    getHwBitDepth(&the_bit_depth);
    setEnumIndex(andor3::PreAmpGainControl, iGain);
    getEnumIndex(andor3::PreAmpGainControl, &the_gain);
    m_adc_gain = static_cast<A3_Gain>(the_gain);
    if ( m_adc_gain != iGain ) {
      DEB_ERROR() << "Proof-reading the ADC readout gain :"
      << "\n\tGot " << m_adc_gain << " back,"
      << "\n\twhile requesting " << iGain;
    }
    setBitDepth(static_cast<A3_BitDepth>(the_bit_depth));
  }
  else {
    DEB_TRACE() << "Setting the gain is not possible for the SIMCAM. Skipping this request (value requested was : " << iGain << ").";
  }
}

void
lima::Andor3::Camera::getAdcGain(A3_Gain &oGain) const
{
  DEB_MEMBER_FUNCT();
//  int the_gain;
//  getEnumIndex(andor3::PreAmpGainControl, &the_gain);
//  oGain = static_cast<A3_Gain>(the_gain);
  oGain = m_adc_gain;
}

void
lima::Andor3::Camera::getAdcGainString(std::string &oGainString) const
{
  AT_WC		the_string[256];
  getEnumString(andor3::PreAmpGainControl, the_string, 255);
  oGainString = WStringToString(std::wstring(the_string));
}


void
lima::Andor3::Camera::setAdcRate(A3_ReadOutRate iRate)
{
  DEB_MEMBER_FUNCT();
  if ( m_real_camera ) {
    int the_rate;
    setEnumIndex(andor3::PixelReadoutRate, iRate);
    getEnumIndex(andor3::PixelReadoutRate, &the_rate);
    m_adc_rate = static_cast<A3_ReadOutRate>(the_rate);
    if ( m_adc_rate != iRate ) {
      DEB_ERROR() << "Proof-reading the ADC readout rate :"
      << "\n\tGot " << m_adc_rate << " back,"
      << "\n\twhile requesting " << iRate;
    }
  }
  else {
    int the_rate;
    setEnumIndex(andor3::PixelReadoutRate, 0);
    getEnumIndex(andor3::PixelReadoutRate, &the_rate);
    m_adc_rate = static_cast<A3_ReadOutRate>(the_rate);
    DEB_TRACE() << "The SIMCAM has only one rate setting (550MHz), making sure that's what we are doing now";
  }
}

void
lima::Andor3::Camera::getAdcRate(A3_ReadOutRate &oRate) const
{
  DEB_MEMBER_FUNCT();
//  int the_rate;
//  getEnumIndex(andor3::PixelReadoutRate, &the_rate);
//  oRate = static_cast<A3_ReadOutRate>(the_rate);
  oRate = m_adc_rate;
}

void
lima::Andor3::Camera::getAdcRateString(std::string &oRateString) const
{
  AT_WC		the_string[256];
  getEnumString(andor3::PixelReadoutRate, the_string, 255);
  oRateString = WStringToString(std::wstring(the_string));
}

void
lima::Andor3::Camera::setElectronicShutterMode(A3_ShutterMode iMode)
{
  DEB_MEMBER_FUNCT();
  int the_mode;
  setEnumIndex(andor3::ElectronicShutteringMode, iMode);
  getEnumIndex(andor3::ElectronicShutteringMode, &the_mode);
  m_electronic_shutter_mode = static_cast<A3_ShutterMode>(the_mode);
  if ( m_electronic_shutter_mode != iMode ) {
    DEB_ERROR() << "Proof-reading the electronic shutter mode :"
    << "\n\tGot " << m_electronic_shutter_mode << " back,"
    << "\n\twhile requesting " << iMode;
  }
  // Setting the trigger mode might change the ADCGain and ADCRate :
  int		the_gain, the_rate;
  
  getEnumIndex(andor3::SimplePreAmpGainControl, &the_gain);
  getEnumIndex(andor3::PixelReadoutRate, &the_rate);
  setSimpleGain(static_cast<A3_SimpleGain>(the_gain));
  setAdcRate(static_cast<A3_ReadOutRate>(the_rate));
}

void
lima::Andor3::Camera::getElectronicShutterMode(A3_ShutterMode &oMode) const
{
  DEB_MEMBER_FUNCT();
//  int the_mode;
//  getEnumIndex(andor3::ElectronicShutteringMode, &the_mode);
//  oMode = static_cast<A3_ShutterMode>(the_mode);
  oMode = m_electronic_shutter_mode;
}

void
lima::Andor3::Camera::getElectronicShutterModeString(std::string &oModeString) const
{
  AT_WC		the_string[256];
  getEnumStringByIndex(andor3::ElectronicShutteringMode, m_electronic_shutter_mode, the_string, 255);
  oModeString = WStringToString(std::wstring(the_string));
}

void
lima::Andor3::Camera::setBitDepth(A3_BitDepth iMode)
{
  DEB_MEMBER_FUNCT();
  int the_mode;
  setEnumIndex(andor3::BitDepth, static_cast<int>(iMode));
  getHwBitDepth(&the_mode);
  m_bit_depth = static_cast<A3_BitDepth>(the_mode);
  if ( m_bit_depth != iMode ) {
    DEB_ERROR() << "Proof-reading the image bit-depth :"
    << "\n\tGot " << m_bit_depth << " back,"
    << "\n\twhile requesting " << iMode;
  }
  
  // Making sure that the pixel encoding is a predictable one, depending on the bit-depth.
  switch (m_bit_depth) {
    case b11:
      setEnumString(andor3::PixelEncoding, L"Mono12");
      break;
    case b16:
      setEnumString(andor3::PixelEncoding, L"Mono16");
      break;
    default:
      break;
  }
}

void
lima::Andor3::Camera::getBitDepth(A3_BitDepth &oMode) const
{
  DEB_MEMBER_FUNCT();
  oMode = m_bit_depth;
}

void
lima::Andor3::Camera::getBitDepthString(std::string &oDepthString) const
{
  AT_WC		the_string[256];
  getEnumStringByIndex(andor3::BitDepth, m_bit_depth, the_string, 255);
  oDepthString = WStringToString(std::wstring(the_string));
}

void
lima::Andor3::Camera::getPxEncodingString(std::string &oPxEncodingString) const
{
  AT_WC		the_string[256];
  getEnumString(andor3::PixelEncoding, the_string, 255);
  oPxEncodingString = WStringToString(std::wstring(the_string));
}

void
lima::Andor3::Camera::getPxEncoding(A3_PixelEncoding &oPxEncoding) const
{
  DEB_MEMBER_FUNCT();
  int the_mode;
  getEnumIndex(andor3::PixelEncoding, &the_mode);
  oPxEncoding = static_cast<A3_PixelEncoding>(oPxEncoding);
}

void
lima::Andor3::Camera::getTriggerModeString(std::string &oModeString) const
{
  AT_WC		the_string[256];
  int		i_trig;
  if (m_real_camera) {
    i_trig = m_trig_mode_map.at(m_trig_mode);
    getEnumStringByIndex(andor3::TriggerMode, m_trig_mode, the_string, 255);
    oModeString = WStringToString(std::wstring(the_string));
  }
  else {
    oModeString = "Advanced";
  }
}

void
lima::Andor3::Camera::setGateLevel(A3_SignalLevel iLevel)
{
  DEB_MEMBER_FUNCT();
  bool flag;
  if (iLevel == Inverted) flag= true;
  else flag= false;
  setEnumString(andor3::IOSelector, L"External Exposure");
  setBool(andor3::IOInvert, flag);
}

void
lima::Andor3::Camera::getGateLevel(A3_SignalLevel &iLevel)
{
  DEB_MEMBER_FUNCT();
  bool flag;
  setEnumString(andor3::IOSelector, L"External Exposure");
  getBool(andor3::IOInvert, &flag);
  if (flag)
    iLevel = Inverted;
  else
    iLevel = Normal;
}

void
lima::Andor3::Camera::setTriggerLevel(A3_SignalLevel iLevel)
{
  DEB_MEMBER_FUNCT();
  bool flag;
  if (iLevel == Inverted) flag= true;
  else flag= false;
  setEnumString(andor3::IOSelector, L"External Trigger");
  setBool(andor3::IOInvert, flag);
}

void
lima::Andor3::Camera::getTriggerLevel(A3_SignalLevel &iLevel)
{
  DEB_MEMBER_FUNCT();
  bool flag;
  setEnumString(andor3::IOSelector, L"External Trigger");
  getBool(andor3::IOInvert, &flag);
  if (flag)
    iLevel = Inverted;
  else
    iLevel = Normal;
}

void
lima::Andor3::Camera::setOutputSignal(A3_OutputSignal iSignal)
{
  DEB_MEMBER_FUNCT();
  if ( propImplemented(andor3::AuxiliaryOutSource) ) {
    setEnumIndex(andor3::AuxiliaryOutSource, iSignal);
  }
  else {
    DEB_TRACE() << "The camera has no fan speed setting... Do nothing !";
  }
}

void
lima::Andor3::Camera::getOutputSignal(A3_OutputSignal &oSignal) const
{
  DEB_MEMBER_FUNCT();
  if ( propImplemented(andor3::AuxiliaryOutSource) ) {
    int  value;
    getEnumIndex(andor3::AuxiliaryOutSource, &value);
    oSignal = static_cast<A3_OutputSignal>(value);
  }
  else {
    DEB_TRACE() << "The camera has no fan speed setting... Do nothing !";
  }  
}

//-----------------------------------------------------
// @brief	retrieve temperature capabilities
//-----------------------------------------------------
void
lima::Andor3::Camera::initTemperature()
{
  DEB_MEMBER_FUNCT();
  if ( propImplemented(andor3::TargetSensorTemperature) && m_detector_model.find("ZYLA5.5")==std::string::npos ) {
    m_has_temperature_sp = true;
    getFloat(andor3::TargetSensorTemperature, &m_temperature_sp);
  } 
  else if ( propImplemented(andor3::TemperatureControl) ) {
    int ntemp, current_idx;
    AT_WC at_string[256];
    float value;

    m_has_temperature_control = true;
    DEB_ALWAYS() << "Camera has temperature setpoints pre-defined";
    getEnumCount(m_camera_handle, andor3::TemperatureControl, &ntemp);
    for (int idx=0; idx != ntemp; ++idx) {
      AT_GetEnumStringByIndex(m_camera_handle, andor3::TemperatureControl, idx, at_string, 255);
      value = std::stof(WStringToString(at_string));
      m_temperature_control_values.push_back(value);
      DEB_ALWAYS() << "Index " << idx << " Setpoint " << value;
    }
    getEnumIndex(andor3::TemperatureControl, &current_idx);
    m_temperature_sp = m_temperature_control_values[current_idx];
  }
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

  if (m_has_temperature_sp) {
    setFloat(andor3::TargetSensorTemperature, temp);
    getFloat(andor3::TargetSensorTemperature, &m_temperature_sp);
    if ( abs(m_temperature_sp - temp) > 0.1) {
      DEB_ERROR() << "Proof-reading temperature set-point : "
		  << "\n\tproof-read = " << m_temperature_sp
		  << "\n\twhile asked to be = " << temp;
      THROW_HW_ERROR(Error) << "Failed on setting temperature set-point";
    }
  }
  else if (m_has_temperature_control) {
    float temp_idx = 0;
    int size = m_temperature_control_values.size();
    int current_idx;
    for (int idx=size-1; idx>= 0; --idx) {
      if (temp <= m_temperature_control_values[idx]) {
         temp_idx = idx;
         break;
      }
    }
    setEnumIndex(andor3::TemperatureControl, temp_idx);
    getEnumIndex(andor3::TemperatureControl, &current_idx);
    m_temperature_sp = m_temperature_control_values[current_idx];
  }
  else {
    DEB_ERROR() << "Camera has no temperature set-point control";
    THROW_HW_ERROR(Error) << "Camera has no temperature set-point control";
  }
}

//-----------------------------------------------------
// @brief	return the temperature set-point // DONE (trusting the cached value)
// @param	temperature in centigrade
//
//-----------------------------------------------------
void
lima::Andor3::Camera::getTemperatureSP(double& temp) const
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
lima::Andor3::Camera::getTemperature(double& temp) const
{
  DEB_MEMBER_FUNCT();
  getFloat(andor3::SensorTemperature, &temp);
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
  setBool(andor3::SensorCooling, flag);
  getBool(andor3::SensorCooling, &m_cooler);
  if ( flag != m_cooler ) {
    DEB_ERROR() << "Failed to properly set the cooler : requested " << flag << ", but got " << m_cooler;
    THROW_HW_ERROR(Error) << "Failed to properly set the cooler";
  }
}

//-----------------------------------------------------
// @brief	Get the Cooler status     // DONE (trusting the cached value)
// @param	flag true-on, false-off
//
//-----------------------------------------------------
void
lima::Andor3::Camera::getCooler(bool& flag) const
{
  DEB_MEMBER_FUNCT();
  flag = m_cooler;
}

//-----------------------------------------------------
// @brief	Gets cooling/temperature status
// @param	status : status as a string
//
//-----------------------------------------------------
void
lima::Andor3::Camera::getCoolingStatus(std::string& status) const
{
  DEB_MEMBER_FUNCT();
  if ( m_real_camera ) {
    wchar_t		wcs_status_string[32];
    getEnumString(andor3::TemperatureStatus, wcs_status_string, 31);
    status = WStringToString(wcs_status_string);
  }
  else {
    DEB_TRACE() << "This has no signification on SIMCAM";
  }
}

void
lima::Andor3::Camera::setBufferOverflow(bool i_overflow)
{
  DEB_MEMBER_FUNCT();
  setBool(andor3::BufferOverflowEvent, i_overflow);
  bool		the_bool;
  getBool(andor3::BufferOverflowEvent, &i_overflow);
  if ( i_overflow != the_bool ) {
    DEB_ERROR() << "Failed to properly set the BufferOverflow mode : requested " << i_overflow << ", but got " << the_bool;
    THROW_HW_ERROR(Error) << "Failed to properly set the BufferOverflow mode";
  }
}

void
lima::Andor3::Camera::getBufferOverflow(bool &o_overflow) const
{
  DEB_MEMBER_FUNCT();
  getBool(andor3::BufferOverflowEvent, &o_overflow);
}


void
lima::Andor3::Camera::getFanSpeedList(std::vector<std::string> &fan_speed_list) const
{
  int		enum_count;
  AT_WC		s_value[1024];
  
  AT_GetEnumCount(m_camera_handle, andor3::FanSpeed, &enum_count);

  for (int idx=0; enum_count != idx; ++idx) {
    AT_GetEnumStringByIndex(m_camera_handle, andor3::FanSpeed, idx, s_value, 1024);
    fan_speed_list.push_back(WStringToString(s_value));
  }
}

void
lima::Andor3::Camera::setFanSpeed(std::string fan_speed)
{
  DEB_MEMBER_FUNCT();
  if ( propImplemented(andor3::FanSpeed) ) {
    const std::wstring fan_set = StringToWString(fan_speed);
    setEnumString(andor3::FanSpeed, fan_set.c_str());
  }
  else {
    DEB_TRACE() << "The camera has no fan speed setting... Do nothing !";
  }
}

void
lima::Andor3::Camera::getFanSpeed(std::string &fan_speed) const
{
  DEB_MEMBER_FUNCT();
  if ( propImplemented(andor3::FanSpeed) ) {
    AT_WC fan_string[256];
    getEnumString(andor3::FanSpeed, fan_string, 255);
    fan_speed = WStringToString(fan_string);
  }
  else {
    DEB_TRACE() << "The camera has no fan speed setting... Do nothing !";
    fan_speed = "not implemented";
  }  
}

void
lima::Andor3::Camera::setOverlap(bool i_overlap)
{
  DEB_MEMBER_FUNCT();
  setBool(andor3::Overlap, i_overlap);
  bool		the_bool;
  getBool(andor3::Overlap, &the_bool);
  if ( i_overlap != the_bool ) {
    DEB_ERROR() << "Failed to properly set the overlap mode : requested " << i_overlap << ", but got " << the_bool;
    THROW_HW_ERROR(Error) << "Failed to properly set the overlap mode";
  }
}

void
lima::Andor3::Camera::getOverlap(bool &o_overlap) const
{
  DEB_MEMBER_FUNCT();
  getBool(andor3::Overlap, &o_overlap);
}

void
lima::Andor3::Camera::setSimpleGain(A3_SimpleGain i_gain)
{
  DEB_MEMBER_FUNCT();
  if ( propImplemented(andor3::SimplePreAmpGainControl) ) {
    int the_err_code = setEnumIndex(andor3::SimplePreAmpGainControl, i_gain);
    if ( AT_SUCCESS != the_err_code ) {
      DEB_ERROR() << "Cannot set SimplePreAmpGainControl to " << i_gain
                  << " - " << DEB_VAR1(error_code(the_err_code));
    }
  }
  else {
    DEB_TRACE() << "SimplePreAmpGainControl not implemented, emulating it in software";
    A3_ShutterMode		the_shutter;
    
    getElectronicShutterMode(the_shutter);
    
    switch (i_gain) {
      case b11_low_gain:
        setAdcGain(Gain1);
        break;
        
      case b11_hi_gain:
        if ( Rolling == the_shutter )
          setAdcGain(Gain4);
        else
          setAdcGain(Gain3);
        break;
        
      case b16_lh_gain:
        if ( Rolling == the_shutter )
          setAdcGain(Gain1_Gain4);
        else
          setAdcGain(Gain1_Gain3);
        break;
        
      default:
        DEB_TRACE() << "Not performing any settings since you provided an unavailable value";
        break;
    }
  }
}

void
lima::Andor3::Camera::getSimpleGain(A3_SimpleGain &o_gain) const
{
  DEB_MEMBER_FUNCT();

  //  (0) "Gain 1 (11 bit)"; implemented = true; available = true.
  //  (1) "Gain 2 (11 bit)"; implemented = true; available = true.
  //  (2) "Gain 3 (11 bit)"; implemented = true; available = true.
  //  (3) "Gain 4 (11 bit)"; implemented = true; available = true.
  //  (4) "Gain 1 Gain 3 (16 bit)"; implemented = true; available = true.
  //  (5) "Gain 1 Gain 4 (16 bit)"; implemented = true; available = true.
  //  (6) "Gain 2 Gain 3 (16 bit)"; implemented = true; available = true.
  //  (7) "Gain 2 Gain 4 (16 bit)"; implemented = true; available = true.
  if ( propImplemented(andor3::SimplePreAmpGainControl) ) {
    int			the_gain;
    getEnumIndex(andor3::SimplePreAmpGainControl, &the_gain);
    o_gain = static_cast<A3_SimpleGain>(the_gain);
  }
  else {
    DEB_TRACE() << "SimplePreAmpGainControl not implemented, emulating it in software";

    A3_Gain					the_gain;
    A3_ShutterMode	the_shutter;
    
    getAdcGain(the_gain);
    getElectronicShutterMode(the_shutter);

    if ( Rolling == the_shutter ) {
      switch (the_gain) {
        case Gain1:
          o_gain = b11_low_gain;
          break;
        case Gain4:
          o_gain = b11_hi_gain;
          break;
        case Gain1_Gain4:
          o_gain = b16_lh_gain;
          break;
          
        default:
          o_gain = none;
          break;
      }
    }
    else {
      switch (the_gain) {
        case Gain1:
          o_gain = b11_low_gain;
          break;
        case Gain3:
          o_gain = b11_hi_gain;
          break;
        case Gain1_Gain3:
          o_gain = b16_lh_gain;
          break;
          
        default:
          o_gain = none;
          break;
      }      
    }
  }
}


void
lima::Andor3::Camera::getSimpleGainString(std::string &o_gainString) const
{
  DEB_MEMBER_FUNCT();
  if ( propImplemented(andor3::SimplePreAmpGainControl) ) {
    AT_WC		the_string[256];
    getEnumString(andor3::SimplePreAmpGainControl, the_string, 255);
    o_gainString = WStringToString(std::wstring(the_string));
  }
  else {
    DEB_TRACE() << "The camera has no simple gain control setting... Do nothing !";
    o_gainString = "not implemented";
  }
}

void
lima::Andor3::Camera::setSpuriousNoiseFilter(bool i_filter)
{
  DEB_MEMBER_FUNCT();
  setBool(andor3::SpuriousNoiseFilter, i_filter);
  bool		the_bool;
  getBool(andor3::SpuriousNoiseFilter, &the_bool);
  if ( i_filter != the_bool ) {
    DEB_ERROR() << "Failed to properly set the spurious noise filter mode : requested " << i_filter << ", but got " << the_bool;
    // THROW_HW_ERROR(Error) << "Failed to properly set the spurious noise filter mode";
  }
}

void
lima::Andor3::Camera::getSpuriousNoiseFilter(bool &o_filter) const
{
  DEB_MEMBER_FUNCT();
  getBool(andor3::SpuriousNoiseFilter, &o_filter);
}

void
lima::Andor3::Camera::setSyncTriggering(bool i_sync)
{
  DEB_MEMBER_FUNCT();
  setBool(andor3::SynchronousTriggering, i_sync);
  bool		the_bool;
  getBool(andor3::SynchronousTriggering, &the_bool);
  if ( i_sync != the_bool ) {
    DEB_ERROR() << "Failed to properly set the SynchronousTriggering mode : requested " << i_sync << ", but got " << the_bool;
    // THROW_HW_ERROR(Error) << "Failed to properly set the SynchronousTriggering mode";
  }
}

void
lima::Andor3::Camera::getSyncTriggering(bool &o_sync) const
{
  DEB_MEMBER_FUNCT();
  getBool(andor3::SynchronousTriggering, &o_sync);
}

void lima::Andor3::Camera::getBytesPerPixel(double &o_value) const
{
  DEB_MEMBER_FUNCT();
  double		the_Bpp;
  getFloat(andor3::BytesPerPixel, &the_Bpp);
  o_value = the_Bpp;
  
}

void
lima::Andor3::Camera::getFirmwareVersion(std::string &o_fwv) const
{
  DEB_MEMBER_FUNCT();
  AT_WC	the_fwv[1024];
  getString(andor3::FirmwareVersion, the_fwv, 1024);
  o_fwv = WStringToString(the_fwv);
}


void
lima::Andor3::Camera::setFrameRate(double i_frame_rate)
{
  DEB_MEMBER_FUNCT();
  setFloat(andor3::FrameRate, i_frame_rate);
}

void
lima::Andor3::Camera::getFrameRate(double &o_frame_rate) const
{
  DEB_MEMBER_FUNCT();
  double			the_fr;
  getFloat(andor3::FrameRate, &the_fr);
  o_frame_rate = the_fr;
}

void
lima::Andor3::Camera::getFrameRateRange(double& o_min_fr, double& o_max_fr) const
{
  DEB_MEMBER_FUNCT();
  double			the_min, the_max;
  
  getFloatMin(andor3::FrameRate, &the_min);
  getFloatMax(andor3::FrameRate, &the_max);
  
  o_min_fr = the_min;
  o_max_fr = the_max;
}

void
lima::Andor3::Camera::getFullRoiControl(bool &o_fullROIcontrol) const
{
  DEB_MEMBER_FUNCT();
  getBool(andor3::FullAOIControl, &o_fullROIcontrol);
}

void
lima::Andor3::Camera::getImageSize(int &o_frame_size) const
{
  DEB_MEMBER_FUNCT();
  AT_64 		the_size;
  getInt(andor3::ImageSizeBytes, &the_size);
  o_frame_size = static_cast<int>(the_size);
}

void
lima::Andor3::Camera::getMaxFrameRateTransfer(double &o_max_transfer_rate) const
{
  DEB_MEMBER_FUNCT();
  double		the_max_fr;
  if ( propImplemented(andor3::MaxInterfaceTransferRate) ) {
    getFloat(andor3::MaxInterfaceTransferRate, &the_max_fr);
  }
  else {
    DEB_TRACE() << "Requested the maximum sustainable frame rate, but the feature is not existent on this camera. Doing a software estimation. Currently considering the Neo only (no easy way to know the camera attached)";
    AT_64			the_frame_size;
    double		the_interface_bandwith = 250.0e6;
    getInt(andor3::ImageSizeBytes, &the_frame_size);
    the_max_fr = the_interface_bandwith / static_cast<double>(the_frame_size);
  }
  o_max_transfer_rate = the_max_fr;
}

void
lima::Andor3::Camera::getReadoutTime(double &o_time) const
{
  DEB_MEMBER_FUNCT();
  double		the_time;
  getFloat(andor3::ReadoutTime, &the_time);
  o_time = static_cast<float>(the_time);
}

void
lima::Andor3::Camera::getSerialNumber(std::string &o_sn) const
{
  DEB_MEMBER_FUNCT();
  AT_WC	the_sn[1024];
  getString(andor3::SerialNumber, the_sn, 1024);
  o_sn = WStringToString(the_sn);
}



/*!
 @brief asking the full acquisition to stop
 @param iImmadiate : if true, the stop should be immediate, without waiting for inflight buffer to be retrieved by LIMA.
 */

void
lima::Andor3::Camera::_stopAcq(bool iImmediate)
{
  DEB_MEMBER_FUNCT();
  
  if ( ! m_real_camera ) {
    DEB_ALWAYS() << "BE VERY CAREFULL : The andor SDK3 camera that you are connected to is a SIMULATED CAMERA !!!";
  }

  AutoMutex		the_lock(m_cond.mutex());
  bool				the_camera_acq;
  
  getBool(andor3::CameraAcquiring, &the_camera_acq);
  
  if ( iImmediate ) {
    DEB_TRACE() << "Sending a FORCEFULL AcquistionStop, whatever is the current status be cause you asked for immediate stop";
    sendCommand(andor3::AcquisitionStop);
    DEB_TRACE() << "Sent an AcquisitionStop in stopAcq immediate == TRUE";
    // Deblocking the acquisition thread (in case it is waiting)
    m_acq_thread_waiting = true; // Asking to stop as soon as not in charge
    // This is dangerous: causes the lock inside the lock ?
    //    _setStatus(Ready, false);
    m_status = Ready;
    // Finally broadcasting :
    m_cond.broadcast();
    DEB_TRACE() << "Returning from the FORCEFULL stopAcq";
    return;
  }

  DEB_TRACE() << "Requested a stop, but waiting the opportunity in the acquisition thread";
  //  if ( the_camera_acq || (Ready != m_status) ) {
  if ( ! m_acq_thread_waiting ) {
    while ( (! iImmediate) && (m_acq_thread_running) ) { // We are still actively retrieving frame buffers
      //  the_lock.unlock();
      //  the_lock.lock();
      m_acq_thread_waiting = true; // Asking to stop as soon as not in charge
      m_cond.broadcast();
      DEB_TRACE() << "Setting ourselves to wait a signal";
      m_cond.wait();
      DEB_TRACE() << "We just received a signal that it is usefull to check again if the acquisition thread is in a stoppable state";
    }
    DEB_TRACE() << "We finally got the signal that the currently exposed frame is now grabbed... We can go on for the acquisition stop";
    the_lock.unlock();

    DEB_TRACE() << "We should now STOP the acquisition";
    sendCommand(andor3::AcquisitionStop);
    DEB_TRACE() << "Sent a AcquisitionStop in stopAcq immediate == FALSE." << "\n\t\tAnd now flushing the buffer queue since no more acquisition is in flight";
    AT_Flush(m_camera_handle);
    DEB_TRACE() << "Finally setting the status to Ready";
    _setStatus(Ready, false);
  }
  else {
    DEB_TRACE() << "One more stopAcq, while the acquisition is either allready stopped or about to be stopped... Skip the current one";
  }
  return;
}


/*!
 @brief Setting the status of the camera, in a thread-safe manner.
 
 If the current status is Fault and the force is 
*/
void
lima::Andor3::Camera::_setStatus(Status iStatus, bool iForce)
{
  DEB_MEMBER_FUNCT();
  DEB_TRACE() << "in _setStatus, about to ask for the lock to mutex";
  AutoMutex aLock(m_cond.mutex());
  DEB_TRACE() << "grabbed the lock";
  if( iForce || (Camera::Fault != m_status) )
    m_status = iStatus;
  m_cond.broadcast();
  DEB_TRACE() << "_setStatus broadcasting and releasing soon the mutex lock.";
}

//-----------------------------------------------------
// @brief just error codes to string
//-----------------------------------------------------
static const char* error_code(int error_code)
{
  const char *error;
  switch(error_code)
    {
  case AT_SUCCESS:			error = "'AT_SUCCESS' : Function call has been successful";							 break;
  case AT_ERR_NOTINITIALISED:		error = "'AT_ERR_NOTINITIALISED' : Function called with an uninitialised handle";				 break;
  case AT_ERR_NOTIMPLEMENTED:		error = "'AT_ERR_NOTIMPLEMENTED' : Feature has not been implemented for the chosen camera";			 break;
  case AT_ERR_READONLY:			error = "'AT_ERR_READONLY' : Feature is read only";								 break;
  case AT_ERR_NOTREADABLE:		error = "'AT_ERR_NOTREADABLE' : Feature is currently not readable";						 break;
  case AT_ERR_NOTWRITABLE:		error = "'AT_ERR_NOTWRITABLE' : Feature is currently not writable";						 break;
  case AT_ERR_OUTOFRANGE:		error = "'AT_ERR_OUTOFRANGE' : Value is outside the maximum and minimum limits";				 break;
  case AT_ERR_INDEXNOTAVAILABLE:	error = "'AT_ERR_INDEXNOTAVAILABLE' : Index is currently not available";					 break;
  case AT_ERR_INDEXNOTIMPLEMENTED:	error = "'AT_ERR_INDEXNOTIMPLEMENTED' : Index is not implemented for the chosen camera";			 break;
  case AT_ERR_EXCEEDEDMAXSTRINGLENGTH:	error = "'AT_ERR_EXCEEDEDMAXSTRINGLENGTH' : String value provided exceeds the maximum allowed length";		 break;
  case AT_ERR_CONNECTION:		error = "'AT_ERR_CONNECTION' : Error connecting to or disconnecting from hardware";				 break;
  case AT_ERR_NODATA:			error = "AT_ERR_NODATA";											 break;
  case AT_ERR_INVALIDHANDLE:		error = "AT_ERR_INVALIDHANDLE";											 break;
  case AT_ERR_TIMEDOUT:			error = "'AT_ERR_TIMEDOUT' : The AT_WaitBuffer function timed out while waiting for data arrive in output queue";break;
  case AT_ERR_BUFFERFULL:		error = "'AT_ERR_BUFFERFULL' : The input queue has reached its capacity";					 break;
  case AT_ERR_INVALIDSIZE:		error = "'AT_ERR_INVALIDSIZE' : The size of a queued buffer did not match the frame size";			 break;
  case AT_ERR_INVALIDALIGNMENT:		error = "'AT_ERR_INVALIDALIGNMENT' : A queued buffer was not aligned on an 8-byte boundary";			 break;
  case AT_ERR_COMM:			error = "'AT_ERR_COMM' : An error has occurred while communicating with hardware";				 break;
  case AT_ERR_STRINGNOTAVAILABLE:	error = "'AT_ERR_STRINGNOTAVAILABLE' : Index / String is not available";					 break;
  case AT_ERR_STRINGNOTIMPLEMENTED:	error = "'AT_ERR_STRINGNOTIMPLEMENTED' : Index / String is not implemented for the chosen camera";		 break;
  case AT_ERR_NULL_FEATURE:		error = "AT_ERR_NULL_FEATURE";											 break;
  case AT_ERR_NULL_HANDLE:		error = "AT_ERR_NULL_HANDLE";											 break;
  case AT_ERR_NULL_IMPLEMENTED_VAR:	error = "AT_ERR_NULL_IMPLEMENTED_VAR";										 break;
  case AT_ERR_NULL_READABLE_VAR:	error = "AT_ERR_NULL_READABLE_VAR";										 break;
  case AT_ERR_NULL_READONLY_VAR:	error = "AT_ERR_NULL_READONLY_VAR";										 break;
  case AT_ERR_NULL_WRITABLE_VAR:	error = "AT_ERR_NULL_WRITABLE_VAR";										 break;
  case AT_ERR_NULL_MINVALUE:		error = "AT_ERR_NULL_MINVALUE";											 break;
  case AT_ERR_NULL_MAXVALUE:		error = "AT_ERR_NULL_MAXVALUE";											 break;
  case AT_ERR_NULL_VALUE:		error = "AT_ERR_NULL_VALUE";											 break;
  case AT_ERR_NULL_STRING:		error = "AT_ERR_NULL_STRING";											 break;
  case AT_ERR_NULL_COUNT_VAR:		error = "AT_ERR_NULL_COUNT_VAR";										 break;
  case AT_ERR_NULL_ISAVAILABLE_VAR:	error = "AT_ERR_NULL_ISAVAILABLE_VAR";										 break;
  case AT_ERR_NULL_MAXSTRINGLENGTH:	error = "AT_ERR_NULL_MAXSTRINGLENGTH";										 break;
  case AT_ERR_NULL_EVCALLBACK:		error = "AT_ERR_NULL_EVCALLBACK";										 break;
  case AT_ERR_NULL_QUEUE_PTR:		error = "AT_ERR_NULL_QUEUE_PTR";										 break;
  case AT_ERR_NULL_WAIT_PTR:		error = "AT_ERR_NULL_WAIT_PTR";											 break;
  case AT_ERR_NULL_PTRSIZE:		error = "AT_ERR_NULL_PTRSIZE";											 break;
  case AT_ERR_NOMEMORY:			error = "AT_ERR_NOMEMORY";											 break;
  case AT_ERR_DEVICEINUSE:		error = "AT_ERR_DEVICEINUSE";											 break;
  case AT_ERR_HARDWARE_OVERFLOW:	error = "AT_ERR_HARDWARE_OVERFLOW";										 break;
    default:
      error = "Unknown";break;
    }
  return error;
}

int
lima::Andor3::Camera::printInfoForProp(const AT_WC * iPropName, A3_TypeInfo iPropType) const
{
  DEB_MEMBER_FUNCT();
  
  int i_err = 0;
  
  AT_BOOL b_exists;
  AT_BOOL b_readonly;
  AT_BOOL b_writable;
  AT_BOOL b_readable;
  
  std::cout << "\nRetrieving information on property named \"" << WStringToString(iPropName) << "\".\n";
  
  int ret_code;
  // Implemented
  if ( AT_SUCCESS != (ret_code = AT_IsImplemented(m_camera_handle, iPropName, &b_exists)) ) {
    std::cout << "Error in printInfoForProp : " << error_code(ret_code);
    return i_err;
  }
  std::cout << "\tIsImplemented = " << atBoolToString(b_exists);
  
  if ( ! b_exists ) {
    std::cout << "No more information to query, since the feature does not \"exists\" for this camera/driver/SDK.";
    return i_err;
  }
  
  // ReadOnly
  AT_IsReadOnly(m_camera_handle, iPropName, &b_readonly);
  std::cout << "\tIsReadOnly = " << atBoolToString(b_readonly);
  
  // Writable
  AT_IsWritable(m_camera_handle, iPropName, &b_writable);
  std::cout << "\tIsWritable = " << atBoolToString(b_writable);
  
  // Readable
  AT_IsReadable(m_camera_handle, iPropName, &b_readable);
  std::cout << "\tIsReadable = " << atBoolToString(b_readable);
  
  if ( ! b_readable ) {
    std::cout << "Since the property is not readable at this time, we will stop here.";
    return i_err;
  }
  
  // Now getting the value itself : we absolutely need now to know the type of the feature :
  if ( Camera::Unknown == iPropType ) {
    std::cout << "Could not retrieve information on a property of unknown type !!\n"
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
      
      if ( AT_SUCCESS == (ret_code = AT_GetInt(m_camera_handle, iPropName, &i_Value)) )
        DEB_TRACE() << "\tValue = " << i_Value;
      else
        DEB_TRACE() << "\tError message : " << error_code(ret_code);
      
      if ( AT_SUCCESS == (ret_code = AT_GetIntMax(m_camera_handle, iPropName, &i_Value)) )
        DEB_TRACE() << "\tMax value = " << i_Value;
      else
        DEB_TRACE() << "\tError message : " << error_code(ret_code);
      
      if ( AT_SUCCESS == (ret_code = AT_GetIntMin(m_camera_handle, iPropName, &i_Value)) )
        DEB_TRACE() << "\tMin value = " << i_Value;
      else
        DEB_TRACE() << "\tError message : " << error_code(ret_code);
      break;
      
    case Camera::Float:
      DEB_TRACE() << "\tFeature of type FLOAT";
      if ( AT_SUCCESS == (ret_code = AT_GetFloat(m_camera_handle, iPropName, &d_Value)) )
        DEB_TRACE() << "\tValue = " << d_Value;
      else
        DEB_TRACE() << "\tError message : " << error_code(ret_code);
      if ( AT_SUCCESS == (ret_code = AT_GetFloatMax(m_camera_handle, iPropName, &d_Value)) )
        DEB_TRACE() << "\tMax value = " << d_Value;
      else
        DEB_TRACE() << "\tError message : " << error_code(ret_code);
      if ( AT_SUCCESS == (ret_code = AT_GetFloatMin(m_camera_handle, iPropName, &d_Value)) )
        DEB_TRACE() << "\tMin value = " << d_Value;
      else
        DEB_TRACE() << "\tError message : " << error_code(ret_code);
      break;
      
    case Camera::Bool:
      DEB_TRACE() << "\tFeature of type BOOLEAN";
      if ( AT_SUCCESS == (ret_code = AT_GetBool(m_camera_handle, iPropName, &b_Value)) )
        DEB_TRACE() << "\tValue = " << atBoolToString(b_Value);
      else
        DEB_TRACE() << "\tError message : " << error_code(ret_code);
      break;
      
    case Camera::Enum:
      std::cout << "\n\tFeature of type ENUM";
      if ( AT_SUCCESS == (ret_code = AT_GetEnumIndex(m_camera_handle, iPropName, &enum_Value)) ) {
        std::cout << "\n\tValue = (" << enum_Value << ")";
        if ( AT_SUCCESS == (ret_code = AT_GetEnumStringByIndex(m_camera_handle, iPropName, enum_Value, s_Value, 1024)) )
          std::cout << " \"" << WStringToString(s_Value) << "\"";
        else
          std::cout << "\tError message : " << error_code(ret_code);
        if ( AT_SUCCESS == (ret_code = AT_IsEnumIndexImplemented(m_camera_handle, iPropName, enum_Value, &b_Value)) )
          std::cout << "; implemented = " << atBoolToString(b_Value);
        else
          std::cout << "\tError message : " << error_code(ret_code);
        if ( AT_SUCCESS == (ret_code = AT_IsEnumIndexAvailable(m_camera_handle, iPropName, enum_Value, &b_Value)) )
          std::cout << "; available = " << atBoolToString(b_Value);
        else
          std::cout << "\tError message : " << error_code(ret_code);
        std::cout << ".";
      }
      else
        std::cout << "\tError message : " << error_code(ret_code);
      
      if ( AT_SUCCESS == (ret_code = AT_GetEnumCount(m_camera_handle, iPropName, &enum_Count)) ) {
        std::cout << "\n\tAvailable choices are (" << enum_Count << ") :";
        for ( int i=0; enum_Count != i; ++i ) {
          std::cout << "\n\t\t(" << i << ")";
          if ( AT_SUCCESS == (ret_code = AT_GetEnumStringByIndex(m_camera_handle, iPropName, i, s_Value, 1024)) )
            std::cout << " \"" << WStringToString(s_Value) << "\"";
          else
            std::cout << "\tError message : " << error_code(ret_code);
          if ( AT_SUCCESS == (ret_code = AT_IsEnumIndexImplemented(m_camera_handle, iPropName, i, &b_Value)) )
            std::cout << "; implemented = " << atBoolToString(b_Value);
          else
            std::cout << "\tError message : " << error_code(ret_code);
          if ( AT_SUCCESS == (ret_code = AT_IsEnumIndexAvailable(m_camera_handle, iPropName, i, &b_Value)) )
            std::cout << "; available = " << atBoolToString(b_Value);
          else
            std::cout << "\tError message : " << error_code(ret_code);
          std::cout << ".";
        }
      }
      else
        std::cout << "\tError message : " << error_code(ret_code);
      
      break;
      
    case Camera::String :
      DEB_TRACE() << "\tFeature of type STRING";
      if ( AT_SUCCESS == (ret_code = AT_GetString(m_camera_handle, iPropName, s_Value, 1024)) )
        DEB_TRACE() << "\tValue = \"" <<  WStringToString(s_Value) << "\"";
      else
        DEB_TRACE() << "\tError message : " << error_code(ret_code);
      
      if ( AT_SUCCESS == (ret_code = AT_GetStringMaxLength(m_camera_handle, iPropName, &s_MaxLen)) ) {
        DEB_TRACE() << "\tMaximum length of this property's string = " << s_MaxLen << ".";
      }
      else
        DEB_TRACE() << "\tError message : " << error_code(ret_code);
      break;
      
    default:
      DEB_TRACE() << "\tNot TREATED case so far !!!";
      break;
  }
  return i_err;
}

bool
lima::Andor3::Camera::propImplemented(const AT_WC * iPropName) const
{
  DEB_MEMBER_FUNCT();
  AT_BOOL		b_exists;

  int ret_code;
  if ( AT_SUCCESS != (ret_code = AT_IsImplemented(m_camera_handle, iPropName, &b_exists)) ) {
    DEB_TRACE() << "Error in printInfoForProp : " << error_code(ret_code);
    return false;
  }
  DEB_TRACE() << "\tIsImplemented = " << atBoolToString(b_exists);
  return b_exists;
}


int
lima::Andor3::Camera::getIntSystem(const AT_WC* Feature, AT_64* Value)
{
  DEB_STATIC_FUNCT();
  return (AT_SUCCESS != AT_GetInt(AT_HANDLE_SYSTEM, Feature, Value));
}



// int
// lima::Andor3::Camera::bufferOverflowCallback(AT_H i_handle, const AT_WC* i_feature, void* i_info)
// {
//   DEB_STATIC_FUNCT();
//   Camera		*the_camera;
  
//   the_camera = static_cast<Camera*>(i_info);
//   DEB_WARNING() << "The camera " << the_camera->m_detector_serial << " has lost (at least) a frame.\n" << "\n\t Will now try to STOP the acquisition (if not already done).";
//   the_camera->_setStatus(Fault, false);
//   return 1;
// }

int
lima::Andor3::Camera::setInt(const AT_WC* Feature, AT_64 Value)
{
  DEB_MEMBER_FUNCT();
  return AT_SetInt(m_camera_handle, Feature, Value);
}

int
lima::Andor3::Camera::getInt(const AT_WC* Feature, AT_64* Value) const
{
  DEB_MEMBER_FUNCT();
  return AT_GetInt(m_camera_handle, Feature, Value);
}

int
lima::Andor3::Camera::getIntMax(const AT_WC* Feature, AT_64* MaxValue) const
{
  DEB_MEMBER_FUNCT();
  return AT_GetIntMax(m_camera_handle, Feature, MaxValue);
}

int
lima::Andor3::Camera::getIntMin(const AT_WC* Feature, AT_64* MinValue) const
{
  DEB_MEMBER_FUNCT();
  return AT_GetIntMin(m_camera_handle, Feature, MinValue);
}

int
lima::Andor3::Camera::setFloat(const AT_WC* Feature, double Value)
{
  DEB_MEMBER_FUNCT();
  return AT_SetFloat(m_camera_handle, Feature, Value);
}

int
lima::Andor3::Camera::getFloat(const AT_WC* Feature, double* Value) const
{
  DEB_MEMBER_FUNCT();
  return AT_GetFloat(m_camera_handle, Feature, Value);
}

int
lima::Andor3::Camera::getFloatMax(const AT_WC* Feature, double* MaxValue) const
{
  DEB_MEMBER_FUNCT();
  return AT_GetFloatMax(m_camera_handle, Feature, MaxValue);
}

int
lima::Andor3::Camera::getFloatMin(const AT_WC* Feature, double* MinValue) const
{
  DEB_MEMBER_FUNCT();
  return AT_GetFloatMin(m_camera_handle, Feature, MinValue);
}

int
lima::Andor3::Camera::setBool(const AT_WC* Feature, bool Value)
{
  DEB_MEMBER_FUNCT();
  AT_BOOL newBool = Value;
  return AT_SetBool(m_camera_handle, Feature, newBool);
}

int
lima::Andor3::Camera::getBool(const AT_WC* Feature, bool* Value) const
{
  DEB_MEMBER_FUNCT();
  AT_BOOL	newBool;
  int i_Err = AT_GetBool(m_camera_handle, Feature, &newBool);
  *Value = newBool;
  return i_Err;
}

int
lima::Andor3::Camera::setEnumIndex(const AT_WC* Feature, int Value)
{
  DEB_MEMBER_FUNCT();
  return AT_SetEnumIndex(m_camera_handle, Feature, Value);
}

int
lima::Andor3::Camera::setEnumString(const AT_WC* Feature, const AT_WC* String)
{
  DEB_MEMBER_FUNCT();
  return AT_SetEnumString(m_camera_handle, Feature, String);
}

int
lima::Andor3::Camera::getEnumIndex(const AT_WC* Feature, int* Value) const
{
  DEB_MEMBER_FUNCT();
  return AT_GetEnumIndex(m_camera_handle, Feature, Value);
}

int
lima::Andor3::Camera::getEnumString(const AT_WC* Feature, AT_WC* String, int StringLength) const
{
  DEB_MEMBER_FUNCT();
  int Value;
  int i_Err = AT_GetEnumIndex(m_camera_handle, Feature, &Value);
  if ( AT_SUCCESS != i_Err )
    return i_Err;
  return AT_GetEnumStringByIndex(m_camera_handle, Feature, Value, String, StringLength);
}

int
lima::Andor3::Camera::getEnumCount(AT_H m_camera_handle,const  AT_WC* Feature, int* Count) const
{
  DEB_MEMBER_FUNCT();
  return AT_GetEnumCount(m_camera_handle, Feature, Count);
}

int
lima::Andor3::Camera::isEnumIndexAvailable(const AT_WC* Feature, int Index, bool* Available) const
{
  DEB_MEMBER_FUNCT();
  AT_BOOL  isAvailable;
  int i_Err = AT_IsEnumIndexAvailable(m_camera_handle, Feature, Index, &isAvailable);
  *Available = isAvailable;
  return i_Err;
}
int
lima::Andor3::Camera::isEnumIndexImplemented(const AT_WC* Feature, int Index, bool* Implemented) const
{
  DEB_MEMBER_FUNCT();
  AT_BOOL  isImplemented;
  int i_Err = AT_IsEnumIndexAvailable(m_camera_handle, Feature, Index, &isImplemented);
  *Implemented = isImplemented;
  return i_Err;
}

int
lima::Andor3::Camera::getEnumStringByIndex(const AT_WC* Feature, int Index, AT_WC* String, int StringLength) const
{
  DEB_MEMBER_FUNCT();
  return AT_GetEnumStringByIndex(m_camera_handle, Feature, Index, String, StringLength);
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
    DEB_ERROR() << "Failed to get Enum Count" << " : error code = " << error_code(i_Err);
    return i_Err;
  }
  for (i_enumIndex = 0; i_enumCount != i_enumIndex; ++i_enumIndex) {
    if ( AT_SUCCESS != getEnumStringByIndex(Feature, i_enumIndex, wcs_enumString, i_maxStringLen) ) {
      DEB_ERROR() << "Failed to get Enum String" << " : error code = " << error_code(i_Err);
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
  return AT_SetString(m_camera_handle, Feature, String);
}

int
lima::Andor3::Camera::getString(const AT_WC* Feature, AT_WC* String, int StringLength) const
{
  DEB_MEMBER_FUNCT();
  return AT_GetString(m_camera_handle, Feature, String, StringLength);
}

int
lima::Andor3::Camera::sendCommand(const AT_WC* Feature)
{
  DEB_MEMBER_FUNCT();
  return AT_Command(m_camera_handle, Feature);
}

int lima::Andor3::Camera::getHwBitDepth(int *bit_depth)
{
  DEB_MEMBER_FUNCT();
  int the_err_code = getEnumIndex(andor3::BitDepth, bit_depth);
  if ( (the_err_code != AT_ERR_NOTIMPLEMENTED) || m_real_camera )
    return the_err_code;

  DEB_WARNING() << "Get BitDepth not implemented for simulated camera: "
		<< "fixing b16";
  *bit_depth = b16;
  return AT_SUCCESS;
}

int lima::Andor3::Camera::getPixelStride() const
{
  DEB_MEMBER_FUNCT();

  AT_64  the_stride;

  getInt(andor3::AOIStride, &the_stride);
 
  return int(the_stride);
}

void lima::Andor3::Camera::getSdkFrameDim(SdkFrameDim &frame_dim, bool last)
{
  DEB_MEMBER_FUNCT();

  SdkFrameDim the_frame_dim;

  if (!last) 
    {
      // Retrieving useful information from the camera :
      
      DEB_TRACE() << "Getting the basic information from the camera : image size (in bytes) and pixel encoding.";
      getInt(andor3::ImageSizeBytes, &the_frame_dim.size);
      getEnumIndex(andor3::PixelEncoding, (int *)&the_frame_dim.encoding);
      getFloat(andor3::BytesPerPixel, &the_frame_dim.bytes_per_pixel);
      int the_bit_depth_index;
      getHwBitDepth(&the_bit_depth_index);  
      DEB_TRACE() << "The image size in bytes is " << the_frame_dim.size
		  << ", the pixel encoding index is " << the_frame_dim.encoding
		  << " and finally byte per pixel " << the_frame_dim.bytes_per_pixel;
      
     // flag for the reconstruction task on decoding the "Mono12Packed" format
      the_frame_dim.is_encoded = the_frame_dim.encoding == Mono12Packed;
      
      // Indeed it seems LIMA is not having the difference between width and stride.
      // So we have to compute the width provided to LIMA from the stride and the
      // byte per pixel settings.
      
      getInt(andor3::AOIWidth, &the_frame_dim.width);
      getInt(andor3::AOIHeight, &the_frame_dim.height);
      getInt(andor3::AOIStride, &the_frame_dim.stride);

      the_frame_dim.is_strided = (the_frame_dim.width * the_frame_dim.bytes_per_pixel) != the_frame_dim.stride;

      switch (the_frame_dim.encoding)
	{
	case Camera::Mono12Packed:
	  the_frame_dim.input_encoding =  L"Mono12Packed";
	  the_frame_dim.output_encoding =  L"Mono16";
	  break;
	case Camera::Mono12:
	  the_frame_dim.input_encoding =  L"Mono12";
	  the_frame_dim.output_encoding =  L"Mono16";
	  break;
	case Camera::Mono16:
	  the_frame_dim.input_encoding =  L"Mono16";
	  the_frame_dim.output_encoding =  L"Mono16";
	  break;
	case Camera::Mono32:
	  the_frame_dim.input_encoding =  L"Mono32";
	  the_frame_dim.output_encoding =  L"Mono32";
	  break;
	}
      m_sdk_frame_dim = the_frame_dim;
    }

  frame_dim = m_sdk_frame_dim;
}

lima::HwBufferCtrlObj* lima::Andor3::Camera::getTempBufferCtrlObj()
{
  DEB_MEMBER_FUNCT();
  return m_temp_buffer_ctrl_obj;
}

void lima::Andor3::Camera::setMaxImageSizeCallbackActive(bool cb_active)
{
    DEB_MEMBER_FUNCT();
    m_maximage_size_cb_active = cb_active;
}

//-----------------------------------------------------
// Taking care of the imptrlementation of the acquisition thread (_AcqThread / m_acq_thread)
//-----------------------------------------------------
lima::Andor3::Camera::_AcqThread::_AcqThread(Camera &aCam) :
m_cam(aCam)
{
  DEB_CONSTRUCTOR();
  // This seems to be useless since :
  // Linux supports PTHREAD_SCOPE_SYSTEM, but not PTHREAD_SCOPE_PROCESS.
  // (http://man7.org/linux/man-pages/man3/pthread_attr_setscope.3.html 2013-04-19) ?
  //  pthread_attr_setscope(&m_thread_attr, PTHREAD_SCOPE_PROCESS);
}


// Signaling to the m_acq_thread that it should quit, then waiting for it to do it.
lima::Andor3::Camera::_AcqThread::~_AcqThread()
{
  DEB_DESTRUCTOR();
  DEB_TRACE() << "Asking the acquisition thread to stop (setting m_cam.m_acq_thread_should_quit to true).";
  AutoMutex the_lock(m_cam.m_cond.mutex());
  m_cam.m_acq_thread_should_quit = true;
  // Signaling the acquisition thread to check if something should be done :
  m_cam.m_cond.broadcast();
  the_lock.unlock();
  
  DEB_TRACE() << "Waiting for the acquisition thread to be done (joining the main thread).";
  join();
}


// Either waiting to be signaled, or looping with the SDK to retrieve frame buffers
void
lima::Andor3::Camera::_AcqThread::threadFunction()
{
  DEB_MEMBER_FUNCT();
  AutoMutex			the_lock(m_cam.m_cond.mutex());
  StdBufferCbMgr		&the_buffer_mgr = m_cam.m_buffer_ctrl_obj->getBuffer();
  
  while (! m_cam.m_acq_thread_should_quit ) {
    DEB_TRACE() << "[andor3 acquisition thread] Top of the loop";
    // We are looping until being «signaled» that we shoul quit.
    while ( m_cam.m_acq_thread_waiting && ! m_cam.m_acq_thread_should_quit ) {
      // Lets wait a signal telling that maybe something has to be done …
      DEB_TRACE() << "[andor3 acquisition thread] Setting the m_acq_thread_running to false (since we are waiting)";
      m_cam.m_acq_thread_running = false; // Making sure the main class/thread knows nothing goes on
      m_cam.m_cond.broadcast();
      DEB_TRACE() << "[andor3 acquisition thread] Waiting acquisition start";
      m_cam.m_cond.wait();
    }
    // The main thread asked to get out of wait mode (setting m_cam.m_acq_thread_waiting to false)
    DEB_TRACE() << "[andor3 acquisition thread] Set running by main thread setting m_cam.m_acq_thread_waiting to false (or m_cam.m_acq_thread_should_quit to true)";
    m_cam.m_acq_thread_running = true;
    
    if ( m_cam.m_acq_thread_should_quit ) {
      // Should return ASAP, so that the thread could be «joined».
      DEB_TRACE() << "[andor3 acquisition thread] Quitting under request from main thread (m_acq_thread_should_quit)";
      return;
    }
    
    m_cam.m_status = Camera::Exposure;
    m_cam.m_cond.broadcast();
    the_lock.unlock();
    
    DEB_TRACE() << "[andor3 acquisition thread] About to start looping to get the images-frame retrieved";
    bool		the_acq_goon = true;
    while ( the_acq_goon && ((0 == m_cam.m_nb_frames_to_collect) || (m_cam.m_nb_frames_to_collect != m_cam.m_image_index)) ) {
      unsigned char  		*the_returned_image;
      int			the_returned_image_size;
      // Use active pooling on WaitBuffer()
      unsigned int		the_wait_timeout = 100;
      int                       the_wait_queue_res;
      

      the_wait_queue_res = AT_WaitBuffer(m_cam.m_camera_handle, &the_returned_image, &the_returned_image_size, the_wait_timeout);
      
      // Testing if we were asked to stop the acquisition thread :
      // It is best to do that as soon as returning from the SDK, since otherwise it might block some 
      // thread interacting with the main thread.
      the_lock.lock();
      the_acq_goon = !m_cam.m_acq_thread_waiting && !m_cam.m_acq_thread_should_quit;
      m_cam.m_acq_thread_running = the_acq_goon;
      m_cam.m_cond.broadcast();
      the_lock.unlock();

      if ( AT_SUCCESS == the_wait_queue_res ) {
        m_cam._setStatus(Readout, false);
        // We managed to get an image buffer returned :
        HwFrameInfoType		the_frame_info;
	bool                    the_frame_read;

        the_frame_info.acq_frame_nb = static_cast<int>(m_cam.m_image_index);
        the_frame_read = the_buffer_mgr.newFrameReady(the_frame_info);        
	DEB_TRACE() << "[andor3 acquisition thread] image " << m_cam.m_image_index <<" published with newFrameReady(), with result " << the_frame_read ;
	the_acq_goon = the_acq_goon && the_frame_read;
        
        ++m_cam.m_image_index;

        if ( m_cam.m_buffer_ringing ) {
          AT_QueueBuffer(m_cam.m_camera_handle, the_returned_image, the_returned_image_size);
        }
      }
      else if (AT_ERR_TIMEDOUT){
	// Active pooling on timeout, fine for Software trigger,i.e IntTrigMult
	continue;
      }
      else {
        DEB_ERROR() << "[andor3 acquisition thread] Problem in retrieving the frame indexed " << m_cam.m_image_index <<"!\n"
        << "\tAT_WaitBuffer returned an error " << the_wait_queue_res << "\n"
	<< "\t" << error_code(the_wait_queue_res) << "\n"
        << "\t!!! returning to WAIT mode !!!";
        m_cam.m_acq_thread_running = the_acq_goon = false;
        m_cam._setStatus(Fault, false);
	break;
      }
      
      if ( ! the_acq_goon ) {
        DEB_TRACE() << "[andor3 acquisition thread] In the middle of acquisition, got the request to stop the frame retrieving activity : m_acq_thread_waiting is " << m_cam.m_acq_thread_waiting << " and m_acq_thread_should_quit is " << m_cam.m_acq_thread_should_quit;
      }
    }
    DEB_TRACE() << "[andor3 acquisition thread] Finished looping !";
    m_cam.sendCommand(andor3::AcquisitionStop);
    DEB_TRACE() << "[andor3 acquisition thread] Sent stop to the andor SDK3";
    m_cam._setStatus(Ready, false);
    // Returning to the waiting mode :
    the_lock.lock();
    m_cam.m_acq_thread_waiting = true;
  }
}

/*
 Local Variables:
 c-file-style: "gnu"
 End:
 */
