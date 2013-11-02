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


// Defining the parameter names of the andor3 SDK :
namespace lima {
  namespace andor3 {
    static const AT_WC* AccumulateCount = L"AccumulateCount";
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
    static const AT_WC* BaselineLevel = L"BaselineLevel";
    static const AT_WC* BitDepth = L"BitDepth";
    static const AT_WC* BytesPerPixel = L"BytesPerPixel";
    static const AT_WC* CameraAcquiring = L"CameraAcquiring";
    static const AT_WC* CameraDump = L"CameraDump";
    static const AT_WC* CameraModel = L"CameraModel";
    static const AT_WC* ControllerID = L"ControllerID";
    static const AT_WC* CycleMode = L"CycleMode";
    static const AT_WC* DeviceCount = L"DeviceCount";

    static const AT_WC* ElectronicShutteringMode = L"ElectronicShutteringMode";
    static const AT_WC* ExposureTime = L"ExposureTime";
    static const AT_WC* FanSpeed = L"FanSpeed";
    static const AT_WC* FirmwareVersion = L"FirmwareVersion";
    static const AT_WC* FrameCount = L"FrameCount";
    static const AT_WC* FrameRate = L"FrameRate";
    static const AT_WC* FullAOIControl = L"FullAOIControl";
    static const AT_WC* ImageSizeBytes = L"ImageSizeBytes";
    static const AT_WC* LUTIndex = L"LUTIndex";
    static const AT_WC* LUTValue = L"LUTValue";
    static const AT_WC* MaxInterfaceTransferRate = L"MaxInterfaceTransferRate";
    
    static const AT_WC* MetadataEnable = L"MetadataEnable";
    static const AT_WC* MetadataFrame = L"MetadataFrame";
    static const AT_WC* MetadataTimestamp = L"MetadataTimestamp";
    static const AT_WC* Overlap = L"Overlap";
    static const AT_WC* PixelCorrection = L"PixelCorrection";
    static const AT_WC* PixelEncoding = L"PixelEncoding";
    static const AT_WC* PixelHeight = L"PixelHeight";
    static const AT_WC* PixelReadoutRate = L"PixelReadoutRate";
    static const AT_WC* PixelWidth = L"PixelWidth";
    //    static const AT_WC* PreAmpGain = L"PreAmpGain"; // Deprecated

    //    static const AT_WC* PreAmpGainChannel = L"PreAmpGainChannel"; // Deprecated
    static const AT_WC* PreAmpGainControl = L"PreAmpGainControl";
    //    static const AT_WC* PreAmpGainSelector = L"PreAmpGainSelector"; // Deprecated
    static const AT_WC* ReadoutTime = L"ReadoutTime";
    static const AT_WC* RowNExposureEndEvent = L"RowNExposureEndEvent";
    static const AT_WC* RowNExposureStartEvent = L"RowNExposureStartEvent";
    static const AT_WC* SensorCooling = L"SensorCooling";
    static const AT_WC* SensorHeight = L"SensorHeight";
    static const AT_WC* SensorTemperature = L"SensorTemperature";
    static const AT_WC* SensorWidth = L"SensorWidth";
    static const AT_WC* SerialNumber = L"SerialNumber";
    static const AT_WC* SimplePreAmpGainControl = L"SimplePreAmpGainControl";
    static const AT_WC* SoftwareTrigger = L"SoftwareTrigger";

    static const AT_WC* SoftwareVersion = L"SoftwareVersion";
    static const AT_WC* SpuriousNoiseFilter = L"SpuriousNoiseFilter";
    static const AT_WC* SynchronousTriggering = L"SynchronousTriggering";
    static const AT_WC* TargetSensorTemperature = L"TargetSensorTemperature";
    static const AT_WC* TemperatureControl = L"TemperatureControl";
    static const AT_WC* TemperatureStatus = L"TemperatureStatus";
    static const AT_WC* TimestampClock = L"TimestampClock";
    static const AT_WC* TimestampClockFrequency = L"TimestampClockFrequency";
    static const AT_WC* TimestampClockReset = L"TimestampClockReset";
    static const AT_WC* TriggerMode = L"TriggerMode";

  }
}

//---------------------------
//- utility variables
//---------------------------
bool lima::Andor3::Camera::sAndorSDK3Initted = false;

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
m_buffer_ctrl_obj(),
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
m_adc_rate(MHz100),
m_electronic_shutter_mode(Rolling),
m_bit_depth(b16),
m_trig_mode(Internal),
m_cooler(true),
m_temperature_sp(5.0)
{
  DEB_CONSTRUCTOR();
  // Initing the maps that serves for error string generation :
  _mapAndor3Error();
  
  // Initialisation of the atcore library :
  if ( ! sAndorSDK3Initted ) {
    if ( m_bitflow_path != "" ) {
      setenv("BITFLOW_INSTALL_DIRS", m_bitflow_path.c_str(), true);
    }
    else {
      setenv("BITFLOW_INSTALL_DIRS", "/usr/local/andor/bitflow", false);
    }
    
    if ( AT_SUCCESS != andor3Error(AT_InitialiseLibrary()) ) {
      DEB_ERROR() << "Library initialization failed, check the config. path" << " : error code = " << m_camera_error_str;
      THROW_HW_ERROR(Error) << "Library initialization failed, check the bitflow path";
    }
  }

  // --- Get available cameras and select the choosen one.
  AT_64 numCameras;
  DEB_TRACE() << "Get all attached cameras";
  if ( AT_SUCCESS != getIntSystem(andor3::DeviceCount, &numCameras) ) {
    DEB_ERROR() << "No camera present!";
    THROW_HW_ERROR(Error) << "No camera present!";
  }
  DEB_TRACE() << "Found "<< numCameras << " camera" << ((numCameras>1)? "s": "");
  DEB_TRACE() << "Try to set current camera to number " << m_camera_number;
  
  if ( m_camera_number < numCameras && m_camera_number >=0 ) {
    // Getting the m_camera_handle WITH some error checking :
    if(andor3Error(AT_Open(m_camera_number, &m_camera_handle))) {
      DEB_ERROR() << "Cannot get camera handle" << " : error code = " << m_camera_error_str;;
      THROW_HW_ERROR(InvalidValue) << "Cannot get camera handle";
    }
  }
  else {
    DEB_ERROR() << "Invalid camera number " << m_camera_number << ", there is "<< numCameras << " available";
    THROW_HW_ERROR(InvalidValue) << "Invalid Camera number ";
  }
  
  // --- Get Camera model (and all other parameters which are not changing during camera setup and usage )
  AT_WC	model[1024];
  if ( AT_SUCCESS != getString(andor3::CameraModel, model, 1024) ) {
    DEB_ERROR() << "Cannot get camera model: " << m_camera_error_str;
    THROW_HW_ERROR(Error) << "Cannot get camera model";
  }
  m_detector_model = WStringToString(std::wstring(model));
  
  if ( m_detector_model != "SIMCAM CMOS" ) {
    m_real_camera = true;
  }
  else {
    m_real_camera = false;
    DEB_TRACE() << "The camera is indeed the SIMULATED camera, all exception for invalid parameter name will be ignored!!!";
    DEB_ALWAYS() << "BE VERY CAREFULL : The andor SDK3 camera that you are connected to is a SIMULATED CAMERA !!!";
  }

  // --- Get Camera Type
  // @TODO This one would be better with a map converting from model to type !!!
  m_detector_type = std::string("sCMOS");
  
  // --- Get Camera Serial number
  AT_WC	serial[1024];
  if ( AT_SUCCESS != getString(andor3::SerialNumber, serial, 1024) ) {
    DEB_ERROR() << "Cannot get camera serial number: " << m_camera_error_str;
    THROW_HW_ERROR(Error) << "Cannot get camera serial number";
  }
  m_detector_serial = WStringToString(std::wstring(serial));

  // --- Get Camera maximum image size 
  AT_64 xmax, ymax;
  // --- Get the max image size of the detector
  if ( AT_SUCCESS != getInt(andor3::SensorWidth, &xmax)) {
    DEB_ERROR() << "Cannot get detector X size" << " : error code = " << m_camera_error_str;
    THROW_HW_ERROR(Error) << "Cannot get detector X size";
  }
  if ( AT_SUCCESS != getInt(andor3::SensorHeight, &ymax)) {
    DEB_ERROR() << "Cannot get detector Y size" << " : error code = " << m_camera_error_str;
    THROW_HW_ERROR(Error) << "Cannot get detector Y size";
  }
  m_detector_size= Size(static_cast<int>(xmax), static_cast<int>(ymax));
  
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
  if (m_cooler) {
    DEB_ERROR() <<"Please stop the cooling before shuting dowm the camera\n"
    << "brutale heating could damage the sensor.\n"
    << "And wait until temperature rises above 5 deg, before shuting down.";

    DEB_ALWAYS() << "The cooler of the camera is ON!!!\n"
    << "we are now waiting the camera to warm-up slowly, "
    << "setting the temperature to 10C and waiting for it "
    << "to rise above 5C before conitnuing the shutdown.";
    setTemperatureSP(6.0);
    double			the_sensor_temperature;
    size_t			the_sensor_temp_wait = 0;
    
    DEB_TRACE() << "While leaving the camera, the temperature provided by the cooler is " << the_sensor_temperature;
    getTemperature(the_sensor_temperature);
    while ( the_sensor_temperature < 5.1 ) {
      sleep(1);
      ++the_sensor_temp_wait;
      getTemperature(the_sensor_temperature);
      DEB_TRACE() << "The temperature provided by the cooler is " << the_sensor_temperature << "(waited approx. " << the_sensor_temp_wait << "s)";
    }
    setCooler(false);
    
    //    THROW_HW_ERROR(Error)<<"Please stop the cooling before shuting dowm the camera\n"
    //    << "brutale heating could damage the sensor.\n"
    //    << "And wait until temperature rises above 5 deg, before shuting down.";
  }
  
  DEB_TRACE() << "Shutdown camera";
  if ( AT_SUCCESS != andor3Error(AT_Close(m_camera_handle)) ) {
    DEB_ERROR() << "Cannot close the camera" << " : error code = " << m_camera_error << ", " <<m_camera_error_str;
    THROW_HW_ERROR(Error) << "Cannot close the camera";
  }
  m_camera_handle = AT_HANDLE_UNINITIALISED;

  if ( AT_SUCCESS != andor3Error(AT_FinaliseLibrary()) ) {
    DEB_ERROR() << "Cannot finalise Andor SDK 3 library" << " : error code = " << m_camera_error << ", " <<m_camera_error_str;
    THROW_HW_ERROR(Error) << "Cannot finalise Andor SDK 3 library";
  }

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
  
  // Retrieving useful information from the camera :
  AT_64			the_image_size;
  int				the_pixel_encoding;
  
  DEB_TRACE() << "Getting the basic information from the camera : image size (in bytes) and pixel encoding.";
  getInt(andor3::ImageSizeBytes, &the_image_size);
  getEnumIndex(andor3::PixelEncoding, &the_pixel_encoding);
  
  DEB_TRACE() << "The image size in bytes is " << the_image_size
  << ", and the pixel encoding index is " << the_pixel_encoding;
  
  // Since LIMA knows only an integer number of bytes per pixel
  // Make sure that the BytesPerPixel is 2 ...
  AT_WC								the_bit_depth_wc[256];
  std::string					the_bit_depth;
  int									the_bit_depth_index;
  
  getEnumString(andor3::BitDepth, the_bit_depth_wc, 255);
  the_bit_depth = WStringToString(std::wstring(the_bit_depth_wc));
  getEnumIndex(andor3::BitDepth, &the_bit_depth_index);
  DEB_TRACE() << "The bit depth of the image is " << the_bit_depth << " (index : " << the_bit_depth_index << ").";
  
  if ( b11 == the_bit_depth_index ) {
    DEB_TRACE() << "Since we are in 11bpp, we impose Mono12 pixel encoding.";
    // We are reading 11/12bpp images,
    // Make sure that we are not in the 12b/packed mode (1.5 byte per pixe, not handled by LIMA).
    setEnumString(andor3::PixelEncoding, L"Mono12");
  }
  
  // Releasing the previously used buffers :
  
  // Getting the SoftBufferCtlMgr to allocate the proper frame buffers :
  AT_64       the_width, the_height, the_stride;
  double			the_byte_p_px;
  
  // Indeed it seems LIMA is not having the difference between width and stride.
  // So we have to compute the width provided to LIMA from the stride and the
  // byte per pixel settings.

  getInt(andor3::AOIWidth, &the_width);
  getInt(andor3::AOIHeight, &the_height);
  getInt(andor3::AOIStride, &the_stride);
  getFloat(andor3::BytesPerPixel, &the_byte_p_px);
  DEB_TRACE() << "Image size parameters are : width " << the_width
  << ", height " << the_height
  << ", stride " << the_stride
  << " and finaly byte per pixel " << the_byte_p_px;
  
  the_width = the_stride>>1; // Dividing by 2, we are for sure in a mode with 2.0 byte per pixel
  DEB_TRACE() << "To ensure that the LIMA image width corresponds to the stride, set it to "
  << the_width;
  lima::Size	the_frame_size(static_cast<int>(the_width), static_cast<int>(the_height)); // Including the full row (that is Stride, not only width)
  FrameDim		the_frame_dim;  // Adding the information from the image/pixel format/depth
  the_frame_dim.setSize(the_frame_size);

  if ( 2.0 != the_byte_p_px ) {
    THROW_HW_ERROR(Error) << "Andor3 SDK : managed to get a setting where there is not exactly 2 Bytes per pixel ! (currently " << the_byte_p_px << "B/px)";
  }
  
  // Setting the other information of the frame :
  switch (the_bit_depth_index) {
    case b11:
      the_frame_dim.setImageType(Bpp12);
      break;
    case b16:
      the_frame_dim.setImageType(Bpp16);
      break;
    default:
      //! TODO : again trouble (signal to user), we don't know how to do that.
      break;
  }

  int 				the_max_frames;
  int					the_alloc_frames;
  
  the_alloc_frames = (0 == m_nb_frames_to_collect) ? 128 : static_cast<int>(m_nb_frames_to_collect);
  DEB_TRACE() << "The number of frames to be collected is set to : " << the_alloc_frames << ", before testing the memory available";
  
  m_buffer_ctrl_obj.setFrameDim(the_frame_dim);
  m_buffer_ctrl_obj.getMaxNbBuffers(the_max_frames);
  DEB_TRACE() << "Given above parameters, maximum numbe of frames in memory is "
  << the_max_frames;
  if (( the_max_frames < the_alloc_frames ) || ( 0 == m_nb_frames_to_collect )) {
    // If not enough memory or continuous acquisition we go into ring buffer mode :
    the_alloc_frames = ( the_max_frames < the_alloc_frames ) ? the_max_frames : the_alloc_frames ;
    m_buffer_ringing = true;
    DEB_TRACE() << "Setting ring mode, since we are either in continuous acquisition or not enough memory";
  }
  else {
    // Otherwise we allocate exactly th number of buffers necessary :
    the_alloc_frames = static_cast<int>(m_nb_frames_to_collect);
    m_buffer_ringing = false;
    DEB_TRACE() << "Setting the buffer single use mode";
  }

  DEB_TRACE() << "After testing available memory (and continuous acquisition), the mode is " << m_buffer_ringing << " and the number of frame to be allocated is : " << the_alloc_frames;
  
  StdBufferCbMgr& the_buffer = m_buffer_ctrl_obj.getBuffer();
  DEB_TRACE() << "Getting StdBufferCbMgr to allocate the buffers that we want to have";
  the_buffer.allocBuffers(the_alloc_frames, 1, the_frame_dim);
  int				the_frame_mem_size = the_frame_dim.getMemSize();
  
  // Handing the frame buffers to the SDK :
  // As proposed by the SDK, we make sure that we start from an empty queue :
  DEB_TRACE() << "Flushing the queue of the framegrabber";
  AT_Flush(m_camera_handle);
  
  // Then queue all the buffers allocated by StdBufferCbMgr
  DEB_TRACE() << "Pushing all the frame buffers to the frame grabber SDK";
  for ( int i=0; the_alloc_frames != i; ++i) {
    void*		the_buffer_ptr = the_buffer.getFrameBufferPtr(i);
    AT_QueueBuffer(m_camera_handle, static_cast<AT_U8*>(the_buffer_ptr), the_frame_mem_size);
    DEB_TRACE() << "Queueing the frame buffer " << i;
  }
  DEB_TRACE() << "Finished queueing " << the_alloc_frames << " frame buffers to andor's SDK3";
  // Seems to me that the «0 == m_nb_frames_to_collect» case corresponds to the continuous case
  // So next line is not making sense (and hence commented out) :
  // #warning Setting properly the continuous vs. fixed acquisition mode of the camera

  if ( 0 == m_nb_frames_to_collect ) {
    // We are in continuous acquisition mode : set the camera appropriately
    DEB_TRACE() << "m_nb_frames_to_collect=0 : this means we want continuous acquisition. Trying to set teh camera accrodingly";
    setEnumString(andor3::CycleMode, L"Continuous");
  }
  else {
    AT_64        the_frame_count = static_cast<AT_64>(m_nb_frames_to_collect);
    DEB_TRACE() << "m_nb_frames_to_collect=" << m_nb_frames_to_collect << " : setting the camera in fixed number of frame mode";
    setEnumString(andor3::CycleMode, L"Fixed");
    DEB_TRACE() << "Then setting the number of frame appropriatly";
    setInt(andor3::FrameCount, the_frame_count);
    getInt(andor3::FrameCount, &the_frame_count);
    DEB_TRACE() << "Now proof-reading the number of frames to collect : " << the_frame_count << " (was requested :" << m_nb_frames_to_collect << ")";
    if ( the_frame_count != m_nb_frames_to_collect ) {
      DEB_ERROR() << "Got erreo !!! Required to collect : " << m_nb_frames_to_collect << " frames, but the SDK is thinking it should collect " << the_frame_count << " frames!!!";
    }
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
    m_buffer_ctrl_obj.getBuffer().setStartTimestamp(Timestamp::now());
    // Sending the start command to the SDK :
    sendCommand(andor3::AcquisitionStart);
  }
  
  if ( Software == m_trig_mode ) {
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
  return &m_buffer_ctrl_obj;
}

//-- Synch control object
bool
lima::Andor3::Camera::checkTrigMode(TrigMode mode)
{
  switch (mode) {
    case IntTrig:
    case IntTrigMult:
    case ExtTrigSingle:
    case ExtTrigMult:
    case ExtGate:
      return true;
      break;
      
    default:
      return false;
      break;
  }
}

void
lima::Andor3::Camera::setTrigMode(TrigMode  mode)
{
  DEB_MEMBER_FUNCT();
  A3_TriggerMode		the_trigger_mode;
  switch (mode) {
    case 	IntTrig:
      the_trigger_mode = Internal;
      break;
    case IntTrigMult:
      the_trigger_mode = Software;
      break;
    case ExtTrigSingle:
      the_trigger_mode = ExternalStart;
      break;
    case ExtTrigMult:
      the_trigger_mode = External;
      break;
    case ExtGate:
      the_trigger_mode = ExternalExposure;
      break;
      
    case ExtStartStop:
    case ExtTrigReadout:
    default:
      the_trigger_mode = Internal;
      THROW_HW_ERROR(Error) << "The triggering mode " << mode
      << " is NOT implemented in this SDK";
      break;
  }
  setTriggerMode(the_trigger_mode);
}

void
lima::Andor3::Camera::getTrigMode(TrigMode& mode)
{
  DEB_MEMBER_FUNCT();
  A3_TriggerMode		the_trigger_mode;
  getTriggerMode(the_trigger_mode);
  
  switch (the_trigger_mode) {
    case Internal:
      mode = IntTrig;
      break;
    case Software:
      mode = IntTrigMult;
      break;
    case ExternalStart:
      mode = ExtTrigSingle;
      break;
    case External:
      mode = ExtTrigMult;
      break;
    case ExternalExposure:
      mode = ExtGate;
      break;
    default:
      mode = IntTrig;
      THROW_HW_ERROR(Error) << "The triggering mode of the SDK " << the_trigger_mode
      << " does not correspond to any mode of LIMA, returning " << mode;
      break;
  }
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
lima::Andor3::Camera::setLatTime(double  lat_time)
{
  DEB_MEMBER_FUNCT();
  double			the_exp_time;
  double			the_rate;
  //  double			the_readout_time;
  
  getFloat(andor3::ExposureTime, &the_exp_time);
//  getFloat(andor3::ReadoutTime, &the_readout_time);
//  lat_time = ( lat_time > the_readout_time ) ? lat_time : the_readout_time;
//  if ( lat_time < the_readout_time ) {
//    lat_time = the_readout_time;
//    THROW_HW_ERROR(Error) << "You have requested a latency "
//  }
  the_rate = 1.0 / (the_exp_time + lat_time);
  setFloat(andor3::FrameRate, the_rate);
}

void
lima::Andor3::Camera::getLatTime(double& lat_time)
{
  DEB_MEMBER_FUNCT();
  double			the_exp_time;
  double			the_rate;

  getFloat(andor3::ExposureTime, &the_exp_time);
  getFloat(andor3::FrameRate, &the_rate);
  
  lat_time = (1.0 / the_rate) - the_exp_time;
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
lima::Andor3::Camera::getLatTimeRange(double& min_lat, double& max_lat) const
{
  DEB_MEMBER_FUNCT();
  double			the_exp_time;
  double			the_rate_min, the_rate_max;

  getFloat(andor3::ExposureTime, &the_exp_time);
  getFloatMin(andor3::FrameRate, &the_rate_min);
  getFloatMax(andor3::FrameRate, &the_rate_max);

  min_lat = (1.0 / the_rate_max) - the_exp_time;
  max_lat = (1.0 / the_rate_min) - the_exp_time;
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
  Bin				the_binning;
  
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
  
  // From now on, we are performing all tests in physical pixels (and we will convert back to super pixel just before returning).
  int				the_bin_nb = the_binning.getX();
  Roi				the_phys_set_roi;
  Roi				the_phys_test_roi;
  Roi				the_phys_hw_roi;
  
  //  the_phys_set_roi = Roi(set_roi.getTopLeft()*the_bin_nb, set_roi.getSize()*the_bin_nb);
  the_phys_set_roi = set_roi.getUnbinned(Bin(the_bin_nb, the_bin_nb));
  the_phys_test_roi = Roi(Point(0, 0), m_detector_size);
  
  DEB_TRACE() << "Requested ROI is : " << set_roi << ", corresponding to " << the_phys_set_roi << " in term of physical pixel";
  // First : check that we are smaller than the maximum AOI for the current binning
  if ( ! the_phys_test_roi.containsRoi(the_phys_set_roi) ) {
    the_phys_set_roi = the_phys_test_roi;
    // the_phys_hw_roi = the_phys_test_roi; // Taken care of later :
  }

  DEB_TRACE() << "After testing if this is included in the max area of the detector, got " << the_phys_set_roi << " in term of physical pixel";
  if ( the_fullaoi_control ) { // If full control : accept the requested ROI.
    the_phys_hw_roi = the_phys_set_roi;
  }
  else {
    
    DEB_TRACE() << "Testing a roi while the camera has only limited support for ROI (FullAOIControl==false)";
    DEB_TRACE() << "We will now scan the possible camera ROIs, selecting the smallest one that include the requested one (lower number of lines)";
    
    // If there is no FullAOIControll, then we should resort to one of the proposition of § 3.5 of the SDK manual (page 42) :
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
      NULL
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
    
    DEB_TRACE() << "The selected hardware ROI is now : " << the_phys_set_roi << " in term of physical pixel";
//    THROW_HW_ERROR(Error) << "Though it is feasible, we currently do not support hardware ROI"
//    << " for devices not having the FullAOIControl set";
    //    hw_roi...;
  }

  hw_roi = the_phys_hw_roi.getBinned(Bin(the_bin_nb, the_bin_nb));
  //  hw_roi = Roi(the_left, the_top, the_width, the_height);
}

void
lima::Andor3::Camera::setRoi(const Roi& set_roi)
{
  DEB_MEMBER_FUNCT();
  Bin			the_binning;
  AT_64		the_left, the_width, the_top, the_height;
  AT_64		the_bin_nb;

  getBin(the_binning);
  the_bin_nb = the_binning.getX();
  the_left = static_cast<AT_64>(set_roi.getTopLeft().x) * the_bin_nb;
  the_width = static_cast<AT_64>(set_roi.getSize().getWidth());
  the_top = static_cast<AT_64>(set_roi.getTopLeft().y) * the_bin_nb;
  the_height = static_cast<AT_64>(set_roi.getSize().getHeight());
  
  // Performing the settings in the order prescribed by the SDK's documentation:
  // Binning, width, left, heigh, top :
  setBin(the_binning);
  setInt(andor3::AOIWidth, the_width);
  setInt(andor3::AOILeft, the_left);
  setInt(andor3::AOIHeight, the_height);
  setInt(andor3::AOITop, the_top);
}

void
lima::Andor3::Camera::getRoi(Roi& hw_roi)
{
  DEB_MEMBER_FUNCT();
  
  AT_64		the_left, the_width, the_top, the_height;
  getInt(andor3::AOIWidth, &the_width);
  getInt(andor3::AOILeft, &the_left);
  getInt(andor3::AOIHeight, &the_height);
  getInt(andor3::AOITop, &the_top);

  hw_roi = Roi(static_cast<int>(the_left), static_cast<int>(the_top), static_cast<int>(the_width), static_cast<int>(the_height));
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
#warning Indeed ROI is also available when the_fullaoi_control is false, only that checkRoi dos not handle it so far.
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
  sizex = sizey = 6.5; // in micron ?
}

void
lima::Andor3::Camera::getStatus(Camera::Status& status)
{
  DEB_MEMBER_FUNCT();
  AutoMutex aLock(m_cond.mutex());
  status = m_status;
  DEB_RETURN() << DEB_VAR1(DEB_HEX(status));

/* OLD version, not using an instance variable :
#warning Complete crap at the moment !!!
  DEB_MEMBER_FUNCT();
  bool		the_acq;
  
  getBool(andor3::CameraAcquiring, &the_acq);
  if ( ! the_acq ) {
    status = Ready;
    return;
  }
  if ( m_image_index < m_nb_frames_to_collect ) {
    status = Exposure;
  }
  else {
    status = Readout;
  }
  return;
 */
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
  return m_image_index;
}

// -- andor3 specific, LIMA don't worry about it !
void
lima::Andor3::Camera::initialiseController()
{
  DEB_MEMBER_FUNCT();
  A3_BitDepth			the_bd = m_bit_depth;
  A3_Gain 				the_gain = m_adc_gain;
  A3_ReadOutRate	the_rate = m_adc_rate;
  
  // Carefully crafting the order, since some are affecting others...
  setElectronicShutterMode(m_electronic_shutter_mode);
  setTriggerMode(m_trig_mode);
  setAdcGain(the_gain);
  setAdcRate(the_rate);
  setBitDepth(the_bd);
  setCooler(m_cooler);
  setTemperatureSP(m_temperature_sp);
  setExpTime(m_exp_time);
  
  AT_64			the_chip_width, the_chip_height;
  getInt(andor3::SensorWidth, &the_chip_width);
  getInt(andor3::SensorHeight, &the_chip_height);
  
  m_detector_size = Size(static_cast<int>(the_chip_width), static_cast<int>(the_chip_height));

  // Setting the ROI to the max :
  Roi aRoi = Roi(0, 0, m_detector_size.getWidth(), m_detector_size.getHeight());
  DEB_TRACE() << "Set the ROI to full frame: "<< aRoi;
  setRoi(aRoi);

  if ( m_real_camera ) {
    // Making sure the «spurious noise filter» is OFF :
    if ( AT_SUCCESS != setBool(andor3::SpuriousNoiseFilter, false) ) {
      DEB_ERROR() << "Cannot set SpuriousNoiseFilter to false" << " : error code = " << m_camera_error_str;
      THROW_HW_ERROR(Error) << "Cannot set SpuriousNoiseFilter to false";
    }
  }
  else {
    DEB_TRACE() << "Since the camera is SIMULATED, it is not possible to set it noise filter (" << "SpuriousNoiseFilter" << ") to OFF.";
  }

  if ( NULL == m_acq_thread ) {
    m_acq_thread = new _AcqThread(*this);
    m_acq_thread->start();
  }
}

void
lima::Andor3::Camera::setAdcGain(A3_Gain iGain)
{
  DEB_MEMBER_FUNCT();
  if ( m_real_camera ) {
    int the_gain;
    setEnumIndex(andor3::PreAmpGainControl, iGain);
    getEnumIndex(andor3::PreAmpGainControl, &the_gain);
    m_adc_gain = static_cast<A3_Gain>(the_gain);
    if ( m_adc_gain != iGain ) {
      DEB_ERROR() << "Proof-reading the ADC readout gain :"
      << "\n\tGot " << m_adc_gain << " back,"
      << "\n\twhile requesting " << iGain;
    }
  }
  else {
    DEB_TRACE() << "Setting the gain is not possible for the SIMCAM. Skipping this request (value requested was : " << iGain << ").";
  }
  // This can automatically update the PixelEncoding and BitDepth to values corresponding to the gain
  // Hence we proof-read the new values and update the cache accordingly :
  int			the_bit_depth; // Pixel encoding will be reset appropriately in the setBitDepth method
  //  int			the_px_encoding;
  
  getEnumIndex(andor3::BitDepth, &the_bit_depth);
  //  getEnumIndex(andor3::PixelEncoding, &the_px_encoding);

  setBitDepth(static_cast<A3_BitDepth>(the_bit_depth));
}

void
lima::Andor3::Camera::getAdcGain(A3_Gain &oGain)
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
  getEnumStringByIndex(andor3::PreAmpGainControl, m_adc_gain, the_string, 255);
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
lima::Andor3::Camera::getAdcRate(A3_ReadOutRate &oRate)
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
  getEnumStringByIndex(andor3::PixelReadoutRate, m_adc_rate, the_string, 255);
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
  
  getEnumIndex(andor3::PreAmpGainControl, &the_gain);
  getEnumIndex(andor3::PixelReadoutRate, &the_rate);
  setAdcGain(static_cast<A3_Gain>(the_gain));
  setAdcRate(static_cast<A3_ReadOutRate>(the_rate));
}

void
lima::Andor3::Camera::getElectronicShutterMode(A3_ShutterMode &oMode)
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
  getEnumIndex(andor3::BitDepth, &the_mode);
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
      setEnumString(andor3::PixelEncoding, L"Mono12");
      break;
    default:
      break;
  }
}

void
lima::Andor3::Camera::getBitDepth(A3_BitDepth &oMode)
{
  DEB_MEMBER_FUNCT();
//  int the_mode;
//  getEnumIndex(andor3::PixelEncoding, &the_mode);
//  oMode = static_cast<A3_BitDepth>(the_mode);
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
lima::Andor3::Camera::getPxEncoding(std::string &oPxEncoding) const
{
  AT_WC		the_string[256];
  getEnumString(andor3::PixelEncoding, the_string, 255);
  oPxEncoding = WStringToString(std::wstring(the_string));
}


/*!
 @brief Setting the trigger mode of the camera through the SDK settings.
 @param iMode : the mode to select
 */
void
lima::Andor3::Camera::setTriggerMode(A3_TriggerMode iMode)
{
  DEB_MEMBER_FUNCT();
  if ( m_real_camera ) {
    int the_mode;
    setEnumIndex(andor3::TriggerMode, static_cast<int>(iMode));
    getEnumIndex(andor3::TriggerMode, &the_mode);
    m_trig_mode = static_cast<A3_TriggerMode>(the_mode);
    if ( m_trig_mode != iMode ) {
      DEB_ERROR() << "Proof-reading the trigger mode :"
      << "\n\tGot " << m_trig_mode << " back,"
      << "\n\twhile requesting " << iMode;
    }
  }
  else { // Simulated camera -> setting it forcibly to «Advanced»
    int the_mode;
    setEnumIndex(andor3::TriggerMode, 5);
    getEnumIndex(andor3::TriggerMode, &the_mode);
    m_trig_mode = static_cast<A3_TriggerMode>(the_mode);
    DEB_TRACE() << "The SIMCAM has only one trigger-mode setting (Advanced), making sure that's what we are doing now";
  }
}

/*!
 @brief Getting the triggering mode the camera is in
 @param oMode : the mode selected upon return
 */
void
lima::Andor3::Camera::getTriggerMode(A3_TriggerMode &oMode)
{
  DEB_MEMBER_FUNCT();
  oMode = m_trig_mode;
}

void
lima::Andor3::Camera::getTriggerModeString(std::string &oModeString) const
{
  AT_WC		the_string[256];
  getEnumStringByIndex(andor3::TriggerMode, m_trig_mode, the_string, 255);
  oModeString = WStringToString(std::wstring(the_string));
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
  setFloat(andor3::TargetSensorTemperature, temp);
  getFloat(andor3::TargetSensorTemperature, &m_temperature_sp);
  if ( abs(m_temperature_sp - temp) > 0.1) {
    DEB_ERROR() << "Proof-reading temperature set-point : "
    << "\n\tproof-read = " << m_temperature_sp
    << "\n\twhile asked to be = " << temp;
    THROW_HW_ERROR(Error) << "Failed on setting temperature set-point";
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
// @brief	Gets cooling/temperature status
// @param	status : status as a string
//
//-----------------------------------------------------
void
lima::Andor3::Camera::getCoolingStatus(std::string& status)
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
lima::Andor3::Camera::getFrameRate(double &o_frame_rate)
{
  DEB_MEMBER_FUNCT();
  double			the_fr;
  getFloat(andor3::FrameRate, &the_fr);
  o_frame_rate = the_fr;
}

void
lima::Andor3::Camera::getFrameRateRange(double& o_min_fr, double& o_max_fr)
{
  DEB_MEMBER_FUNCT();
  double			the_min, the_max;
  
  getFloatMin(andor3::FrameRate, &the_min);
  getFloatMax(andor3::FrameRate, &the_max);
  
  o_min_fr = the_min;
  o_max_fr = the_max;
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
#warning The current implementation looses the acquired frames that were not transfered yet, in the case of a stopAcq.
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

    // If we were not asked for immediate leaving, let the acquisition thread end it :
    //    if ( ! iImmediate ) {
    //      return;
    //    }
    
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
lima::Andor3::Camera::getIntSystem(const AT_WC* Feature, AT_64* Value)
{
  DEB_STATIC_FUNCT();
  return (AT_SUCCESS != AT_GetInt(AT_HANDLE_SYSTEM, Feature, Value));
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
  AutoMutex					the_lock(m_cam.m_cond.mutex());
  StdBufferCbMgr		&the_buffer = m_cam.m_buffer_ctrl_obj.getBuffer();

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
    DEB_ALWAYS() << "[andor3 acquisition thread] Set running by main thread setting m_cam.m_acq_thread_waiting to false (or m_cam.m_acq_thread_should_quit to true)";
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
      int								the_returned_image_size;
      unsigned int			the_wait_timeout = AT_INFINITE;
      int               the_wait_queue_res;
      

      DEB_ALWAYS() << "[andor3 acquisition thread] Waiting for buffer index " << m_cam.m_image_index << " (AT_WaitBuffer)";
      the_wait_queue_res = AT_WaitBuffer(m_cam.m_camera_handle, &the_returned_image, &the_returned_image_size, the_wait_timeout);
      DEB_ALWAYS() << "[andor3 acquisition thread] DONE waiting for buffer index " << m_cam.m_image_index;
      
      // Testing if we were asked to stop the acquisition thread :
      // It is best to do that as soon as returning from the SDK, since otherwise it might block some 
      // thread interacting with the main thread.
      DEB_TRACE() << "[andor3 acquisition thread] Locking to test if we were asked to stop";
      the_lock.lock();
      the_acq_goon = !m_cam.m_acq_thread_waiting && !m_cam.m_acq_thread_should_quit;
      m_cam.m_acq_thread_running = the_acq_goon;
      DEB_TRACE() << "[andor3 acquisition thread] Should we continue at the end of this iteration : " << m_cam.m_acq_thread_running << " AKA " << the_acq_goon;
      m_cam.m_cond.broadcast();
      DEB_TRACE() << "[andor3 acquisition thread] Just broadcasted for other threads to know that it might be interesting to change state";
      the_lock.unlock();
      DEB_TRACE() << "[andor3 acquisition thread] Just Unlocked after state potential modification";

      if ( AT_SUCCESS == the_wait_queue_res ) {
        m_cam._setStatus(Readout, false);
        // We managed to get an image buffer returned :
        HwFrameInfoType		the_frame_info;
	bool                    the_frame_read;

        the_frame_info.acq_frame_nb = static_cast<int>(m_cam.m_image_index);
        the_frame_read = the_buffer.newFrameReady(the_frame_info);
        DEB_TRACE() << "[andor3 acquisition thread] image " << m_cam.m_image_index <<" published with newFrameReady(), with result " << the_frame_read ;
	the_acq_goon = the_acq_goon && the_frame_read;
        
        ++m_cam.m_image_index;

        if ( m_cam.m_buffer_ringing ) {
          DEB_TRACE() << "[andor3 acquisition thread] As we are using a ring-buffer : re-queueing the acquired image on the buffer queue.";
          AT_QueueBuffer(m_cam.m_camera_handle, the_returned_image, the_returned_image_size);
          DEB_TRACE() << "[andor3 acquisition thread] There is NO guarantee that LIMA will be done with this buffer BEFORE andor SDK3 is changing its content !!!";
        }
      }
      else {
        DEB_ERROR() << "[andor3 acquisition thread] Problem in retrieving the frame indexed " << m_cam.m_image_index <<"!\n"
        << "\tAT_WaitBuffer returned an error " << the_wait_queue_res << "\n"
        << "\t" << m_cam.m_andor3_error_maps[the_wait_queue_res] << "\n"
        << "\t!!! returning to WAIT mode !!!";
        m_cam.m_acq_thread_running = the_acq_goon = false;
        m_cam._setStatus(Fault, false);
	break;
      }
      DEB_TRACE() << "[andor3 acquisition thread] End of the iteration, next iteration will be for image index " << m_cam.m_image_index;
      
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
