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
  //  delete m_acq_thread;
  //  m_acq_thread = NULL;
  
  // Close camera
  if (m_cooler) {
    DEB_ERROR() <<"Please stop the cooling before shuting dowm the camera\n"
    << "brutale heating could damage the sensor.\n"
    << "And wait until temperature rises above 5 deg, before shuting down.";
    
    THROW_HW_ERROR(Error)<<"Please stop the cooling before shuting dowm the camera\n"
    << "brutale heating could damage the sensor.\n"
    << "And wait until temperature rises above 5 deg, before shuting down.";
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
  int					the_bit_depth;
  getEnumIndex(andor3::BitDepth, &the_bit_depth);
  DEB_TRACE() << "The bith depth (index) of the image is " << the_bit_depth;
  
  
  if ( b11 == the_bit_depth ) {
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
  switch (the_bit_depth) {
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
  
  m_buffer_ctrl_obj.setFrameDim(the_frame_dim);
  m_buffer_ctrl_obj.getMaxNbBuffers(the_max_frames);
  DEB_TRACE() << "Given above parameters, maximum numbe of frames in memory is "
  << the_max_frames;
  if (( the_max_frames < the_alloc_frames ) || ( 0 == m_nb_frames_to_collect )) {
    // If not enough memory or continuous acquisition we go into ring buffer mode :
    the_alloc_frames = the_max_frames;
    m_buffer_ringing = true;
    DEB_TRACE() << "Setting ring mode, since we are either in continuous acquisition or not enough memory";
  }
  else {
    // Otherwise we allocate exactly th number of buffers necessary :
    the_alloc_frames = static_cast<int>(m_nb_frames_to_collect);
    m_buffer_ringing = false;
    DEB_TRACE() << "Setting the buffer single use mode";
  }
  
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
  }
  DEB_TRACE() << "Finished queueing " << the_alloc_frames << " frame buffers to andor's SDK3";
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
  DEB_TRACE() << "Starting the acquisition by the camera";
  sendCommand(andor3::AcquisitionStart);
  
  DEB_TRACE() << "Resuming the action of the acquisition thread";
  AutoMutex    the_lock(m_cond.mutex());
  m_acq_thread_waiting = false;
  m_cond.broadcast();
  
  DEB_TRACE() << "Done, the acquisition is now started and the frame retrieving should take place in parallel in a second thread";
}

void
lima::Andor3::Camera::stopAcq()
{
  _stopAcq(false);
}

// -- detector info object
void
lima::Andor3::Camera::getImageType(ImageType& type)
{
  DEB_MEMBER_FUNCT();
}

void
lima::Andor3::Camera::setImageType(ImageType type)
{
  DEB_MEMBER_FUNCT();
}

void
lima::Andor3::Camera::getDetectorType(std::string& type)
{
  DEB_MEMBER_FUNCT();
}

void
lima::Andor3::Camera::getDetectorModel(std::string& model)
{
  DEB_MEMBER_FUNCT();
}

void
lima::Andor3::Camera::getDetectorImageSize(Size& size)
{
  DEB_MEMBER_FUNCT();
}

// -- Buffer control object
lima::HwBufferCtrlObj*
lima::Andor3::Camera::getBufferCtrlObj()
{
  DEB_MEMBER_FUNCT();
}

//-- Synch control object
void
lima::Andor3::Camera::setTrigMode(TrigMode  mode)
{
  DEB_MEMBER_FUNCT();
}

void
lima::Andor3::Camera::getTrigMode(TrigMode& mode)
{
  DEB_MEMBER_FUNCT();
}

void
lima::Andor3::Camera::setExpTime(double  exp_time)
{
  DEB_MEMBER_FUNCT();
}

void
lima::Andor3::Camera::getExpTime(double& exp_time)
{
  DEB_MEMBER_FUNCT();
}

void
lima::Andor3::Camera::setLatTime(double  lat_time)
{
  DEB_MEMBER_FUNCT();
}
void
lima::Andor3::Camera::getLatTime(double& lat_time)
{
  DEB_MEMBER_FUNCT();
}

void
lima::Andor3::Camera::getExposureTimeRange(double& min_expo, double& max_expo) const
{
  DEB_MEMBER_FUNCT();
}
void
lima::Andor3::Camera::getLatTimeRange(double& min_lat, double& max_lat) const
{
  DEB_MEMBER_FUNCT();
}

void
lima::Andor3::Camera::setNbFrames(int  nb_frames)
{
  DEB_MEMBER_FUNCT();
}
void
lima::Andor3::Camera::getNbFrames(int& nb_frames)
{
  DEB_MEMBER_FUNCT();
}
void
lima::Andor3::Camera::getNbHwAcquiredFrames(int &nb_acq_frames)
{
  DEB_MEMBER_FUNCT();
}

void
lima::Andor3::Camera::checkRoi(const Roi& set_roi, Roi& hw_roi)
{
  DEB_MEMBER_FUNCT();
}
void
lima::Andor3::Camera::setRoi(const Roi& set_roi)
{
  DEB_MEMBER_FUNCT();
}
void
lima::Andor3::Camera::getRoi(Roi& hw_roi)
{
  DEB_MEMBER_FUNCT();
}

bool
lima::Andor3::Camera::isBinningAvailable()
{
  DEB_MEMBER_FUNCT();
}

void
lima::Andor3::Camera::checkBin(Bin&)
{
  DEB_MEMBER_FUNCT();
}
void
lima::Andor3::Camera::setBin(const Bin&)
{
  DEB_MEMBER_FUNCT();
}

void
lima::Andor3::Camera::getBin(Bin&)
{
  DEB_MEMBER_FUNCT();
}

void
lima::Andor3::Camera::setShutterMode(ShutterMode mode)
{
  DEB_MEMBER_FUNCT();
}

void
lima::Andor3::Camera::getShutterMode(ShutterMode& mode)
{
  DEB_MEMBER_FUNCT();
}

void
lima::Andor3::Camera::setShutter(bool flag)
{
  DEB_MEMBER_FUNCT();
}

void
lima::Andor3::Camera::getShutter(bool& flag)
{
  DEB_MEMBER_FUNCT();
}

void
lima::Andor3::Camera::getPixelSize(double& sizex, double& sizey)
{
  DEB_MEMBER_FUNCT();
}

void
lima::Andor3::Camera::getStatus(Camera::Status& status)
{
  DEB_MEMBER_FUNCT();
}

// --- Acquisition interface
void
lima::Andor3::Camera::reset()
{
  DEB_MEMBER_FUNCT();
}

int
lima::Andor3::Camera::getNbHwAcquiredFrames()
{
  DEB_MEMBER_FUNCT();
}

// -- andor3 specific, LIMA don't worry about it !
void
lima::Andor3::Camera::initialiseController()
{
  DEB_MEMBER_FUNCT();
  setAdcGain(m_adc_gain);
  setAdcRate(m_adc_rate);
  setElectronicShutterMode(m_electronic_shutter_mode);
  setBitDepth(m_bit_depth);
  setCooler(m_cooler);
  setTemperatureSP(m_temperature_sp);
  setExpTime(m_exp_time);
  setTrigMode(IntTrig);
  
  Size sizeMax;
  getDetectorImageSize(sizeMax);

  // Setting the ROI to the max :
  Roi aRoi = Roi(0, 0, sizeMax.getWidth(), sizeMax.getHeight());
  DEB_TRACE() << "Set the ROI to full frame: "<< aRoi;
  setRoi(aRoi);

  // Making sure the «spurious noise filter» is OFF :
  if ( AT_SUCCESS != setBool(andor3::SpuriousNoiseFilter, false) ) {
    DEB_ERROR() << "Cannot set SpuriousNoiseFilter to false" << " : error code = " << m_camera_error_str;
    THROW_HW_ERROR(Error) << "Cannot set SpuriousNoiseFilter to false";
  }

  if ( NULL == m_acq_thread ) {
    m_acq_thread = new _AcqThread(*this);
  }
}

void
lima::Andor3::Camera::setAdcGain(A3_Gain iGain)
{
  DEB_MEMBER_FUNCT();
  int the_gain;
  setEnumIndex(andor3::PreAmpGainControl, iGain);
  getEnumIndex(andor3::PreAmpGainControl, &the_gain);
  m_adc_gain = static_cast<A3_Gain>(the_gain);
  if ( m_adc_rate != iGain ) {
    DEB_ERROR() << "Prood-reading the ADC readout gain :"
    << "\n\tGot " << m_adc_gain << "back,"
    << "\n\twhile requesting " << iGain;
  }
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
lima::Andor3::Camera::setAdcRate(A3_ReadOutRate iRate)
{
  DEB_MEMBER_FUNCT();
  int the_rate;
  setEnumIndex(andor3::PixelReadoutRate, iRate);
  getEnumIndex(andor3::PixelReadoutRate, &the_rate);
  m_adc_rate = static_cast<A3_ReadOutRate>(the_rate);
  if ( m_adc_rate != iRate ) {
    DEB_ERROR() << "Prood-reading the ADC readout rate :"
    << "\n\tGot " << m_adc_rate << "back,"
    << "\n\twhile requesting " << iRate;
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
lima::Andor3::Camera::setElectronicShutterMode(A3_ShutterMode iMode)
{
  DEB_MEMBER_FUNCT();
  int the_mode;
  setEnumIndex(andor3::ElectronicShutteringMode, iMode);
  getEnumIndex(andor3::ElectronicShutteringMode, &the_mode);
  m_electronic_shutter_mode = static_cast<A3_ShutterMode>(the_mode);
  if ( m_electronic_shutter_mode != iMode ) {
    DEB_ERROR() << "Prood-reading the electronic shutter mode :"
    << "\n\tGot " << m_electronic_shutter_mode << "back,"
    << "\n\twhile requesting " << iMode;
  }
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
lima::Andor3::Camera::setBitDepth(A3_BitDepth iMode)
{
  DEB_MEMBER_FUNCT();
  int the_mode;
  setEnumIndex(andor3::PixelEncoding, static_cast<int>(iMode));
  getEnumIndex(andor3::PixelEncoding, &the_mode);
  m_bit_depth = static_cast<A3_BitDepth>(the_mode);
  if ( m_bit_depth != iMode ) {
    DEB_ERROR() << "Prood-reading the image bit-depth :"
    << "\n\tGot " << m_bit_depth << "back,"
    << "\n\twhile requesting " << iMode;
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
  wchar_t		wcs_status_string[32];
  getEnumString(andor3::TemperatureControl, wcs_status_string, 31);
  status = WStringToString(wcs_status_string);
}

/*!
 @brief asking the full acquisition to stop
 @param iImmadiate : if true, the stop should be immediate, without waiting for inflight buffer to be retrieved by LIMA.
 */

void
_stopAcq(bool iImmediate)
{
  
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
  // This seems to be useless since :
  // Linux supports PTHREAD_SCOPE_SYSTEM, but not PTHREAD_SCOPE_PROCESS.
  // (http://man7.org/linux/man-pages/man3/pthread_attr_setscope.3.html 2013-04-19) ?
  //  pthread_attr_setscope(&m_thread_attr, PTHREAD_SCOPE_PROCESS);
}


// Signaling to the m_acq_thread that it should quit, then waiting for it to do it.
lima::Andor3::Camera::_AcqThread::~_AcqThread()
{
  AutoMutex aLock(m_cam.m_cond.mutex());
  m_cam.m_acq_thread_should_quit = true;
  m_cam.m_cond.broadcast();
  aLock.unlock();
  
  join();
}


// Either waiting to be signaled, or looping with the SDK to retrieve frame buffers
void
lima::Andor3::Camera::_AcqThread::threadFunction()
{
  
}

/*
 Local Variables:
 c-file-style: "gnu"
 End:
 */
