//###########################################################################
// This file is part of LImA, a Library for Image Acquisition
//
// Copyright (C) : 2009-2015
// European Synchrotron Radiation Facility
// BP 220, Grenoble 38043
// FRANCE
//
// This is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 3 of the License, or
// (at your option) any later version.
//
// This software is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, see <http://www.gnu.org/licenses/>.
//###########################################################################

#include "Andor3ReconstructionCtrlObj.h"
#include "Andor3Camera.h"

#include "processlib/LinkTask.h"
#include "processlib/SoftRoi.h"
#include "processlib/ProcessExceptions.h"

using namespace lima;
using namespace lima::Andor3;


class _ReconstructionTask : public LinkTask
{
  DEB_CLASS_NAMESPC(DebModCameraCom, "_ReconstructionTask", "Andor3");
public:
  _ReconstructionTask()
  {
    AT_InitialiseUtilityLibrary();
  }

  ~_ReconstructionTask()
  {
    AT_FinaliseUtilityLibrary();
  }

  void setSdkFrameDim(Camera::SdkFrameDim& frame_dim){m_sdk_frame_dim = frame_dim;};
  void setBuffer( HwBufferCtrlObj* buffer_ctrl_obj) {m_buffer_ctrl_obj= (SoftBufferCtrlObj *)buffer_ctrl_obj;};

  virtual Data process(Data&);
private:
  Camera::SdkFrameDim m_sdk_frame_dim;
  SoftBufferCtrlObj* m_buffer_ctrl_obj;
};

Data  _ReconstructionTask::process(Data& src)
{
  DEB_MEMBER_FUNCT();

  StdBufferCbMgr &the_buffer_mgr = m_buffer_ctrl_obj->getBuffer();
  if (src.depth() != 2) 
    {
      char ErrorBuff[1024];
      snprintf(ErrorBuff,sizeof(ErrorBuff),
	       "_ReconstructionTask: can't decode data only for 12bit packed, 2 bytes and data depth is %d",
	       src.depth());
      throw ProcessException(ErrorBuff);     
    }
  
  int frame_number = src.frameNumber;
  void * dest_buffer_ptr = src.data();
  void * src_buffer_ptr = the_buffer_mgr.getFrameBufferPtr(frame_number);
  
  int res = AT_ConvertBuffer(static_cast<AT_U8*>(src_buffer_ptr), 
			     static_cast<AT_U8*>(dest_buffer_ptr),
			     m_sdk_frame_dim.width,
			     m_sdk_frame_dim.height, 
			     m_sdk_frame_dim.stride,
			     m_sdk_frame_dim.input_encoding,
			     m_sdk_frame_dim.output_encoding);

  if (! res == AT_SUCCESS)
    {
      THROW_HW_ERROR(Error) << "Cannot convert buffer to Mono16";
      
    }
  
  // Mono12Packed encoding: 3 bytes for 2 pixels
  //   ----------------------------------------------------------------------------------
  //  |Bits 11:4 = Most Significant Bits (MSB) | Bits 3:0 = Least Significant Bits (LSB) |
  //   ----------------------------------------------------------------------------------
  //  In buffer memory (ImageBuffer)
  //  -----------------------------------------------
  // | ImageBuffer+0 |           Pixel A (MSB)       |
  //  -----------------------------------------------
  // | ImageBuffer+1 | Pixel B (LSB) | Pixel A (LSB) |
  //  -----------------------------------------------
  // | ImageBuffer+2 |           Pixel B (MSB)       |
  //  -----------------------------------------------
  // | ImageBuffer+3 |           Pixel C (MSB)       |
  //  -----------------------------------------------
  // | ImageBuffer+4 | Pixel D (LSB) | Pixel C (LSB) |
  //  -----------------------------------------------
  // | ImageBuffer+5 |           Pixel D (MSB)       |
  //  -----------------------------------------------
  //

  /***
  int nb_pixels = src.size()/src.depth();
  // for this unpacking algorithm we seek memory from bottom to top address
  // this is not super efficient but a choice to not use 2 buffer systems for memory copies.
  // move to the latest pixel memory position
  unsigned short* dst_data = (unsigned short*)src.data()+nb_pixels-1;
  // move the packed ptr to the last byte as well
  int coded_size = (int) (nb_pixels * 1.5);
  unsigned char* coded_ptr = (unsigned char*) src.data() + coded_size;
  
  
  unsigned short msb_byte1, msb_byte2, lsb_byte;
  while (nb_pixels)
    {
      msb_byte2 = *(--coded_ptr); lsb_byte= *(--coded_ptr); msb_byte1 = *(--coded_ptr);
      
      *dst_data = (msb_byte2 << 4) + (lsb_byte >> 4);
      *(--dst_data) = (msb_byte1 << 4) + (lsb_byte & 0xF);
      nb_pixels-=2;
      --dst_data;
    }
  ***/  
  return src;
}

//-----------------------------------------------------
// @brief Ctor
//-----------------------------------------------------
ReconstructionCtrlObj::ReconstructionCtrlObj(Camera& cam):
  m_cam(cam), m_active(false)
{
  DEB_CONSTRUCTOR();

  m_task = new _ReconstructionTask();
  m_task->setProcessingInPlace(true);
}

//-----------------------------------------------------
// @brief Dtor
//-----------------------------------------------------
ReconstructionCtrlObj::~ReconstructionCtrlObj()
{
  DEB_DESTRUCTOR();
  delete m_task;
}

void ReconstructionCtrlObj::prepareAcq()
{
  DEB_MEMBER_FUNCT();
  Camera::SdkFrameDim the_sdk_frame_dim;
  m_cam.getSdkFrameDim(the_sdk_frame_dim, true);
  if (!the_sdk_frame_dim.is_encoded && ! the_sdk_frame_dim.is_strided)
    {
      m_active = false;
      reconstructionChange(NULL);
    }
  else
    {
      m_active = true;
      m_task->setSdkFrameDim(the_sdk_frame_dim);
      m_task->setBuffer(m_cam.getTempBufferCtrlObj());
      reconstructionChange(m_task);
    }
}

//-----------------------------------------------------
// @brief return the task if active otherwise return NULL
//-----------------------------------------------------
LinkTask* ReconstructionCtrlObj::getReconstructionTask()
{
  DEB_MEMBER_FUNCT();

  if (m_active) return m_task;
  else return NULL;
}

void ReconstructionCtrlObj::getActive(bool& active) const
{
  DEB_MEMBER_FUNCT();
  active = m_active;
}
