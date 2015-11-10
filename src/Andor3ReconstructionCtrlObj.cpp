//###########################################################################
// This file is part of LImA, a Library for Image Acquisition
//
// Copyright (C) : 2009-2013
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

#include "processlib/SoftRoi.h"

using namespace lima;
using namespace lima::Andor3;

//-----------------------------------------------------
// @brief Ctor
//-----------------------------------------------------
ReconstructionCtrlObj::ReconstructionCtrlObj(Camera& cam, bool active):
  m_cam(cam), m_active(active)
{
  DEB_CONSTRUCTOR();

  m_task = new Tasks::SoftRoi();
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
  
  Roi roi;
  Size detectorSize;

  if (m_active) {    
    m_cam.getRoi(roi);
    m_cam.getDetectorImageSize(detectorSize);
    DEB_TRACE() << DEB_VAR1(roi);
    DEB_TRACE() << DEB_VAR1(detectorSize);
    // do nothing if a roi is set, checkRoi return the hw_roi with stride to get a SoftRoi to be applied.
    if (roi.getSize().getWidth() == detectorSize.getWidth() && roi.getSize().getHeight() == detectorSize.getHeight()) {
      if (m_cam.getPixelStride() != roi.getSize().getWidth()) {
	const Point& topLeft = roi.getTopLeft();
	const Point& bottomRight = roi.getBottomRight();
	m_task->setRoi(topLeft.x, bottomRight.x, topLeft.y, bottomRight.y);	
	reconstructionChange(m_task);
      } else {
	reconstructionChange(NULL);
      }
    } else {
      reconstructionChange(NULL);      
    }
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

void ReconstructionCtrlObj::setActive(bool active)
{
  DEB_MEMBER_FUNCT();
  m_active = active;
  if (!active) reconstructionChange(NULL);
}

void ReconstructionCtrlObj::getActive(bool& active) const
{
  DEB_MEMBER_FUNCT();
  active = m_active;
}
