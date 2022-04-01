/* andor3 plugin hardware interface class
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
//#include <sstream>
//#include <iostream>
//#include <string>
//#include <math.h>

// Camera SDK headers :

// LImA headers :

// Andor3 plugin headers :
#include "Andor3Interface.h"


//---------------------------
//- utility variables
//---------------------------


//---------------------------
//- @brief constructor
//---------------------------
lima::Andor3::Interface::Interface(lima::Andor3::Camera& cam) :
m_cam(cam),
m_cap_list()
{
  DEB_CONSTRUCTOR();
  
  m_det_info = new DetInfoCtrlObj(m_cam, this);
  m_sync = new SyncCtrlObj(m_cam);
  m_roi = new RoiCtrlObj(m_cam, this);
  m_bin = new BinCtrlObj(m_cam, this);
  m_reconstruction = new ReconstructionCtrlObj(m_cam);

  // Taking care of the content of the CapList, once for all :
  m_cap_list.push_back(HwCap(m_det_info));
  m_cap_list.push_back(m_cam.getBufferCtrlObj());
  m_cap_list.push_back(m_sync);
  m_cap_list.push_back(HwCap(m_roi));
  m_cap_list.push_back(HwCap(m_bin));
  m_cap_list.push_back(HwCap(m_reconstruction));

}

//---------------------------
//- @brief destructor
//---------------------------
lima::Andor3::Interface::~Interface()
{
  DEB_DESTRUCTOR();
  
  delete m_det_info;
  delete m_sync;
  delete m_roi;
  delete m_bin;
  delete m_reconstruction;
}

void
lima::Andor3::Interface::getCapList(HwInterface::CapList &o_cap_list) const
{
  DEB_MEMBER_FUNCT();
  o_cap_list = m_cap_list;
}



void
lima::Andor3::Interface::reset(ResetLevel reset_level)
{
  DEB_MEMBER_FUNCT();
  DEB_PARAM() << DEB_VAR1(reset_level);

  stopAcq();
  //#warning Should make sure that within finite time the camera returns to status Ready !!!
  // Should be done now...
}

void
lima::Andor3::Interface::prepareAcq()
{
  DEB_MEMBER_FUNCT();
  m_cam.prepareAcq();
  m_reconstruction->prepareAcq();
}

void
lima::Andor3::Interface::startAcq()
{
  DEB_MEMBER_FUNCT();
  m_cam.startAcq();
}

void
lima::Andor3::Interface::stopAcq()
{
  DEB_MEMBER_FUNCT();
  m_cam.stopAcq();
}

void
lima::Andor3::Interface::getStatus(StatusType& status)
{
  DEB_MEMBER_FUNCT();
  Camera::Status andor3_status = Camera::Ready;
  m_cam.getStatus(andor3_status);
  switch (andor3_status)
  {
    case Camera::Ready:
      status.acq = AcqReady;
      status.det = DetIdle;
      break;
    case Camera::Exposure:
      status.det = DetExposure;
      status.acq = AcqRunning;
      break;
    case Camera::Readout:
       status.det = DetReadout;
       status.acq = AcqRunning;
       break;
      /*
       case Camera::Latency:
       status.det = DetLatency;
       status.acq = AcqRunning;
       break;
       */
    case Camera::Fault:
      status.det = DetFault;
      status.acq = AcqFault;
  }
  status.det_mask = DetExposure | DetReadout | DetLatency;
  
  DEB_RETURN() << DEB_VAR1(status);
}

int
lima::Andor3::Interface::getNbHwAcquiredFrames()
{
  DEB_MEMBER_FUNCT();
  return m_cam.getNbHwAcquiredFrames();
}

void
lima::Andor3::Interface::updateValidRanges()
{
  DEB_MEMBER_FUNCT();
  if ( m_sync ) {
    m_sync->updateValidRanges();
  }
}

void
lima::Andor3::Interface::setAdcGain(Camera::A3_Gain iGain)
{
  DEB_MEMBER_FUNCT();
  m_cam.setAdcGain(iGain);
  updateValidRanges();
}

void
lima::Andor3::Interface::getAdcGain(Camera::A3_Gain &oGain) const
{
  DEB_MEMBER_FUNCT();
  m_cam.getAdcGain(oGain);
}

void
lima::Andor3::Interface::getAdcGainString(std::string &oGainString) const
{
  DEB_MEMBER_FUNCT();
  m_cam.getAdcGainString(oGainString);
}

void
lima::Andor3::Interface::setSimpleGain(std::string iGain)
{
  DEB_MEMBER_FUNCT();
  m_cam.setSimpleGain(iGain);
  updateValidRanges();
}

void
lima::Andor3::Interface::getSimpleGain(std::string &oGain) const
{
  DEB_MEMBER_FUNCT();
  m_cam.getSimpleGain(oGain);
}

void
lima::Andor3::Interface::getSimpleGainList(std::vector<std::string> &gain_list) const
{
  DEB_MEMBER_FUNCT();
  m_cam.getSimpleGainList(gain_list);
}



void
lima::Andor3::Interface::setAdcRate(std::string iRate)
{
  DEB_MEMBER_FUNCT();
  m_cam.setAdcRate(iRate);
  updateValidRanges();
}

void
lima::Andor3::Interface::getAdcRate(std::string &oRate) const
{
  DEB_MEMBER_FUNCT();
  m_cam.getAdcRate(oRate);
}

void
lima::Andor3::Interface::getAdcRateList(std::vector<std::string> &oRateList) const
{
  DEB_MEMBER_FUNCT();
  m_cam.getAdcRateList(oRateList);
}

void
lima::Andor3::Interface::setElectronicShutterMode(std::string iMode)
{
  DEB_MEMBER_FUNCT();
  m_cam.setElectronicShutterMode(iMode);
  updateValidRanges();
}

void
lima::Andor3::Interface::getElectronicShutterMode(std::string &oMode) const
{
  DEB_MEMBER_FUNCT();
  m_cam.getElectronicShutterMode(oMode);
}

void
lima::Andor3::Interface::getElectronicShutterModeList(std::vector<std::string> &oModeList) const
{
  DEB_MEMBER_FUNCT();
  m_cam.getElectronicShutterModeList(oModeList);
}

void
lima::Andor3::Interface::setOverlap(bool i_overlap)
{
  DEB_MEMBER_FUNCT();
  m_cam.setOverlap(i_overlap);
  updateValidRanges();
}

void
lima::Andor3::Interface::getOverlap(bool &o_overlap) const
{
  DEB_MEMBER_FUNCT();
  m_cam.getOverlap(o_overlap);
}

void
lima::Andor3::Interface::setSyncTriggering(bool i_sync)
{
  DEB_MEMBER_FUNCT();
  m_cam.setSyncTriggering(i_sync);
  updateValidRanges();
}

void
lima::Andor3::Interface::getSyncTriggering(bool &o_sync) const
{
  DEB_MEMBER_FUNCT();
  m_cam.getSyncTriggering(o_sync);
}

/*
 Local Variables:
 c-file-style: "gnu"
 End:
 */
