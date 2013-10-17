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
m_cap_list() // ,
//m_det_info(cam),
//m_sync(cam),
//m_bin(cam),
//m_roi(cam),
{
  DEB_CONSTRUCTOR();
  
  m_det_info = new DetInfoCtrlObj(m_cam);
  m_sync = new SyncCtrlObj(m_cam);
#warning To be implemented !!!
  //  m_roi = new RoiCtrlObj(m_cam);
  //  m_bin = new BinCtrlObj(m_cam);
  
  // Taking care of the content of the CapList, once for all :
  m_cap_list.push_back(HwCap(m_det_info));
  m_cap_list.push_back(m_cam.getBufferCtrlObj());
  m_cap_list.push_back(m_sync);
  
  //  m_cap_list.push_back(HwCap(m_roi));
  //  m_cap_list.push_back(HwCap(m_bin));

}

//---------------------------
//- @brief destructor
//---------------------------
lima::Andor3::Interface::~Interface()
{
  DEB_DESTRUCTOR();
}

void
lima::Andor3::Interface::reset(ResetLevel reset_level)
{
  DEB_MEMBER_FUNCT();
  DEB_PARAM() << DEB_VAR1(reset_level);

  stopAcq();
#warning Should make sure that within finite time the camera returns to status Ready !!!
}

void
lima::Andor3::Interface::prepareAcq()
{
  DEB_MEMBER_FUNCT();
  m_cam.prepareAcq();
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
       case Camera::Fault:
       status.det = DetFault;
       status.acq = AcqFault;
       */
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




/*
 Local Variables:
 c-file-style: "gnu"
 End:
 */
