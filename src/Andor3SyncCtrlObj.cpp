/* andor3 plugin synchronisation information class
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
#include "Andor3SyncCtrlObj.h"


//---------------------------
//- utility variables
//---------------------------


//---------------------------
//- @brief constructor
//---------------------------
lima::Andor3::SyncCtrlObj::SyncCtrlObj(lima::Andor3::Camera& cam) :
m_cam(cam)
{
  DEB_CONSTRUCTOR();
}

//---------------------------
//- @brief destructor
//---------------------------
lima::Andor3::SyncCtrlObj::~SyncCtrlObj()
{
  DEB_DESTRUCTOR();
}


#pragma mark -
#pragma mark Trigger Mode :
bool
lima::Andor3::SyncCtrlObj::checkTrigMode(TrigMode trig_mode)
{
  DEB_MEMBER_FUNCT();
  return m_cam.checkTrigMode(trig_mode);
}
void
lima::Andor3::SyncCtrlObj::setTrigMode(TrigMode  trig_mode)
{
  DEB_MEMBER_FUNCT();
  m_cam.setTrigMode(trig_mode);
}
void
lima::Andor3::SyncCtrlObj::getTrigMode(TrigMode& trig_mode)
{
  DEB_MEMBER_FUNCT();
  m_cam.getTrigMode(trig_mode);
}

#pragma mark -
#pragma mark Exposition time :
void
lima::Andor3::SyncCtrlObj::setExpTime(double  exp_time)
{
  DEB_MEMBER_FUNCT();
  m_cam.setExpTime(exp_time);
  updateValidRanges();
}
void
lima::Andor3::SyncCtrlObj::getExpTime(double& exp_time)
{
  DEB_MEMBER_FUNCT();
  m_cam.getExpTime(exp_time);
}

#pragma mark -
#pragma mark Latency time :
void
lima::Andor3::SyncCtrlObj::setLatTime(double  lat_time)
{
  DEB_MEMBER_FUNCT();
  m_cam.setLatTime(lat_time);
  updateValidRanges();
}
void
lima::Andor3::SyncCtrlObj::getLatTime(double& lat_time)
{
  DEB_MEMBER_FUNCT();
  m_cam.getLatTime(lat_time);
}

#pragma mark -
#pragma mark Frame numbers :
void
lima::Andor3::SyncCtrlObj::setNbHwFrames(int  nb_frames)
{
  DEB_MEMBER_FUNCT();
  m_cam.setNbFrames(nb_frames);
}
void
lima::Andor3::SyncCtrlObj::getNbHwFrames(int& nb_frames)
{
  DEB_MEMBER_FUNCT();
  m_cam.getNbHwAcquiredFrames(nb_frames);
}

void
lima::Andor3::SyncCtrlObj::getValidRanges(ValidRangesType& valid_ranges)
{
  DEB_MEMBER_FUNCT();
  double min_time;
  double max_time;
  m_cam.getExposureTimeRange(min_time, max_time);
  valid_ranges.min_exp_time = min_time;
  valid_ranges.max_exp_time = max_time;
  
  m_cam.getLatTimeRange(min_time, max_time);
  valid_ranges.min_lat_time = min_time;
  valid_ranges.max_lat_time = max_time; 

  DEB_RETURN() << DEB_VAR2(valid_ranges.min_exp_time, valid_ranges.max_exp_time);
  DEB_RETURN() << DEB_VAR2(valid_ranges.min_lat_time, valid_ranges.max_lat_time);
}


#pragma mark -
#pragma mark Acquisition possibilities updates :
void
lima::Andor3::SyncCtrlObj::updateValidRanges()
{
  DEB_MEMBER_FUNCT();
  DEB_ALWAYS() << "about to update the valid range for exposure and latency times...";
  ValidRangesType		the_v_range;
  getValidRanges(the_v_range);
  validRangesChanged(the_v_range);
}

/*
 Local Variables:
 c-file-style: "gnu"
 End:
 */
