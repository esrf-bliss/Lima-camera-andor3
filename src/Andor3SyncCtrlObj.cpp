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
  double min_exp_time, max_exp_time;
  double min_frame_rate, max_frame_rate;

  m_cam.getExpTime(m_exp_time);
  m_cam.getExposureTimeRange(min_exp_time, max_exp_time);
  m_cam.getFrameRateRange(min_frame_rate, max_frame_rate);
  m_cam.getReadoutTime(m_readout_time);

  m_lat_time = 0;
  m_max_acq_period = 1.0/ min_frame_rate;
  
  m_valid_ranges.min_exp_time = min_exp_time;
  m_valid_ranges.max_exp_time = max_exp_time;

  // Latency is a total elpased time between two exposure, which
  // include the readout time + a delay time.
  // So the minimum latency is equal to the readout time
  m_valid_ranges.min_lat_time = m_readout_time;
  m_valid_ranges.max_lat_time = m_max_acq_period - m_exp_time + m_readout_time;
  
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
  DEB_PARAM() << DEB_VAR1(exp_time);
  m_exp_time = exp_time;

  m_cam.setExpTime(exp_time);
  adjustFrameRate();

  updateValidRanges();
}
void
lima::Andor3::SyncCtrlObj::getExpTime(double& exp_time)
{
  DEB_MEMBER_FUNCT();
  exp_time = m_exp_time;
}

#pragma mark -
#pragma mark Latency time :
void
lima::Andor3::SyncCtrlObj::setLatTime(double  lat_time)
{
  DEB_MEMBER_FUNCT();
  DEB_PARAM() << DEB_VAR1(lat_time);

  m_lat_time = lat_time;
}

void
lima::Andor3::SyncCtrlObj::getLatTime(double& lat_time)
{
  DEB_MEMBER_FUNCT();
  lat_time = m_lat_time;
}

void 
lima::Andor3::SyncCtrlObj::adjustFrameRate()
{
  DEB_MEMBER_FUNCT();

  double min_frame_rate, max_frame_rate;
  double frame_rate;

  // If the exposure_time is less than readout_time the SDK
  // set the frame rate to the maximum which is 1.0/readout_time,
  // so in that case the latency_time is null and cannot be ajusted
  // readout_time can evolve according to ADC speed and  hw roi/bin.

  m_cam.getReadoutTime(m_readout_time);
  m_cam.getFrameRateRange(min_frame_rate, max_frame_rate);

  // update the min latency_time it can change with other hw parameters.
  m_valid_ranges.min_lat_time = m_readout_time;

  m_max_acq_period = 1.0/min_frame_rate;

  // here the latency_time is disabled
  if (m_exp_time < m_readout_time*1.001) {
    m_lat_time = 0;
  } else {
    frame_rate = 1.0/(m_exp_time + m_lat_time - m_readout_time);
    if (frame_rate > max_frame_rate) frame_rate = max_frame_rate;
  
    m_cam.setFrameRate(frame_rate);
    m_valid_ranges.max_lat_time = m_max_acq_period - m_exp_time - m_readout_time;
  }

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

  valid_ranges = m_valid_ranges;


  DEB_RETURN() << DEB_VAR2(valid_ranges.min_exp_time, valid_ranges.max_exp_time);
  DEB_RETURN() << DEB_VAR2(valid_ranges.min_lat_time, valid_ranges.max_lat_time);
}


#pragma mark -
#pragma mark Acquisition possibilities updates :
void
lima::Andor3::SyncCtrlObj::updateValidRanges()
{
  DEB_MEMBER_FUNCT();

  ValidRangesType		the_v_range;
  getValidRanges(the_v_range);
  validRangesChanged(the_v_range);
}

/*
 Local Variables:
 c-file-style: "gnu"
 End:
 */
