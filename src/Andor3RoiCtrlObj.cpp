/* andor3 plugin ROI class
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

#include "Andor3RoiCtrlObj.h"

#include "Andor3Interface.h"

using namespace lima::Andor3;

//-----------------------------------------------------
// @brief Ctor
//-----------------------------------------------------
lima::Andor3::RoiCtrlObj::RoiCtrlObj(Camera& cam, Interface *interface)
: m_cam(cam), m_interface(interface)
{
  DEB_CONSTRUCTOR();    
}

//-----------------------------------------------------
// @brief Dtor
//-----------------------------------------------------
lima::Andor3::RoiCtrlObj::~RoiCtrlObj()
{
  DEB_DESTRUCTOR();
}

//-----------------------------------------------------
// @brief
//-----------------------------------------------------
void
lima::Andor3::RoiCtrlObj::getRoi(Roi& roi)
{
  DEB_MEMBER_FUNCT();
  m_cam.getRoi(roi);
}

//-----------------------------------------------------
// @brief
//-----------------------------------------------------
void
lima::Andor3::RoiCtrlObj::checkRoi(const Roi& set_roi, Roi& hw_roi)
{
  DEB_MEMBER_FUNCT();
  m_cam.checkRoi(set_roi, hw_roi);
}

//-----------------------------------------------------
// @brief
//-----------------------------------------------------
void
lima::Andor3::RoiCtrlObj::setRoi(const Roi& roi)
{
  DEB_MEMBER_FUNCT();
  Roi real_roi;
  checkRoi(roi,real_roi);
  m_cam.setRoi(real_roi);
  m_interface->updateValidRanges();
}

