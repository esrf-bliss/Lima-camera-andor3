/* andor3 plugin detector information class
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
#include "Andor3DetInfoCtrlObj.h"
#include "Andor3Interface.h"


//---------------------------
//- utility variables
//---------------------------


//---------------------------
//- @brief constructor
//---------------------------
lima::Andor3::DetInfoCtrlObj::DetInfoCtrlObj(lima::Andor3::Camera& cam, Interface *interface) :
m_cam(cam), m_interface(interface)
{
  DEB_CONSTRUCTOR();
}

//---------------------------
//- @brief destructor
//---------------------------
lima::Andor3::DetInfoCtrlObj::~DetInfoCtrlObj()
{
  DEB_DESTRUCTOR();
}

void
lima::Andor3::DetInfoCtrlObj::getMaxImageSize(Size& max_image_size)
{
  DEB_MEMBER_FUNCT();
  m_cam.getDetectorImageSize(max_image_size);
}

void
lima::Andor3::DetInfoCtrlObj::getDetectorImageSize(Size& det_image_size)
{
  DEB_MEMBER_FUNCT();
  m_cam.getDetectorImageSize(det_image_size);
}

void
lima::Andor3::DetInfoCtrlObj::getDefImageType(ImageType& def_image_type)
{
  DEB_MEMBER_FUNCT();
  m_cam.getImageType(def_image_type);
}

void
lima::Andor3::DetInfoCtrlObj::getCurrImageType(ImageType& curr_image_type)
{
  DEB_MEMBER_FUNCT();
  m_cam.getImageType(curr_image_type);
}

void
lima::Andor3::DetInfoCtrlObj::setCurrImageType(ImageType  curr_image_type)
{
  DEB_MEMBER_FUNCT();
  m_cam.setImageType(curr_image_type);
  m_interface->updateValidRanges();
}

void
lima::Andor3::DetInfoCtrlObj::getPixelSize(double& xsize, double& ysize)
{
  DEB_MEMBER_FUNCT();
  m_cam.getPixelSize(xsize, ysize);
}

void
lima::Andor3::DetInfoCtrlObj::getDetectorType(std::string& det_type)
{
  DEB_MEMBER_FUNCT();
  m_cam.getDetectorType(det_type);
}

void
lima::Andor3::DetInfoCtrlObj::getDetectorModel(std::string& det_model)
{
  DEB_MEMBER_FUNCT();
  m_cam.getDetectorModel(det_model);
}


void
lima::Andor3::DetInfoCtrlObj::registerMaxImageSizeCallback(HwMaxImageSizeCallback& cb)
{
  DEB_MEMBER_FUNCT();
  m_cam.registerMaxImageSizeCallback(cb);
}

void
lima::Andor3::DetInfoCtrlObj::unregisterMaxImageSizeCallback(HwMaxImageSizeCallback& cb)
{
  DEB_MEMBER_FUNCT();
  m_cam.unregisterMaxImageSizeCallback(cb);
}


/*
 Local Variables:
 c-file-style: "gnu"
 End:
 */
