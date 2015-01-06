#ifndef ANDOR3ROICTRLOBJ_H
#define ANDOR3ROICTRLOBJ_H

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

#include "lima/HwInterface.h"
#include "Andor3Camera.h"


namespace lima
{
  namespace Andor3
  {
    
    class Interface;
    
    /*******************************************************************
     * \class RoiCtrlObj
     * \brief Control object providing Andor3 Roi interface
     *******************************************************************/
    
    class RoiCtrlObj : public HwRoiCtrlObj
    {
	    DEB_CLASS_NAMESPC(DebModCamera, "RoiCtrlObj", "Andor3");
      
    public:
	    RoiCtrlObj(Camera& cam, Interface *interface);
	    virtual ~RoiCtrlObj();
      
	    virtual void getRoi(Roi& hw_roi);
	    virtual void checkRoi(const Roi& set_roi, Roi& hw_roi);
	    virtual void setRoi(const Roi& set_roi);
      
    private:
	    Camera& m_cam;
      Interface *m_interface;
    };
    
    
  } // namespace Andor3
} // namespace lima

#endif // ANDOR3ROICTRLOBJ_H

