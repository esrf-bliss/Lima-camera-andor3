#ifndef ANDOR3BINCTRLOBJ_H
#define ANDOR3BINCTRLOBJ_H

/* andor3 plugin binning class
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
     * \class BinCtrlObj
     * \brief Control object providing Andor3 Bin interface
     *******************************************************************/
    class BinCtrlObj : public HwBinCtrlObj
    {
	    DEB_CLASS_NAMESPC(DebModCamera, "BinCtrlObj", "Andor3");
      
	  public:
	    BinCtrlObj(Camera& cam, Interface *interface);
	    virtual ~BinCtrlObj();
	    
	    virtual void getBin(Bin& bin);
	    virtual void checkBin(Bin& bin);
	    virtual void setBin(const Bin& bin);
	  private:
	    Camera& m_cam;
      Interface *m_interface;
      
    };
    
  } // namespace Andor3
} // namespace lima

#endif // ANDOR3BINCTRLOBJ_H
