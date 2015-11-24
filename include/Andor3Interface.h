#ifndef ANDOR3INTERFACE_H
#define ANDOR3INTERFACE_H

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

// Camera SDK headers :

// LImA headers :
#include "lima/HwInterface.h"

// Andor3 plugin headers :
#include "Andor3Camera.h"
#include "Andor3DetInfoCtrlObj.h"
#include "Andor3SyncCtrlObj.h"
#include "Andor3RoiCtrlObj.h"
#include "Andor3BinCtrlObj.h"
#include "Andor3ReconstructionCtrlObj.h"

namespace lima
{
  namespace Andor3
  {
    class Interface;
    
    
    /*******************************************************************
     * \class Interface
     * \brief Andor3 hardware interface
     *******************************************************************/
    
    class Interface : public HwInterface
    {
	    DEB_CLASS_NAMESPC(DebModCamera, "Andor3Interface", "Andor3");
      
    public:
	    Interface(Camera& cam);
	    virtual ~Interface();
      
	    //- From HwInterface
	    virtual void    getCapList(CapList& o_cap_list) const;
	    virtual void    reset(ResetLevel reset_level);
	    virtual void    prepareAcq();
	    virtual void    startAcq();
	    virtual void    stopAcq();
	    virtual void    getStatus(StatusType& status);
	    virtual int     getNbHwAcquiredFrames();
	   
      // Making the CtAcquisition object to update its cached range for expo. and lat. times
      virtual void 		updateValidRanges();
      
      // Specific to andor-sdk-3... Only a wrapper to the camera equivalent methods
      //   but completed by the update of the CtAcquisition object when changing a setting.
      virtual void		setAdcGain(Camera::A3_Gain iGain);
      virtual void		getAdcGain(Camera::A3_Gain &oGain) const;
      virtual void		getAdcGainString(std::string &oGainString) const;
      virtual void		setSimpleGain(Camera::A3_SimpleGain iGain);
      virtual void		getSimpleGain(Camera::A3_SimpleGain &oGain) const;
      virtual void		getSimpleGainString(std::string &oGainString) const;
      virtual void		setAdcRate(Camera::A3_ReadOutRate iRate);
      virtual void		getAdcRate(Camera::A3_ReadOutRate &oRate) const;
      virtual void		getAdcRateString(std::string &oRateString) const;
      virtual void		setElectronicShutterMode(Camera::A3_ShutterMode iMode);
      virtual void		getElectronicShutterMode(Camera::A3_ShutterMode &oMode) const;
      virtual void		getElectronicShutterModeString(std::string &oModeString) const;

      virtual void setOverlap(bool i_overlap);
      virtual void getOverlap(bool &o_overlap) const;
      virtual void setSyncTriggering(bool i_sync);
      virtual void getSyncTriggering(bool &o_sync) const;

      // Giving the possibility to get directly the camera object :
      Camera& getCamera()
      { return m_cam; }
        
      
    private:
      Camera&         m_cam;
      CapList         m_cap_list;
      DetInfoCtrlObj  *m_det_info;
      SyncCtrlObj     *m_sync;
      RoiCtrlObj			*m_roi;
      BinCtrlObj			*m_bin;
      ReconstructionCtrlObj *m_reconstruction;
    };
    
    
    
  } // namespace Andor3
} // namespace lima


#endif  /* ANDOR3INTERFACE_H */

/*
 Local Variables:
 c-file-style: "gnu"
 End:
 */
