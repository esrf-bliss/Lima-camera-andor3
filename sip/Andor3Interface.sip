/* andor3 plugin hardware interface class interface generation
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

namespace Andor3
{

  class Interface : HwInterface
  {
%TypeHeaderCode
#include <Andor3Interface.h>
#include <Andor3Camera.h>
%End
  
  protected:

  public:
    Interface(Andor3::Camera& cam /In,KeepReference/);
    virtual ~Interface();


    //- From HwInterface
    virtual void    getCapList(std::vector<HwCap> &cap_list /Out/) const;
    virtual void    reset(ResetLevel reset_level /In/ );
    virtual void    prepareAcq();
    virtual void    startAcq();
    virtual void    stopAcq();
    virtual void    getStatus(StatusType& status /Out/ );
    virtual int     getNbHwAcquiredFrames();

      // Making the CtAcquisition object to update its cached range for expo. and lat. times
    virtual void 		updateValidRanges();
      
    // Specific to andor-sdk-3... Only a wrapper to the camera equivalent methods
    //   but completed by the update of the CtAcquisition object when changing a setting.
    virtual void		setAdcGain(Andor3::Camera::A3_Gain iGain /In/ );
    virtual void		getAdcGain(Andor3::Camera::A3_Gain &oGain /Out/ ) const;
    virtual void		getAdcGainString(std::string &oGainString /Out/ ) const;
    virtual void		setSimpleGain(std::string iGain /In/ );
    virtual void		getSimpleGain(std::string &oGain /Out/ ) const;
    virtual void		getSimpleGainList(std::vector<std::string> &gain_list /Out/ ) const;
    virtual void		setAdcRate(std::string iRate /In/ );
    virtual void		getAdcRate(std::string &oRate /Out/ ) const;
    virtual void		getAdcRateList(std::vector<std::string> &oRateList /Out/ ) const;
    virtual void		setElectronicShutterMode(Andor3::Camera::A3_ShutterMode iMode /In/ );
    virtual void		getElectronicShutterMode(Andor3::Camera::A3_ShutterMode &oMode /Out/ ) const;
    virtual void		getElectronicShutterModeString(std::string &oModeString /Out/ ) const;

    virtual void setOverlap(bool i_overlap /In/ );
    virtual void getOverlap(bool &o_overlap /Out/ ) const;
    virtual void setSyncTriggering(bool i_sync /In/ );
    virtual void getSyncTriggering(bool &o_sync /Out/ ) const;
  };
};
