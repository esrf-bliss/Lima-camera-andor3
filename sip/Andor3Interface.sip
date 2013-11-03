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
  public:
// SIP is not working with the namespace, we have to give the enums and use them directly in here, even the one coming from other parts of LIMA :
    enum A3_Gain { Gain1 = 0, Gain2 = 1, Gain3 = 2, Gain4 = 3, Gain1_Gain3 = 4, Gain1_Gain4 = 5, Gain2_Gain3 = 6, Gain2_Gain4 = 7 };
    enum A3_ShutterMode { Rolling = 0, Global = 1 };
    enum A3_ReadOutRate { MHz10 = 0, MHz100 = 1, MHz200 = 2, MHz280 = 3 };

  public:
    Interface(Andor3::Camera& cam /In/ );
    virtual ~Interface();


    //- From HwInterface
// SIP is not knowing what is CapList ... We have to use the native type rather than the typedef :
//    virtual void    getCapList(CapList& /Out/ ) const;
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
    virtual void		setAdcGain(A3_Gain iGain /In/ );
    virtual void		getAdcGain(A3_Gain &oGain /Out/ ) const;
    virtual void		getAdcGainString(std::string &oGainString /Out/ ) const;
    virtual void		setAdcRate(A3_ReadOutRate iRate /In/ );
    virtual void		getAdcRate(A3_ReadOutRate &oRate /Out/ ) const;
    virtual void		getAdcRateString(std::string &oRateString /Out/ ) const;
    virtual void		setElectronicShutterMode(A3_ShutterMode iMode /In/ );
    virtual void		getElectronicShutterMode(A3_ShutterMode &oMode /Out/ ) const;
    virtual void		getElectronicShutterModeString(std::string &oModeString /Out/ ) const;

  };
};
