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
%End
  public:
    Interface(Andor3::Camera& cam /In/ );
    virtual ~Interface();

    //- From HwInterface
    virtual void    getCapList(CapList& /Out/ ) const;
// Ou ?
//     virtual void	getCapList(std::vector<HwCap> &cap_list /Out/) const;
    virtual void    reset(ResetLevel reset_level /In/ );
    virtual void    prepareAcq();
    virtual void    startAcq();
    virtual void    stopAcq();
    virtual void    getStatus(StatusType& status /Out/ );
    virtual int     getNbHwAcquiredFrames();

  };
};
