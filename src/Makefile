# andor3 plugin Makefile
# Copyright (C) 2013 IPANEMA USR3461, CNRS/MCC.
# Written by Serge Cohen <serge.cohen@synchrotron-soleil.fr>
#
# This file is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 3 of
# the License, or (at your option) any later version.
#
# This file is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Public License for more details.
#
# You should have received a copy of the GNU General Public
# License along with this file. If not, see <http://www.gnu.org/licenses/>.



andor3-objs = Andor3Camera.o Andor3Interface.o Andor3DetInfoCtrlObj.o Andor3SyncCtrlObj.o Andor3RoiCtrlObj.o Andor3BinCtrlObj.o

SRCS = $(andor3-objs:.o=.cpp) 

CXXFLAGS += -I../include -I../../../hardware/include -I../../../common/include \
			-I/usr/local/include \
			-Wall -pthread -fPIC -g

all:	Andor3.o

Andor3.o:	$(andor3-objs)
	$(LD) -o $@ -r $+

clean:
	rm -f *.o *.P

%.o : %.cpp
	$(COMPILE.cpp) -MD $(CXXFLAGS) -o $@ $<
	@cp $*.d $*.P; \
	sed -e 's/#.*//' -e 's/^[^:]*: *//' -e 's/ *\\$$//' \
	-e '/^$$/ d' -e 's/$$/ :/' < $*.d >> $*.P; \
	rm -f $*.d

-include $(SRCS:.cpp=.P)
