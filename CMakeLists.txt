###########################################################################
# This file is part of LImA, a Library for Image Acquisition
#
#  Copyright (C) : 2009-2017
#  European Synchrotron Radiation Facility
#  CS40220 38043 Grenoble Cedex 9 
#  FRANCE
# 
#  Contact: lima@esrf.fr
# 
#  This is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 3 of the License, or
#  (at your option) any later version.
# 
#  This software is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# 
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, see <http://www.gnu.org/licenses/>.
############################################################################

set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--add-needed")

set(andor3_srcs src/Andor3Camera.cpp  src/Andor3Interface.cpp
	 src/Andor3DetInfoCtrlObj.cpp src/Andor3SyncCtrlObj.cpp
	 src/Andor3BinCtrlObj.cpp src/Andor3ReconstructionCtrlObj.cpp
	 src/Andor3RoiCtrlObj.cpp)
	 
file(STRINGS "VERSION" andor3_vers)
add_library(limaandor3 SHARED ${andor3_srcs})

target_include_directories(limaandor3 PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_include_directories(limaandor3 PUBLIC "/usr/local/include")

find_library(ANDOR3_LIB_ATCORE atcore)
find_library(ANDOR3_LIB_ATUTILITY atutility)
if (("${ANDOR3_LIB_ATCORE}" STREQUAL "ANDOR3_LIB_ATCORE-NOTFOUND"))
	message(FATAL_ERROR "andor3 : libatcore not found, you can search it in cmake-gui or try to install it if it isn't here.")
	endif()
if (("${ANDOR3_LIB_ATUTILITY}" STREQUAL "ANDOR3_LIB_ATUTILITY-NOTFOUND"))
	message(FATAL_ERROR "andor3 : libatutility not found, you can search it in cmake-gui or try to install it if it isn't here.")
endif()
target_link_libraries(limaandor3 limacore)
target_link_libraries(limaandor3 ${ANDOR3_LIB_ATCORE})
target_link_libraries(limaandor3 ${ANDOR3_LIB_ATUTILITY})

set_target_properties(limaandor3 PROPERTIES VERSION ${andor3_vers} SOVERSION "1")

install(TARGETS limaandor3 LIBRARY DESTINATION lib)

if(COMPILE_SIP)
    set(NAME "andor3")
	include(CameraRunSIP)
    install(FILES python/__init__.py DESTINATION "${PYTHON_SITE_PACKAGES_DIR}/Lima/Andor3")
endif()
