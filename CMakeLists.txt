############################################################################
# This file is part of LImA, a Library for Image Acquisition
#
# Copyright (C) : 2009-2017
# European Synchrotron Radiation Facility
# BP 220, Grenoble 38043
# FRANCE
#
# This is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3 of the License, or
# (at your option) any later version.
#
# This software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, see <http://www.gnu.org/licenses/>.
############################################################################
# CMakeLists.txt for Andor3 camera

SET(CMAKE_SHARED_LINKER_FLAGS "-Wl,--add-needed")

SET(andor3_srcs src/Andor3Camera.cpp  src/Andor3Interface.cpp
	 src/Andor3DetInfoCtrlObj.cpp src/Andor3SyncCtrlObj.cpp
	 src/Andor3BinCtrlObj.cpp src/Andor3ReconstructionCtrlObj.cpp
	 src/Andor3RoiCtrlObj.cpp)
	 
FILE(STRINGS "VERSION" andor3_vers)
ADD_LIBRARY(limaandor3 SHARED ${andor3_srcs})

TARGET_INCLUDE_DIRECTORIES(limaandor3 PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
TARGET_INCLUDE_DIRECTORIES(limaandor3 PUBLIC "${COMMON_PATH}/include")

FIND_LIBRARY(ANDOR3_LIB_ATCORE atcore)
FIND_LIBRARY(ANDOR3_LIB_ATUTILITY atutility)
IF (("${ANDOR3_LIB_ATCORE}" STREQUAL "ANDOR3_LIB_ATCORE-NOTFOUND"))
	MESSAGE(FATAL_ERROR "andor3 : libatcore not found, you can search it in cmake-gui or try to install it if it isn't here.")
	ENDIF()
IF (("${ANDOR3_LIB_ATUTILITY}" STREQUAL "ANDOR3_LIB_ATUTILITY-NOTFOUND"))
	MESSAGE(FATAL_ERROR "andor3 : libatutility not found, you can search it in cmake-gui or try to install it if it isn't here.")
ENDIF()
TARGET_LINK_LIBRARIES(limaandor3 limacore)
TARGET_LINK_LIBRARIES(limaandor3 ${ANDOR3_LIB_ATCORE})
TARGET_LINK_LIBRARIES(limaandor3 ${ANDOR3_LIB_ATUTILITY})

SET_TARGET_PROPERTIES(limaandor3 PROPERTIES VERSION ${andor3_vers} SOVERSION "1")

INSTALL(TARGETS limaandor3 LIBRARY DESTINATION lib)

IF(COMPILE_SIP)
    SET(NAME "andor3")
	include(CameraRunSIP)
    INSTALL(FILES python/__init__.py DESTINATION "${PYTHON_SITE_PACKAGES_DIR}/Lima/Andor3")
ENDIF()
