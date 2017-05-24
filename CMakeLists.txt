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

#FIND_LIBRARY(ANDOR3_LIB_ATCORE atcore ${COMMON_PATH}/lib)
#FIND_LIBRARY(ANDOR3_LIB_ATUTILITY atutility ${COMMON_PATH}/lib)


TARGET_LINK_LIBRARIES(limaandor3 limacore)
TARGET_LINK_LIBRARIES(limaandor3 ${ANDOR3_LIB_ATCORE})
TARGET_LINK_LIBRARIES(limaandor3 ${ANDOR3_LIB_ATUTILITY})

SET_TARGET_PROPERTIES(limaandor3 PROPERTIES VERSION ${andor3_vers} SOVERSION "1")

INSTALL(TARGETS limaandor3 LIBRARY DESTINATION lib)

IF(COMPILE_SIP)
    SET(NAME "andor3")
    SET(INCLUDES)
    FILE(GLOB sipfiles RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/sip" "${CMAKE_CURRENT_SOURCE_DIR}/sip/*.sip")
    FOREACH(sipfile ${sipfiles})
        SET(INCLUDES 
        "${INCLUDES} 
        %Include ${sipfile}")
    ENDFOREACH()

    CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/sip/limamodules.sip.in sip/limaandor3.sip)
    SET(SIP_CONCAT_PARTS 1)
    SET(SIP_INCLUDES ${SIP_INCLUDES}
        "${CMAKE_SOURCE_DIR}/third-party/Processlib/sip"
        "${CMAKE_BINARY_DIR}/sip/core"
        "${CMAKE_SOURCE_DIR}/third-party/Processlib/tasks/sip"
        "${CMAKE_SOURCE_DIR}/common/sip"
        "${CMAKE_SOURCE_DIR}/hardware/sip"
        "${CMAKE_SOURCE_DIR}/control/sip"
        "${CMAKE_SOURCE_DIR}/control/software_operation/sip"
        "${CMAKE_CURRENT_SOURCE_DIR}/sip")
    set(SIP_DISABLE_FEATURES WITH_CONFIG)
    set(SIP_CHECK_EXC ON)
    ADD_SIP_PYTHON_MODULE(limaandor3 ${CMAKE_CURRENT_BINARY_DIR}/sip/limaandor3.sip)
    TARGET_INCLUDE_DIRECTORIES(python_module_limaandor3 PRIVATE
                               ${PYTHON_INCLUDE_DIRS}
                               "${CMAKE_SOURCE_DIR}/sip"
                               "${CMAKE_SOURCE_DIR}/sip/core")
    TARGET_LINK_LIBRARIES(python_module_limaandor3 limaandor3)

    INSTALL(FILES python/__init__.py DESTINATION "${PYTHON_SITE_PACKAGES_DIR}/Lima/Andor3")
ENDIF()
