set(ANDOR3_INCLUDE_DIRS)
set(ANDOR3_LIBRARIES)
set(ANDOR3_DEFINITIONS)

find_path(ANDOR3_INCLUDE_DIRS "atcore.h")
find_library(ANDOR3_ATCORE_LIB atcore)
find_library(ANDOR3_ATUTILITY_LIB atutility)
list(APPEND ANDOR3_LIBRARIES ${ANDOR3_ATCORE_LIB} ${ANDOR3_ATUTILITY_LIB})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Andor3 DEFAULT_MSG
  ANDOR3_LIBRARIES
  ANDOR3_INCLUDE_DIRS
)
