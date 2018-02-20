cmake_minimum_required(VERSION 2.8.9)

project (telephony)

# Supported configurations
set(CMAKE_CONFIGURATION_TYPES
    "Debug"
    "Release"
  )
set(CMAKE_CONFIGURATION_TYPES ${CMAKE_CONFIGURATION_TYPES} CACHE STRING "Available build configurations." FORCE)

if (RIL_FOR_MDM_LE)
    add_definitions(-DRIL_FOR_MDM_LE)
endif (RIL_FOR_MDM_LE)

#Conditionally enable GCOV coverage: default OFF
option( GCOV_COVERAGE OFF )
set( GCOV_COVERAGE OFF )
if (GCOV_COVERAGE)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -fprofile-arcs -ftest-coverage")
Set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -O0 -fprofile-arcs -ftest-coverage")
endif (GCOV_COVERAGE)

# find subdirectories with CMakeLists.txt and add them to be built.
MACRO(add_CMakeDirs)
    FILE(GLOB subdirlist *)
    FOREACH(subdir ${subdirlist})
        IF(IS_DIRECTORY ${subdir} AND EXISTS ${subdir}/CMakeLists.txt)
            add_subdirectory( ${subdir} )
        ENDIF()
    ENDFOREACH()
ENDMACRO()

add_CMakeDirs()

# <--------------------------------------------->
# adding RPM generation
# <--------------------------------------------->
include (InstallRequiredSystemLibraries)
set(CPACK_PACKAGE_VERSION "1.0")
set(CPACK_GENERATOR "RPM")
set(CPACK_PACKAGE_RELEASE 1)
set(CPACK_PACKAGE_CONTACT "Automotive BU")
set(CPACK_PACKAGE_VENDOR "QTI Inc.")
set(CPACK_PACKAGING_INSTALL_PREFIX "/usr")
SET(CPACK_SOURCE_GENERATOR "TGZ")
SET(CPACK_SOURCE_IGNORE_FILES "/raw/" "/\\\\.git/")
include(CPack)

