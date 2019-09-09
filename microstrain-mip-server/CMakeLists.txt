project(IHMCMicroStrainDrivers)
cmake_minimum_required(VERSION 3.10)

#IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
#  SET(CMAKE_INSTALL_PREFIX
#    ${CMAKE_SOURCE_DIR} CACHE PATH "ValkyrieHardwareDrivers install prefix" FORCE
#    )
#ENDIF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)

if (WIN32 OR APPLE)
    message(FATAL_ERROR "Microstrain Drivers requires linux, exiting.")
endif()

set(CMAKE_BUILD_TYPE RelWithDebInfo)

add_subdirectory(csrc)

find_program(LSB_RELEASE_EXEC lsb_release)
execute_process(COMMAND ${LSB_RELEASE_EXEC} -irs
    OUTPUT_VARIABLE LSB_RELEASE_ID_SHORT
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

string(REPLACE "\n" " " LSB_DETAILS ${LSB_RELEASE_ID_SHORT})
message(STATUS "LSB RELEASE INFO: ${LSB_DETAILS}")

if ("${LSB_DETAILS}" MATCHES "^.*Ubuntu.*18\.04.*$" OR "${LSB_DETAILS}" MATCHES "^.*Ubuntu.*16\.04.*$")
    message(STATUS "Ubuntu with systemd detected")
    SET(INIT_SYSTEMD True CACHE BOOL "Use the systemd init system" FORCE)
    SET(INIT_UPSTART False CACHE BOOL "Use the upstart init system" FORCE)
elseif("${LSB_DETAILS}" MATCHES "^.*Ubuntu.*14\.04.*$")
    message(STATUS "Ubuntu with upstart detected")
    SET(INIT_SYSTEMD False CACHE BOOL "Use the systemd init system" FORCE)
    SET(INIT_UPSTART True CACHE BOOL "Use the upstart init system" FORCE)
endif()

if( INIT_SYSTEM )
    if( ${INIT_SYSTEM} STREQUAL "systemd")
        SET(INIT_SYSTEMD True CACHE BOOL "Use the systemd init system" FORCE)
        SET(INIT_UPSTART False CACHE BOOL "Use the upstart init system" FORCE)
    elseif( ${INIT_SYSTEM} STREQUAL "upstart")
        SET(INIT_SYSTEMD False CACHE BOOL "Use the systemd init system" FORCE)
        SET(INIT_UPSTART True CACHE BOOL "Use the upstart init system" FORCE)
    elseif( ${INIT_SYSTEM} STREQUAL "disable" )
        message(STATUS "Disabling init system install")
        SET(INIT_SYSTEMD False CACHE BOOL "Use the systemd init system" FORCE)
        SET(INIT_UPSTART False CACHE BOOL "Use the upstart init system" FORCE)
    else()
        message(WARNING "Unknown init system: INIT_SYSTEM=${INIT_SYSTEM}")
        SET(INIT_SYSTEMD False CACHE BOOL "Use the systemd init system")
        SET(INIT_UPSTART False CACHE BOOL "Use the upstart init system")
    endif()
endif()

if( INIT_SYSTEMD )
    message(STATUS "Init system set to systemd")
    add_subdirectory(systemd)
elseif( INIT_UPSTART )
    message(STATUS "Init system set to upstart")
    add_subdirectory(upstart)
else()
    message(STATUS "INIT_SYSTEM not set")
endif()
