project(IHMCMicroStrainDrivers)
cmake_minimum_required(VERSION 2.6)

#IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
#  SET(CMAKE_INSTALL_PREFIX
#    ${CMAKE_SOURCE_DIR} CACHE PATH "ValkyrieHardwareDrivers install prefix" FORCE
#    )
#ENDIF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)

set(CMAKE_BUILD_TYPE RelWithDebInfo)

add_subdirectory(csrc)
add_subdirectory(upstart)
