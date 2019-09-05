cmake_minimum_required(VERSION 2.8)

project(IHMCMicroStrainDrivers)

#find_package(JNI REQUIRED)

include_directories(${JNI_INCLUDE_DIRS})


#add_library(LZO SHARED us_ihmc_valkyrie_util_LZO.cpp)
#target_link_libraries(LZO lzo2)
#INSTALL(TARGETS LZO RUNTIME DESTINATION lib LIBRARY DESTINATION lib)
add_subdirectory("MicrostrainMIPServer")
add_subdirectory("MicrostrainCalibrator")
