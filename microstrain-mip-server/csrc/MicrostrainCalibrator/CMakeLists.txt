cmake_minimum_required(VERSION 3.10)
project(IHMCMicroStrainDrivers VERSION "0.1.0")

add_executable(MicrostrainCalibrator MicrostrainCalibrator.cpp)
target_link_libraries(MicrostrainCalibrator rt)
install(TARGETS MicrostrainCalibrator RUNTIME DESTINATION /usr/local/bin DESTINATION /usr/local/bin)