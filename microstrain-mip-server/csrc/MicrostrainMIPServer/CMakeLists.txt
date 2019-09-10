cmake_minimum_required(VERSION 3.10)
project(IHMCMicroStrainDrivers VERSION "0.1.0")

add_executable(MicrostrainMipServer MicrostrainMipServer.cpp)
install(TARGETS MicrostrainMipServer RUNTIME DESTINATION /usr/local/bin DESTINATION /usr/local/bin)