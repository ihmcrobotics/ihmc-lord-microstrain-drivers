project(IHMCMicroStrainDrivers)

add_executable(MicrostrainCalibrator MicrostrainCalibrator.cpp)
target_link_libraries(MicrostrainCalibrator rt)
install(TARGETS MicrostrainCalibrator RUNTIME DESTINATION /usr/local/bin DESTINATION /usr/local/bin)
