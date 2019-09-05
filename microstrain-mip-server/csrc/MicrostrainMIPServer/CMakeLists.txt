project(IHMCMicroStrainDrivers)

add_executable(MicrostrainMipServer MicrostrainMipServer.cpp)
install(TARGETS MicrostrainMipServer RUNTIME DESTINATION bin)
