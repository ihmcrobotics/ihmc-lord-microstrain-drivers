cmake_minimum_required(VERSION 3.10)
project(IHMCMicroStrainDrivers VERSION "0.1.0")

install(FILES microstrain@.service DESTINATION /etc/systemd/system)
install(FILES start-server.sh DESTINATION /usr/local/bin)
install(FILES 90-microstrain.rules DESTINATION /etc/udev/rules.d)

set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA postinst)