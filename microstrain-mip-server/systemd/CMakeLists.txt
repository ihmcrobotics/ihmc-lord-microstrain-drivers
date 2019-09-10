project(IHMCMicroStrainDrivers)

install(FILES microstrain@.service DESTINATION /etc/systemd/system)
install(FILES start-server.sh DESTINATION /usr/local/bin)
install(FILES 90-microstrain.rules DESTINATION /etc/udev/rules.d)

set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA postinst)
