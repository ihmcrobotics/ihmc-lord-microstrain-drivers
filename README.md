# IHMC LORD MicroStrain Drivers

This project provides integration with LORD MicroStrain VRU/IMU products that use the "MIP" Packet Communication Protocol, an example of which can be found here: https://www.microstrain.com/sites/default/files/3dm-gx5-15_dcp_manual_8500-0067_0.pdf

This release has been tested on the 3DM-GX5-15, but should be useable with many different IMUs from this line.

Currently this project provides a simple Calibration tool for re-zeroing the Gyro bias on the IMU (the MicrostrainCalibrator) and a simple systemd service that starts when an IMU is connected to tty/serial via USB and broadcasts the MIP packet over UDP (MicrostrainMipServer). Configuration of the IMU and packet structure needs to be done using the MIP Monitor tool: https://www.microstrain.com/software#mipMonitorSection

It also provides Java classes for receiving the UDP packets and populating simple Java objects with the data.

## A Note on Filter Modes

The 3DM-GX* series IMUs provide two filters, a "Complimentary Filter" (CF) and a non-linear Kalman filter (EKF). The `MicrostrainMipServer` and provided Java classes support both of these modes without modification but not simultaneously, which it is possible to configure the IMU to use. We only support either only EKF or only CF packets. You can "disable" a mode in MIP Monitor by clearing all fields from the packet builder UI under the tab of the filter you don't want to use.

## `MicrostrainCalibrator` usage

```bash
$ MicrostrainCalibrator -d /dev/<device-id>
```

The MicrostrainCalibrator must be run with permissions needed to read serial devices. The easiest way to do so is to run the command with root priveleges, e.g. using `sudo`. The device ID passed in does not need to be the full udev DEVPATH, you should use the `/dev/ttyACM*` path.

The calibrator initiate the on-board gyro bias capture. The IMU should remain completely still during calibration or the calibration may fail.

## `MicrostrainMipServer` usage

```bash
$ MicrostrainMipServer -h <hostname or ip> -p <port number> -d /dev/<device-id>
```

Use the hostname argument to tell the server which network interface to broadcast on and the port argument to assign which port to send UDP packets to.

## systemd service

The systemd service installs the `microstrain@.service` template unit at `/etc/systemd/system/microstrain@.service` and udev rules at `/etc/udev/rules.d/90-microstrain.rules` that will start a `MicrostrainMipServer` instance under the service name `microstrain@'/dev/<device-id> <short serial number>'`. The last four digits of the short serial number are used to send out to port `5####` on the interface(s) associated with `127.0.0.1`. This is facilitated by a helper script called `start-server.sh` if you need to modify the systemd service behavior.

## Java classes

This project provides two helper classes, a simple data structure class `MicroStrainData.java` and a helper class for receiving UDP packets and packing them in to the data structure `MicroStrainUDPPacketListener.java`. There are helper methods in the UDP Packet Listener class that take in a serial number and determine which port to receive on using the same logic as the systemd setup. Data is packed without modification, and as such uses the same units and references frames as outlined in the MIP Packet docs linked above. There are helper static fields in the `MicroStrainData` class for use with other IHMC software (e.g. transforms and scale factors).