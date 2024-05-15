# ArduPilot with OpenDroneID based avoidance

This project is a fork of ardupilot that is intended to demonstrate DAA using the Broadcast Remote ID. An on-board reciever for RemoteID signals is used to inform the drone autopilot about the presence of nearby UAV's so that it can inform the pilot, and take evasive action if necessary.

This firmware is intended to be used in combination with [RID](https://github.com/uci-overRID/RID).

## Hardware Setup

A ESP32-S3 or ESP32-C3 board is necessary for this system in addtion to the fllight controller compatible with ardupilot.

[RID](https://github.com/uci-overRID/RID) presents the device it is flashed to as a mavlink serial device and provides the location of other drones to us. The device that runs the RID firmware should be connected to your flight controller over SERIAL.

Flash the firmware to the ESP32 by connecting it to you computer and running the following commands:

```bash
git clone https://github.com/uci-overRID/RID.git
cd RID
git submodule init
git submodule update --init --recursive
pio run -t upload
```

Now you must wire the ESP32 to your one of your flight controller's SERIAL/UART interfacess.

## Parameters

If the [RID](https://github.com/uci-overRID/RID) device is connected on SERIAL1 then you must run the following:

```
param set SERIAL1_PROTOCOL 2
```

Further to enable the avoidance systems in ardupilot you must:

```
param set 
```

# ArduPilot Project

Thank you to the ArduPilot project and all it's contributors for making this possible and providing such an awesome open source system.

For ardupilot documentation, go to their [repository](https://github.com/ArduPilot/ardupilot/) or [the ardupilot website](https://ardupilot.org/)
