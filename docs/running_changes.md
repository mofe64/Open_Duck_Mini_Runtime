# Changes from upstream for this duck

This file explains what we changed in the runtime, Pi setup, and hardware, and why. For the physical pin map, see [CURRENT_WIRING.md](CURRENT_WIRING.md).

## Hardware and Pi setup

- **Battery:** We replaced the earlier 18650 cells with a two-cell Molicel P28A pack. The old cells could not supply enough current during walking, so the Pi shut down. With the P28A pack charged to 8.4 V, the duck walked for almost a minute without a Pi shutdown.
- **IMU bus:** We kept the BNO055 on GPIO2 and GPIO3, enabled `dtoverlay=i2c-gpio,i2c_gpio_sda=2,i2c_gpio_scl=3,bus=8` on the Pi, and made the runtime use bus 8 by default. The hardware I²C bus gave unstable sensor readings; the software bus gave steady readings on the same wires.
- **IMU orientation:** We set `"imu_upside_down": true` in the Pi's `duck_config.json` and recalibrated the sensor. This build mounts the IMU upside down, so the runtime must interpret its axes accordingly.
- **Speaker overlay:** We use `no-sdmode=true` with the MAX98357A sound overlay. This lets the speaker work while GPIO4 remains available for the left foot switch.
- **Foot switches, eyes, and IMU address:** We changed the runtime to use GPIO4 and GPIO17 for the feet, GPIO27 and GPIO22 for the eyes, and address `0x29` for the BNO055. These match this duck's wiring. We enabled automatic eye blinking in its Pi config after connecting the eyes through 100Ω series resistors.
- **Ears:** We moved the ear signals to GPIO15 and GPIO14 and removed the serial console from those pins. The runtime uses `pigpio` so the ears can move while I²S sound is active. We limited ear commands to 1400–1600 µs because that is the range checked on this build.
- **Pi dependency:** We added `RPi.GPIO` to the ARM Linux install requirements so the GPIO code needed by the IMU and foot switches is installed with the runtime.

## Motors and walking

- **Motor IDs and offsets:** We changed the joint-to-ID map to match this duck's swapped leg wiring: left leg IDs 10–14 and right leg IDs 20–24. We set this duck's soft offsets in `~/duck_config.json` so commanded positions match its assembly.
- **Walking startup:** We changed startup to move from the joints' current positions into the walking pose over 20 seconds, then raise motor gains in stages. The old startup moved quickly and raised all gains at once. By default, the legs use gain 30 during walking and the head uses gain 8.
- **Stopping the walk:** We changed the runtime to stop if a motor position or speed read fails during walking, or a position read fails while paused. It turns off motor torque on Ctrl+C or another stop, and warns the operator if the motor bus prevents torque from being disabled. This prevents the old loop from retrying failed reads continuously.
- **Controller over SSH:** We enabled background joystick events before Pygame starts. Without this, the controller paired with the Pi but button presses did not reach the walk when it ran over SSH.
- **Optional fixed head:** We added `--freeze-head` to hold the neck and head at the walking start pose during the walk. Startup still moves those joints into place, and their servos stay powered to hold the pose.
