# Changes from upstream

1. **Wiring:** Use GPIO4 and GPIO17 for the left and right foot switches, GPIO27 for the left eye, GPIO22 for the right eye, and I²C address `0x29` for the BNO055. These match our robot's wiring; upstream uses different pins and the default IMU address.
2. **I²C speed:** Set the Pi's I²C bus to **10 kHz**. At the default speed, the BNO055 sometimes returned bad readings. The Pi reports 10 kHz after reboot. Gyro and acceleration reads now work, but together reach only about 36.5 Hz, below the walk loop's 50 Hz target. This is a Pi setting, not a Python change.
3. **Pi install:** Add `RPi.GPIO` for ARM Linux. The IMU and foot switch code needs it, but the package install did not include it.
4. **Eyes:** Enable automatic blinking in the Pi config after both eyes lit separately and the runtime blink test ran for 15 seconds. The 100Ω resistors remain in series; current was not measured.
5. **Ears:** Prepare `pigpio` control for the left ear on GPIO15 and right ear on GPIO14. The old code used GPIO13/12 and rounded pulse duty cycle too coarsely. The Pi runs `pigpiod` with the PWM clock so I²S sound can keep using PCM. Ears remain disabled until neutral positions and limits are checked.
