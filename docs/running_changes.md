# Changes from upstream

1. **Wiring:** Use GPIO4 and GPIO17 for the left and right foot switches, GPIO27 and GPIO22 for the eyes, and I²C address `0x29` for the BNO055. These match our robot's wiring; upstream uses different pins and the default IMU address.
2. **I²C speed:** Set the Pi's I²C bus to **10 kHz**. At the default speed, the BNO055 sometimes returned bad readings. The Pi reports 10 kHz after reboot, and 20 chip ID reads at `0x29` passed. This is a Pi setting, not a Python change.
3. **Pi install:** Add `RPi.GPIO` for ARM Linux. The IMU and foot switch code needs it, but the package install did not include it.
