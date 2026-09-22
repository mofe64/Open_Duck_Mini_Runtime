# Current robot wiring and readiness

This map is for our manually installed build and comes from the physical checks on 20 September 2026. Pin numbers in code use BCM GPIO numbers, not physical header positions.

| Device | Physical pin | BCM GPIO or bus | Runtime status |
|---|---:|---|---|
| Left foot, normally open | 7 | GPIO4 | Configured as active-low input with pull-up |
| Right foot, normally open | 11 | GPIO17 | Configured as active-low input with pull-up |
| Foot common | 25 | Ground | Checked |
| BNO055 SDA / SCL | 3 / 5 | I²C GPIO2 / GPIO3 | Address 0x29 configured in both IMU readers |
| BNO055 power / ground | 1 / 34 | 3.3 V / ground | Communication checked at 10 kHz |
| Speaker amplifier BCLK / DIN / LRC | 12 / 40 / 35 | GPIO18 / GPIO21 / GPIO19 | Audible test passed; boot overlay survives reboot |
| Speaker amplifier power / ground | 17 / 39 | 3.3 V / ground | Low-volume test only |
| Right ear signal | 8 | GPIO14 | `pigpio` driver prepared; neutral and limits untested |
| Left ear signal | 10 | GPIO15 | `pigpio` driver prepared; neutral and limits untested |
| Ear power / ground | BEC output | Separate supply | Voltage not measured remotely |
| Left / right LED positive | 13 / 15 | GPIO27 / GPIO22 | Both lit separately with 100Ω series resistors; current not measured |
| LED negatives | 9 | Ground | Checked |
| Motor bus adapter | USB | `/dev/ttyACM0` during check | All 14 expected IDs returned positions; movement not tested |

## Before running the robot

1. The 10 kHz I²C setting survives reboot. The BNO055 gave one zero quaternion immediately after mode setup, then 179 valid readings. Gyro and acceleration reads together topped out at about 36.5 Hz, below the walk loop's 50 Hz target. Calibrate and resolve that timing before using it for control.
2. The MAX98357A overlay with `no-sdmode=true` survives reboot. Sound plays, and GPIO4 remains available for the left foot input.
3. Keep `expression_features.antennas` set to `false` in `duck_config.json`. The antenna class now targets GPIO15/14 through the local `pigpio` service. The serial-console boot argument was removed. Check each ear's neutral position and safe limits before enabling it.
4. Both eyes lit separately: GPIO27 is left and GPIO22 is right. The 15-second runtime blink check passed, and `expression_features.eyes` is enabled in the Pi config. Current was not measured; the fitted 100Ω resistors limit it below the LEDs' 1 W rating.
5. Calibrate and verify motor joint assignment, offsets, and direction before commanding motion. All 14 IDs returned positions, which proves communication only. `scripts/check_motors.py` changes gains and includes movement tests, so it is not a read-only diagnostic.

Ear neutral and limits, IMU timing, and motor calibration remain before movement.
