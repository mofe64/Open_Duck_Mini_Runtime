# Current robot wiring and readiness

This map is for our manually installed build and comes from the physical checks on 20 September 2026. Pin numbers in code use BCM GPIO numbers, not physical header positions.

| Device | Physical pin | BCM GPIO or bus | Runtime status |
|---|---:|---|---|
| Left foot, normally open | 7 | GPIO4 | Configured as active-low input with pull-up |
| Right foot, normally open | 11 | GPIO17 | Configured as active-low input with pull-up |
| Foot common | 25 | Ground | Checked |
| BNO055 SDA / SCL | 3 / 5 | I²C GPIO2 / GPIO3 | Address 0x29 configured in both IMU readers |
| BNO055 power / ground | 1 / 34 | 3.3 V / ground | Communication checked at 10 kHz |
| Speaker amplifier BCLK / DIN / LRC | 12 / 40 / 35 | GPIO18 / GPIO21 / GPIO19 | Audible test passed; boot overlay still needs persistence |
| Speaker amplifier power / ground | 17 / 39 | 3.3 V / ground | Low-volume test only |
| Right ear signal | 8 | GPIO14 | Brief movement test passed; runtime driver not adapted |
| Left ear signal | 10 | GPIO15 | Brief movement test passed; runtime driver not adapted |
| Ear power / ground | BEC output | Separate supply | Voltage not measured remotely |
| LED positive leads | 13 / 15 | GPIO27 / GPIO22 | One 100Ω series resistor per Zerodis 1 W white LED (3.0–3.6 V), reported fitted; both output pins mapped in `Eyes` |
| LED negatives | 9 | Ground | Checked |
| Motor bus adapter | USB | `/dev/ttyACM0` during check | All 14 expected IDs answered ping; movement not tested |

## Before running the robot

1. Persist a 10 kHz I²C bus setting on the Pi. At the default bus speed the BNO055 returned intermittent corrupt status values; a short 20-sample check passed at 10 kHz. Calibrate the IMU and validate orientation and sustained sampling before using it for control.
2. Persist the MAX98357A overlay with `no-sdmode=true`. The overlay's default use of GPIO4 conflicts with the left foot input. The test overlay was loaded only temporarily.
3. Keep `expression_features.antennas` set to `false` in `duck_config.json`. The existing antenna class targets GPIO13/12 with `pwmio`; changing those constants to GPIO15/14 would not establish a suitable 50 Hz servo signal. Select and validate a stable driver, and remove the serial-console boot argument before permanent use of GPIO14/15.
4. Keep `expression_features.eyes` set to `false` until a first low-risk LED test on the Pi confirms voltage, current, and brightness. The reported 100Ω resistor is in series with each 3.0–3.6 V white LED, which is being operated below its 1 W rating. The `Eyes` class now drives GPIO27 and GPIO22 together; left/right eye assignment is still unknown and does not affect simultaneous blinking. Do not assume these GPIO outputs can power the LEDs at their rated 1 W.
5. Calibrate and verify motor joint assignment, offsets, and direction before commanding motion. The 14 successful bus pings prove communication only. `scripts/check_motors.py` changes gains and includes movement tests, so it is not a read-only diagnostic.

The code changes in this branch cover the confirmed sensor and LED pin wiring. Pi boot settings, ear drive, LED validation and side identification, and calibration remain to be completed on the robot.
