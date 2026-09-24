# Current robot wiring

This map is for our manually installed build and comes from the physical checks on 20 September 2026. Pin numbers in code use BCM GPIO numbers, not physical header positions.

For changes to the runtime and Pi setup, see [running_changes.md](running_changes.md).

| Device                             | Physical pin | BCM GPIO or bus             | Runtime status                                                       |
| ---------------------------------- | ------------ | --------------------------- | -------------------------------------------------------------------- |
| Left foot, normally open           | 7            | GPIO4                       | Released/pressed states checked with both switches                   |
| Right foot, normally open          | 11           | GPIO17                      | Released/pressed states checked with both switches                   |
| Foot common                        | 25           | Ground                      | Checked                                                              |
| BNO055 SDA / SCL                   | 3 / 5        | I²C GPIO2 / GPIO3           | Address 0x29; software I²C on bus 8 tested clean                     |
| BNO055 power / ground              | 1 / 34       | 3.3 V / ground              | Same wiring produced 600 clean stationary samples on software I²C    |
| Speaker amplifier BCLK / DIN / LRC | 12 / 40 / 35 | GPIO18 / GPIO21 / GPIO19    | Audible test passed; boot overlay survives reboot                    |
| Speaker amplifier power / ground   | 17 / 39      | 3.3 V / ground              | Low-volume test only                                                 |
| Right ear signal                   | 8            | GPIO14                      | `pigpio` control enabled; 1400–1600 µs tested                        |
| Left ear signal                    | 10           | GPIO15                      | `pigpio` control enabled; 1400–1600 µs tested                        |
| Ear power / ground                 | BEC output   | Separate supply             | Voltage not measured remotely                                        |
| Left / right LED positive          | 13 / 15      | GPIO27 / GPIO22             | Both lit separately with 100Ω series resistors; current not measured |
| LED negatives                      | 9            | Ground                      | Checked                                                              |
| Motor bus adapter                  | USB          | `/dev/ttyACM0` during check | All 14 joints tracked a supported walk pose test                     |

The two 18650 cells feed a BMS. The BMS output feeds the servo driver through a switch and also feeds a 5 V BEC for the Pi and other components.

## IMU check and calibration

The BNO055 uses the pins and address in the table above. Set `"imu_upside_down": true` in `~/duck_config.json` because this build mounts the sensor upside down. When launched from the `scripts` directory, the walk loads `imu_calib_data.pkl` if that file is present.

To check the readings while the duck is still, run:

```bash
cd ~/Open_Duck_Mini_Runtime/scripts
"$HOME/.venvs/openduck/bin/python" ../mini_bdx_runtime/mini_bdx_runtime/raw_imu.py --upside-down
```

To recalibrate after changing the IMU, run this from the same `scripts` directory:

```bash
PYTHONPATH="$PWD/../mini_bdx_runtime" "$HOME/.venvs/openduck/bin/python" calibrate_imu.py --upside-down
```
