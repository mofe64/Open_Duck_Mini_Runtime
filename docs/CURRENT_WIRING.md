# Current robot wiring and readiness

This map is for our manually installed build and comes from the physical checks on 20 September 2026. Pin numbers in code use BCM GPIO numbers, not physical header positions.

| Device | Physical pin | BCM GPIO or bus | Runtime status |
|---|---:|---|---|
| Left foot, normally open | 7 | GPIO4 | Configured as active-low input with pull-up |
| Right foot, normally open | 11 | GPIO17 | Configured as active-low input with pull-up |
| Foot common | 25 | Ground | Checked |
| BNO055 SDA / SCL | 3 / 5 | I²C GPIO2 / GPIO3 | Address 0x29; software I²C on bus 8 tested clean |
| BNO055 power / ground | 1 / 34 | 3.3 V / ground | Same wiring produced 600 clean stationary samples on software I²C |
| Speaker amplifier BCLK / DIN / LRC | 12 / 40 / 35 | GPIO18 / GPIO21 / GPIO19 | Audible test passed; boot overlay survives reboot |
| Speaker amplifier power / ground | 17 / 39 | 3.3 V / ground | Low-volume test only |
| Right ear signal | 8 | GPIO14 | `pigpio` control enabled; 1400–1600 µs tested |
| Left ear signal | 10 | GPIO15 | `pigpio` control enabled; 1400–1600 µs tested |
| Ear power / ground | BEC output | Separate supply | Voltage not measured remotely |
| Left / right LED positive | 13 / 15 | GPIO27 / GPIO22 | Both lit separately with 100Ω series resistors; current not measured |
| LED negatives | 9 | Ground | Checked |
| Motor bus adapter | USB | `/dev/ttyACM0` during check | All 14 expected IDs returned positions; movement not tested |

## Before running the robot

1. Hardware I²C produced bit-sized gyro and acceleration jumps at default, 10 kHz, and 5 kHz settings. With the hardware controller disabled and `dtoverlay=i2c-gpio,i2c_gpio_sda=2,i2c_gpio_scl=3,bus=8`, the same wiring produced 600 stationary samples with no read errors or suspicious jumps. An isolated 300-pair gyro and acceleration read test achieved 338.6 pairs/s, above the 50 Hz walk target; full-loop timing is still unverified. The updated runtime then produced 252 raw output samples without the earlier large spikes after startup. The IMU readers now default to bus 8 in code. IMU calibration and the correct upside-down axis response still need verification on bus 8.
2. The MAX98357A overlay with `no-sdmode=true` survives reboot. Sound plays, and GPIO4 remains available for the left foot input.
3. The antenna class targets GPIO15/14 through the local `pigpio` service. The serial-console boot argument was removed. Each ear moved and stopped during the 1400–1600 µs test, so the runtime is limited to that range and antennas are enabled in the Pi config.
4. Both eyes lit separately: GPIO27 is left and GPIO22 is right. The 15-second runtime blink check passed, and `expression_features.eyes` is enabled in the Pi config. Current was not measured; the fitted 100Ω resistors limit it below the LEDs' 1 W rating.
5. Calibrate and verify motor joint assignment, offsets, and direction before commanding motion. All 14 IDs returned positions, which proves communication only. `scripts/check_motors.py` changes gains and includes movement tests, so it is not a read-only diagnostic.

IMU calibration, axis checks, and power stability remain before walking.

## IMU on software I²C

The Pi currently uses software I²C bus 8 on the existing GPIO2/GPIO3 wires. The runtime defaults to bus 8, so no bus entry is needed in `~/duck_config.json`. Set `"imu_upside_down": true` for this build.

The old `i2c_arm_baudrate=5000` setting belongs to the disabled hardware I²C controller and does not set bus 8's speed. The `i2c-gpio` overlay uses its own default 2 µs clock delay (about 100 kHz); see the [Raspberry Pi overlay reference](https://github.com/raspberrypi/firmware/blob/master/boot/overlays/README).

Run the stationary test from the scripts directory; it loads `imu_calib_data.pkl` when one exists:

```bash
cd ~/Open_Duck_Mini_Runtime/scripts
"$HOME/.venvs/openduck/bin/python" ../mini_bdx_runtime/mini_bdx_runtime/raw_imu.py --upside-down
```

The earlier calibration was captured through the faulty hardware bus with the upright remap. It remains in the renamed corrupt checkout after the repository recovery. The fresh checkout has no calibration file, so recalibrate on bus 8 with the upside-down remap before walking:

```bash
PYTHONPATH="$PWD/../mini_bdx_runtime" "$HOME/.venvs/openduck/bin/python" calibrate_imu.py --upside-down
```

The Pi's boot config backup is `/boot/firmware/config.txt.before-software-i2c` if the software bus needs to be rolled back.
