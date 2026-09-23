# Current robot wiring and readiness

This map is for our manually installed build and comes from the physical checks on 20 September 2026. Pin numbers in code use BCM GPIO numbers, not physical header positions.

| Device | Physical pin | BCM GPIO or bus | Runtime status |
|---|---:|---|---|
| Left foot, normally open | 7 | GPIO4 | Released/pressed states checked with both switches |
| Right foot, normally open | 11 | GPIO17 | Released/pressed states checked with both switches |
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
| Motor bus adapter | USB | `/dev/ttyACM0` during check | All 14 joints tracked a supported walk pose test |

## Before running the robot

1. Hardware I²C produced bit-sized gyro and acceleration jumps at default, 10 kHz, and 5 kHz settings. With the hardware controller disabled and `dtoverlay=i2c-gpio,i2c_gpio_sda=2,i2c_gpio_scl=3,bus=8`, the same wiring produced 600 stationary samples with no read errors or suspicious jumps. An isolated 300-pair gyro and acceleration read test achieved 338.6 pairs/s, above the 50 Hz walk target; full-loop timing is still unverified. The IMU readers now default to bus 8 in code. The IMU was recalibrated on bus 8 and gave steady readings with motor power on. Upright, nose-down, and right-side-down checks confirmed the configured upside-down axis mapping.
2. The MAX98357A overlay with `no-sdmode=true` survives reboot. Sound plays, and GPIO4 remains available for the left foot input.
3. The antenna class targets GPIO15/14 through the local `pigpio` service. The serial-console boot argument was removed. Each ear moved and stopped during the 1400–1600 µs test, so the runtime is limited to that range and antennas are enabled in the Pi config.
4. Both eyes lit separately: GPIO27 is left and GPIO22 is right. The 15-second runtime blink check passed, and `expression_features.eyes` is enabled in the Pi config. Current was not measured; the fitted 100Ω resistors limit it below the LEDs' 1 W rating.
5. All 14 motors followed a supported, 20-second move into the walk start pose at gain 8. While holding that pose, leg gains were raised in stages to 30 and head gains stayed at 8. The largest position error fell to about 0.03 rad, the Pi stayed connected, and `vcgencmd get_throttled` stayed at `0x0`. The duck returned to its starting pose. This does not test dynamic walking. `scripts/check_motors.py` changes gains and includes movement tests, so it is not a read-only diagnostic.
6. The Bluetooth controller paired and Linux received its buttons and sticks. Pygame's controller class detected A presses over SSH once background joystick input was enabled. The first policy run reached the starting pose, then the motor serial connection failed shortly after unpausing. The Pi stayed on, but the walk did not complete.

The cause of the motor connection failure is unknown. The walk code now stops when motor feedback is lost and warns if it cannot turn off torque over the failed connection. In that case, support the duck and switch off the shared power supply. The earlier SSH disconnect during `find_soft_offsets.py` has not been explained either. That script uses default offsets and raised all 14 motor gains to 32, unlike the supported test above.

## IMU on software I²C

The Pi currently uses software I²C bus 8 on the existing GPIO2/GPIO3 wires. The runtime defaults to bus 8, so no bus entry is needed in `~/duck_config.json`. Set `"imu_upside_down": true` for this build.

The old `i2c_arm_baudrate=5000` setting belongs to the disabled hardware I²C controller and does not set bus 8's speed. The `i2c-gpio` overlay uses its own default 2 µs clock delay (about 100 kHz); see the [Raspberry Pi overlay reference](https://github.com/raspberrypi/firmware/blob/master/boot/overlays/README).

Run the stationary test from the scripts directory; it loads `imu_calib_data.pkl` when one exists:

```bash
cd ~/Open_Duck_Mini_Runtime/scripts
"$HOME/.venvs/openduck/bin/python" ../mini_bdx_runtime/mini_bdx_runtime/raw_imu.py --upside-down
```

The earlier calibration was captured through the faulty hardware bus with the upright remap and remains in the renamed corrupt checkout. The Pi has since been recalibrated on bus 8 with the upside-down remap. The current `scripts/imu_calib_data.pkl` is loaded when walking starts from the scripts directory. To recalibrate after changing the IMU, run:

```bash
PYTHONPATH="$PWD/../mini_bdx_runtime" "$HOME/.venvs/openduck/bin/python" calibrate_imu.py --upside-down
```

The Pi's boot config backup is `/boot/firmware/config.txt.before-software-i2c` if the software bus needs to be rolled back.
