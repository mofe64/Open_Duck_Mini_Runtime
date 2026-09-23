# Changes from upstream

1. **Wiring:** Use GPIO4 and GPIO17 for the left and right foot switches, GPIO27 for the left eye, GPIO22 for the right eye, and I²C address `0x29` for the BNO055. These match our robot's wiring; upstream uses different pins and the default IMU address.
2. **IMU:** Use bus 8 for the motion sensor by default. The old bus sometimes gave sudden false readings, even when slowed down. Bus 8 gave 600 steady readings and was fast enough in a separate speed test. The code no longer needs a bus setting in `duck_config.json`; the old 5 kHz setting does not affect bus 8. We still need to recalibrate the sensor and check its directions before walking.
3. **Pi install:** Add `RPi.GPIO` for ARM Linux. The IMU and foot switch code needs it, but the package install did not include it.
4. **Eyes:** Enable automatic blinking in the Pi config after both eyes lit separately and the runtime blink test ran for 15 seconds. The 100Ω resistors remain in series; current was not measured.
5. **Ears:** Use `pigpio` for the left ear on GPIO15 and right ear on GPIO14. The old code used GPIO13/12 and rounded pulse duty cycle too coarsely. The Pi runs `pigpiod` with the PWM clock so I²S sound can keep using PCM. Limit commands to 1400–1600 µs, the range tested on both ears, and enable them in the Pi config.
