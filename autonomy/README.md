# Duck depth and commands

Step 2 sends camera depth from the duck to a Mac.
Step 3 sends walking commands from the Mac to the duck.
The duck applies all walking safety rules.
See [Step 3](#step-3-walking-commands) for commands and movement tests.
The camera uses a 2000 mm range.
Both images rotate 180 degrees before other image operations.

UDP sends separate network messages without a delivery guarantee.
A `hello` message asks the duck to send depth images.
The latest valid `hello` selects the viewer address.
The viewer sends one `hello` each second.
The duck stops sending after three seconds without a `hello`.

## Requirements

A venv is a separate Python environment.
Use Python 3.9 or later for these modules.
Use the existing Python 3.12 environment at `~/tof-venv` on the duck.
It contains ArducamDepthCamera 0.1.24 and NumPy 1.26.4.
The Mac viewer uses NumPy 1.26.4 and OpenCV 4.10.0.84.
OpenCV provides the image window and colours.

The duck also needs `v4l2-ctl`, a tool that reads and sets camera formats.
The existing camera setup already contains this tool.
Keep both devices on the same Wi-Fi network.
Allow incoming UDP traffic on port 5601 at the duck.
The streamer accepts IPv4 and IPv6, the two common network address formats.
The viewer uses the address returned for the duck host.

## Prepare the duck

The existing camera test needs a small format helper.
This folder includes its source in `compat/readonly_format_compat.c`.
The helper checks the active sensor format after the driver refuses a format change.
It accepts that error only when the requested and active formats match.
It applies only to the streamer process.
The streamer restores the previous video format when it exits.

1. Connect to the duck.

   ```sh
   ssh mofe@bdxmini.local
   ```

2. Open the runtime folder.

   ```sh
   cd ~/Open_Duck_Mini_Runtime
   ```

3. Build the format helper.

   ```sh
   cc -shared -fPIC -O2 -Wall -Wextra -o autonomy/compat/readonly_format_compat.so autonomy/compat/readonly_format_compat.c -ldl -pthread
   ```

4. Start the streamer.

   ```sh
   LD_PRELOAD="$PWD/autonomy/compat/readonly_format_compat.so" OPENBLAS_NUM_THREADS=1 ~/tof-venv/bin/python -m autonomy.duck_depth_streamer
   ```

5. Press Ctrl+C to stop the streamer.

Do not run another camera program while the streamer runs.
Leave the streamer running for the Mac procedure.

## Prepare the Mac

Use the Mac runtime copy when this folder already exists there.
Use the copy procedure below when the files exist only on the duck.
Run these commands from the Mac runtime folder.

1. Create the viewer environment.

   ```sh
   python3.12 -m venv autonomy/.venv
   ```

2. Install the viewer packages.

   ```sh
   autonomy/.venv/bin/python -m pip install -r autonomy/requirements-mac.txt
   ```

3. Start the viewer.

   ```sh
   autonomy/.venv/bin/python -m autonomy.mac_depth_viewer
   ```

4. Press Q to close the viewer.

The window shows three vertical zones: left, centre, and right.
Each zone shows its 10th percentile distance from valid reduced pixels.
A percentile is a value below which a given percentage of readings falls.
This value is a near-distance estimate, not the smallest reading.
Red means near, and blue means far.
Black means no valid depth.

The window also shows received frames per second and missing frames.
Late and duplicate frames do not replace newer frames.
The missing-frame count uses gaps between frame numbers.
The count excludes time before the first frame and long connection gaps.
The window removes depth readings after one second without a new frame.
The Pi time records frame receipt with a clock that counts seconds since boot.

## Copy files from the duck

Run these steps on the Mac.

1. Create a destination folder.

   ```sh
   mkdir -p ~/duck-depth
   ```

2. Copy the viewer and shared modules.

   ```sh
   scp -r mofe@bdxmini.local:~/Open_Duck_Mini_Runtime/autonomy ~/duck-depth/
   ```

3. Open the destination folder.

   ```sh
   cd ~/duck-depth
   ```

4. Follow the Mac procedure above.

## Options

Run either module with `--help` to see all options.

| Streamer option | Default | Meaning |
| --- | --- | --- |
| `--bind` | `::` | Listen on all IPv4 and IPv6 addresses. |
| `--port` | `5601` | Listen on this UDP port. |
| `--fps` | `10` | Set the camera and send rate. |
| `--min-confidence` | `30` | Reject pixels below this value. |
| `--hello-timeout` | `3` | Stop sending after this many seconds without a hello. |
| `--status-interval` | `5` | Print status at this interval in seconds. |
| `--device` | `0` | Select the camera device number. |

| Viewer option | Default | Meaning |
| --- | --- | --- |
| `--host` | `bdxmini.local` | Select the duck host or address. |
| `--port` | `5601` | Select the duck UDP port. |
| `--scale` | `12` | Enlarge each pixel by this factor. |
| `--headless` | Off | Print status without a window. |
| `--duration` | `0` | Stop after this many seconds; zero removes the time limit. |

## Image and message rules

The streamer rejects non-finite, zero, negative, and over-range depth readings.
It also rejects non-finite confidence values and values below `--min-confidence`.
It reduces each 4 by 4 block to the median valid depth.
It rounds the result to whole millimetres.
An empty block becomes zero.
The result has 60 columns and 45 rows.

This camera does not support the library's direct frame-rate control.
It normally produces 30 depth frames per second.
The streamer disables automatic rate changes and uses the library's frame-skip controls.
At 10 frames per second, it keeps one frame in three before depth processing.
This reduces camera work before image reduction.
The read loop also limits the send rate to `--fps`.

Each depth message contains 5424 bytes.
Each message has one 24-byte header and 5400 depth bytes.
The header contains `ODUK`, version 1, message type, header size, frame number, Pi time, width, and height.
Header numbers and depth values use big-endian byte order, with the highest-value byte first.
Depth values use unsigned 16-bit integers.
Pi time uses a 64-bit floating-point value in seconds.

Each frame uses one UDP message.
The network can split that message into smaller IP packets.
Loss of any part loses the whole frame.
Message types 3 and 4 carry the command and state data described below.
These messages have no sender identity checks or encryption.
Use this version on your local Wi-Fi network.

The status centre distance is the median valid depth in the centre third.
The viewer zone distance uses the 10th percentile instead.
The percentile keeps some response to small near objects while reducing single-pixel errors.
The block median can remove objects smaller than one block.
Obstacle avoidance needs separate tests before walking.

## Tests

Run tests from the runtime folder.

1. Run the hardware-free depth tests on the Mac.

   ```sh
   autonomy/.venv/bin/python -m unittest discover -s tests -p test_autonomy.py -v
   ```

2. Run the hardware-free depth tests on the duck.

   ```sh
   ~/tof-venv/bin/python -m unittest discover -s tests -p test_autonomy.py -v
   ```

3. Check the live stream from the Mac.

   ```sh
   autonomy/.venv/bin/python -m autonomy.mac_depth_viewer --headless --duration 10
   ```

## Depth checked on 16 October 2026

All 46 runtime tests passed on the Mac.
All 21 depth tests passed on the duck.
The Python 3.9 syntax check passed; execution used Python 3.12.
The local UDP check received 101 frames at 10.01 frames per second.
Every checked message contained 5424 bytes and a 45 by 60 depth array.

The Mac window displayed real depth over Wi-Fi with both IPv4 and IPv6.
One 20-second window test received 194 frames and counted five missing frames.
The duck continued at about 10 frames per second.
The window removed old depth after the stream stopped.
Hello expiry and clean camera shutdown passed their checks.

Initial CPU use was about 56% of one core.
Frame skips and faster block processing reduced CPU use.
Fifteen final two-second CPU samples ranged from 28.4% to 39.0%, with a 35.1% average.
The 25% CPU target remains unmet.
Most remaining CPU work runs in the camera library thread.
Operation with the walking policy remains untested.

The camera closed cleanly and restored its previous video format.
Boot configuration, system Python, and the walking script kept their original file hashes.
No test moved the duck.
Measured distance accuracy and long-duration operation remain untested.

## Step 3: walking commands

The command link uses UDP port 5602.
Depth still uses port 5601.
The command link works with IPv4 and IPv6.
The walking environment uses Python 3.11.16 and its existing NumPy package.
Step 3 needs no new packages.

The Mac sender uses the existing `autonomy/.venv` environment.
A gyro measures rotation rate.
An accelerometer measures acceleration, including gravity in these readings.

A watchdog stops auto when commands stop arriving.
The command timeout is 0.5 seconds.
The safety class reserves one control step before that limit.
At 50 control steps per second, that margin is 0.02 seconds.
A delayed control loop can still delay the applied stop.
Check the control budget in movement test 4.

RB toggles auto mode.
Auto needs a recent, valid movement command and centred sticks.
A fall, A, RB, a stop command, or stick travel above 20% leaves auto.
Every exit from auto applies zero walking speeds for that step.
Manual control resumes on later steps.
Auto never resumes without another RB press.

The safety class measures each stick's distance from its centre.
It checks all four raw axes, including both sticks in head-control mode.
It ignores old, repeated, malformed, and non-finite commands.
A rejected command does not reset the watchdog.
A stop remains active for the step even when a newer movement command follows it.
The link leaves auto if incoming traffic exceeds its two-millisecond read budget.

Auto sets all head commands to zero.
It also holds the final head motor targets from the previous step.
Policy output cannot change those targets in auto.
`--freeze-head` holds the start pose in every mode.
Without that flag, startup warns that manual mode can move the head.
Entering and leaving auto plays a 150 ms sound when the speaker is enabled.

The current duck configuration starts paused and disables the speaker.
A press includes a release.
A and RB respond to each new press when autonomy is enabled.
Holding either button does not repeat the mode change.

Recovery and unpause return to manual mode.
Recovery discards queued commands and requires a new command.
Zero walking speed keeps motor torque enabled.
The walking policy can still step on the spot at zero speed.

### Walking flags

| Flag | Default | Meaning |
| --- | --- | --- |
| `--autonomy` | Off | Enable the Mac command link and duck safety checks. |
| `--autonomy-bind` | `::` | Listen on all IPv4 and IPv6 addresses. |
| `--autonomy-port` | `5602` | Select the command port. |
| `--autonomy-forward-cap` | `0.10` | Limit forward and backward speed in metres per second. |
| `--autonomy-sideways-cap` | `0.05` | Limit sideways speed in metres per second. |
| `--autonomy-turn-cap` | `0.6` | Limit turn rate in radians per second. |

The caps apply to Mac commands.
Manual speed limits keep their previous values.
Autonomy requires the controller and fall detection.
It rejects observation replay, which would bypass the selected speeds.
Without `--autonomy`, the walking script imports and calls no autonomy code.

### Mac sender flags

| Flag | Default | Meaning |
| --- | --- | --- |
| `--host` | `bdxmini.local` | Select the duck host or address. |
| `--port` | `5602` | Select the command port. |
| `--forward` | `0.05` | Set forward speed in metres per second. |
| `--sideways` | `0` | Set sideways speed in metres per second. |
| `--turn` | `0` | Set turn rate in radians per second. |
| `--duration` | `0` | Stop sending after this many seconds; zero removes the time limit. |
| `--stop` | Off | Send zero-speed stop commands for one second. |

The sender prints applied speeds, mode, sensor readings, and the last mode change reason.
Ctrl+C stops sending without a final stop command.
Duration expiry has the same behaviour.
The duck watchdog handles both cases.
The sender starts its sequence number from Mac clock time.
This lets a restarted sender use newer numbers.

1. Send explicit stop commands from the Mac runtime folder.

   ```sh
   autonomy/.venv/bin/python -m autonomy.mac_send_command --stop
   ```

### Test without motors

The fake loop imports no motor, sensor, or walking modules.
`--auto-on-command` simulates one RB press.
It does not control real hardware.
Use port 15602 to keep this test separate from the walking port.

1. Open the runtime folder on the Pi.

   ```sh
   cd ~/Open_Duck_Mini_Runtime
   ```

2. Start the fake loop on the Pi.

   ```sh
   OPENBLAS_NUM_THREADS=1 ~/.venvs/openduck/bin/python -m autonomy.fake_walk_loop --port 15602 --duration 20 --auto-on-command
   ```

3. Open the runtime folder on the Mac.

   ```sh
   cd /Users/mofe/dev/openduck/Open_Duck_Mini_Runtime
   ```

4. Send test commands for five seconds.

   ```sh
   autonomy/.venv/bin/python -m autonomy.mac_send_command --port 15602 --duration 5
   ```

5. Check the fake loop's watchdog time and CPU result.

### Movement test 1: stand and watchdog

Perform all movement tests yourself.
Starting the walking script moves joints to the start pose, including when it starts paused.

1. Put the duck on a stand with its legs free.

2. Open the scripts folder on the Pi.

   ```sh
   cd ~/Open_Duck_Mini_Runtime/scripts
   ```

3. Start the walking script on the Pi.

   ```sh
   ~/.venvs/openduck/bin/python -u v2_rl_walk_mujoco.py --onnx_model_path ~/BEST_WALK_ONNX_2.onnx --autonomy --freeze-head
   ```

4. Open the runtime folder on the Mac.

   ```sh
   cd /Users/mofe/dev/openduck/Open_Duck_Mini_Runtime
   ```

5. Start the Mac sender.

   ```sh
   autonomy/.venv/bin/python -m autonomy.mac_send_command --forward 0.05
   ```

6. Press A if the duck starts paused.
7. Press RB to enter auto.
8. Check that the Mac prints `mode=auto` and `applied=(0.050,0.000,0.000)`.
9. Check that the legs perform forward walking.
10. Press Ctrl+C in the Mac sender terminal.
11. Check that the Pi prints the command timeout reason.
12. Check that forward walking stops within 0.5 seconds.

The policy can keep stepping on the spot after the command reaches zero.
This test checks whether the walking command reaches zero.

### Movement test 2: stick, A, and RB exits

Keep the duck on the stand with its legs free.

1. Start the Mac sender again.

   ```sh
   autonomy/.venv/bin/python -m autonomy.mac_send_command --forward 0.05
   ```

2. Press RB to enter auto.
3. Move the left stick beyond 20% of its travel.
4. Check that the Pi prints `Controller stick override.`
5. Centre both sticks.
6. Repeat steps 2 to 5 with the right stick.
7. Press RB to enter auto.
8. Press A to pause walking.
9. Check that the Mac prints `mode=paused` and zero applied speeds.
10. Press A to return to manual mode.
11. Press RB to enter auto.
12. Press RB to leave auto.
13. Check that the Pi prints `RB selected manual mode.`
14. Press Ctrl+C in the Mac sender terminal.
15. Press Ctrl+C in the Pi walking terminal.

### Movement test 3: measure speed on the floor

At the requested 0.05 m/s, 2 m corresponds to 40 seconds.
The actual speed depends on the walking policy and floor.

1. Mark a straight two-metre path on the floor.
2. Put the duck at the start mark.
3. Hold the controller.
4. Start the walking script from the Pi scripts folder.

   ```sh
   ~/.venvs/openduck/bin/python -u v2_rl_walk_mujoco.py --onnx_model_path ~/BEST_WALK_ONNX_2.onnx --autonomy --freeze-head
   ```

5. Start the Mac sender from the Mac runtime folder.

   ```sh
   autonomy/.venv/bin/python -m autonomy.mac_send_command --forward 0.05 --duration 60
   ```

6. Press A if the duck starts paused.
7. Start a stopwatch.
8. Press RB to enter auto.
9. Press RB when the duck reaches the finish mark.
10. Stop the stopwatch.
11. Record the elapsed seconds.
12. Calculate actual speed as two divided by the elapsed seconds.
13. Press Ctrl+C in the Mac sender terminal.
14. Press Ctrl+C in the Pi walking terminal.

### Movement test 4: depth and walking together

Use the same floor setup and keep the controller in hand.
The previous depth test used an average of 35.1% of one CPU core.
This combined movement test remains untested.

1. Open the runtime folder in a second Pi terminal.

   ```sh
   cd ~/Open_Duck_Mini_Runtime
   ```

2. Start the depth streamer in that terminal.

   ```sh
   LD_PRELOAD="$PWD/autonomy/compat/readonly_format_compat.so" OPENBLAS_NUM_THREADS=1 ~/tof-venv/bin/python -m autonomy.duck_depth_streamer
   ```

3. Start walking with output recording from the Pi scripts folder.

   ```sh
   ~/.venvs/openduck/bin/python -u v2_rl_walk_mujoco.py --onnx_model_path ~/BEST_WALK_ONNX_2.onnx --autonomy --freeze-head 2>&1 | tee /tmp/openduck-step3-walk.log
   ```

4. Open the runtime folder in a second Mac terminal.

   ```sh
   cd /Users/mofe/dev/openduck/Open_Duck_Mini_Runtime
   ```

5. Open the depth window in that terminal.

   ```sh
   autonomy/.venv/bin/python -m autonomy.mac_depth_viewer
   ```

6. Start the command sender in the first Mac terminal.

   ```sh
   autonomy/.venv/bin/python -m autonomy.mac_send_command --forward 0.05 --duration 60
   ```

7. Press A if the duck starts paused.
8. Press RB to enter auto.
9. Check for `Policy control budget exceeded` messages during the test.
10. Press A to pause after the test.
11. Press Ctrl+C in the Pi walking terminal.
12. Count the budget messages on the Pi.

   ```sh
   grep -c 'Policy control budget exceeded' /tmp/openduck-step3-walk.log
   ```

13. Record the count and test duration.
14. Press Ctrl+C in the depth streamer terminal.
15. Press Q in the depth viewer window.

### Command message format

A sequence number is a counter used to reject old or repeated messages.
Commands use a 64-bit sequence number, three 32-bit speeds, and a one-byte stop flag.
Each command contains 29 bytes, including the common header.
State uses its own 64-bit sequence number, mode, nine 32-bit values, and a short reason.
The nine values contain applied speeds, gyro readings, and accelerometer readings.
Gyro values use radians per second; accelerometer values use metres per second squared.

State reasons use UTF-8 text, a common text encoding, with a 96-byte limit.
The state header contains 54 bytes, including the reason length.
All message numbers use the same byte order as depth messages.
State reports the speeds selected for policy input.
Movement test 3 measures actual body speed.

### Run all hardware-free tests

1. Run the full suite from the Mac runtime folder.

   ```sh
   .venv/bin/python -m unittest discover -s tests -v
   ```

2. Run the full suite from the Pi runtime folder.

   ```sh
   ~/.venvs/openduck/bin/python -m unittest discover -s tests -v
   ```

### Commands checked on 16 October 2026

All 93 hardware-free tests passed on the Mac and Pi.
The Pi tests used the walking environment's Python 3.11.16 and NumPy 1.26.4.
The shared protocol needed no new packages.
The Pi source now includes the Mac's existing fall checks and fall tests.

The fake loop held 50 control steps per second.
Measured watchdog stops occurred 0.480 to 0.500 seconds after the last accepted command.
The 25-second watchdog run used an average of 1.27% of one CPU core.
Its longest control step took 1.21 ms, with no control overruns.
`top` showed approximately 1% to 2% CPU use during the first network test.

Commands above all three caps returned the correct limited speeds.
The Mac sender worked through `bdxmini.local` and the duck's IPv4 address.
Ctrl+C sent no stop message, and the watchdog left auto.
The separate stop test left auto with reason `Mac stop command.`
It also rejected 27 older movement messages after the newer stop messages.

No test started the walking program or moved a motor.
Head holding, button handling, fall handling, and sound duration passed mocked tests.
Real movement, measured speed, audible feedback, and combined depth operation remain untested.
