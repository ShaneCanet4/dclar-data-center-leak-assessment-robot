# DCLAR — Data Center Leak Assessment Robot

DCLAR is a Python prototype for planning a small robot's search through a data
center floor. It models rack rows as obstacles, chooses a reachable floor tile,
generates a coverage pattern inside that tile, and records pivot-and-drive
waypoints in map coordinates. The Tkinter window is a visualization of the
planner; it is not a robot controller or a leak sensor.

## Main systems

| System | File | Responsibility |
| --- | --- | --- |
| Map routing | `route.py` | Builds the rack map and uses breadth-first search (BFS) to find shortest four-direction paths around blocked tiles. |
| Tile coverage | `search.py` | Creates zigzag or circle-spiral waypoints inside and beyond a selected 1 ft tile, using the 7 in × 5 in robot footprint. |
| Simulation and data | `main.py` | Requires an explicit Simulation or Robot hardware run mode, accepts spill locations, shows the current command vector, and saves a JSON plan. |
| USB serial link | `serial_link.py`, `serial_protocol.py` | Lists COM ports, connects the desktop app to the main-control ESP32, and formats text commands. |
| ESP32 radio bridge and robot receiver | `esp32/` | Main-control USB/TFT bridge, robot ESP-NOW receiver, bounded motor jogs, and a manifest/segment/tile-completion protocol for calibrated timed paths. |
| Communication checks | `test_serial_protocol.py`, `test_robot_motion.py`, `test_esp32_communication.py` | Unit-checks existing message helpers and geometry calculations, and provides a USB PING/PONG check. The tile transaction still needs end-to-end hardware validation. |
| Living system architecture | `docs/system_design.tex` | Detailed architecture and data-flow description, current feature status, student design ownership, AI-assisted implementation contributions, and hardware limitations. Update this document alongside meaningful feature changes. |

Project-owned MicroPython source files are grouped in `esp32/`. When deploying,
copy the selected board's modules from that folder to the ESP32 device root;
the board does not use an `esp32/` subfolder.

## ESP32 main-control bridge

`esp32/esp32_main_control.py` is MicroPython firmware for the main-control ESP32.
The desktop `main.py` starts paused and asks the operator to select a run mode.
**Simulation** animates the map on the computer. **Robot hardware (ESP-NOW)**
requires a matching USB PING/PONG and robot radio PONG before declaring a tile
path. The desktop sends a tile manifest, streams one segment at a time, and
marks the tile searched only after the matching robot `TILE_DONE` confirmation.
The jog panel's local motor mode sends short jog commands to the main-control Feather. The third run mode, **Control-board test (local)**, exercises a complete one-tile UI transaction on that same board without the robot ESP32 or ESP-NOW. After USB PING/PONG, the app requests `FAKE_ROBOT|ON`, waits for `MODE|FAKE_ROBOT|ON`, sends `MISSION_START` and one `SEG` at a time, then waits for `TILE_DONE` before marking that tile searched. The board executes each segment as short 30% duty motor pulses and returns `MISSION_READY`, `RECEIVED`, `DONE`, and `TILE_DONE` over USB. The app then sends `FAKE_ROBOT|OFF` and waits for its acknowledgement before reporting radio-bridge mode. The serial activity log shows each request and reply.

This is a bounded signal and UI integration test, not calibrated travel: each turn/drive segment is capped at 500 ms, and a timer can confirm completion even if a motor only buzzes or stalls. The map advances on the matching final token by design; visually inspect the motors separately. This mode runs only one tile per app session. For this mode, copy `esp32/main_control_motor_driver.py` and `esp32/motor_test_protocol.py` to the main-control board root in addition to the normal bridge and display modules.

### Configure and run

1. Install MicroPython on the main-control ESP32 and connect it to the computer
   with a USB data cable.
2. In Thonny, select **MicroPython (ESP32)** and the board's COM port. Copy
   `esp32/esp32_main_control.py` and `esp32/board_display.py` to the MicroPython device.
   `board_display.py` uses the same display driver and font as the known-working
   board test: download `st7789py.py` and `vga1_16x32.py` from the upstream
   links below and save them in the local `esp32/` folder, then copy them to
   the device root.
   They are in the upstream [ST7789 MicroPython driver](https://github.com/russhughes/st7789py_mpy/tree/master/lib)
   and its [ROM font folder](https://github.com/russhughes/st7789py_mpy/tree/master/romfonts).
   If the board was erased while flashing MicroPython, copy all four files again.
   Save these modules at the device root in Thonny; the `esp32/` prefix is only
   the folder name on the computer. Save `esp32_main_control.py` on the device
   as **`main.py`** so MicroPython starts the bridge after reset. If it is only
   run from Thonny, the bridge stops when the session is interrupted and the
   board returns to the `>>>` Python prompt.
   In `esp32_main_control.py`, set `WIFI_CHANNEL` to the robot's channel and
   replace `ROBOT_MAC` with the robot's six-byte address, for example
   `b'\\x24\\x6F\\x28\\xAA\\xBB\\xCC'`.
3. Reset the board. Confirm the serial output prints `READY|channel|mac` and
   stays in the bridge loop (there should be no `>>>` prompt). It initializes the built-in
   240×135 ST7789 display using the Feather ESP32-S3 TFT pins shown in the
   provided pinout: power GPIO21, CS GPIO7, DC GPIO39, reset GPIO40,
   backlight GPIO45, and SPI SCK/MOSI GPIO36/GPIO35. The screen shows the latest
   USB command, radio transmit, or robot reply.
4. On the computer, install dependencies with `python -m pip install -r
   requirements.txt`, run `python main.py`, select the ESP32's COM port in the
   USB link row, and click **Connect**. Close Thonny's serial connection first
   if it has the COM port open.

#### Windows USB connection checklist

Run the desktop app from this project directory so its modules are found. If
using the Python 3.12 installation that has pyserial, run
`py -3.12 .\main.py`. If another interpreter is selected, install pyserial
into that exact interpreter and launch with it; a package installed for one
Python does not make it available to another. For example:

```powershell
py -3.12 -m pip install -r requirements.txt
py -3.12 .\main.py
```

In the port list, select the entry described as **USB Serial Device** (COM8 in
the currently observed setup); COM3 and COM4 were Bluetooth serial ports. Port
numbers can change after reconnecting, so identify the board by its USB
description and by the successful **Connect + identify** PING/PONG test.
Thonny and the desktop app cannot use the same COM port at the same time: close
Thonny's Shell/serial session before connecting from the app.

The USB bridge must be running on the main-control ESP32. In Thonny, save
`esp32/esp32_main_control.py` to the device as `/main.py`, with
`esp32/board_display.py`, `esp32/st7789py.py`, and `esp32/vga1_16x32.py` at the device root, then
reset the board. The serial output should show `READY|channel|mac` and remain
in the bridge loop. A `>>>` prompt means the program returned to the REPL; a
line like `PING|GUI0001` typed or sent at that prompt is treated as Python and
will produce `NameError`. Confirm the bridge is running before sending app
commands.

While disconnected, the COM list refreshes automatically when USB devices are
plugged in or removed. Each choice includes Windows' USB device description.
**Connect + identify** sends a unique `PING` and only reports an ESP32 match
after receiving the matching `PONG` token.

The USB serial protocol and ESP-NOW payloads are plain text, with one message
per line and fields separated by `|`. The **Robot control / communication
test** window sends these commands. A square task is:

```text
TASK|1|0|0|Zigzag
```

The fields are message type, task ID, zero-based row, zero-based column, and
pattern name. The equivalent Python f-string is:

```python
message = f"TASK|{task_id}|{row}|{column}|{pattern}"
```

For direct coordinate and instruction testing, the app sends a `MOVE` line:

```text
MOVE|2|1.25|3.50|90.0|DRIVE
```

Its fields are task ID, target X/Y in feet, heading in degrees (or `AUTO`), and
instruction (`DRIVE` or `TURN`). `DRIVE` heads toward the target; a numeric
heading must agree within 3 degrees. `TURN` pivots in place to the requested
heading. `TASK` is rejected with `PATH_SEGMENTS_REQUIRED`; use **Send planned
path** to send planner output.

### Robot receiver and path execution protocol

Copy `esp32/esp32_robot_control.py`, `esp32/motor_driver.py`, and
`esp32/motor_test_protocol.py` to the robot ESP32 root using Thonny.
`robot_motion.py` is an offline geometry helper and is not required by this
firmware. Set both ESP32s to the same `WIFI_CHANNEL`; the main board also needs
the robot's MAC in `ROBOT_MAC`. The robot learns the main board's MAC from its
first radio packet.

Robot hardware mode has a two-hop startup check. The desktop first identifies
the main-control board over USB with matching `PING|token` / `PONG|token`. It
then sends `RADIO_PING|token` through the bridge. The robot replies with a
path-readiness status and its commanded pose, for example
`RADIO_PONG|token|PATH_READY|POSE_VALID|0.5000|0.5000|0.00`. It reports
`PATH_NOT_CONFIGURED` when calibration/driver setup is missing or
`POSE_UNKNOWN` after a robot-side jog or interrupted movement. The desktop
checks pose agreement before it sends a tile path. A radio reply alone never
starts a motor. Place the robot at its configured start pose before rebooting;
the reported pose is not measured by a sensor.

For each tile, the desktop sends a manifest describing the route and expected
segment count, then streams each planned vector entry after the preceding
segment is acknowledged:

```text
MISSION_START|mission_id|tile_row|tile_column|pattern|segment_count
```

For example, `MISSION_START|H0001|0|0|Zigzag|18` declares tile row 0, column 0
with 18 ordered segments. The robot validates the path setup and replies
`MISSION_READY|H0001|18`. Each segment is then sent as:

```text
SEG|mission_id|sequence|pivot_degrees|distance_ft|target_x_ft|target_y_ft|heading_degrees
```

The bridge forwards every message unchanged over ESP-NOW. The robot checks the
mission ID and sequence, executes a pivot followed by a straight drive using
the configured open-loop rates, and replies to each segment:

```text
RECEIVED|mission_id|sequence
DONE|mission_id|sequence|estimated_x_ft|estimated_y_ft|estimated_heading_degrees
```

After the declared final sequence, the robot checks that the commanded endpoint
matches the declared tile center, then sends
`TILE_DONE|mission_id|tile_row|tile_column|segment_count`. Only a matching
`TILE_DONE` causes the GUI to mark the tile searched and move its target to the
next tile. `TX`, `MISSION_READY`, `RECEIVED`, and intermediate `DONE` messages
do not complete the tile. A failed segment, timeout, or operator stop sends
`MISSION_STOP|mission_id` and leaves the tile incomplete.

The robot has no wheel encoders or localization feedback. `DONE` and
`TILE_DONE` confirm that the scheduled motor timings elapsed and outputs were
stopped; the coordinates remain open-loop estimates. Battery sag, friction,
load, startup delay, and motor mismatch can change actual travel. Calibration
values `DRIVE_SPEED_MM_S` and `TURN_RATE_DEG_S` in
`esp32/esp32_robot_control.py` deliberately start as `None`, so path-ready
preflight fails closed until measured on the assembled robot at the selected
`PATH_DUTY_U16`. Do not put guessed values there or interpret protocol
completion as physical arrival.

### Robot ESP32 radio and motor jog test

Open **Robot control / communication test** after connecting to the main-control
ESP32. **Ping robot radio** sends a token to the robot MAC configured in
`esp32_main_control.py`; a matching `RADIO_PONG` confirms the packet round trip.
The motor jog controls send bounded packets in this format:

```text
RADIO_PING|token
MOTOR_TEST|test_id|direction_a|direction_b|duty_u16|duration_ms
MOTOR_STOP|test_id
```

The robot answers `RADIO_PING` with `RADIO_PONG|token|path_status|pose_status|x|y|heading`.
`path_status` is `PATH_READY` or `PATH_NOT_CONFIGURED`; `pose_status` is
`POSE_VALID` or `POSE_UNKNOWN`.
The panel offers forward, reverse, pivot-left/right, and Motor A/B-only jogs.
Choose duty from 1--75 percent and duration from 100--1500 ms. The default is
30 percent for 400 ms. The robot acknowledges start and reports when the timer
ends; the reply does not verify that a wheel moved. Check movement visually,
starting with the drive wheels raised and one motor at a time. Wireless STOP is
not an emergency disconnect.

Example: `MOTOR_TEST|J0001|1|1|19660|400` commands both motors forward for
400 ms at about 30 percent duty. The robot replies `MOTOR_TEST_STARTED|J0001|400`
and then `MOTOR_TEST_DONE|J0001|elapsed_ms` when its timer expires. `MOTOR_STOP|J0001`
disables PWM; the robot may coast rather than brake instantly.

Both motor-driver modules now use the same Adafruit ESP32-S3 TFT Feather
mapping: GPIO5/GPIO6 for Motor A direction, GPIO9/GPIO10 for Motor B direction,
and GPIO11/GPIO12 for 1 kHz PWM to ENA/ENB. Remove both ENA and ENB jumpers
before connecting these GPIOs. This shared mapping makes the motor-driver
wiring the same on the main-control and robot Feather boards. It is specific to
the Feather ESP32-S3 shown in the pinout; do not use it on a classic ESP32
module, where some of these pins are reserved for flash or unavailable. Verify
the exact robot board before connecting any GPIOs.

### Local motor test on the main-control Feather

The desktop **Robot control / communication test** window can also test motors
from the main-control Adafruit ESP32-S3 TFT Feather without a second robot
ESP32. Connect the motor driver to the main-control Feather; the robot ESP32
does not need to be plugged in for local jogs. Copy
`esp32/esp32_main_control.py` to the Feather as `/main.py`, plus
`esp32/board_display.py`, `esp32/main_control_motor_driver.py`,
`esp32/motor_test_protocol.py`, `esp32/st7789py.py`, and `esp32/vga1_16x32.py` to the device root.
The last two display modules are upstream dependencies; keep local copies in
`esp32/` alongside the project firmware. The
robot's `esp32/motor_driver.py`, `esp32/robot_motion.py`, and
`esp32/esp32_robot_control.py` are not used in this mode.

In the desktop control window, click **Enable local motor mode**. Wait for the
board display to show **FAKE ROBOT ON** and the app to report that the
control-board motor outputs are active. That acknowledgement means the local
driver module loaded and fake mode is enabled; the desktop then routes the
existing bounded Forward/Reverse/Pivot/Motor A/B/STOP jog commands to the
control board's own GPIO pins. Fake mode resets to OFF whenever the board
restarts. Click **Disable local motor mode** to return motor jogs to the normal
ESP-NOW robot path.

Wire the motor driver to Feather GPIO5/GPIO6 (Motor A direction),
GPIO9/GPIO10 (Motor B direction), and GPIO11/GPIO12 (ENA/ENB PWM). Remove both
ENA/ENB jumpers. These are Feather header pins D5, D6, D9, D10, D11, and D12;
see the [Adafruit ESP32-S3 TFT Feather pinout](https://learn.adafruit.com/adafruit-esp32-s3-tft-feather/pinouts).
Keep motor power on the motor driver's rated supply and share ground with the
Feather. Start with the drive wheels raised and low duty. The timed software
stop is not a physical emergency stop; keep motor power easy to disconnect.

For an L298N module, connect a current-capable motor supply to its labeled motor
input, connect grounds, and power the ESP32 separately over USB for initial
tests. Never connect ESP32 3V3 to the driver's 5 V rail. Module regulator
jumpers and 5 V terminal direction vary, so verify the exact board before
using its 5 V rail. The firmware's 75 percent PWM ceiling is not a guarantee
that a 3--6 V TT motor is protected: L298 output drop varies with current, and
PWM applies supply-voltage pulses. Check the L298N, motor, and supply ratings;
avoid a rectangular smoke-detector 9 V battery for motor power.

The app's **Ping ESP32** button sends `PING|GUI####`, where `####` is an
app-local counter (for example, `PING|GUI0022`). The `GUI` prefix identifies
that the request came from the GUI's manual test button. The board echoes the
same token in `PONG|GUI0022`, letting the app match the response to its ping.
It is only a request ID; it is not a robot ID, MAC address, or command. The
automatic **Connect + identify** check uses the `APP####` prefix instead.
Both PING types are answered locally over USB, and the TFT displays the
request/reply. This checks the computer-to-board serial link without requiring
the robot. `RX|mac|message` lines are forwarded from the robot and shown in the
control window log.

The robot reports scheduled segment and tile completion separately:

```text
DONE|H0001|0|0.7500|0.5000|0.00
TILE_DONE|H0001|0|0|18
```

Keep each ESP-NOW message at or below 240 encoded bytes. Send one short task or
waypoint at a time rather than a full detailed path. The bridge prints
`READY|channel|mac` at startup, `TX|message` after a successful send,
`TX_FAIL|message` if ESP-NOW reports a failed send, and
`RX|sender_mac|robot_message` for each robot message. It prints `ERROR|reason`
for an oversized or otherwise invalid serial command.

The bridge does not retry commands. The desktop checks mission IDs, tile
coordinates, segment count, and sequence order. It updates the map only after
the matching `TILE_DONE`, not after a radio transmit or segment acknowledgement.

### Testing and validation procedure

Use these checks in order. The first checks are computer-only; the later checks
need the main-control board and, for radio reception, a robot ESP32.

#### 1. Check text protocol formatting

From the project folder, run:

```powershell
python -m unittest test_serial_protocol.py test_robot_motion.py
```

Pass condition: all listed test cases pass. These cover `PING`, `TASK`, `MOVE`,
and `SEG` formatting, invalid values, `RX` parsing, and differential-drive
geometry calculations. They do not open a COM port or drive motors. The new
tile-manifest and end-to-end robot completion behavior still need protocol and
hardware validation below.

#### 2. Check the USB serial round trip

1. Flash/run `esp32/esp32_main_control.py` on the main-control ESP32, with
   `esp32/board_display.py`, `esp32/st7789py.py`, and `esp32/vga1_16x32.py` copied to the board.
2. Close Thonny's serial connection so it releases the COM port. Connect the
   board to the computer with a USB data cable.
3. Run `python test_esp32_communication.py`. Select the COM port shown for the
   ESP32. Or pass it directly, for example:

   ```powershell
   python test_esp32_communication.py --port COM5
   ```

4. The script sends `PING|token` and waits up to eight seconds for the matching
   `PONG|token`.

Pass condition: the script prints `PASS`, the matching `PONG` appears on the
serial console, and the TFT shows the PING/PONG status. This verifies USB data
in both directions and the board's display update path. It does not exercise
ESP-NOW.

#### 3. Check connection through the desktop app

1. Run `python main.py` after installing `requirements.txt`.
2. In **Main-control ESP32 USB link**, select the ESP32 COM port and click
   **Connect**. The app sends an automatic tokenized PING; a matching PONG
   identifies the selected port as the controller and confirms USB communication.
3. Open **Robot control / communication test** and click **Ping ESP32**.

Pass condition: the app log contains the sent `PING|token` and received
`PONG|token`, the link status says the USB communication test passed, and the
TFT displays the status. This checks the same USB path using the GUI's serial
reader and port selection.

#### 4. Validate the robot startup handshake

This check requires the robot ESP32 to be powered, configured on the same
channel, and running firmware that understands the text protocol.

1. Connect the computer to the main-control ESP32 and confirm its matching USB
   PING/PONG.
2. In the main window select **Robot hardware (ESP-NOW)** and click **Start**.
   Confirm the app sends a tokenized radio ping and receives a matching robot
   reply and matching estimated pose before it attempts a tile manifest.
3. With calibration values unset, expect `PATH_NOT_CONFIGURED`; this confirms
   radio reachability while preventing an uncalibrated path from starting.
4. Only after safe physical motor checks, measure straight speed and pivot
   rate at the configured duty. Enter these values in
   `esp32/esp32_robot_control.py`, restart the robot, and confirm preflight
   reports `PATH_READY|POSE_VALID` with a pose matching the desktop. The robot
   must be physically placed at its configured start before boot; its pose is
   not measured from hardware.

Pass condition: the matching token is shown in the desktop log and the app
reports the robot's path-readiness status separately from simple radio
connectivity. A bridge `TX` by itself is not a pass.

#### 5. Test motor jog packets

1. Confirm the L298N wiring, remove ENA/ENB jumpers, and raise the drive wheels.
2. Connect the main-control ESP32, then press **Ping robot radio** and confirm
   a matching `RADIO_PONG` in the activity log.
3. Send Motor A-only and Motor B-only jogs at the default 30 percent for 400 ms.
   Confirm each selected motor turns; visually check direction and abnormal
   noise or heating. Use **STOP motors** to send a stop packet.
4. Try the short forward and pivot jogs only after both individual motors work.

Pass condition: the robot replies to each packet and the expected motor(s) are
visibly turning. The software reply alone cannot verify motor movement. Remove
the battery if a motor keeps running or the driver becomes hot.

Forward/pivot labels assume both channel wiring directions match. If a motor
turns the wrong way, set its `MOTOR_A_DIRECTION_INVERTED` or
`MOTOR_B_DIRECTION_INVERTED` option in `esp32/motor_driver.py` after checking
each channel separately.

#### 6. Validate scheduled tile completion behavior

1. Use wheels-raised, bounded tests in a clear area. Select Robot hardware mode
   and observe `MISSION_START`, the matching `MISSION_READY`, then one matching
   `RECEIVED` and scheduled `DONE` per sequence.
2. Confirm the robot's final `TILE_DONE|mission|row|column|count` has the same
   mission ID, active tile, and declared segment count.
3. Confirm the GUI updates the robot pose only on segment `DONE` and marks the
   tile searched / chooses the next tile only on matching `TILE_DONE`.
4. Confirm malformed, stale, or out-of-order replies are ignored. Disconnect
   radio or stop a test safely and confirm a missing confirmation cannot
   advance the map. Keep the physical motor-supply disconnect within reach.
5. Repeat straight and pivot trials and record the actual measured endpoint
   error. A `TILE_DONE` means the schedule ran; it cannot verify measured
   arrival because there is no wheel feedback.

Pass condition: the tile state advances exactly once after the matching final
tile reply, and does not advance on `TX`, `MISSION_READY`, intermediate `DONE`,
an error, or a timeout. Physical validation separately requires observed
wheel movement, correct direction, repeatable calibration, and stopping.

#### Troubleshooting

- **No COM ports appear:** open Windows Device Manager and check **Ports (COM & LPT)**.
  Unplug/reconnect the board to identify the new entry, confirm the USB-C cable
  supports data, then click **Refresh** in the app. Install dependencies with
  `python -m pip install -r requirements.txt` using the same Python interpreter
  that launches `main.py`.
- **You see a COM port in Device Manager but the app list is empty:** run
  `python main.py` from the updated source folder, install pyserial in that
  Python environment, then click **Refresh**. The prebuilt `release/DCLAR.exe`
  may not include these latest serial changes. If launching with
  `C:\\msys64\\mingw64\\bin\\python.exe`, first run
  `& "C:\\msys64\\mingw64\\bin\\python.exe" -m ensurepip --upgrade`, then
  `& "C:\\msys64\\mingw64\\bin\\python.exe" -m pip install pyserial`.
  This Python installation may not include pip by default; another option is
  or use a Python 3.12 environment that has pyserial with `py -3.12 main.py`.
- **The port cannot be opened:** close Thonny or any other serial monitor that
  may already have the COM port open.
- **No `PONG` arrives:** check the selected COM port, confirm the board is
  running this firmware, and reset/reconnect it before retrying.
- **The TFT is blank:** confirm the board matches the Feather ESP32-S3 TFT
  pinout, both MicroPython files are on the device, and the firmware reports
  no `DISPLAY_ERROR` over serial.
- **`TX_FAIL` or no robot `RX`:** check the robot MAC address and shared Wi-Fi
  channel. Confirm the robot is running its ESP-NOW receiver and replying in
  the documented text format.

## How the Python files communicate

`main.py` is the coordinator. It imports the route functions from `route.py`
and the map/pattern tools from `search.py`. The flow is:

1. `main.py` asks `route.nearest_unsearched(...)` which clear tile to search.
2. It asks `route.path_between(...)` for the cell-by-cell route to that tile.
   This route only gets the robot to the tile; it does not mark transit tiles
   as searched.
3. `main.py` asks `search.make_search_path(...)` for the ordered points inside
   the chosen tile. `search.py` gives local tile coordinates; `main.py` adds
   the tile's row and column to convert them into map coordinates.
4. The Tkinter animation repeatedly moves the simulated robot toward those
   points. Once all points in the tile are reached, `main.py` calls
   `grid.complete_target()` and then plans the next tile.

`route.py` and `search.py` do not call each other and do not control the GUI.
They are planning/data modules called by `main.py`.

## Where the digital-twin data lives

The current tile map is a `SearchGrid` object in memory, created in
`main.py`. A map cell is an integer pair `(row, column)`, with row 0 and column
0 at the upper-left. `grid.obstacles` and `grid.searched` are sets of these
pairs, `grid.order` is the completion-order list, and `grid.current_cell` and
`grid.target_cell` identify the current and selected cells. The continuous
robot pose (`robot_row`, `robot_column`, `robot_heading`) and the exact
sub-tile spill locations are also held in `main.py` variables. This is a
software model of the map and robot, not a sensor-fed or continuously saved
digital twin.

Paths are Python lists while the program runs. `planned_route` contains
`(row, column)` map cells. `path_points` contains `(x, y)` fractions relative
to the active tile. `movement_command_vector` is a list of dictionaries, one
per upcoming straight segment, with pivot angle, global heading, target `x` and
`y` in feet, and forward distance. When a tile finishes, its local waypoints
and derived `dx`, `dy`, distance, and heading vectors are appended to
`completed_path_records`.

Clicking **Save path plan** writes a snapshot to `search_path_plan.json` beside
`main.py`. JSON turns Python lists and dictionaries into readable text. It
includes rack layout, completed tile paths, planned route cells, the active
tile path, the current movement-command vector, spill patches, and simulated
leak events. The program currently writes that snapshot but does not reload it
on startup; unsaved progress is lost when the app closes. See
[`docs/system_design.tex`](docs/system_design.tex) for the coordinate and
field-by-field data model.

The robot's front arrow follows its 5 in width. Its 7 in long axis lies across
the arrow and provides the nominal swath width. Spill placement uses two clicks:
select a clear 1 ft map tile, then click the spill's sub-patch in the enlarged
tile view. The saved plan contains both the tile index and the spill's position
within that tile. In the simulator, the robot latches a stop when it comes
within 3.5 in of an annotated patch. The **Acknowledge leak** button is required
before motion resumes, and that patch will not trigger a second stop.

For the current 12 in square, the zigzag uses two lane centers 5 in apart.
Each idealized swath is 7 in wide, so the lanes overlap by 2 in and cover the
tile width. More lanes would add redundant coverage and take longer; increase
the overlap if sensor uncertainty later requires denser sampling.

## Run from Python

Use Python 3.10 or newer with Tkinter available:

```powershell
python main.py
```

Choose **Add spill**, click a clear tile, then click inside the tile outline in
the zoom view. Click **Save path plan** to write `search_path_plan.json` beside
`main.py`.

## Run the Windows executable

The prebuilt Windows application is [`release/DCLAR.exe`](release/DCLAR.exe).
Double-click it to open the app. Windows may show its normal downloaded-app
prompt because this student prototype is not code-signed.

To rebuild it after changing the Python files, install the build tool and run:

```powershell
python -m pip install -r requirements.txt
python -m PyInstaller --noconfirm --distpath release main.spec
```

The resulting executable is `release/DCLAR.exe`. Rebuild it whenever you want
the executable to include newer source code.

## Planning and coordinate conventions

- Each map tile is 1 ft × 1 ft; rack rows are blocked map cells.
- Map `x` increases right/east and `y` increases down/south. Heading is
  0° east, 90° south, 180° west, and 270° north. Positive pivot angles turn
  clockwise/right in this screen-based frame.
- The route planner chooses the reachable unsearched tile with the fewest
  four-neighbor map moves. Ties are resolved from top to bottom, then left to
  right.
- The local planner supplies ordered positions. The motion-command vector
  converts each straight segment into a pivot angle, target global `x`/`y`,
  heading, and forward distance. JSON stores the full remaining vector; the
  window shows its first 12 commands for readability.
- Spill patches are annotated coordinates; the simulated proximity stop does
  not represent a physical sensor measurement or cleanup action.

The local coverage pattern may leave the selected tile and is not checked
against neighboring racks. Sensor field of view, localization error, real turn
radius, wheel slip, collision clearance, and motor-level control are not modeled.
The 3.5 in detection radius is a demonstration setting, not a sensor
specification. Treat the planner output as a learning prototype until those
physical details are measured and included.

## Repository name

Suggested GitHub repository: `dclar-data-center-leak-assessment-robot`.
