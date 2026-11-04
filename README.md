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
| Simulation and data | `main.py` | Animates pivot-in-place and forward-only motion, accepts spill locations, shows the current command vector, and saves a JSON plan. |
| USB serial link | `serial_link.py`, `serial_protocol.py` | Lists COM ports, connects the desktop app to the main-control ESP32, and formats text commands. |
| ESP32 radio bridge and robot receiver | `esp32/` | Main-control USB/TFT bridge, robot ESP-NOW receiver, and differential-drive path-to-encoder translator. |
| Communication checks | `test_serial_protocol.py`, `test_robot_motion.py`, `test_esp32_communication.py` | Unit-checks message and wheel-target formatting and tests USB PING/PONG. |
| System description | `docs/system_design.tex` | Higher-level, physical-system-oriented description of the planner, data flow, coordinate frame, and limits. |

## ESP32 main-control bridge

`esp32/esp32_main_control.py` is MicroPython firmware for the main-control ESP32.
The desktop `main.py` selects a USB serial port and sends commands to the main
board. **Send planned path** streams one segment at a time and waits for the
matching robot `DONE` before sending the next. A target is marked searched
only after all its segments report completion.

### Configure and run

1. Install MicroPython on the main-control ESP32 and connect it to the computer
   with a USB data cable.
2. In Thonny, select **MicroPython (ESP32)** and the board's COM port. Copy
   `esp32/esp32_main_control.py` and `esp32/board_display.py` to the MicroPython device.
   `board_display.py` uses the same display driver and font as the known-working
   board test: also copy `st7789py.py` and `vga1_16x32.py` to the device root.
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
`board_display.py`, `st7789py.py`, and `vga1_16x32.py` at the device root, then
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

Copy `esp32/esp32_robot_control.py` and `esp32/robot_motion.py` to the robot
ESP32 root using Thonny. Set both ESP32s to the same `WIFI_CHANNEL`; the main
board also needs the robot's MAC in `ROBOT_MAC`. The robot learns the main
board's MAC from its first radio packet.

The desktop sends each planned vector entry as a simple f-string encoded line:

```text
SEG|mission_id|sequence|pivot_degrees|distance_ft|target_x_ft|target_y_ft|heading_degrees
```

The bridge forwards the line over ESP-NOW. The robot replies
`RECEIVED|mission|sequence`, reports calculated encoder targets as
`PLAN|mission|sequence|pivot:left_ticks:right_ticks,drive:left_ticks:right_ticks`,
then sends `DONE|mission|sequence` after its motor backend succeeds. Failures
return `ERR|mission|sequence|reason`. The desktop waits for matching `DONE`
before it sends the next segment; only the final completion advances the map.

`robot_motion.py` translates pivot-and-drive segments into signed wheel encoder
targets. Positive angle means clockwise/right in the map: left wheel forward,
right wheel backward. For pivot angle `a` and wheel-center track `t`, each
wheel travels `t * radians(a) / 2`; convert that arc using tire circumference
and encoder ticks per revolution. A straight drive gives both wheels equal
positive ticks. Negative angles reverse the turn. Wheel track is the distance
between wheel centerlines, not the 7 x 5 inch body size.

Physical movement remains disabled until wheel diameter, wheel track, encoder
ticks per wheel revolution, and maximum wheel speed are measured/configured in
`esp32_robot_control.py`, and an encoder-aware `motor_driver.py` supplies
`move_wheels(left_ticks, right_ticks, max_speed_mm_s, timeout_ms)` and `stop()`.
It must stop on timeout/fault and return only after closed-loop completion.
No motor pinout or driver model has been assumed. Without that setup, the
receiver returns a configuration error instead of moving. The reported pose
is commanded odometry, not localization; there is no obstacle safety model.

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

The robot replies to path segments using this completion format:

```text
DONE|H0001|0
```

Keep each ESP-NOW message at or below 240 encoded bytes. Send one short task or
waypoint at a time rather than a full detailed path. The bridge prints
`READY|channel|mac` at startup, `TX|message` after a successful send,
`TX_FAIL|message` if ESP-NOW reports a failed send, and
`RX|sender_mac|robot_message` for each robot message. It prints `ERROR|reason`
for an oversized or otherwise invalid serial command.

The bridge does not retry commands. The desktop checks mission and sequence
IDs and updates the map after all planned segments report `DONE`.

### Testing and validation procedure

Use these checks in order. The first checks are computer-only; the later checks
need the main-control board and, for radio reception, a robot ESP32.

#### 1. Check text protocol formatting

From the project folder, run:

```powershell
python -m unittest test_serial_protocol.py test_robot_motion.py
```

Pass condition: all twelve tests pass. These cover `PING`, `TASK`, `MOVE`, and
`SEG` formatting, invalid values, `RX` parsing, and differential-drive wheel
target signs. They do not open a COM port or drive motors.

#### 2. Check the USB serial round trip

1. Flash/run `esp32/esp32_main_control.py` on the main-control ESP32, with
   `esp32/board_display.py`, `st7789py.py`, and `vga1_16x32.py` copied to the board.
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

#### 4. Check command forwarding over ESP-NOW

This check requires the robot ESP32 to be powered, configured on the same
channel, and running firmware that understands the text protocol.

1. Connect the computer to the main-control ESP32 as above.
2. Use **Send planned path**. Verify each `SEG` is followed by `RECEIVED`,
   `PLAN`, and matching `DONE` relayed as `RX|mac|...`. The next segment must
   wait for `DONE`; stale mission/sequence replies should be ignored.

Pass condition: matching robot replies are visible on both main board and
desktop. `TX` alone only means the main ESP32 reported a successful radio send.
Motion completion requires measured geometry and a configured motor driver;
otherwise the robot should return a clear setup error.

#### 5. Validate path completion behavior

Only after all segments have matching `DONE` replies does the app mark the
target searched. Robot setup errors, stale mission IDs, and wrong sequence
numbers must not advance the map.

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
