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
| ESP32 radio bridge | `esp32_main_control.py` | Runs on the USB-connected main-control ESP32; forwards computer commands to the robot over ESP-NOW and relays robot messages back over USB serial. |
| System description | `docs/system_design.tex` | Higher-level, physical-system-oriented description of the planner, data flow, coordinate frame, and limits. |

## ESP32 main-control bridge

`esp32_main_control.py` is MicroPython firmware for the main-control ESP32. It
is separate from the desktop Tkinter program in `main.py`: the desktop program
does not yet open a serial port or apply robot completion messages to its map.
The bridge provides the communication link that can be connected to that
program in a later step.

### Configure and run

1. Install MicroPython on the main-control ESP32 and connect it to the computer
   over USB.
2. Open `esp32_main_control.py` in a MicroPython tool such as Thonny and set
   `WIFI_CHANNEL` to the channel used by the robot ESP32.
3. Replace `ROBOT_MAC` with the robot's six-byte MAC address, for example
   `b'\\x24\\x6F\\x28\\xAA\\xBB\\xCC'`. The robot must use the same Wi-Fi channel.
4. Save/run the script on the main-control ESP32. It prints a `READY` line
   when ESP-NOW and the serial bridge are initialized.

The USB serial protocol and ESP-NOW payloads are plain text, with one message
per line and fields separated by `|`. The computer sends the command directly;
the bridge forwards it unchanged to the robot. For example, a task command is:

```text
TASK|1|0|0|Zigzag
```

The fields are message type, task ID, zero-based row, zero-based column, and
pattern name. The equivalent Python f-string is:

```python
message = f"TASK|{task_id}|{row}|{column}|{pattern}"
```

The robot firmware should parse the same fields and, after completing the
square, reply with a line such as:

```text
DONE|1|0|0
```

Keep each ESP-NOW message at or below 240 encoded bytes. Send one short task or
waypoint at a time rather than a full detailed path. The bridge prints
`READY|channel|mac` at startup, `TX|message` after a successful send,
`TX_FAIL|message` if ESP-NOW reports a failed send, and
`RX|sender_mac|robot_message` for each robot message. It prints `ERROR|reason`
for an oversized or otherwise invalid serial command.

The desktop program can parse a received `RX` line, then act only when its
robot message is `DONE` for the active task ID and tile. It should mark that
square complete and select the next target after that matching completion;
`TX` only reports that the radio send succeeded. The bridge does not retry
commands, track task IDs, or update the Tkinter map by itself. The robot
firmware and future desktop serial integration must implement those behaviors.

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
