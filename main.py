"""Two-scale grid search demo. Run from this folder with `python main.py`.

The program has two kinds of movement:
1. ``route.py`` moves between the centers of 1 ft map squares.
2. ``search.py`` makes a pattern inside the selected 1 ft square.

Tkinter calls ``update_search`` repeatedly while the window is open. Each call
moves the dot a small amount, redraws the canvases, and schedules the next call.

Tkinter mini-glossary for reading this file:
- ``tk`` is Tkinter's basic toolkit; ``ttk`` supplies the newer themed widgets.
- In ``object.method(arguments)``, the object is the widget/data being acted on.
- A keyword argument like ``fill="red"`` names an option and its value.
- A widget is a GUI object: window, label, button, dropdown, or drawing canvas.
- The first argument to a widget constructor is its parent/container widget.
- Options such as ``text=``, ``fill=``, and ``width=`` customize that widget.
- ``.grid(...)`` places a widget in a row/column inside its parent.
- ``row=`` and ``column=`` select that grid slot; ``padx``/``pady`` add outside
  spacing; ``sticky`` says which edges the widget should stick to (``w``=left,
  ``n``=top, ``ew``=stretch horizontally).
- Canvas x/y positions are pixels measured from its upper-left corner.
- ``Canvas.create_rectangle(x1, y1, x2, y2)`` draws between two opposite corners.
- ``Canvas.create_line(x1, y1, x2, y2)`` draws between two endpoints.
- ``Canvas.create_oval(x1, y1, x2, y2)`` draws an oval inside that bounding box.
- ``Canvas.create_polygon(x1, y1, x2, y2, ...)`` connects several corner points.
- ``Canvas.create_text(x, y)`` puts a text label at a point.
- ``StringVar.get()`` reads a Tkinter variable; ``.set(value)`` updates it.
- ``widget.configure(option=value)`` changes an existing widget option.
- ``widget.bind(event_name, function)`` calls a function when an event occurs.
- ``window.after(ms, function)`` asks Tkinter to call a function later; it does
  not pause/freeze the app like a sleep call would.
- ``window.protocol(name, function)`` customizes a window event such as closing.
- ``window.mainloop()`` starts Tkinter's event loop and keeps the GUI responsive.
"""

import math                         # Distance calculation for smooth movement.
import json                         # Save the planned route as a readable file.
import uuid                         # Keep mission IDs unique across app restarts.
import tkinter as tk                # Base windows, Canvas drawing, and events.
from tkinter import ttk             # Themed labels, buttons, frames, and menus.
from pathlib import Path             # Build a save-file path beside this script.
from tkinter import messagebox

from serial_link import SerialLink, available_ports
from serial_protocol import (
    make_move_message,
    make_mission_start_message,
    make_ping_message,
    make_segment_message,
    make_task_message,
)
from esp32.motor_test_protocol import (
    MAX_MOTOR_TEST_DURATION_MS,
    make_motor_stop_message,
    make_motor_test_message,
    make_robot_radio_ping_message,
)

from route import nearest_unsearched, path_between, uniform_rack_obstacles
from search import (
    ROBOT_ARROW_ALONG_WIDTH,
    ROBOT_LENGTH_IN,
    ROBOT_WIDTH_IN,
    SearchGrid,
    make_search_path,
)


# ---------------------------- Display dimensions ----------------------------
# Every map cell represents a 1 ft by 1 ft patch of floor. ROWS and COLUMNS
# count physical grid squares; CELL_PIXELS only controls their screen size.
ROWS = 10
COLUMNS = 10
CELL_PIXELS = 48
MAP_WIDTH = COLUMNS * CELL_PIXELS
MAP_HEIGHT = ROWS * CELL_PIXELS
LOCAL_PIXELS = 420
LOCAL_TILE_INSET = 102               # Blank zoom margin around the 1 ft tile.
DOT_RADIUS = 7
FRAME_MS = 25                      # Time between animation updates, in ms.
TURN_RATE_DEG_PER_SECOND = 240     # How quickly the robot can rotate in place.
# Demo-only leak proximity: within 3.5 in of a marked point, latch a stop.
# A real sensor must replace this placeholder with its measured detection event.
SIMULATED_LEAK_RADIUS_FT = ROBOT_LENGTH_IN / 24
grid = SearchGrid(ROWS, COLUMNS)   # The map data is separate from its drawing.
# Generate fixed, uniform rack rows: one clear foot at the perimeter, racks one
# foot deep, and two clear feet (two grid cells) of aisle between rack rows.
DEFAULT_RACK_OBSTACLES = uniform_rack_obstacles(
    ROWS,
    COLUMNS,
    border_clearance_cells=1,
    rack_depth_cells=1,
    aisle_gap_cells=2,
)
grid.obstacles.update(DEFAULT_RACK_OBSTACLES)
RACK_ROWS = sorted({row for row, _column in DEFAULT_RACK_OBSTACLES})

# Robot's position in map coordinates. (row=2.5, column=3.5) is the center
# of row 3, column 4. These are continuous coordinates, so the dot moves
# smoothly inside the current 1 ft square.
robot_row = 0.5                    # Continuous map position: row 0, centered.
robot_column = 0.5                 # Continuous map position: column 0, centered.
robot_heading = 0.0                # Radians: 0 points right/east; positive turns down.
path_points = []                   # Local fractional points inside target cell.
path_index = 0                     # Which local path point is next.
travel_points = []                 # Map cells to pass through before target.
planned_route = []                 # Full route.py result, including current/target.
active_pattern = "Zigzag"          # Pattern assigned to the current square.
completed_path_records = []        # Previous finished patterns for saving/export.
spill_points = []                   # Exact spill coordinates: tile plus local x/y.
spill_mode = False                  # When True, clicks place spills in two steps.
spill_selection_cell = None          # Tile chosen before selecting its small patch.
detected_spill_indices = set()       # Do not repeatedly stop for one detected patch.
leak_detection_events = []           # Record the pose and spill point at each stop.
leak_stop_latched = False            # Requires an operator acknowledgment to resume.
movement_command_vector = []       # Upcoming pivot-and-drive commands for display/save.
local_canvas = None                # Enlarged one-square pattern canvas.
route_status = None                # Completion/obstacle count shown under both views.
main_after_id = None               # Timer handle, so close can cancel the callback.
paused = True
esp32_link = None
control_window = None
control_log = None
serial_log_history = []
next_task_id = 1
next_ping_id = 1
serial_port_map = {}
pending_ping_tokens = set()
pending_motor_test_tokens = set()
pending_robot_ping_tokens = set()
next_motor_test_id = 1
next_robot_ping_id = 1
active_motor_test_token = None
fake_robot_mode = False
fake_robot_mode_pending = None
fake_mode_button_text = None
motor_test_status_text = None
last_serial_port_signature = None
hardware_mission_counter = 1
hardware_session_id = uuid.uuid4().hex[:8].upper()
hardware_mission_id = None
hardware_path_commands = []
hardware_path_index = 0
hardware_waiting_sequence = None
hardware_waiting_ready = False
hardware_waiting_tile_done = False
hardware_preflight_token = None
hardware_preflight_stage = None
hardware_radio_ready = False
local_control_ready = False
local_test_started = False
usb_link_ready = False
run_mode = None


# -------------------------------- Main window -------------------------------
# Tk() creates the single application window. Both map views live inside it.
window = tk.Tk()
# title() changes the text in the operating-system title bar. resizable(False,
# False) disables horizontal and vertical resizing for this fixed-size layout.
window.title("1 ft Grid Search Robot")
window.resizable(False, False)

# A Frame groups controls. grid() lays widgets out in rows and columns, like a
# table. Tkinter also has pack() and place() for other layout styles.
toolbar = ttk.Frame(window, padding=8)
# grid(row, column) places the toolbar at row 0, column 0 of the main window.
toolbar.grid(row=0, column=0, sticky="ew")
# Label(parent, text=...) creates text that is displayed but not edited.
ttk.Label(toolbar, text="Next-square route:").grid(row=0, column=0, padx=(0, 5))
# StringVar is a Tkinter-managed string that a menu can display and update.
route_choice = tk.StringVar(value="Nearest unsearched (avoid obstacles)")
route_menu = ttk.Combobox(
    toolbar, textvariable=route_choice,
    values=("Nearest unsearched (avoid obstacles)",),
    state="readonly", width=34,
)
# Combobox is a dropdown. textvariable links its selected choice to route_choice;
# values lists choices; readonly prevents typing a value that is not listed.
route_menu.grid(row=0, column=1, padx=(0, 12))

ttk.Label(toolbar, text="Search inside each square:").grid(row=0, column=2, padx=(0, 5))
# This dropdown chooses the pattern inside a cell; it does not choose the cell.
pattern_choice = tk.StringVar(value="Zigzag")
pattern_menu = ttk.Combobox(
    toolbar, textvariable=pattern_choice,
    values=("Zigzag", "Circle spiral"), state="readonly", width=14,
)
# The selected pattern is kept in pattern_choice for the search planner to read.
pattern_menu.grid(row=0, column=3, padx=(0, 12))

ttk.Label(toolbar, text="Speed:").grid(row=0, column=4, padx=(0, 5))
speed_choice = tk.StringVar(value="Normal")
speed_menu = ttk.Combobox(
    toolbar, textvariable=speed_choice,
    values=("Slow", "Normal", "Fast"), state="readonly", width=8,
)
# The selected speed is read later with speed_choice.get().
speed_menu.grid(row=0, column=5, padx=(0, 10))

# command is the function to call on click. lambda delays the call until then.
pause_button = ttk.Button(toolbar, text="Pause", command=lambda: toggle_pause())
# Button's command runs after a mouse click or keyboard activation.
pause_button.grid(row=0, column=6, padx=(0, 5))
ttk.Button(toolbar, text="Reset", command=lambda: reset_search()).grid(row=0, column=7)
ttk.Button(toolbar, text="Save path plan", command=lambda: save_path_plan()).grid(
    row=0, column=8, padx=(8, 0)
)
spill_button = ttk.Button(toolbar, text="Add spill: off", command=lambda: toggle_spill_mode())
spill_button.grid(row=0, column=9, padx=(8, 0))
ttk.Label(toolbar, text="Run mode:").grid(row=0, column=10, padx=(12, 4))
run_mode_choice = tk.StringVar(value="Select mode")
run_mode_menu = ttk.Combobox(
    toolbar, textvariable=run_mode_choice,
    values=("Simulation", "Robot hardware (ESP-NOW)", "Control-board test (local)"),
    state="readonly", width=26,
)
run_mode_menu.grid(row=0, column=11, padx=(0, 6))
run_mode_button = ttk.Button(
    toolbar, text="Start", command=lambda: start_selected_mode()
)
run_mode_button.grid(row=0, column=12)

# Put both visualizations next to each other in one frame/window.
view_frame = ttk.Frame(window, padding=(10, 0, 10, 0))
# The frame occupies row 1 in the main window. Its own row/columns arrange the
# map and close-up side by side, independent of the toolbar's positions.
view_frame.grid(row=1, column=0, sticky="w")
ttk.Label(view_frame, text="Map: next target and robot position").grid(
    row=0, column=0, sticky="w", pady=(0, 4)
)
pattern_info_text = tk.StringVar()
ttk.Label(view_frame, textvariable=pattern_info_text, wraplength=LOCAL_PIXELS).grid(
    row=0, column=1, sticky="w", padx=(12, 0), pady=(0, 4)
)

canvas = tk.Canvas(
    view_frame, width=MAP_WIDTH, height=MAP_HEIGHT,
    background="white", highlightthickness=0,
)
# Canvas(width, height) creates a blank pixel drawing area. background sets its
# fill color; highlightthickness=0 removes Tkinter's extra focus border.
canvas.grid(row=1, column=0, sticky="n")
# bind() attaches a mouse event to a callback. Tkinter passes the click event;
# event.x/event.y are the click's pixel coordinates inside this canvas.
canvas.bind("<Button-1>", lambda event: handle_map_click(event.x, event.y))

# This close-up used to live in a second window. Keeping it beside the map lets
# you see the chosen map square and the path inside that square together.
local_canvas = tk.Canvas(
    view_frame, width=LOCAL_PIXELS, height=LOCAL_PIXELS,
    background="#f6f7f8", highlightthickness=0,
)
# This is another Canvas widget in the same window, used as a magnified view.
local_canvas.grid(row=1, column=1, padx=(12, 0), sticky="n")
local_canvas.bind(
    "<Button-1>", lambda event: handle_spill_patch_click(event.x, event.y)
)

# StringVar holds text that Tkinter widgets can observe. When its value changes,
# the Label connected with textvariable below displays the new text automatically.
status_text = tk.StringVar(value="Starting search…")
# Label(textvariable=...) displays the live StringVar; padding adds space around it.
ttk.Label(window, textvariable=status_text, padding=8).grid(row=2, column=0, sticky="w")
ttk.Label(
    window,
    text=("Dark gray = rack rows; 2 ft aisles and 1 ft perimeter are clear. "
          "Robot body = 7×5 in; arrow follows its 5 in width. "
          "Blue arrows in zoom = search path; orange = next route/waypoint."),
    wraplength=MAP_WIDTH + LOCAL_PIXELS + 30,
    padding=(8, 0, 8, 8),
).grid(row=3, column=0, sticky="w")
route_status = tk.StringVar()
# This other status label reports how many cells are complete/obstacles.
ttk.Label(window, textvariable=route_status, padding=(8, 0, 8, 8)).grid(
    row=4, column=0, sticky="w"
)
command_text = tk.StringVar(value="Next movement instruction will appear here.")
# wraplength is a maximum text line width in pixels; Tkinter wraps longer text.
ttk.Label(
    window, textvariable=command_text,
    wraplength=MAP_WIDTH + LOCAL_PIXELS + 30,
    padding=(8, 0, 8, 10),
).grid(row=5, column=0, sticky="w")
movement_vector_text = tk.StringVar(value="[]")
ttk.Label(
    window,
    textvariable=movement_vector_text,
    wraplength=MAP_WIDTH + LOCAL_PIXELS + 30,
    padding=(8, 0, 8, 10),
).grid(row=6, column=0, sticky="w")

# USB connection controls stay in the main window so the correct COM port is
# visible while the planner is running.
serial_frame = ttk.LabelFrame(window, text="Main-control ESP32 USB link", padding=8)
serial_frame.grid(row=7, column=0, padx=8, pady=(0, 10), sticky="ew")
serial_status_text = tk.StringVar(value="Disconnected")
ttk.Label(serial_frame, text="COM port:").grid(row=0, column=0, padx=(0, 5))
serial_port_choice = tk.StringVar()
serial_port_menu = ttk.Combobox(
    serial_frame, textvariable=serial_port_choice, values=(), state="readonly", width=14
)
serial_port_menu.grid(row=0, column=1, padx=(0, 6))
ttk.Button(serial_frame, text="Refresh", command=lambda: refresh_serial_ports()).grid(
    row=0, column=2, padx=(0, 6)
)
serial_connect_button = ttk.Button(
    serial_frame, text="Connect + identify", command=lambda: toggle_serial_connection()
)
serial_connect_button.grid(row=0, column=3, padx=(0, 10))
ttk.Button(
    serial_frame, text="Robot control / communication test",
    command=lambda: open_robot_control(),
).grid(row=0, column=4, padx=(0, 10))
ttk.Label(serial_frame, textvariable=serial_status_text).grid(row=0, column=5, sticky="w")


def map_center(row, column):
    """Convert an integer map cell index to that cell's pixel center.

    Adding 0.5 selects the center rather than the top/left boundary.
    Multiplying by CELL_PIXELS converts cell units to screen pixels. Canvas
    positions are (x, y), so column maps to x while row maps to y.
    """
    return (column + 0.5) * CELL_PIXELS, (row + 0.5) * CELL_PIXELS


def robot_body_polygon(center_x, center_y, heading, pixels_per_inch):
    """Return the robot's four body corners for drawing on a Canvas.

    The forward arrow lies along the 5 in width. The 7 in length is across the
    arrow. Unit vectors let us rotate the rectangle to match the robot heading.
    The returned flat list is suitable for ``Canvas.create_polygon(*points)``.
    """
    forward_x, forward_y = math.cos(heading), math.sin(heading)
    side_x, side_y = -forward_y, forward_x  # A perpendicular (90-degree) vector.
    half_forward = ROBOT_WIDTH_IN * pixels_per_inch / 2
    half_side = ROBOT_LENGTH_IN * pixels_per_inch / 2

    corners = []
    for forward_sign, side_sign in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
        x = center_x + forward_sign * half_forward * forward_x + side_sign * half_side * side_x
        y = center_y + forward_sign * half_forward * forward_y + side_sign * half_side * side_y
        corners.extend((x, y))
    return corners


def cardinal_name(angle_radians):
    """Name the nearest compass direction for this canvas heading.

    The map uses screen axes: 0 degrees is right/east, 90 is down/south,
    180 is left/west, and 270 is up/north.
    """
    directions = ("East", "South", "West", "North")
    # Add half a 90-degree sector before int() truncates. This gives ordinary
    # nearest-direction rounding rather than Python's tie-to-even round.
    quarter_turn = int(((angle_radians % (2 * math.pi)) + math.pi / 4) / (math.pi / 2)) % 4
    return directions[quarter_turn]


def current_destination():
    """Return the next waypoint as (row, column, description), or None.

    While traveling between squares, the next waypoint is an intermediate
    cell's center. While searching a square, it is the next local pattern point.
    """
    if travel_points:
        row, column = travel_points[0]
        return row + 0.5, column + 0.5, f"transit through row {row + 1}, column {column + 1}"
    if grid.target_cell is not None and path_index < len(path_points):
        row, column = grid.target_cell
        local_x, local_y = path_points[path_index]
        return row + local_y, column + local_x, f"square row {row + 1}, column {column + 1}"
    return None


def update_command_readout():
    """Explain the next movement in plain language beside the path display."""
    global movement_command_vector
    if hardware_mission_id is not None:
        if hardware_waiting_tile_done:
            command_text.set("Final segment confirmed; waiting for robot TILE_DONE.")
        elif hardware_waiting_ready:
            command_text.set("Tile path declared; waiting for robot MISSION_READY.")
        else:
            command_text.set(
                "Robot-confirmed segment %d/%d; awaiting the next completion token."
                % (hardware_path_index, len(hardware_path_commands))
            )
        movement_vector_text.set(
            "Confirmed scheduled segments: %d/%d; tile state waits for TILE_DONE."
            % (hardware_path_index, len(hardware_path_commands))
        )
        return
    if leak_stop_latched:
        command_text.set(
            "LEAK DETECTED: robot stopped at its current pose. Acknowledge to resume."
        )
        movement_vector_text.set(
            "Movement-command vector paused; the full plan remains saved on request."
        )
        return
    if grid.complete:
        # StringVar.set(value) changes text shown by its connected Label.
        command_text.set("No next movement: all reachable squares are complete.")
        movement_command_vector = []
        movement_vector_text.set("Upcoming movement-command vector: []")
        return
    destination = current_destination()
    if destination is None:
        command_text.set("Preparing the next waypoint…")
        movement_vector_text.set("Upcoming movement-command vector: []")
        return

    target_row, target_column, description = destination
    # Subtract current position from destination to form the movement vector.
    # Positive dx means right/east; positive dy means down/south in canvas axes.
    dx = target_column - robot_column
    dy = target_row - robot_row        # Positive y means down/south on the map.
    # hypot(x,y) is the Pythagorean length sqrt(x**2 + y**2).
    distance = math.hypot(dx, dy)
    if distance < 0.005:
        command_text.set(f"At waypoint for {description}; advancing to the next point.")
    else:
        # atan2(y, x) converts the vector into an angle while preserving its quadrant.
        # atan2 gives the angle of the vector, in radians, and handles all quadrants.
        desired_heading = math.atan2(dy, dx)
        turn = (desired_heading - robot_heading + math.pi) % (2 * math.pi) - math.pi
        desired_direction = cardinal_name(desired_heading)
        if abs(turn) > math.radians(3):
            turn_word = "right" if turn > 0 else "left"
            command_text.set(
                f"Next: pivot {turn_word} {abs(math.degrees(turn)):.0f}° in place, "
                f"then drive forward {distance:.2f} ft toward {desired_direction} ({description})."
            )
        else:
            command_text.set(
                f"Next: drive forward {distance:.2f} ft toward {desired_direction} ({description})."
            )

    # Build and display the complete remaining plan. Each list item is one
    # waypoint command record with its own pivot angle and global x/y target.
    points = []
    for next_row, next_column in travel_points:
        points.append((next_column + 0.5, next_row + 0.5))
    if grid.target_cell is not None:
        row, column = grid.target_cell
        points.extend((column + x, row + y) for x, y in path_points[path_index:])
    movement_command_vector = motion_commands_from_pose(
        robot_column, robot_row, robot_heading, points
    )
    if movement_command_vector:
        # This compact readable string is Python-list-like on purpose: it shows
        # the same ordered records that are saved as JSON by the Save button.
        display_items = []
        # Show the next dozen moves so the label stays readable for a person.
        # The full vector remains in movement_command_vector and in the JSON
        # file, including every small segment of a circle-spiral pattern.
        display_limit = 12
        for item in movement_command_vector[:display_limit]:
            pivot = item["pivot_angle_degrees"]
            if abs(pivot) < 0.05:
                pivot_text = "KEEP HEADING"
            else:
                side = "RIGHT" if pivot > 0 else "LEFT"
                pivot_text = f"PIVOT {side} {abs(pivot):.0f}°"
            display_items.append(
                f"{{{pivot_text}; DRIVE to x={item['target_x_ft']:.2f}, "
                f"y={item['target_y_ft']:.2f} ft; heading={item['global_heading_degrees']:.0f}°}}"
            )
        remaining_count = len(movement_command_vector) - display_limit
        if remaining_count > 0:
            display_items.append(
                f"... {remaining_count} more commands (Save path plan for the full vector)"
            )
        movement_vector_text.set("Upcoming movement-command vector: [" + ", ".join(display_items) + "]")
    else:
        movement_vector_text.set("Upcoming movement-command vector: []")


def motion_commands_from_pose(start_x, start_y, start_heading, waypoints):
    """Make one pivot-and-forward command record for each destination point.

    x/y are in feet from the map's upper-left corner. The global angle frame is
    the same as the drawing: 0° east, 90° south, 180° west, 270° north. A
    positive pivot angle means clockwise/right on this screen; negative means
    counterclockwise/left. Each record is a vector entry the robot could later
    receive: pivot to a global heading, then drive forward to global x/y.
    """
    commands = []
    x, y, heading = start_x, start_y, start_heading
    for waypoint_x, waypoint_y in waypoints:
        dx, dy = waypoint_x - x, waypoint_y - y
        distance = math.hypot(dx, dy)
        if distance < 1e-6:
            continue  # Already at this waypoint; no pivot or drive is needed.
        target_heading = math.atan2(dy, dx)
        # Wrap the angle to the shortest signed pivot in [-180°, 180°].
        pivot = (target_heading - heading + math.pi) % (2 * math.pi) - math.pi
        commands.append({
            "pivot_angle_degrees": round(math.degrees(pivot), 2),
            "global_heading_degrees": round(math.degrees(target_heading) % 360, 2),
            "target_x_ft": round(waypoint_x, 4),
            "target_y_ft": round(waypoint_y, 4),
            "distance_forward_ft": round(distance, 4),
            "action": "PIVOT_THEN_DRIVE_FORWARD",
        })
        x, y, heading = waypoint_x, waypoint_y, target_heading
    return commands


def check_simulated_leak_detection():
    """Stop at the first marked spill point reached by the simulated robot.

    The prototype has no physical sensor, so this uses proximity to a manually
    placed spill marker as a demonstration of the required stop behavior. A
    hardware sensor adapter should call the same stop behavior when its own
    detector reports a leak, using a calibrated confidence and location.
    """
    global paused, leak_stop_latched
    if leak_stop_latched:
        return False

    for spill_index, spill in enumerate(spill_points):
        if spill_index in detected_spill_indices:
            continue
        spill_x = spill["column"] + spill["x_fraction"]
        spill_y = spill["row"] + spill["y_fraction"]
        # hypot() gives straight-line distance in feet in the global map frame.
        distance_ft = math.hypot(robot_column - spill_x, robot_row - spill_y)
        if distance_ft <= SIMULATED_LEAK_RADIUS_FT:
            detected_spill_indices.add(spill_index)
            event = {
                "spill_index_1_based": spill_index + 1,
                "tile_row_1_based": spill["row"] + 1,
                "tile_column_1_based": spill["column"] + 1,
                "spill_x_ft": round(spill_x, 4),
                "spill_y_ft": round(spill_y, 4),
                "robot_x_ft_at_stop": round(robot_column, 4),
                "robot_y_ft_at_stop": round(robot_row, 4),
                "robot_heading_degrees_at_stop": round(
                    math.degrees(robot_heading) % 360, 2
                ),
                "distance_to_spill_ft": round(distance_ft, 4),
                "detection_method": "simulated proximity to marked spill",
            }
            leak_detection_events.append(event)
            paused = True
            leak_stop_latched = True
            pause_button.configure(text="Acknowledge leak")
            status_text.set(
                f"SIMULATED LEAK DETECTED in row {spill['row'] + 1}, "
                f"column {spill['column'] + 1}. Robot stopped at "
                f"x={robot_column:.2f} ft, y={robot_row:.2f} ft. "
                "Acknowledge leak to resume."
            )
            return True
    return False


def advance_forward_toward(destination_row, destination_column):
    """Turn in place toward a waypoint, then move only along the front arrow.

    Return True after reaching the waypoint. The robot never changes its x/y
    position sideways: while advancing, position changes along cos/sin of the
    heading. It may rotate in place first to face the next path segment.
    """
    global robot_row, robot_column, robot_heading
    dr = destination_row - robot_row
    dc = destination_column - robot_column
    distance = math.hypot(dr, dc)
    if distance < 1e-9:
        robot_row, robot_column = destination_row, destination_column
        return True

    # atan2(row-change, column-change) is atan2(y, x): it points the nose from
    # the robot's current position toward the destination.
    desired_heading = math.atan2(dr, dc)
    # Wrap the angle difference into [-pi, pi] so the robot takes the shorter
    # direction when turning (for example, turn left 10 degrees, not right 350).
    turn_error = (desired_heading - robot_heading + math.pi) % (2 * math.pi) - math.pi
    # radians() converts degrees/s to radians/s; multiply by frame seconds to
    # get the maximum angle the robot may turn during this one update.
    max_turn_this_frame = math.radians(TURN_RATE_DEG_PER_SECOND) * FRAME_MS / 1000
    if abs(turn_error) > max_turn_this_frame:
        robot_heading += math.copysign(max_turn_this_frame, turn_error)
        return False  # It is turning in place; do not move sideways or forward yet.

    robot_heading = desired_heading
    # min() prevents overshooting: use the normal frame step unless the target
    # is closer, in which case move only the remaining distance.
    step = min(speed_feet_per_second() * FRAME_MS / 1000, distance)
    # Forward-only translation: advance on the direction the arrow points.
    robot_column += math.cos(robot_heading) * step
    robot_row += math.sin(robot_heading) * step
    if step >= distance - 1e-9:
        robot_row, robot_column = destination_row, destination_column
        return True
    return False


def draw_map(target_canvas, show_robot):
    """Paint a map canvas from the current SearchGrid state.

    Canvas is Tkinter's drawable surface for rectangles, lines, text, and ovals.
    We clear and redraw this small map each frame; that is simpler to learn than
    keeping track of and moving every individual shape.
    """
    # target_canvas is the canvas passed into this function (here: the big map).
    # show_robot is a True/False switch that decides whether to draw the robot.
    # Delete old shapes so the moving dot does not leave a trail behind it.
    # delete("all") removes every drawn item from this canvas. The special tag
    # "all" means every shape; canvases can also delete shapes by their own tag.
    target_canvas.delete("all")
    # Paint cell fills first, then borders and the dot on top of them.
    # Each set member is a 2-item tuple. `for row, column in ...` unpacks the
    # tuple, assigning its first item to row and second to column each iteration.
    for row, column in grid.searched:
        x, y = column * CELL_PIXELS, row * CELL_PIXELS
        # create_rectangle(x1, y1, x2, y2, ...) uses opposite corners of a box.
        # Here x,y are its upper-left corner; x+CELL_PIXELS,y+CELL_PIXELS are
        # the lower-right. fill colors the inside; outline colors the border.
        target_canvas.create_rectangle(
            x, y, x + CELL_PIXELS, y + CELL_PIXELS, fill="#b9e7c4", outline=""
        )
    for row, column in grid.obstacles:
        x, y = column * CELL_PIXELS, row * CELL_PIXELS
        # Same rectangle method for an obstacle cell, but with dark gray fill.
        target_canvas.create_rectangle(
            x, y, x + CELL_PIXELS, y + CELL_PIXELS, fill="#525a63", outline=""
        )

    # A tile may contain several spill patches. The map is too zoomed out to
    # show their exact positions, so it displays one red marker per tile.
    # Keep every precise sub-tile coordinate in spill_points and in saved JSON.
    spill_tiles = {(item["row"], item["column"]) for item in spill_points}
    for row, column in spill_tiles:
        x, y = map_center(row, column)
        tile_spill_indices = [
            index for index, item in enumerate(spill_points)
            if (item["row"], item["column"]) == (row, column)
        ]
        detected_here = any(index in detected_spill_indices for index in tile_spill_indices)
        target_canvas.create_oval(
            x - 10, y - 10, x + 10, y + 10,
            fill="#e1f5e8" if detected_here else "#fff1f1",
            outline="#20833c" if detected_here else "#c52222", width=2,
        )
        target_canvas.create_text(
            x, y, text="!", fill="#20833c" if detected_here else "#c52222",
            font=("Arial", 13, "bold")
        )

    # While choosing a spill, outline its selected tile so the user can see
    # which 1 ft square the close-up patch click will refer to.
    if target_canvas is canvas and spill_selection_cell is not None:
        row, column = spill_selection_cell
        target_canvas.create_rectangle(
            column * CELL_PIXELS + 2, row * CELL_PIXELS + 2,
            (column + 1) * CELL_PIXELS - 2, (row + 1) * CELL_PIXELS - 2,
            outline="#c52222", width=3, dash=(4, 2),
        )

    # The single map shows completed visit numbers and the selected target.
    if target_canvas is canvas:
        # This is a list comprehension: call map_center for every (row, column)
        # pair in the recorded completion order and collect the results.
        centers = [map_center(r, c) for r, c in grid.order]
        # zip(centers, centers[1:]) pairs each center with the one after it,
        # making consecutive route segments. The final center has no next pair.
        for start, end in zip(centers, centers[1:]):
            # The * operator unpacks a 2-item tuple into two function arguments:
            # create_line(x1, y1, x2, y2, ...), rather than create_line(tuple,...).
            # create_line gets two endpoints (x1,y1,x2,y2). fill sets line color;
            # width is thickness in pixels.
            target_canvas.create_line(*start, *end, fill="#3274b9", width=2)
        # The orange dashed line is the planned map-level route from the
        # robot's last completed square toward its current target.
        route_centers = [map_center(row, col) for row, col in planned_route]
        for start, end in zip(route_centers, route_centers[1:]):
            # dash is a repeating dash/gap length; arrow=tk.LAST puts an arrow
            # head at the final endpoint to show which way this route proceeds.
            target_canvas.create_line(
                *start, *end, fill="#ef8b18", width=2, dash=(5, 3), arrow=tk.LAST
            )
        # enumerate(iterable, start=1) adds a counting number to each cell so
        # the route order displayed to a person begins at 1 instead of 0.
        for number, (row, column) in enumerate(grid.order, start=1):
            x, y = map_center(row, column)
            # create_text(x,y) places a label centered at that pixel position.
            # str(number) converts the integer visit number into display text.
            target_canvas.create_text(x, y, text=str(number), fill="#174923")

        if grid.target_cell is not None:
            row, column = grid.target_cell
            x, y = column * CELL_PIXELS, row * CELL_PIXELS
            # Draw an inset rectangle so the orange outline sits inside the
            # chosen cell; width controls how thick that outline is.
            target_canvas.create_rectangle(
                x + 2, y + 2, x + CELL_PIXELS - 2, y + CELL_PIXELS - 2,
                outline="#ef8b18", width=3,
            )

    # N cells require N+1 boundary lines. Each line is positioned in pixels.
    for column in range(COLUMNS + 1):
        x = column * CELL_PIXELS
        target_canvas.create_line(x, 0, x, MAP_HEIGHT, fill="#9099a2")
    for row in range(ROWS + 1):
        y = row * CELL_PIXELS
        target_canvas.create_line(0, y, MAP_WIDTH, y, fill="#9099a2")

    # Label a full default rack row only when all of its interior cells remain
    # blocked. The outer row/column on every side is the 1 ft perimeter aisle.
    if target_canvas is canvas:
        for row in RACK_ROWS:
            if all((row, col) in grid.obstacles for col in range(1, COLUMNS - 1)):
                target_canvas.create_text(
                    MAP_WIDTH / 2,
                    (row + 0.5) * CELL_PIXELS,
                    text="DATA CENTER RACK ROW",
                    fill="white",
                    font=("Arial", 8, "bold"),
                )

    # The arrow shows the robot's nose/front direction. In this display, angle
    # 0 points right; increasing angle points down because canvas y grows down.
    if show_robot:
        x, y = robot_column * CELL_PIXELS, robot_row * CELL_PIXELS
        # create_polygon draws the rotated 5 in by 7 in body from corner points.
        # Scale 4 px/in converts physical dimensions into map-canvas pixels.
        body_points = robot_body_polygon(x, y, robot_heading, CELL_PIXELS / 12)
        target_canvas.create_polygon(
            *body_points, fill="#f2a0a0", outline="#751919", width=2
        )
        nose_x = x + math.cos(robot_heading) * DOT_RADIUS * 2.4
        nose_y = y + math.sin(robot_heading) * DOT_RADIUS * 2.4
        # create_line draws the robot's heading arrow from its center to its nose.
        # arrow=tk.LAST adds the arrow point at the end of the line.
        target_canvas.create_line(
            x, y, nose_x, nose_y, fill="#171717", width=3, arrow=tk.LAST
        )
        # create_oval takes a bounding box just like create_rectangle; with a
        # square box, the oval is a circle representing the robot's body.
        target_canvas.create_oval(
            x - DOT_RADIUS, y - DOT_RADIUS, x + DOT_RADIUS, y + DOT_RADIUS,
            fill="#d62929", outline="#751919", width=1,
        )


def draw_local_pattern():
    """Draw the planned path inside one enlarged 1 ft square.

    search.py returns fractional local coordinates from 0 to 1. Scaling those
    fractions to this canvas keeps the path independent of display resolution.
    """
    # delete("all") removes every old path/label/dot item from this canvas.
    local_canvas.delete("all")
    # Leave a wide margin around the tile in the zoom. The robot center may
    # travel slightly beyond a tile edge, and its body extends farther; this
    # margin makes those outside-the-tile parts visible instead of clipping them.
    inset = LOCAL_TILE_INSET
    # Draw the outline of one 1 ft square. The two corners define its bounds;
    # fill is the inside color and outline is the border color.
    local_canvas.create_rectangle(
        inset, inset, LOCAL_PIXELS - inset, LOCAL_PIXELS - inset,
        fill="white", outline="#505a64", width=2,
    )

    # Show the active pattern in blue. On completion the red dot starts the
    # next cell's path; the map above records each whole cell as one operation.
    # Before a target is selected, show a preview from the dropdown selection.
    # During two-step spill placement, preview the selected pattern in the
    # selected tile. Otherwise show the active robot path or dropdown preview.
    choosing_spill_patch = spill_mode and spill_selection_cell is not None
    if choosing_spill_patch:
        points = make_search_path(pattern_choice.get())
        view_cell = spill_selection_cell
        view_pattern = pattern_choice.get()
    else:
        points = path_points or make_search_path(pattern_choice.get())
        view_cell = grid.target_cell
        view_pattern = active_pattern
    xy = [
        (inset + x * (LOCAL_PIXELS - 2 * inset),
         inset + y * (LOCAL_PIXELS - 2 * inset))
        for x, y in points
    ]
    if len(xy) > 1:
        # Zigzag has few, meaningful turns, so number all its waypoints. The
        # spiral has many tiny points, so show an arrow every several segments.
        arrow_every = 1 if view_pattern == "Zigzag" else 8
        for index, (start, end) in enumerate(zip(xy, xy[1:])):
            # Here create_line draws one segment between consecutive waypoints.
            # arrow=tk.LAST displays its direction; tk.NONE means no arrowhead.
            local_canvas.create_line(
                *start, *end,
                fill="#3274b9", width=2,
                arrow=tk.LAST if index % arrow_every == 0 else tk.NONE,
            )
            if view_pattern == "Zigzag":
                # create_text(x,y,...) centers the supplied text at this point.
                # font=(family,size) selects a compact font for small step labels.
                local_canvas.create_text(
                    start[0] + 8, start[1] - 7, text=str(index + 1),
                    fill="#245281", font=("Arial", 8),
                )
        # These two create_text calls label the beginning and end of the path.
        # anchor controls which corner of the text sits at its x/y position.
        if view_pattern == "Zigzag":
            local_canvas.create_text(xy[0][0] + 9, xy[0][1] + 10,
                                     text="START", anchor="nw", fill="#147a36",
                                     font=("Arial", 8, "bold"))
            local_canvas.create_text(xy[-1][0] - 8, xy[-1][1] + 10,
                                     text="END", anchor="ne", fill="#7a3ca0",
                                     font=("Arial", 8, "bold"))
        else:
            # The spiral is retraced inward, so it starts and ends at center.
            local_canvas.create_text(xy[0][0] + 9, xy[0][1] + 10,
                                     text="START / END", anchor="nw", fill="#147a36",
                                     font=("Arial", 8, "bold"))

        # Highlight the next waypoint in orange so the user can follow the
        # current instruction against the full blue pattern.
        if (not choosing_spill_patch and view_cell is not None
                and not travel_points and path_index < len(xy)):
            row, column = view_cell
            local_x = robot_column - column
            local_y = robot_row - row
            robot_xy = (inset + local_x * (LOCAL_PIXELS - 2 * inset),
                        inset + local_y * (LOCAL_PIXELS - 2 * inset))
            waypoint = xy[path_index]
            # Highlight the current robot-to-waypoint segment in orange.
            local_canvas.create_line(
                *robot_xy, *waypoint, fill="#ef8b18", width=3, arrow=tk.LAST
            )
            # An oval with a 10x10 pixel bounding box makes a waypoint marker.
            local_canvas.create_oval(
                waypoint[0] - 5, waypoint[1] - 5,
                waypoint[0] + 5, waypoint[1] + 5,
                fill="#ef8b18", outline="",
            )

    # The local preview's arrow/dot mirrors the robot's position and heading
    # relative to its current target cell, when it is inside that cell.
    if grid.target_cell is not None and not choosing_spill_patch:
        local_x = robot_column - grid.target_cell[1]
        local_y = robot_row - grid.target_cell[0]
        # While transiting between map cells, hide the local zoomed robot. During
        # searching, allow it to appear slightly outside the tile boundary.
        # Include the small area beyond each tile edge that the search route
        # uses. The 0.3 fraction is 3.6 in, which fits inside the zoom margin.
        if (not travel_points and -0.3 <= local_x <= 1.3 and -0.3 <= local_y <= 1.3):
            px = inset + local_x * (LOCAL_PIXELS - 2 * inset)
            py = inset + local_y * (LOCAL_PIXELS - 2 * inset)
            # Convert the global heading into this zoom view's pixel scale.
            local_pixels_per_inch = (LOCAL_PIXELS - 2 * inset) / 12
            nose_length = ROBOT_WIDTH_IN * local_pixels_per_inch / 2 + 8
            local_nose_x = px + math.cos(robot_heading) * nose_length
            local_nose_y = py + math.sin(robot_heading) * nose_length
            # This zoom shows the whole body footprint: 5 in along the arrow,
            # 7 in perpendicular to it. At this scale it is a large rectangle.
            local_body = robot_body_polygon(
                px, py, robot_heading, local_pixels_per_inch
            )
            local_canvas.create_polygon(
                *local_body, fill="#f2a0a0", outline="#751919", width=2
            )
            # The close-up uses the same line/arrow idea as the main map, but
            # with a larger pixel length because this is a zoomed-in drawing.
            local_canvas.create_line(px, py, local_nose_x, local_nose_y,
                                     fill="#171717", width=3, arrow=tk.LAST)

    # Draw each saved spill patch in this tile's zoomed view. During spill
    # selection, this is the precise area where the user is about to click.
    if view_cell is not None:
        view_row, view_column = view_cell
        for spill_index, spill in enumerate(spill_points):
            if (spill["row"], spill["column"]) != (view_row, view_column):
                continue
            spill_x = inset + spill["x_fraction"] * (LOCAL_PIXELS - 2 * inset)
            spill_y = inset + spill["y_fraction"] * (LOCAL_PIXELS - 2 * inset)
            local_canvas.create_oval(
                spill_x - 7, spill_y - 7, spill_x + 7, spill_y + 7,
                fill="#20833c" if spill_index in detected_spill_indices else "#e52323",
                outline="white", width=2,
            )
            local_canvas.create_text(
                spill_x, spill_y, text="!", fill="white", font=("Arial", 9, "bold")
            )


def redraw():
    """Refresh the map and close-up pattern view in the same window."""
    draw_map(canvas, show_robot=True)
    draw_local_pattern()
    update_command_readout()
    if active_pattern == "Zigzag":
        pattern_info_text.set(
            "Zigzag: 2 lanes, 5 in apart (2 in overlap) | Robot: 7×5 in, arrow along width"
        )
    else:
        pattern_info_text.set(
            "Circle spiral | Robot: 7×5 in, arrow along width (body shown in red)"
        )
    route_status.set(
        f"Completed: {len(grid.searched)} / {ROWS * COLUMNS - len(grid.obstacles)} "
        f"searchable squares    |    Obstacles: {len(grid.obstacles)}"
        f"    |    Spill patches: {len(spill_points)}"
        f"    |    Simulated detections: {len(leak_detection_events)}"
    )


def vectors_for_waypoints(points):
    """Convert ordered (x, y) waypoints into direction and distance vectors."""
    vectors = []
    for start, end in zip(points, points[1:]):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        vectors.append({
            "dx_ft": round(dx, 4),
            "dy_ft": round(dy, 4),
            "distance_ft": round(math.hypot(dx, dy), 4),
            # Heading convention matches the GUI: zero=right/east, positive
            # angle goes down/south because screen/map y increases downward.
            "heading_degrees": round(math.degrees(math.atan2(dy, dx)) % 360, 2),
        })
    return vectors


def make_path_record(cell, pattern, waypoints):
    """Build a JSON-friendly record for one square's completed search path."""
    # The local waypoints are fractions of a 1 ft square, so multiplying by
    # 1 ft makes their numerical values equal to distances in feet from top-left.
    xy_points = [(x * 1.0, y * 1.0) for x, y in waypoints]
    return {
        "cell_row_1_based": cell[0] + 1,
        "cell_column_1_based": cell[1] + 1,
        "pattern": pattern,
        "waypoints_local_ft": [
            {"x_ft_from_left": round(x, 4), "y_ft_from_top": round(y, 4)}
            for x, y in xy_points
        ],
        "movement_vectors": vectors_for_waypoints(xy_points),
    }


def save_path_plan():
    """Write completed square paths and the active plan to a JSON file.

    JSON is plain text: you can open the file in VS Code or use it later as
    input to another program. Saving is triggered by the toolbar button.
    """
    active_record = None
    if grid.target_cell is not None:
        active_record = make_path_record(grid.target_cell, active_pattern, path_points)
    # Route cells use (row, column); convert to (x, y)=(column, row) so the same
    # vector helper can describe travel between their 1 ft centers.
    route_xy_points = [(column * 1.0, row * 1.0) for row, column in planned_route]
    data = {
        "cell_size_ft": 1,
        "coordinate_convention": {
            "x": "increases right/east",
            "y": "increases down/south in this map display",
            "heading_degrees": "0=east, 90=south, 180=west, 270=north",
        },
        "robot_footprint_in": {
            "length": ROBOT_LENGTH_IN,
            "width_along_forward_arrow": ROBOT_WIDTH_IN,
            "arrow_is_perpendicular_to_length": ROBOT_ARROW_ALONG_WIDTH,
        },
        "uniform_rack_layout": {
            "rack_rows_1_based": [row + 1 for row in RACK_ROWS],
            "rack_depth_ft": 1,
            "clear_aisle_between_racks_ft": 2,
            "clear_perimeter_ft": 1,
        },
        "completed_square_paths": completed_path_records,
        "planned_route_cells_1_based": [
            {"row": row + 1, "column": column + 1}
            for row, column in planned_route
        ],
        "planned_route_movement_vectors": vectors_for_waypoints(route_xy_points),
        "current_square_path": active_record,
        "current_movement_command_vector": movement_command_vector,
        "simulated_leak_detection_radius_ft": SIMULATED_LEAK_RADIUS_FT,
        "leak_detection_events": leak_detection_events,
        "spill_patches": [
            {
                "tile_row_1_based": spill["row"] + 1,
                "tile_column_1_based": spill["column"] + 1,
                "x_fraction_from_tile_left": round(spill["x_fraction"], 4),
                "y_fraction_from_tile_top": round(spill["y_fraction"], 4),
                "x_ft_from_map_left": round(spill["column"] + spill["x_fraction"], 4),
                "y_ft_from_map_top": round(spill["row"] + spill["y_fraction"], 4),
            }
            for spill in spill_points
        ],
    }
    # __file__ is this Python file's path; with_name keeps its folder but swaps
    # the filename, putting the JSON beside main.py.
    output_file = Path(__file__).with_name("search_path_plan.json")
    try:
        # open(mode="w") creates or replaces a text file; encoding chooses UTF-8.
        with output_file.open("w", encoding="utf-8") as file:
            # json.dump turns Python dictionaries/lists into readable JSON text.
            json.dump(data, file, indent=2)
        status_text.set(f"Path plan saved to {output_file.name} in the project folder.")
    except OSError as error:
        status_text.set(f"Could not save the plan: {error}")


def toggle_spill_mode():
    """Switch between obstacle editing and two-click spill placement."""
    global spill_mode, spill_selection_cell
    spill_mode = not spill_mode
    spill_selection_cell = None
    spill_button.configure(text="Add spill: on" if spill_mode else "Add spill: off")
    if spill_mode:
        status_text.set(
            "Spill mode: first click a clear map tile; then click the spill's sub-patch in the zoom view."
        )
    else:
        status_text.set("Obstacle mode: click a map tile to toggle an obstacle.")
    redraw()


def handle_map_click(x, y):
    """Handle a map click according to the currently selected editing mode."""
    row, column = int(y // CELL_PIXELS), int(x // CELL_PIXELS)
    if not (0 <= row < ROWS and 0 <= column < COLUMNS):
        return
    cell = (row, column)
    if spill_mode:
        if cell in grid.obstacles:
            status_text.set("A spill marker cannot be placed on a rack or obstacle tile.")
        else:
            global spill_selection_cell
            spill_selection_cell = cell
            status_text.set(
                f"Selected tile row {row + 1}, column {column + 1}. "
                "Now click the spill's sub-patch in the zoom view."
            )
        redraw()
        return
    toggle_obstacle_at(x, y)


def handle_spill_patch_click(x, y):
    """Store a spill at the clicked sub-patch of the selected 1 ft tile.

    The zoom canvas draws the selected tile between LOCAL_TILE_INSET and its
    opposite margin. Dividing the click's offset by that tile width converts
    screen pixels to fractions from 0 to 1, which can be saved in JSON.
    """
    global spill_selection_cell
    if not spill_mode or spill_selection_cell is None:
        return
    tile_left = LOCAL_TILE_INSET
    tile_right = LOCAL_PIXELS - LOCAL_TILE_INSET
    if not (tile_left <= x <= tile_right and tile_left <= y <= tile_right):
        status_text.set("Click inside the outlined square in the zoom view to place the spill.")
        return

    tile_pixel_width = tile_right - tile_left
    x_fraction = (x - tile_left) / tile_pixel_width
    y_fraction = (y - tile_left) / tile_pixel_width
    row, column = spill_selection_cell
    spill_points.append({
        "row": row,
        "column": column,
        "x_fraction": x_fraction,
        "y_fraction": y_fraction,
    })
    status_text.set(
        f"Added spill sub-patch in row {row + 1}, column {column + 1}. "
        "Click here again for another patch, or select another map tile."
    )
    redraw()


def toggle_obstacle_at(x, y):
    """Convert a mouse click to a cell index and toggle its obstacle state.

    Dividing pixel coordinates by cell width and taking int() rounds down to
    the row and column under the click. The bounds check ignores clicks outside
    the grid. SearchGrid protects completed squares and the current target.
    """
    if hardware_mission_id is not None:
        status_text.set("The robot is executing its declared tile path; pause or stop before editing the map.")
        return
    # // is floor division: 115 // 48 is 2, meaning pixel 115 is in cell 2.
    # int() turns that result into an integer index. The comma assignment
    # unpacks the two results into row and column variables.
    row, column = int(y // CELL_PIXELS), int(x // CELL_PIXELS)
    if 0 <= row < ROWS and 0 <= column < COLUMNS:
        cell = (row, column)
        # Do not let a mouse edit place an obstacle under the robot's planned
        # transit corridor. The target and completed squares are also protected.
        robot_cell = (int(robot_row), int(robot_column))
        if cell in travel_points or cell == robot_cell:
            status_text.set("That square is under the robot or on its current travel path.")
        elif grid.toggle_obstacle(cell):
            redraw()


# ---------------------- Within-square and between-square ---------------------
def choose_next_target():
    """Ask route.py for the next square and prepare its local search pattern.

    This is the hand-off between map-level routing and in-square searching.
    Intermediate cells are for travel only; they are not marked searched unless
    the robot performs their complete local pattern.
    """
    # global tells Python these names refer to the variables at the top of this
    # file; without it, assigning here would create local variables instead.
    global path_points, path_index, travel_points, planned_route, active_pattern
    target = nearest_unsearched(
        grid.current_cell, grid.searched, grid.obstacles, grid.rows, grid.columns
    )
    if target is None:
        grid.target_cell = None
        grid.complete = True
        planned_route = []
        if len(grid.searched) == ROWS * COLUMNS - len(grid.obstacles):
            status_text.set("Search complete: every available 1 ft square was searched.")
        else:
            status_text.set("No reachable unsearched squares remain; clear an obstacle or reset.")
        redraw()
        return

    grid.target_cell = target
    route = path_between(
        grid.current_cell, target, grid.obstacles, grid.rows, grid.columns
    )
    planned_route = route or []
    # Visit each intermediate cell center to route around obstacles. Transit
    # squares are not marked searched; only the selected target gets a pattern.
    # route[1:] is a slice: take every route cell after the current cell because
    # the robot is already at its center. `A if test else B` is a conditional
    # expression; use an empty list when no route exists.
    travel_points = route[1:] if route else []
    active_pattern = pattern_choice.get()
    path_points = make_search_path(active_pattern)
    path_index = 0
    row, column = target
    status_text.set(
        f"Searching row {row + 1}, column {column + 1} (1 ft square); "
        f"completed {len(grid.searched)}."
    )
    redraw()


def change_pattern(*_):
    """Set the in-square pattern for the next square to be searched.

    Tkinter event callbacks receive an event argument. ``*_`` accepts it but
    ignores it. If the robot is already sweeping a square, let it finish its
    current safe path; changing it halfway could require a diagonal reposition
    that makes the rectangular footprint cross the cell boundary. If the robot
    is still traveling between squares, update the upcoming square's pattern.
    """
    global path_points, path_index, active_pattern
    if hardware_mission_id is not None:
        pattern_choice.set(active_pattern)
        status_text.set("The active robot tile pattern is fixed until its mission ends.")
        return
    if grid.target_cell is not None and travel_points:
        # The robot has not started this square yet, so replace its pending path.
        active_pattern = pattern_choice.get()
        path_points = make_search_path(active_pattern)
        path_index = 0
    elif grid.target_cell is not None:
        status_text.set("The selected pattern will be used on the next square; finishing this one first.")
    redraw()


# bind(sequence, callback) registers a function for an event. This event name
# means the user picked a new dropdown item; Tkinter passes that event to callback.
pattern_menu.bind("<<ComboboxSelected>>", change_pattern)


def speed_feet_per_second():
    """Return visualization speed in feet (grid widths) per second."""
    # In this visualization one grid unit is 1 ft.
    return {"Slow": 0.25, "Normal": 0.6, "Fast": 1.2}[speed_choice.get()]


def update_search():
    """Advance one animation frame, then schedule the following frame.

    Tkinter's ``after(milliseconds, function)`` schedules a callback without
    blocking the window. A loop with sleep() would prevent the GUI from handling
    clicks and repainting. Each callback moves toward a destination by a limited
    step, redraws, and schedules the next callback.
    """
    # path_index and main_after_id are reassigned by this callback; global
    # updates the shared app state declared at the top of this file.
    global path_index, main_after_id
    if run_mode != "simulation":
        main_after_id = window.after(FRAME_MS, update_search)
        return
    # Check before any movement in this frame. Once inside the demo detection
    # radius, the simulation latches a stop and waits for operator acknowledgment.
    if check_simulated_leak_detection():
        redraw()
        main_after_id = window.after(FRAME_MS, update_search)
        return

    # Keep a callback scheduled while paused or complete. Resume can continue
    # without accidentally starting a second animation loop.
    if paused or grid.complete:
        main_after_id = window.after(FRAME_MS, update_search)
        return
    if grid.target_cell is None:
        choose_next_target()
        main_after_id = window.after(FRAME_MS, update_search)
        return

    # First travel between cell centers along route.py's obstacle-free route.
    if travel_points:
        next_row, next_column = travel_points[0]
        destination_row, destination_column = next_row + 0.5, next_column + 0.5
        # This helper turns the arrow toward the center, then advances forward.
        if advance_forward_toward(destination_row, destination_column):
            travel_points.pop(0)
        redraw()
        main_after_id = window.after(FRAME_MS, update_search)
        return

    # Now inside the target square: follow its local points from search.py.
    row, column = grid.target_cell
    local_x, local_y = path_points[path_index]
    destination_row = row + local_y
    destination_column = column + local_x
    if advance_forward_toward(destination_row, destination_column):
        path_index += 1
        if path_index >= len(path_points):
            # Only now has the selected full-square pattern finished.
            completed_path_records.append(
                make_path_record(grid.target_cell, active_pattern, path_points)
            )
            grid.complete_target()
            choose_next_target()

    redraw()
    main_after_id = window.after(FRAME_MS, update_search)


def toggle_pause():
    """Pause or resume motion without closing the application window."""
    global paused, leak_stop_latched
    if hardware_mission_id is not None:
        stop_hardware_mission(
            "Operator stopped the robot path; verify the robot stopped physically"
        )
        return
    if run_mode != "simulation":
        serial_status_text.set("Select Simulation and click Start to animate the map")
        return
    if leak_stop_latched:
        # A leak stop is intentionally latched; the operator must acknowledge
        # it before the path planner can issue motion again.
        leak_stop_latched = False
        paused = False
        pause_button.configure(text="Pause")
        status_text.set(
            "Leak acknowledged. Search resumes; that marked spill will not trigger twice."
        )
        redraw()
        return
    paused = not paused
    # configure(option=value) edits an existing button; changing text changes
    # what action the button advertises after this click.
    pause_button.configure(text="Resume" if paused else "Pause")
    if paused:
        status_text.set("Paused. The current 1 ft square is not marked complete yet.")


def reset_search():
    """Clear search progress and restore the default racks and start position."""
    global robot_row, robot_column, robot_heading, path_points, path_index
    global travel_points, planned_route, active_pattern, paused, spill_mode
    global spill_selection_cell
    global leak_stop_latched
    if hardware_mission_id is not None:
        serial_status_text.set("Wait for or stop the active hardware path before resetting")
        return
    if run_mode == "hardware" and hardware_radio_ready:
        serial_status_text.set(
            "Hardware pose cannot be reset in software. Reposition and reboot the robot, then restart the app."
        )
        return
    if run_mode == "local_test" and local_test_started:
        serial_status_text.set(
            "Local test pulses have no wheel feedback. Reposition the robot and restart the app to reset its pose."
        )
        return
    grid.reset()
    # SearchGrid.reset() clears all obstacles, so restore the standard rack
    # layout immediately afterward. Manually toggled obstacles are reset away.
    grid.obstacles.update(DEFAULT_RACK_OBSTACLES)
    robot_row = robot_column = 0.5
    robot_heading = 0.0
    path_points = []
    path_index = 0
    travel_points = []
    planned_route = []
    active_pattern = pattern_choice.get()
    completed_path_records.clear()
    spill_points.clear()
    detected_spill_indices.clear()
    leak_detection_events.clear()
    spill_mode = False
    spill_selection_cell = None
    leak_stop_latched = False
    spill_button.configure(text="Add spill: off")
    paused = True
    # Restore the button's original label after resetting the simulation.
    pause_button.configure(text="Pause")
    status_text.set("Reset complete. Click Start to run the selected mode.")
    choose_next_target()
    redraw()


def refresh_serial_ports():
    """Refresh the COM-port menu from ports currently detected by Windows."""
    global serial_port_map, last_serial_port_signature
    previous_choice = serial_port_choice.get()
    previous_device = serial_port_map.get(previous_choice, previous_choice.split(" —", 1)[0])
    try:
        ports = available_ports()
    except Exception as error:
        serial_port_map = {}
        serial_port_menu.configure(values=())
        serial_port_choice.set("")
        serial_status_text.set("Port scan failed: %s" % error)
        return

    last_serial_port_signature = tuple(
        (port.device, getattr(port, "description", None)) for port in ports
    )

    choices = []
    serial_port_map = {}
    for port_info in ports:
        description = (getattr(port_info, "description", None) or "USB serial device").strip()
        label = "%s — %s" % (port_info.device, description)
        choices.append(label)
        serial_port_map[label] = port_info.device

    serial_port_menu.configure(values=choices)
    matching_choice = next(
        (label for label, device in serial_port_map.items() if device == previous_device),
        None,
    )
    if matching_choice:
        serial_port_choice.set(matching_choice)
    elif choices:
        serial_port_choice.set(choices[0])
    else:
        serial_port_choice.set("")

    if not choices and esp32_link is None:
        serial_status_text.set("No USB serial ports found")


def monitor_serial_port_changes():
    """Notice USB devices plugged in after the app starts, while disconnected."""
    global last_serial_port_signature
    if esp32_link is None:
        try:
            ports = available_ports()
            signature = tuple(
                (port.device, getattr(port, "description", None)) for port in ports
            )
        except Exception:
            signature = None
        if signature is not None and signature != last_serial_port_signature:
            refresh_serial_ports()
    window.after(2000, monitor_serial_port_changes)


def toggle_serial_connection():
    """Connect to or disconnect from the selected USB serial port."""
    global esp32_link, usb_link_ready, hardware_radio_ready
    if esp32_link is not None:
        if hardware_mission_id is not None:
            stop_hardware_mission("USB disconnected; robot mission stopped")
        esp32_link.close()
        esp32_link = None
        usb_link_ready = False
        hardware_radio_ready = False
        pending_ping_tokens.clear()
        serial_connect_button.configure(text="Connect + identify")
        serial_status_text.set("Disconnected")
        append_serial_log("USB serial disconnected")
        return

    port_choice = serial_port_choice.get().strip()
    port = serial_port_map.get(port_choice, port_choice.split(" —", 1)[0])
    if not port:
        refresh_serial_ports()
        serial_status_text.set("Select the ESP32 COM port first")
        return
    try:
        esp32_link = SerialLink(port)
        usb_link_ready = False
        hardware_radio_ready = False
        serial_connect_button.configure(text="Disconnect")
        serial_status_text.set("Connected to %s; waiting for board…" % port)
        append_serial_log("Opening %s" % port)
        window.after(1200, send_connection_ping)
    except Exception as error:
        esp32_link = None
        serial_status_text.set("Could not open %s: %s" % (port, error))


def send_connection_ping():
    """Check USB communication shortly after a COM port opens."""
    global next_ping_id
    if esp32_link is None:
        return
    token = "APP%04d" % next_ping_id
    next_ping_id += 1
    pending_ping_tokens.add(token)
    message = make_ping_message(token)
    if send_esp32_line(message):
        serial_status_text.set("Connected; checking USB communication…")

        def usb_ping_timeout():
            if token in pending_ping_tokens:
                pending_ping_tokens.discard(token)
                serial_status_text.set("No matching USB PONG; check the selected port and bridge firmware")
                if hardware_preflight_stage == "usb":
                    run_mode_button.configure(text="Retry link check", state="normal")

        window.after(3000, usb_ping_timeout)


def start_selected_mode():
    """Start computer simulation, radio robot mode, or local motor hardware test."""
    global run_mode, paused, hardware_preflight_stage, hardware_preflight_token
    global next_robot_ping_id, hardware_radio_ready, local_control_ready
    selected = run_mode_choice.get()
    if hardware_mission_id is not None:
        serial_status_text.set("Wait for the active robot tile to finish before changing modes")
        return
    requested_mode = {
        "Simulation": "simulation",
        "Robot hardware (ESP-NOW)": "hardware",
        "Control-board test (local)": "local_test",
    }.get(selected)
    if (run_mode is not None and requested_mode != run_mode
            and (grid.order or abs(robot_row - 0.5) > 1e-6
                 or abs(robot_column - 0.5) > 1e-6)):
        serial_status_text.set(
            "Reset the map and return the robot to its known start pose before changing run modes"
        )
        return
    if selected == "Simulation":
        if hardware_preflight_token is not None:
            pending_robot_ping_tokens.discard(hardware_preflight_token)
        hardware_preflight_token = None
        hardware_preflight_stage = None
        hardware_radio_ready = False
        local_control_ready = False
        run_mode = "simulation"
        paused = False
        pause_button.configure(text="Pause", state="normal")
        run_mode_button.configure(text="Simulation running")
        status_text.set("SIMULATION MODE: map motion is generated on this computer.")
        serial_status_text.set("Simulation active; no robot movement is being commanded")
        redraw()
        return
    if selected == "Control-board test (local)" and local_test_started:
        serial_status_text.set(
            "One local tile test already ran. Reposition the robot, then restart the app before another run."
        )
        return
    if selected not in ("Robot hardware (ESP-NOW)", "Control-board test (local)"):
        status_text.set("Choose a run mode, then click Start.")
        return
    is_local_test = selected == "Control-board test (local)"
    if fake_robot_mode and not is_local_test:
        serial_status_text.set("Turn off local fake-robot motor mode before robot-path operation")
        return
    run_mode = "local_test" if is_local_test else "hardware"
    paused = True
    hardware_radio_ready = False
    pause_button.configure(text="Robot controlled", state="disabled")
    run_mode_button.configure(text="Checking links…", state="disabled")
    if esp32_link is None:
        hardware_preflight_stage = "usb"
        run_mode_button.configure(text="Start", state="normal")
        serial_status_text.set(
            "Click Connect + identify in the USB link below. This Start button does not open the COM port."
        )
        return
    if not usb_link_ready:
        hardware_preflight_stage = "usb"
        run_mode_button.configure(text="Waiting for USB PONG…", state="normal")
        serial_status_text.set("Robot mode selected. Waiting for matching USB PING/PONG.")
        if not pending_ping_tokens:
            send_connection_ping()
        return
    if is_local_test:
        if active_motor_test_token is not None:
            hardware_preflight_stage = None
            run_mode_button.configure(text="Wait for motor jog", state="normal")
            serial_status_text.set("Stop or wait for the current motor jog before starting a tile test")
            return
        if fake_robot_mode:
            local_control_ready = True
            start_hardware_path()
        else:
            request_local_control_mode()
        return
    if active_motor_test_token is not None:
        hardware_preflight_stage = None
        run_mode_button.configure(text="Wait for motor jog", state="normal")
        serial_status_text.set("Wait for or stop the current motor jog before starting a tile path")
        return
    if hardware_preflight_stage == "radio" and hardware_preflight_token is not None:
        serial_status_text.set("Waiting for the robot radio validation already in progress")
        return
    request_robot_path_preflight()


def request_robot_path_preflight():
    """Ping the robot through the bridge before allowing a hardware mission."""
    global next_robot_ping_id, hardware_preflight_stage, hardware_preflight_token
    if esp32_link is None or not usb_link_ready:
        serial_status_text.set("USB board identification is required before robot radio check")
        return
    token = "RUN%04d" % next_robot_ping_id
    next_robot_ping_id += 1
    hardware_preflight_token = token
    hardware_preflight_stage = "radio"
    pending_robot_ping_tokens.add(token)
    if not send_esp32_line(make_robot_radio_ping_message(token)):
        pending_robot_ping_tokens.discard(token)
        hardware_preflight_stage = None
        run_mode_button.configure(text="Start robot mode", state="normal")
        return
    run_mode_button.configure(text="Checking robot radio…", state="disabled")
    serial_status_text.set("USB link passed; waiting for the robot ESP32 radio reply.")

    def radio_preflight_timeout():
        global hardware_preflight_stage, hardware_preflight_token
        if hardware_preflight_token == token and token in pending_robot_ping_tokens:
            pending_robot_ping_tokens.discard(token)
            hardware_preflight_token = None
            hardware_preflight_stage = None
            run_mode_button.configure(text="Start robot mode", state="normal")
            serial_status_text.set(
                "Robot radio check timed out. Check robot power, channel, firmware, and MAC."
            )

    window.after(2500, radio_preflight_timeout)


def request_local_control_mode():
    """Ask the main-control ESP32 to load its local motor driver for one-tile testing."""
    global hardware_preflight_stage, local_control_ready
    if esp32_link is None or not usb_link_ready:
        serial_status_text.set("USB PING/PONG is required before local motor testing")
        return
    hardware_preflight_stage = "local_mode"
    local_control_ready = False
    if not send_esp32_line("FAKE_ROBOT|ON"):
        hardware_preflight_stage = None
        run_mode_button.configure(text="Retry local test", state="normal")
        return
    run_mode_button.configure(text="Loading local motor driver…", state="disabled")
    serial_status_text.set("Waiting for the control board to confirm LOCAL PATH TEST mode.")

    def local_mode_timeout():
        global hardware_preflight_stage
        if hardware_preflight_stage == "local_mode" and run_mode == "local_test":
            hardware_preflight_stage = None
            run_mode_button.configure(text="Retry local test", state="normal")
            serial_status_text.set(
                "No local-mode confirmation. Check main_control_motor_driver.py on the board."
            )

    window.after(3000, local_mode_timeout)


def append_serial_log(message):
    """Append a received/sent serial event to the optional control window."""
    serial_log_history.append(str(message))
    if len(serial_log_history) > 300:
        del serial_log_history[:-300]
    if control_log is None:
        return
    try:
        control_log.configure(state="normal")
        control_log.insert("end", str(message) + "\n")
        control_log.see("end")
        control_log.configure(state="disabled")
    except tk.TclError:
        pass


def send_esp32_line(message):
    """Send one text-protocol line to the connected main-control ESP32."""
    if esp32_link is None:
        serial_status_text.set("Connect to the main-control ESP32 first")
        append_serial_log("Not sent: USB serial is disconnected")
        return False
    try:
        esp32_link.send_line(message)
        append_serial_log("PC -> ESP32: " + message)
        return True
    except Exception as error:
        serial_status_text.set("Serial send failed: %s" % error)
        append_serial_log("Serial send failed: %s" % error)
        return False


def stop_hardware_mission(message):
    """Clear the queued path after the robot reports an execution error."""
    global hardware_mission_id, hardware_path_commands, hardware_waiting_sequence
    global hardware_waiting_ready, hardware_waiting_tile_done
    global local_control_ready, hardware_preflight_stage
    if hardware_mission_id is not None and esp32_link is not None:
        send_esp32_line("MISSION_STOP|%s" % hardware_mission_id)
    if run_mode == "local_test" and fake_robot_mode and esp32_link is not None:
        hardware_preflight_stage = "local_disable"
        send_esp32_line("FAKE_ROBOT|OFF")
        schedule_local_mode_off_timeout()
        local_control_ready = False
    hardware_mission_id = None
    hardware_path_commands = []
    hardware_waiting_sequence = None
    hardware_waiting_ready = False
    hardware_waiting_tile_done = False
    pause_button.configure(text="Robot controlled", state="disabled")
    run_mode_button.configure(text="Retry selected mode", state="normal")
    serial_status_text.set(message)
    append_serial_log(message)


def send_next_hardware_segment():
    """Send one planned segment and wait for its matching robot DONE reply."""
    global hardware_waiting_sequence, hardware_waiting_tile_done
    if hardware_mission_id is None:
        return
    if hardware_path_index >= len(hardware_path_commands):
        hardware_waiting_tile_done = True
        serial_status_text.set("Segments are finished; waiting for robot TILE_DONE")
        return
    command = hardware_path_commands[hardware_path_index]
    try:
        message = make_segment_message(
            hardware_mission_id, hardware_path_index, command
        )
    except Exception as error:
        stop_hardware_mission("Path command invalid: %s" % error)
        return
    hardware_waiting_sequence = hardware_path_index
    if send_esp32_line(message):
        serial_status_text.set(
            "Sent path segment %d/%d; waiting for robot DONE" % (
                hardware_path_index + 1, len(hardware_path_commands)
            )
        )
        mission_id = hardware_mission_id
        sequence = hardware_waiting_sequence

        def segment_timeout():
            if (hardware_mission_id == mission_id
                    and hardware_waiting_sequence == sequence):
                stop_hardware_mission(
                    "No matching robot DONE for segment %d; mission stopped." % sequence
                )

        window.after(20000, segment_timeout)
    else:
        stop_hardware_mission("Could not send path segment")


def finish_hardware_mission():
    """Advance the tile only after the robot confirms the declared path finished."""
    global hardware_mission_id, hardware_path_commands, hardware_waiting_sequence, paused
    global hardware_waiting_ready, hardware_waiting_tile_done, local_control_ready
    global hardware_preflight_stage
    finished_cell = grid.target_cell
    if finished_cell is not None:
        completed_path_records.append(
            make_path_record(finished_cell, active_pattern, path_points)
        )
        grid.complete_target()
        choose_next_target()
    hardware_mission_id = None
    hardware_path_commands = []
    hardware_waiting_sequence = None
    hardware_waiting_ready = False
    hardware_waiting_tile_done = False
    paused = True
    pause_button.configure(text="Robot controlled", state="disabled")
    if run_mode == "local_test":
        run_mode_button.configure(text="Local tile test complete", state="normal")
        serial_status_text.set(
            "Control board confirmed the timed tile test; disabling local mode and returning to radio bridge."
        )
        append_serial_log("LOCAL TILE_DONE accepted; marked one tile searched")
        if fake_robot_mode and esp32_link is not None:
            local_control_ready = False
            hardware_preflight_stage = "local_disable"
            send_esp32_line("FAKE_ROBOT|OFF")
        schedule_local_mode_off_timeout()
    else:
        run_mode_button.configure(text="Robot running", state="disabled")
        serial_status_text.set("Robot confirmed the tile path; map advanced to the next tile")
        append_serial_log("Robot TILE_DONE confirmation accepted; marked tile searched")
    redraw()
    if run_mode == "hardware" and not grid.complete:
        window.after(400, start_hardware_path)
    elif grid.complete:
        run_mode_button.configure(text="Robot path complete", state="disabled")


def schedule_local_mode_off_timeout():
    """Report when the board does not acknowledge restoration of radio mode."""
    def local_mode_off_timeout():
        if hardware_preflight_stage == "local_disable":
            serial_status_text.set(
                "No MODE|FAKE_ROBOT|OFF reply. Check the USB link and controller before selecting another mode."
            )
            append_serial_log("Local mode-off acknowledgement timed out")
    window.after(3000, local_mode_off_timeout)


def handle_robot_radio_event(line):
    """Process a robot reply relayed by the main-control ESP32."""
    global hardware_path_index, robot_row, robot_column, robot_heading
    global active_motor_test_token
    global hardware_radio_ready, hardware_preflight_stage, hardware_preflight_token
    global hardware_waiting_ready, hardware_waiting_tile_done
    fields = line.split("|", 2)
    if len(fields) < 3 or fields[0] != "RX":
        return
    robot_fields = fields[2].split("|")
    kind = robot_fields[0]

    if kind == "RADIO_PONG" and len(robot_fields) >= 2:
        token = robot_fields[1]
        if token in pending_robot_ping_tokens:
            pending_robot_ping_tokens.discard(token)
            path_status = robot_fields[2] if len(robot_fields) >= 3 else "PATH_STATUS_UNKNOWN"
            pose_status = robot_fields[3] if len(robot_fields) >= 4 else "POSE_STATUS_UNKNOWN"
            message = "Robot ESP32 radio ping passed (%s; %s; %s)." % (
                token, path_status, pose_status
            )
            if token == hardware_preflight_token:
                hardware_preflight_stage = None
                hardware_preflight_token = None
                pose_matches = False
                if path_status == "PATH_READY" and pose_status == "POSE_VALID" and len(robot_fields) >= 7:
                    try:
                        remote_x = float(robot_fields[4])
                        remote_y = float(robot_fields[5])
                        remote_heading = float(robot_fields[6])
                        heading_error = (
                            remote_heading - math.degrees(robot_heading) + 180
                        ) % 360 - 180
                        pose_matches = (
                            all(math.isfinite(value) for value in (remote_x, remote_y, remote_heading))
                            and abs(remote_x - robot_column) <= 0.02
                            and abs(remote_y - robot_row) <= 0.02
                            and abs(heading_error) <= 0.5
                        )
                    except ValueError:
                        pose_matches = False
                hardware_radio_ready = path_status == "PATH_READY" and pose_matches
                if hardware_radio_ready:
                    run_mode_button.configure(text="Robot connected; starting tile…", state="disabled")
                    serial_status_text.set(message + " Starting the first tile mission.")
                    start_hardware_path()
                elif path_status == "PATH_READY" and not pose_matches:
                    run_mode_button.configure(text="Robot pose not synchronized", state="normal")
                    serial_status_text.set(
                        message + " Place the robot at its configured start, reboot it, reset the app map, and reconnect."
                    )
                else:
                    run_mode_button.configure(text="Robot path unavailable", state="normal")
                    serial_status_text.set(
                        message + " Configure calibrated drive speed, turn rate, and motor driver first."
                    )
            if motor_test_status_text is not None:
                motor_test_status_text.set(message)
            append_serial_log(message)
        else:
            append_serial_log("Ignored unmatched robot radio ping: " + token)
        return
    if kind.startswith("MOTOR_TEST_") and len(robot_fields) >= 2:
        is_local = fields[1] == "LOCAL"
        token = robot_fields[1]
        if token not in pending_motor_test_tokens:
            append_serial_log("Ignored unmatched motor-test reply: " + fields[2])
            return
        if kind == "MOTOR_TEST_STARTED":
            duration = robot_fields[2] if len(robot_fields) >= 3 else "?"
            message = "%s jog running (%s ms). Watch the motors." % (
                "Local" if is_local else "Robot", duration
            )
            if motor_test_status_text is not None:
                motor_test_status_text.set(message)
            append_serial_log("Robot acknowledged motor jog %s" % token)
            return
        pending_motor_test_tokens.discard(token)
        if active_motor_test_token == token:
            active_motor_test_token = None
        detail = robot_fields[2] if len(robot_fields) >= 3 else ""
        if kind == "MOTOR_TEST_DONE":
            message = "%s timed jog ended; visually inspect motion (no wheel feedback)." % (
                "Local" if is_local else "Robot"
            )
        elif kind == "MOTOR_TEST_STOPPED":
            message = "%s acknowledged motor stop." % ("Control board" if is_local else "Robot")
        else:
            message = "Motor-test error: %s" % detail
        if motor_test_status_text is not None:
            motor_test_status_text.set(message)
        append_serial_log("Robot %s for %s %s" % (kind, token, detail))
        return

    if kind == "RECEIVED" and len(robot_fields) >= 3:
        mission, sequence = robot_fields[1], robot_fields[2]
        if mission == hardware_mission_id and sequence == str(hardware_waiting_sequence):
            serial_status_text.set("Robot accepted segment %s; executing" % sequence)
        return
    if kind == "MISSION_READY" and len(robot_fields) >= 3:
        mission, count_text = robot_fields[1], robot_fields[2]
        if mission == hardware_mission_id and hardware_waiting_ready:
            try:
                accepted_count = int(count_text)
            except ValueError:
                stop_hardware_mission("Robot returned an invalid MISSION_READY count")
                return
            if accepted_count != len(hardware_path_commands):
                stop_hardware_mission("Robot accepted a different tile segment count")
                return
            hardware_waiting_ready = False
            hardware_waiting_tile_done = False
            append_serial_log("Robot accepted complete tile manifest (%s segments)" % accepted_count)
            send_next_hardware_segment()
        return
    if kind == "TILE_DONE" and len(robot_fields) >= 5:
        mission = robot_fields[1]
        try:
            row, column, count = map(int, robot_fields[2:5])
        except ValueError:
            append_serial_log("Ignored malformed TILE_DONE confirmation")
            return
        if (mission != hardware_mission_id or not hardware_waiting_tile_done
                or hardware_path_index != len(hardware_path_commands)
                or grid.target_cell != (row, column)
                or count != len(hardware_path_commands)):
            append_serial_log("Ignored stale/unmatched TILE_DONE: " + fields[2])
            return
        finish_hardware_mission()
        return
    if kind == "PLAN":
        append_serial_log("Robot wheel plan: " + fields[2])
        return
    if kind == "ERR":
        stop_hardware_mission("Robot rejected command: " + fields[2])
        return
    if kind == "MISSION_STOPPED":
        append_serial_log("Robot confirmed mission stop: " + "|".join(robot_fields[1:]))
        return
    if kind != "DONE" or len(robot_fields) < 3:
        return

    mission, sequence_text = robot_fields[1], robot_fields[2]
    try:
        sequence = int(sequence_text)
    except ValueError:
        return
    if (
        mission != hardware_mission_id
        or sequence != hardware_waiting_sequence
        or sequence != hardware_path_index
    ):
        append_serial_log("Ignored stale/unmatched robot DONE: " + fields[2])
        return

    # DONE confirms the segment schedule ended. Without wheel feedback, this
    # pose remains an open-loop estimate and TILE_DONE is not physical sensing.
    command = hardware_path_commands[hardware_path_index]
    try:
        robot_column = float(robot_fields[3]) if len(robot_fields) >= 6 else command["target_x_ft"]
        robot_row = float(robot_fields[4]) if len(robot_fields) >= 6 else command["target_y_ft"]
        heading_degrees = float(robot_fields[5]) if len(robot_fields) >= 6 else command["global_heading_degrees"]
    except ValueError:
        stop_hardware_mission("Robot returned invalid pose fields with DONE")
        return
    if not all(math.isfinite(value) for value in (robot_column, robot_row, heading_degrees)):
        stop_hardware_mission("Robot returned non-finite pose fields with DONE")
        return
    robot_heading = math.radians(heading_degrees)
    hardware_path_index += 1
    hardware_waiting_sequence = None
    if hardware_path_index >= len(hardware_path_commands):
        hardware_waiting_tile_done = True
        serial_status_text.set("All segments acknowledged; waiting for robot TILE_DONE")
        mission_id = hardware_mission_id

        def tile_done_timeout():
            if hardware_mission_id == mission_id and hardware_waiting_tile_done:
                stop_hardware_mission("Robot did not confirm completion of the tile path")

        window.after(5000, tile_done_timeout)
    else:
        redraw()
        send_next_hardware_segment()


def start_hardware_path():
    """Declare one tile path, then stream after radio or local-mode readiness."""
    global paused
    global hardware_mission_counter, hardware_mission_id
    global hardware_path_commands, hardware_path_index, hardware_waiting_sequence
    global hardware_waiting_ready, hardware_waiting_tile_done
    global local_test_started
    is_local_test = run_mode == "local_test"
    if run_mode not in ("hardware", "local_test"):
        serial_status_text.set("Select a hardware run mode and complete its link checks first")
        return
    if esp32_link is None:
        serial_status_text.set("Connect to the main-control ESP32 first")
        return
    if active_motor_test_token is not None:
        serial_status_text.set("Wait for or stop the active motor jog before a planned path")
        return
    if is_local_test and not (fake_robot_mode and local_control_ready):
        serial_status_text.set("The main-control board has not confirmed local motor-test mode")
        return
    if not is_local_test and fake_robot_mode:
        serial_status_text.set("Turn off fake-robot mode; it directs motor jogs to the control board")
        return
    if not is_local_test and not hardware_radio_ready:
        serial_status_text.set("Robot radio validation is required before starting a path")
        return
    if leak_stop_latched:
        serial_status_text.set("Acknowledge the leak stop before sending a path")
        return
    if hardware_mission_id is not None:
        serial_status_text.set("A hardware path is already in progress")
        return
    if not movement_command_vector:
        update_command_readout()
    if not movement_command_vector:
        serial_status_text.set("There is no planned movement path to send")
        return

    hardware_mission_id = "H%s%04d" % (hardware_session_id, hardware_mission_counter)
    hardware_mission_counter += 1
    hardware_path_commands = [dict(command) for command in movement_command_vector]
    hardware_path_index = 0
    hardware_waiting_sequence = None
    hardware_waiting_ready = True
    hardware_waiting_tile_done = False
    paused = True
    local_test_started = local_test_started or is_local_test
    pause_button.configure(
        text="STOP local test" if is_local_test else "STOP robot path", state="normal"
    )
    run_mode_button.configure(
        text="Pulsing test motors…" if is_local_test else "Robot executing",
        state="disabled",
    )
    redraw()
    append_serial_log(
        "Starting %s with %d segments" % (
            hardware_mission_id, len(hardware_path_commands)
        )
    )
    target_row, target_column = grid.target_cell
    try:
        manifest = make_mission_start_message(
            hardware_mission_id, target_row, target_column, active_pattern,
            len(hardware_path_commands),
        )
    except Exception as error:
        stop_hardware_mission("Could not declare tile path: %s" % error)
        return
    if not send_esp32_line(manifest):
        stop_hardware_mission("Could not send tile mission manifest")
        return
    mission_id = hardware_mission_id

    def ready_timeout():
        if hardware_mission_id == mission_id and hardware_waiting_ready:
            stop_hardware_mission("Robot did not accept the tile mission manifest")

    window.after(5000, ready_timeout)


def poll_serial_events():
    """Move background serial-reader events onto Tkinter's UI thread."""
    global fake_robot_mode, fake_robot_mode_pending
    global usb_link_ready, hardware_preflight_stage, hardware_radio_ready
    global local_control_ready
    if esp32_link is not None:
        while True:
            try:
                line = esp32_link.events.get_nowait()
            except Exception:
                break
            append_serial_log("ESP32 -> PC: " + line)
            if line.startswith("READY|"):
                fake_robot_mode = False
                fake_robot_mode_pending = None
                usb_link_ready = False
                hardware_radio_ready = False
                local_control_ready = False
                if fake_mode_button_text is not None:
                    fake_mode_button_text.set("Enable local motor mode")
                serial_status_text.set("Connected to %s; board ready" % esp32_link.port)
            elif line.startswith("PONG|"):
                token = line.split("|", 1)[1]
                if token in pending_ping_tokens:
                    pending_ping_tokens.discard(token)
                    usb_link_ready = True
                    serial_status_text.set("ESP32 identified; USB ping passed")
                    if hardware_preflight_stage == "usb":
                        if run_mode == "local_test":
                            if fake_robot_mode:
                                local_control_ready = True
                                start_hardware_path()
                            else:
                                request_local_control_mode()
                        else:
                            request_robot_path_preflight()
                else:
                    serial_status_text.set("Unexpected PONG token: %s" % token)
            elif line.startswith("MODE|FAKE_ROBOT|"):
                fake_robot_mode = line.endswith("|ON")
                fake_robot_mode_pending = None
                if run_mode == "local_test" and hardware_preflight_stage == "local_mode" and fake_robot_mode:
                    local_control_ready = True
                    hardware_preflight_stage = None
                    append_serial_log("Control board accepted local path-test mode")
                    start_hardware_path()
                elif run_mode == "local_test" and not fake_robot_mode:
                    local_control_ready = False
                    if hardware_preflight_stage == "local_disable":
                        hardware_preflight_stage = None
                        serial_status_text.set(
                            "Local tile test finished; main-control ESP32 returned to radio-bridge mode."
                        )
                        append_serial_log("Control board returned to radio bridge mode")
                if fake_mode_button_text is not None:
                    fake_mode_button_text.set(
                        "Disable local motor mode" if fake_robot_mode
                        else "Enable local motor mode"
                    )
                if motor_test_status_text is not None:
                    motor_test_status_text.set(
                        "Local motor mode ON: control-board outputs are active."
                        if fake_robot_mode else "Local motor mode OFF; motor jogs use ESP-NOW."
                    )
            elif line.startswith("ERROR|MOTOR_DRIVER|"):
                fake_robot_mode_pending = None
                if run_mode == "local_test":
                    hardware_preflight_stage = None
                    local_control_ready = False
                    run_mode_button.configure(text="Retry local test", state="normal")
                    serial_status_text.set(
                        "Local hardware test could not load the motor driver: "
                        + line.split("|", 2)[2]
                    )
                if motor_test_status_text is not None:
                    motor_test_status_text.set("Fake mode failed: %s" % line.split("|", 2)[2])
            elif line.startswith("RX|"):
                serial_status_text.set(
                    "Local motor-test response received"
                    if line.startswith("RX|LOCAL|") else "Robot radio message received"
                )
                handle_robot_radio_event(line)
            elif line.startswith("TX_FAIL|") or line.startswith("ERROR|"):
                serial_status_text.set(line)
            elif line.startswith("SERIAL_ERROR|"):
                serial_status_text.set(line)
    window.after(100, poll_serial_events)


def open_robot_control():
    """Open task, coordinate, and USB communication controls in a child window."""
    global control_window, control_log, next_task_id, next_ping_id
    global next_motor_test_id, next_robot_ping_id
    global fake_mode_button_text
    global active_motor_test_token, motor_test_status_text
    if control_window is not None and control_window.winfo_exists():
        control_window.deiconify()
        control_window.lift()
        return

    control_window = tk.Toplevel(window)
    control_window.title("ESP32 Robot Control")
    control_window.geometry("680x840")
    control_window.protocol("WM_DELETE_WINDOW", close_robot_control)
    ttk.Label(
        control_window,
        text="Send text commands through the main-control ESP32.",
        padding=(10, 10, 10, 4),
    ).grid(row=0, column=0, sticky="w")

    connection_frame = ttk.Frame(control_window, padding=(10, 0, 10, 8))
    connection_frame.grid(row=1, column=0, sticky="ew")
    ttk.Label(connection_frame, textvariable=serial_status_text).grid(row=0, column=0, sticky="w")

    task_frame = ttk.LabelFrame(control_window, text="Search-square task", padding=10)
    task_frame.grid(row=2, column=0, padx=10, pady=4, sticky="ew")
    ttk.Label(task_frame, text="Task ID").grid(row=0, column=0, sticky="w")
    task_id_var = tk.StringVar(value=str(next_task_id))
    task_id_entry = ttk.Entry(task_frame, textvariable=task_id_var, width=9)
    task_id_entry.grid(row=1, column=0, padx=(0, 8), sticky="w")
    ttk.Label(task_frame, text="Row (0-based)").grid(row=0, column=1, sticky="w")
    row_var = tk.StringVar(value=str(grid.target_cell[0] if grid.target_cell else 0))
    row_entry = ttk.Entry(task_frame, textvariable=row_var, width=9)
    row_entry.grid(row=1, column=1, padx=(0, 8), sticky="w")
    ttk.Label(task_frame, text="Column (0-based)").grid(row=0, column=2, sticky="w")
    column_var = tk.StringVar(value=str(grid.target_cell[1] if grid.target_cell else 0))
    column_entry = ttk.Entry(task_frame, textvariable=column_var, width=9)
    column_entry.grid(row=1, column=2, padx=(0, 8), sticky="w")
    ttk.Label(task_frame, text="Pattern").grid(row=0, column=3, sticky="w")
    task_pattern_var = tk.StringVar(value=pattern_choice.get())
    ttk.Combobox(
        task_frame, textvariable=task_pattern_var,
        values=("Zigzag", "Circle spiral"), state="readonly", width=15,
    ).grid(row=1, column=3, sticky="w")

    def use_active_target():
        if grid.target_cell is not None:
            row_var.set(str(grid.target_cell[0]))
            column_var.set(str(grid.target_cell[1]))
            task_pattern_var.set(pattern_choice.get())

    def send_square_task():
        nonlocal task_id_var
        global next_task_id
        try:
            message = make_task_message(
                task_id_var.get(), row_var.get(), column_var.get(), task_pattern_var.get()
            )
            if send_esp32_line(message):
                next_task_id = int(task_id_var.get()) + 1
                task_id_var.set(str(next_task_id))
        except Exception as error:
            messagebox.showerror("Invalid square task", str(error), parent=control_window)

    ttk.Button(task_frame, text="Use active map target", command=use_active_target).grid(
        row=2, column=0, columnspan=2, pady=(10, 0), sticky="w"
    )
    ttk.Button(task_frame, text="Send task", command=send_square_task).grid(
        row=2, column=3, pady=(10, 0), sticky="e"
    )

    move_frame = ttk.LabelFrame(control_window, text="Coordinate / instruction", padding=10)
    move_frame.grid(row=3, column=0, padx=10, pady=4, sticky="ew")
    move_id_var = tk.StringVar(value=str(next_task_id))
    x_var = tk.StringVar(value="0.50")
    y_var = tk.StringVar(value="0.50")
    heading_var = tk.StringVar(value="AUTO")
    instruction_var = tk.StringVar(value="DRIVE")
    for column, label in enumerate(("Task ID", "X (ft)", "Y (ft)", "Heading (deg)")):
        ttk.Label(move_frame, text=label).grid(row=0, column=column, padx=(0, 6), sticky="w")
    move_id_entry = ttk.Entry(move_frame, textvariable=move_id_var, width=10)
    move_id_entry.grid(row=1, column=0, padx=(0, 6), sticky="w")
    ttk.Entry(move_frame, textvariable=x_var, width=10).grid(row=1, column=1, padx=(0, 6), sticky="w")
    ttk.Entry(move_frame, textvariable=y_var, width=10).grid(row=1, column=2, padx=(0, 6), sticky="w")
    ttk.Entry(move_frame, textvariable=heading_var, width=14).grid(row=1, column=3, sticky="w")
    ttk.Label(move_frame, text="Instruction").grid(row=2, column=0, pady=(8, 0), sticky="w")
    ttk.Entry(move_frame, textvariable=instruction_var, width=32).grid(
        row=3, column=0, columnspan=3, padx=(0, 8), sticky="ew"
    )

    def send_coordinate_instruction():
        global next_task_id
        try:
            message = make_move_message(
                move_id_var.get(), x_var.get(), y_var.get(), heading_var.get(),
                instruction_var.get(),
            )
            if send_esp32_line(message):
                next_task_id = int(move_id_var.get()) + 1
                move_id_var.set(str(next_task_id))
                task_id_var.set(str(next_task_id))
        except Exception as error:
            messagebox.showerror("Invalid coordinate command", str(error), parent=control_window)

    ttk.Button(move_frame, text="Send coordinate command", command=send_coordinate_instruction).grid(
        row=3, column=3, padx=(6, 0), sticky="e"
    )

    motor_frame = ttk.LabelFrame(control_window, text="Robot / local motor jog test", padding=10)
    motor_frame.grid(row=4, column=0, padx=10, pady=4, sticky="ew")
    ttk.Label(
        motor_frame,
        text="Local motor mode routes short jogs to this control board. Control-board test mode also runs a one-tile pulse sequence.",
        wraplength=630,
    ).grid(row=0, column=0, columnspan=5, sticky="w")
    ttk.Label(motor_frame, text="PWM duty (%)").grid(row=1, column=0, pady=(8, 0), sticky="w")
    motor_duty_var = tk.StringVar(value="30")
    ttk.Entry(motor_frame, textvariable=motor_duty_var, width=8).grid(
        row=2, column=0, padx=(0, 10), sticky="w"
    )
    ttk.Label(motor_frame, text="Run time (ms; max 1500)").grid(
        row=1, column=1, pady=(8, 0), sticky="w"
    )
    motor_duration_var = tk.StringVar(value="400")
    ttk.Entry(motor_frame, textvariable=motor_duration_var, width=10).grid(
        row=2, column=1, padx=(0, 10), sticky="w"
    )

    def send_robot_radio_ping():
        global next_robot_ping_id
        if esp32_link is None:
            motor_test_status_text.set("Connect to the main-control ESP32 first.")
            return
        if fake_robot_mode:
            motor_test_status_text.set("Fake robot mode is active; radio ping is not used for local motors.")
            return
        token = "RP%04d" % next_robot_ping_id
        message = make_robot_radio_ping_message(token)
        if send_esp32_line(message):
            next_robot_ping_id += 1
            pending_robot_ping_tokens.add(token)
            motor_test_status_text.set("Robot radio ping sent; waiting for the configured robot MAC.")

            def radio_ping_timeout():
                if token in pending_robot_ping_tokens:
                    pending_robot_ping_tokens.discard(token)
                    if motor_test_status_text is not None:
                        motor_test_status_text.set(
                            "No robot ping reply. Check robot power, channel, firmware, and configured MAC."
                        )

            window.after(1500, radio_ping_timeout)

    fake_mode_button_text = tk.StringVar(value=(
        "Disable local motor mode" if fake_robot_mode else "Enable local motor mode"
    ))

    def toggle_fake_robot_mode():
        global fake_robot_mode_pending
        if hardware_mission_id is not None or run_mode in ("hardware", "local_test"):
            motor_test_status_text.set("Leave Robot hardware mode before changing the local motor-test mode.")
            return
        if esp32_link is None:
            motor_test_status_text.set("Connect to the main-control ESP32 first.")
            return
        if fake_robot_mode_pending is not None:
            motor_test_status_text.set("Waiting for control-board mode confirmation.")
            return
        if active_motor_test_token is not None:
            motor_test_status_text.set("Wait for the active jog or press STOP before changing mode.")
            return
        requested = not fake_robot_mode
        if send_esp32_line("FAKE_ROBOT|%s" % ("ON" if requested else "OFF")):
            fake_robot_mode_pending = requested
            motor_test_status_text.set("Waiting for TFT/control-board mode confirmation...")

            def mode_confirm_timeout():
                global fake_robot_mode_pending
                if fake_robot_mode_pending == requested:
                    fake_robot_mode_pending = None
                    motor_test_status_text.set(
                        "No mode confirmation. Update/restart the control-board firmware and check USB."
                    )

            window.after(2500, mode_confirm_timeout)

    ttk.Button(motor_frame, textvariable=fake_mode_button_text, command=toggle_fake_robot_mode).grid(
        row=2, column=3, padx=(8, 0), sticky="ew"
    )
    ttk.Button(motor_frame, text="Ping robot radio", command=send_robot_radio_ping).grid(
        row=2, column=4, padx=(8, 0), sticky="w"
    )

    def motor_test_reply_timeout(token):
        global active_motor_test_token
        if active_motor_test_token != token:
            return
        append_serial_log("No robot completion reply for motor test %s; sending STOP" % token)
        send_esp32_line(make_motor_stop_message(token))
        if motor_test_status_text is not None:
            motor_test_status_text.set(
                "No completion reply; STOP sent. Waiting for stop acknowledgement."
            )

        def motor_stop_ack_timeout():
            global active_motor_test_token
            if active_motor_test_token != token:
                return
            pending_motor_test_tokens.discard(token)
            active_motor_test_token = None
            if motor_test_status_text is not None:
                motor_test_status_text.set(
                    "No stop reply. Check the robot and remove motor power if needed."
                )

        window.after(1500, motor_stop_ack_timeout)

    def send_motor_jog(direction_a, direction_b, description):
        global next_motor_test_id, active_motor_test_token
        if esp32_link is None:
            if motor_test_status_text is not None:
                motor_test_status_text.set("Connect to the main-control ESP32 first.")
            return
        if fake_robot_mode_pending is not None:
            motor_test_status_text.set("Wait for local motor mode confirmation before jogging.")
            return
        if hardware_mission_id is not None:
            motor_test_status_text.set("Stop or finish the planned path before a motor jog.")
            return
        if active_motor_test_token is not None:
            motor_test_status_text.set("Wait for the active jog or press STOP.")
            return
        try:
            duty_percent = int(motor_duty_var.get())
            duration_ms = int(motor_duration_var.get())
            if not 1 <= duty_percent <= 75:
                raise ValueError("PWM duty must be from 1% through 75%")
            duty_u16 = int(round(duty_percent * 65535 / 100.0))
            token = "J%04d" % next_motor_test_id
            message = make_motor_test_message(
                token, direction_a, direction_b, duty_u16, duration_ms
            )
            if send_esp32_line(message):
                next_motor_test_id += 1
                active_motor_test_token = token
                pending_motor_test_tokens.add(token)
                motor_test_status_text.set("Sent %s; waiting for robot acknowledgement." % description)
                window.after(
                    MAX_MOTOR_TEST_DURATION_MS + 1500,
                    motor_test_reply_timeout,
                    token,
                )
        except Exception as error:
            messagebox.showerror("Invalid motor test", str(error), parent=control_window)

    jog_buttons = (
        ("Forward", 1, 1),
        ("Reverse", -1, -1),
        ("Pivot left", -1, 1),
        ("Pivot right", 1, -1),
        ("Motor A only", 1, 0),
        ("Motor B only", 0, 1),
    )
    for index, (label, direction_a, direction_b) in enumerate(jog_buttons):
        ttk.Button(
            motor_frame,
            text=label,
            command=lambda da=direction_a, db=direction_b, name=label:
                send_motor_jog(da, db, name),
        ).grid(row=3 + index // 4, column=index % 4, padx=(0, 6), pady=(8, 0), sticky="ew")
    ttk.Button(
        motor_frame,
        text="STOP motors",
        command=lambda: send_motor_stop(),
    ).grid(row=4, column=2, columnspan=2, pady=(8, 0), sticky="ew")
    motor_test_status_text = tk.StringVar(
        value=(
            "A jog is still active; wait for its reply or send STOP."
            if active_motor_test_token is not None
            else "Raise the drive wheels for first tests; packets control the configured robot MAC."
        )
    )
    ttk.Label(
        motor_frame,
        textvariable=motor_test_status_text,
        wraplength=630,
    ).grid(row=5, column=0, columnspan=5, pady=(8, 0), sticky="w")
    ttk.Label(
        motor_frame,
        text="The reply confirms a timed command only. Check wheel motion visually; wireless STOP is not an emergency disconnect.",
        wraplength=630,
    ).grid(row=6, column=0, columnspan=5, pady=(4, 0), sticky="w")
    for column in range(5):
        motor_frame.columnconfigure(column, weight=1)

    def send_motor_stop():
        token = active_motor_test_token or "STOP"
        if send_esp32_line(make_motor_stop_message(token)):
            pending_motor_test_tokens.add(token)
            if motor_test_status_text is not None:
                motor_test_status_text.set("STOP packet sent; waiting for robot reply.")

    test_frame = ttk.LabelFrame(control_window, text="USB communication test", padding=10)
    test_frame.grid(row=5, column=0, padx=10, pady=4, sticky="ew")
    ttk.Label(
        test_frame,
        text="PING/PONG verifies the computer ↔ main-control ESP32 USB link.",
    ).grid(row=0, column=0, sticky="w")

    def send_usb_ping():
        global next_ping_id
        try:
            token = "GUI%04d" % next_ping_id
            next_ping_id += 1
            pending_ping_tokens.add(token)
            message = make_ping_message(token)
            if send_esp32_line(message):
                serial_status_text.set("PING sent; waiting for PONG…")
        except Exception as error:
            messagebox.showerror("Communication test", str(error), parent=control_window)

    ttk.Button(test_frame, text="Ping ESP32", command=send_usb_ping).grid(
        row=1, column=0, pady=(8, 0), sticky="w"
    )
    ttk.Label(
        test_frame,
        text="MOVE accepts DRIVE or TURN. Use AUTO to face a DRIVE target; TURN needs a numeric heading.",
        wraplength=510,
    ).grid(row=2, column=0, pady=(6, 0), sticky="w")
    ttk.Button(
        test_frame,
        text="Send active tile path (Robot hardware mode)",
        command=lambda: start_hardware_path(),
    ).grid(row=3, column=0, pady=(8, 0), sticky="w")

    log_frame = ttk.LabelFrame(control_window, text="Serial activity", padding=8)
    log_frame.grid(row=6, column=0, padx=10, pady=(4, 10), sticky="nsew")
    control_window.rowconfigure(6, weight=1)
    control_window.columnconfigure(0, weight=1)
    control_log = tk.Text(log_frame, height=9, width=68, state="disabled", wrap="word")
    control_log.grid(row=0, column=0, sticky="nsew")
    control_log.configure(state="normal")
    for previous_line in serial_log_history:
        control_log.insert("end", previous_line + "\n")
    control_log.see("end")
    control_log.configure(state="disabled")
    log_frame.rowconfigure(0, weight=1)
    log_frame.columnconfigure(0, weight=1)
    if esp32_link is None:
        append_serial_log("Connect to the ESP32 from the main window first.")


def close_robot_control():
    """Close the child window while keeping the main serial connection open."""
    global control_window, control_log, motor_test_status_text
    if active_motor_test_token is not None and esp32_link is not None:
        send_esp32_line(make_motor_stop_message(active_motor_test_token))
    if control_window is not None:
        control_window.destroy()
    control_window = None
    control_log = None
    motor_test_status_text = None


def close_main_window():
    """Cancel the pending timer before destroying the app's windows.

    after_cancel() prevents a scheduled callback from trying to use widgets
    after Tkinter has destroyed them.
    """
    if main_after_id is not None:
        # after_cancel(id) removes a callback that was queued with window.after().
        window.after_cancel(main_after_id)
    if esp32_link is not None:
        if hardware_mission_id is not None:
            send_esp32_line("MISSION_STOP|%s" % hardware_mission_id)
        if run_mode == "local_test" and fake_robot_mode:
            send_esp32_line("FAKE_ROBOT|OFF")
        if active_motor_test_token is not None:
            send_esp32_line(make_motor_stop_message(active_motor_test_token))
        esp32_link.close()
    # destroy() closes this window and releases its Tk widgets.
    window.destroy()


# protocol() replaces the normal window-close action with our cleanup function.
window.protocol("WM_DELETE_WINDOW", close_main_window)
refresh_serial_ports()
window.after(2000, monitor_serial_port_changes)
choose_next_target()
status_text.set("Choose Simulation, Robot hardware, or Control-board test; then click Start.")
redraw()
# after(ms, callback) queues the first animation frame 25 ms from now.
main_after_id = window.after(FRAME_MS, update_search)
window.after(100, poll_serial_events)
# mainloop() is Tkinter's event loop. It keeps the app open and processes input,
# redraw requests, and scheduled after() callbacks until the window is closed.
window.mainloop()

