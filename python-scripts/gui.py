import queue
import tkinter as tk
from tkinter import ttk
from collections import defaultdict
import math

import matplotlib
matplotlib.use("TkAgg")

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from matplotlib.ticker import MultipleLocator
from matplotlib.patches import Polygon
from itertools import cycle



class TelemetryGUI:
    """
    Thread-safe telemetry dashboard.

    Server threads call:
        gui.update_robot(robot_id, telemetry_dict)

    GUI runs on main thread and consumes queued updates.
    """

    # Sensor geometry
    MAX_VALID_ULTRASONIC_CM = 200.0
    MIN_VALID_ULTRASONIC_CM = 1.0

    FRONT_SENSOR_FORWARD_OFFSET_CM = 9.5
    FRONT_SENSOR_LATERAL_OFFSET_CM = 0.0
    FRONT_SENSOR_ANGLE_OFFSET_DEG = 0.0

    LEFT_SENSOR_FORWARD_OFFSET_CM = 0.0
    LEFT_SENSOR_LATERAL_OFFSET_CM = 16.0
    LEFT_SENSOR_ANGLE_OFFSET_DEG = 90.0

    # Arena display settings: fixed 4m x 4m = 400cm x 400cm
    ARENA_SIZE_CM = 400
    HALF_ARENA_CM = ARENA_SIZE_CM / 2
    GRID_SPACING_CM = 10

    # Robot safety box
    SAFETY_BOX_SIZE_CM = 40.0
    SAFETY_BOX_HALF_CM = SAFETY_BOX_SIZE_CM / 2.0

    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Robot Telemetry Dashboard")
        self.root.geometry("1600x900")

        self.telemetry_queue = queue.Queue()

        # latest per-robot state
        self.robot_states = {}

        # per-robot history
        self.robot_history = defaultdict(lambda: {
            "t": [],
            "x": [],
            "y": [],
            "theta": [],
            "front_ultra": [],
            "left_ultra": [],
            "front_echo_x": [],
            "front_echo_y": [],
            "left_echo_x": [],
            "left_echo_y": [],
        })

        self.robot_colors = {}
        self.color_cycle = cycle([
            "tab:blue",
            "tab:orange",
            "tab:green",
            "tab:red",
            "tab:purple",
            "tab:brown",
            "tab:pink",
            "tab:gray",
            "tab:olive",
            "tab:cyan",
        ])

        self._build_layout()
        self.root.after(100, self._process_queue)

    # ----------------------------
    # Public API
    # ----------------------------
    def update_robot(self, robot_id: str, telemetry: dict):
        self.telemetry_queue.put((robot_id, telemetry))

    def run(self):
        self.root.mainloop()

    # ----------------------------
    # Layout
    # ----------------------------
    def _build_layout(self):
        mainframe = ttk.Frame(self.root, padding=8)
        mainframe.pack(fill=tk.BOTH, expand=True)

        # Left panel: compact robot table
        left_panel = ttk.Frame(mainframe, width=320)
        left_panel.pack(side=tk.LEFT, fill=tk.Y, expand=False)
        left_panel.pack_propagate(False)

        robots_frame = ttk.LabelFrame(left_panel, text="Robots")
        robots_frame.pack(fill=tk.X, expand=False, padx=(0, 8), pady=(0, 8))

        columns = ("robot_id", "state", "x", "y", "theta")
        self.tree = ttk.Treeview(robots_frame, columns=columns, show="headings", height=4)

        self.tree.heading("robot_id", text="robot_id")
        self.tree.heading("state", text="state")
        self.tree.heading("x", text="x")
        self.tree.heading("y", text="y")
        self.tree.heading("theta", text="theta")

        self.tree.column("robot_id", width=80, anchor="center")
        self.tree.column("state", width=70, anchor="center")
        self.tree.column("x", width=50, anchor="center")
        self.tree.column("y", width=50, anchor="center")
        self.tree.column("theta", width=60, anchor="center")

        self.tree.pack(fill=tk.X, expand=False)

        # Right panel: one large arena plot
        right_panel = ttk.Frame(mainframe)
        right_panel.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        self.fig = Figure(figsize=(10, 8), tight_layout=True)
        self.ax_traj = self.fig.add_subplot(111)

        self.canvas = FigureCanvasTkAgg(self.fig, master=right_panel)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
    
    def _get_robot_color(self, robot_id: str) -> str:
        if robot_id not in self.robot_colors:
            self.robot_colors[robot_id] = next(self.color_cycle)
        return self.robot_colors[robot_id]

    def _draw_robot_safety_box(self, ax, x: float, y: float, theta_deg: float, label: str, color: str):
        """
        Draw a rotated 40cm x 40cm safety square centered on the robot.
        """
        half = self.SAFETY_BOX_HALF_CM
        theta_rad = math.radians(theta_deg)

        # Square corners in robot-local coordinates
        local_corners = [
            (-half, -half),
            ( half, -half),
            ( half,  half),
            (-half,  half),
        ]

        world_corners = []
        for lx, ly in local_corners:
            wx = x + lx * math.cos(theta_rad) - ly * math.sin(theta_rad)
            wy = y + lx * math.sin(theta_rad) + ly * math.cos(theta_rad)
            world_corners.append((wx, wy))

        patch = Polygon(
            world_corners,
            closed=True,
            fill=False,
            linewidth=1.5,
            linestyle="-",
            alpha=0.8,
            label=label,
            edgecolor=color,
        )
        ax.add_patch(patch)

    # ----------------------------
    # Queue processing
    # ----------------------------
    def _process_queue(self):
        updated = False

        while not self.telemetry_queue.empty():
            robot_id, telemetry = self.telemetry_queue.get()

            self.robot_states[robot_id] = telemetry
            hist = self.robot_history[robot_id]

            t = telemetry.get("t_ms")
            x = telemetry.get("x_cm")
            y = telemetry.get("y_cm")
            theta = telemetry.get("theta_deg")
            front = telemetry.get("front_ultrasonic_cm")
            left = telemetry.get("left_ultrasonic_cm")

            if t is not None:
                hist["t"].append(float(t) / 1000.0)
            if x is not None:
                hist["x"].append(float(x))
            if y is not None:
                hist["y"].append(float(y))
            if theta is not None:
                hist["theta"].append(float(theta))
            if front is not None:
                hist["front_ultra"].append(float(front))
            if left is not None:
                hist["left_ultra"].append(float(left))

            if x is not None and y is not None and theta is not None:
                x = float(x)
                y = float(y)
                theta = float(theta)

                if front is not None:
                    pt = self._compute_echo_point(
                        x, y, theta, float(front),
                        self.FRONT_SENSOR_FORWARD_OFFSET_CM,
                        self.FRONT_SENSOR_LATERAL_OFFSET_CM,
                        self.FRONT_SENSOR_ANGLE_OFFSET_DEG,
                    )
                    if pt is not None:
                        ex, ey = pt
                        hist["front_echo_x"].append(ex)
                        hist["front_echo_y"].append(ey)

                if left is not None:
                    pt = self._compute_echo_point(
                        x, y, theta, float(left),
                        self.LEFT_SENSOR_FORWARD_OFFSET_CM,
                        self.LEFT_SENSOR_LATERAL_OFFSET_CM,
                        self.LEFT_SENSOR_ANGLE_OFFSET_DEG,
                    )
                    if pt is not None:
                        ex, ey = pt
                        hist["left_echo_x"].append(ex)
                        hist["left_echo_y"].append(ey)

            updated = True

        if updated:
            self._refresh_table()
            self._refresh_plot()

        self.root.after(100, self._process_queue)

    # ----------------------------
    # Math helpers
    # ----------------------------
    def _world_sensor_position(
        self,
        x: float,
        y: float,
        theta_deg: float,
        forward_offset_cm: float,
        lateral_offset_cm: float,
    ):
        theta_rad = math.radians(theta_deg)

        sensor_x = (
            x
            + forward_offset_cm * math.cos(theta_rad)
            - lateral_offset_cm * math.sin(theta_rad)
        )
        sensor_y = (
            y
            + forward_offset_cm * math.sin(theta_rad)
            + lateral_offset_cm * math.cos(theta_rad)
        )
        return sensor_x, sensor_y

    def _compute_echo_point(
        self,
        x: float,
        y: float,
        theta_deg: float,
        ultrasonic_cm: float,
        forward_offset_cm: float,
        lateral_offset_cm: float,
        sensor_angle_offset_deg: float,
    ):
        if not (self.MIN_VALID_ULTRASONIC_CM <= ultrasonic_cm <= self.MAX_VALID_ULTRASONIC_CM):
            return None

        sensor_x, sensor_y = self._world_sensor_position(
            x, y, theta_deg, forward_offset_cm, lateral_offset_cm
        )

        ray_angle_deg = theta_deg + sensor_angle_offset_deg
        ray_angle_rad = math.radians(ray_angle_deg)

        ex = sensor_x + ultrasonic_cm * math.cos(ray_angle_rad)
        ey = sensor_y + ultrasonic_cm * math.sin(ray_angle_rad)

        return ex, ey

    def _draw_latest_ray(
        self,
        ax,
        x: float,
        y: float,
        theta_deg: float,
        ultrasonic_cm: float,
        forward_offset_cm: float,
        lateral_offset_cm: float,
        sensor_angle_offset_deg: float,
        label: str,
        linestyle: str,
        color: str,
    ):
        if not (self.MIN_VALID_ULTRASONIC_CM <= ultrasonic_cm <= self.MAX_VALID_ULTRASONIC_CM):
            return

        sensor_x, sensor_y = self._world_sensor_position(
            x, y, theta_deg, forward_offset_cm, lateral_offset_cm
        )

        ray_angle_rad = math.radians(theta_deg + sensor_angle_offset_deg)
        ex = sensor_x + ultrasonic_cm * math.cos(ray_angle_rad)
        ey = sensor_y + ultrasonic_cm * math.sin(ray_angle_rad)

        ax.plot([sensor_x, ex], [sensor_y, ey], linestyle=linestyle, label=label, color=color)

    # ----------------------------
    # Refresh UI
    # ----------------------------
    def _refresh_table(self):
        self.tree.delete(*self.tree.get_children())

        for robot_id, state in self.robot_states.items():
            self.tree.insert(
                "",
                "end",
                values=(
                    robot_id,
                    state.get("state", ""),
                    round(float(state.get("x_cm", 0.0)), 2),
                    round(float(state.get("y_cm", 0.0)), 2),
                    round(float(state.get("theta_deg", 0.0)), 2),
                ),
            )

    def _refresh_plot(self):
        self.ax_traj.clear()

        self.ax_traj.set_title("Robot Trajectory + Ultrasonic Points")
        self.ax_traj.set_xlabel("x (cm)")
        self.ax_traj.set_ylabel("y (cm)")

        # Arena is 0 → 400 cm
        self.ax_traj.set_xlim(0, self.ARENA_SIZE_CM)
        self.ax_traj.set_ylim(0, self.ARENA_SIZE_CM)

        # Keep physical scale equal
        self.ax_traj.set_aspect("equal", adjustable="box")

        # Major ticks every 50 cm (labels)
        self.ax_traj.xaxis.set_major_locator(MultipleLocator(50))
        self.ax_traj.yaxis.set_major_locator(MultipleLocator(50))

        # Minor ticks every 10 cm (grid squares)
        self.ax_traj.xaxis.set_minor_locator(MultipleLocator(10))
        self.ax_traj.yaxis.set_minor_locator(MultipleLocator(10))

        # Draw grids
        self.ax_traj.grid(which="major", linewidth=1.0)
        self.ax_traj.grid(which="minor", linewidth=0.3)

        for robot_id, hist in self.robot_history.items():
            xs = hist["x"]
            ys = hist["y"]
            color = self._get_robot_color(robot_id)

            if xs and ys:
                self.ax_traj.plot(xs, ys, marker="o", linestyle="-", label=f"{robot_id} path", color=color)

                theta_deg = hist["theta"][-1]
                theta_rad = math.radians(theta_deg)
                arrow_len = 8.0
                dx = arrow_len * math.cos(theta_rad)
                dy = arrow_len * math.sin(theta_rad)

                self.ax_traj.arrow(
                    xs[-1],
                    ys[-1],
                    dx,
                    dy,
                    head_width=3.0,
                    head_length=4.0,
                    length_includes_head=True,
                )

                # Robot center point
                self.ax_traj.scatter(
                    [xs[-1]],
                    [ys[-1]],
                    s=50,
                    label=f"{robot_id} center",
                    zorder=5,
                    color=color,
                )

                # Rotated safety box
                self._draw_robot_safety_box(
                    self.ax_traj,
                    xs[-1],
                    ys[-1],
                    theta_deg,
                    label=f"{robot_id} safety box",
                    color=color,
                )

                if hist["front_ultra"]:
                    self._draw_latest_ray(
                        self.ax_traj,
                        xs[-1],
                        ys[-1],
                        theta_deg,
                        hist["front_ultra"][-1],
                        self.FRONT_SENSOR_FORWARD_OFFSET_CM,
                        self.FRONT_SENSOR_LATERAL_OFFSET_CM,
                        self.FRONT_SENSOR_ANGLE_OFFSET_DEG,
                        label=f"{robot_id} latest front ray",
                        linestyle="--",
                        color=color,
                    )

            if hist["front_echo_x"] and hist["front_echo_y"]:
                self.ax_traj.scatter(
                    hist["front_echo_x"],
                    hist["front_echo_y"],
                    s=20,
                    alpha=0.75,
                    label=f"{robot_id} front hits",
                    color=color,
                )

            if hist["left_echo_x"] and hist["left_echo_y"]:
                self.ax_traj.scatter(
                    hist["left_echo_x"],
                    hist["left_echo_y"],
                    s=20,
                    alpha=0.75,
                    label=f"{robot_id} left hits",
                    color=color,
                )

        handles, labels = self.ax_traj.get_legend_handles_labels()
        unique = dict(zip(labels, handles))
        if unique:
            self.ax_traj.legend(unique.values(), unique.keys(), loc="best")

        self.canvas.draw()