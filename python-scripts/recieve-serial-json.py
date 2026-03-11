import json
import math
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import serial

PORT = "COM5"
BAUD = 9600
OUTPUT_IMAGE = "robot_trajectory.png"

# ------------------------------
# Sensor settings
# ------------------------------
MAX_VALID_ULTRASONIC_CM = 200.0
MIN_VALID_ULTRASONIC_CM = 1.0

# Front sensor: 9.5 cm ahead, centered, facing forward
FRONT_SENSOR_FORWARD_OFFSET_CM = 9.5
FRONT_SENSOR_LATERAL_OFFSET_CM = 0.0
FRONT_SENSOR_ANGLE_OFFSET_DEG = 0.0

# Left sensor: 16 cm to the left of center, facing left
LEFT_SENSOR_FORWARD_OFFSET_CM = 0.0
LEFT_SENSOR_LATERAL_OFFSET_CM = 16.0
LEFT_SENSOR_ANGLE_OFFSET_DEG = 90.0

# ------------------------------
# Telemetry storage
# ------------------------------
times_ms: list[float] = []
xs: list[float] = []
ys: list[float] = []
thetas_deg: list[float] = []

front_ultrasonic_cms: list[float] = []
left_ultrasonic_cms: list[float] = []

# Front sensor obstacle points
front_echo_xs: list[float] = []
front_echo_ys: list[float] = []

# Left sensor obstacle points
left_echo_xs: list[float] = []
left_echo_ys: list[float] = []

# Open serial port
ser = serial.Serial(PORT, BAUD, timeout=1)


def read_serial_packet() -> dict | None:
    """Read one line from serial and parse JSON."""
    line = ser.readline().decode("utf-8", errors="replace").strip()

    if not line:
        return None

    try:
        return json.loads(line)
    except json.JSONDecodeError:
        print(f"Skipped non-JSON line: {line}")
        return None


def world_sensor_position(
    x: float,
    y: float,
    theta_deg: float,
    forward_offset_cm: float,
    lateral_offset_cm: float,
) -> tuple[float, float]:
    """Convert sensor local offset to world position."""
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


def maybe_add_echo_point(
    x: float,
    y: float,
    theta_deg: float,
    ultrasonic_cm: float,
    forward_offset_cm: float,
    lateral_offset_cm: float,
    sensor_angle_offset_deg: float,
    out_xs: list[float],
    out_ys: list[float],
) -> None:
    """Convert ultrasonic reading into a world-space obstacle point."""
    if not (MIN_VALID_ULTRASONIC_CM <= ultrasonic_cm <= MAX_VALID_ULTRASONIC_CM):
        return

    sensor_x, sensor_y = world_sensor_position(
        x, y, theta_deg, forward_offset_cm, lateral_offset_cm
    )

    ray_angle_deg = theta_deg + sensor_angle_offset_deg
    ray_angle_rad = math.radians(ray_angle_deg)

    ex = sensor_x + ultrasonic_cm * math.cos(ray_angle_rad)
    ey = sensor_y + ultrasonic_cm * math.sin(ray_angle_rad)

    out_xs.append(ex)
    out_ys.append(ey)


def draw_sensor_ray(
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
):
    """Draw latest ray for a sensor."""
    if not (MIN_VALID_ULTRASONIC_CM <= ultrasonic_cm <= MAX_VALID_ULTRASONIC_CM):
        return

    sensor_x, sensor_y = world_sensor_position(
        x, y, theta_deg, forward_offset_cm, lateral_offset_cm
    )

    ray_angle_rad = math.radians(theta_deg + sensor_angle_offset_deg)

    ex = sensor_x + ultrasonic_cm * math.cos(ray_angle_rad)
    ey = sensor_y + ultrasonic_cm * math.sin(ray_angle_rad)

    ax.plot([sensor_x, ex], [sensor_y, ey], linestyle=linestyle, label=label)


def draw_trajectory(ax):
    """Draw robot trajectory plus ultrasonic obstacle points."""
    ax.clear()

    ax.set_title("Robot Trajectory + Ultrasonic Points")
    ax.set_xlabel("x (cm)")
    ax.set_ylabel("y (cm)")
    ax.grid(True)
    ax.set_aspect("equal", adjustable="box")

    has_robot = bool(xs and ys)
    has_front_echo = bool(front_echo_xs and front_echo_ys)
    has_left_echo = bool(left_echo_xs and left_echo_ys)

    if has_robot:
        ax.plot(xs, ys, marker="o", linestyle="-", label="robot path")

        ax.annotate("Start", (xs[0], ys[0]))
        ax.annotate("End", (xs[-1], ys[-1]))

        # Current heading arrow
        theta_rad = math.radians(thetas_deg[-1])
        arrow_len = 8.0
        dx = arrow_len * math.cos(theta_rad)
        dy = arrow_len * math.sin(theta_rad)
        ax.arrow(
            xs[-1],
            ys[-1],
            dx,
            dy,
            head_width=3.0,
            head_length=4.0,
            length_includes_head=True,
        )

        # Latest front sensor ray
        draw_sensor_ray(
            ax,
            xs[-1],
            ys[-1],
            thetas_deg[-1],
            front_ultrasonic_cms[-1],
            FRONT_SENSOR_FORWARD_OFFSET_CM,
            FRONT_SENSOR_LATERAL_OFFSET_CM,
            FRONT_SENSOR_ANGLE_OFFSET_DEG,
            label="latest front ray",
            linestyle="--",
        )

        # Latest left sensor ray
        draw_sensor_ray(
            ax,
            xs[-1],
            ys[-1],
            thetas_deg[-1],
            left_ultrasonic_cms[-1],
            LEFT_SENSOR_FORWARD_OFFSET_CM,
            LEFT_SENSOR_LATERAL_OFFSET_CM,
            LEFT_SENSOR_ANGLE_OFFSET_DEG,
            label="latest left ray",
            linestyle=":",
        )

    if has_front_echo:
        ax.scatter(front_echo_xs, front_echo_ys, s=15, alpha=0.7, label="front ultrasonic hits")

    if has_left_echo:
        ax.scatter(left_echo_xs, left_echo_ys, s=15, alpha=0.7, label="left ultrasonic hits")

    all_x = []
    all_y = []

    if has_robot:
        all_x.extend(xs)
        all_y.extend(ys)

    if has_front_echo:
        all_x.extend(front_echo_xs)
        all_y.extend(front_echo_ys)

    if has_left_echo:
        all_x.extend(left_echo_xs)
        all_y.extend(left_echo_ys)

    if all_x and all_y:
        min_x, max_x = min(all_x), max(all_x)
        min_y, max_y = min(all_y), max(all_y)
        pad = 10
        ax.set_xlim(min_x - pad, max_x + pad)
        ax.set_ylim(min_y - pad, max_y + pad)

    ax.legend(loc="best")


def draw_ultrasonic(ax):
    """Draw ultrasonic distances vs time for both sensors."""
    ax.clear()

    ax.set_title("Ultrasonic Distance vs Time")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("distance (cm)")
    ax.grid(True)

    if times_ms and front_ultrasonic_cms and left_ultrasonic_cms:
        times_s = [(t - times_ms[0]) / 1000.0 for t in times_ms]

        ax.plot(times_s, front_ultrasonic_cms, marker="o", linestyle="-", label="front")
        ax.plot(times_s, left_ultrasonic_cms, marker="o", linestyle="-", label="left")

        ax.annotate("Start", (times_s[0], front_ultrasonic_cms[0]))
        ax.annotate("Latest", (times_s[-1], front_ultrasonic_cms[-1]))

        all_u = front_ultrasonic_cms + left_ultrasonic_cms
        min_u = min(all_u)
        max_u = max(all_u)
        pad = max(5, 0.1 * (max_u - min_u + 1))
        ax.set_ylim(min_u - pad, max_u + pad)

        ax.legend(loc="best")


def update(frame):
    """Animation callback: read serial data and redraw plots."""
    packet = read_serial_packet()
    if packet is not None:
        try:
            t_ms = float(packet["t_ms"])
            x = float(packet["x_cm"])
            y = float(packet["y_cm"])
            theta = float(packet["theta_deg"])
            front_ultrasonic = float(packet["front_ultrasonic_cm"])
            left_ultrasonic = float(packet["left_ultrasonic_cm"])
        except (KeyError, ValueError, TypeError) as exc:
            print(f"Bad packet {packet}: {exc}")
            return

        times_ms.append(t_ms)
        xs.append(x)
        ys.append(y)
        thetas_deg.append(theta)
        front_ultrasonic_cms.append(front_ultrasonic)
        left_ultrasonic_cms.append(left_ultrasonic)

        maybe_add_echo_point(
            x, y, theta, front_ultrasonic,
            FRONT_SENSOR_FORWARD_OFFSET_CM,
            FRONT_SENSOR_LATERAL_OFFSET_CM,
            FRONT_SENSOR_ANGLE_OFFSET_DEG,
            front_echo_xs,
            front_echo_ys,
        )

        maybe_add_echo_point(
            x, y, theta, left_ultrasonic,
            LEFT_SENSOR_FORWARD_OFFSET_CM,
            LEFT_SENSOR_LATERAL_OFFSET_CM,
            LEFT_SENSOR_ANGLE_OFFSET_DEG,
            left_echo_xs,
            left_echo_ys,
        )

        print(
            f"Received: t={t_ms:.0f} ms, x={x:.3f}, y={y:.3f}, theta={theta:.3f}, "
            f"front={front_ultrasonic:.3f}, left={left_ultrasonic:.3f}"
        )

    draw_trajectory(ax_traj)
    draw_ultrasonic(ax_ultra)


def save_trajectory():
    """Save final trajectory + ultrasonic image."""
    if not xs or not ys or not times_ms:
        print("No data received, nothing to save.")
        return

    fig2, (ax2_traj, ax2_ultra) = plt.subplots(1, 2, figsize=(14, 7))

    # ----- trajectory + echo points -----
    ax2_traj.set_title("Robot Trajectory + Ultrasonic Points")
    ax2_traj.set_xlabel("x (cm)")
    ax2_traj.set_ylabel("y (cm)")
    ax2_traj.grid(True)
    ax2_traj.set_aspect("equal", adjustable="box")

    ax2_traj.plot(xs, ys, marker="o", linestyle="-", label="robot path")
    ax2_traj.annotate("Start", (xs[0], ys[0]))
    ax2_traj.annotate("End", (xs[-1], ys[-1]))

    theta_rad = math.radians(thetas_deg[-1])
    arrow_len = 8.0
    dx = arrow_len * math.cos(theta_rad)
    dy = arrow_len * math.sin(theta_rad)
    ax2_traj.arrow(
        xs[-1],
        ys[-1],
        dx,
        dy,
        head_width=3.0,
        head_length=4.0,
        length_includes_head=True,
    )

    if front_echo_xs and front_echo_ys:
        ax2_traj.scatter(front_echo_xs, front_echo_ys, s=15, alpha=0.7, label="front ultrasonic hits")

    if left_echo_xs and left_echo_ys:
        ax2_traj.scatter(left_echo_xs, left_echo_ys, s=15, alpha=0.7, label="left ultrasonic hits")

    draw_sensor_ray(
        ax2_traj,
        xs[-1],
        ys[-1],
        thetas_deg[-1],
        front_ultrasonic_cms[-1],
        FRONT_SENSOR_FORWARD_OFFSET_CM,
        FRONT_SENSOR_LATERAL_OFFSET_CM,
        FRONT_SENSOR_ANGLE_OFFSET_DEG,
        label="latest front ray",
        linestyle="--",
    )

    draw_sensor_ray(
        ax2_traj,
        xs[-1],
        ys[-1],
        thetas_deg[-1],
        left_ultrasonic_cms[-1],
        LEFT_SENSOR_FORWARD_OFFSET_CM,
        LEFT_SENSOR_LATERAL_OFFSET_CM,
        LEFT_SENSOR_ANGLE_OFFSET_DEG,
        label="latest left ray",
        linestyle=":",
    )

    all_x = xs + front_echo_xs + left_echo_xs
    all_y = ys + front_echo_ys + left_echo_ys
    min_x, max_x = min(all_x), max(all_x)
    min_y, max_y = min(all_y), max(all_y)
    pad_xy = 10
    ax2_traj.set_xlim(min_x - pad_xy, max_x + pad_xy)
    ax2_traj.set_ylim(min_y - pad_xy, max_y + pad_xy)
    ax2_traj.legend(loc="best")

    # ----- ultrasonic vs time -----
    ax2_ultra.set_title("Ultrasonic Distance vs Time")
    ax2_ultra.set_xlabel("time (s)")
    ax2_ultra.set_ylabel("distance (cm)")
    ax2_ultra.grid(True)

    times_s = [(t - times_ms[0]) / 1000.0 for t in times_ms]
    ax2_ultra.plot(times_s, front_ultrasonic_cms, marker="o", linestyle="-", label="front")
    ax2_ultra.plot(times_s, left_ultrasonic_cms, marker="o", linestyle="-", label="left")
    ax2_ultra.annotate("Start", (times_s[0], front_ultrasonic_cms[0]))
    ax2_ultra.annotate("Latest", (times_s[-1], front_ultrasonic_cms[-1]))

    all_u = front_ultrasonic_cms + left_ultrasonic_cms
    min_u = min(all_u)
    max_u = max(all_u)
    pad_u = max(5, 0.1 * (max_u - min_u + 1))
    ax2_ultra.set_ylim(min_u - pad_u, max_u + pad_u)
    ax2_ultra.legend(loc="best")

    output_path = Path(OUTPUT_IMAGE)
    fig2.tight_layout()
    fig2.savefig(output_path, dpi=200, bbox_inches="tight")
    plt.close(fig2)
    print(f"Saved trajectory image to {output_path.resolve()}")


fig, (ax_traj, ax_ultra) = plt.subplots(1, 2, figsize=(14, 7))
ani = FuncAnimation(fig, update, interval=100, cache_frame_data=False)

try:
    plt.show()
finally:
    save_trajectory()
    ser.close()