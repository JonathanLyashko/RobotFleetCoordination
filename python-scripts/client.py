import json
import os
import socket
import threading
import time

import serial
from dotenv import load_dotenv

# ----------------------------
# Configuration
# ----------------------------
load_dotenv()

SERVER_HOST = os.getenv("SERVER_HOST_IP_ADDRESS")
SERVER_PORT = int(os.getenv("SERVER_PORT", "9000"))

SERIAL_PORT = os.getenv("LOCAL_SERIAL_PORT")
SERIAL_BAUD = int(os.getenv("SERIAL_BAUD", "115200"))

ROBOT_ID = os.getenv("ROBOT_ID", "robot_A")
CLIENT_NAME = os.getenv("CLIENT_NAME", f"{ROBOT_ID}_laptop")

# Command types that should be forwarded from server -> robot serial
FORWARD_TO_ROBOT_TYPES = {
    "path_assignment",
    "pause",
    "resume",
    "stop",
    "toggle_gripper",
}

serial_write_lock = threading.Lock()

DEFAULT_TURN_SPEED = 150
DEFAULT_DRIVE_SPEED = 200
SERIAL_RETRY_COUNTS = {
    "pause": 5,
    "stop": 5,
    "resume": 3,
}
SERIAL_RETRY_DELAY_S = 0.03


def send_json_line_over_socket(sock: socket.socket, payload: dict) -> None:
    message = json.dumps(payload, separators=(",", ":")) + "\n"
    sock.sendall(message.encode("utf-8"))


def compact_payload_for_serial(payload: dict) -> dict:
    if payload.get("type") != "path_assignment":
        return payload

    compact = {
        "type": "path_assignment",
        "robot_id": payload.get("robot_id"),
        "path_id": payload.get("path_id"),
        "replace_existing": bool(payload.get("replace_existing", True)),
    }

    waypoints = []
    for waypoint in payload.get("waypoints", []):
        x_cm = waypoint.get("x_cm")
        y_cm = waypoint.get("y_cm")
        if x_cm is None or y_cm is None:
            continue

        waypoints.append({
            "x_cm": int(round(float(x_cm))),
            "y_cm": int(round(float(y_cm))),
        })

    compact["waypoints"] = waypoints

    motion = payload.get("motion")
    if isinstance(motion, dict):
        turn_speed = motion.get("turn_speed_deg_per_sec")
        drive_speed = motion.get("drive_speed_deg_per_sec")

        if turn_speed is not None:
            turn_speed = int(turn_speed)
        if drive_speed is not None:
            drive_speed = int(drive_speed)

        if turn_speed not in {None, DEFAULT_TURN_SPEED} or drive_speed not in {None, DEFAULT_DRIVE_SPEED}:
            compact["motion"] = {}
            if turn_speed is not None:
                compact["motion"]["turn_speed_deg_per_sec"] = turn_speed
            if drive_speed is not None:
                compact["motion"]["drive_speed_deg_per_sec"] = drive_speed

    return compact


def encode_payload_for_serial(payload: dict) -> str:
    msg_type = payload.get("type")

    if msg_type == "pause":
        return "P\n"
    if msg_type == "resume":
        return "R\n"
    if msg_type == "stop":
        return "S\n"

    compact_payload = compact_payload_for_serial(payload)
    return json.dumps(compact_payload, separators=(",", ":")) + "\n"


def send_json_line_over_serial(ser: serial.Serial, payload: dict) -> None:
    message = encode_payload_for_serial(payload)
    retry_count = SERIAL_RETRY_COUNTS.get(payload.get("type"), 1)
    with serial_write_lock:
        for attempt in range(retry_count):
            ser.write(message.encode("utf-8"))
            ser.flush()
            if attempt + 1 < retry_count:
                time.sleep(SERIAL_RETRY_DELAY_S)


def forward_serial_to_server(ser: serial.Serial, sock: socket.socket) -> None:
    """
    Continuously read newline-delimited JSON from serial and forward it to the server.
    """
    while True:
        try:
            line = ser.readline().decode("utf-8", errors="replace").strip()

            if not line:
                continue

            print(f"[SERIAL->TCP raw] {line}")

            # Try to parse so we can validate / enrich if needed
            try:
                msg = json.loads(line)
            except json.JSONDecodeError:
                print(f"[WARN] Ignoring non-JSON serial line: {line}")
                continue

            # Ensure robot identity exists if omitted
            if "robot_id" not in msg:
                msg["robot_id"] = ROBOT_ID

            send_json_line_over_socket(sock, msg)
            print(f"[SERIAL->TCP sent] {msg.get('type')}")

        except Exception as e:
            print(f"[ERROR serial->server] {e}")
            break


def receive_from_server(sock: socket.socket, ser: serial.Serial) -> None:
    """
    Continuously read newline-delimited JSON from server.
    Forward command messages to robot over serial.
    """
    buffer = ""

    while True:
        try:
            data = sock.recv(4096)

            if not data:
                print("Server disconnected")
                break

            buffer += data.decode("utf-8", errors="replace")

            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                line = line.strip()

                if not line:
                    continue

                print(f"[TCP->CLIENT raw] {line}")

                try:
                    msg = json.loads(line)
                except json.JSONDecodeError:
                    print(f"[WARN] Ignoring non-JSON server line: {line}")
                    continue

                msg_type = msg.get("type", "")

                # Handle server hello/ack/etc locally
                if msg_type in {"hello_ack", "ack", "heartbeat_ack", "error"}:
                    print(f"[TCP->CLIENT local] {msg}")
                    continue

                # Forward actual robot commands to serial
                if msg_type in FORWARD_TO_ROBOT_TYPES:
                    send_json_line_over_serial(ser, msg)
                    print(f"[TCP->SERIAL forwarded] {msg_type} to robot")

                else:
                    print(f"[TCP->CLIENT ignored] unknown/unhandled type: {msg_type}")

        except Exception as e:
            print(f"[ERROR server->client] {e}")
            break


def heartbeat_loop(sock: socket.socket) -> None:
    """
    Optional heartbeat from laptop client to arbiter.
    """
    while True:
        try:
            heartbeat = {
                "type": "heartbeat",
                "robot_id": ROBOT_ID,
                "state": "connected",
                "t_ms": int(time.time() * 1000),
            }
            send_json_line_over_socket(sock, heartbeat)
            time.sleep(2.0)
        except Exception as e:
            print(f"[ERROR heartbeat] {e}")
            break


def main() -> None:
    if not SERVER_HOST:
        print("Missing SERVER_HOST_IP_ADDRESS in .env")
        return

    if not SERIAL_PORT:
        print("Missing LOCAL_SERIAL_PORT in .env")
        return

    try:
        # Open serial port to robot
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
        print(f"Opened serial port {SERIAL_PORT} @ {SERIAL_BAUD}")

        # Connect to central arbiter
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((SERVER_HOST, SERVER_PORT))
        print(f"Connected to server {SERVER_HOST}:{SERVER_PORT}")

        # Register this laptop/robot with the arbiter
        hello_msg = {
            "type": "hello",
            "robot_id": ROBOT_ID,
            "client_name": CLIENT_NAME,
            "state": "ready",
        }
        send_json_line_over_socket(sock, hello_msg)
        print(f"[CLIENT->TCP sent] hello for {ROBOT_ID}")

        # Start background receiver for commands from server
        threading.Thread(
            target=receive_from_server,
            args=(sock, ser),
            daemon=True,
        ).start()

        # Optional heartbeat thread
        threading.Thread(
            target=heartbeat_loop,
            args=(sock,),
            daemon=True,
        ).start()

        # Main loop: forward robot telemetry to server
        forward_serial_to_server(ser, sock)

    except serial.SerialException as e:
        print(f"Serial error: {e}")

    except ConnectionRefusedError:
        print("Connection refused. Is the arbiter running?")

    except TimeoutError:
        print("Connection timed out. Check network settings/firewall.")

    except Exception as e:
        print(f"Unexpected error: {e}")


if __name__ == "__main__":
    main()
