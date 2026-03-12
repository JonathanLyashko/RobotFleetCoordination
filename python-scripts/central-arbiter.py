import json
import socket
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, Optional, Any
from gui import TelemetryGUI

HOST = "0.0.0.0"
PORT = 9000
MAX_CLIENTS = 10


@dataclass
class ClientSession:
    client_id: int
    conn: socket.socket
    addr: tuple
    name: str = ""
    robot_id: Optional[str] = None
    state: str = "connected"
    last_heartbeat: float = field(default_factory=time.time)
    last_telemetry: Optional[dict] = None
    last_status: Optional[dict] = None
    current_path_id: Optional[str] = None


clients_lock = threading.Lock()

# keyed by client_id
client_sessions: Dict[int, ClientSession] = {}

# keyed by robot_id
robots_by_id: Dict[str, int] = {}

next_client_id = 1

# Initiate GUI
gui = TelemetryGUI()


def send_json(conn: socket.socket, message: dict) -> None:
    data = json.dumps(message) + "\n"
    conn.sendall(data.encode("utf-8"))


def recv_lines(conn: socket.socket):
    buffer = ""
    while True:
        data = conn.recv(4096)
        if not data:
            break
        buffer += data.decode("utf-8", errors="replace")
        while "\n" in buffer:
            line, buffer = buffer.split("\n", 1)
            line = line.strip()
            if line:
                yield line


def get_session(client_id: int) -> Optional[ClientSession]:
    with clients_lock:
        return client_sessions.get(client_id)


def update_robot_mapping(client_id: int, robot_id: str) -> None:
    with clients_lock:
        # If some old session claimed this robot_id, overwrite it
        robots_by_id[robot_id] = client_id
        session = client_sessions.get(client_id)
        if session:
            session.robot_id = robot_id


def send_to_robot(robot_id: str, message: dict) -> bool:
    with clients_lock:
        client_id = robots_by_id.get(robot_id)
        if client_id is None:
            return False
        session = client_sessions.get(client_id)
        if session is None:
            return False
        conn = session.conn

    try:
        send_json(conn, message)
        return True
    except Exception as exc:
        print(f"[!] Failed to send to robot {robot_id}: {exc}")
        return False


def print_robot_table() -> None:
    with clients_lock:
        print("\n=== Robot Table ===")
        if not client_sessions:
            print("(no clients connected)")
        for client_id, session in client_sessions.items():
            print(
                f"client_id={client_id} "
                f"robot_id={session.robot_id} "
                f"name={session.name!r} "
                f"state={session.state!r} "
                f"path_id={session.current_path_id!r} "
                f"last_heartbeat={session.last_heartbeat:.1f}"
            )
        print("===================\n")


def handle_hello(client_id: int, msg: dict, conn: socket.socket) -> None:
    name = msg.get("name", f"client_{client_id}")
    robot_id = msg.get("robot_id")

    with clients_lock:
        session = client_sessions[client_id]
        session.name = name

        if robot_id is not None:
            session.robot_id = str(robot_id)
            robots_by_id[session.robot_id] = client_id

        # Optional initial state
        if "state" in msg:
            session.state = str(msg["state"])

    print(f"[Client {client_id}] registered name={name!r}, robot_id={robot_id!r}")

    send_json(conn, {
        "type": "ack",
        "for": "hello",
        "client_id": client_id,
        "robot_id": robot_id,
    })


def handle_telemetry(client_id: int, msg: dict) -> None:
    with clients_lock:
        session = client_sessions[client_id]

        # Allow robot_id to be reasserted on telemetry
        robot_id = msg.get("robot_id")
        if robot_id is not None:
            robot_id = str(robot_id)
            session.robot_id = robot_id
            robots_by_id[robot_id] = client_id

        # Optional robot state piggybacked on telemetry
        if "state" in msg:
            session.state = str(msg["state"])

        if "path_id" in msg:
            session.current_path_id = str(msg["path_id"])

        session.last_telemetry = msg

    if session.robot_id:
        gui.update_robot(session.robot_id, msg)

    robot_label = session.robot_id if session.robot_id else f"client_{client_id}"
    print(f"[{robot_label}] telemetry: {msg}")


def handle_status(client_id: int, msg: dict) -> None:
    with clients_lock:
        session = client_sessions[client_id]

        robot_id = msg.get("robot_id")
        if robot_id is not None:
            robot_id = str(robot_id)
            session.robot_id = robot_id
            robots_by_id[robot_id] = client_id

        if "state" in msg:
            session.state = str(msg["state"])

        if "path_id" in msg:
            session.current_path_id = str(msg["path_id"])

        session.last_status = msg

    robot_label = session.robot_id if session.robot_id else f"client_{client_id}"
    print(f"[{robot_label}] status: {msg}")


def handle_ack(client_id: int, msg: dict) -> None:
    session = get_session(client_id)
    robot_label = session.robot_id if session and session.robot_id else f"client_{client_id}"
    print(f"[{robot_label}] ack: {msg}")


def handle_heartbeat(client_id: int, conn: socket.socket) -> None:
    with clients_lock:
        session = client_sessions[client_id]
        session.last_heartbeat = time.time()
        robot_id = session.robot_id

    send_json(conn, {
        "type": "heartbeat_ack",
        "robot_id": robot_id,
        "server_t": time.time(),
    })


def handle_client(client_id: int, conn: socket.socket, addr) -> None:
    print(f"[+] Client {client_id} connected from {addr}")

    try:
        send_json(conn, {
            "type": "hello_ack",
            "client_id": client_id,
            "message": "connected",
        })

        for line in recv_lines(conn):
            try:
                msg = json.loads(line)
            except json.JSONDecodeError:
                print(f"[Client {client_id}] Bad JSON: {line}")
                continue

            msg_type = msg.get("type", "")

            if msg_type == "hello":
                handle_hello(client_id, msg, conn)

            elif msg_type == "telemetry":
                handle_telemetry(client_id, msg)

            elif msg_type == "heartbeat":
                handle_heartbeat(client_id, conn)

            elif msg_type == "status":
                handle_status(client_id, msg)

            elif msg_type == "ack":
                handle_ack(client_id, msg)

            else:
                print(f"[Client {client_id}] unknown message type: {msg_type}")
                send_json(conn, {
                    "type": "error",
                    "message": f"unknown message type: {msg_type}",
                })

    except ConnectionResetError:
        print(f"[!] Client {client_id} connection reset")
    except Exception as exc:
        print(f"[!] Client {client_id} error: {exc}")
    finally:
        with clients_lock:
            session = client_sessions.pop(client_id, None)
            if session and session.robot_id:
                # Remove robot mapping only if it still points to this session
                mapped_client_id = robots_by_id.get(session.robot_id)
                if mapped_client_id == client_id:
                    robots_by_id.pop(session.robot_id, None)

        conn.close()
        print(f"[-] Client {client_id} disconnected")
        print_robot_table()


def accept_loop(server_sock: socket.socket) -> None:
    global next_client_id

    while True:
        conn, addr = server_sock.accept()

        with clients_lock:
            if len(client_sessions) >= MAX_CLIENTS:
                send_json(conn, {
                    "type": "error",
                    "message": "server full",
                })
                conn.close()
                continue

            client_id = next_client_id
            next_client_id += 1

            client_sessions[client_id] = ClientSession(
                client_id=client_id,
                conn=conn,
                addr=addr,
            )

        thread = threading.Thread(
            target=handle_client,
            args=(client_id, conn, addr),
            daemon=True,
        )
        thread.start()
        print_robot_table()

def server_main() -> None:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_sock:
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_sock.bind((HOST, PORT))
        server_sock.listen()
        print(f"Server listening on {HOST}:{PORT}")

        accept_loop(server_sock)


def main() -> None:
    server_thread = threading.Thread(target=server_main, daemon=True)
    server_thread.start()

    gui.run()


if __name__ == "__main__":
    main()