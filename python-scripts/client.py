import socket
import serial
import threading
import os
from dotenv import load_dotenv

# ----------------------------
# Configuration
# ----------------------------

load_dotenv()

SERVER_PORT = 9000

SERIAL_BAUD = 9600

SERVER_HOST = os.getenv('SERVER_HOST_IP_ADDRESS')
SERIAL_PORT = os.getenv('LOCAL_SERIAL_PORT')


def forward_serial_to_server(ser, sock):
    """
    Continuously read lines from serial and forward them to the server.
    """
    while True:
        try:
            line = ser.readline().decode("utf-8", errors="replace").strip()

            if not line:
                continue

            print(f"[SERIAL] {line}")

            message = line + "\n"
            sock.sendall(message.encode("utf-8"))

        except Exception as e:
            print(f"[ERROR serial->server] {e}")
            break


def receive_from_server(sock):
    """
    Continuously read data from the server and print it.
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
                if line:
                    print(f"[SERVER] {line}")

        except Exception as e:
            print(f"[ERROR server->client] {e}")
            break


def main():
    try:
        # Open serial port
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
        print(f"Opened serial port {SERIAL_PORT}")

        # Connect to server
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((SERVER_HOST, SERVER_PORT))

        print(f"Connected to server {SERVER_HOST}:{SERVER_PORT}")

        # Start thread to receive server messages
        threading.Thread(
            target=receive_from_server,
            args=(sock,),
            daemon=True
        ).start()

        # Forward serial telemetry
        forward_serial_to_server(ser, sock)

    except serial.SerialException as e:
        print(f"Serial error: {e}")

    except ConnectionRefusedError:
        print("Connection refused. Is the arbiter running?")

    except Exception as e:
        print(f"Unexpected error: {e}")


if __name__ == "__main__":
    main()