import socket

# Use '0.0.0.0' to listen on all available network interfaces
HOST = '0.0.0.0'
PORT = 65432

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    print(f"[*] Listening on {HOST}:{PORT}")
    conn, addr = s.accept()
    with conn:
        print(f"[*] Accepted connection from: {addr}")
        while True:
            data = conn.recv(1024)
            if not data:
                break
            print(f"  Received: {data.decode()}")
            conn.sendall(data)
