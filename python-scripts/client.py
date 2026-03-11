import socket

# Use the IP address from your ipconfig output
HOST = '10.36.18.242' 
PORT = 65432

# Create a TCP/IP socket
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    try:
        # Connect to the server
        s.connect((HOST, PORT))
        print(f"Successfully connected to {HOST}:{PORT}")
        
        # Send a message (must be encoded to bytes)
        message = "Hello from the other device!"
        s.sendall(message.encode())
        
        # Wait for a response from the server
        data = s.recv(1024)
        print(f"Received from server: {data.decode()}")
        
    except ConnectionRefusedError:
        print("Error: The connection was refused. Is the server script running?")
    except TimeoutError:
        print("Error: Connection timed out. Check your firewall or network settings.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
