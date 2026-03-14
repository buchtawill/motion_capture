#!../mocap_env/bin/python3
import socket
import sys
import inspect

# -----------------------
# Configurable parameters
# -----------------------
HOST = "0.0.0.0"      # Listen on all interfaces
PORT = 5000           # Change this to whatever port you want


# -----------------------
# Debug print helper
# -----------------------
def log(msg: str):
    func = inspect.stack()[1].function
    print(f"INFO [socket_server.py::{func}] {msg}")


# -----------------------
# Main server function
# -----------------------
def start_server(host=HOST, port=PORT):
    log(f"Starting server on {host}:{port}")

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            s.bind((host, port))
        except OSError as e:
            log(f"Failed to bind: {e}")
            sys.exit(1)

        s.listen()
        log("Server is listening for connections...")

        try:
            while True:
                conn, addr = s.accept()
                with conn:
                    log(f"Connection accepted from {addr}")

                    while True:
                        data = conn.recv(1024)
                        if not data:
                            log("Client disconnected")
                            break

                        decoded = data.decode(errors="replace")
                        log(f"Received data: '{decoded.strip()}'")
        except KeyboardInterrupt:
            log("Control C received, shutting down server")
            sys.exit(0)


# -----------------------
# Entrypoint
# -----------------------
if __name__ == "__main__":
    # Optional: allow setting port as command-line arg
    if len(sys.argv) == 2:
        PORT = int(sys.argv[1])
    start_server()