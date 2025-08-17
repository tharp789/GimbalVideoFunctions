import socket
import urllib.parse

while True:
    try:
        parsed_url = urllib.parse.urlparse("rtsp://192.168.144.25:8554/main.264")
        if parsed_url.scheme != 'rtsp':
            raise ValueError("Invalid RTSP URL")
        host = parsed_url.hostname
        port = parsed_url.port or 554  # Default RTSP port
        # Create a socket connection to the RTSP server
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.settimeout(0.5)
            sock.connect((host, port))
            stream_connected = True     
            print("RTSP server is reachable.")

    except (socket.timeout, socket.error) as e:
        stream_connected = False
        print(f"RTSP server is not reachable: {e}")