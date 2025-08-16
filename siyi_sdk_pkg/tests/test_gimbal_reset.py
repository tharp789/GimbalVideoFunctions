import serial
import time
import socket

# Final command packet (as per your description)
# AA 01 02 01 55 01 00 D0 2E 11 11 01 66
gimbal_on_packet = bytes([
    0xAA, 0x01, 0x02, 0x01,
    0x55, 0x01, 0x00, 0xD0,
    0x2E, 0x11, 0x11, 0x01,
    0x66
])

# Set up the serial port
ser = serial.Serial(
    port='COM3',       # Change to your actual port
    baudrate=115200,   # Adjust baudrate to match device
    timeout=1
)

gimbal_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_ip = '192.168.144.25'
server_port = 37260

print("Sending gimbal reset command...")
gimbal_socket.sendto(gimbal_on_packet, (server_ip, server_port))

# Wait for a response
response = gimbal_socket.recv(1024)
print("Response from gimbal:", response)

