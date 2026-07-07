import socket

UDP_IP = "127.0.0.1"
UDP_PORT = 5005

server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

server_socket.bind(
