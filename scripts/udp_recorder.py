import socket
import pickle

sock = socket.socket(type=socket.SOCK_DGRAM)
sock.bind(('0.0.0.0', 9999))
data = b''
try:
    while True:
        data += sock.recv(4096)
        print(f'{len(data)} bytes received')
finally:
    with open('lidar.dat', 'wb') as file:
        file.write(data)
