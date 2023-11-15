import socket

sock = socket.socket(type=socket.SOCK_DGRAM)
sock.bind(('0.0.0.0', 9999))
while True:
    dat = sock.recv(5000)
    print(dat)
