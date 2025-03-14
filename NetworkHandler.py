import socket
import json

class NetworkHandler:
    def __init__(self, UDP_IP, SEND_PORT, RECV_PORT):
        self.udp_ip = UDP_IP # Store as instance variable
        self.send_port = SEND_PORT
        self.recv_port = RECV_PORT

        self.send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.recv_sock.bind((self.udp_ip, self.recv_port))
        self.recv_sock.setblocking(False)
    
    def send_force(self, force):
        message = json.dumps(force.tolist())
        self.send_sock.sendto(message.encode(), (self.udp_ip, self.send_port))
    
    def receive_position(self):
        try:
            data, _ = self.recv_sock.recvfrom(1024)
            position, _ = json.loads(data.decode())
            return position
        except BlockingIOError:
            return None
