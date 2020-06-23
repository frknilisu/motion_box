import socket
import json
import sys

BUFSIZE = 4096


def _send(socket, send_data):
    #print("Sending..")
    json_data = json.JSONEncoder().encode(send_data)
    socket.sendall(json_data.encode())


def _recv(socket):
    print("Receiving..")
    recv_data = socket.recv(BUFSIZE)
    json_data = json.loads(recv_data.decode())

    return json_data


class Server(object):
    backlog = 1
    client = None

    def __init__(self, host='127.0.0.1', port=65432):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Error for using port
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((host, port))
        self.socket.listen(self.backlog)

    def __del__(self):
        self.close()

    def accept(self):
        if self.client:
            self.client.close()

        self.client, self.client_addr = self.socket.accept()
        print("Accepted connection from: {}".format(self.client_addr))

    def send(self, data):
        if not self.client:
            raise Exception('Cannot send data, no client is connected.')

        _send(self.client, data)

    def recv(self):
        if not self.client:
            raise Exception('Cannot receive data, no client is connected.')

        return _recv(self.client)

    def close(self):
        if self.client:
            self.client.close()
            self.client = None

        if self.socket:
            self.socket.close()
            self.socket = None


class Client(object):
    socket = None

    def __init__(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def __del__(self):
        self.close()

    def connect(self, host, port):
        self.socket.connect((host, port))
        return self

    def send(self, data):
        if not self.socket:
            raise Exception('You have to connect first before sending data.')

        _send(self.socket, data)
        return self

    def recv(self):
        if not self.socket:
            raise Exception('You have to connect first before receiving data.')

        return _recv(self.socket)

    def close(self):
        if self.socket:
            self.socket.close()
            self.socket = None
