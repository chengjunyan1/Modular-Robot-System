from Server.Tools import *
import socket

class ServerInterface:
    def __init__(self):
        self.port=59777
        self.host='127.0.0.1' 

    def sendSignal(self,Mid,signal):
        message=str(Mid)+list2str(signal)
        message=str.encode(message)
        s=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((self.host, self.port))
        s.sendall(message)
        s.close()
    
    def sendMessage(self,message):
        message='M'+message
        message=str.encode(message)
        s=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((self.host, self.port))
        s.sendall(message)
        s.close()
