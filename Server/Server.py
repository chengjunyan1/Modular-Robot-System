from Tools import *
from ROS import *
import socket

class Server:
    def __init__(self):
        self.host= '127.0.0.1'
        self.port=59777

    def receive(self,signal):
        if signal[0]=="M":
            head='M'
            message=signal[1::]
        else:
            head=eval(signal[0])
            message=eval(signal[1::])
        return head, message

    def listen(self):
        s=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind((self.host,  self.port))
        s.listen(5)
        while True:
            conn, _ = s.accept()
            data = conn.recv(1024)
            Mid, message=self.receive(data.decode())
            if Mid=="M":
                print(message)
            else:
                publish('Module'+str(Mid),str(message)) # publish to ROS
                if message[6]==0 and message[7]==0:
                    print('Module',Mid,'Reached Target Point.')
            conn.close()
        s.close()
        
    def ROSlisten(self):
        listen()
    