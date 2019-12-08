from Server.Server import *
import sys

S=Server()
print('Server Listening')
ros_mode=0 if len(sys.argv)==1 else 1
if ros_mode:
    print('ROS Mode')
    S.ROSlisten()
else:
    print('Server Mode')
    S.listen()