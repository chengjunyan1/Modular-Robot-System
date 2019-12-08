# -*- coding: UTF-8 -*-

import olympe
import requests
import json
import time,tempfile,os,cv2
from olympe.messages.ardrone3.Piloting import *
from olympe.messages.ardrone3.PilotingState import *
from olympe.messages.gimbal import *
from olympe.messages.camera import *
from configs import *

global drone
if connect_sim_drone:
    drone = olympe.Drone("10.202.0.1")
    drone.connection()
elif connect_real_drone:
    pass

""" Piloting  """

def  takeoff():
    global drone
    drone(
        TakeOff()
        >> FlyingStateChanged(state="hovering", _timeout=5)
    ).wait()

def landing():
    global drone
    drone(Landing()).wait()

def movexy(value):
    global drone
    drone(
        moveBy(value[0],value[1],0,0) # x y z yaw
        >> FlyingStateChanged(state="hovering", _timeout=5)
    ).wait()

def updown(value):
    global drone
    drone(
        moveBy(0,0,-value,0) # x y z yaw
        >> FlyingStateChanged(state="hovering", _timeout=5)
    ).wait()

def drone_yaw(value):
    global drone
    drone(
        moveBy(0,0,0,value) # x y z yaw
        >> FlyingStateChanged(state="hovering", _timeout=5)
    ).wait()

""" Camera & Gimbal """

def camera_init():
    global drone
    drone(set_camera_mode(cam_id=0,value=1)).wait()
    drone(
        set_photo_mode(
            cam_id=0, 
            mode=0, 
            format=1,  # 0:full 1:rectangle(not work)
            file_format=0, 
            burst=0,  # ignored in single shot
            bracketing=0,  # ignored in single shot
            capture_interval=0, # ignored in single shot
        )
    ).wait()

def gimbal(degree):
    global drone
    drone(set_target(
        gimbal_id=0,
        control_mode="position",
        yaw_frame_of_reference="absolute", 
        yaw=0.0,
        pitch_frame_of_reference="absolute",
        pitch=degree,
        roll_frame_of_reference="absolute",  
        roll=0.0,
    )).wait()

def capture():
    global drone
    drone(take_photo(cam_id=0)).wait()
    root_url="http://10.202.0.1"
    time.sleep(1) 
    media_url=root_url+"/api/v1/media/medias"
    response= requests.get(media_url)
    medias=json.loads(response.text)
    capture_url=root_url+medias[-1]['resources'][0]['url']
    response= requests.get(capture_url)
    new_capture = requests.get(capture_url).content
    return new_capture

""" Utils """

def d2r(d):
    return d*3.14/180


#---------- Quick Controls ----------# 

def cap(save=0):
    new_capture=capture()
    if save:
        capture_path='temp/capture.jpg'
        img_path=root_path+capture_path
        with open(img_path, 'wb') as file:
            file.write(new_capture)
    nparr = np.fromstring(new_capture, np.uint8)
    img = cv2.imdecode(nparr,cv2.IMREAD_GRAYSCALE)
    return img

def dx(x):
    movexy([x,0])

def dy(y):
    movexy([0,y])

def dt(t): # dtheta, unit: degree
    drone_yaw(d2r(t))

#---------- Quick Init ----------# 

def drone_init(option='m18'):
    if option=='m15':
        m15()
    elif option=='m18':
        m18()

def m15(): # radius 0.5m
    takeoff()
    gimbal(-90)
    camera_init()
    updown(1.5)
    movexy([-0.13,0])
    time.sleep(3)

def m18(): # radius: 0.6m safe: 0.5
    takeoff()
    gimbal(-90)
    camera_init()
    updown(1.8)
    movexy([-0.13,0])
    time.sleep(3)