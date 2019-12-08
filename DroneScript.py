import csv
import cv2
import os,sys
import tempfile
import time
from configs import root_path

import olympe
from olympe.messages.ardrone3.Piloting import TakeOff
from olympe.messages.ardrone3.Piloting import moveBy
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged
from olympe.messages.gimbal import *
global frame_coun,save_number
frame_count=0
save_number=10

class DroneStreaming:

    def __init__(self):
        self.drone = olympe.Drone(
            "10.202.0.1",
            loglevel=3,
        )
        self.tempd = tempfile.mkdtemp(prefix="olympe_streaming_test_")
        print("Olympe streaming example output dir: {}".format(self.tempd))
        self.h264_frame_stats = []
        self.h264_stats_file = open(
            os.path.join(self.tempd, 'h264_stats.csv'), 'w+')
        self.h264_stats_writer = csv.DictWriter(
            self.h264_stats_file, ['fps', 'bitrate'])
        self.h264_stats_writer.writeheader()

    def start(self):
        self.drone.connection()
        self.drone.set_streaming_output_files(
            h264_data_file=os.path.join(self.tempd, 'h264_data.264'),
            h264_meta_file=os.path.join(self.tempd, 'h264_metadata.json'),
        )
        self.drone.set_streaming_callbacks(
            raw_cb=self.yuv_frame_cb,
        )
        self.drone.start_video_streaming()

    def stop(self):
        self.drone.stop_video_streaming()
        self.drone.disconnection()
        self.h264_stats_file.close()

    def yuv_frame_cb(self, yuv_frame):
        info = yuv_frame.info()
        cv2_cvt_color_flag = {
            olympe.PDRAW_YUV_FORMAT_I420: cv2.COLOR_YUV2BGR_I420,
            olympe.PDRAW_YUV_FORMAT_NV12: cv2.COLOR_YUV2BGR_NV12,
        }[info["yuv"]["format"]]
        cv2frame = cv2.cvtColor(yuv_frame.as_ndarray(), cv2_cvt_color_flag)
        frame_rs=cv2.resize(cv2frame, (int(cv2frame.shape[1]/2),int(cv2frame.shape[0]/2)), interpolation=cv2.INTER_CUBIC)
        cv2.imshow("Drone Streaming", frame_rs)
        cv2.waitKey(1) 
        global frame_count,save_number
        frame_count+=1
        if frame_count==save_number:
            cv2.imwrite(root_path+'temp/frame.jpg', cv2frame)
            frame_count=0

    def init(self):    
        self.drone(
            TakeOff()
            >> FlyingStateChanged(state="hovering", _timeout=5)
        ).wait()   
        self.drone(set_target(
            gimbal_id=0,
            control_mode="position",
            yaw_frame_of_reference="absolute", 
            yaw=0.0,
            pitch_frame_of_reference="absolute",
            pitch=-90,
            roll_frame_of_reference="absolute",  
            roll=0.0,
        )).wait()
        self.drone(
            moveBy(0,0,-1.8,0) # x y z yaw
            >> FlyingStateChanged(state="hovering", _timeout=5)
        ).wait()
        self.drone(
            moveBy(-0.13,0,0,0) # x y z yaw
            >> FlyingStateChanged(state="hovering", _timeout=5)
        ).wait()

    def hovering(self):
        while True:
            self.drone(
                moveBy(0,0,0,0) # x y z yaw
                >> FlyingStateChanged(state="hovering", _timeout=5)
            ).wait()

def run(option):
    streaming = DroneStreaming()
    streaming.start()
    if option:
        streaming.init()
    streaming.hovering()

init=0 if len(sys.argv)==1 else sys.argv[1]
run(init)