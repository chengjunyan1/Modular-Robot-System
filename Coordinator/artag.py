import Coordinator.apriltags3 as apriltags
from configs import *

global detector
detector = apriltags.Detector(searchpath=[root_path+'Coordinator/apriltags/lib'],
                        families='tag16h5',
                        nthreads=1,
                        quad_decimate=1.0,
                        quad_sigma=0.0,
                        refine_edges=1,
                        decode_sharpening=0.25,
                        debug=0)

def detect(img):
    global detector
    return detector.detect(img)