from MSRR  import *
from configs import *
import sys,cv2,time

init=0 if len(sys.argv)==1 else sys.argv[1]
X=MSRR(init)

"""---------- Instructions ----------"""

X.reconfig(C3)
X.reconfig(C4)
