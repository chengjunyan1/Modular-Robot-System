from MSRR  import *
from configs import *
import sys,cv2,time

init=0 if len(sys.argv)==1 else sys.argv[1]
X=MSRR(init)

"""---------- Instructions ----------"""

X.reconfig(C3)
X.reconfig(C4)

# save_path=os.path.join(root_path,'temp')
# img_path=os.path.join(save_path,'frame.jpg')
# img=cv2.imread(img_path)
# frame_rs=cv2.resize(img, (int(img.shape[1]/2),int(img.shape[0]/2)), interpolation=cv2.INTER_CUBIC)
# cv2.imshow('Path',frame_rs)
# cv2.waitKey(10000)
# print(1111)