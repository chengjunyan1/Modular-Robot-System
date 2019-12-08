# Coordinator for MSRR

from Coordinator.artag import *
from configs import *
import numpy as np
import cv2,math,random,time
if not drone_stand_alone:
    from Coordinator.drone import *

class Coordinator:
    def __init__(self):
        self.center={} # coord of each center in FW
        self.corner={} # coord of each corner in FW
        self.theta={} # theta of each FRi wrt FW
        self.root=None
        self.unit=None
        self.scale=None

    def setRoot(self,root):
        self.root=root

    def SetPose(self,option='m18'): # set pose of coordinator, do it when not using drone script
        drone_init(option)

    def Coord(self,img=None): # coord wrt FR of root
        if img is None:
            if drone_stand_alone:
                img=cv2.imread(root_path+'temp/frame.jpg',cv2.IMREAD_GRAYSCALE)
            else:
                img=cap()
        while True:
            try:
                coord=detect(img)
                break
            except:
                print('Coordinate error, try again...')
                return False
        if coord==[]:
            print('Coordinate failed, try again...')
            return False
        tags=[]
        for i in coord:
            tags.append(i.tag_id)
        for i in tags:
            if i not in modules:
                print('Coordinate false, try again...')
                return False
        if self.root not in tags:
            print("Root invalid. Randomly set root.\n")
            self.root=random.choice(tags)
        center_r=self.findCenter(coord,self.root)
        if center_r is None:
            return False
        corner_r=self.findCorner(coord,self.root)
        trans=np.array(center_r) # trans from FW to Froot
        xroot=(np.array(corner_r[2])+np.array(corner_r[1]))/2-trans
        yroot=(np.array(corner_r[2])+np.array(corner_r[3]))/2-trans
        xroot/=np.sqrt(np.sum(xroot*xroot))
        yroot/=np.sqrt(np.sum(yroot*yroot))
        RCW=np.array([xroot,yroot]) # rot from Fcoord to an initial FW
        for i in coord:
            self.center[i.tag_id]=np.dot(RCW,(np.array(i.center)-trans).T).T
            self.corner[i.tag_id]=np.dot(RCW,(np.array(i.corners)-trans).T).T
            xr=(self.corner[i.tag_id][2]+self.corner[i.tag_id][1])/2-self.center[i.tag_id]
            self.theta[i.tag_id]=math.atan2(xr[1], xr[0]) # rot of XR wrt XRoot (which is [1,0] in Froot)
        return self
    
    def unitCoord(self,unit=1,img=None): # change unit to length of module
        success=False
        while not success:
            success=self.Coord(img)
        Lx=0
        Ly=0
        for i in self.center:
            Lx+=(self.corner[self.root][1][0]-self.corner[self.root][0][0])/(unit*len(self.center))
            Ly+=(self.corner[self.root][3][1]-self.corner[self.root][0][1])/(unit*len(self.center))
        unitCoord=Coordinator()
        unitCoord.scale=[Lx,Ly]
        unitCoord.unit=unit
        for i in self.center:
            unitCoord.corner[i]=np.zeros([4,2])
            unitCoord.corner[i][0]=[self.corner[i][0][0]/Lx,self.corner[i][0][1]/Ly]
            unitCoord.corner[i][1]=[self.corner[i][1][0]/Lx,self.corner[i][1][1]/Ly]
            unitCoord.corner[i][2]=[self.corner[i][2][0]/Lx,self.corner[i][2][1]/Ly]
            unitCoord.corner[i][3]=[self.corner[i][3][0]/Lx,self.corner[i][3][1]/Ly]
            unitCoord.center[i]=[self.center[i][0]/Lx,self.center[i][1]/Ly]
            unitCoord.theta[i]=self.theta[i]
        return unitCoord

    def findCenter(self,coord,tag): # find center of tag from coord result
        for i in coord:
            if i.tag_id==tag:
                return i.center
        return None

    def findCorner(self,coord,tag): # find corner of tag from coord result
        for i in coord:
            if i.tag_id==tag:
                return i.corners

    def Print(self):
        print('\nRoot Module:',self.root)
        print('____________________\n')
        for i in self.center:
            print('Moduele',i,' \nCenter:',np.round(self.center[i],3),
            '\nTheta:',round(math.degrees(self.theta[i]),3),
            '(Degrees)\nCorners:\n',np.round(self.corner[i],3),'\n')