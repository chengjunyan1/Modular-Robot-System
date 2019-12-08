# Path Finder for MSRR

import numpy as np
import matplotlib.pyplot as plt
import copy,os,time,cv2
from itertools import permutations
from Actuator.pathfinders import JPS,AStar
from configs import *

class PathFinder:
    def __init__(self,uc,safe_rate=1.0,finder='JPS',map_size=15): # scale: length of unit
        self.unit=uc.unit
        self.scale=uc.scale
        self.map_size=map_size
        edge=self.unit*self.map_size
        self.map=np.zeros([2*edge,2*edge])
        self.center=uc.center
        self.corner=uc.corner
        self.theta=uc.theta
        self.safe_rate=safe_rate # ratio of robot length to apriltag length
        self.coordMap()
        if finder=='JPS':
            self.Finder=JPS.JPS
        elif finder=='A*':
            self.Finder=AStar.AStar
        
    def coordMap(self):
        edge=self.unit*self.map_size
        for i in self.center:
            x=round(self.center[i][0])+edge
            y=round(self.center[i][1])+edge
            xl=x-self.unit*self.safe_rate
            xh=x+self.unit*self.safe_rate
            yl=y-self.unit*self.safe_rate
            yh=y+self.unit*self.safe_rate
            for a in range(int(xl),int(xh)+1):
                for b in range(int(yl),int(yh)+1):
                    if a<edge*2 and b<edge*2:
                        self.map[a,b]=1
    
    def updateMap(self,vmap,vc):
        self.map=vmap
        self.center=vc
    
    def findPath(self,sp,ep,move=0,plot=1,nin=1,opt=1): # nin: no interact
        seq=list(range(len(sp)))
        if opt:
            per=list(permutations(seq))
        else:
            per=[list(range(len(ep)))]
        minpath=[]
        mincost=9999999
        minsq=[]
        for s in per:
            success=1
            pathi=[]
            costi=0
            sqi=list(s)
            for i in seq:
                spi=sp[i]
                epi=ep[s[i]]
                spi=[round(spi[0]),round(spi[1])]
                epi=[round(epi[0]),round(epi[1])]
                if move==0:
                    vmap=copy.deepcopy(self.map)
                else:
                    vmap,_=self.moveMap([move[i]])
                P=self.Finder(vmap)
                P.findPath(spi,epi)
                path=P.path
                if path==[]:
                    success=0
                    break
                else:
                    costi+=len(path)
                    pathi.append(path)
                    if nin:
                        vmap=self.pathMap(path,vmap)
            if costi<mincost and success:
                minpath=pathi
                mincost=costi
                minsq=sqi
        if plot:
            paths=[]
            for i in minpath:
                paths+=i
            self.plotPath(paths)
        RealPath=[]
        for i in minpath:
            RealPath.append(self.recoverPath(i))
        return RealPath,minsq

    def move(self,ids,eps,plot=1,nin=1,rsq=0,opt=1):
        sps=[]
        edge=self.unit*self.map_size
        for i in ids:
            x=round(self.center[i][0])+edge
            y=round(self.center[i][1])+edge
            sps.append([x,y])
        vep=[]
        for i in eps:
            vep.append([i[0]+edge,i[1]+edge])
        RealPath,sq=self.findPath(sps,vep,ids,plot=plot,nin=nin,opt=opt)
        if RealPath==[]:
            print('Path Planning Failed, No Path')
            return False
        vmap,vc=self.moveMap(ids)
        rp={}
        for i in range(len(ids)):
            vc[ids[i]]=[eps[sq[i]][0],eps[sq[i]][1]]
        for i in range(len(sq)):
            rp[ids[i]]=RealPath[sq[i]]
        vmap=self.vcMap(vc)
        if rsq:
            return rp,vmap,vc,sq
        return rp,vmap,vc
    
    def dock(self,docks,plot=1,nin=1,tolerent_rate=1.0,opt=1): # docks: [M1 M2 F1 F2 ori rev], same in planner
        ids=[]
        eps=[]
        vts=[]
        for i in docks:
            vc,vt=self.dockPoint(i,tolerent_rate=tolerent_rate) # safe_rate: tolerent of whether close to target 
            ids.append(i[0])
            eps.append(vc)
            vts.append(vt)
        rp,vmap,vc,sq=self.move(ids,eps,plot,nin,rsq=1,opt=opt)
        vt={}
        for i in range(len(sq)):
            vt[docks[i][0]]=vts[sq[i]]
        return rp,vt,vmap,vc
    
    def plotMap(self,vmap=None):
        if vmap is None:
            vmap=self.map
        plt.imshow(1-vmap,interpolation='nearest',cmap='bone')
        plt.xticks(())
        plt.yticks(())
        save_path=os.path.join(root_path,'temp')
        img_path=os.path.join(save_path,'path.jpg')
        plt.savefig(img_path)
        img=cv2.imread(img_path)
        img_rs=cv2.resize(img, (int(img.shape[1]/2),int(img.shape[0]/2)), interpolation=cv2.INTER_CUBIC)
        cv2.imshow('Path',img_rs)
        cv2.waitKey(2000)
    
    def plotPath(self,path):
        edge=self.unit*self.map_size
        pmap=copy.deepcopy(self.map)
        for i in path:
            xl=i[0]-self.unit//4
            xh=i[0]+self.unit//4
            yl=i[1]-self.unit//4
            yh=i[1]+self.unit//4
            for a in range(int(xl),int(xh)+1):
                for b in range(int(yl),int(yh)+1):
                    if a<edge*2 and b<edge*2:
                        pmap[a,b]=1
        self.plotMap(pmap)
    
    def pathMap(self,path,vmap=None): # for no intersect path
        if vmap is None:
            vmap=self.map
        edge=self.unit*self.map_size
        pmap=copy.deepcopy(vmap)
        for i in path:
            x=round(i[0])
            y=round(i[1])
            xl=x-self.unit*self.safe_rate
            xh=x+self.unit*self.safe_rate
            yl=y-self.unit*self.safe_rate
            yh=y+self.unit*self.safe_rate
            for a in range(int(xl),int(xh)+1):
                for b in range(int(yl),int(yh)+1):
                    if a<edge*2 and b<edge*2:
                        pmap[a,b]=1
        return pmap
    
    def moveMap(self,moved,vmap=None,safe_rate=None): # update map by removing moving module
        if vmap is None:
            vmap=self.map
        if safe_rate is None:
            safe_rate=self.safe_rate
        edge=self.unit*self.map_size
        mmap=np.zeros([2*edge,2*edge])
        vc={}
        for i in self.center:
            if i not in moved:
                vc[i]=self.center[i]
                x=round(self.center[i][0])+edge
                y=round(self.center[i][1])+edge
                xl=x-self.unit*safe_rate
                xh=x+self.unit*safe_rate
                yl=y-self.unit*safe_rate
                yh=y+self.unit*safe_rate
                for a in range(int(xl),int(xh)+1):
                    for b in range(int(yl),int(yh)+1):
                        if a<edge*2 and b<edge*2:
                            mmap[a,b]=1
        return mmap,vc
    
    def vcMap(self,vc,safe_rate=None): # add new centers to map
        if safe_rate is None:
            safe_rate=self.safe_rate
        edge=self.unit*self.map_size
        vcmap=np.zeros([2*edge,2*edge])
        for i in vc:
            x=round(vc[i][0])+edge
            y=round(vc[i][1])+edge
            xl=x-self.unit*safe_rate
            xh=x+self.unit*safe_rate
            yl=y-self.unit*safe_rate
            yh=y+self.unit*safe_rate
            for a in range(int(xl),int(xh)+1):
                for b in range(int(yl),int(yh)+1):
                    if a<edge*2 and b<edge*2:
                        vcmap[a,b]=1
        return vcmap
    
    def recoverPath(self,Path):
        RealPath=[]
        for i in Path:
            RealPath.append(self.recoverPoint(i))
        return RealPath
    
    def recoverPoint(self,Point):
        edge=self.unit*self.map_size
        return [(Point[0]-edge)*self.scale[0],(Point[1]-edge)*self.scale[1]]
    
    def dockPoint(self,dock,tolerent_rate=1.0): # Face id: 0:Top 1:Left 2:Right 3:Bottom 
        M1=dock[0]
        M2=dock[1]
        F1=dock[2]
        F2=dock[3]
        #ori=dock[4] # not use now, just for same as in planner
        rev=dock[5]
        Tcenter=self.center[M2]
        Tcorner=self.corner[M2]
        coord=[0,0,0,0]
        coord[0]=2*((Tcorner[2]+Tcorner[3])/2-Tcenter)*tolerent_rate+Tcenter # up
        coord[1]=2*((Tcorner[0]+Tcorner[1])/2-Tcenter)*tolerent_rate+Tcenter # down
        coord[2]=2*((Tcorner[0]+Tcorner[3])/2-Tcenter)*tolerent_rate+Tcenter # left
        coord[3]=2*((Tcorner[2]+Tcorner[1])/2-Tcenter)*tolerent_rate+Tcenter # right
        relRot=relativeRot([F1,F2],rev)
        relCoord=relativeCoord(relRot,F1)
        vTheta=relRot*3.14/2+self.theta[M2]
        vTheta=vTheta if vTheta<3.14 else vTheta-6.28
        vCoord=coord[relCoord]
        return vCoord,vTheta

#------------ utils ------------#

def relativeRot(c,rev=0): # M1 to M2, when M1 try to dock M2
    if c==[0,0] or c==[1,1] or c==[2,2] or c==[3,3]:
        return (2+2*rev) # 180
    elif c==[0,3] or c==[3,0] or c==[1,2] or c==[2,1]:
        return (0+2*rev) # 0
    elif c==[0,2] or c==[1,0] or c==[2,3] or c==[3,1]:
        return (1+2*rev) # 90
    elif c==[0,1] or c==[1,3] or c==[2,0] or c==[3,2]:
        return (3+2*rev) # 270
    
def relativeCoord(Rot,c):
    if Rot%4==0:
        C=[1,3,2,0]
    elif Rot%4==2:
        C=[0,2,3,1]
    elif Rot%4==1:
        C=[3,0,1,2]
    elif Rot%4==3:
        C=[2,1,0,3]
    return C[c]
    
def dictAdd(a,b):
    d={}
    for i in a:
        d[i]=a[i]
    for i in b:
        d[i]=b[i]
    return d