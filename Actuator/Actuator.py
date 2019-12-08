# Actuator Server for MSRR

import numpy as np
from Actuator.PF import *

class Actuator:
    def __init__(self):
        self.PF=None
    
    def initPF(self,uc,safe_rate=1.0,finder='JPS',map_size=15):
        self.PF=PathFinder(uc,safe_rate,finder,map_size)
       
    def Dock(self,docks,plot=1,nin=1,tolerent_rate=1.0,opt=1,batch_size=3):
        rps={}
        vts={}
        batch=len(docks)//batch_size+int(len(docks)%batch_size!=0)
        if plot:
            t1=time.time()
            self.PF.plotMap()
        for i in range(batch):
            rp,vt,vmap,vc=self.PF.dock(docks[i*batch_size:min((i+1)*batch_size,len(docks))],plot,nin,tolerent_rate,opt)
            self.PF.updateMap(vmap,vc)
            rps=dictAdd(rps,rp)
            vts=dictAdd(vts,vt)
            if plot:
                self.PF.plotMap(vmap)
        if plot:
            print('Path Planning Time:',time.time()-t1)
        return rps,vts