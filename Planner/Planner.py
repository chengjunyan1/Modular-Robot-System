# High-Level Planners for MSRR

from Planner.MFSC import *
from Planner.MSRP import *
from Planner.MRGR import *
from Planner.UT import *
from Planner.VT import *
import random

class Planner:
    def __init__(self):
        self.M=M()

    def Config(self,C):
        C(self.M)

    def SAssembly(self,rdcb=0):
        self.SA=[] # Self-Assembly plan   
        for e in self.M.PBreak:
            self.M.delE(e,0)
        if self.M.GP==[]:
            print('\n*Present configurations not found')
            return -1
        GPid=0
        for gp in self.M.GP:
            GPStart=listSum(self.M.NP[:GPid])
            gp.Troot()
            SAi=[]
            VH=[]
            H=gp.H
            Hnum=max(H)+1
            for i in range(Hnum):
                SAi.append([])
                VH.append([])
            for i in range(gp.Vnum):
                VH[H[i]].append(i)
            SAi[0]=VH[0][0]+GPStart
            for i in range(1,Hnum):
                for j in VH[i]:
                    for k in VH[i-1]:
                        e=gp.reqE(j+GPStart,k+GPStart)
                        if e!=-1:
                            SAi[i].append(e)
            if self.M.PBreak!=[]:
                CR=[]
                for i in self.M.PBreak:
                    v0=i[0]
                    v1=i[1]
                    if v0 in gp.Vid and v1 in gp.Vid:
                        CR.append(i)
                SAi.append(CR)
            if SAi[-1]==[]:
                SAi=SAi[:-1]
            self.SA.append(SAi)
            GPid+=1
        for e in self.M.PBreak:
            self.M.addE(e,0)
                        
    def RPlanner(self,msc=0,rdcb=0,detail=0,fast=1,rdmp=0,update=1): # recommend: rdmp off, fast recommend on, others off if can
        error=0
        if self.M.GP==[]:
            print('\n*Present configurations not found')
            error=1
        if self.M.GT==[]:
            print('\n*Target configurations not found')
            error=1
        if error:
            return -1   
        for e in self.M.PBreak:
            self.M.delE(e,0)
        for e in self.M.TBreak:
            self.M.delE(e,1)
        self.M.buildTree()
        self.M.R=CSC(self.M)
        if msc:
            self.M.F=MSC(self.M,self.M.R)
        else:
            self.M.F=MFS(self.M.R)
        self.SR=SRP(self.M,fast=fast,rdmp=rdmp,detail=detail,msc=msc)
        for e in self.M.PBreak:
            self.M.addE(e,0)
        for e in self.M.TBreak:
            self.M.addE(e,1)
        if update:
            self.M.updateGP() # after reconfig, set GT as GP, set GT as null

    def cycleDetect(self):
        self.M.PCycle=[]
        self.M.TCycle=[]
        for i in self.M.GP:
            pc=bruteCycle(i.E)
            if pc!=[]:
                self.M.PCycle+=pc
        for i in self.M.GT:
            tc=bruteCycle(i.E)
            if tc!=[]:
                self.M.TCycle+=tc
    
    def cycleBreak(self,fast=1): # !!!cycle break should be done in topo setting, and dont do it after coord 
        self.cycleDetect()
        self.M.PBreak=[]
        self.M.TBreak=[]
        PBS=cycleBreak(self.M.PCycle)
        TBS=cycleBreak(self.M.TCycle)
        Max=0
        if fast:
            self.M.PBreak=random.choice(PBS)
            self.M.TBreak=random.choice(TBS)
        else:
            for pb in PBS:
                for tb in TBS:
                    breakp=[]
                    breakt=[]
                    for e in pb:
                        breakp.append(e)
                        self.M.delE(e,0)
                    for e in tb:
                        breakt.append(e)
                        self.M.delE(e,1)
                    if not (breakp==[] and breakt==[]):
                        self.M.Treset()
                        RFsize=sizeRF(self.M)
                        if RFsize>Max:
                            self.M.PBreak=breakp
                            self.M.TBreak=breakt
                            Max=RFsize
                        for e in breakp:
                            self.M.addE(e,0)
                        for e in breakt:
                            self.M.addE(e,1)

    def initTopo(self,rdcb=0): # init Topo by a default topo
        trying=1
        while trying:
            self.cycleBreak(rdcb)    
            for e in self.M.PBreak:
                self.M.delE(e,0)
            for e in self.M.TBreak:
                self.M.delE(e,1)
            self.M.buildTree()
            if self.M.checkTree():
                trying=0
            else:
                for e in self.M.PBreak:
                    self.M.addE(e,0)
                for e in self.M.TBreak:
                    self.M.addE(e,1)
        self.M.PTopo=defaultTopo(self.M.GP)
        self.M.TTopo=defaultTopo(self.M.GT)
        for e in self.M.PBreak:
            self.M.addE(e,0)
        for e in self.M.TBreak:
            self.M.addE(e,1)
    
    def coordTopo(self,Coord,rdcb=0): # get present Topo based on Coord (should trans to module unit)
        trying=1
        while trying:
            self.cycleBreak(rdcb)    
            for e in self.M.PBreak:
                self.M.delE(e,0)
            for e in self.M.TBreak:
                self.M.delE(e,1)
            self.M.buildTree()
            if self.M.checkTree():
                trying=0
            else:
                for e in self.M.PBreak:
                    self.M.addE(e,0)
                for e in self.M.TBreak:
                    self.M.addE(e,1)
        self.M.PTopo=[]
        for i in self.M.GP:
            root=i.R
            if root not in Coord.center:
                return False
            center=Coord.center[root]
            theta=Coord.theta[root]
            self.M.PTopo.append([list(center),float(theta)])
        for e in self.M.PBreak:
            self.M.addE(e,0)
        for e in self.M.TBreak:
            self.M.addE(e,1)
        return True

    def Print(self,msc=1,rdcb=1,fast=1,rdmp=0):
        Mprint(self,msc=msc,rdcb=rdcb,fast=fast,rdmp=rdmp)