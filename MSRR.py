# MSRR System

from Planner.Planner import *
from Coordinator.Coordinator import *
from Actuator.Actuator import *
from Server.Interface import *
import time,socket,configs
import  _thread

class MSRR:
    def __init__(self,option=0):
        self.MP=Planner()
        self.RC=Coordinator()
        self.AS=Actuator()
        self.SI=ServerInterface()
        self.SetRoot() # must have a module 0 as root  to init
        self.resolution=pf_map_resolution 
        self.semaphore=0 # for multi threads
        self.SILock=0
        self.initialize(option)
    
    def initialize(self,option):
        if option:
            self.write_log(0)
        if (connect_sim_drone or connect_real_drone) and not drone_stand_alone:
            self.SetPose()

    def reset(self):
        self.MP=Planner()
        self.RC=Coordinator()
        self.AS=Actuator()
        self.SetRoot()
    
    def feed(self,C):
        C(self.MP.M,0)

    def reconfig(self,vC,Coord=1,Target=0,msc=1,rdcb=1,fast=1,rdmp=0):
        print('\Start a New Reconfiguration')
        time.sleep(1)
        for i in range(planner_buffer_time): # Buffering time
            print(planner_buffer_time-i)
            time.sleep(1)
        print('Reconfiguration Start')
        time.sleep(1)
        print('Planning for Reconfiguration...')
        P=self.ReconfigPlan(vC,Coord=Coord)
        print('Reconfiguration Planning finished')
        log=self.read_log()
        for i in P[1]:
            if i>=log:
                print('Planning for Path of stage ',i,' ...')
                rp,vt=self.DockPath(P[1][i])
                print('Planning for Path of stage ',i,' finished')
                self.PathSignal(rp,vt)
                while self.semaphore!=0:
                    pass
                self.write_log(i+1)
            self.write_log(0)
    
    #------------ COORDINATOR ------------#

    def SetPose(self,pose='m18'): # Use it when not using DroneScript
        self.RC.SetPose(pose)

    def SetRoot(self,root=0): # Must Set Root before Coord, could set an arbitrary root (default 0) at first
        self.RC.setRoot(root)

    def Coord(self,img=None):
        if img is not None:
            img = cv2.imread(img,cv2.IMREAD_GRAYSCALE)
        return self.RC.Coord(img=img)

    def UnitCoord(self,unit=1,img=None):
        if img is not None:
            img = cv2.imread(img,cv2.IMREAD_GRAYSCALE)
        return self.RC.unitCoord(unit=unit,img=img)

    def CoordinatorPrint(self):
        self.RC.Print()

    #------------ PLANNER ------------#
    # Two Ways of using it
    # 1. SetConfig-->SetTopo-->Plan
    # 2. Directly use ReconfigPlan

    def ReconfigPlan(self,vC,Coord=1,Target=0,msc=1,rdcb=1,fast=1,rdmp=0): # vC: new config vC(M,mode) mode:0 GP 1 GT
        ActionPlan=[[],{}] # [[Undockings],[Dockings]]
        Root=[]
        if self.MP.M.GP==[]:
            vC(self.MP.M,0)
            self.SetTopo(Coord=Coord,Target=Target)
            Plan=self.PlanSA(rdcb=rdcb)
            for i in Plan:
                Root.append(i[0])
                for j in range(1,len(i)):
                    ActionPlan[1][j-1]=[]
            for i in Plan:
                for j in range(1,len(i)):
                    ActionPlan[1][j-1]+=i[j]
        else:
            vC(self.MP.M,1)
            self.SetTopo(Coord=Coord,Target=Target)
            Plan=self.PlanSR(msc=msc,rdcb=rdcb,detail=0,fast=fast,rdmp=rdmp,update=1)
            ActionPlan[0]=Plan[3]
            Root=Plan[1]
            for i in range(len(Plan[4])):
                ActionPlan[1][i]=Plan[4][i]
        ActionPlan[1]=self.ActionPlan(ActionPlan[1],Root)
        print('Undocking:')
        print(ActionPlan[0])
        print('Docking:')
        for i in ActionPlan[1]:
            print('stage',i,ActionPlan[1][i])
        return ActionPlan
    
    def ActionPlan(self,Plan,Root):
        for i in Plan:
            plan=Plan[i]
            vplan=[]
            for j in plan:
                if j[0] in Root:
                    vplan.append([j[1],j[0],j[3],j[2],j[4],j[5]])
                    Root.append(j[1])
                else:
                    vplan.append(j)
                    Root.append(j[0])
            Plan[i]=vplan
        return Plan

    def SetConfig(self,C): # Set Config first
        self.MP.Config(C)
    
    def SetTopo(self,Coord=0,Target=0,rdcb=1): # Then Set Topo of Graphs
        if not (Coord and Target):
            self.MP.initTopo(rdcb)
        if Coord:
            unitCoord=self.RC.unitCoord()
            self.MP.coordTopo(unitCoord,rdcb)
        if Target:
            self.MP.M.TTopo=Target # set a target topo instead of using default topo, [[[dx,dy],rot],...], e.g [[[0, 0], 0], [[3, 0], 0]]

    def PlanSA(self,rdcb=0):
        if self.MP.SAssembly(rdcb=rdcb)!=-1:
            return self.MP.SA # [root, dock]
        
    def PlanSR(self,msc=1,rdcb=1,detail=0,fast=1,rdmp=0,update=1):
        if self.MP.RPlanner(fast=fast,rdmp=rdmp,rdcb=rdcb,detail=detail,msc=msc,update=update)!=-1:
            return self.MP.SR # [SC,RT,CT,UD,DS] DS: Dock Seq

    def PlannerPrint(self,msc=1,rdcb=1,fast=1,rdmp=0):
        self.MP.Print(msc=msc,rdcb=rdcb,fast=fast,rdmp=rdmp)

    def ConfigDicover(self,V): # Given infomation of V, construct GPs
        pass
                
    #------------ ACTUATOR ------------#

    def DockPath(self,docks,finder='JPS',safe_rate=1.0,plot=1,nin=1,opt=1,batch_size=3):
        uc=self.UnitCoord(self.resolution)
        self.AS.initPF(uc,safe_rate,finder,pf_map_size)
        rp,vt=self.AS.Dock(docks,plot,nin,pf_tolerent_rate,opt)
        return rp,vt

    def PathSignal(self,rp,vt):
        dt=signal_dt
        for i in rp:
            if multi_threads:
                _thread.start_new_thread(self.SendPath,(i,rp,vt,dt))
            else:
                self.SendPath(i,rp,vt,dt)
            self.semaphore-=1
    
    def SendPath(self,Mid,rp,vt,dt):
        for i in range(len(rp[Mid])):
            while self.SILock:
                pass
            self.SILock=1
            ts=time.time()
            uc=self.UnitCoord(self.resolution)
            dtr=signal_dt+time.time()-ts
            dxd= 0 if i==len(rp[Mid])-1 else (rp[Mid][i+1][0]-rp[Mid][i][0])/dtr
            dyd=0 if i==len(rp[Mid])-1 else (rp[Mid][i+1][1]-rp[Mid][i][1] )/dtr
            signal=[round(uc.center[Mid][0],3),round(uc.center[Mid][1],3),round(uc.theta[Mid],3),
                            round(rp[Mid][i][0],3),round(rp[Mid][i][1],3),round(vt[Mid],3),
                            round(dxd,3),round(dyd,3),0,round(dtr,3)]
            print('Sending Signal to Module',Mid,'...','Process:',i+1,'/',len(rp[Mid]))
            self.SI.sendSignal(Mid,signal)
            self.SILock=0 
            time.sleep(dt)
        self.semaphore+=1

    #/******* utils *******/#

    def read_log(self):
        log_path=configs.root_path+'temp/log.txt'
        log=open(log_path,'r')
        stage=log.read()
        log.close()
        return int(stage)

    def write_log(self,stage):
        log_path=configs.root_path+'temp/log.txt'
        log=open(log_path,'w')
        log.write(str(stage))
        log.close()