# MSRR Graph-based Representation

from Planner.UT import *
import random

""" MSRR Representations """

# Face id: 0:Top 1:Left 2:Right 3:Bottom 
class V: # Vertices, Module representation
    def __init__(self,ID,VNum):
        self.id=ID
        self.VNum=VNum # total num of modules
        self.C=[0,0,0,0] # connect status: T L R B, 1:connect, 2:connect2(for B)
        self.D=[0,0,0,0] # docking modeule in each face
        self.rot=[0,0,0,0] # rot of each face
        # TREE
        self.i=[] # faces of in edges
        self.o=[] # faces of out edges
        self.h=-1 # Height of this v
        self.leaf=0 # whether leaf
        self.iNum=0 # in degree
        self.oNum=0 # out degree
        self.CN=[0,0,0,0] # connect number
        self.rCN=[0,0,0,0] # CN received table

    def dock(self,c,ori,rot,V):
        self.D[c]=V
        self.rot[c]=rot
        if c==3: # Bottom, have 2 oris
            self.C[c]=ori
        else:
            self.C[c]=1
    
    def undock(self,c): # c: face id
        self.C[c]=0      
        self.D[c]=0
        self.rot[c]=0
    
    def Treset(self): 
        self.h=-1 # Height of this v
        self.i=[] # faces of in edges
        self.o=[] # faces of out edge
        self.leaf=0 # whether leaf
        self.iNum=0 # in degree
        self.oNum=0 # out degree
        self.CN=[0,0,0,0] # connect number
        self.rCN=[0,0,0,0] # CN received table
    
    def Tvisit(self,visitor,UPCN=1): # create Tree
        if visitor!=0: # 0 means root
            self.h=visitor.h+1
            for i in range(4):
                d=self.D[i]
                if d!=visitor and d!=0:
                    if i not in self.o:
                        self.o.append(i) # add out edge  
                        self.oNum=self.oNum+1
                elif d==visitor:
                    if i not in self.i:
                        self.i.append(i) # add in edge 
                        self.iNum=self.iNum+1
        else:
            self.h=0
            for i in range(4):
                if self.D[i]!=0 and i not in self.o:
                    self.o.append(i)
        # leaf: Bottom-up updateCN, non-leaf: Top-down buildTree
        if self.o==[]:
            self.leaf=1
            if UPCN:
                self.updateCN(self,0)
        else:
            for i in self.o:
                self.D[i].Tvisit(self)
    
    def updateCN(self,v,CN):
        for i in range(4): # find visitor face
            if v==self.D[i]:
                break
        self.rCN[i]=1 # received CN from this face
        self.CN[i]=CN+1 if self.leaf!=1 else CN  # CN of v plus v itself 
        CNo=0
        for i in self.o:
            CNo=CNo+self.CN[i]
        for i in self.i: # max 1 in-degree in Tree, null in root
            self.CN[i]=self.VNum-1-CNo
            self.D[i].updateCN(self,CNo)
        # check whether self is root
        for i in self.o: # if all visited, pass
            if self.rCN[i]==0:
                return
        for i in self.CN: # if all CN <= N/2, pass
            if i>self.VNum/2:
                return
        global Root
        Root=self.id

class G: # Configuration Graph
    def __init__(self,NStart,N):
        self.V=[]
        self.E=[]
        self.Vnum=N
        self.Enum=0
        self.NStart=NStart
        self.Vid=list(range(NStart,NStart+N))
        for i in self.Vid:
            self.V.append(V(i,N))
        # Tree info
        self.R=-1 # Root module id
        self.C=-1 # Coordinate
        self.O=[] # Output vertices list
        self.I=[] # Input vertices list
        self.H=[] # Height of each v
        
    def Vindex(self,v):
        return self.Vid.index(v)
    
    def Treset(self):
        self.R=-1 # Root module id
        self.C=-1 # Coordinate
        self.O=[] # Output vertices list
        self.I=[] # Input vertices list
        self.H=[] # Height of each v
        for i in self.V:
            i.Treset()  
    
    def Tinfo(self):
        self.O=[] # Output vertices list
        self.I=[] # Input vertices list
        self.H=[] # Height of each v
        for i in self.V:
            Oid=[]
            for j in i.o:
                Oid.append(i.D[j].id)
            self.O.append(Oid)
            Iid=[]
            for j in i.i:
                Iid.append(i.D[j].id)
            self.I.append(Iid)
            self.H.append(i.h)
        
    def Trebuild(self,root):
        for i in range(self.Vnum):
            self.V[i].Treset()
        self.R=root
        self.V[self.Vindex(root)].Tvisit(0,0)
        self.Tinfo()
        self.reCoord(root)
        
    def reCoord(self,center):
        cindex=self.Vindex(center)
        Rot=[]
        self.C=[]
        for i in range(self.Vnum):
            Rot.append([])
            self.C.append([])
        Rot[cindex]=0
        self.C[cindex]=[0,0]
        self.C=calCoord(center,Rot,self)
        
    def addEdge(self,v1,v2,c1,c2,ori=1,rot=0): # orientation only for connection between bottoms
        if self.reqE(v1,v2)!=-1:
            return -1
        E=[v1,v2,c1,c2,ori,rot] # rot for others, 0 for 0 1 for 180
        self.E.append(E)
        self.Enum+=1
        v1i=self.Vid.index(v1)
        v2i=self.Vid.index(v2)
        self.V[v1i].dock(c1,ori,rot,self.V[v2i])
        self.V[v2i].dock(c2,ori,rot,self.V[v1i])
    
    def delEdge(self,v1,v2): # connect e.g. 31 between node 1 and 3
        found=0
        for i in range(self.Enum):
            E=self.E[i]
            ev1=E[0]
            ev2=E[1]
            if ev1==v1 and ev2==v2:
                found=1
                break
        if not found:
            return -1
        del self.E[i]
        c1=E[2]
        c2=E[3]
        v1-=self.NStart
        v2-=self.NStart
        self.V[v1].undock(c1)
        self.V[v2].undock(c2)
        self.Enum-=1
    
    def Troot(self):
        for i in self.V:
            i.Treset()
        v0=random.choice(range(self.Vnum))
        self.V[v0].Tvisit(0)
        for i in self.V:
            i.Treset()
        global Root
        self.R=Root
        self.V[self.Vindex(self.R)].Tvisit(0,1)
        self.Tinfo()
        self.reCoord(self.R)
    
    def reqE(self,v1,v2):
        for i in self.E:
            if Eeq(i[:2],[v1,v2]):
                return i
        return -1
    
    def reqR(self):
        if self.R==-1:
            self.Troot()
        return self.R
    
    def dockingID(self,v,WO=[]):
        ID=[]
        D=self.V[self.Vindex(v)].D
        for i in D:
            if i!=0:
                if i.id not in WO:
                    ID.append(i.id)
        return ID

class M: # MSRR representation with Multiple Graph support
    def __init__(self):
        self.NP=[] # num of modules
        self.NT=[] # target module num
        self.GP=[] # G Present
        self.GT=[] # G Target
        
    def checkTree(self):
        for i in self.GP:
            if [] in i.C:
                return False
        for i in self.GT:
            if [] in i.C:
                return False
        return True
            
    def setGP(self,NP):
        NPStart=listSum(self.NP)
        self.NP.append(NP)
        self.GP.append(G(NPStart,NP))
        return len(self.NP)-1 # id of this GP
        
    def setGT(self,NT):
        NTStart=listSum(self.NT)
        self.NT.append(NT)
        self.GT.append(G(NTStart,NT))
        self.RP=0 # Reconfiguration Plan
        return len(self.NT)-1 # id of this GT
        
    def addGP(self,GPid,v1,v2,c1,c2,ori=1,rot=0): # input id start from 1 (for convinient)
        GPStart=listSum(self.NP[:GPid])
        self.GP[GPid].addEdge(GPStart+v1-1,GPStart+v2-1,c1,c2,ori,rot)
    
    def addGT(self,GTid,v1,v2,c1,c2,ori=1,rot=0): # input id start from 1 (for convinient)
        GTStart=listSum(self.NT[:GTid])
        self.GT[GTid].addEdge(GTStart+v1-1,GTStart+v2-1,c1,c2,ori,rot)
    
    def GPid(self,v):
        for i in range(len(self.GP)):
            if v in self.GP[i].Vid:
                return i
    
    def GTid(self,v):
        for i in range(len(self.GT)):
            if v in self.GT[i].Vid:
                return i
            
    def GTroots(self):
        rv=[]
        for i in self.GT:
            rv.append(i.reqR())
        return rv
    
    def GProots(self):
        rv=[]
        for i in self.GP:
            rv.append(i.reqR())
        return rv
        
    def Treset(self):
        for i in self.GP:
            i.Treset()
        for i in self.GT:
            i.Treset()
    
    def addE(self,e,mode):
        v1=e[0]
        v2=e[1]
        if mode==0:
            G=self.GP
        else:
            G=self.GT
        for i in G:
            if v1 in i.Vid and v2 in i.Vid:
                i.addEdge(e[0],e[1],e[2],e[3],e[4],e[5])
                return
    
    def delE(self,e,mode):
        v1=e[0]
        v2=e[1]
        if mode==0:
            G=self.GP
        else:
            G=self.GT
        for i in G:
            if v1 in i.Vid and v2 in i.Vid:
                i.delEdge(v1,v2)
                return
        
    def buildTree(self):
        for i in self.GP:
            i.Troot()
        for i in self.GT:
            i.Troot()

    def updateGP(self):
        self.NP=[]
        self.GP=[]
        for gid in range(len(self.GT)):
            self.setGP(self.NT[gid])
            for i in self.GT[gid].E:
                self.GP[gid].addEdge(i[0],i[1],i[2],i[3],i[4],i[5])
        self.NT=[]
        self.GT=[]
        self.SA=[]
        self.RP=[]
        self.PTopo=self.TTopo