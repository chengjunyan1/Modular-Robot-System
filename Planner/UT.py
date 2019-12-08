# Utils & Tools

from itertools import permutations,combinations,product
import math,random

""" UTILS """

# COMMON UTILS 

def Eeq(e1,e2): # e=[v1,v2]
    if (e1[0]==e2[0] and e1[1]==e2[1]) or (e1[0]==e2[1] and e1[1]==e2[0]):
        return True
    return False

def inEset(e,E): # e=[v1,v2] E=[e1,e2,e3...]
    for i in E:
        if Eeq(e,i):
            return True
    return False

def subsetOf(a,b):
    for i in a:
        if i not in b:
            return False
    return True

def eqSet(a,b):
    if subsetOf(a,b) and subsetOf(b,a):
        return True
    return False

def inSet(s,S):
    for i in S:
        if eqSet(s,i):
            return True
    return False

def overlap(a,b):
    for i in a:
        if i in b:
            return True
    return False

def listDel(a,ds):
    na=[]
    for i in a:
        if i not in ds:
            na.append(i)
    return na

def listAdd(a,b):
    n=[]
    for i in range(len(a)):
        n.append(a[i]+b[i])
    return n

def listSum(a):
    s=0
    for i in a:
        s=s+i
    return s

def listSub(a,b):
    n=[]
    for i in range(len(a)):
        n.append(a[i]-b[i])
    return n

def reduceSet(S):
    if S==[]:
        return S
    R=[]
    for i in S:
        if not inSet(i,R):
            R.append(i)
    return R

def mapAdd(m1,m2):
    for i in m2:
        m1[i]=m2[i]
    return m1

def beMapped(a,M):
    for i in M:
        if a==M[i]:
            return True
    return False

def revMap(a,Map):
    for i in Map:
        if Map[i]==a:
            return i

def randSamp(a,WO=[]):
    s=[]
    for i in a:
        if i not in WO:
            s.append(i)
    return random.choice(s)

class HVertice:
    def __init__(self,ID):
        self.id=ID
        self.C=[] # connecting V
        self.h=-1
    
    def connect(self,v):
        if v not in self.C:
            self.C.append(v)
        
    def visit(self,visitor):
        if visitor==0: # means root
            self.h=0
            for i in self.C:
                i.visit(self)
        else:
            self.h=visitor.h+1
            for i in self.C:
                if i.id!=visitor.id:
                    i.visit(self)

# MSRR UTILS
    
def relativeRot(c,rot=0): # from c1 to c2
    if c==[0,0] or c==[1,1] or c==[2,2] or c==[3,3]:
        return 2+2*rot # 180
    elif c==[0,3] or c==[3,0] or c==[1,2] or c==[2,1]:
        return 0+2*rot # 0
    elif c==[0,2] or c==[1,0] or c==[2,3] or c==[3,1]:
        return 1+2*rot # 90
    elif c==[0,1] or c==[1,3] or c==[2,0] or c==[3,2]:
        return 3+2*rot # 270
    
def relativeCoord(Rot,c):
    if Rot%4==0:
        C=[[0,-1],[1,0],[-1,0],[0,1]]
    elif Rot%4==2:
        C=[[0,1],[-1,0],[1,0],[0,-1]]
    elif Rot%4==1:
        C=[[1,0],[0,1],[0,-1],[-1,0]]
    elif Rot%4==3:
        C=[[-1,0],[0,-1],[0,1],[1,0]]
    return C[c]

def calCoord(v0,Rot,G):
    v0id=G.Vindex(v0)
    for i in G.O[v0id]: # out-edge of v0
        e=G.reqE(v0,i)
        if e[0]==v0:
            c0=e[2]
            c1=e[3]
        elif e[1]==v0:
            c0=e[3]
            c1=e[2]
        Rot[G.Vindex(i)]=Rot[v0id]+relativeRot([c0,c1],e[5])
        G.C[G.Vindex(i)]=listAdd(G.C[v0id],relativeCoord(Rot[v0id],c0))
        G.C=calCoord(i,Rot,G)        
    return G.C

def GID(GS,v):
    for i in range(len(GS)):
        if v in GS[i].Vid:
            return i

""" TOOLS """

# CSC TOOLS

def VMapped(Mi,mode):
    VM=[]
    for i in Mi:
        v1=i[0] if mode==0 else i[2]
        v2=i[1] if mode==0 else i[3]
        if v1 not in VM:
            VM.append(v1)
        if v2 not in VM:
            VM.append(v2)
    return VM

def genWO(v,Mi,mode): # vertices connected that been mapped for v
    WO=[]
    for i in Mi:
        vp1=i[0] if mode==0 else i[2]
        vp2=i[1] if mode==0 else i[3]
        if vp1==v:
            WO.append(vp2)
        if vp2==v:
            WO.append(vp1)
    return WO

def condF(e1,e2): #****** whether e1 and e2 have same topology ******#
    if e1[2:5]==e2[2:5]:
        if e1[2]==e1[3]: # symmetry
            return 3
        return 1
    elif e1[2]==e2[3] and e1[3]==e2[2] and e1[4]==e2[4]:
        return 2
    return False

def newC(e1,e2):
    vp1=e1[0]
    vp2=e1[1]
    vt1=e2[0]
    vt2=e2[1]
    if condF(e1,e2)==3:
        return [vp1,vp2,vt1,vt2],[vp1,vp2,vt2,vt1]
    elif condF(e1,e2)==1:
        return [vp1,vp2,vt1,vt2]
    elif condF(e1,e2)==2:
        return [vp1,vp2,vt2,vt1]
    else:
        return []

def getMapping(M,s=0):
    Mapping={}
    for i in M:
        if i[0]+s not in Mapping:
            Mapping[i[0]+s]=i[2]+s
        if i[1]+s not in Mapping:
            Mapping[i[1]+s]=i[3]+s
    return Mapping

# MFS TOOLS

def VCSC(M,s=0): # Vertices in a CSC
    V=[]
    Mapping=getMapping(M,s)
    for i in Mapping:
        if i not in V:
            V.append(i)
    return V

def allVCSC(M,s=0): # Vertices in all CSC
    V=[]
    for i in M:
        V.append(VCSC(i,s))
    return V

def MCSC(M,s=0): # Mapped Vertices in a CSC
    V=[]
    Mapping=getMapping(M,s)
    for i in Mapping:
        m=Mapping[i]
        if m not in V:
            V.append(m)
    return V

def allMCSC(M,s=0): # Mapped Vertices in all CSC
    V=[]
    for i in M:
        V.append(MCSC(i,s))
    return V

# SRP TOOLS
    
def combineSCC(R,f): # return a SCC of f
    SCC=[]
    for i in f:
        SCC=SCC+R[i]
    return SCC
    
def buildCluster(M,f,mode,split=1):
    R=M.R
    if mode==0:
        G=M.GP
        CSC=VCSC
        N=listSum(M.NP)
    else:
        G=M.GT
        CSC=MCSC
        N=listSum(M.NT)
    cluster=[]
    for i in range(len(G)):
        cluster.append([])
    for i in f:
        V=CSC(R[i])
        Gid=GID(G,V[0])
        cluster[Gid].append(V)
    for i in range(N):
        add=1
        for j in cluster:
            for k in j:
                if i in k:
                    add=0
                    break
        if add:
            Gid=GID(G,i)
            cluster[Gid].append([i])
    if split==0:
        rv=[]
        for i in cluster:
            rv=rv+i
    else:
        rv=cluster
    return rv

def clusterID(cluster,v):
    cNum=len(cluster)
    for i in range(cNum):
        if v in cluster[i]:
            return i

def inCSC(cluster,v):
    for i in cluster:
        if len(i)>1:
            if v in i:
                return True
    return False
    
def makeE(S,mode): # mode: 0 GP 1 GT
    E=[]
    for i in S:
        e=i[:-2] if mode==0 else i[-2:]
        E.append(e)
    return E

def mCoord(Coord): # return max and min x and y
    xmax=0
    xmin=0
    ymax=0
    ymin=0
    for i in Coord:
        x=i[0]
        y=i[1]
        if x>xmax:
            xmax=x
        if x<xmin:
            xmin=x
        if y>ymax:
            ymax=y
        if y<ymin:
            ymin=y
    return xmax,xmin,ymax,ymin

def updateCoord(PCoord,TCoord,Coord):
    NCoord=[TCoord]
    for i in Coord:
        if i!=PCoord:
            NCoord.append(i)
    return NCoord
    
def rectDist(coord1,coord2,Coord): # square Distance
    if coord1[0]<coord2[0]:
        x=coord1[0]
        X=coord2[0]
    else:
        x=coord2[0]
        X=coord1[0]
    if coord1[1]<coord2[1]:
        y=coord1[1]
        Y=coord2[1]
    else:
        y=coord2[1]
        Y=coord1[1]
    xmax,xmin,ymax,ymin=mCoord(Coord)
    sq1=2*(X-xmin+ymax-y)-(X-x+Y-y)
    sq2=2*(xmax-x+ymax-y)-(X-x+Y-y)
    sq3=2*(X-xmin+Y-ymin)-(X-x+Y-y)
    sq4=2*(xmax-x+Y-ymin)-(X-x+Y-y)
    return min(sq1,sq2,sq3,sq4)

def eulerDist(coord1,coord2):
    dx=coord1[0]-coord2[0]
    dy=coord1[1]-coord2[1]
    return math.sqrt(dx*dx+dy*dy)
    
def moveCost(M,D): #****** moving cost for moving a cluster ******#
    return len(M)*D

def pMapping(a,b): # potential mappings from a to b
    if len(a)>len(b):
        p=b #permutations
        c=a #combinations
    else:
        p=a
        c=b
    pm=permutations(p)
    cm=combinations(c,len(p))
    pM=[]
    for i in pm:
        for j in cm:
            pMij={}
            for k in range(len(p)):
                if len(a)>len(b):
                    pMij[j[k]]=i[k]
                else:
                    pMij[i[k]]=j[k]
            pM.append(pMij)
    return pM

def rMapping(a,b): # random mapping from a to b
    if len(a)>len(b):
        p=b #permutations(random)
        c=a #combinations(random)
    else:
        p=a
        c=b
    i=p
    j=random.sample(c,len(p))
    pM=[]
    pMij={}
    for k in range(len(p)):
        if len(a)>len(b):
            pMij[j[k]]=i[k]
        else:
            pMij[i[k]]=j[k]
    pM.append(pMij)
    return pM

#/****** Add-on Supports ******/#

# Multi Graph Support

def rotz(c,deg):
    x=c[0]
    y=c[1]
    rad=deg/180*math.pi
    nx=x*math.cos(rad)-y*math.sin(rad)
    ny=y*math.cos(rad)+x*math.sin(rad)
    return [nx,ny]

def topoCoord(C,topo): # topo: loc&rot
    nC=[]
    for i in C:
        ni=rotz(i,topo[1])
        nC.append([ni[0]+topo[0][0],ni[1]+topo[0][1]])
    return nC

def rectTopo(G):
    minx=0
    miny=0
    maxx=0
    maxy=0
    for i in G.C:
        ix=i[0]
        iy=i[1]
        if ix>maxx:
            maxx=ix
        if ix<minx:
            minx=ix
        if iy>maxy:
            maxy=iy
        if iy<miny:
            miny=iy
    return minx,miny,maxx,maxy

def defaultTopo(GS):
    if GS==[]:
        return []
    topo=[[[0,0],0]]
    minx,miny,maxx,maxy=rectTopo(GS[0])
    xo=maxx
    for i in range(1,len(GS)):
        minx,miny,maxx,maxy=rectTopo(GS[i])
        loc=[xo-minx+1,0]
        xo=maxx
        rot=0
        topo.append([loc,rot])
    return topo

# Cycle Detection Support

def cycleCheck(e):
    V={}
    for i in e:
        if i[0] not in V:
            V[i[0]]=1
        else:
            V[i[0]]+=1
        if i[1] not in V:
            V[i[1]]=1
        else:
            V[i[1]]+=1
    for i in V:
        if V[i]!=2:
            return False
    return True
        
def bruteCycle(E): # just brute
    Enum=len(E)
    Elist=list(range(Enum))
    Erecord=[]
    Cycle=[]
    for i in range(3,Enum+1):
        EC=list(combinations(Elist,i))
        for ec in EC:
            stop=0
            for j in Erecord:
                if subsetOf(j,ec):
                    stop=1
                    break
            if stop:
                break
            e=[]
            for j in ec:
                e.append(E[j])
            if cycleCheck(e):
                Cycle.append(e)
                Erecord.append(ec)
    return Cycle

def cycleBreak(Cycle):
    if Cycle==[]:
        return [[]]
    if len(Cycle)==1:
        CBC=[]
        for i in list(range(len(Cycle[0]))):
            CBC.append([i])
    else:
        Clen=[]
        pparams=''
        count=0
        for i in Cycle:
            Clen.append(list(range(len(i))))
            pparams+='Clen['+str(count)+'],'
            count+=1
        CBC=eval('list(product('+pparams[:-1]+'))')
    CB=[]
    for i in CBC:
        cb=[]
        count=0
        add=1
        for j in i:
            e=Cycle[count][j]
            if e in cb:
                add=0
                break
            cb.append(e)
            count+=1
        if add:
            CB.append(cb)
    return CB

def Vcycle(cycle):
    V=[]
    for c in cycle:
        Vc=[]
        for i in c:
            if i[0] not in Vc:
                Vc.append(i[0])
            if i[1] not in Vc:
                Vc.append(i[1])
        V.append(Vc)
    return V