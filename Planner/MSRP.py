# Self-Reconfiguration Planner (SRP)

from Planner.UT import *
import copy
from itertools import combinations,product 

# Select a best SCC from all SCCs

""" Graphs """

class subGraph:
    def __init__(self,E,G): # E: E set G: referenced G
        self.E=[]
        self.V=[]
        self.CN=[]
        self.Center=-1
        for i in E:
            Gid=GID(G,i[0]) # edge must be in one G
            self.E.append(G[Gid].reqE(i[0],i[1]))
            if i[0] not in self.V:
                self.V.append(i[0])
                self.CN.append([0,0,0,0])
            if i[1] not in self.V:
                self.V.append(i[1])
                self.CN.append([0,0,0,0])
    
    def nearE(self,v):
        rv=[]
        for i in self.E:
            if i[0]==v or i[1]==v:
                rv.append(i)
        return rv
    
    def Tvisit(self,v,visitor,T):
        T+=1
        nE=self.nearE(v)
        for i in nE:
            v1=i[0]
            v2=i[1]
            if v1==v:
                if v2!=visitor:
                    T=self.Tvisit(v2,v1,T)
            elif v2==v:
                if v1!=visitor:
                    T=self.Tvisit(v1,v2,T)
        return T
    
    def findCenter(self):
        Vnum=len(self.V)
        for i in range(Vnum):
            v=self.V[i]
            nE=self.nearE(v)
            for j in nE:
                if j[0]==v:
                    self.CN[i][j[2]]=self.Tvisit(j[1],j[0],0)
                elif j[1]==v:
                    self.CN[i][j[3]]=self.Tvisit(j[0],j[1],0)
        for i in range(Vnum):
            found=1
            for j in self.CN[i]:
                if j>Vnum/2:
                    found=0
                    break
            if found:
                self.Center=self.V[i]
                break

class multiGraph:
    def __init__(self,GS,topo):
        self.C=[]
        self.E=[]
        for i in range(len(GS)):
            GSiC=topoCoord(GS[i].C,topo[i])
            self.C=self.C+GSiC
            self.E=self.E+GS[i].E

class clusterGraph:
    def __init__(self,cluster,GS,center,topo=0):
        if topo!=0: # means multi-G
            # reTopo
            Cid=0
            for i in GS:
                if center in i.Vid:
                    break
                Cid+=1
            cloc=topo[Cid][0]
            crot=topo[Cid][1]
            ntopo=[]
            for i in range(len(topo)):
                nloc=listSub(topo[i][0],cloc)
                nrot=topo[i][1]-crot
                ntopo.append([nloc,nrot])
            # build G
            Gcenter=[]
            for i in range(len(GS)):
                if i!=Cid:
                    GS[i].Troot()
                    Gcenter.append(GS[i].R)
                else:
                    GS[i].Trebuild(center)
                    Gcenter.append(center)
            G=multiGraph(GS,ntopo)
            NStart=0
        else:
            G=GS
            NStart=G.NStart
        # build CG
        self.V=cluster
        self.center=center
        GCoord=G.C
        self.Coord=[]
        self.E=[] 
        self.CE=[] # Connecting edge
        for i in self.V:
            num=len(i)
            coord=[0,0]
            for j in i:
                coord=listAdd(coord,GCoord[j-NStart])
            coord[0]=coord[0]/num
            coord[1]=coord[1]/num
            self.Coord.append(coord)
        for i in G.E:
            v1=i[0]
            v2=i[1]
            cid1=clusterID(self.V,v1)
            cid2=clusterID(self.V,v2)
            if cid1!=cid2:
                if [cid1,cid2] not in self.E:
                    self.E.append([cid1,cid2])
                if i not in self.CE:
                    self.CE.append(i)

    def hierarchy(self):
        VV=[]
        for i in range(len(self.V)):
            VV.append(HVertice(i))
        for i in self.E:
            VV[i[0]].connect(VV[i[1]])
            VV[i[1]].connect(VV[i[0]])
        CCid=clusterID(self.V,self.center)
        VV[CCid].visit(0)
        H=[]
        for i in VV:
            H.append(i.h)
        return H
      
""" Planners """
  
def mapCost(CGP,CGT,Map): # return cost
    Pcenter=CGP.center
    PCid=clusterID(CGP.V,Pcenter)
    cost=0
    visited=[]
    Coord=CGP.Coord+CGT.Coord # CGP.Coord
    for i in Map:
        pcid=clusterID(CGP.V,i)
        tcid=clusterID(CGT.V,Map[i])
        if pcid!=PCid and pcid not in visited:
            visited.append(pcid)
            PCoord=CGP.Coord[pcid]
            TCoord=CGT.Coord[tcid]
            M=CGP.V[pcid]
            D=rectDist(PCoord,TCoord,Coord) #eulerDist(PCoord,TCoord)
            #Coord=updateCoord(PCoord,TCoord,Coord)
            cost=cost+moveCost(M,D)
    return cost
    
def evalF(M,f,fast=1,rdmp=0,msc=0): # get potenciel centers
    R=M.R
    clusterP=buildCluster(M,f,0,0) # not split
    clusterT=buildCluster(M,f,1)
    collectT=[]
    for i in clusterT:
        collectT=collectT+i
    centerP={}
    centerT=[]
    cscP={}
    cscT=[]
    count=0
    for cp in clusterP:
        if len(cp)==1:
            centerP[count]=cp[0]
        else:
            for i in f:
                if eqSet(VCSC(R[i]),cp):
                    SCid=i
            SG=subGraph(makeE(R[SCid],0),M.GP)
            SG.findCenter()
            centerP[count]=SG.Center # centerP is the center of this SC
            cscP[count]=SG.Center
        count+=1
    CTid=0
    for cti in clusterT:
        cscT.append([])
        centerT.append([])
        count=0
        for ct in cti:
            if len(ct)==1:
                centerT[CTid].append(ct[0])
            else:
                for i in f:
                    if eqSet(MCSC(R[i]),ct):
                        SCid=i
                SG=subGraph(makeE(R[SCid],1),M.GT)
                SG.findCenter()
                centerT[CTid].append(SG.Center)
                cscT[CTid].append(count)
            count+=1
        CTid+=1
    # cluster Mapping
    clusterMapping={}
    Mapping=getMapping(combineSCC(R,f))
    for i in Mapping:
        for cpid in range(len(clusterP)):
            if i in clusterP[cpid]:
                Mapper=cpid
                break
        for CTid in range(len(clusterT)):
            for ctid in range(len(clusterT[CTid])):
                if Mapping[i] in clusterT[CTid][ctid]:
                    MappedCenter=centerT[CTid][ctid]
                    break
        clusterMapping[centerP[Mapper]]=MappedCenter
    # enumerate the center
    Center=[]
    if len(clusterP)<len(clusterT):
        virtualM=0
        for i in range(len(clusterP),len(clusterT)):
            virtualM+=1
            clusterP.append([-virtualM])
            centerP[len(clusterP)-1]=-virtualM
    if fast: # fast mode: search only csc and roots
        cps_set=[]
        for i in cscP:
            cps_set.append(i)
        if not (msc and cps_set!=[]):
            for i in M.GProots():
                if not inCSC(clusterP,i):
                    cps_set.append(clusterID(clusterP,i))
        if len(cps_set)<len(clusterT):
            for i in range(len(clusterT)-len(cps_set)):
                cps_set.append(randSamp(list(range(len(clusterP))),cps_set))
        cps=list(permutations(cps_set,len(clusterT)))
        pparam=''
        cts_set=[]
        GTr=M.GTroots()
        for i in range(len(clusterT)):
            cscTi=cscT[i]
            if not inCSC(clusterT[i],GTr[i]):
                cscTi.append(clusterID(clusterT[i],GTr[i]))
            cts_set.append(cscTi)
            pparam=pparam+'cts_set['+str(i)+'],'
        pparam=pparam[:-1]
        cts=eval('list(product('+pparam+'))')
    else: # brute mode: brute-force search
        cps=list(permutations(list(range(len(clusterP))),len(clusterT)))
        pparam=''
        for i in range(len(clusterT)):
            pparam=pparam+'list(range(len(clusterT['+str(i)+']))),'
        pparam=pparam[:-1]
        cts=eval('list(product('+pparam+'))')
    for i in cps:
        CenterP=[]
        for k in i:
            CenterP.append(centerP[k])
        for j in cts:
            CenterT=[]
            for CTid in range(len(j)):
                CenterT.append(centerT[CTid][j[CTid]])
            add=1
            for i in range(len(CenterP)):
                cpi=CenterP[i]
                cti=CenterT[i]
                if cpi in clusterMapping:
                    if cti!=clusterMapping[cpi]:
                        add=0
                        break
            if add:
                Center.append([CenterP,CenterT])
                
    # evaluate each center combination
    minMap=0
    mincost=9999999
    mincenter=0
    minCGP=0
    minCGT=0
    minroot=0
    for center in Center:
        stop=0
        nM=copy.deepcopy(Mapping)
        for i in range(len(center[0])):
            if center[0][i] not in nM:
                if beMapped(center[1][i],nM):
                    stop=1
                    break
                nM[center[0][i]]=center[1][i]
        if stop:
            continue
        CGProot=0
        CGTroot=0
        Nmax=0
        for i in range(len(center[0])):
            for j in clusterP:
                if center[0][i] in j:
                    Nroot=len(j)
                    break
            if Nroot>Nmax:
                Nmax=Nroot
                CGProot=center[0][i]
                CGTroot=center[1][i]
        CGP=clusterGraph(clusterP,M.GP,CGProot,M.PTopo)
        CGT=clusterGraph(collectT,M.GT,CGTroot,M.TTopo)
        PV=[] # V not in mapping in GP
        TV=[] # V not mapped in GT
        for i in range(listSum(M.NP)):
            if i not in nM:
                PV.append(i)
        for i in range(listSum(M.NT)):
            if not beMapped(i,nM):
                TV.append(i)
        pM=rMapping(PV,TV) if rdmp else pMapping(PV,TV) # potential mappings
        for i in pM:
            Map=mapAdd(copy.deepcopy(nM),i)
            cost=mapCost(CGP,CGT,Map)
            if cost<mincost:
                mincost=cost
                minMap=Map
                mincenter=center[0]
                minroot=CGProot
                minCGP=CGP
                CGTcenter=[]
                for i in M.GT:
                    if CGTroot in i.Vid:
                        CGTcenter.append(CGTroot)
                    else:
                        i.Troot()
                        CGTcenter.append(i.R)
                minCGT=[]
                for i in range(len(clusterT)):
                    minCGT.append(clusterGraph(clusterT[i],M.GT[i],CGTcenter[i]))
    return minMap,mincost,mincenter,minroot,minCGP,minCGT

def dockSeq(CGT,Map):
    virtualM=0
    for i in Map:
        if i<0:
            virtualM+=1
    H=[]
    for i in CGT:
        H+=i.hierarchy()
    DS=[]
    for i in range(max(H)+1):
        DS.append([])
    for cgt in CGT:
        for i in cgt.CE:
            e=copy.deepcopy(i)
            v1=e[0]
            v2=e[1]
            cid1=clusterID(cgt.V,v1)
            cid2=clusterID(cgt.V,v2)
            h1=H[cid1]
            h2=H[cid2]
            e[0]=revMap(v1,Map)
            if e[0]==None:
                virtualM+=1
                Map[-virtualM]=v1
                e[0]=-virtualM
            e[1]=revMap(v2,Map)
            if e[1]==None:
                virtualM+=1
                Map[-virtualM]=v2
                e[1]=-virtualM
            DS[min(h1,h2)].append(e)
    rDS=[]
    for i in DS:
        rDS.append(i) if i!=[] else None
    return rDS

def SRP(M,fast=1,rdmp=0,detail=0,msc=0): #***ASSUME GP is aligned from left to right by their ID
    minMap=0
    minCost=9999999
    mincenter=0 # point not move (single v or center of sc)
    minCGP=0
    minCGT=0
    minF=-1 # sub configs
    for i in M.F:
        Map,cost,center,root,CGP,CGT=evalF(M,i,fast,rdmp,msc)
        if cost<minCost:
            minCost=cost
            minMap=Map
            mincenter=center
            minroot=root
            minCGP=CGP
            minCGT=CGT
            minF=i
    minSC=[]
    for i in minF:
        minSC.append(M.R[i])
    M.buildTree() # recover root of M
    # results processing
    MP=minMap # mapping
    SC=allVCSC(minSC) # sub-configurations
    CT=[]
    for i in mincenter:
        add=1
        for j in SC:
            if i in j:
                add=0
                CT.append(j)
        if add:
            CT.append([i])
    for i in CT:
        if minroot in i:
            RT=i
            break
    UD=[]
    for i in minCGP.CE:
        UD.append(i[:2])
    for i in M.PBreak:
        UD.append(i[:2])
    DS=dockSeq(minCGT,MP)
    if M.TBreak!=[]:
        DSi=[]
        for i in M.TBreak:
            e=[revMap(i[0],MP),revMap(i[1],MP)]+i[2::]
            DSi.append(e)
        DS.append(DSi)
    if detail:
        return SC,RT,CT,UD,DS,MP,minCost,minCGP,minCGT
    return SC,RT,CT,UD,DS