# Min-Fixed Sub-Configurations (MFSC)

from Planner.UT import *
import copy,random
from itertools import combinations,product 

# Calculate CSC set R and combination F

""" CSC: Get all potential Common Sub Configuarations """
    
def extandCSC(GP,GT,Ci):
    vC=[]
    VC=VMapped(Ci,0) # vertices mapped in GP
    for i1 in VC:
        Mapping=getMapping(Ci+vC)
        i2=Mapping[i1] # correspongding v for i in GT
        WO1=genWO(i1,Ci+vC,0)
        WO2=genWO(i2,Ci+vC,1)
        V1=GP.dockingID(i1,WO1) #vertices could use in GP from i
        V2=GT.dockingID(i2,WO2) #vertices could use in GP from i
        for j1 in V1: # i,j from an edge
            e1=GP.reqE(i1,j1)
            for j2 in V2:
                e2=GT.reqE(i2,j2)
                nC=newC(e1,e2)
                if nC!=[]:
                    if len(nC)==2:
                        if correctCSC(Ci+vC+[nC[0]]) and correctCSC(Ci+vC+[nC[1]]):
                            return vC+[nC[0]],vC+[nC[1]]
                        elif correctCSC(Ci+vC+[nC[0]]):
                            return vC+[nC[0]]
                        elif correctCSC(Ci+vC+[nC[1]]):
                            return vC+[nC[1]]
                    if correctCSC(Ci+vC+[nC]):
                        vC=vC+[nC]
    return vC

def searchCSC(GP,GT,C):
    Cn=[]
    CHANGE=0
    for Ci in C:
        vC=extandCSC(GP,GT,Ci)
        if vC!=[]:
            CHANGE=1
            if isinstance(vC,tuple):
                Cn.append(Ci+vC[0])
                Cn.append(Ci+vC[1])
            else:
                if correctCSC(Ci+vC):
                    Cn.append(Ci+vC)
    if CHANGE:
        return Cn,CHANGE
    else:
        return C,CHANGE

def correctCSC(m):
    mapping={}
    for i in m:
        vp1=i[0]
        vp2=i[1]
        vt1=i[2]
        vt2=i[3]
        if vp1 not in mapping:
            mapping[vp1]=vt1
        else:
            if mapping[vp1]!=vt1:
                return False
        if vp2 not in mapping:
            mapping[vp2]=vt2
        else:
            if mapping[vp2]!=vt2:
                return False
    return True

def reduceCSC(C):
    R=[]
    Vrecord=[]
    Mrecord=[]
    for i in C:
        V=VCSC(i)
        M=MCSC(i)
        Rnum=len(R)
        add=1
        for j in range(Rnum):
            Ri=R[j]
            Vi=VCSC(Ri)
            Mi=MCSC(Ri)
            if eqSet(V,Vi) and eqSet(M,Mi):
                add=0
                break
        if add:
            R.append(i)
            Vrecord.append(V)
            Mrecord.append(M)
    return R

def CSCg(GP,GT):
    C=[]
    for i in GP.E:
        for j in GT.E: 
            F=newC(i,j)
            if F!=[]:
                if isinstance(F,tuple):
                    C.append([F[0]])
                    C.append([F[1]])
                else:
                    C.append([F])
    if C==[]:
        return []
    CHANGE=1
    while CHANGE:
        C,CHANGE=searchCSC(GP,GT,C)
    R=reduceCSC(C) # finer result
    return R

def CSC(M): 
    R=[]
    for i in M.GP:
        for j in M.GT:
            R=R+CSCg(i,j)
    return R

""" MFS: Min-Fixed Set of CSC with maximum fixed vertices """

def newAvailable(R,scc):
    Rnum=len(R)
    V=allVCSC(R)
    M=allMCSC(R)
    available=[]
    for i in scc:
        Vscc=[]
        Mscc=[]
        for j in i:
            Vscc+=VCSC(R[j])
            Mscc+=MCSC(R[j])
        availablei=[]
        for j in range(Rnum):
            if (not overlap(Vscc,V[j])) and (not overlap(Mscc,M[j])):
                availablei.append(j)
        available.append(availablei)
    return available
        
def newSCC(scc,available,i): # adding available of i to now
    nscc=[]
    for n in available[i]:
        scci=copy.deepcopy(scc[i])
        nscc.append(scci+[n])
    return nscc

def searchSCC(R): # search all SC Combinations
    scc=[[]]
    CHANGE=1
    while CHANGE:
        CHANGE=0
        nscc=[]
        available=newAvailable(R,scc)
        for i in range(len(scc)):
            availablei=available[i]
            if availablei!=[]:
                CHANGE=1
                nscc+=newSCC(scc,available,i)
            else:
                nscc.append(scc[i])
        scc=nscc
    return scc
        
def MFS(R,rm=0): # Maximum-Fixed Set of CSC
    S=searchSCC(R) # !!!high computational cost 
    F=[]
    Max=0
    V=[]
    for i in S:
        Vi=[]
        for j in i:
            Vi+=VCSC(R[j])
        V.append(Vi)
        if len(Vi)>Max:
            Max=len(Vi)
    for i in range(len(S)):
        if len(V[i])==Max and S[i]!=[]:
            F.append(S[i])
    if rm:
        return reduceSet(F),Max
    return reduceSet(F)

# Add-on: single Max Sub Config support

def MSC(M,R): 
    A=[]
    for i in R:
        Pid=M.GPid(i[0][0])
        Tid=M.GTid(i[0][2])
        size=len(VCSC(i))
        A.append([Pid,Tid,size])
    TA=[]
    for i in A:
         if [i[0],i[1]] not in TA:
             TA.append([i[0],i[1]])
    MA=[]
    for i in TA:
        maxi=0
        maxj=[]
        count=0
        for j in A:
            if [j[0],j[1]]==i:
                if j[2]>maxi:
                    maxi=j[2]
                    maxj=[count]
                elif j[2]==maxi:
                    maxj.append(count)
            count+=1
        MA.append(random.choice(maxj)) 
    Pn=list(range(len(M.GP)))
    Tn=list(range(len(M.GT)))
    pTn=list(permutations(Tn))
    maxp=0
    maxn=0
    for t in pTn:
        mg=min(len(Pn),len(t))
        cPn=list(combinations(Pn,mg))
        cTn=list(combinations(t,mg))
        for i in cPn:
            for j in cTn:
                pair=[]
                stop=0
                for k in range(mg):
                    pk=[i[k],j[k]]
                    if pk in TA:
                        pair.append(pk)
                    else:
                        stop=1
                        break
                if not stop:
                    mp=0
                    for p in pair:
                        mp+=A[MA[TA.index(p)]][2]
                    if mp>maxn:
                        maxn=mp
                        maxp=pair
    S=[]
    for i in maxp:
        S.append(MA[TA.index(i)])
    return [S]

# Add-on: Cycle Break Support
    
def sizeRF(M):
    CM=CSC(M)
    _,Max=MFS(CM,1)
    return Max