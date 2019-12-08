# Visualization Tools

from Planner.UT import *
import time

def Mprint(MP,msc=0,rdcb=0,fast=1,rdmp=0,update=1):
    print('\nPresent Topology:')
    GPid=0
    for i in MP.M.PTopo:
        print('\n| GP',GPid,'|','Location:',i[0],'Rotation:',i[1])
        GPid=GPid+1
    print('\nTarget Topology:')
    GTid=0
    for i in MP.M.TTopo:
        print('\n| GT',GTid,'|','Location:',i[0],'Rotation:',i[1])
        GTid=GTid+1
    print('\nCycles in GP:')
    if MP.M.PCycle!=[]:
        print('')
        print(Vcycle(MP.M.PCycle))
    else:
        print('\n*No cycles in GP')
    print('\nCycles in GT:')
    if MP.M.TCycle!=[]:
        print('\n',Vcycle(MP.M.TCycle))
    else:
        print('\n*No cycles in GT')
    print('\n------------------------------------','\n')
    print('Self-Assembly Planning:')
    if MP.SAssembly(rdcb=rdcb)!=-1:
        Gid=0
        for sa in MP.SA:
            print('\n| G',Gid,'|\n')
            c=0
            Gid=Gid+1
            for i in sa:
                if c==0:
                    print('Root Module:',i)
                else:
                    print('\nStage',c)
                    print(i)
                c=c+1
    ts=time.time()
    print('\n------------------------------------','\n')
    print('Self-Reconfiguration Planning:')
    if MP.RPlanner(fast=fast,rdmp=rdmp,rdcb=rdcb,detail=1,msc=msc,update=update)!=-1:
        print('\nI.Clusters','\n')
        print(MP.SR[0],'\n')
        print('II.Root Modules','\n')
        print(MP.SR[1],'\n')
        print('II.Graph Centers','\n')
        print(MP.SR[2],'\n')
        print('III.Undocking','\n')
        if MP.SR[3]==[]:
            print('*No Module Undocking\n')
        else:
            print(MP.SR[3],'\n')
        print('IV.Docking Sequence')
        if MP.SR[4]==[]:
            print('\n*No Module Docking')
        else:
            c=1
            for i in MP.SR[4]:
                print('\nStage',c)
                print(i)
                c=c+1
    print('\n------------------------------------','\n')
    print('Time:',time.time()-ts)
    print('Estimated Cost:',MP.SR[6],'\n')

# Visualization

def Mvisual(MP):
    pass