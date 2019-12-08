""" General Configs """

root_path='/home/chengjunyan1/Documents/MSRR/'
# root_path='C:\ChengJunyan1\MSRR'
# root_path=os.path.abspath(os.curdir)

connect_sim_drone=1
connect_real_drone=0
drone_stand_alone=1
multi_threads=1
modules=[0,1,2,3,4,5,6,7,8,9,10]

pf_tolerent_rate=1.00 # How close the contact point to the target contact point
pf_map_size=15 # size of the map in pf
pf_map_resolution=15 # resolution of the map in pf
signal_dt=0.001
planner_buffer_time=3
ros_on=1

""" MSRR Configs """

def C1(M,mode):
    if mode==0:
        Set=M.setGP
        Add=M.addGP
    else:
        Set=M.setGT
        Add=M.addGT
    Gid=Set(11)
    Add(Gid,1,3,0,0)
    Add(Gid,2,1,0,3)
    Add(Gid,2,4,1,1)
    Add(Gid,5,4,0,0)
    Add(Gid,2,6,2,2)
    Add(Gid,7,6,0,0)
    Add(Gid,3,10,2,1,rot=1)
    Add(Gid,10,11,0,0)
    Add(Gid,3,8,1,2,rot=1)
    Add(Gid,8,9,0,0)

def C2(M,mode):
    if mode==0:
        Set=M.setGP
        Add=M.addGP
    else:
        Set=M.setGT
        Add=M.addGT
    Gid=Set(11)
    Add(Gid,1,8,0,0)
    Add(Gid,1,2,1,2)
    Add(Gid,2,6,3,0)
    Add(Gid,2,4,0,0)
    Add(Gid,1,3,2,1)
    Add(Gid,3,7,3,0)
    Add(Gid,3,5,0,0)
    Add(Gid,8,9,3,0)
    Add(Gid,9,10,3,0)
    Add(Gid,10,11,3,0)
    
def C3(M,mode):
    if mode==0:
        Set=M.setGP
        Add=M.addGP
    else:
        Set=M.setGT
        Add=M.addGT  
    Gid=Set(7)
    Add(Gid,1,2,3,0)
    Add(Gid,2,3,3,0)
    Add(Gid,2,4,2,1)
    Add(Gid,4,5,2,1)
    Add(Gid,5,6,0,3)
    Add(Gid,5,7,3,0)

def C4(M,mode):
    if mode==0:
        Set=M.setGP
        Add=M.addGP
    else:
        Set=M.setGT
        Add=M.addGT
    Gid=Set(7)
    Add(Gid,1,2,3,0)
    Add(Gid,2,3,3,0)
    Add(Gid,3,4,3,0)
    Add(Gid,4,5,3,0)
    Add(Gid,5,6,3,0)
    Add(Gid,6,7,3,0)

def C5(M,mode):
    if mode==0:
        Set=M.setGP
        Add=M.addGP
    else:
        Set=M.setGT
        Add=M.addGT  
    Gid=Set(9)
    Add(Gid,1,2,0,0)
    Add(Gid,2,3,3,0)
    Add(Gid,1,6,3,0)
    Add(Gid,6,7,3,0)
    Add(Gid,1,8,1,0)
    Add(Gid,8,9,3,0)
    Add(Gid,1,4,2,0)
    Add(Gid,4,5,3,0)
    
def C6(M,mode):
    if mode==0:
        Set=M.setGP
        Add=M.addGP
    else:
        Set=M.setGT
        Add=M.addGT
    Gid=Set(9)
    Add(Gid,1,2,1,2)
    Add(Gid,2,4,0,3)
    Add(Gid,2,5,3,0)
    Add(Gid,1,8,0,0)
    Add(Gid,8,9,3,0)
    Add(Gid,1,3,2,1)
    Add(Gid,3,6,0,3)
    Add(Gid,3,7,3,0)

def C7(M,mode):
    if mode==0:
        Set=M.setGP
        Add=M.addGP
    else:
        Set=M.setGT
        Add=M.addGT
    Gid=Set(6)
    Add(Gid,1,2,2,1)
    Add(Gid,2,3,3,0)
    Add(Gid,3,4,1,2)
    Add(Gid,4,1,0,3)
    Add(Gid,5,2,1,2)
    Add(Gid,6,3,1,2)
    Add(Gid,5,6,3,0)

# DEMOS

def D1(M): # Multi Graph
    C3(M,0)
    C4(M,0)
    C1(M,1)
    C6(M,1)
    return M

def D2(M): # Module Lack
    C6(M,0)
    C3(M,1)
    C4(M,1)
    return M

def D3(M): # Cycle Exist
    C7(M,0)
    C4(M,1)
    return M

def D4(M): # Contine Reconfig
    C3(M,1)
    C4(M,1)
    C5(M,1)
    return M

# Single Transforms

def T12(M):
    C1(M,0)
    C2(M,1)
    return M

def T34(M):
    C3(M,0)
    C4(M,1)
    return M

def T56(M):
    C5(M,0)
    C6(M,1)
    return M