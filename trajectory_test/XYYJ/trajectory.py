import numpy as np
import multiprocessing as   mp
from   pycrazyswarm     import * 
import time

class agent():

    def __init__(self,index):

        self.index=index

    def get_input_traj(self,P,T):

        W=np.ones(len(P))
        polyx = np.polyfit(T, P[:,0], 7,w=W)  
        polyy = np.polyfit(T, P[:,1], 7,w=W)
        polyz = np.polyfit(T, P[:,2], 7,w=W)
        self.input_traj=[polyx,polyy,polyz]

        

def begin_trajectory(agent_list):

    start = time.time()

    Ployx=[]
    Ployy=[]
    Ployz=[]

    vel=np.zeros(3)
    acc=np.zeros(3)
    yaw=0.0
    omega=np.zeros(3)


    for agent in agent_list:
        Ployx+=[agent.input_traj[0]]
        Ployy+=[agent.input_traj[1]]
        Ployz+=[agent.input_traj[2]]

    while True:
        t = time.time()-start + 0.0
        for i in range(len(agent_list)):
            
            x=np.polyval(Ployx[i],t)
            y=np.polyval(Ployx[i],t)
            z=np.polyval(Ployz[i],t)

            pos=np.array([x,y,z])
            
            swarm.allcfs.crazyflies[i].cmdFullState( pos, vel, acc, yaw, omega)
        
        time.sleep(0.03)



if __name__=='__main__':

    agent_list=[]
    agent_list+=[agent(0)]
    agent_list+=[agent(1)]

    h=0.2
    K=20

    ini_x=[
        np.array([1.0,1.0,0.5]),
        np.array([1.0,2.0,0.5])
    ]

    target=[
        np.array([3.0,1.0,0.5]),
        np.array([3.0,2.0,0.5])
    ]


    P=[]

    for i in range(len(ini_x)):
        p=np.zeros((K,3))
        p[:,0]=np.linspace(ini_x[i][0],target[i][0],K)
        p[:,1]=np.linspace(ini_x[i][1],target[i][1],K)
        p[:,2]=np.linspace(ini_x[i][2],target[i][2],K)
        P+=[p]
    
    T=h*np.linspace(0,K,K)

    for i in range(len(agent_list)):
        agent_list[i].get_input_traj(P[i],T)

    global swarm
    swarm = Crazyswarm()

    global timeHelper
    timeHelper = swarm.timeHelper

    swarm.allcfs.takeoff(targetHeight=0.5, duration=2.0)

    # 各自到达出发点
    i=0
    for cf in swarm.allcfs.crazyflies:
        # go to prdefined initial position
        pos=ini_x[i]
        cf.goTo(pos, 0, 2.0)
        i+=1
    timeHelper.sleep(2.0)

    # 输入轨迹
    input_traj=mp.Process(target=begin_trajectory, args=(agent_list,))
    input_traj.start()
    start=time.time()
    time.sleep(2.0)
    print("time is:"+str(time.time()-start))
    if input_traj.is_alive():
        input_traj.terminate()
    
    # 降落
    swarm.allcfs.land(targetHeight=0.02, duration=1.5)
    timeHelper.sleep(1.5)