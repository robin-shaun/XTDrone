import numpy as np
import matplotlib.pyplot as plt
import random
import pandas as pd
import copy
from aco import ACO
from geographiclib.geodesic import Geodesic
import math
geod = Geodesic.WGS84

class Env():
    def __init__(self, vehicle_num, target_num, map_size, visualized=True, time_cost=None, repeat_cost=None):
        self.vehicles_position = np.zeros(vehicle_num,dtype=np.int32) # 无人机位置
        self.vehicles_speed = np.zeros(vehicle_num,dtype=np.int32) # 无人机速度
        self.targets = np.zeros(shape=(target_num+1,4),dtype=np.int32) # 目标属性
        if vehicle_num==6:
            self.size='small'
        self.map_size = map_size
        self.speed_range = [10, 10, 10]
        #self.time_lim = 1e6
        self.time_lim = self.map_size / self.speed_range[1]
        self.vehicles_lefttime = np.ones(vehicle_num,dtype=np.float32) * self.time_lim # 剩余的可用时间
        self.distant_mat = np.zeros((target_num+1,target_num+1),dtype=np.float32)
        self.total_reward = 0
        self.reward = 0
        self.visualized = visualized
        self.time = 0
        self.time_cost = time_cost
        self.repeat_cost = repeat_cost
        self.end = False
        self.assignment = [[] for i in range(vehicle_num)] # 最终的分配路径
        self.task_generator()
        
    def task_generator(self):
        for i in range(self.vehicles_speed.shape[0]): # 确定每个无人机的速度
            choose = random.randint(0,2)
            self.vehicles_speed[i] = self.speed_range[choose]
        for i in range(self.targets.shape[0]-1): # 确定每个目标点的坐标位置和奖励、消耗值
            self.targets[i+1,0] = random.randint(1,self.map_size) - 0.5*self.map_size # x position
            self.targets[i+1,1] = random.randint(1,self.map_size) - 0.5*self.map_size # y position
            self.targets[i+1,2] = random.randint(1,10) # reward
            self.targets[i+1,3] = random.randint(5,10) # time consumption to finish the mission  
        for i in range(self.targets.shape[0]): # 计算距离矩阵
            for j in range(self.targets.shape[0]):
                self.distant_mat[i,j] = np.linalg.norm(self.targets[i,:2]-self.targets[j,:2])
        self.targets_value = copy.deepcopy((self.targets[:,2]))
        
    def run(self, assignment, algorithm, play, rond):
        self.assignment = assignment
        self.algorithm = algorithm
        self.play = play
        self.rond = rond
        self.get_total_reward()
        if self.visualized:
            self.visualize()        
            
    def reset(self):
        self.vehicles_position = np.zeros(self.vehicles_position.shape[0],dtype=np.int32)
        self.vehicles_lefttime = np.ones(self.vehicles_position.shape[0],dtype=np.float32) * self.time_lim
        self.targets[:,2] = self.targets_value
        self.total_reward = 0
        self.reward = 0
        self.end = False
        
    def get_total_reward(self):
        for i in range(len(self.assignment)):
            speed = self.vehicles_speed[i]
            for j in range(len(self.assignment[i])):
                position = self.targets[self.assignment[i][j],:4]
                self.total_reward = self.total_reward + position[2]
                if j == 0:
                    self.vehicles_lefttime[i] = self.vehicles_lefttime[i] - np.linalg.norm(position[:2]) / speed - position[3]
                else:
                    self.vehicles_lefttime[i] = self.vehicles_lefttime[i] - np.linalg.norm(position[:2]-position_last[:2]) / speed - position[3]
                position_last = position
                if self.vehicles_lefttime[i] > self.time_lim:
                    self.end = True
                    break
            if self.end:
                self.total_reward = 0
                break
            
    def visualize(self):
        if self.assignment == None:
            plt.scatter(x=0,y=0,s=200,c='k')
            plt.scatter(x=self.targets[1:,0],y=self.targets[1:,1],s=self.targets[1:,2]*10,c='r')
            plt.title('Target distribution')
            plt.savefig('task_pic/'+self.size+'/'+self.algorithm+ "-%d-%d.png" % (self.play,self.rond))
            plt.cla()
        else:
            plt.title('Task assignment by '+self.algorithm +', total reward : '+str(self.total_reward))     
            plt.scatter(x=0,y=0,s=200,c='k')
            plt.scatter(x=self.targets[1:,0],y=self.targets[1:,1],s=self.targets[1:,2]*10,c='r')
            for i in range(len(self.assignment)):
                trajectory = np.array([[0,0,20]])
                for j in range(len(self.assignment[i])):
                    position = self.targets[self.assignment[i][j],:3]
                    trajectory = np.insert(trajectory,j+1,values=position,axis=0)  
                plt.scatter(x=trajectory[1:,0],y=trajectory[1:,1],s=trajectory[1:,2]*10,c='b')
                plt.plot(trajectory[:,0], trajectory[:,1]) 
            plt.savefig('task_pic/'+self.size+'/'+self.algorithm+ "-%d-%d.png" % (self.play,self.rond))
            plt.cla()
            
def evaluate(vehicle_num, target_num, map_size):
    if vehicle_num==6:
        size='small'
        
    re_aco=[[] for i in range(2)]
    for i in range(1):
        env = Env(vehicle_num,target_num,map_size,visualized=True)
        for j in range(1):
            aco = ACO(vehicle_num,target_num,env.vehicles_speed,env.targets,env.time_lim)
            path_new, time = aco.run()
            env.run(path_new,'ACO',i+1,j+1)
            re_aco[i].append((env.total_reward,time))
            env.reset()

    
    
if __name__=='__main__':
    '''
    vehicle number: scalar
    speeds of vehicles: array
    target number: scalar
    targets: array, the first line is depot, the first column is x position, the second column is y position, the third column is reward and the forth column is time consumption to finish the mission
    time limit: scalar
    '''
    evaluate(6,30,1e3)
