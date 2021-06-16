import numpy as np
import matplotlib.pyplot as plt
import random

class Env():
    def __init__(self, vehicle_num, target_num, map_size, visualized):
        self.visualized = visualized
        self.vehicles_position = np.zeros(vehicle_num, dtype=np.int32)
        self.vehicles_speed = np.zeros(vehicle_num, dtype=np.int32)
        self.targets = np.zeros(shape=(target_num + 1, 4), dtype=np.int32)
        self.distant_mat = np.zeros((target_num + 1, target_num + 1), dtype=np.float32)
        self.map_size_x = map_size[0]
        self.map_size_y = map_size[1]
        self.speed_range = [1, 1, 1]
        self.time_lim = max(self.map_size_x, self.map_size_y )/ self.speed_range[0]
        self.vehicles_lefttime = np.ones(vehicle_num, dtype=np.float32) * self.time_lim
        self.assignment = [[] for i in range(vehicle_num)]
        self.total_reward = 0
        for i in range(self.vehicles_speed.shape[0]):
            speed_type = random.randint(0,2)
            self.vehicles_speed[i] = self.speed_range[speed_type]
        self.time_limit = max(self.map_size_x, self.map_size_y) / self.speed_range[0]
        self.end = False
        self.task_generator()

    def task_generator(self):
        for i in range(self.targets.shape[0]-1):
            self.targets[i+1,0] = random.randint(1,self.map_size_x) - 0.5*self.map_size_x # x position
            self.targets[i+1,1] = random.randint(1,self.map_size_y) - 0.5*self.map_size_y # y position
            self.targets[i + 1, 2] = random.randint(1, 10)  # reward
            self.targets[i + 1, 3] = random.randint(5, 30)  # time consumption to finish the mission
            self.targets[self.targets.shape[0]-2, 0] = 20
            self.targets[self.targets.shape[0] - 2, 1] = -27
            for i in range(self.targets.shape[0]):
                for j in range(self.targets.shape[0]-i):
                    self.distant_mat[i, j] = np.linalg.norm(self.targets[i, :2] - self.targets[j, :2])

    def visualize(self):
        if self.assignment == None:
            plt.scatter(x=0,y=0,s=200,c='k')
            plt.scatter(x=self.targets[1:,0],y=self.targets[1:,1],s=self.targets[1:,2]*10,c='r')
            plt.title('Target distribution')
            plt.savefig('task_pic/'+'/'+self.algorithm+ "-%d-%d.png" % (self.play,self.rond))
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
            plt.savefig('task_picture/'+self.algorithm+ "-%d-%d.png" % (self.play,self.rond))
            plt.cla()

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





