import random
import numpy as np
import math
import time
import os
from geographiclib.geodesic import Geodesic
import math
geod = Geodesic.WGS84

# 计算方位角函数
def azimuthAngle(pos):
    if(pos[0] > 0 and pos[1] > 0):
        angle = math.atan(pos[1] / pos[0])
    elif(pos[0] > 0 and pos[1] < 0):
        angle = math.atan(pos[1] / pos[0]) + 2 * math.pi
    elif(pos[0] < 0 and pos[1] > 0):
        angle = math.atan(pos[1] / pos[0]) + math.pi
    elif(pos[0] < 0 and pos[1] < 0):
        angle = math.atan(pos[1] / pos[0]) + math.pi

    return (angle * 180 / math.pi)

def angle_dis(pos):
    angle = azimuthAngle(pos)
    distance = math.sqrt(pow(pos[0],2)+pow(pos[1],2))
    return angle, distance

class ACO():
    def __init__(self, vehicle_num, target_num,vehicle_speed, target, time_lim):
        self.num_type_ant = vehicle_num
        self.num_city = target_num+1 #number of cities
        self.group = 200 # 200组蚂蚁，每组蚂蚁有6只
        self.num_ant = self.group*self.num_type_ant #number of ants
        self.ant_vel = vehicle_speed
        self.cut_time = time_lim
        self.oneee = np.zeros((4,1))
        self.target = target
        self.alpha = 1 #pheromone 
        self.beta = 2  
        self.k1 = 0.03
        self.iter_max = 50
    #matrix of the distances between cities 
    def distance_matrix(self):
        dis_mat = []
        for i in range(self.num_city):
            dis_mat_each = []
            for j in range(self.num_city):
                dis = math.sqrt(pow(self.target[i][0]-self.target[j][0],2)+pow(self.target[i][1]-self.target[j][1],2))
                dis_mat_each.append(dis)
            dis_mat.append(dis_mat_each)
        return dis_mat
    def run(self):
        print("ACO start, pid: %s" % os.getpid())
        start_time = time.time()
        #distances of nodes
        dis_list = self.distance_matrix()
        dis_mat = np.array(dis_list)
        value_init = self.target[:,2].transpose()
        delay_init = self.target[:,3].transpose()        
        pheromone_mat = np.ones((self.num_type_ant,self.num_city,self.num_city)) # 信息素矩阵6*31*31，6只蚂蚁在31*31条的路径上留下的信息素
        #velocity of ants
        path_new = [[0]for i in range (self.num_type_ant)]
        count_iter = 0
        while count_iter < self.iter_max:
            path_sum = np.zeros((self.num_ant,1))
            time_sum = np.zeros((self.num_ant,1))
            value_sum = np.zeros((self.num_ant,1))
            path_mat=[[0]for i in range (self.num_ant)]
            value = np.zeros((self.group,1))
            atten = np.ones((self.num_type_ant,1)) * 0.2
            for ant in range(self.num_ant): # 对每只蚂蚁
                ant_type = ant % self.num_type_ant
                visit = 0 # 当前节点
                if ant_type == 0:
                    unvisit_list=list(range(1,self.num_city))#have not visit
                for j in range(1,self.num_city): # 行走的第几个城市
            #choice of next city
                    trans_list=[]
                    tran_sum=0
                    trans=0
                    #if len(unvisit_list)==0:
                        #print('len(unvisit_list)==0')
                    for k in range(len(unvisit_list)):  # 计算本组蚂蚁没走过的城市的启发式函数
                        trans +=np.power(pheromone_mat[ant_type][visit][unvisit_list[k]],self.alpha)*np.power(value_init[unvisit_list[k]]*self.ant_vel[ant_type]/(dis_mat[visit][unvisit_list[k]]*delay_init[unvisit_list[k]]),self.beta)
                        #trans +=np.power(pheromone_mat[ant_type][unvisit_list[k]],self.alpha)*np.power(0.05*value_init[unvisit_list[k]],self.beta)
                        trans_list.append(trans)
                    tran_sum = trans        
                    rand = random.uniform(0,tran_sum)
                    for t in range(len(trans_list)): # 轮盘赌选择下一个点
                        if(rand <= trans_list[t]):
                            visit_next = unvisit_list[t]
                            break
                        else:        
                            continue
                    path_mat[ant].append(visit_next) # 每只蚂蚁走过的路径
                    path_sum[ant] += dis_mat[path_mat[ant][j-1]][path_mat[ant][j]] # 每只蚂蚁走过的总路径长度
                    time_sum[ant] += path_sum[ant] / self.ant_vel[ant_type] + delay_init[visit_next] # 每只蚂蚁使用的时间
                    if time_sum[ant] > self.cut_time:
                        time_sum[ant]-=path_sum[ant] / self.ant_vel[ant_type] + delay_init[visit_next]                      
                        path_mat[ant].pop()                
                        break
                    value_sum[ant] += value_init[visit_next] # 每只蚂蚁的value
                    unvisit_list.remove(visit_next)#update
                    visit = visit_next
                if (ant_type) == self.num_type_ant-1:
                    small_group = int(ant/self.num_type_ant)
                    for k in range (self.num_type_ant):
                        value[small_group]+= value_sum[ant-k] # 每组蚂蚁的value
            #iteration
            if count_iter == 0:
                value_new = max(value)
                value = value.tolist()
                for k in range (0,self.num_type_ant):
                    path_new[k] = path_mat[value.index(value_new)*self.num_type_ant+k] # 200组蚂蚁中value最大的蚂蚁的路径
                    path_new[k].remove(0) # 移除起点
            else:
                if max(value) > value_new:
                    value_new = max(value)
                    value = value.tolist()
                    for k in range (0,self.num_type_ant):
                        path_new[k] = path_mat[value.index(value_new)*self.num_type_ant+k]
                        path_new[k].remove(0)

            #update pheromone
            pheromone_change = np.zeros((self.num_type_ant,self.num_city,self.num_city))
            for i in range(self.num_ant):
                length = len(path_mat[i])
                m = i%self.num_type_ant
                n = int(i/self.num_type_ant)
                for j in range(length-1):   
                    pheromone_change[m][path_mat[i][j]][path_mat[i][j+1]]+= value_init[path_mat[i][j+1]]*self.ant_vel[m]/(dis_mat[path_mat[i][j]][path_mat[i][j+1]]*delay_init[path_mat[i][j+1]])
                atten[m] += (value_sum[i]/(np.power((value_new-value[n]),4)+1))/self.group

            for k in range (self.num_type_ant):
                pheromone_mat[k]=(1-atten[k])*pheromone_mat[k]+pheromone_change[k]
            count_iter += 1

        print("ACO result:", path_new)
        middle_point = [47.3978502, 8.5455944]
        path_new_point = []
        for z in range(len(path_new)):
            path_new_point.append(self.target[path_new[z], 0:2])
        print("ACO path points m: ", path_new_point)
        
        path_ang_dis = []
        log = []
        for i in range(len(path_new_point)):
            a_path_ang_dis = []
            a_log = []
            for j in range(len(path_new_point[i])):
                angle, distance = angle_dis(path_new_point[i][j])
                g = geod.Direct(middle_point[0], middle_point[1], angle, distance)
                a_log.append([g['lat2'], g['lon2']])
                a_path_ang_dis.append([angle, distance])
            path_ang_dis.append(a_path_ang_dis)
            log.append(a_log)
        
        print("ACO angle and distance: ", path_ang_dis)

        print("ACO log: ", log)
        end_time = time.time()
        print("ACO time:", end_time - start_time)
        return path_new, end_time - start_time

