import random

import numpy as np

import math

class Aco():
    
    def __init__(self, vehicle_num, target_num,vehicle_speed, target, time_lim):

        self.num_type_ant = vehicle_num

        self.num_city = target_num+1 #number of cities

        self.group = 200

        self.num_ant = self.group*self.num_type_ant #number of ants

        self.ant_vel = vehicle_speed

        self.cut_time = time_lim
        
        self.oneee = np.zeros((4,1))

        self.target = target

        self.alpha = 1 #pheromone 

        self.beta = 1  #qiwangyinzi

        self.k1 = 0.03

        self.iter_max = 300

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

    def get_assignment_aco(self):

        #distances of nodes

        dis_list = self.distance_matrix()

        dis_mat = np.array(dis_list)

        value_init = self.target[:,2].transpose()

        delay_init = self.target[:,3].transpose()        

        pheromone_mat = np.ones((self.num_type_ant,self.num_city))

        #velocity of ants
        path_new = [[0]for i in range (self.num_type_ant)]

        count_iter = 0

        while count_iter < self.iter_max:

            path_sum = np.zeros((self.num_ant,1))

            time_sum = np.zeros((self.num_ant,1))

            value_sum = np.zeros((self.num_ant,1))

            path_mat=[[0]for i in range (self.num_ant)]

            value = np.zeros((self.group,1))

            for ant in range(self.num_ant):

                ant_type = ant % self.num_type_ant

                visit = 0

                if ant_type == 0:

                    unvisit_list=list(range(1,self.num_city))#have not visit

                for j in range(1,self.num_city):

            #choice of next city

                    trans_list=[]

                    tran_sum=0

                    trans=0

                    #if len(unvisit_list)==0:
                        #print('len(unvisit_list)==0')

                    for k in range(len(unvisit_list)):  # to decide which node to visit

                #trans +=np.power(pheromone_mat[ant_type][visit][unvisit_list[k]],alpha)*np.power(k1*value_mat[visit][unvisit_list[k]],beta)
                        trans +=np.power(pheromone_mat[ant_type][unvisit_list[k]],self.alpha)*np.power(0.05*value_init[unvisit_list[k]],self.beta)

                        trans_list.append(trans)

                    tran_sum = trans
        
                    rand = random.uniform(0,tran_sum)

                    for t in range(len(trans_list)):

                        if(rand <= trans_list[t]):

                            visit_next = unvisit_list[t]

                            break

                        else:
                            
                            continue

                    path_mat[ant].append(visit_next)

                    path_sum[ant] += dis_mat[path_mat[ant][j-1]][path_mat[ant][j]]

                    time_sum[ant] += path_sum[ant] / self.ant_vel[ant_type] + delay_init[visit_next]

                    if time_sum[ant] > self.cut_time:

                        time_sum[ant]-=path_sum[ant] / self.ant_vel[ant_type] + delay_init[visit_next]
                        
                        path_mat[ant].pop()
                
                        break

                    value_sum[ant] += value_init[visit_next]
            
                    unvisit_list.remove(visit_next)#update
            
                    visit = visit_next

                if (ant_type) == self.num_type_ant-1:

                    small_group = int(ant/self.num_type_ant)

                    for k in range (self.num_type_ant):

                        value[small_group]+= value_sum[ant-k]
                  
    #diedai

            print(max(value))
        
            if count_iter == 0:

                value_new = max(value)

                value = value.tolist()
   
                for k in range (0,self.num_type_ant):

                    path_new[k] = path_mat[value.index(value_new)*self.num_type_ant+k]

                    path_new[k].remove(0)

                    #time[k] = time_sum[value.index(value_new)*self.num_type_ant+k]
        
            else:

                if max(value) > value_new:

                    value_new = max(value)

                    value = value.tolist()
     
                    for k in range (0,self.num_type_ant):

                        path_new[k] = path_mat[value.index(value_new)*self.num_type_ant+k]

                        path_new[k].remove(0)

                        #time[k] = time_sum[value.index(value_new)*self.num_type_ant+k]

    # update pheromone

            pheromone_change=np.zeros((self.num_type_ant,self.num_city))

            for i in range(self.num_ant):

                length = len(path_mat[i])

                m = i%self.num_type_ant

                n = int(i/self.num_type_ant)

                for j in range(length-1):   
                    
                    if value[n] == value_new:

                        atten = 0.1

                    else:

                        atten = 0.3
                    
                    #a=value_sum[i]/(np.power((value[n]-value_new),2)+1)

                    pheromone_change[m][path_mat[i][j+1]]+= self.k1*value_init[path_mat[i][j+1]]


            for m in range (self.num_type_ant):

                #print('pheromone_change[m]:',pheromone_change[m])
        
                pheromone_mat[m]=(1-atten)*pheromone_mat[m]+pheromone_change[m]
    
            count_iter += 1

            print('count_iter:',count_iter)
        
            print('the largest value：',value_new)

            print('the best road：',path_new)

        #print('time_limit:',self.cut_time)

        #print('time：',time)

        return path_new




