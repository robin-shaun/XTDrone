import numpy as np
import random


class ga():
	def __init__(self,vehicle_num,vehicles_speed,target_num,targets,time_lim):
		#vehicles_speed,targets in the type of narray
		self.vehicle_num=vehicle_num
		self.vehicles_speed=vehicles_speed
		self.target_num=target_num
		self.targets=targets
		self.time_lim=time_lim
		self.map=np.zeros(shape=(target_num+1,target_num+1),dtype=float)
		self.pop_size=100
		self.p_cross=0.6
		self.p_mutate=0.005
		for i in range(target_num+1):
			self.map[i,i]=0
			for j in range(i):
				self.map[j,i]=self.map[i,j]=np.linalg.norm(targets[i,:2]-targets[j,:2])
		self.pop=np.zeros(shape=(self.pop_size,vehicle_num-1+target_num-1),dtype=np.int32)
		self.ff=np.zeros(self.pop_size,dtype=float)
		for i in range(self.pop_size):
			for j in range(vehicle_num-1):
				self.pop[i,j]=random.randint(0,target_num)
			for j in range(target_num-1):
				self.pop[i,vehicle_num+j-1]=random.randint(0,target_num-j-1)
			self.ff[i]=self.fitness(self.pop[i,:])
		self.tmp_pop=np.array([])
		self.tmp_ff=np.array([])
		self.tmp_size=0
	
	def fitness(self,gene):
		ins=np.zeros(self.target_num+1,dtype=np.int32)
		seq=np.zeros(self.target_num,dtype=np.int32)
		ins[self.target_num]=1
		for i in range(self.vehicle_num-1):
			ins[gene[i]]+=1
		rest=np.array(range(1,self.target_num+1))
		for i in range(self.target_num-1):
			seq[i]=rest[gene[i+self.vehicle_num-1]]
			rest=np.delete(rest,gene[i+self.vehicle_num-1])
		seq[self.target_num-1]=rest[0]
		i=0          #index of vehicle
		pre=0        #index of last target
		post=0       #index of ins/seq
		t=0
		reward=0
		while i<self.vehicle_num:
			if ins[post]>0:
				i+=1
				ins[post]-=1
				pre=0
				t=0
			else:
				t+=self.targets[pre,3]
				past=self.map[pre,seq[post]]/self.vehicles_speed[i]
				t+=past
				if t<self.time_lim:
					reward+=self.targets[seq[post],2]
				pre=seq[post]
				post+=1
		return reward
	
	def selection(self):
		roll=np.zeros(self.tmp_size,dtype=float)
		roll[0]=self.tmp_ff[0]
		for i in range(1,self.tmp_size):
			roll[i]=roll[i-1]+self.tmp_ff[i]
		for i in range(self.pop_size):
			xx=random.uniform(0,roll[self.tmp_size-1])
			j=0
			while xx>roll[j]:
				j+=1
			self.pop[i,:]=self.tmp_pop[j,:]
			self.ff[i]=self.tmp_ff[j]
			
	def mutation(self):
		for i in range(self.tmp_size):
			flag=False
			for j in range(self.vehicle_num-1):
				if random.random()<self.p_mutate:
					self.tmp_pop[i,j]=random.randint(0,self.target_num)
					flag=True
			for j in range(self.target_num-1):
				if random.random()<self.p_mutate:
					self.tmp_pop[i,self.vehicle_num+j-1]=random.randint(0,self.target_num-j-1)
					flag=True
			if flag:
				self.tmp_ff[i]=self.fitness(self.tmp_pop[i,:])
	
	def crossover(self):
		new_pop=[]
		new_ff=[]
		new_size=0
		for i in range(0,self.pop_size,2):
			if random.random()<self.p_cross:
				x1=random.randint(0,self.vehicle_num-2)
				x2=random.randint(0,self.target_num-2)+self.vehicle_num
				g1=self.pop[i,:]
				g2=self.pop[i+1,:]
				g1[x1:x2]=self.pop[i+1,x1:x2]
				g2[x1:x2]=self.pop[i,x1:x2]
				new_pop.append(g1)
				new_pop.append(g2)
				new_ff.append(self.fitness(g1))
				new_ff.append(self.fitness(g2))
				new_size+=2
		self.tmp_size=self.pop_size+new_size
		self.tmp_pop=np.zeros(shape=(self.tmp_size,self.vehicle_num-1+self.target_num-1),dtype=np.int32)
		self.tmp_pop[0:self.pop_size,:]=self.pop
		self.tmp_pop[self.pop_size:self.tmp_size,:]=np.array(new_pop)
		self.tmp_ff=np.zeros(self.tmp_size,dtype=float)
		self.tmp_ff[0:self.pop_size]=self.ff
		self.tmp_ff[self.pop_size:self.tmp_size]=np.array(new_ff)
	
	def run(self):
		cut=0
		e=self.tmp_ff>cut
		while e.sum()<=self.tmp_size*0.5:
			self.crossover()
			self.mutation()
			self.selection()
			cut=self.tmp_ff.max()*0.9
			e=self.tmp_ff>cut
		i=0
		for j in range(self.tmp_size):
			if self.tmp_ff[i]<self.tmp_ff[j]:
				i=j
		gene=self.tmp_pop[i]
		
		ins=np.zeros(self.target_num+1,dtype=np.int32)
		seq=np.zeros(self.target_num,dtype=np.int32)
		ins[self.target_num]=1
		for i in range(self.vehicle_num-1):
			ins[gene[i]]+=1
		rest=np.array(range(1,self.target_num+1))
		for i in range(self.target_num-1):
			seq[i]=rest[gene[i+self.vehicle_num-1]]
			rest=np.delete(rest,gene[i+self.vehicle_num-1])
		seq[self.target_num-1]=rest[0]
		task_assignment=[[] for i in range(self.vehicle_num)]
		i=0          #index of vehicle
		pre=0        #index of last target
		post=0       #index of ins/seq
		t=0
		while i<self.vehicle_num:
			if ins[post]>0:
				i+=1
				ins[post]-=1
				pre=0
				t=0
			else:
				t+=self.targets[pre,3]
				past=self.map[pre,seq[post]]/self.vehicles_speed[i]
				t+=past
				if t<self.time_lim:
					task_assignment[i].append(seq[post])
				pre=seq[post]
				post+=1
		return task_assignment