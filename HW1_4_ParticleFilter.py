# -*- coding: utf-8 -*-
"""
Created on Sat Mar 24 01:41:30 2018

@author: Raj Soni
"""
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import math
pi=math.pi
oo=pd.read_csv('odom.csv')
ya=pd.read_csv('yaw.csv')

noise=0.2

## Landmarks
lx=[0,0,1.337,5.227,3.284,6.426,9.547,10.180,10.180,9.537]
ly=[8.985,3.597,0,0.439,13.810,13.810,13.810,9.577,3.542,0]

landmarks=np.zeros(shape=(10,2))

for i in range(10):
    landmarks[i][0]=lx[i]
    landmarks[i][1]=ly[i]

## Sub-sampling the odom file to get initial set of particles 
subsampling_no=5
x=oo["field.pose.pose.position.x"][::subsampling_no]*-1
x=np.array(x)
y=oo["field.pose.pose.position.y"][::subsampling_no]
y=np.array(y)
yaa=ya[::subsampling_no]
yaa=np.array(yaa)
no=len(yaa)      ## Number of particles  


particles=np.zeros(shape=(no,3))
for i in range(len(yaa)):
    particles[i][0]=x[i]
    particles[i][1]=y[i]
    particles[i][2]=yaa[i]


## Defining a particle for obtaining the groundtruth sensor measurements
gx=5 
gy=6
gya=-0.00226893


def gaussian(mu,sigma,x):
        return np.exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / np.sqrt(2.0 * pi * (sigma ** 2))

def move(x,y,orientation,d,angle): 
        result=0
        s=(np.random.normal((angle,noise)))%(2*(math.pi))
        #print(s)
        d=np.random.normal(d,noise)
        beta=(d/20)*math.tan(s[1])
        if (beta>=0.001):
            r=d/beta
            cx=x-(math.sin(orientation)*r)
            cy=y+(math.cos(orientation)*r)
            x=cx+((math.sin(orientation+beta))*r)
            y=cy-((math.cos(orientation+beta))*r)
            orientation=(orientation+beta)%(2*math.pi)
        else:    
            x=x+(d*math.cos(orientation))
            y=y+(d*math.sin(orientation))
            orientation=(orientation+beta)%(2*math.pi)
        result=[x,y,orientation]
        return result
   
def sense(x,y,orientation,n):
        Z = []
        if(n==0):
              for i in range(len(landmarks)): 
                  z1=((math.atan2(landmarks[i][0]-y,landmarks[i][1]-x))-orientation)
                  z1=z1%(2*math.pi)
                  Z.append(z1)
        if(n==1):
              for i in range(len(landmarks)): 
                  z1=((math.atan2(landmarks[i][0]-y,landmarks[i][1]-x))-orientation)
                  z1=z1%(2*math.pi)
                  z1=z1+np.random.normal(0, noise)
                  Z.append(z1)
        return Z
    
def beam(x,y,o,measurements):
        predicted_measurements = sense(x,y,o,0)
        # compute errors    
        prob = 1.0;
        #print("m:",len(measurements))
        #print("pm:",len(predicted_measurements))
        for i in range(len(landmarks)):
            prob *= gaussian(measurements[i],0.5,predicted_measurements[i])
        return prob
    
def particle_filter(p,d,angle,measurements):
    N=p.shape[0]
    for t in range(n_motion):
        ## Plotting the particles position after each iteration 
        plt.scatter(p[:,0],p[:,1])
        plt.title("Iteration:"+str(t+1))
        plt.ylim(min_x,max_x)
        plt.xlim(min_y,max_y)
        plt.show()
        p2= np.zeros(shape=(no,3))
        for i in range(N):
            rr=(move(p[i][0],p[i][1],p[i][2],d,angle))
            p2[i][0]=rr[0]
            p2[i][1]=rr[1]
            p2[i][2]=rr[2]
        p = p2
        w=[]
        for i in range(N):
            w.append(beam(p[i][0],p[i][1],p[i][2],measurements[t]))
        p3= np.zeros(shape=(no,3))
        index = int(np.random.random() * N)
        beta = 0.0
        mw = max(w)
        ss=0
        ###  Sampling based on the weights of each particle
        for i in range(N):
            beta += np.random.random() * 2.0 * mw
            while beta > w[index]:
                beta -= w[index]
                index = (index + 1) % N
            p3[ss][0]=p[index][0]
            p3[ss][1]=p[index][1]
            p3[ss][2]=p[index][2]
            ss+=1
        p = p3
    return p  


  
gxx=gx
gyy=gy
gyaa=gya    
sensors=[]

## Defining the parameters(distance and angle) for performing move
distance=0.9
angle=1*(math.pi/2)


## To obtain the ground truth sensor measurements with respect to the landmarks, we will sense after performing 
## the motion for n_motion=10 times 
n_motion=10

for i  in range(n_motion):   
    r=move(gxx,gyy,gyaa,distance,angle) 
    gxx=r[0]
    gyy=r[1]
    gyaa=r[2]
    ## Ground Truth Sensor
    sensors.append(sense(gxx,gyy,gyaa,1))

## Plotting the movement

print("\nMotion for particle to get the ground truth sensor measurements with respect to the 10 landmarks after doing 10 move and sense steps")   
plt.figure()
plt.title("Movement after n_motion=10 iterations")
plt.scatter(gx,gy,label="Initial Position")
plt.scatter(gxx,gyy,label="Final Position")
plt.legend()
plt.show()
print("Ground truth Sensor measurements:",sensors)
gxxx=gyy
gyyy=gxx

## Describing the axis for plotting graphs based on obtained final position of particle,to accomodate it in the 
## graph
min_x=-0.5
max_x=10
min_y=-0.5
max_y=10
if(gxxx<-0.5):
    min_x=gxxx-1
if(gxxx>10):
    max_x=gxxx+1
if(gyyy<-0.5):
    min_y=gyyy-1
if(gyyy>10):
    max_y=gyyy+1
    

print("\n Applying Particle Filter Algorithm:")
plt.figure()
result=particle_filter(particles,distance,angle,sensors)

print("Ground truth location coordinates:")
print("x:",gxx)
print("y:",gyy)
print("orientation:",gyaa)


print("\nEstimated coordinates by Particle Algorithm:")
print("x:",np.mean(result[:,0]))
print("y:",np.mean(result[:,1]))
print("orientation:",np.mean(result[:,2]))