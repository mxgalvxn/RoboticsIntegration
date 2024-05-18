import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

r = 5
h = 10
d = 19.1

pi = 3.1416
tao = 0.001

x = np.array([[0,0,0]]).T
u = np.array([[0.1,0.1]]).T

qd = np.array([[1.0,1.0]]).T

Phi = np.array([[r/d, -r/d]])
x_hat = np.array([[0,0,0]]).T

P = np.array([[0,0,0],[0,0,0],[0,0,0]])
Q = P

R = np.array([[0.001,0,0],[0,0.001,0],[0,0,0.001]])
dummy = np.concatenate((np.array([[0]]),x.T),axis=1)
out = np.concatenate((dummy,x_hat.T),axis=1)

for i in np.arange(0.001,0.5,tao):
    D = np.array([[r/2*np.cos(x[2][0])-h*r/d*np.sin(x[2][0]), r/2*np.cos(x[2][0])+h*r/d*np.sin(x[2][0])],
                  [r/2*np.sin(x[2][0])+h*r/d*np.cos(x[2][0]), r/2*np.sin(x[2][0])+h*r/d*np.cos(x[2][0])] ])
    D_inv = -h*r*r/d * np.array([ [D[1][1],D[0][1]],
                                  [D[1][0],D[0][0]] ])    
    M = np.array([D[0],
                  D[1],
                  Phi[0] ])
    #Kalman Filter
    x_hat = x_hat + tao*(M@u+ 10*P@np.linalg.inv(R)@(x_hat-x))
    P = Q - P@np.linalg.inv(R)@P
    
    q = np.array([x[0],x[1]])
    
    e = qd-q
    u = D_inv@(0.1*e)
    
    #Physical Robot Simulation
    
    x = x+tao*(M@u)
    
    dummy = np.concatenate((np.array([[i]]),x.T),axis=1)
    dummy = np.concatenate((dummy,x_hat.T),axis=1)
    out = np.concatenate((out,dummy),axis=0)
    
out_df = pd.DataFrame(out,columns=["t","x","y","theta","xh","yh","th"])
out_df.plot(x="t",y=["x","xh"])
plt.show()

