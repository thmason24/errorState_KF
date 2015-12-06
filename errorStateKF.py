import numpy as np
import transformations as trans
import matplotlib.pyplot as plt


dt = 0.01
time = np.arange(0,500,dt)
sigma = 0.05

yaw = np.sin(time/6) 
pitch = 1.5*yaw
roll = np.sin(time/4)




yawRate = np.diff(yaw) / np.diff(time)  #psi
pitchRate = np.diff(pitch) / np.diff(time) #phi
rollRate = np.diff(roll) / np.diff(time)   #theta

yaw = np.delete(yaw,0)
pitch = np.delete(pitch,0)
roll = np.delete(roll,0)
time = np.delete(time,0)


p = rollRate - yawRate * np.sin(pitch)
q = pitchRate * np.cos(roll) + yawRate * np.sin(roll) * np.cos(pitch)
r = -pitchRate * np.sin(roll) + yawRate * np.cos(roll) * np.cos(pitch)

p = p + sigma * np.random.randn(len(p))
q = q + sigma * np.random.randn(len(q))
r = r + sigma * np.random.randn(len(r))



quat_ins = trans.quaternion_from_euler(0,0,0)

yawEst = []
pitchEst = []
rollEst = []

omegas = []
for i,t in enumerate(time):
#integrate ins
    omega = np.array([0,p[i], q[i], r[i]])
    quat_ins = quat_ins + 0.5*dt*trans.quaternion_multiply(quat_ins, omega)
    yawEst.append(trans.euler_from_quaternion(quat_ins)[2])
    pitchEst.append(trans.euler_from_quaternion(quat_ins)[1]) 
    rollEst.append(trans.euler_from_quaternion(quat_ins)[0])


plt.subplot(3,1,1)

plt.plot(time,yaw)
plt.plot(time,yawEst)

plt.subplot(3,1,2)

plt.plot(time,pitch)
plt.plot(time,pitchEst)

plt.subplot(3,1,3)

plt.plot(time,roll)
plt.plot(time,rollEst)
