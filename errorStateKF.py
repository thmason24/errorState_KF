import numpy as np
import transformations as trans
import matplotlib.pyplot as plt


dt = 0.1
time = np.arange(0,50,dt)
yaw = np.sin(time/2)
pitch = yaw
roll = yaw

yawRate = np.diff(yaw) / np.diff(time)
rollRate = np.diff(roll) / np.diff(time)
pitchRate = np.diff(pitch) / np.diff(time)

yaw = np.delete(yaw,0)
pitch = np.delete(pitch,0)
roll = np.delete(roll,0)
time = np.delete(time,0)


p = rollRate - yawRate * np.sin(pitch)
q = pitchRate * np.cos(roll) + yawRate * np.sin(roll) * np.cos(pitch)
r = -pitchRate * np.sin(roll) + yawRate * np.cos(roll) * np.cos(pitch)

quat_ins = trans.quaternion_from_euler(0,0,0)

angles = []
omegas = []
for i,t in enumerate(time):
#integrate ins
    omega = np.array([0,p[i], q[i], r[i]])
    quat_ins = quat_ins + 0.5*dt*trans.quaternion_multiply(quat_ins, omega)
    angles.append(trans.euler_from_quaternion(quat_ins)[0])
    omegas.append(p[i])


plt.plot(time,yaw)
plt.plot(time,angles)
#plt.plot(time,omegas)
