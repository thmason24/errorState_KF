import numpy as np
import transformations as trans
import matplotlib.pyplot as plt


dt = 0.01
time = np.arange(0,50,dt)
sigma = 0.04

yaw = 0.1*np.sin(time/6) 
pitch = 1.5*yaw
roll =0.1*np.sin(time/4)




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

gain = 0.0000001
gain = 0.5
yawEst = []
yawINS = []
pitchEst = []
pitchINS = []
rollEst = []
rollINS = []
yaw_meas = []
omegas = []
yaw_crct = []
for i,t in enumerate(time):
#   integrate ins
    omega = np.array([0,p[i], q[i], r[i]])
    quat_ins = quat_ins + 0.5*dt*trans.quaternion_multiply(quat_ins, omega)
    yawINS.append(trans.euler_from_quaternion(quat_ins)[2])
    pitchINS.append(trans.euler_from_quaternion(quat_ins)[0]) 
    rollINS.append(trans.euler_from_quaternion(quat_ins)[1])


#   generate truth quaternion
    #quat_meas = trans.quaternion_from_euler(yaw[i],pitch[i],roll[i])
    quat_meas = trans.quaternion_from_euler(roll[i],pitch[i],yaw[i])
        
    yaw_meas.append(trans.euler_from_quaternion(quat_meas)[0])
        
    q_error = trans.quaternion_multiply(quat_meas,trans.quaternion_inverse(quat_ins))
    q_error = q_error / np.linalg.norm(q_error)
    q_error = np.hstack((1,q_error[1:5]))
    #errorAngles = q_error[1:5]
    q_innov = np.hstack((1,gain*q_error[1:5]))
    q_est = trans.quaternion_multiply(q_innov,quat_ins)
    
    
    
    
    yawEst.append(trans.euler_from_quaternion(q_est)[2])
    pitchEst.append(trans.euler_from_quaternion(q_est)[1])
    rollEst.append(trans.euler_from_quaternion(q_est)[0])
    

#   calculate quaternion rotation from INS solution to measured solution


plt.subplot(3,1,1)

plt.plot(time,yaw)
plt.plot(time,yawEst)

plt.subplot(3,1,2)

plt.plot(time,pitch)
plt.plot(time,pitchEst)

plt.subplot(3,1,3)

plt.plot(time,roll)
plt.plot(time,rollEst)

plt.figure()
plt.plot(time,yaw)
#plt.plot(time,yawINS)
plt.plot(time,yawEst)
