import matplotlib.pyplot as plt
  
poses = []
t_poses = []
velocities = []
t_velocities = []
accelerations = []
t_accelerations = []
  
f_poses = open('poses.txt','r')
for row_poses in f_poses:
    if not row_poses:
        continue
    row_poses = row_poses.split(' ')
    poses.append(float(row_poses[1]))
    t_poses.append(float(row_poses[0]))

plt.figure(1)
plt.plot(t_poses, poses)  
plt.xlabel("s")
plt.ylabel("m")
plt.title('Poses', fontsize = 20)
plt.legend()

f_velocities = open('velocities.txt','r')
for row_velocities in f_velocities:
    if not row_velocities:
        continue
    row_velocities = row_velocities.split(' ')
    velocities.append(float(row_velocities[1]))
    t_velocities.append(float(row_velocities[0]))

plt.figure(2)
plt.plot(t_velocities, velocities)  
plt.xlabel("s")
plt.ylabel("m/s")
plt.title('Velocities', fontsize = 20)
plt.legend()

plt.figure(3)
f_accelerations = open('accelerations.txt','r')
for row_acceleration in f_accelerations:
    if not row_acceleration:
        continue
    row_acceleration = row_acceleration.split(' ')
    accelerations.append(float(row_acceleration[1]))
    t_accelerations.append(float(row_acceleration[0]))
  
plt.plot(t_accelerations, accelerations) 
plt.xlabel("s")
plt.ylabel("m/s²") 
plt.title('Accelerations', fontsize = 20)
plt.legend()
plt.show()