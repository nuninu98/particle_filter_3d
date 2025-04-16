import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
from scipy.spatial.transform import Slerp

gt = np.loadtxt("gt.txt")
pf_imu = np.loadtxt("pf_imu.txt")
pf_odom = np.loadtxt("pf_odom.txt")
raw_odom = np.loadtxt("raw_odom.txt")

pf_imu_degen10 = np.loadtxt("pf_imu_degen_10.txt")
pf_imu_degen20 = np.loadtxt("pf_imu_degen_20.txt")
pf_imu_degen30 = np.loadtxt("pf_imu_degen_30.txt")
pf_imu_degen40 = np.loadtxt("pf_imu_degen_40.txt")
pf_imu_degen50 = np.loadtxt("pf_imu_degen_50.txt")
def interpolate(p1, p2, t): #pose1(t, x, y, z, qw, qx, qy, qz) pose2 t1 t2 t
    t1 = p1[0]
    t2 = p2[0]
    if t == t1:
        return p1
    elif t == t2:
        return p2
    output = np.zeros(8)
    output[0] = t
    x1 = p1[1]
    x2 = p2[1]
    w = (t - t1) / (t2 - t1)
    output[1] = x1 + (x2 - x1)*w
    
    y1 = p1[2]
    y2 = p2[2]
    output[2] = y1 + (y2 - y1)*w

    z1 = p1[3]
    z2 = p2[3]
    output[3] = z1 + (z2 - z1)*w
    q1 = Rotation.from_quat([p1[5], p1[6], p1[7], p1[4]])
    q2 = Rotation.from_quat([p2[5], p2[6], p2[7], p2[4]])
    key_rots = Rotation.concatenate([q1, q2])
    slerp = Slerp([0, 1], key_rots)
    interp = slerp(w)
    q = interp.as_quat()
    output[4] = q[3]
    output[5] = q[0]
    output[6] = q[1]
    output[7] = q[2]
    return output

def toSE3(pose):
    t = np.zeros((3, 1))
    t[0, 0] = pose[1]
    t[1, 0] = pose[2]  
    t[2, 0] = pose[3]    
    q = Rotation.from_quat([pose[5], pose[6], pose[7], pose[4]])
    R = q.as_matrix()
    P = np.block([[R, t], [0, 0, 0, 1]])
    return P

def diff(pose1, pose2):
    P1 = toSE3(pose1)
    P2 = toSE3(pose2)
    dP = np.linalg.inv(P1).dot(P2)
    dt = np.linalg.norm(dP[0:2, 3])
    dq = Rotation.from_matrix(dP[0:3, 0:3])
    dr = np.linalg.norm(dq.as_rotvec())
    return dt, dr

def rmse(data, ground_truth):
    pid = 0
    length = np.shape(data)[0]
    N = 0
    dt_sum = 0
    dr_sum = 0
    for i in range(length):
        stamp = data[i][0]
        while pid < (np.shape(ground_truth)[0] - 1):
            if stamp >= ground_truth[pid, 0] and stamp <= ground_truth[pid + 1, 0]:
                break
            pid = pid + 1
        if pid >= np.shape(ground_truth)[0]:
            break
        gt_pose = interpolate(gt[pid], gt[pid + 1], stamp)
        pose = data[i]
        dt, dr = diff(pose, gt_pose)
        dt_sum = dt*dt + dt_sum
        dr_sum = dr*dr + dr_sum
        N = N + 1
    return np.array([np.sqrt(dt_sum/N), np.sqrt(dr_sum / N)])

def max_err(data, ground_truth):
    pid = 0
    length = np.shape(data)[0]
    dt_max = 0
    dr_max = 0
    for i in range(length):
        stamp = data[i][0]
        while pid < (np.shape(ground_truth)[0] - 1):
            if stamp >= ground_truth[pid, 0] and stamp <= ground_truth[pid + 1, 0]:
                break
            pid = pid + 1
        if pid >= np.shape(ground_truth)[0]:
            break
        gt_pose = interpolate(gt[pid], gt[pid + 1], stamp)
        pose = data[i]
        dt, dr = diff(pose, gt_pose)
        dt_max = max(dt, dt_max)
        dr_max = max(dr, dr_max)
    return np.array([dt_max, dr_max])

def travel_dist(data):
    length = np.shape(data)[0]
    dist = 0.0
    pose_prev = np.array([0, 0, 0, 0, 0, 0, 0, 1])
    for i in range(length):
        drift = diff(pose_prev, data[i])
        dist = dist + drift[0]
        pose_prev = data[i]
    return dist
        


print("IMU RMSE: ", rmse(pf_imu, gt))
print("PF + DR RMSE: ", rmse(pf_odom, gt))
print("DR RMSE: ", rmse(raw_odom, gt))

print("IMU Max Err: ", max_err(pf_imu, gt))
print("PF + DR Max Err: ", max_err(pf_odom, gt))
print("DR Max Err: ", max_err(raw_odom, gt))

print("GT travel: ", travel_dist(gt))
print("Degen0 travel: ",travel_dist(pf_imu))
print("Degen10 travel: ",travel_dist(pf_imu_degen10))
print("Degen20 travel: ",travel_dist(pf_imu_degen20))
print("Degen30 travel: ",travel_dist(pf_imu_degen30))
print("Degen40 travel: ",travel_dist(pf_imu_degen40))
print("Degen50 travel: ",travel_dist(pf_imu_degen50))

plt.figure(0)
plt.plot(gt[:,1], gt[:,2], '--k', label='Ground Truth')
plt.plot(pf_imu[:,1], pf_imu[:,2], '-r', label='PF (IMU Preintegration)')
plt.plot(pf_odom[:,1], pf_odom[:,2], '-b', label='PF (Deadreckoning)')
plt.plot(raw_odom[:,1], raw_odom[:,2], '-g', label='Deadreckoning')
plt.title('Trajectory Comparison')
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.legend(loc='upper left', prop={'size': 7})


fig1 = plt.figure(1)
ax1 = fig1.add_subplot(1, 1, 1, projection='3d')
ax1.set_zlim(-0.5, 0.5)
ax1.plot3D(pf_imu[:,1], pf_imu[:,2], pf_imu[:, 3], '-r', label='PF (IMU Preintegration)')
ax1.plot3D(gt[:,1], gt[:,2], gt[:, 3], '--k', label='Ground Truth')
ax1.plot3D(pf_odom[:,1], pf_odom[:,2], pf_odom[:, 3], '-b', label='PF (Deadreckoning)')
ax1.plot3D(raw_odom[:,1], raw_odom[:,2], raw_odom[:, 3], '-g', label='Deadreckoning')


plt.figure(2)

plt.plot(gt[:,1], gt[:,2], '--k', label='Ground Truth')
plt.plot(pf_imu[:,1], pf_imu[:,2], '-r', label='0%')
plt.plot(pf_imu_degen10[:,1], pf_imu_degen10[:,2], '-b', label='10%')
plt.plot(pf_imu_degen20[:,1], pf_imu_degen20[:,2], '-g', label='20%')
plt.plot(pf_imu_degen30[:,1], pf_imu_degen30[:,2], '-m', label='30%')
plt.plot(pf_imu_degen40[:,1], pf_imu_degen40[:,2], '-c', label='40%')
plt.plot(pf_imu_degen50[:,1], pf_imu_degen50[:,2], '-y', label='50%')
plt.title('Trajectory Under Degeneracy')
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.legend(loc='center', prop={'size': 7})

fig2 = plt.figure(3)
ax2 = fig2.add_subplot(1, 1, 1, projection='3d')
ax2.set_zlim(-0.5, 0.5)
ax2.plot3D(gt[:,1], gt[:,2], gt[:, 3], '--k', label='Ground Truth')
ax2.plot3D(pf_imu[:,1], pf_imu[:,2], pf_imu[:, 3], '-r', label='0%')
ax2.plot3D(pf_imu_degen10[:,1], pf_imu_degen10[:,2], pf_imu_degen10[:, 3], '-b', label='10%')
ax2.plot3D(pf_imu_degen20[:,1], pf_imu_degen20[:,2], pf_imu_degen20[:, 3], '-g', label='20%')
ax2.plot3D(pf_imu_degen30[:,1], pf_imu_degen30[:,2], pf_imu_degen30[:, 3], '-m', label='30%')
ax2.plot3D(pf_imu_degen40[:,1], pf_imu_degen40[:,2], pf_imu_degen40[:, 3], '-c', label='40%')
ax2.plot3D(pf_imu_degen50[:,1], pf_imu_degen50[:,2], pf_imu_degen50[:, 3], '-y', label='50%')
ax2.set_title('Trajectory Under Degeneracy')
ax2.legend(loc='upper left')
ax2.set_xlabel('x [m]')
ax2.set_ylabel('y [m]')
ax2.set_zlabel('z [m]')

err_rmse = np.array([rmse(pf_imu, gt), rmse(pf_imu_degen10, gt), rmse(pf_imu_degen20, gt), 
       rmse(pf_imu_degen30, gt), rmse(pf_imu_degen40, gt), rmse(pf_imu_degen50, gt)])

# fig3 = plt.figure(4)
# sfig1 = fig3.add_subplot(2, 1, 1)
# sfig2 = fig3.add_subplot(2, 1, 2)
fig3, (sfig1, sfig2) = plt.subplots(2, 1, sharex=True)
rates = [0, 10, 20, 30, 40, 50]
sfig1.plot(rates, err_rmse[:,0], '-ro', label='RMSE')
sfig2.plot(rates, (180.0/np.pi)*err_rmse[:,1], '-ro')
# sfig1.set_title('Translation')
# sfig2.set_title('Rotation')

err_max = np.array([max_err(pf_imu, gt), max_err(pf_imu_degen10, gt), max_err(pf_imu_degen20, gt), 
       max_err(pf_imu_degen30, gt), max_err(pf_imu_degen40, gt), max_err(pf_imu_degen50, gt)])
sfig1.plot(rates, err_max[:,0], '-bo', label='Max Error')
sfig2.plot(rates, (180.0/np.pi)*err_max[:,1], '-bo')
sfig1.set_ylabel('Translation [m]')
# sfig1.legend()
# sfig2.legend()
fig3.legend()
sfig2.set_xlabel('Degeneration Rate [%]')
sfig2.set_ylabel('Heading [Deg]')
plt.suptitle('Trajectory Error With Degeneracy')
plt.show()


