import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
from scipy.spatial.transform import Slerp

folder = "hospital"
gt = np.loadtxt(folder + "/gt.txt")
pf_imu = np.loadtxt(folder+"/pf_imu.txt")
pf_odom = np.loadtxt(folder+"/pf_odom.txt")
raw_odom = np.loadtxt(folder+"/raw_odom.txt")

degen10 = np.loadtxt(folder + "/degen_10.txt")
degen30 = np.loadtxt(folder + "/degen_30.txt")
degen50 = np.loadtxt(folder + "/degen_50.txt")

degen10_denoise = np.loadtxt(folder + "/degen_10_denoise.txt")
degen30_denoise = np.loadtxt(folder + "/degen_30_denoise.txt")
degen50_denoise = np.loadtxt(folder + "/degen_50_denoise.txt")


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
        if pid >= np.shape(ground_truth)[0]-1:
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
        if pid >= np.shape(ground_truth)[0]-1:
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
        
def plotTrajectory(traj, figure, label, color, use_3d=False):
    if not use_3d:
        figure.plot(traj[:,1], traj[:,2], color, label=label)
    else:
        figure.plot(traj[:,1], traj[:,2], traj[:,3],color, label=label)
print("PF+IMU RMSE: ", rmse(pf_imu, gt))
print("PF+IMU Max Err: ", max_err(pf_imu, gt))

print("PF+ODOM RMSE: ", rmse(pf_odom, gt))
print("PF+ODOM Max Err: ", max_err(pf_odom, gt))

print("Degen10 (Denoise) RMSE: ", rmse(degen10_denoise, gt))
print("Degen10 (Denoise) Max Err: ", max_err(degen10_denoise, gt))
print("Degen30 (Denoise) RMSE: ", rmse(degen30_denoise, gt))
print("Degen30 (Denoise) Max Err: ", max_err(degen30_denoise, gt))
print("Degen50 (Denoise) RMSE: ", rmse(degen50_denoise, gt))
print("Degen50 (Denoise) Max Err: ", max_err(degen50_denoise, gt))

print("GT travel: ", travel_dist(gt))
print("Degen0 travel: ",travel_dist(pf_imu))
print("Degen10 (Denoise) travel: ",travel_dist(degen10_denoise))
print("Degen30 (Denoise) travel: ",travel_dist(degen30_denoise))
print("Degen50 (Denoise) travel: ",travel_dist(degen50_denoise))

fig0 = plt.figure(0)
fig0_ax = fig0.add_subplot(1, 1, 1)
plotTrajectory(gt, fig0_ax, 'Ground Truth', '--k')
plotTrajectory(pf_imu, fig0_ax, 'PF (IMU Preintegration)', '-r')
plotTrajectory(pf_odom, fig0_ax, 'PF (Deadreckoning)', '-b')
plotTrajectory(raw_odom, fig0_ax, 'Deadreckoning', '-g')
plt.title('Trajectory Comparison')
plt.legend()
plt.xlabel('x [m]')
plt.ylabel('y [m]')


fig1 = plt.figure(1)
fig1_ax = fig1.add_subplot(1, 1, 1, projection='3d')
plotTrajectory(gt, fig1_ax, 'Ground Truth', '--k', True)
plotTrajectory(pf_imu, fig1_ax, 'PF (IMU Preintegration)', '-r', True)
plotTrajectory(pf_odom, fig1_ax, 'PF (Deadreckoning)', '-b', True)
plotTrajectory(raw_odom, fig1_ax, 'Deadreckoning', '-g', True)
fig1_ax.set_zlim(-5, 5)
fig1_ax.set_title('Trajectory Comparison')
fig1_ax.legend(loc='upper left')

fig2 = plt.figure(2)
fig2_ax = fig2.add_subplot(1, 1, 1)
plotTrajectory(gt, fig2_ax, 'Ground Truth', '--k')
plotTrajectory(degen10, fig2_ax, '10%', '-r')
plotTrajectory(degen30, fig2_ax, '30%', '-g')
plotTrajectory(degen50, fig2_ax, '50%', '-b')
fig2_ax.set_title('Trajectory Under Degeneracy')
fig2_ax.set_xlabel('x [m]')
fig2_ax.set_ylabel('y [m]')
fig2_ax.legend(loc='upper left')

fig3 = plt.figure(3)
fig3_ax = fig3.add_subplot(1, 1, 1, projection='3d')
plotTrajectory(gt, fig3_ax, 'Ground Truth', '--k', True)
plotTrajectory(degen10, fig3_ax, '10%', '-r', True)
plotTrajectory(degen30, fig3_ax, '30%', '-g', True)
plotTrajectory(degen50, fig3_ax, '50%', '-b', True)
fig3_ax.set_title('Trajectory Under Degeneracy')
fig3_ax.set_xlabel('x [m]')
fig3_ax.set_ylabel('y [m]')
fig3_ax.set_zlabel('z [m]')
fig3_ax.set_zlim(-5, 5)
fig3_ax.legend(loc='upper left')

fig4 = plt.figure(4)
fig4_ax = fig4.add_subplot(1, 1, 1)
plotTrajectory(gt, fig4_ax, 'Ground Truth', '--k')
plotTrajectory(degen10_denoise, fig4_ax, '10%', '-r')
plotTrajectory(degen30_denoise, fig4_ax, '30%', '-g')
plotTrajectory(degen50_denoise, fig4_ax, '50%', '-b')
fig4_ax.set_title('Trajectory Under Degeneracy (Denoised)')
fig4_ax.set_xlabel('x [m]')
fig4_ax.set_ylabel('y [m]')
fig4_ax.legend(loc='upper left')

fig5 = plt.figure(5)
fig5_ax = fig5.add_subplot(1, 1, 1, projection='3d')
plotTrajectory(gt, fig5_ax, 'Ground Truth', '--k', True)
plotTrajectory(degen10_denoise, fig5_ax, '10%', '-r', True)
plotTrajectory(degen30_denoise, fig5_ax, '30%', '-g', True)
plotTrajectory(degen50_denoise, fig5_ax, '50%', '-b', True)
fig5_ax.set_title('Trajectory Under Degeneracy(Denoised)')
fig5_ax.set_xlabel('x [m]')
fig5_ax.set_ylabel('y [m]')
fig5_ax.set_zlabel('z [m]')
fig5_ax.set_zlim(-5, 5)
fig5_ax.legend(loc='upper left')



# err_rmse = np.array([rmse(pf_imu, gt), rmse(pf_imu_degen10, gt), rmse(pf_imu_degen20, gt), 
#        rmse(pf_imu_degen30, gt), rmse(pf_imu_degen40, gt), rmse(pf_imu_degen50, gt)])

# # fig3 = plt.figure(4)
# # sfig1 = fig3.add_subplot(2, 1, 1)
# # sfig2 = fig3.add_subplot(2, 1, 2)
# fig3, (sfig1, sfig2) = plt.subplots(2, 1, sharex=True)
# rates = [0, 10, 20, 30, 40, 50]
# sfig1.plot(rates, err_rmse[:,0], '-ro', label='RMSE')
# sfig2.plot(rates, (180.0/np.pi)*err_rmse[:,1], '-ro')
# # sfig1.set_title('Translation')
# # sfig2.set_title('Rotation')

# err_max = np.array([max_err(pf_imu, gt), max_err(pf_imu_degen10, gt), max_err(pf_imu_degen20, gt), 
#        max_err(pf_imu_degen30, gt), max_err(pf_imu_degen40, gt), max_err(pf_imu_degen50, gt)])
# sfig1.plot(rates, err_max[:,0], '-bo', label='Max Error')
# sfig2.plot(rates, (180.0/np.pi)*err_max[:,1], '-bo')
# sfig1.set_ylabel('Translation [m]')
# # sfig1.legend()
# # sfig2.legend()
# fig3.legend()
# sfig2.set_xlabel('Degeneration Rate [%]')
# sfig2.set_ylabel('Heading [Deg]')
# plt.suptitle('Trajectory Error With Degeneracy')
plt.show()


