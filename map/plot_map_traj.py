import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
from scipy.spatial.transform import Slerp
import open3d as o3d

folder = "warehouse"
gt = np.loadtxt(folder + "/gt.txt")
all_poses = np.loadtxt(folder + "/all_poses.txt")
# loops = np.loadtxt(folder + "/loop_indices.txt")
pcd = o3d.io.read_point_cloud(folder+"/GlobalMap.pcd")
pcd_points = np.asarray(pcd.points)

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

def plotLoopClosure(loop_closures, figure):
    length = np.shape(loop_closures)[0]
    alpha = 0.8
    for i in range(length):
        if i == 0:
            figure.plot([loop_closures[i, 0], loop_closures[i, 3]], [loop_closures[i, 1], loop_closures[i, 4]], '--b', label='Loop Closures', alpha=alpha)
        else:
            figure.plot([loop_closures[i, 0], loop_closures[i, 3]], [loop_closures[i, 1], loop_closures[i, 4]], '--b', alpha=alpha)
        figure.plot(loop_closures[i, 0], loop_closures[i, 1], 'bo',alpha=alpha, markersize=1)
        figure.plot(loop_closures[i, 3], loop_closures[i, 4], 'bo', alpha=alpha, markersize=1)
print("PF+IMU RMSE: ", rmse(all_poses, gt))
print("PF+IMU Max Err: ", max_err(all_poses, gt))

fig0 = plt.figure(0)
fig0_ax = fig0.add_subplot(1, 1, 1)
plotTrajectory(gt, fig0_ax, 'Ground Truth', '--k')
plotTrajectory(all_poses, fig0_ax, 'NV-LIOM', '-r')
# plotLoopClosure(loops, fig0_ax)
fig0_ax.scatter(pcd_points[:,0], pcd_points[:,1], s=1, color='c', alpha=0.2)
fig0_ax.set_xlim(-20, 20)
plt.title('SLAM Trajectory')
plt.legend()
plt.xlabel('x [m]')
plt.ylabel('y [m]')


fig1 = plt.figure(1)
fig1_ax = fig1.add_subplot(1, 1, 1, projection='3d')
plotTrajectory(gt, fig1_ax, 'Ground Truth', '--k', True)
plotTrajectory(all_poses, fig1_ax, 'NV-LIOM', '-r', True)
fig1_ax.set_zlim(-5, 5)
fig1_ax.set_title('SLAM Trajectory')
fig1_ax.legend(loc='upper left')

plt.show()


