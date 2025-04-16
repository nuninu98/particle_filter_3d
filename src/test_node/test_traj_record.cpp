#include <ros/ros.h>
#include <fstream>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <gazebo_msgs/ModelStates.h>

using namespace std;
Eigen::Matrix4d init_pose = Eigen::Matrix4d::Zero();
Eigen::Matrix4d init_pose_gt = Eigen::Matrix4d::Zero();
vector<pair<double, Eigen::Matrix4d>> path, path_gt;
ros::Subscriber sub_pose, sub_gt_pose;

void poseCallback(const nav_msgs::OdometryConstPtr& localization_pose){
    double time = localization_pose->header.stamp.toSec();
    Eigen::Matrix4d se3 = Eigen::Matrix4d::Identity();
    se3(0, 3) = localization_pose->pose.pose.position.x;
    se3(1, 3) = localization_pose->pose.pose.position.y;
    se3(2, 3) = localization_pose->pose.pose.position.z;
    Eigen::Quaterniond q(localization_pose->pose.pose.orientation.w, localization_pose->pose.pose.orientation.x, 
        localization_pose->pose.pose.orientation.y, localization_pose->pose.pose.orientation.z);
    se3.block<3, 3>(0, 0) = q.toRotationMatrix();
    if(init_pose == Eigen::Matrix4d::Zero()){
        init_pose = se3;
        return;
    }
    Eigen::Matrix4d rel_pose = init_pose.inverse() * se3;
    path.push_back({time, rel_pose});
}

void gtPoseCallback(const gazebo_msgs::ModelStatesConstPtr& model_states){
    double time = ros::Time::now().toSec();
    Eigen::Matrix4d se3 = Eigen::Matrix4d::Identity();
    int id = find(model_states->name.begin(), model_states->name.end(), "jackal") - model_states->name.begin();
    se3(0, 3) = model_states->pose[id].position.x;
    se3(1, 3) = model_states->pose[id].position.y;
    se3(2, 3) = model_states->pose[id].position.z;
    Eigen::Quaterniond q(model_states->pose[id].orientation.w, model_states->pose[id].orientation.x, 
        model_states->pose[id].orientation.y, model_states->pose[id].orientation.z);
    se3.block<3, 3>(0, 0) = q.toRotationMatrix();
    if(init_pose_gt == Eigen::Matrix4d::Zero()){
        init_pose_gt = se3;
        return;
    }
    Eigen::Matrix4d rel_pose = init_pose_gt.inverse() * se3;
    if(path_gt.empty()){
        path_gt.push_back({time, rel_pose});
    }
    else if(time - path_gt.back().first > 1.0e-7){
        path_gt.push_back({time, rel_pose});
    }
    
}


void saveTrajectory(const string& file_path, const vector<pair<double, Eigen::Matrix4d>>& traj){
    if(traj.empty()){
        return;
    }
    ofstream file(file_path);
    for(int i = 0; i < traj.size(); ++i){
        double time = traj[i].first;
        double x = traj[i].second(0, 3);
        double y = traj[i].second(1, 3);
        double z = traj[i].second(2, 3);
        Eigen::Quaterniond q(traj[i].second.block<3, 3>(0, 0));
        file << to_string(time)<<" "<<x<<" "<<y<<" "<<z<<" "<<q.w()<<" "<<q.x()<<" "<<q.y()<<" "<<q.z()<<endl;
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "test_traj_record");
    ros::NodeHandle nh;
    sub_pose = nh.subscribe("localization_pose", 100, poseCallback);
    sub_gt_pose = nh.subscribe("gazebo/model_states", 100, gtPoseCallback);
    while(ros::ok()){
        ros::spinOnce();
    }
    string folder_path = "/home/nuninu98/test_ws/src/particle_filter_3d/trajectory/";
    string file_name = "pf_imu_degen_50.txt";
    saveTrajectory(folder_path + file_name, path);

    string file_name_gt = "gt.txt";
    saveTrajectory(folder_path + file_name_gt, path_gt);

}