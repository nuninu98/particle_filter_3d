#include <particle_filter_3d/api_class/gicp_localization.h>

GicpLocalization::GicpLocalization(): queue_(), pnh_("~"), spinner_(0, &queue_), kill_flag_(false),pose_(Eigen::Matrix4d::Identity()), tf_lidar_robot_(Eigen::Matrix4d::Identity()){
    nh_.setCallbackQueue(&queue_);
    string lidar_topic = "";
    pnh_.param<string>("lidar_topic", lidar_topic, "/velodyne_points");
    sub_lidar_ = nh_.subscribe(lidar_topic, 1, &GicpLocalization::lidarCallback, this);

    string odom_topic = "";
    pnh_.param<string>("odom_topic", odom_topic, "/odometry/filtered");
    sub_odom_ = nh_.subscribe(odom_topic, 1, &GicpLocalization::odomCallback, this);

    sub_initpose_ = nh_.subscribe("initialpose", 1, &GicpLocalization::initialPoseCallback, this);

    string map_file_path = "";
    pnh_.param<string>("map_file", map_file_path, "empty");
    
    pub_map_ = nh_.advertise<sensor_msgs::PointCloud2>("gicp_map", 1);
    pub_query_ = nh_.advertise<sensor_msgs::PointCloud2>("gicp_query", 1);
    pub_target_ = nh_.advertise<sensor_msgs::PointCloud2>("gicp_target", 1);
    voxel_.setLeafSize(0.3, 0.3, 0.3);

    pcl::PointCloud<pcl::PointXYZI>::Ptr raw_map(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::io::loadPCDFile(map_file_path, *raw_map);
    voxel_.setInputCloud(raw_map);
    voxel_.filter(map_);
    submap_thread_ = thread(&GicpLocalization::submapFlagCallback, this);
    submap_thread_.detach();
    submap_radius_ = 30.0;
    last_odom_.header.stamp = ros::Time(0.0);
    submap_.reset(new pcl::PointCloud<pcl::PointXYZI>(map_));
    addSubmapFlag(pose_);

    tf_lidar_robot_(2, 3) = 0.4;

    gicp_.setMaxCorrespondenceDistance(1.0);
    gicp_.setTransformationEpsilon(0.01);
    gicp_.setMaximumIterations(50);
    gicp_.setInputTarget(submap_);
    
    spinner_.start();
}

GicpLocalization::~GicpLocalization(){
    spinner_.stop();
    nh_.shutdown();

    kill_flag_ = true;
    submap_cv_.notify_all();
    waitForKill();
}

void GicpLocalization::odomCallback(const nav_msgs::OdometryConstPtr& odom){
    if(submap_->empty()){
        return;
    }
    if(last_odom_.header.stamp == ros::Time(0.0)){
        last_odom_ = *odom;
        return;
    }
    Eigen::Matrix4d prev_se3 = odom2SE3(last_odom_);
    Eigen::Matrix4d curr_se3 = odom2SE3(*odom);
    Eigen::Matrix4d diff = prev_se3.inverse() * curr_se3;
    pose_mtx_.lock();
    pose_ = pose_ * diff;
    pose_mtx_.unlock();
    last_odom_ = *odom;
    
    Eigen::Matrix4d update_diff = last_submap_update_pose_.inverse() * curr_se3;
    double dist = sqrt(pow(update_diff(0, 3), 2) + pow(update_diff(1, 3), 2) + pow(update_diff(2, 3), 2));
    if(dist > 0.3){
        addSubmapFlag(pose_);
    }
    pubPoseTF();
}

Eigen::Matrix4d GicpLocalization::odom2SE3(const nav_msgs::Odometry& odom){
    Eigen::Matrix4d se3 = Eigen::Matrix4d::Identity();
    se3(0, 3) = odom.pose.pose.position.x;
    se3(1, 3) = odom.pose.pose.position.y;
    se3(2, 3) = odom.pose.pose.position.z;

    Eigen::Quaterniond q(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, 
    odom.pose.pose.orientation.y, odom.pose.pose.orientation.z);
    se3.block<3, 3>(0, 0) = q.toRotationMatrix();
    return se3;
}

void GicpLocalization::pubPoseTF(){
    geometry_msgs::TransformStamped map2odom_tf;
    map2odom_tf.header.stamp = ros::Time::now();
    map2odom_tf.header.frame_id = "map";
    map2odom_tf.child_frame_id = "odom";
    Eigen::Matrix4d odom_se3 = odom2SE3(last_odom_);
    Eigen::Matrix4d map2odom_se3 = pose_ * odom_se3.inverse();

    map2odom_tf.transform.translation.x = map2odom_se3(0, 3);
    map2odom_tf.transform.translation.y = map2odom_se3(1, 3);
    map2odom_tf.transform.translation.z = map2odom_se3(2, 3);

    Eigen::Quaterniond q(map2odom_se3.block<3, 3>(0, 0));
    map2odom_tf.transform.rotation.w = q.w();
    map2odom_tf.transform.rotation.x = q.x();
    map2odom_tf.transform.rotation.y = q.y();
    map2odom_tf.transform.rotation.z = q.z();

    broadcaster_.sendTransform(map2odom_tf);
}

void GicpLocalization::lidarCallback(const sensor_msgs::PointCloud2ConstPtr& cloud){
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr submap_copy(new pcl::PointCloud<pcl::PointXYZI>());
    *submap_copy = *submap_; 

    pcl::PointCloud<pcl::PointXYZI> raw_cloud;
    pcl::fromROSMsg(*cloud, raw_cloud);
    pcl::PointCloud<pcl::PointXYZI>::Ptr tf_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(raw_cloud, *tf_cloud, tf_lidar_robot_);
    pcl::PointCloud<pcl::PointXYZI>::Ptr query(new pcl::PointCloud<pcl::PointXYZI>());
    voxel_.setInputCloud(tf_cloud);
    voxel_.filter(*query);
    gicp_.setInputSource(query);
    unique_lock<mutex> lock(submap_mtx_);
    
    if(submap_->empty()){
        return;
    }
    ros::Time tic = ros::Time::now();
    pcl::PointCloud<pcl::PointXYZI> query_tf;
    gicp_.align(query_tf, pose_.cast<float>());
    pose_mtx_.lock();
    pose_ = gicp_.getFinalTransformation().cast<double>();
    pose_mtx_.unlock();
    //cout<<"TIME: "<<(ros::Time::now() - tic).toSec()<<endl;
}

void GicpLocalization::addSubmapFlag(const Eigen::Matrix4d& pose){
    unique_lock<mutex> lock(submap_mtx_);
    submap_flag_queue_.push(pose);
    

    
    submap_cv_.notify_all();
}

void GicpLocalization::submapFlagCallback(){
    while(true){
        unique_lock<mutex> lock(submap_mtx_);
        submap_cv_.wait(lock, [this]{return !submap_flag_queue_.empty() || kill_flag_;});
        if(kill_flag_){
            kill_done_ = true;
            kill_cv_.notify_all();
            return;
        }
        while(!submap_flag_queue_.empty()){
            Eigen::Matrix4d center = submap_flag_queue_.front();
            submap_->clear();
            for(const auto& elem : map_){
                double dist = sqrt(pow(center(0, 3) - elem.x, 2)+pow(center(1, 3) - elem.y, 2)+pow(center(2, 3) - elem.z, 2));
                if(dist < submap_radius_){
                    submap_->push_back(elem);
                } 
            }
            
            last_submap_update_pose_ = center;
            sensor_msgs::PointCloud2 submap_ros;
            pcl::toROSMsg(*submap_, submap_ros);
            submap_ros.header.frame_id = "map";
            submap_ros.header.stamp = ros::Time::now();
            pub_target_.publish(submap_ros);
            submap_flag_queue_.pop();
            gicp_.setInputTarget(submap_);
        }

    }
}

void GicpLocalization::waitForKill(){
    while(true){
        unique_lock<mutex> lock(kill_mtx_);
        kill_cv_.wait(lock, [this]{return kill_done_;});
        if(kill_done_){
            return;
        }
    }
}

void GicpLocalization::initialize(const Eigen::Matrix4d& pose){
    addSubmapFlag(pose);
    pose_mtx_.lock();
    pose_ = pose;
    pose_mtx_.unlock();
    sensor_msgs::PointCloud2 map_ros;
    pcl::toROSMsg(map_, map_ros);
    map_ros.header.frame_id = "map";
    map_ros.header.stamp = ros::Time::now();
    pub_map_.publish(map_ros);
}

void GicpLocalization::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_2d){
    Eigen::Matrix4d pose_se3 = Eigen::Matrix4d::Identity();
    pose_se3(0, 3) = pose_2d->pose.pose.position.x;
    pose_se3(1, 3) = pose_2d->pose.pose.position.y;
    pose_se3(2, 3) = 0.0;

    Eigen::Quaterniond q(pose_2d->pose.pose.orientation.w, pose_2d->pose.pose.orientation.x, pose_2d->pose.pose.orientation.y, pose_2d->pose.pose.orientation.z);
    pose_se3.block<3, 3>(0, 0) = q.toRotationMatrix();
    initialize(pose_se3);
}