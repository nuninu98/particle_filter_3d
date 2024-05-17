#include <particle_filter_3d/api_class/particle_filter.h>

ParticleFilter::ParticleFilter(): queue_(), spinner_(0, &queue_), pnh_("~"), N_particles_(1000), pose_(Eigen::Matrix4d::Identity()),
tf_lidar2_robot_(Eigen::Matrix4d::Identity()), listener_(buffer_), kill_flag_(false), kill_done_(false){
    nh_.setCallbackQueue(&queue_);
    particles_.resize(N_particles_, Particle());
    grid_submap_.initialize(100.0, 100.0, 40.0, 0.1);
    
    string lidar_topic = "";
    pnh_.param<string>("lidar_topic", lidar_topic, "velodyne_points");
    sub_lidar_ = nh_.subscribe(lidar_topic, 1, &ParticleFilter::lidarCallback, this);

    string odom_topic = "";
    pnh_.param<string>("odom_topic", odom_topic, "odometry/filtered");
    sub_odometry_ = nh_.subscribe(odom_topic, 1, &ParticleFilter::odomCallback, this);
    
    string map_file_path = "";
    pnh_.param<string>("map_file", map_file_path, "odometry/filtered");
    pcl::io::loadPCDFile(map_file_path, pcd_map_);

    pub_map_ = nh_.advertise<sensor_msgs::PointCloud2>("pcd_map", 1);
    pub_particles_ = nh_.advertise<visualization_msgs::MarkerArray>("particles", 1);
    
    submap_thread_ = thread(&ParticleFilter::submapFlagCallback, this);
    submap_thread_.detach();
    last_odom_.header.stamp = ros::Time(0.0);
    tf_lidar2_robot_(2, 3) = 0.4; 
    // geometry_msgs::TransformStamped robot2Lidar;
    // bool got_tf = true;
    // while(true){
    //     try
    //     {
    //         robot2Lidar = buffer_.lookupTransform("base_link", "velodyne", ros::Time(0));
    //     }
    //     catch(const std::exception& e)
    //     {
    //         got_tf = false;
    //         std::cerr << e.what() << '\n';
    //     }
    //     if(got_tf){
    //         cout<<"GOT: "<<robot2Lidar.transform.translation.z<<endl;

    //         break;
    //     }
    // }
    
    
    spinner_.start();
}

ParticleFilter::~ParticleFilter(){
    addSubmapFlag(Eigen::Matrix4d::Identity());  
    spinner_.stop();
    kill_flag_ = true;
    submap_cv_.notify_all();
    waitForKill();
}

void ParticleFilter::waitForKill(){
    while(true){
        unique_lock<mutex> lock(kill_mtx_);
        kill_cv_.wait(lock, [this]{return kill_done_;});
        if(kill_done_){
            return;
        }
    }
}

void ParticleFilter::submapFlagCallback(){
    while(true){
        unique_lock<mutex> lock(submap_mtx_);
        submap_cv_.wait(lock, [this]{return !submap_flag_queue_.empty() || kill_flag_;});
        if(kill_flag_){
            kill_done_ = true;
            kill_cv_.notify_all();
            return;
        }
        while(!submap_flag_queue_.empty()){
            grid_submap_.generateSubmap(pcd_map_, submap_flag_queue_.front());
            submap_flag_queue_.pop();
        }

    }
    
}

void ParticleFilter::lidarCallback(const sensor_msgs::PointCloud2ConstPtr& cloud){
    unique_lock<mutex> lock(mtx_);
    if(pcd_map_.empty()){
        return;
    }
    //========================Particle Estimation========================
    pcl::PointCloud<pcl::PointXYZI> raw_lidar, lidar_robot;
    pcl::fromROSMsg(*cloud, raw_lidar);
    pcl::transformPointCloud(raw_lidar, lidar_robot, tf_lidar2_robot_);

    //===================================================================
}

void ParticleFilter::odomCallback(const nav_msgs::OdometryConstPtr& odom){
    unique_lock<mutex> lock(mtx_);
    if(pcd_map_.empty()){
        return;
    }
    //=============================Particle Prediction=====================
    if(last_odom_.header.stamp == ros::Time(0.0)){
        last_odom_ = *odom;
        
        return;
    }
    Eigen::Vector3d linvel(odom->twist.twist.linear.x, odom->twist.twist.linear.y, odom->twist.twist.linear.z);
    Eigen::Vector3d angvel(odom->twist.twist.angular.x, odom->twist.twist.angular.y, odom->twist.twist.angular.z);
    if(linvel.norm() < 0.01 && angvel.norm() < 0.01){
        publishParticle();
        last_odom_ = *odom;
        return;
    }
    ros::Time begin = ros::Time::now();
    double dt = (odom->header.stamp - last_odom_.header.stamp).toSec();
    random_device rd;
    mt19937 gen(rd());
    normal_distribution<double> nd_vx(odom->twist.twist.linear.x, 0.1);
    normal_distribution<double> nd_vy(odom->twist.twist.linear.y, 0.05);
    normal_distribution<double> nd_vz(odom->twist.twist.linear.z, 0.05);
    normal_distribution<double> nd_wx(odom->twist.twist.angular.x, 0.05);
    normal_distribution<double> nd_wy(odom->twist.twist.angular.y, 0.05);
    normal_distribution<double> nd_wz(odom->twist.twist.angular.z, 0.05);
    for(auto& p : particles_){
        Eigen::VectorXd dp = Eigen::VectorXd::Zero(6);
        dp(0) = nd_vx(gen) * dt;
        dp(1) = nd_vy(gen) * dt;
        dp(2) = nd_vz(gen) * dt;
        dp(3) = nd_wx(gen) * dt;
        dp(4) = nd_wy(gen) * dt;
        dp(5) = nd_wz(gen) * dt;
        p.predict(toSE3(dp));
    }
    publishParticle();
    calculatePose();
    last_odom_ = *odom;
    //=====================================================================
}

Eigen::VectorXd ParticleFilter::toPose6d(const Eigen::Matrix4d& se3){
    Eigen::VectorXd pose6d = Eigen::VectorXd::Zero(6);
    pose6d(0) = se3(0, 3);
    pose6d(1) = se3(1, 3);
    pose6d(2) = se3(2, 3);
    
    auto rod_angle = Eigen::AngleAxisd(se3.block<3, 3>(0, 0));
    Eigen::Vector3d angle_vec = rod_angle.angle() * rod_angle.axis();
    pose6d(3) = angle_vec(0);
    pose6d(4) = angle_vec(1);
    pose6d(5) = angle_vec(2);
    return pose6d;
}

Eigen::Matrix4d ParticleFilter::toSE3(const Eigen::VectorXd& pose6d){
    Eigen::Matrix4d se3 = Eigen::Matrix4d::Identity();
    se3(0, 3) = pose6d(0);
    se3(1, 3) = pose6d(1);
    se3(2, 3) = pose6d(2);

    Eigen::Vector3d angle_3d;
    angle_3d(0) = pose6d(3);
    angle_3d(1) = pose6d(4);
    angle_3d(2) = pose6d(5);
    double angle = angle_3d.norm();
    Eigen::Vector3d angle_axis = angle_3d.normalized();
    Eigen::AngleAxisd rod_angle(angle, angle_axis);
    se3.block<3, 3>(0, 0) = rod_angle.toRotationMatrix();
    return se3;      
}

void ParticleFilter::initialize(const Eigen::Matrix4d& pose){
    unique_lock<mutex> lock(mtx_);
    particles_.clear();
    Eigen::VectorXd pose_center = toPose6d(pose);
    
    random_device rd;
    mt19937 gen(rd());
    normal_distribution<double> nd_x(0.0, 0.3);
    normal_distribution<double> nd_y(0.0, 0.3);
    normal_distribution<double> nd_z(0.0, 0.3);
    normal_distribution<double> nd_rx(0.0, 0.1);
    normal_distribution<double> nd_ry(0.0, 0.1);
    normal_distribution<double> nd_rz(0.0, 0.1);
    
    for(size_t i = 0; i < N_particles_; ++i){
        Eigen::VectorXd rand_vec = Eigen::VectorXd::Zero(6);
        rand_vec(0) = nd_x(gen);
        rand_vec(1) = nd_y(gen);
        rand_vec(2) = nd_z(gen);
        rand_vec(3) = nd_rx(gen);
        rand_vec(4) = nd_ry(gen);
        rand_vec(5) = nd_rz(gen);
        Eigen::VectorXd pose6d = rand_vec + pose_center;
        Particle p(toSE3(pose6d));
        particles_.push_back(p);
    }
    publishMap();
    calculatePose();
    addSubmapFlag(pose_);
}

void ParticleFilter::calculatePose(){
    Eigen::Vector3d pose_avg = Eigen::Vector3d::Zero();
    Eigen::MatrixXd Q(4, N_particles_);
    for(int i = 0; i < N_particles_; ++i){
        Eigen::Matrix4d pose = particles_[i].getPose();
        Eigen::Quaterniond q(pose.block<3, 3>(0, 0));
        Eigen::Vector4d q_vec(q.x(), q.y(), q.z(), q.w());
        Q.col(i) = q_vec;
    }
    Eigen::MatrixXd QQT = Q * Q.transpose();
    Eigen::JacobiSVD<Eigen::Matrix4d> svd(QQT, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::MatrixXd V = svd.matrixV();
    Eigen::Quaterniond quat_avg(V(3, 0), V(0, 0), V(1, 0), V(2, 0));
    pose_.block<3, 3>(0, 0) = quat_avg.toRotationMatrix();
    pose_(0, 3) = pose_avg(0);
    pose_(1, 3) = pose_avg(1);
    pose_(2, 3) = pose_avg(2);

    //================Send TF=================
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = ros::Time::now();
    tf_msg.header.frame_id = "map";
    tf_msg.child_frame_id = "odom";
    Eigen::Matrix4d odom_se3 = Eigen::Matrix4d::Identity();
    odom_se3(0, 3) = last_odom_.pose.pose.position.x;
    odom_se3(1, 3) = last_odom_.pose.pose.position.y;
    odom_se3(2, 3) = last_odom_.pose.pose.position.z;
    Eigen::Quaterniond odom_q(last_odom_.pose.pose.orientation.w, last_odom_.pose.pose.orientation.x, last_odom_.pose.pose.orientation.y, last_odom_.pose.pose.orientation.z);
    odom_se3.block<3, 3>(0, 0) = odom_q.toRotationMatrix();
    Eigen::Matrix4d map2odom = pose_ * odom_se3.inverse();
    tf_msg.transform.translation.x = map2odom(0, 3);
    tf_msg.transform.translation.y = map2odom(1, 3);
    tf_msg.transform.translation.z = map2odom(2, 3);
    Eigen::Quaterniond diff_q(map2odom.block<3, 3>(0, 0));
    tf_msg.transform.rotation.w = diff_q.w();
    tf_msg.transform.rotation.x = diff_q.x();
    tf_msg.transform.rotation.y = diff_q.y();
    tf_msg.transform.rotation.z = diff_q.z();
    broadcaster_.sendTransform(tf_msg);
    //================================================
}

void ParticleFilter::publishMap(){
    sensor_msgs::PointCloud2 map_ros;
    pcl::toROSMsg(pcd_map_, map_ros);
    map_ros.header.stamp = ros::Time::now();
    map_ros.header.frame_id = "map";
    pub_map_.publish(map_ros);
}

void ParticleFilter::publishParticle(){
    visualization_msgs::MarkerArray arr;
    ros::Time stamp = ros::Time::now();
    int id = 0;
    for(const auto& p : particles_){
        visualization_msgs::Marker p_marker;
        p_marker.header.stamp = stamp;
        p_marker.header.frame_id = "map";
        p_marker.id = id;
        p_marker.type = visualization_msgs::Marker::ARROW;
        p_marker.color.a = 1.0;
        p_marker.color.r = 255.0;
        p_marker.color.g = 0.0;
        p_marker.color.b = 0.0;
        p_marker.scale.x = 0.1;
        p_marker.scale.y = 0.03;
        p_marker.scale.z = 0.03;
        auto pose = p.getPose();
        p_marker.pose.position.x = pose(0, 3);
        p_marker.pose.position.y = pose(1, 3);
        p_marker.pose.position.z = pose(2, 3);
        Eigen::Quaterniond q(pose.block<3, 3>(0, 0));
        p_marker.pose.orientation.w = q.w();
        p_marker.pose.orientation.x = q.x();
        p_marker.pose.orientation.y = q.y();
        p_marker.pose.orientation.z = q.z();
        arr.markers.push_back(p_marker);
        id++;
    }
    pub_particles_.publish(arr);
}

void ParticleFilter::addSubmapFlag(const Eigen::Matrix4d& pose){
    unique_lock<mutex> lock(submap_mtx_);
    submap_flag_queue_.push(pose);
    submap_cv_.notify_all();
}

