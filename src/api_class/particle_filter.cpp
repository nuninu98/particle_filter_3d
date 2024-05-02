#include <particle_filter_3d/api_class/particle_filter.h>

ParticleFilter::ParticleFilter(): queue_(), spinner_(0, &queue_), pnh_("~"), N_particles_(1000), pose_(Eigen::Matrix4d::Identity()),
tf_lidar2_robot_(Eigen::Matrix4d::Identity()), listener_(buffer_), kill_flag_(false), kill_done_(false){
    nh_.setCallbackQueue(&queue_);
    particles_.resize(N_particles_, Particle());
    grid_submap_.initialize(100.0, 100.0, 40.0, 0.1);
    sub_lidar_ = nh_.subscribe("lidar_topic", 1, &ParticleFilter::lidarCallback, this);
    sub_odometry_ = nh_.subscribe("odom_topic", 1, &ParticleFilter::odomCallback, this);
    submap_thread_ = thread(&ParticleFilter::submapFlagCallback, this);
    submap_thread_.detach();
    //buffer_.lookupTransform("base_link", "velodyne", ros::Time(0));
    spinner_.start();
}

ParticleFilter::~ParticleFilter(){  
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
        
    }
    
    
}

void ParticleFilter::lidarCallback(const sensor_msgs::PointCloud2ConstPtr& cloud){
    unique_lock<mutex> lock(mtx_);
    //========================Particle Estimation========================
    pcl::PointCloud<pcl::PointXYZI> raw_lidar, lidar_robot;
    pcl::fromROSMsg(*cloud, raw_lidar);
    pcl::transformPointCloud(raw_lidar, lidar_robot, tf_lidar2_robot_);

    //===================================================================
}

void ParticleFilter::odomCallback(const nav_msgs::OdometryConstPtr& odom){
    unique_lock<mutex> lock(mtx_);
    //=============================Particle Prediction=====================
    
    //=====================================================================
}

Eigen::VectorXd ParticleFilter::toPose6d(const Eigen::Matrix4d& se3){
    Eigen::VectorXd pose6d = Eigen::VectorXd::Zero(6);
    pose6d(0) = se3(0, 3);
    pose6d(1) = se3(1, 3);
    pose6d(2) = se3(2, 3);
    
    auto rod_angle = Eigen::AngleAxisd(se3.block<3, 3>(0, 0));
    pose6d.block<3, 1>(0, 3) = rod_angle.angle() * rod_angle.axis();
    return pose6d;
}

Eigen::Matrix4d ParticleFilter::toSE3(const Eigen::VectorXd& pose6d){
    Eigen::Matrix4d se3 = Eigen::Matrix4d::Identity();
    se3(0, 3) = pose6d(0);
    se3(1, 3) = pose6d(1);
    se3(2, 3) = pose6d(2);

    Eigen::Vector3d angle_3d = pose6d.block<3, 1>(0, 3);
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
    normal_distribution<double> nd_x(0.0, 0.5);
    normal_distribution<double> nd_y(0.0, 0.5);
    normal_distribution<double> nd_z(0.0, 0.5);
    normal_distribution<double> nd_rx(0.0, 0.5);
    normal_distribution<double> nd_ry(0.0, 0.5);
    normal_distribution<double> nd_rz(0.0, 0.5);
    
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
    calculatePose();
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
}

