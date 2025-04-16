#include <particle_filter_3d/api_class/particle_filter.h>
namespace PARTICLE_FILTER_3D{
    ParticleFilter::ParticleFilter(): queue_(), spinner_(0, &queue_), pnh_("~"), N_particles_(1000), pose_(Eigen::Matrix4d::Identity()),
    Trl_(Eigen::Matrix4d::Identity()), listener_(buffer_), kill_flag_(false), kill_done_(false), Trc_(Eigen::Matrix4d::Identity()){
        nh_.setCallbackQueue(&queue_);
        particles_.resize(N_particles_, Particle());
        grid_submap_.initialize(60.0, 60.0, 30.0, 0.05);

        int particles_num;
        
        pnh_.param<int>("num_particles", particles_num, 1000);
        N_particles_ = particles_num;

        string lidar_topic = "";
        pnh_.param<string>("lidar_topic", lidar_topic, "velodyne_points");
        sub_lidar_ = nh_.subscribe(lidar_topic, 1, &ParticleFilter::lidarCallback, this, ros::TransportHints().tcpNoDelay());
        
        string map_folder = "";
        pnh_.param<string>("map_folder", map_folder, "");
        string map_file = "";
        pnh_.param<string>("map_file", map_file, "");
        pcl::io::loadPCDFile(map_folder + "/"+map_file, pcd_map_);

        vector<string> objects;
        pnh_.param<vector<string>>("objects", objects, vector<string>());
        
        if(!loadObjectMap(map_folder, objects)){
            ROS_ERROR_STREAM("Failed to load objects");
        }

        pnh_.param<bool>("publish_odom_tf", pub_odom_tf_, false);    

        pub_map_ = nh_.advertise<sensor_msgs::PointCloud2>("pcd_map", 1);
        pub_particles_ = nh_.advertise<visualization_msgs::MarkerArray>("particles", 1);
        
        submap_thread_ = thread(&ParticleFilter::submapFlagCallback, this);
        submap_thread_.detach();
        last_odom_.header.stamp = ros::Time(0.0);
        last_tf_stamp_ = ros::Time(0.0);
        
        sub_initpose_ = nh_.subscribe("initialpose", 1, &ParticleFilter::initialPoseCallback, this);

        voxel_.setLeafSize(0.3, 0.3, 0.3);
        initialize(Eigen::Matrix4d::Identity());
        
        pub_pose_ = nh_.advertise<nav_msgs::Odometry>("localization_pose", 1);
        pub_object_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("object_cloud", 1);
                
        bool enable_preintegration = false;
        pnh_.param<bool>("imu_preintegration/enable", enable_preintegration, false); 
        if(enable_preintegration){
            ip_ = new IMUPreintegration();
            sub_odometry_ = nh_.subscribe("imu_odom", 1, &ParticleFilter::odomCallback, this, ros::TransportHints().tcpNoDelay());
        }
        else{
            string odom_topic = "";
            pnh_.param<string>("odom_topic", odom_topic, "odometry/filtered");
            sub_odometry_ = nh_.subscribe(odom_topic, 1, &ParticleFilter::odomCallback, this);
        }

        vector<double> lidar_R, lidar_T;
        if(pnh_.param<vector<double>>("lidar/R", lidar_R, vector<double>())){
            Trl_(0, 0) = lidar_R[0];
            Trl_(0, 1) = lidar_R[1];
            Trl_(0, 2) = lidar_R[2];
            Trl_(1, 0) = lidar_R[3];
            Trl_(1, 1) = lidar_R[4];
            Trl_(1, 2) = lidar_R[5];
            Trl_(2, 0) = lidar_R[6];
            Trl_(2, 1) = lidar_R[7];
            Trl_(2, 2) = lidar_R[8];
        }
        if(pnh_.param<vector<double>>("lidar/t", lidar_T, vector<double>())){
            Trl_(0, 3) = lidar_T[0];
            Trl_(1, 3) = lidar_T[1];
            Trl_(2, 3) = lidar_T[2];
        }


        pnh_.param<double>("alpha_v", alpha_v_, 3.0); 
        pnh_.param<double>("alpha_w", alpha_w_, 1.0); 
        
        bool enable_camera = false;
        pnh_.param<bool>("camera/enable", enable_camera, false); 
        if(enable_camera){
            sub_yolo_ = nh_.subscribe("/image_raw/yolo", 1, &ParticleFilter::yoloResultCallback, this);
            vector<double> R;
            pnh_.param<vector<double>>("camera/R", R, vector<double>());
            Trc_(0, 0) = R[0];
            Trc_(0, 1) = R[1];
            Trc_(0, 2) = R[2];
            Trc_(1, 0) = R[3];
            Trc_(1, 1) = R[4];
            Trc_(1, 2) = R[5];
            Trc_(2, 0) = R[6];
            Trc_(2, 1) = R[7];
            Trc_(2, 2) = R[8];

            vector<double> t;
            pnh_.param<vector<double>>("camera/t", t, vector<double>());
            Trc_(0, 3) = t[0];
            Trc_(1, 3) = t[1];
            Trc_(2, 3) = t[2];

            vector<double> K;
            pnh_.param<vector<double>>("camera/K", K, vector<double>());
            K_.reset(new gtsam::Cal3_S2(K[0], K[4], K[1], K[2], K[5]));
        }
        yolo_result_.header.stamp = ros::Time(0.0);

        pub_labeled_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("labeld_cloud", 1);
        omp_init_lock(&omp_lock_);
        spinner_.start();
    }

    ParticleFilter::~ParticleFilter(){
        addSubmapFlag(Eigen::Matrix4d::Identity());  
        spinner_.stop();
        kill_flag_ = true;
        submap_cv_.notify_all();
        waitForKill();
        delete ip_;
        omp_destroy_lock(&omp_lock_);
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

    void ParticleFilter::resample(pcl::PointCloud<pcl::PointXYZI>& raw_lidar){
        
        pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_robot(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::transformPointCloud(raw_lidar, *lidar_robot, Trl_);
        pcl::PointCloud<pcl::PointXYZI> lidar_robot_ds;
        voxelize(lidar_robot, lidar_robot_ds, 0.2);
                
        #pragma omp parallel for
        for(size_t i = 0; i < N_particles_; ++i){
            Eigen::Matrix4d Twr = particles_[i].getPose();
            Eigen::Matrix4d Tcw = (Twr * Trc_).inverse();
            grid_submap_.updateScore(lidar_robot_ds, particles_[i]);
        }
        //cout<<"==="<<endl;
        
        double weight_sum = 0.0;
        for(size_t i = 0; i < N_particles_; ++i){
            double w = particles_[i].getWeight();
            weight_sum += w;
        }
        
        for(size_t i = 0; i < N_particles_; ++i){
            double w = particles_[i].getWeight();
            double w_norm = w/ weight_sum;
            particles_[i].setWeight(w_norm);
        }
      
        
        vector<double> prefix_sum;
        for(size_t i = 0; i < N_particles_; ++i){
            if(i == 0){
                prefix_sum.push_back(particles_[i].getWeight());
            }
            else{
                prefix_sum.push_back(prefix_sum.back() + particles_[i].getWeight());
            }
        }
        double point = (double)rand() / (double)RAND_MAX;
        int particle_id = 0;
        double step = 1.0 / N_particles_;
        vector<Particle> new_particles;
        for(int i = 0; i < N_particles_; ++i){
            if(point >= 1.0){
                point -= 1.0;
                particle_id = 0;
            }
            while(prefix_sum[particle_id] < point){
                particle_id++;
            }
            Particle p(particles_[particle_id]);
            p.setWeight(1.0 / N_particles_);
            new_particles.push_back(p);
            point += step;
        }
        
        particles_ = new_particles;
        calculatePose();
        //====Testing======
        Particle opt_p(pose_);
        grid_submap_.updateScore(lidar_robot_ds, opt_p);
        // cout<<"WEIGHT: "<<(opt_p.getWeight() / lidar_robot_ds.size())<<endl;
        //=================
    }
    void ParticleFilter::lidarCallback(const sensor_msgs::PointCloud2ConstPtr& cloud){
        unique_lock<mutex> lock(mtx_);
        if(pcd_map_.empty()){
            return;
        }
       
        pcl::PointCloud<pcl::PointXYZI> raw_lidar;
        pcl::fromROSMsg(*cloud, raw_lidar);
        
        Eigen::Matrix4d last_submap_pose = submap_updated_pose_;
        resample(raw_lidar);
        
        
        // Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
        // Eigen::Vector3d v_avg = pose_.block<3, 1>(0, 3);
        // for(int i = 0; i < N_particles_; ++i){
        //     Eigen::Vector3d v = particles_[i].getPose().block<3, 1>(0, 3);
        //     Eigen::Vector3d diff = v - v_avg;
        //     cov += (diff * diff.transpose())/N_particles_;
        // }
        // Eigen::JacobiSVD<Eigen::Matrix3d> svd(cov, Eigen::ComputeFullU | Eigen::ComputeFullV);
        // auto eigMax = svd.singularValues();
        // cout<<"eigmax: "<<eigMax.transpose()<<endl;
        // //======Debug Semantic======
        // sensor_msgs::PointCloud2 label_ros;
        // pcl::toROSMsg(raw_lidar, label_ros);
        // label_ros.header = cloud->header;
        // pub_labeled_cloud_.publish(label_ros);
        // //===========================
        publishParticle();
        Eigen::Matrix4d from_submap_update = last_submap_pose.inverse() * pose_;
        if(from_submap_update.block<3, 1>(0, 3).norm() > 10.0){
            addSubmapFlag(pose_);
        }
        nav_msgs::Odometry odom_msg;
        odom_msg.header.frame_id = "map";
        odom_msg.header.stamp = cloud->header.stamp;
        odom_msg.pose.pose.position.x = pose_(0, 3);
        odom_msg.pose.pose.position.y = pose_(1, 3);
        odom_msg.pose.pose.position.z = pose_(2, 3);

        Eigen::Quaterniond pose_quat(pose_.block<3, 3>(0, 0));
        odom_msg.pose.pose.orientation.w = pose_quat.w();
        odom_msg.pose.pose.orientation.x = pose_quat.x();
        odom_msg.pose.pose.orientation.y = pose_quat.y();
        odom_msg.pose.pose.orientation.z = pose_quat.z();
        if(ip_init_){
            pub_pose_.publish(odom_msg);
        }
        
        if(ip_ != nullptr){
            ip_->optimWithPose(pose_, cloud->header.stamp.toSec());
        }
        
    }

    void ParticleFilter::odomCallback(const nav_msgs::OdometryConstPtr& odom){
        unique_lock<mutex> lock(mtx_);
        //=============================Particle Prediction=====================
        if(last_odom_.header.stamp == ros::Time(0.0)){
            last_odom_ = *odom;
            initOdomStamp_ = odom->header.stamp.toSec();
            return;
        }
        
        if(pcd_map_.empty()){
            last_odom_ = *odom;
            return;
        }
        ros::Time begin = ros::Time::now();
        double dt = (odom->header.stamp - last_odom_.header.stamp).toSec();
        random_device rd;
        mt19937 gen(rd());

        normal_distribution<double> nd_vx(0.0,  max(alpha_v_* abs(odom->twist.twist.linear.x), 0.3));
        normal_distribution<double> nd_vy(0.0,  max(alpha_v_* abs(odom->twist.twist.linear.y), 0.3));
        normal_distribution<double> nd_vz(0.0,  max(alpha_v_* abs(odom->twist.twist.linear.z), 0.3));
        normal_distribution<double> nd_wx(0.0,  max(alpha_w_* abs(odom->twist.twist.angular.x), 0.1));
        normal_distribution<double> nd_wy(0.0,  max(alpha_w_* abs(odom->twist.twist.angular.y), 0.1));
        normal_distribution<double> nd_wz(0.0,  max(alpha_w_* abs(odom->twist.twist.angular.z), 0.1));
        gtsam::Pose3 P_odomprev(gtsam::Rot3::Quaternion(last_odom_.pose.pose.orientation.w, last_odom_.pose.pose.orientation.x, last_odom_.pose.pose.orientation.y, last_odom_.pose.pose.orientation.z), 
                                gtsam::Point3(last_odom_.pose.pose.position.x, last_odom_.pose.pose.position.y, last_odom_.pose.pose.position.z));
        
        gtsam::Pose3 P_odomcurr(gtsam::Rot3::Quaternion(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z), 
                                gtsam::Point3(odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z));
        gtsam::Pose3 dP = P_odomprev.inverse() * P_odomcurr;
        for(auto& p : particles_){
            //Eigen::VectorXd dp = Eigen::VectorXd::Zero(6);
            // dp(0) = nd_vx(gen) * dt;
            // dp(1) = nd_vy(gen) * dt;
            // dp(2) = nd_vz(gen) * dt;
            // dp(3) = nd_wx(gen) * dt;
            // dp(4) = nd_wy(gen) * dt;
            // dp(5) = nd_wz(gen) * dt;
            Eigen::VectorXd dp = Eigen::VectorXd::Zero(6);
            if(odom->header.stamp.toSec() > initOdomStamp_ + 5.5){
                ROS_INFO_ONCE("INIT!");
                ip_init_ = true;
                dp = toPose6d(dP.matrix());
            }
            dp(0) += nd_vx(gen) * dt;
            dp(1) += nd_vy(gen) * dt;
            dp(2) += nd_vz(gen) * dt;
            dp(3) += nd_wx(gen) * dt;
            dp(4) += nd_wy(gen) * dt;
            dp(5) += nd_wz(gen) * dt;
            p.predict(toSE3(dp));
        
        }
        //publishParticle();
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
        normal_distribution<double> nd_x(0.0, 0.1);
        normal_distribution<double> nd_y(0.0, 0.1);
        normal_distribution<double> nd_z(0.0, 0.2);
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
        if(ip_ != nullptr){
            ip_->reset();
            last_odom_.header.stamp = ros::Time(0.0);
            ip_init_ = false;
        }
    }

    void ParticleFilter::calculatePose(){
        Eigen::Vector3d pose_avg = Eigen::Vector3d::Zero();
        Eigen::MatrixXd Q(4, N_particles_);
        for(int i = 0; i < N_particles_; ++i){
            double weight = particles_[i].getWeight();
            Eigen::Matrix4d pose = particles_[i].getPose();
            Eigen::Quaterniond q(pose.block<3, 3>(0, 0));
            Eigen::Vector4d q_vec(q.x(), q.y(), q.z(), q.w());
            Q.col(i) = weight * q_vec;
            pose_avg = pose_avg + (pose.block<3, 1>(0, 3) / N_particles_); 
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
        if(last_odom_.header.stamp == ros::Time(0.0)){
            return;
        }
        ros::Time stamp = ros::Time::now();
        if((stamp - last_tf_stamp_).toSec() > 0.001){
            vector<geometry_msgs::TransformStamped> tfs;
            geometry_msgs::TransformStamped tf_msg;
            tf_msg.header.stamp = stamp;
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
            tfs.push_back(tf_msg);
            if(pub_odom_tf_){
                geometry_msgs::TransformStamped tf_odom;
                tf_odom.header.stamp = stamp;
                tf_odom.header.frame_id = "odom";
                tf_odom.child_frame_id = "base_link";
                tf_odom.transform.translation.x = last_odom_.pose.pose.position.x;
                tf_odom.transform.translation.y = last_odom_.pose.pose.position.y; 
                tf_odom.transform.translation.z = last_odom_.pose.pose.position.z; 
                tf_odom.transform.rotation = last_odom_.pose.pose.orientation;
                tfs.push_back(tf_odom);
            }
            broadcaster_.sendTransform(tfs);
            last_tf_stamp_ = stamp;
        }
        
        //================================================
    }

    void ParticleFilter::publishMap(){
        sensor_msgs::PointCloud2 map_ros;
        pcl::toROSMsg(pcd_map_, map_ros);
        map_ros.header.stamp = ros::Time::now();
        map_ros.header.frame_id = "map";
        pub_map_.publish(map_ros);

        pcl::PointCloud<pcl::PointXYZI> obj_cloud;
        int id = 0;
        for(auto& nc : object_cloud_){
            for(auto& pt : nc.second.points){
                pt.intensity = 100.0 * (id+1);
            }
            obj_cloud = obj_cloud + nc.second;
            ++id;
        }
        sensor_msgs::PointCloud2 obj_ros;
        pcl::toROSMsg(obj_cloud, obj_ros);
        obj_ros.header.frame_id = "map";
        obj_ros.header.stamp = ros::Time::now();
        pub_object_cloud_.publish(obj_ros);

    }

    void ParticleFilter::publishParticle(){
        visualization_msgs::MarkerArray arr;
        ros::Time stamp = ros::Time::now();
        int id = 0;
        for(const auto& p : particles_){
            if(id % 20 != 0){
                id++;
                continue;
            }
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
            p_marker.lifetime = ros::Duration(0.12);
            arr.markers.push_back(p_marker);
            id++;
        }
        pub_particles_.publish(arr);
    }

    void ParticleFilter::addSubmapFlag(const Eigen::Matrix4d& pose){
        unique_lock<mutex> lock(submap_mtx_);
        submap_flag_queue_.push(pose);
        submap_updated_pose_ = pose;
        submap_cv_.notify_all();
    }

    void ParticleFilter::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_2d){
        Eigen::Matrix4d pose_se3 = Eigen::Matrix4d::Identity();
        pose_se3(0, 3) = pose_2d->pose.pose.position.x;
        pose_se3(1, 3) = pose_2d->pose.pose.position.y;
        pose_se3(2, 3) = 0.0;

        Eigen::Quaterniond q(pose_2d->pose.pose.orientation.w, pose_2d->pose.pose.orientation.x, pose_2d->pose.pose.orientation.y, pose_2d->pose.pose.orientation.z);
        pose_se3.block<3, 3>(0, 0) = q.toRotationMatrix();
        initialize(pose_se3);
    }

    bool ParticleFilter::loadObjectMap(const string& folder, const vector<string>& objects){ // name x y z qw qx qy qz rx ry tz
        object_cloud_.clear();
        name_ids_.clear();
        for(size_t i = 0; i < objects.size(); ++i){
            pcl::PointCloud<pcl::PointXYZI> obj_cloud;
            pcl::io::loadPCDFile(folder + "/"+objects[i]+".pcd", obj_cloud);
            if(name_ids_.find(objects[i]) == name_ids_.end()){
                name_ids_.insert({objects[i], i+1}); // to set 0 as empty cell
            }
            object_cloud_.insert({objects[i], obj_cloud});
        }
        
        return true;
        // ifstream object_file(path);
        // if(!object_file.good()){
        //     return false;
        // }
        // for(string line; getline(object_file, line);){
        //     vector<string> splitted;
        //     string str_left = line;
        //     while(!str_left.empty()){
        //         int id = str_left.find(" ");
        //         if(id == string::npos){
        //             splitted.push_back(str_left);
        //             break;
        //         }
        //         string split = str_left.substr(0, id);
        //         str_left = str_left.substr(id + 1);
        //         splitted.push_back(split);
        //     }
        //     if(splitted.size() != 11){
        //         return false;
        //     }
        //     string name = splitted[0];
            
        //     gtsam::Vector3 radii;
        //     Eigen::Matrix4d se3 = Eigen::Matrix4d::Identity();
        //     se3(0, 3) = stod(splitted[1]);
        //     se3(1, 3) = stod(splitted[2]);
        //     se3(2, 3) = stod(splitted[3]);
        //     double qw = stod(splitted[4]);
        //     double qx = stod(splitted[5]);
        //     double qy = stod(splitted[6]);
        //     double qz = stod(splitted[7]);
        //     radii(0) = stod(splitted[8]);
        //     radii(1) = stod(splitted[9]);
        //     radii(2) = stod(splitted[10]);
        //     Eigen::Quaterniond quat(qw, qx, qy, qz);
        //     se3.block<3, 3>(0, 0) = quat.toRotationMatrix();
            
        //     gtsam::Pose3 pose(se3);
        //     gtsam_quadrics::ConstrainedDualQuadric Q(pose, radii);
        //     shared_ptr<Object> obj(new Object(name, Q)); 
        //     if(name_ids_.find(name) == name_ids_.end()){
        //         name_ids_.insert({name, objects_.size()});
        //     }
        //     objects_.push_back(obj);
            
        // }
        // return true;

    }


    void ParticleFilter::yoloResultCallback(const yolo_protocol::YoloResultConstPtr& yolo_result){
        unique_lock<mutex> lock(yolo_lock_);
        yolo_result_ = *yolo_result;
        semantic_mask_ = cv::Mat::zeros(yolo_result->original.height, yolo_result->original.width, CV_8UC1);
        size_t N_detections = yolo_result->masks.size();
        for(size_t i = 0; i < N_detections; ++i){
            sensor_msgs::ImageConstPtr mask_msg = boost::make_shared<sensor_msgs::Image const>(yolo_result->masks[i]);
            cv_bridge::CvImageConstPtr mask_bridge = cv_bridge::toCvShare(mask_msg, "mono8");
            cv::Mat mask = mask_bridge->image.clone() > 60;
            string name = yolo_result->detections.detections[i].header.frame_id;
            semantic_mask_.setTo(name_ids_[name], mask);
        }
        // cv::imshow("test", 50 * semantic_mask_);
        // cv::waitKey(1);
    }
}
