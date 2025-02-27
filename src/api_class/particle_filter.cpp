#include <particle_filter_3d/api_class/particle_filter.h>
namespace PARTICLE_FILTER_3D{
    ParticleFilter::ParticleFilter(): queue_(), spinner_(0, &queue_), pnh_("~"), N_particles_(1000), pose_(Eigen::Matrix4d::Identity()),
    tf_lidar2_robot_(Eigen::Matrix4d::Identity()), listener_(buffer_), kill_flag_(false), kill_done_(false){
        nh_.setCallbackQueue(&queue_);
        particles_.resize(N_particles_, Particle());
        grid_submap_.initialize(100.0, 100.0, 40.0, 0.1);

        int particles_num;
        
        pnh_.param<int>("num_particles", particles_num, 1000);
        N_particles_ = particles_num;

        string lidar_topic = "";
        pnh_.param<string>("lidar_topic", lidar_topic, "velodyne_points");
        sub_lidar_ = nh_.subscribe(lidar_topic, 1, &ParticleFilter::lidarCallback, this);
        
        string map_file_path = "";
        pnh_.param<string>("map_file", map_file_path, "odometry/filtered");
        pcl::io::loadPCDFile(map_file_path, pcd_map_);

        string object_file_path = "";
        pnh_.param<string>("object_file", object_file_path, "");
        
        if(!loadObjectMap(object_file_path)){
            ROS_ERROR_STREAM("No such object file: " + object_file_path);
        }

        pnh_.param<bool>("publish_odom_tf", pub_odom_tf_, false);    

        pub_map_ = nh_.advertise<sensor_msgs::PointCloud2>("pcd_map", 1);
        pub_particles_ = nh_.advertise<visualization_msgs::MarkerArray>("particles", 1);
        
        submap_thread_ = thread(&ParticleFilter::submapFlagCallback, this);
        submap_thread_.detach();
        last_odom_.header.stamp = ros::Time(0.0);
        last_tf_stamp_ = ros::Time(0.0);
        tf_lidar2_robot_(2, 3) = 0.4; 

        sub_initpose_ = nh_.subscribe("initialpose", 1, &ParticleFilter::initialPoseCallback, this);

        voxel_.setLeafSize(0.3, 0.3, 0.3);
        initialize(Eigen::Matrix4d::Identity());
        
        pub_pose_ = nh_.advertise<nav_msgs::Odometry>("localization_pose", 1);
        pub_map_objects_ = nh_.advertise<visualization_msgs::MarkerArray>("map_objects", 1);
                
        bool enable_preintegration = false;
        pnh_.param<bool>("imu_preintegration/enable", enable_preintegration, false); 
        if(enable_preintegration){
            ip_ = new IMUPreintegration();
            sub_odometry_ = nh_.subscribe("imu_odom", 1, &ParticleFilter::odomCallback, this);
        }
        else{
            string odom_topic = "";
            pnh_.param<string>("odom_topic", odom_topic, "odometry/filtered");
            sub_odometry_ = nh_.subscribe(odom_topic, 1, &ParticleFilter::odomCallback, this);
        }

        pnh_.param<double>("alpha_v", alpha_v_, 3.0); 
        pnh_.param<double>("alpha_w", alpha_w_, 1.0); 
        
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

    double last_flag_stamp = -1;
    void ParticleFilter::lidarCallback(const sensor_msgs::PointCloud2ConstPtr& cloud){
        unique_lock<mutex> lock(mtx_);
        if(pcd_map_.empty()){
            return;
        }
       
        ros::Time begin = ros::Time::now();
        pcl::PointCloud<pcl::PointXYZI> raw_lidar;
        pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_robot(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI> lidar_robot_ds;
        pcl::fromROSMsg(*cloud, raw_lidar);
        pcl::transformPointCloud(raw_lidar, *lidar_robot, tf_lidar2_robot_);
        
        voxelize(lidar_robot, lidar_robot_ds, 0.2);
        // lidar_robot_ds = *lidar_robot;
        
        Eigen::Matrix4d last_submap_pose = submap_updated_pose_;
        for(int cnt = 0; cnt < 3; cnt++){
            submap_mtx_.lock();
            for(size_t i = 0; i < N_particles_; ++i){
                grid_submap_.updateScore(lidar_robot_ds, particles_[i]);
            }
            double weigth_sum = 0.0;
            for(size_t i = 0; i < N_particles_; ++i){
                double w = particles_[i].getWeight();
                weigth_sum += w;
            }
            for(size_t i = 0; i < N_particles_; ++i){
                double w = particles_[i].getWeight();
                particles_[i].setWeight(w/ weigth_sum);
            }
            
            submap_mtx_.unlock();
            
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
        }
        
        
        publishParticle();
        calculatePose();
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
        pub_pose_.publish(odom_msg);
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

        normal_distribution<double> nd_vx(0.0,  alpha_v_* abs(odom->twist.twist.linear.x));
        normal_distribution<double> nd_vy(0.0,  alpha_v_* abs(odom->twist.twist.linear.y));
        normal_distribution<double> nd_vz(0.0,  alpha_v_* abs(odom->twist.twist.linear.z));
        normal_distribution<double> nd_wx(0.0, alpha_w_* abs(odom->twist.twist.angular.x));
        normal_distribution<double> nd_wy(0.0, alpha_w_* abs(odom->twist.twist.angular.y));
        normal_distribution<double> nd_wz(0.0, alpha_w_* abs(odom->twist.twist.angular.z));
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
            if(odom->header.stamp.toSec() > initOdomStamp_ + 1.5){
                ROS_INFO_ONCE("INIT!");
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
        normal_distribution<double> nd_x(0.0, 0.05);
        normal_distribution<double> nd_y(0.0, 0.05);
        normal_distribution<double> nd_z(0.0, 0.05);
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
        }
    }

    void ParticleFilter::calculatePose(){
        Eigen::Vector3d pose_avg = Eigen::Vector3d::Zero();
        Eigen::MatrixXd Q(4, N_particles_);
        for(int i = 0; i < N_particles_; ++i){
            Eigen::Matrix4d pose = particles_[i].getPose();
            Eigen::Quaterniond q(pose.block<3, 3>(0, 0));
            Eigen::Vector4d q_vec(q.x(), q.y(), q.z(), q.w());
            Q.col(i) = q_vec;
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

        visualization_msgs::MarkerArray obj_vis;
        ros::Time stamp = ros::Time::now();
        for(int i = 0; i < objects_.size(); ++i){
            gtsam_quadrics::ConstrainedDualQuadric Q = objects_[i]->Q();
            string name = objects_[i]->name();
            visualization_msgs::Marker base;
            base.header.frame_id = "map";
            base.header.stamp = stamp;
            base.pose.position.x = Q.pose().translation().x();
            base.pose.position.y = Q.pose().translation().y();
            base.pose.position.z = Q.pose().translation().z();
            Eigen::Quaterniond q(Q.pose().matrix().block<3, 3>(0, 0));
            base.pose.orientation.w = q.w();
            base.pose.orientation.x = q.x();
            base.pose.orientation.y = q.y();
            base.pose.orientation.z = q.z();
            base.color.a = 1.0;
            
            visualization_msgs::Marker obj_quad = base;
            obj_quad.id = 2*i;
            obj_quad.type = visualization_msgs::Marker::SPHERE;
            obj_quad.scale.x = Q.radii()(0);
            obj_quad.scale.y = Q.radii()(1);
            obj_quad.scale.z = Q.radii()(2);
            obj_quad.color.r = 255.0;
            obj_quad.color.g = 0.0;
            obj_quad.color.b = 255.0;

            visualization_msgs::Marker name_text = base;
            name_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            name_text.id = 2*i + 1;
            name_text.text = name;
            name_text.scale.x = Q.radii().norm();
            name_text.scale.y = Q.radii().norm();
            name_text.scale.z = Q.radii().norm();
            obj_vis.markers.push_back(obj_quad);
            obj_vis.markers.push_back(name_text);
        }
        pub_map_objects_.publish(obj_vis);
    }

    void ParticleFilter::publishParticle(){
        visualization_msgs::MarkerArray arr;
        ros::Time stamp = ros::Time::now();
        int id = 0;
        for(const auto& p : particles_){
            if(id % 10 != 0){
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

    bool ParticleFilter::loadObjectMap(const string& path){ // name x y z qw qx qy qz rx ry tz
        ifstream object_file(path);
        if(!object_file.good()){
            return false;
        }
        for(string line; getline(object_file, line);){
            vector<string> splitted;
            string str_left = line;
            while(!str_left.empty()){
                int id = str_left.find(" ");
                if(id == string::npos){
                    splitted.push_back(str_left);
                    break;
                }
                string split = str_left.substr(0, id);
                str_left = str_left.substr(id + 1);
                splitted.push_back(split);
            }
            if(splitted.size() != 11){
                return false;
            }
            string name = splitted[0];
            
            gtsam::Vector3 radii;
            Eigen::Matrix4d se3 = Eigen::Matrix4d::Identity();
            se3(0, 3) = stod(splitted[1]);
            se3(1, 3) = stod(splitted[2]);
            se3(2, 3) = stod(splitted[3]);
            double qw = stod(splitted[4]);
            double qx = stod(splitted[5]);
            double qy = stod(splitted[6]);
            double qz = stod(splitted[7]);
            radii(0) = stod(splitted[8]);
            radii(1) = stod(splitted[9]);
            radii(2) = stod(splitted[10]);
            Eigen::Quaterniond quat(qw, qx, qy, qz);
            se3.block<3, 3>(0, 0) = quat.toRotationMatrix();
            
            gtsam::Pose3 pose(se3);
            gtsam_quadrics::ConstrainedDualQuadric Q(pose, radii);
            shared_ptr<Object> obj(new Object(name, Q)); 
            objects_.push_back(obj);
        }
        return true;

    }
}
