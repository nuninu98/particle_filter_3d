#include <particle_filter_3d/api_class/particle_filter.h>
#include <particle_filter_3d/api_class/imu_preintegration.h>

#include <particle_filter_3d/api_class/imu_preintegration.h>

namespace PARTICLE_FILTER_3D{
    IMUPreintegration::IMUPreintegration(): queue_(), spinner_(0, &queue_),
    key_(1), doneFirstOpt_(false), lastImuT_imu_(-1), lastImuT_opt_(-1), imuIntegratorOpt_(nullptr), imuIntegratorImu_(nullptr){
        nh_.setCallbackQueue(&queue_);

        double imuAccNoise = 3.9939570888238808e-03;
        double imuGyrNoise = 1.5636343949698187e-03;
        double imuAccBiasN = 6.4356659353532566e-05;
        double imuGyrBiasN = 3.5640318696367613e-05;
        double imuGravity = 9.80511;
        double imuRPYWeight = 0.01;

        boost::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(imuGravity);
        p->accelerometerCovariance  = gtsam::Matrix33::Identity(3,3) * pow(imuAccNoise, 2); // acc white noise in continuous
        p->gyroscopeCovariance      = gtsam::Matrix33::Identity(3,3) * pow(imuGyrNoise, 2); // gyro white noise in continuous
        p->integrationCovariance    = gtsam::Matrix33::Identity(3,3) * pow(1e-4, 2); // error committed in integrating position from velocities
        gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished()); // assume zero initial bias

        priorPoseNoise_  = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished()); // rad,rad,rad,m, m, m
        priorVelNoise_  = gtsam::noiseModel::Isotropic::Sigma(3, 1e4); // m/s
        priorBiasNoise_  = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3); // 1e-2 ~ 1e-3 seems to be good
        correctionNoise_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished()); // rad,rad,rad,m, m, m
        correctionNoise2_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1, 1, 1, 1, 1, 1).finished()); // rad,rad,rad,m, m, m
        noiseModelBetweenBias_ = (gtsam::Vector(6) << imuAccBiasN, imuAccBiasN, imuAccBiasN, imuGyrBiasN, imuGyrBiasN, imuGyrBiasN).finished();
        
        imuIntegratorImu_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // setting up the IMU integration for IMU message thread
        imuIntegratorOpt_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // setting up the IMU integration for optimization        

        sub_imu_ = nh_.subscribe("/imu/data", 2000, &IMUPreintegration::imuCallback, this);
        sub_preintegration_flag_ = nh_.subscribe("preintegration_flag", 1, &IMUPreintegration::flagCallback, this);
        
        pub_imu_odom_ = nh_.advertise<nav_msgs::Odometry>("imu_odom", 1);

        spinner_.start();
    }

    IMUPreintegration::~IMUPreintegration(){

    }

    void IMUPreintegration::imuCallback(const sensor_msgs::ImuConstPtr& imu){
        unique_lock<mutex> lock(imu_mtx_);
        sensor_msgs::Imu imu_data = *imu;
        imuQueOpt_.push_back(imu_data);
        imuQueImu_.push_back(imu_data);
        if(!doneFirstOpt_ || !systemInitialized_){
            return;
        }
        double imuTime = imu->header.stamp.toSec();
        double dt = (lastImuT_imu_ < 0) ? (1.0/ 500.0) : (imuTime - lastImuT_imu_);
        lastImuT_imu_ = imuTime;
        imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(imu_data.linear_acceleration.x, imu_data.linear_acceleration.y, imu_data.linear_acceleration.z),
                                                gtsam::Vector3(imu_data.angular_velocity.x,    imu_data.angular_velocity.y,    imu_data.angular_velocity.z), dt);
        //gtsam::NavState currentState = imuIntegratorImu_->predict(prevStateOdom_, prevBiasOdom_);
        gtsam::NavState currentState = imuIntegratorImu_->predict(prevState_, prevBias_);

        gtsam::Pose3 robot2Imu(Eigen::Matrix4d::Identity());
        gtsam::Pose3 imuPose(currentState.quaternion(), currentState.position());
        gtsam::Pose3 robotPose = imuPose.compose(robot2Imu.inverse());
        nav_msgs::Odometry imu_odom;
        imu_odom.header.stamp = ros::Time::now();
        imu_odom.header.frame_id = "map";
        imu_odom.pose.pose.position.x = robotPose.translation().x();
        imu_odom.pose.pose.position.y = robotPose.translation().y();
        imu_odom.pose.pose.position.z = robotPose.translation().z();

        imu_odom.pose.pose.orientation.w = robotPose.rotation().toQuaternion().w();
        imu_odom.pose.pose.orientation.x = robotPose.rotation().toQuaternion().x();
        imu_odom.pose.pose.orientation.y = robotPose.rotation().toQuaternion().y();
        imu_odom.pose.pose.orientation.z = robotPose.rotation().toQuaternion().z();

        Eigen::Vector3d vel_world(currentState.velocity().x(), currentState.velocity().y(), currentState.velocity().z());
        Eigen::Vector3d vel_robot = robotPose.matrix().inverse().block<3, 3>(0, 0) * vel_world;
        // cout<<"vel robot: "<<vel_robot.transpose()<<endl;
        // cout<<"---"<<endl;
        imu_odom.twist.twist.linear.x = vel_robot(0);
        imu_odom.twist.twist.linear.y = vel_robot(1);
        imu_odom.twist.twist.linear.z = vel_robot(2);

        // imu_odom.twist.twist.linear.x = currentState.velocity().x();
        // imu_odom.twist.twist.linear.y = currentState.velocity().y();
        // imu_odom.twist.twist.linear.z = currentState.velocity().z();

        imu_odom.twist.twist.angular.x = imu_data.angular_velocity.x + prevBiasOdom_.gyroscope().x();
        imu_odom.twist.twist.angular.y = imu_data.angular_velocity.y + prevBiasOdom_.gyroscope().y();
        imu_odom.twist.twist.angular.z = imu_data.angular_velocity.z + prevBiasOdom_.gyroscope().z();
        
        pub_imu_odom_.publish(imu_odom);
    }

    void IMUPreintegration::flagCallback(const nav_msgs::OdometryConstPtr& stamped_pose){
        unique_lock<mutex> lock(imu_mtx_);
        double currentCorrectionTime = stamped_pose->header.stamp.toSec();
        
        if(imuQueOpt_.empty()){
            return;
        }
        geometry_msgs::Pose pose = stamped_pose->pose.pose;
        gtsam::Pose3 robotPose(gtsam::Rot3::Quaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z),
                                gtsam::Point3(pose.position.x, pose.position.y, pose.position.z));
        gtsam::Pose3 robot2Imu(Eigen::Matrix4d::Identity());
        if(!systemInitialized_){
            while(!imuQueOpt_.empty()){
                double imuTime = imuQueOpt_.front().header.stamp.toSec();
                if(imuTime < currentCorrectionTime){
                    lastImuT_opt_ = imuTime;
                    imuQueOpt_.pop_front();
                }
                else{
                    break;
                }
            }

            prevPose_ = robotPose.compose(robot2Imu);
            gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, priorPoseNoise_);
            graphFactors_.add(priorPose);

            prevVel_ = gtsam::Vector3(0, 0, 0);
            gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, priorVelNoise_);
            graphFactors_.add(priorVel);
            // initial bias
            prevBias_ = gtsam::imuBias::ConstantBias();
            gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, priorBiasNoise_);
            graphFactors_.add(priorBias);

            prevState_= gtsam::NavState(prevPose_, prevVel_);
            // add values
            graphValues_.insert(X(0), prevPose_);
            graphValues_.insert(V(0), prevVel_);
            graphValues_.insert(B(0), prevBias_);
            // optimize once
            optimizer_.update(graphFactors_, graphValues_);
            graphFactors_.resize(0);
            graphValues_.clear();

            imuIntegratorImu_->resetIntegrationAndSetBias(prevBias_);
            imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);
            
            key_ = 1;
            systemInitialized_ = true;
            return;
        }
        if(key_ == 100){
            gtsam::noiseModel::Gaussian::shared_ptr updatedPoseNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer_.marginalCovariance(X(key_-1)));
            gtsam::noiseModel::Gaussian::shared_ptr updatedVelNoise  = gtsam::noiseModel::Gaussian::Covariance(optimizer_.marginalCovariance(V(key_-1)));
            gtsam::noiseModel::Gaussian::shared_ptr updatedBiasNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer_.marginalCovariance(B(key_-1)));
            // reset graph
            optimizer_ = gtsam::ISAM2();
            graphFactors_.resize(0);
            graphValues_.clear();
            // add pose
            gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, updatedPoseNoise);
            graphFactors_.add(priorPose);
            // add velocity
            gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, updatedVelNoise);
            graphFactors_.add(priorVel);
            // add bias
            gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, updatedBiasNoise);
            graphFactors_.add(priorBias);
            // add values
            graphValues_.insert(X(0), prevPose_);
            graphValues_.insert(V(0), prevVel_);
            graphValues_.insert(B(0), prevBias_);
            // optimize once
            optimizer_.update(graphFactors_, graphValues_);
            graphFactors_.resize(0);
            graphValues_.clear();

            key_ = 1;
        }
        while (!imuQueOpt_.empty())
        {
            // pop and integrate imu data that is between two optimizations
            sensor_msgs::Imu* imu_data = &imuQueOpt_.front();
            double imuTime = imu_data->header.stamp.toSec();
            if (imuTime < currentCorrectionTime)
            {
                double dt = (lastImuT_opt_ < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_opt_);
                imuIntegratorOpt_->integrateMeasurement(
                        gtsam::Vector3(imu_data->linear_acceleration.x, imu_data->linear_acceleration.y, imu_data->linear_acceleration.z),
                        gtsam::Vector3(imu_data->angular_velocity.x, imu_data->angular_velocity.y, imu_data->angular_velocity.z), dt); 
                lastImuT_opt_ = imuTime;
                imuQueOpt_.pop_front();
            }
            else{
                break;
            }  
        }
        const gtsam::PreintegratedImuMeasurements& preint_imu = dynamic_cast<const gtsam::PreintegratedImuMeasurements&>(*imuIntegratorOpt_);
        gtsam::ImuFactor imu_factor(X(key_ - 1), V(key_ - 1), X(key_), V(key_), B(key_ - 1), preint_imu);
        graphFactors_.add(imu_factor);
        graphFactors_.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(key_ - 1), B(key_), gtsam::imuBias::ConstantBias(),
                         gtsam::noiseModel::Diagonal::Sigmas(sqrt(imuIntegratorOpt_->deltaTij()) * noiseModelBetweenBias_)));
        gtsam::Pose3 curPose = robotPose.compose(robot2Imu);
        gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key_), curPose, correctionNoise_);
        graphFactors_.add(pose_factor);
        gtsam::NavState propState = imuIntegratorOpt_->predict(prevState_, prevBias_);
        graphValues_.insert(X(key_), propState.pose());
        graphValues_.insert(V(key_), propState.v());
        graphValues_.insert(B(key_), prevBias_);
        // optimize
        optimizer_.update(graphFactors_, graphValues_);
        optimizer_.update();
        graphFactors_.resize(0);
        graphValues_.clear();
        // Overwrite the beginning of the preintegration for the next step.
        // cout<<"PROP: "<<propState<<endl;
        // cout<<"CURR POSE: "<<curPose<<endl;
        // cout<<"VEL PRV: "<<prevVel_<<endl;
        gtsam::Values result = optimizer_.calculateEstimate();
        prevPose_  = result.at<gtsam::Pose3>(X(key_));
        prevVel_   = result.at<gtsam::Vector3>(V(key_));
        prevState_ = gtsam::NavState(prevPose_, prevVel_);
        prevBias_  = result.at<gtsam::imuBias::ConstantBias>(B(key_));
        // Reset the optimization preintegration object.
        imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);
        // cout<<"VEL OPT: "<<prevVel_<<endl;
        // cout<<"====="<<endl;

        //IMU Odometry bias setting
        prevStateOdom_ = prevState_;
        prevBiasOdom_ = prevBias_;
        
        // first pop imu message older than current correction data
        double lastImuQT = -1;
        bool bias_updated = false;
        while (!imuQueImu_.empty())
        {
            sensor_msgs::Imu imu_data = imuQueImu_.front();
            double imuTime = imuQueImu_.front().header.stamp.toSec();
            if(imuTime > currentCorrectionTime){
                if(!bias_updated){
                    imuIntegratorImu_->resetIntegrationAndSetBias(prevBiasOdom_);
                    bias_updated = true;
                }
                double dt = (lastImuQT < 0) ? (1.0 / 500.0) :(imuTime - lastImuQT);
                imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(imu_data.linear_acceleration.x, imu_data.linear_acceleration.y, imu_data.linear_acceleration.z),
                                                        gtsam::Vector3(imu_data.angular_velocity.x, imu_data.angular_velocity.y, imu_data.angular_velocity.z), dt);
            }
            lastImuQT = imuTime;
            imuQueImu_.pop_front();
        }

        ++key_;
        doneFirstOpt_ = true;

    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "test_particle_filter");
    PARTICLE_FILTER_3D::ParticleFilter pf;
    PARTICLE_FILTER_3D::IMUPreintegration ip;
    Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
    pf.initialize(I);

    ros::spin();
}