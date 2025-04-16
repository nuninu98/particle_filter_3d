#ifndef __PARTICLE_FILTER_IMU_PREINTEGRATION_H__
#define __PARTICLE_FILTER_IMU_PREINTEGRATION_H__

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <ros/callback_queue.h>
#include <ros/spinner.h>
#include <mutex>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <nav_msgs/Odometry.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)

using namespace std;

namespace PARTICLE_FILTER_3D{
    class IMUPreintegration{
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        private:
            ros::NodeHandle nh_, pnh_;
            ros::CallbackQueue queue_;
            ros::AsyncSpinner spinner_;
            ros::Subscriber sub_imu_;

            ros::Publisher pub_imu_odom_;

            mutex imu_mtx_;
            gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise_;
            gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise_;
            gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise_;
            gtsam::noiseModel::Diagonal::shared_ptr correctionNoise_;
            gtsam::noiseModel::Diagonal::shared_ptr correctionNoise2_;
            gtsam::Vector noiseModelBetweenBias_;


            gtsam::PreintegratedImuMeasurements *imuIntegratorOpt_;
            gtsam::PreintegratedImuMeasurements *imuIntegratorImu_;
            bool systemInitialized_ = false;
            deque<sensor_msgs::Imu> imuQueOpt_;
            deque<sensor_msgs::Imu> imuQueImu_;

            gtsam::Pose3 prevPose_;
            gtsam::Vector3 prevVel_;
            gtsam::NavState prevState_;
            gtsam::imuBias::ConstantBias prevBias_;

            gtsam::NavState prevStateOdom_;
            gtsam::imuBias::ConstantBias prevBiasOdom_;

            bool doneFirstOpt_;
            double lastImuT_imu_;
            double lastImuT_opt_;

            gtsam::ISAM2 optimizer_;
            gtsam::NonlinearFactorGraph graphFactors_;
            gtsam::Values graphValues_;

            int key_;
            Eigen::Matrix4d Tri_;

            void imuCallback(const sensor_msgs::ImuConstPtr& imu);

            bool getPreintegratedOdom(const sensor_msgs::Imu& imu_data, nav_msgs::Odometry& imu_odom);
        public:
            IMUPreintegration();

            void optimWithPose(const Eigen::Matrix4d& robot_pose, double currentCorrectionTime);

            ~IMUPreintegration();

            void reset();
    };
}

#endif