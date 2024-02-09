#include "utility.h"

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

using gtsam::symbol_shorthand::B;  // Bias    (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::V;  // Lin Vel (xdot,ydot,zdot)
using gtsam::symbol_shorthand::X;  // Pose3   (x,y,z,r,p,y)

namespace liosam
{
  namespace imu_preintegration
  {

    /*//{ class ImuPreintegration() */
    class ImuPreintegration : public nodelet::Nodelet
    {
    public:
      /*//{ parameters */
      std::string uavName;

      // Topics
      string imuTopic;

      // Frames
      std::string lidarFrame;
      std::string imuFrame;
      std::string baselinkFrame;
      std::string odometryFrame;

      // IMU
      double gravity;

      double linAccBiasNoise;
      double angVelBiasNoise;
      double linAccNoise;
      double angVelNoise;

      /*//}*/

      // TF
      tf::StampedTransform tfLidar2Imu;

      // IMU TF
      Eigen::Matrix3d extRot;
      Eigen::Quaterniond extQRPY;

      std::mutex mtx;

      ros::Subscriber subImu;
      ros::Subscriber subOdometry;
      ros::Publisher pubPreOdometry;
      ros::Publisher pubLinAcc;
      ros::Publisher pubLinAccBias;
      ros::Publisher pubAngVel;
      ros::Publisher pubAngVelBias;

      bool systemInitialized = false;

      gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
      gtsam::noiseModel::Diagonal::shared_ptr priorLinVelNoise;
      gtsam::noiseModel::Diagonal::shared_ptr priorAngVelNoise;
      gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise;
      gtsam::noiseModel::Diagonal::shared_ptr correctionNoise;
      gtsam::noiseModel::Diagonal::shared_ptr correctionNoise2;
      gtsam::Vector noiseModelBetweenBias;


      gtsam::PreintegratedImuMeasurements* imuIntegratorOpt_;
      gtsam::PreintegratedImuMeasurements* imuIntegratorPredict_;

      std::deque<sensor_msgs::Imu> imuQueOpt;
      std::deque<sensor_msgs::Imu> imuQuePredict;

      gtsam::Pose3 prevPose_;
      gtsam::Vector3 prevLinVel_;
      gtsam::NavState prevState_;
      gtsam::imuBias::ConstantBias prevBias_;

      gtsam::NavState prevStateOdom_;
      gtsam::imuBias::ConstantBias prevBiasOdom_;

      bool doneFirstOpt = false;
      double lastImuT_predict = -1;
      double lastImuT_opt = -1;

      gtsam::ISAM2 optimizer;
      gtsam::NonlinearFactorGraph graphFactors;
      gtsam::Values graphValues;

      const double delta_t = 0;

      int key = 1;

      gtsam::Pose3 imu2Lidar;
      gtsam::Pose3 lidar2Imu;

    public:
      /*//{ onInit() */
      virtual void onInit()
      {
        ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();
        /*//{ load parameters */
        mrs_lib::ParamLoader pl(nh, "ImuPreintegration");

        pl.loadParam("uavName", uavName);

        pl.loadParam("lidarFrame", lidarFrame);
        pl.loadParam("baselinkFrame", baselinkFrame);
        pl.loadParam("odometryFrame", odometryFrame);

        pl.loadParam("imu/topic", imuTopic);
        addNamespace("/" + uavName, imuTopic);
        pl.loadParam("imu/frame_id", imuFrame);
        addNamespace("/" + uavName, imuFrame);
        pl.loadParam("imu/gravity", gravity);
        pl.loadParam("imu/noise/linAccBias", linAccBiasNoise);
        pl.loadParam("imu/noise/angVelBias", angVelBiasNoise);
        pl.loadParam("imu/noise/linAcc", linAccNoise);
        pl.loadParam("imu/noise/angVel", angVelNoise);

        if (!pl.loadedSuccessfully())
        {
          ROS_ERROR("[imuPreintegration]: Could not load all parameters!");
          ros::shutdown();
        }
        /*//}*/

        tf::StampedTransform tfLidar2Baselink;
        findLidar2ImuTf(lidarFrame, imuFrame, baselinkFrame, extRot, extQRPY, tfLidar2Baselink, tfLidar2Imu);

        imu2Lidar =
          gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(-tfLidar2Imu.getOrigin().x(), -tfLidar2Imu.getOrigin().y(), -tfLidar2Imu.getOrigin().z()));
        lidar2Imu =
          gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(tfLidar2Imu.getOrigin().x(), tfLidar2Imu.getOrigin().y(), tfLidar2Imu.getOrigin().z()));


        subImu = nh.subscribe<sensor_msgs::Imu>(imuTopic, 10, &ImuPreintegration::imuHandler, this,
                                                                     ros::TransportHints().tcpNoDelay());
        subOdometry = nh.subscribe<nav_msgs::Odometry>("liosam/preintegration/odom_mapping_incremental_in", 5, &ImuPreintegration::odometryHandler, this,
                                                        ros::TransportHints().tcpNoDelay());

        pubPreOdometry = nh.advertise<nav_msgs::Odometry>("liosam/preintegration/odom_preintegrated_out", 10);
        pubLinAcc = nh.advertise<geometry_msgs::Vector3Stamped>("liosam/preintegration/lin_acc_out", 10);
        pubLinAccBias = nh.advertise<geometry_msgs::Vector3Stamped>("liosam/preintegration/ang_vel_out", 10);
        pubAngVel = nh.advertise<geometry_msgs::Vector3Stamped>("liosam/preintegration/lin_acc_bias_out", 10);
        pubAngVelBias = nh.advertise<geometry_msgs::Vector3Stamped>("liosam/preintegration/ang_vel_bias_out", 10);

        boost::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(gravity);
        p->accelerometerCovariance                       = gtsam::Matrix33::Identity(3, 3) * pow(linAccNoise, 2);  // acc white noise in continuous
        p->gyroscopeCovariance                           = gtsam::Matrix33::Identity(3, 3) * pow(angVelNoise, 2);  // gyro white noise in continuous
        p->integrationCovariance = gtsam::Matrix33::Identity(3, 3) * pow(1e-4, 2);                       // error committed in integrating position from velocities

        gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());  // assume zero initial bias

        priorPoseNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished());  // rad,rad,rad,m, m, m
        priorLinVelNoise = gtsam::noiseModel::Isotropic::Sigma(3, 1e4);                                                             // m/s
        priorAngVelNoise = gtsam::noiseModel::Isotropic::Sigma(3, 1e4);                                                             // m/s
        priorBiasNoise = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3);  // 1e-2 ~ 1e-3 seems to be good
        correctionNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished());  // rad,rad,rad,m, m, m
        correctionNoise2 = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1, 1, 1, 1, 1, 1).finished());                // rad,rad,rad,m, m, m
        noiseModelBetweenBias = (gtsam::Vector(6) << linAccBiasNoise, linAccBiasNoise, linAccBiasNoise, angVelBiasNoise, angVelBiasNoise, angVelBiasNoise).finished();

        imuIntegratorPredict_ = new gtsam::PreintegratedImuMeasurements(p);  // setting up the IMU integration for IMU message thread
        imuIntegratorOpt_ = new gtsam::PreintegratedImuMeasurements(p);      // setting up the IMU integration for optimization

        ROS_INFO("\033[1;32m----> [ImuPreintegration]: initialized.\033[0m");
      }
      /*//}*/

      /*//{ resetOptimization() */
      void resetOptimization()
      {
        gtsam::ISAM2Params optParameters;
        optParameters.relinearizeThreshold = 0.1;
        optParameters.relinearizeSkip = 1;
        optimizer = gtsam::ISAM2(optParameters);

        gtsam::NonlinearFactorGraph newGraphFactors;
        graphFactors = newGraphFactors;

        gtsam::Values NewGraphValues;
        graphValues = NewGraphValues;
      }
      /*//}*/

      /*//{ resetParams() */
      void resetParams()
      {
        lastImuT_predict = -1;
        doneFirstOpt = false;
        systemInitialized = false;
      }
      /*//}*/

      /*//{ odometryHandler() */
      void odometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
      {
        ROS_INFO_ONCE("[ImuPreintegration]: odometryHandler first callback");
        std::lock_guard<std::mutex> lock(mtx);

        const double currentCorrectionTime = ROS_TIME(odomMsg);

        // make sure we have imu data to integrate
        if (imuQueOpt.empty())
        {
          return;
        }

        const float p_x = odomMsg->pose.pose.position.x;
        const float p_y = odomMsg->pose.pose.position.y;
        const float p_z = odomMsg->pose.pose.position.z;
        const float r_x = odomMsg->pose.pose.orientation.x;
        const float r_y = odomMsg->pose.pose.orientation.y;
        const float r_z = odomMsg->pose.pose.orientation.z;
        const float r_w = odomMsg->pose.pose.orientation.w;
        const bool degenerate = (int)odomMsg->pose.covariance[0] == 1;
        const gtsam::Pose3 lidarPose = gtsam::Pose3(gtsam::Rot3::Quaternion(r_w, r_x, r_y, r_z), gtsam::Point3(p_x, p_y, p_z));


        // 0. initialize system
        if (!systemInitialized)
        {
          resetOptimization();

          // pop old IMU message
          while (!imuQueOpt.empty())
          {
            if (ROS_TIME(&imuQueOpt.front()) < currentCorrectionTime - delta_t)
            {
              lastImuT_opt = ROS_TIME(&imuQueOpt.front());
              imuQueOpt.pop_front();
            } else
            {
              break;
            }
          }

          // initial pose
          prevPose_ = lidarPose.compose(lidar2Imu);
          const gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, priorPoseNoise);
          graphFactors.add(priorPose);

          // initial linear velocity
          prevLinVel_ = gtsam::Vector3(0, 0, 0);
          const gtsam::PriorFactor<gtsam::Vector3> priorLinVel(V(0), prevLinVel_, priorLinVelNoise);
          graphFactors.add(priorLinVel);

          // initial bias
          prevBias_ = gtsam::imuBias::ConstantBias();
          const gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, priorBiasNoise);
          graphFactors.add(priorBias);

          // add values
          graphValues.insert(X(0), prevPose_);
          graphValues.insert(V(0), prevLinVel_);
          graphValues.insert(B(0), prevBias_);

          // optimize once
          optimizer.update(graphFactors, graphValues);
          graphFactors.resize(0);
          graphValues.clear();

          imuIntegratorPredict_->resetIntegrationAndSetBias(prevBias_);
          imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);

          key = 1;
          systemInitialized = true;
          return;
        }


        // reset graph for speed
        if (key == 100) 
        {

          ROS_INFO("[ImuPreintegration]: resetting graph");

          // get updated noise before reset
          const gtsam::noiseModel::Gaussian::shared_ptr updatedPoseNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(X(key - 1)));
          const gtsam::noiseModel::Gaussian::shared_ptr updatedLinVelNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(V(key - 1)));
          const gtsam::noiseModel::Gaussian::shared_ptr updatedBiasNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(B(key - 1)));

          // reset graph
          resetOptimization();

          // add pose
          const gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, updatedPoseNoise);
          graphFactors.add(priorPose);

          // add linear velocity
          const gtsam::PriorFactor<gtsam::Vector3> priorLinVel(V(0), prevLinVel_, updatedLinVelNoise);
          graphFactors.add(priorLinVel);

          // add bias
          const gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, updatedBiasNoise);
          graphFactors.add(priorBias);

          // add values
          graphValues.insert(X(0), prevPose_);
          graphValues.insert(V(0), prevLinVel_);
          graphValues.insert(B(0), prevBias_);

          // optimize once
          optimizer.update(graphFactors, graphValues);
          graphFactors.resize(0);
          graphValues.clear();

          key = 1;
        }


        // 1. integrate imu data and optimize
        while (!imuQueOpt.empty())
        {
          // pop and integrate imu data that is between two optimizations
          const sensor_msgs::Imu* thisImu = &imuQueOpt.front();
          const double imuTime = ROS_TIME(thisImu);
          if (imuTime < currentCorrectionTime - delta_t)
          {
            const double dt = (lastImuT_opt < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_opt);

            if (dt <= 0)
            {
              ROS_WARN_COND(dt < 0, "invalid dt (opt): (%0.2f - %0.2f) = %0.2f", imuTime, lastImuT_opt, dt);
              imuQueOpt.pop_front();
              continue;
            }

          imuIntegratorOpt_->integrateMeasurement(gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z), 
              gtsam::Vector3(thisImu->angular_velocity.x, thisImu->angular_velocity.y, thisImu->angular_velocity.z), dt);

            lastImuT_opt = imuTime;
            imuQueOpt.pop_front();
          } else
          {
            break;
          }
        }
        // add imu factor to graph
    const gtsam::PreintegratedImuMeasurements& preint_imu = dynamic_cast<const gtsam::PreintegratedImuMeasurements&>(*imuIntegratorOpt_);
    const gtsam::ImuFactor                     imu_factor(X(key - 1), V(key - 1), X(key), V(key), B(key - 1), preint_imu);
        graphFactors.add(imu_factor);

        // add imu bias between factor
        graphFactors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(
            B(key - 1), B(key), gtsam::imuBias::ConstantBias(),
            gtsam::noiseModel::Diagonal::Sigmas(sqrt(imuIntegratorOpt_->deltaTij()) * noiseModelBetweenBias)));

        // add pose factor
        const gtsam::Pose3 curPose = lidarPose.compose(lidar2Imu);
        const gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key), curPose, degenerate ? correctionNoise2 : correctionNoise);
        graphFactors.add(pose_factor);

        // insert predicted values
        const gtsam::NavState propState_ = imuIntegratorOpt_->predict(prevState_, prevBias_);
        graphValues.insert(X(key), propState_.pose());
        graphValues.insert(V(key), propState_.v());
        graphValues.insert(B(key), prevBias_);
        /* ROS_INFO("[ImuPreintegration]: preintegrated: pos: %.2f %.2f %2.f rot: %.2f %.2f %.2f lin_vel: %.2f %.2f %.2f", */
        /*          propState_.pose().translation().x(), propState_.pose().translation().y(), propState_.pose().translation().z(), */
        /*          propState_.pose().rotation().roll(), propState_.pose().rotation().pitch(), propState_.pose().rotation().yaw(), propState_.velocity()[0], */
        /*          propState_.velocity()[1], propState_.velocity()[2]); */
        /* ROS_INFO("[ImuPreintegration]: lin_acc_bias: %.2f %.2f %.2f ang_vel_bias: %.2f %.2f %.2f", prevBias_.accelerometer()[0], prevBias_.accelerometer()[1], */
        /*          prevBias_.accelerometer()[2], prevBias_.gyroscope()[0], prevBias_.gyroscope()[1], prevBias_.gyroscope()[2]); */

        // optimize
        cout << "****************************************************" << endl;
        graphFactors.print("[ImuPreintegration]: graph\n");
        optimizer.update(graphFactors, graphValues);
        optimizer.update();
        graphFactors.resize(0);
        graphValues.clear();

        // Overwrite the beginning of the preintegration for the next step.
        const gtsam::Values result = optimizer.calculateEstimate();
        prevPose_ = result.at<gtsam::Pose3>(X(key));
        prevLinVel_ = result.at<gtsam::Vector3>(V(key));
        prevState_ = gtsam::NavState(prevPose_, prevLinVel_);
        prevBias_ = result.at<gtsam::imuBias::ConstantBias>(B(key));

        // Reset the optimization preintegration object.
        imuIntegratorOpt_->resetIntegration();

        // check optimization
        if (failureDetection(prevLinVel_, prevBias_))
        {
          resetParams();
          return;
        }


        // 2. after optiization, re-propagate imu odometry preintegration
        prevStateOdom_ = prevState_;
        prevBiasOdom_ = prevBias_;

        // first pop imu message older than current correction data
        double lastImuQT = -1;
        while (!imuQuePredict.empty() && ROS_TIME(&imuQuePredict.front()) < currentCorrectionTime - delta_t)
        {
          lastImuQT = ROS_TIME(&imuQuePredict.front());
          imuQuePredict.pop_front();
        }

        // repropogate
        if (!imuQuePredict.empty())
        {
          // reset bias use the newly optimized bias
          imuIntegratorPredict_->resetIntegrationAndSetBias(prevBiasOdom_);

          // integrate imu message from the beginning of this optimization
          for (int i = 0; i < (int)imuQuePredict.size(); ++i)
          {
            sensor_msgs::Imu* thisImu = &imuQuePredict[i];
            const double imuTime = ROS_TIME(thisImu);
            const double dt = (lastImuQT < 0) ? (1.0 / 500.0) : (imuTime - lastImuQT);

            if (dt <= 0)
            {
              ROS_WARN_COND(dt < 0, "invalid dt (QT): (%0.2f - %0.2f) = %0.2f", imuTime, lastImuQT, dt);
              continue;
            }

          imuIntegratorPredict_->integrateMeasurement(gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                                                  gtsam::Vector3(thisImu->angular_velocity.x, thisImu->angular_velocity.y, thisImu->angular_velocity.z), dt);
            lastImuQT = imuTime;
          }
        }

        geometry_msgs::Vector3Stamped lin_acc_bias_msg;
        lin_acc_bias_msg.header.stamp = ros::Time::now();
        lin_acc_bias_msg.header.frame_id = "fcu";
        lin_acc_bias_msg.vector.x = prevBias_.accelerometer()[0];
        lin_acc_bias_msg.vector.y = prevBias_.accelerometer()[1];
        lin_acc_bias_msg.vector.z = prevBias_.accelerometer()[2];
        pubLinAccBias.publish(lin_acc_bias_msg);

        geometry_msgs::Vector3Stamped ang_vel_bias_msg;
        ang_vel_bias_msg.header.stamp = ros::Time::now();
        ang_vel_bias_msg.header.frame_id = "fcu";
        ang_vel_bias_msg.vector.x = prevBias_.gyroscope()[0];
        ang_vel_bias_msg.vector.y = prevBias_.gyroscope()[1];
        ang_vel_bias_msg.vector.z = prevBias_.gyroscope()[2];
        pubAngVelBias.publish(ang_vel_bias_msg);

        ++key;
        doneFirstOpt = true;
      }
      /*//}*/

      /*//{ failureDetection() */
      bool failureDetection(const gtsam::Vector3& velCur, const gtsam::imuBias::ConstantBias& biasCur)
      {
        const Eigen::Vector3f vel(velCur.x(), velCur.y(), velCur.z());
        if (vel.norm() > 30)
        {
          ROS_WARN("Large velocity (%0.1f, %0.1f, %0.1f), reset IMU-preintegration!", vel.x(), vel.y(), vel.z());
          return true;
        }

        const Eigen::Vector3f bla(biasCur.accelerometer().x(), biasCur.accelerometer().y(), biasCur.accelerometer().z());
        const Eigen::Vector3f baa(biasCur.gyroscope().x(), biasCur.gyroscope().y(), biasCur.gyroscope().z());
        if (bla.norm() > 1.0 || baa.norm() > 1.0)
        {
          ROS_WARN("Large bias, reset IMU-preintegration!");
          return true;
        }

        return false;
      }
      /*//}*/

      /*//{ imuHandler() */
      void imuHandler(const sensor_msgs::Imu::ConstPtr& msg_in)
      {
        ROS_INFO_ONCE("[ImuPreintegration]: imuHandler first callback");
        std::lock_guard<std::mutex> lock(mtx);

        const sensor_msgs::Imu thisImu = imuConverter(*msg_in, extRot, extQRPY);

        imuQueOpt.push_back(thisImu);
        imuQuePredict.push_back(thisImu);

        if (doneFirstOpt == false)
        {
          ROS_INFO_THROTTLE(1.0, "[ImuPreintegration]: waiting for first optimalization");
          return;
        }

        const double imuTime = ROS_TIME(msg_in);
        const double dt = (lastImuT_predict < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_predict);
        if (dt <= 0)
        {
          ROS_WARN_COND(dt < 0, "invalid dt (imu): (%0.2f - %0.2f) = %0.2f", imuTime, lastImuT_predict, dt);
          return;
        }
        lastImuT_predict = imuTime;

        // integrate this single imu message
        imuIntegratorPredict_->integrateMeasurement(gtsam::Vector3(thisImu.linear_acceleration.x, thisImu.linear_acceleration.y, thisImu.linear_acceleration.z),
                                            gtsam::Vector3(thisImu.angular_velocity.x, thisImu.angular_velocity.y, thisImu.angular_velocity.z), dt);

        // predict odometry
        const gtsam::NavState currentState = imuIntegratorPredict_->predict(prevStateOdom_, prevBiasOdom_);

        // publish odometry
        nav_msgs::Odometry::Ptr odometry = boost::make_shared<nav_msgs::Odometry>();
        odometry->header.stamp = msg_in->header.stamp;
        odometry->header.frame_id = odometryFrame;
        odometry->child_frame_id = baselinkFrame;

        // transform imu pose to ldiar
        const gtsam::Pose3 imuPose = gtsam::Pose3(currentState.quaternion(), currentState.position());
        const gtsam::Pose3 lidarPose = imuPose.compose(imu2Lidar);

        odometry->pose.pose.position.x = lidarPose.translation().x();
        odometry->pose.pose.position.y = lidarPose.translation().y();
        odometry->pose.pose.position.z = lidarPose.translation().z();
        odometry->pose.pose.orientation.x = lidarPose.rotation().toQuaternion().x();
        odometry->pose.pose.orientation.y = lidarPose.rotation().toQuaternion().y();
        odometry->pose.pose.orientation.z = lidarPose.rotation().toQuaternion().z();
        odometry->pose.pose.orientation.w = lidarPose.rotation().toQuaternion().w();

        odometry->twist.twist.linear.x = currentState.velocity().x();
        odometry->twist.twist.linear.y = currentState.velocity().y();
        odometry->twist.twist.linear.z = currentState.velocity().z();
        odometry->twist.twist.angular.x = thisImu.angular_velocity.x + prevBiasOdom_.gyroscope().x();
        odometry->twist.twist.angular.y = thisImu.angular_velocity.y + prevBiasOdom_.gyroscope().y();
        odometry->twist.twist.angular.z = thisImu.angular_velocity.z + prevBiasOdom_.gyroscope().z();
        pubPreOdometry.publish(odometry);

        geometry_msgs::Vector3Stamped accBiasMsg;
        accBiasMsg.header.stamp = ros::Time::now();
        accBiasMsg.header.frame_id = "fcu";
        accBiasMsg.vector.x = prevBiasOdom_.accelerometer().x();
        accBiasMsg.vector.y = prevBiasOdom_.accelerometer().y();
        accBiasMsg.vector.z = prevBiasOdom_.accelerometer().z();
        pubLinAccBias.publish(accBiasMsg);

        geometry_msgs::Vector3Stamped gyroBiasMsg;
        gyroBiasMsg.header.stamp = ros::Time::now();
        gyroBiasMsg.header.frame_id = "fcu";
        gyroBiasMsg.vector.x = prevBiasOdom_.gyroscope().x();
        gyroBiasMsg.vector.y = prevBiasOdom_.gyroscope().y();
        gyroBiasMsg.vector.z = prevBiasOdom_.gyroscope().z();
        pubAngVelBias.publish(gyroBiasMsg);
      }
      /*//}*/
    };
    /*//}*/

  }  // namespace imu_preintegration
}  // namespace liosam

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(liosam::imu_preintegration::ImuPreintegration, nodelet::Nodelet)
