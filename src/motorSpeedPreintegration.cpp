#include "utility.h"

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

#include <motor_speed_factor/motor_speed_factor.h>
#include <motor_speed_factor/motor_speed_bias.h>

using gtsam::symbol_shorthand::B;  // Bias    (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::V;  // Lin Vel (xdot,ydot,zdot)
using gtsam::symbol_shorthand::W;  // Ang Vel (rdot,pdot,ydot)
using gtsam::symbol_shorthand::X;  // Pose3   (x,y,z,r,p,y)

namespace liosam
{
namespace motor_speed_preintegration
{

/*//{ class MotorSpeedPreintegration() */
class MotorSpeedPreintegration : public nodelet::Nodelet {

public:
  /*//{ parameters */
  std::string uavName;

  // Frames
  std::string lidarFrame;
  std::string imuFrame;
  std::string baselinkFrame;
  std::string odometryFrame;

  // IMU
  string imuType;

  // Motor Speeds
  int   numMotors;
  float mass;
  float gravity;
  float propMass;
  float motorConstant;
  float momentConstant;
  float linAccNoise;
  float angAccNoise;
  float linAccBiasNoise;
  float angAccBiasNoise;

  /*//}*/

  // TF
  tf::StampedTransform tfLidar2Imu;

  // IMU TF
  Eigen::Matrix3d    extRot;
  Eigen::Quaterniond extQRPY;

  std::mutex mtx;

  ros::Subscriber subMotorSpeed;
  ros::Subscriber subOdometry;
  ros::Publisher  pubImuOdometry;
  ros::Publisher  pubLinAcc;
  ros::Publisher  pubLinAccBias;
  ros::Publisher  pubAngAcc;
  ros::Publisher  pubAngAccBias;

  bool systemInitialized = false;

  gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
  gtsam::noiseModel::Diagonal::shared_ptr priorLinVelNoise;
  gtsam::noiseModel::Diagonal::shared_ptr priorAngVelNoise;
  gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise;
  gtsam::noiseModel::Diagonal::shared_ptr correctionNoise;
  gtsam::noiseModel::Diagonal::shared_ptr correctionNoise2;
  gtsam::Vector                           noiseModelBetweenBias;


  gtsam::PreintegratedMotorSpeedMeasurements* motorSpeedIntegratorOpt_;
  gtsam::PreintegratedMotorSpeedMeasurements* motorSpeedIntegratorPredict_;

  std::deque<mrs_msgs::Float64ArrayStamped> motorSpeedQueOpt;
  std::deque<mrs_msgs::Float64ArrayStamped> motorSpeedQueImu;

  gtsam::Pose3                          prevPose_;
  gtsam::Vector3                        prevLinVel_;
  gtsam::Vector3                        prevAngVel_;
  gtsam::FullState                      prevState_;
  gtsam::motor_speed_bias::ConstantBias prevBias_;

  gtsam::motor_speed_bias::ConstantBias initBias_;

  gtsam::FullState                      prevStateOdom;
  gtsam::motor_speed_bias::ConstantBias prevBiasOdom;

  bool   doneFirstOpt            = false;
  double lastMotorSpeedT_predict = -1;
  double lastMotorSpeedT_opt     = -1;

  gtsam::ISAM2                optimizer;
  gtsam::NonlinearFactorGraph graphFactors;
  gtsam::Values               graphValues;

  const double delta_t = 0;

  int key = 1;

  const gtsam::Pose3 imu2Lidar =
      gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(-tfLidar2Imu.getOrigin().x(), -tfLidar2Imu.getOrigin().y(), -tfLidar2Imu.getOrigin().z()));
  const gtsam::Pose3 lidar2Imu =
      gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(tfLidar2Imu.getOrigin().x(), tfLidar2Imu.getOrigin().y(), tfLidar2Imu.getOrigin().z()));

  bool is_initialized_ = false;

public:
  /*//{ onInit()() */
  virtual void onInit() {
    ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();

    /*//{ load parameters */
    mrs_lib::ParamLoader pl(nh, "motorSpeedPreintegration");

    pl.loadParam("uavName", uavName);

    pl.loadParam("imuType", imuType);

    pl.loadParam("lidarFrame", lidarFrame);
    pl.loadParam(imuType + "/frame", imuFrame);
    pl.loadParam("baselinkFrame", baselinkFrame);
    pl.loadParam("odometryFrame", odometryFrame);
    addNamespace(uavName, lidarFrame);
    addNamespace(uavName, imuFrame);
    addNamespace(uavName, baselinkFrame);
    addNamespace(uavName, odometryFrame);

    pl.loadParam("motor_speeds/mass", mass);
    pl.loadParam("motor_speeds/gravity", gravity);
    pl.loadParam("motor_speeds/numMotors", numMotors);
    pl.loadParam("motor_speeds/propMass", propMass);
    pl.loadParam("motor_speeds/motorConstant", motorConstant);
    pl.loadParam("motor_speeds/momentConstant", momentConstant);

    pl.loadParam("motor_speeds/linAccBiasNoise", linAccBiasNoise);
    pl.loadParam("motor_speeds/angAccBiasNoise", angAccBiasNoise);

    pl.loadParam("motor_speeds/linAccNoise", linAccNoise);
    pl.loadParam("motor_speeds/angAccNoise", angAccNoise);

    double init_bias_acc_lin_x, init_bias_acc_lin_y, init_bias_acc_lin_z;
    pl.loadParam("motor_speeds/initBias/linear/x", init_bias_acc_lin_x);
    pl.loadParam("motor_speeds/initBias/linear/y", init_bias_acc_lin_y);
    pl.loadParam("motor_speeds/initBias/linear/z", init_bias_acc_lin_z);

    double init_bias_acc_ang_x, init_bias_acc_ang_y, init_bias_acc_ang_z;
    pl.loadParam("motor_speeds/initBias/angular/x", init_bias_acc_ang_x);
    pl.loadParam("motor_speeds/initBias/angular/y", init_bias_acc_ang_y);
    pl.loadParam("motor_speeds/initBias/angular/z", init_bias_acc_ang_z);

    if (!pl.loadedSuccessfully()) {
      ROS_ERROR("[MotorSpeedPreintegration]: Could not load all parameters!");
      ros::shutdown();
    }
    /*//}*/

      initBias_ = gtsam::motor_speed_bias::ConstantBias((gtsam::Vector3() << init_bias_acc_lin_x, init_bias_acc_lin_y, init_bias_acc_lin_z).finished(), (gtsam::Vector3() <<  init_bias_acc_ang_x, init_bias_acc_ang_y, init_bias_acc_ang_z).finished());

    tf::StampedTransform tfLidar2Baselink;
    findLidar2ImuTf(lidarFrame, imuFrame, baselinkFrame, extRot, extQRPY, tfLidar2Baselink, tfLidar2Imu);

    subMotorSpeed = nh.subscribe<mrs_msgs::Float64ArrayStamped>("liosam/preintegration/motor_speeds_in", 10, &MotorSpeedPreintegration::motorSpeedHandler, this,
                                                                ros::TransportHints().tcpNoDelay());
    subOdometry   = nh.subscribe<nav_msgs::Odometry>("liosam/preintegration/odom_mapping_incremental_in", 5, &MotorSpeedPreintegration::odometryHandler, this,
                                                   ros::TransportHints().tcpNoDelay());

    pubImuOdometry = nh.advertise<nav_msgs::Odometry>("liosam/preintegration/odom_preintegrated_out", 10);
    pubLinAcc      = nh.advertise<geometry_msgs::Vector3Stamped>("liosam/preintegration/lin_acc_out", 10);
    pubAngAcc      = nh.advertise<geometry_msgs::Vector3Stamped>("liosam/preintegration/ang_acc_out", 10);
    pubLinAccBias  = nh.advertise<geometry_msgs::Vector3Stamped>("liosam/preintegration/lin_acc_bias_out", 10);
    pubAngAccBias  = nh.advertise<geometry_msgs::Vector3Stamped>("liosam/preintegration/ang_acc_bias_out", 10);

    boost::shared_ptr<gtsam::MotorSpeedParams> p = gtsam::MotorSpeedParams::MakeSharedU(gravity);
    p->setMass(mass + (float)numMotors * propMass);
    p->setMotorConstant(motorConstant);
    p->setMomentConstant(momentConstant);
    p->setNumRotors(numMotors);
    p->setRotorDirs(std::vector<int>{-1, -1, 1, 1});
    /* p->accelerometerCovariance                       = gtsam::Matrix33::Identity(3, 3) * pow(imuAccNoise, 2);  // acc white noise in continuous */
    /* p->gyroscopeCovariance                           = gtsam::Matrix33::Identity(3, 3) * pow(imuGyrNoise, 2);  // gyro white noise in continuous */
    p->setAccelerationCovariance(gtsam::Matrix33::Identity(3, 3) * pow(linAccNoise, 2));  // acc white noise in continuous
    p->setAlphaCovariance(gtsam::Matrix33::Identity(3, 3) * pow(angAccNoise, 2));         // acc white noise in continuous
    p->integrationCovariance = gtsam::Matrix33::Identity(3, 3) * pow(1e-4, 2);            // error committed in integrating position from velocities
    gtsam::motor_speed_bias::ConstantBias prior_imu_bias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());  // assume zero initial bias

    priorPoseNoise   = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished());  // rad,rad,rad,m, m, m
    priorLinVelNoise = gtsam::noiseModel::Isotropic::Sigma(3, 1e4);                                                               // m/s
    priorAngVelNoise = gtsam::noiseModel::Isotropic::Sigma(3, 1e4);                                                               // m/s
    priorBiasNoise   = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3);                                                           // 1e-2 ~ 1e-3 seems to be good
    correctionNoise  = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished());  // rad,rad,rad,m, m, m
    correctionNoise2 = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1, 1, 1, 1, 1, 1).finished());                 // rad,rad,rad,m, m, m
    noiseModelBetweenBias =
        (gtsam::Vector(6) << linAccBiasNoise, linAccBiasNoise, linAccBiasNoise, angAccBiasNoise, angAccBiasNoise, angAccBiasNoise).finished();

    motorSpeedIntegratorPredict_ = new gtsam::PreintegratedMotorSpeedMeasurements(p);  // setting up the motor speed integration for IMU message thread
    motorSpeedIntegratorOpt_     = new gtsam::PreintegratedMotorSpeedMeasurements(p);  // setting up the motor speed integration for optimization

    ROS_INFO("\033[1;32m----> [MotorSpeedPreintegration]: initialized.\033[0m");
    is_initialized_ = true;
  }
  /*//}*/

  /*//{ resetOptimization() */
  void resetOptimization() {
    gtsam::ISAM2Params optParameters;
    optParameters.relinearizeThreshold = 0.1;
    optParameters.relinearizeSkip      = 1;
    optimizer                          = gtsam::ISAM2(optParameters);

    gtsam::NonlinearFactorGraph newGraphFactors;
    graphFactors = newGraphFactors;

    gtsam::Values NewGraphValues;
    graphValues = NewGraphValues;
  }
  /*//}*/

  /*//{ resetParams() */
  void resetParams() {
    lastMotorSpeedT_predict = -1;
    doneFirstOpt            = false;
    systemInitialized       = false;
  }
  /*//}*/

  /*//{ odometryHandler() */
  // callback of incremental odometry from mapping
  void odometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg) {

    if (!is_initialized_) {
      return;
    }

    ros::Time t_start = ros::Time::now();

    ROS_INFO_ONCE("[MotorSpeedPreintegration]: odometryHandler first callback");
    std::lock_guard<std::mutex> lock(mtx);

    const double currentCorrectionTime = ROS_TIME(odomMsg);

    // make sure we have imu data to integrate
    if (motorSpeedQueOpt.empty()) {
      ROS_INFO_THROTTLE(1.0, "[MotorSpeedPreintegration]: no data to integrate");
      return;
    }

    const float        p_x        = odomMsg->pose.pose.position.x;
    const float        p_y        = odomMsg->pose.pose.position.y;
    const float        p_z        = odomMsg->pose.pose.position.z;
    const float        r_x        = odomMsg->pose.pose.orientation.x;
    const float        r_y        = odomMsg->pose.pose.orientation.y;
    const float        r_z        = odomMsg->pose.pose.orientation.z;
    const float        r_w        = odomMsg->pose.pose.orientation.w;
    const bool         degenerate = (int)odomMsg->pose.covariance[0] == 1;
    const gtsam::Pose3 lidarPose  = gtsam::Pose3(gtsam::Rot3::Quaternion(r_w, r_x, r_y, r_z), gtsam::Point3(p_x, p_y, p_z));


    // 0. initialize system
    if (!systemInitialized) {
      resetOptimization();

      // pop old IMU message
      while (!motorSpeedQueOpt.empty()) {
        if (ROS_TIME(&motorSpeedQueOpt.front()) < currentCorrectionTime - delta_t) {
          lastMotorSpeedT_opt = ROS_TIME(&motorSpeedQueOpt.front());
          motorSpeedQueOpt.pop_front();
        } else {
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

      // initial angular velocity
      prevAngVel_ = gtsam::Vector3(0, 0, 0);
      const gtsam::PriorFactor<gtsam::Vector3> priorAngVel(V(0), prevAngVel_, priorAngVelNoise);
      graphFactors.add(priorAngVel);

      // initial bias
      const gtsam::PriorFactor<gtsam::motor_speed_bias::ConstantBias> priorBias(B(0), initBias_, priorBiasNoise);
      graphFactors.add(priorBias);

      // add values
      graphValues.insert(X(0), prevPose_);
      graphValues.insert(V(0), prevLinVel_);
      graphValues.insert(W(0), prevAngVel_);
      graphValues.insert(B(0), initBias_);

      // optimize once
      optimizer.update(graphFactors, graphValues);
      graphFactors.resize(0);
      graphValues.clear();

      motorSpeedIntegratorPredict_->resetIntegration();
      motorSpeedIntegratorOpt_->resetIntegration();

      key               = 1;
      systemInitialized = true;
      ROS_INFO_THROTTLE(1.0, "[MotorSpeedPreintegration]: initialized graph");
      return;
    }

    // reset graph for speed
    if (key == 100) {
      ROS_INFO("[MotorSpeedPreintegration]: resetting graph");
      // get updated noise before reset
      const gtsam::noiseModel::Gaussian::shared_ptr updatedPoseNoise   = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(X(key - 1)));
      const gtsam::noiseModel::Gaussian::shared_ptr updatedLinVelNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(V(key - 1)));
      const gtsam::noiseModel::Gaussian::shared_ptr updatedAngVelNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(W(key - 1)));
      const gtsam::noiseModel::Gaussian::shared_ptr updatedBiasNoise   = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(B(key - 1)));
      // reset graph
      resetOptimization();
      // add pose
      const gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, updatedPoseNoise);
      graphFactors.add(priorPose);
      // add linear velocity
      const gtsam::PriorFactor<gtsam::Vector3> priorLinVel(V(0), prevLinVel_, updatedLinVelNoise);
      graphFactors.add(priorLinVel);
      // add angular velocity
      const gtsam::PriorFactor<gtsam::Vector3> priorAngVel(W(0), prevAngVel_, updatedAngVelNoise);
      graphFactors.add(priorAngVel);
      // add bias
      const gtsam::PriorFactor<gtsam::motor_speed_bias::ConstantBias> priorBias(B(0), prevBias_, updatedBiasNoise);
      graphFactors.add(priorBias);
      // add values
      graphValues.insert(X(0), prevPose_);
      graphValues.insert(V(0), prevLinVel_);
      graphValues.insert(W(0), prevAngVel_);
      graphValues.insert(B(0), prevBias_);
      // optimize once
      optimizer.update(graphFactors, graphValues);
      graphFactors.resize(0);
      graphValues.clear();

      key = 1;
    }


    ros::Time t_reset = ros::Time::now();
    bool is_motor_speed_integrated = false;

    // 1. integrate imu data and optimize
    while (!motorSpeedQueOpt.empty()) {
      // pop and integrate imu data that is between two optimizations
      const mrs_msgs::Float64ArrayStamped* thisMotorSpeeds = &motorSpeedQueOpt.front();
      const double                         imuTime         = ROS_TIME(thisMotorSpeeds);
      if (imuTime < currentCorrectionTime - delta_t) {
        const double dt = (lastMotorSpeedT_opt < 0) ? (1.0 / 500.0) : (imuTime - lastMotorSpeedT_opt);

        if (dt <= 0) {
          ROS_WARN_COND(dt < 0, "invalid dt (opt): (%0.2f - %0.2f) = %0.2f", imuTime, lastMotorSpeedT_opt, dt);
          motorSpeedQueOpt.pop_front();
          continue;
        }

        motorSpeedIntegratorOpt_->integrateMeasurement(
            gtsam::Vector4(thisMotorSpeeds->values[0], thisMotorSpeeds->values[1], thisMotorSpeeds->values[2], thisMotorSpeeds->values[3]), dt);
        ROS_INFO_THROTTLE(1.0, "[MotorSpeedPreintegration]: motor speeds: %.2f %.2f %.2f %.2f dt: %.2f", thisMotorSpeeds->values[0], thisMotorSpeeds->values[1],
                          thisMotorSpeeds->values[2], thisMotorSpeeds->values[3], dt);

        lastMotorSpeedT_opt = imuTime;
        motorSpeedQueOpt.pop_front();
        is_motor_speed_integrated = true;

      } else {
        break;
      }
    }

    if (!is_motor_speed_integrated) {
      ROS_INFO("[MotorSpeedPreintegration]: No motor speeds were integrated. Skipping optimization.");
      return;
    }

    ros::Time t_integrate = ros::Time::now();

    // add motor speed factor to graph
    const gtsam::PreintegratedMotorSpeedMeasurements& preint_motor_speeds =
        dynamic_cast<const gtsam::PreintegratedMotorSpeedMeasurements&>(*motorSpeedIntegratorOpt_);
    const gtsam::MotorSpeedFactor motor_speed_factor(X(key - 1), V(key - 1), W(key - 1), X(key), V(key), W(key), B(key), preint_motor_speeds);
    const gtsam::Vector3          lin_acc_b = preint_motor_speeds.lastAcc();
    const gtsam::Vector3          ang_acc_b = preint_motor_speeds.lastAlpha();

    graphFactors.add(motor_speed_factor);
    // add imu bias between factor
    graphFactors.add(gtsam::BetweenFactor<gtsam::motor_speed_bias::ConstantBias>(
        B(key - 1), B(key), gtsam::motor_speed_bias::ConstantBias(),
        gtsam::noiseModel::Diagonal::Sigmas(sqrt(motorSpeedIntegratorOpt_->deltaTij()) * noiseModelBetweenBias)));
    // add pose factor
    const gtsam::Pose3                     curPose = lidarPose.compose(lidar2Imu);
    const gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key), curPose, degenerate ? correctionNoise2 : correctionNoise);
    graphFactors.add(pose_factor);
    // insert predicted values
    const gtsam::FullState propState_ = motorSpeedIntegratorOpt_->predict(prevState_, prevBias_);
    graphValues.insert(X(key), propState_.pose());
    graphValues.insert(V(key), propState_.v());
    graphValues.insert(W(key), propState_.w());
    graphValues.insert(B(key), prevBias_);
    ROS_INFO_THROTTLE(1.0, "[MotorSpeedPreintegration]: preintegrated: pos: %.2f %.2f %2.f rot: %.2f %.2f %.2f lin_vel: %.2f %.2f %.2f ang_vel: %.2f %.2f %.2f",
                      propState_.pose().translation().x(), propState_.pose().translation().y(), propState_.pose().translation().z(),
                      propState_.pose().rotation().roll(), propState_.pose().rotation().pitch(), propState_.pose().rotation().yaw(),
                      propState_.linVelocity()[0], propState_.linVelocity()[1], propState_.linVelocity()[2], propState_.angVelocity()[0],
                      propState_.angVelocity()[1], propState_.angVelocity()[2]);
    ROS_INFO_THROTTLE(1.0, "[MotorSpeedPreintegration]: lin_acc_bias: %.2f %.2f %.2f ang_acc_bias: %.2f %.2f %.2f", prevBias_.linAcc()[0],
                      prevBias_.linAcc()[1], prevBias_.linAcc()[2], prevBias_.angAcc()[0], prevBias_.angAcc()[1], prevBias_.angAcc()[2]);

    // optimize
    /* cout << "****************************************************" << endl; */
    /* graphFactors.print("[MotorSpeedPreintegration]: graph\n"); */
    optimizer.update(graphFactors, graphValues);
    optimizer.update();
    graphFactors.resize(0);
    graphValues.clear();
    // Overwrite the beginning of the preintegration for the next step.
    const gtsam::Values result = optimizer.calculateEstimate();
    prevPose_                  = result.at<gtsam::Pose3>(X(key));
    prevLinVel_                = result.at<gtsam::Vector3>(V(key));
    prevAngVel_                = result.at<gtsam::Vector3>(W(key));
    prevState_                 = gtsam::FullState(prevPose_, prevLinVel_, prevAngVel_);
    prevBias_                  = result.at<gtsam::motor_speed_bias::ConstantBias>(B(key));
    // Reset the optimization preintegration object.
    motorSpeedIntegratorOpt_->resetIntegration();
    // check optimization
    if (failureDetection(prevLinVel_, prevBias_)) {
      resetParams();
      return;
    }

    ros::Time t_optimization = ros::Time::now();

    // 2. after optiization, re-propagate imu odometry preintegration
    prevStateOdom = prevState_;
    prevBiasOdom  = prevBias_;
    // first pop imu message older than current correction data
    double lastMotorSpeedQT = -1;
    while (!motorSpeedQueImu.empty() && ROS_TIME(&motorSpeedQueImu.front()) < currentCorrectionTime - delta_t) {
      lastMotorSpeedQT = ROS_TIME(&motorSpeedQueImu.front());
      motorSpeedQueImu.pop_front();
    }
    // repropogate
    if (!motorSpeedQueImu.empty()) {
      // reset bias use the newly optimized bias
      motorSpeedIntegratorPredict_->resetIntegration();
      // integrate imu message from the beginning of this optimization
      ros::Time ms_stamp;
      for (int i = 0; i < (int)motorSpeedQueImu.size(); ++i) {
        mrs_msgs::Float64ArrayStamped* thisMotorSpeeds = &motorSpeedQueImu[i];
        ms_stamp = thisMotorSpeeds->header.stamp;
        const double                   imuTime         = ROS_TIME(thisMotorSpeeds);
        const double                   dt              = (lastMotorSpeedQT < 0) ? (1.0 / 500.0) : (imuTime - lastMotorSpeedQT);

        if (dt <= 0) {
          ROS_WARN_COND(dt < 0, "[MotorSpeedPreintegration]: invalid dt (QT): (%0.2f - %0.2f) = %0.2f", imuTime, lastMotorSpeedQT, dt);
          continue;
        }

        motorSpeedIntegratorPredict_->integrateMeasurement(
            gtsam::Vector4(thisMotorSpeeds->values[0], thisMotorSpeeds->values[1], thisMotorSpeeds->values[2], thisMotorSpeeds->values[3]), dt);
        lastMotorSpeedQT = imuTime;
      }

      /* ROS_INFO("[MotorSpeedPreintegration]: motor speed integration delay: %.4f", (ros::Time::now() - ms_stamp).toSec()); */

        const gtsam::Vector3          lin_acc_b = motorSpeedIntegratorPredict_->lastAcc();
        const gtsam::Vector3          ang_acc_b = motorSpeedIntegratorPredict_->lastAlpha();
        geometry_msgs::Vector3Stamped lin_acc_msg;
        gtsam::Vector3                lin_acc_w = prevPose_.rotation() * lin_acc_b;
        lin_acc_msg.header.stamp                = ros::Time::now();
        lin_acc_msg.header.frame_id             = "fcu";
        lin_acc_msg.vector.x                    = lin_acc_w[0];
        lin_acc_msg.vector.y                    = lin_acc_w[1];
        lin_acc_msg.vector.z                    = lin_acc_w[2];
        pubLinAcc.publish(lin_acc_msg);

        geometry_msgs::Vector3Stamped ang_acc_msg;
        ang_acc_msg.header.stamp    = ros::Time::now();
        ang_acc_msg.header.frame_id = "fcu";
        ang_acc_msg.vector.x        = ang_acc_b[0];
        ang_acc_msg.vector.y        = ang_acc_b[1];
        ang_acc_msg.vector.z        = ang_acc_b[2];
        pubAngAcc.publish(ang_acc_msg);

    }

    geometry_msgs::Vector3Stamped lin_acc_bias_msg;
    lin_acc_bias_msg.header.stamp    = ros::Time::now();
    lin_acc_bias_msg.header.frame_id = "fcu";
    lin_acc_bias_msg.vector.x        = prevBias_.linAcc()[0];
    lin_acc_bias_msg.vector.y        = prevBias_.linAcc()[1];
    lin_acc_bias_msg.vector.z        = prevBias_.linAcc()[2];
    pubLinAccBias.publish(lin_acc_bias_msg);

    geometry_msgs::Vector3Stamped ang_acc_bias_msg;
    ang_acc_bias_msg.header.stamp    = ros::Time::now();
    ang_acc_bias_msg.header.frame_id = "fcu";
    ang_acc_bias_msg.vector.x        = prevBias_.angAcc()[0];
    ang_acc_bias_msg.vector.y        = prevBias_.angAcc()[1];
    ang_acc_bias_msg.vector.z        = prevBias_.angAcc()[2];
    pubAngAccBias.publish(ang_acc_bias_msg);

    ++key;
    doneFirstOpt = true;
  }
  /*//}*/

  /*//{ failureDetection() */
  bool failureDetection(const gtsam::Vector3& velCur, const gtsam::motor_speed_bias::ConstantBias& biasCur) {
    const Eigen::Vector3f vel(velCur.x(), velCur.y(), velCur.z());
    if (vel.norm() > 30) {
      ROS_WARN("[MotorSpeedPreintegration]: Large velocity (%0.1f, %0.1f, %0.1f), reset IMU-preintegration!", vel.x(), vel.y(), vel.z());
      return true;
    }

    const Eigen::Vector3f bla(biasCur.linAcc().x(), biasCur.linAcc().y(), biasCur.linAcc().z());
    const Eigen::Vector3f baa(biasCur.angAcc().x(), biasCur.angAcc().y(), biasCur.angAcc().z());
    /* if (bla.norm() > 1.0 || baa.norm() > 1.0) { */
    /*   ROS_WARN("[MotorSpeedPreintegration]: Large bias, reset IMU-preintegration!"); */
    /*   return true; */
    /* } */

    return false;
  }
  /*//}*/

  /*//{ motorSpeedHandler() */
  void motorSpeedHandler(const mrs_msgs::Float64ArrayStamped::ConstPtr& msg_in) {

    if (!is_initialized_) {
      return;
    }

    ROS_INFO_ONCE("[MotorSpeedPreintegration]: motorSpeedHandler first callback");
    std::lock_guard<std::mutex> lock(mtx);

    motorSpeedQueOpt.push_back(*msg_in);
    motorSpeedQueImu.push_back(*msg_in);

    if (doneFirstOpt == false) {
      ROS_INFO_THROTTLE(1.0, "[MotorSpeedPreintegration]: waiting for first optimalization");
      return;
    }

    const double imuTime = ROS_TIME(msg_in);
    const double dt      = (lastMotorSpeedT_predict < 0) ? (1.0 / 500.0) : (imuTime - lastMotorSpeedT_predict);
    if (dt <= 0) {
      ROS_WARN_COND(dt < 0, "[MotorSpeedPreintegration]: invalid dt (imu): (%0.2f - %0.2f) = %0.2f", imuTime, lastMotorSpeedT_predict, dt);
      return;
    }
    lastMotorSpeedT_predict = imuTime;

    // integrate this single imu message
    motorSpeedIntegratorPredict_->integrateMeasurement(gtsam::Vector4(msg_in->values[0], msg_in->values[1], msg_in->values[2], msg_in->values[3]), dt);

    // predict odometry
    const gtsam::FullState currentState = motorSpeedIntegratorPredict_->predict(prevStateOdom, prevBias_);

    // publish odometry
    nav_msgs::Odometry::Ptr odometry = boost::make_shared<nav_msgs::Odometry>();
    odometry->header.stamp           = msg_in->header.stamp;
    odometry->header.frame_id        = odometryFrame;
    odometry->child_frame_id         = baselinkFrame;

    // transform imu pose to ldiar
    const gtsam::Pose3 imuPose   = gtsam::Pose3(currentState.quaternion(), currentState.position());
    const gtsam::Pose3 lidarPose = imuPose.compose(imu2Lidar);

    odometry->pose.pose.position.x    = lidarPose.translation().x();
    odometry->pose.pose.position.y    = lidarPose.translation().y();
    odometry->pose.pose.position.z    = lidarPose.translation().z();
    odometry->pose.pose.orientation.x = lidarPose.rotation().toQuaternion().x();
    odometry->pose.pose.orientation.y = lidarPose.rotation().toQuaternion().y();
    odometry->pose.pose.orientation.z = lidarPose.rotation().toQuaternion().z();
    odometry->pose.pose.orientation.w = lidarPose.rotation().toQuaternion().w();

    odometry->twist.twist.linear.x  = currentState.linVelocity().x();
    odometry->twist.twist.linear.y  = currentState.linVelocity().y();
    odometry->twist.twist.linear.z  = currentState.linVelocity().z();
    odometry->twist.twist.angular.x = currentState.angVelocity().x();
    odometry->twist.twist.angular.y = currentState.angVelocity().y();
    odometry->twist.twist.angular.z = currentState.angVelocity().z();
    pubImuOdometry.publish(odometry);
  }
  /*//}*/
};
/*//}*/

}  // namespace motor_speed_preintegration
}  // namespace liosam

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(liosam::motor_speed_preintegration::MotorSpeedPreintegration, nodelet::Nodelet)