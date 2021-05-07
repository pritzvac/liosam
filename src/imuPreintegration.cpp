#include "utility.h"

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

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

using gtsam::symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

namespace liosam
{
namespace imu_preintegration
{

/*//{ class TransformFusion() */
class TransformFusion : public ParamServer {
public:
  std::mutex mtx;

  ros::Subscriber subImuOdometry;
  ros::Subscriber subLaserOdometry;

  ros::Publisher pubImuOdometry;
  ros::Publisher pubImuPath;

  Eigen::Affine3f lidarOdomAffine;
  Eigen::Affine3f imuOdomAffineFront;
  Eigen::Affine3f imuOdomAffineBack;


  double                    lidarOdomTime = -1;
  deque<nav_msgs::Odometry> imuOdomQueue;

public:
  /*//{ TransformFusion() */
  TransformFusion(ros::NodeHandle& nh_) {

    subLaserOdometry =
        nh_.subscribe<nav_msgs::Odometry>("liosam/mapping/odometry", 5, &TransformFusion::lidarOdometryHandler, this, ros::TransportHints().tcpNoDelay());
    subImuOdometry =
        nh_.subscribe<nav_msgs::Odometry>(odomTopic + "_incremental", 2000, &TransformFusion::imuOdometryHandler, this, ros::TransportHints().tcpNoDelay());

    pubImuOdometry = nh_.advertise<nav_msgs::Odometry>(odomTopic, 2000);
    /* ROS_WARN("pubImuOdometry topic: %s", pubImuOdometry.getTopic().c_str()); */
    pubImuPath = nh_.advertise<nav_msgs::Path>("liosam/imu/path", 1);
  }
  /*//}*/

  /*//{ odom2affine() */
  Eigen::Affine3f odom2affine(nav_msgs::Odometry odom) {
    double x, y, z, roll, pitch, yaw;
    x = odom.pose.pose.position.x;
    y = odom.pose.pose.position.y;
    z = odom.pose.pose.position.z;
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(odom.pose.pose.orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
    return pcl::getTransformation(x, y, z, roll, pitch, yaw);
  }
  /*//}*/

  /*//{ lidarOdometryHandler() */
  void lidarOdometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg) {
    std::lock_guard<std::mutex> lock(mtx);

    lidarOdomAffine = odom2affine(*odomMsg);

    lidarOdomTime = odomMsg->header.stamp.toSec();
  }
  /*//}*/

  /*//{ imuOdometryHandler() */
  void imuOdometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg) {
    // TODO: publish TFs correctly as `map -> odom -> fcu` (control has to handle this by continuously republishing the reference in map frame)

    // publish tf map->odom (inverted tf-tree)
    static tf::TransformBroadcaster tfMap2Odom;
    static tf::Transform            map_to_odom = tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(0, 0, 0));
    tfMap2Odom.sendTransform(tf::StampedTransform(map_to_odom.inverse(), odomMsg->header.stamp, odometryFrame, mapFrame));

    std::lock_guard<std::mutex> lock(mtx);

    imuOdomQueue.push_back(*odomMsg);

    // get latest odometry (at current IMU stamp)
    if (lidarOdomTime == -1) {
      return;
    }
    while (!imuOdomQueue.empty()) {
      if (imuOdomQueue.front().header.stamp.toSec() <= lidarOdomTime) {
        imuOdomQueue.pop_front();
      } else {
        break;
      }
    }
    const Eigen::Affine3f imuOdomAffineFront = odom2affine(imuOdomQueue.front());
    const Eigen::Affine3f imuOdomAffineBack  = odom2affine(imuOdomQueue.back());
    const Eigen::Affine3f imuOdomAffineIncre = imuOdomAffineFront.inverse() * imuOdomAffineBack;
    const Eigen::Affine3f imuOdomAffineLast  = lidarOdomAffine * imuOdomAffineIncre;
    float                 x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(imuOdomAffineLast, x, y, z, roll, pitch, yaw);

    // publish latest odometry
    tf::Transform tCur;
    tCur.setOrigin(tf::Vector3(x, y, z));
    tCur.setRotation(tf::createQuaternionFromRPY(roll, pitch, yaw));
    if (lidarFrame != baselinkFrame) {
      tCur = tCur * tfLidar2Baselink;
    }
    tCur.setRotation(tCur.getRotation().normalized());

    nav_msgs::Odometry::Ptr laserOdometry = boost::make_shared<nav_msgs::Odometry>(imuOdomQueue.back());
    laserOdometry->header.frame_id        = odometryFrame;
    laserOdometry->child_frame_id         = baselinkFrame;
    laserOdometry->pose.pose.position.x   = tCur.getOrigin().getX();
    laserOdometry->pose.pose.position.y   = tCur.getOrigin().getY();
    laserOdometry->pose.pose.position.z   = tCur.getOrigin().getZ();
    tf::quaternionTFToMsg(tCur.getRotation(), laserOdometry->pose.pose.orientation);

    if (std::isfinite(laserOdometry->pose.pose.orientation.x) && std::isfinite(laserOdometry->pose.pose.orientation.y) &&
        std::isfinite(laserOdometry->pose.pose.orientation.z) && std::isfinite(laserOdometry->pose.pose.orientation.w)) {
      pubImuOdometry.publish(laserOdometry);

      // publish tf odom->fcu (inverted tf-tree)
      static tf::TransformBroadcaster tfOdom2BaseLink;
      tf::StampedTransform            odom_2_baselink = tf::StampedTransform(tCur.inverse(), odomMsg->header.stamp, baselinkFrame, odometryFrame);
      tfOdom2BaseLink.sendTransform(odom_2_baselink);

      // publish IMU path
      static nav_msgs::Path::Ptr imuPath        = boost::make_shared<nav_msgs::Path>();
      static double              last_path_time = -1;
      const double               imuTime        = imuOdomQueue.back().header.stamp.toSec();
      if (imuTime - last_path_time > 0.1) {
        last_path_time = imuTime;
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp    = imuOdomQueue.back().header.stamp;
        pose_stamped.header.frame_id = odometryFrame;
        pose_stamped.pose            = laserOdometry->pose.pose;
        imuPath->poses.push_back(pose_stamped);
        while (!imuPath->poses.empty() && imuPath->poses.front().header.stamp.toSec() < lidarOdomTime - 1.0) {
          imuPath->poses.erase(imuPath->poses.begin());
        }
        if (pubImuPath.getNumSubscribers() > 0) {
          imuPath->header.stamp    = imuOdomQueue.back().header.stamp;
          imuPath->header.frame_id = odometryFrame;
          pubImuPath.publish(imuPath);
        }
      }
    }
  }
  /*//}*/
};
/*//}*/

/*//{ class ImuPreintegrationImpl() */
class ImuPreintegrationImpl : public ParamServer {
public:
  std::mutex mtx;

  ros::Subscriber subImu;
  ros::Subscriber subOdometry;
  ros::Publisher  pubImuOdometry;

  bool systemInitialized = false;

  gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
  gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise;
  gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise;
  gtsam::noiseModel::Diagonal::shared_ptr correctionNoise;
  gtsam::noiseModel::Diagonal::shared_ptr correctionNoise2;
  gtsam::Vector                           noiseModelBetweenBias;


  gtsam::PreintegratedImuMeasurements* imuIntegratorOpt_;
  gtsam::PreintegratedImuMeasurements* imuIntegratorImu_;

  std::deque<sensor_msgs::Imu> imuQueOpt;
  std::deque<sensor_msgs::Imu> imuQueImu;

  gtsam::Pose3                 prevPose_;
  gtsam::Vector3               prevVel_;
  gtsam::NavState              prevState_;
  gtsam::imuBias::ConstantBias prevBias_;

  gtsam::NavState              prevStateOdom;
  gtsam::imuBias::ConstantBias prevBiasOdom;

  bool   doneFirstOpt = false;
  double lastImuT_imu = -1;
  double lastImuT_opt = -1;

  gtsam::ISAM2                optimizer;
  gtsam::NonlinearFactorGraph graphFactors;
  gtsam::Values               graphValues;

  const double delta_t = 0;

  int key = 1;

  const gtsam::Pose3 imu2Lidar =
      gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(-tfLidar2Imu.getOrigin().x(), -tfLidar2Imu.getOrigin().y(), -tfLidar2Imu.getOrigin().z()));
  const gtsam::Pose3 lidar2Imu =
      gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(tfLidar2Imu.getOrigin().x(), tfLidar2Imu.getOrigin().y(), tfLidar2Imu.getOrigin().z()));

public:
  /*//{ ImuPreintegrationImpl() */
  ImuPreintegrationImpl(ros::NodeHandle& nh_) {
    subImu      = nh_.subscribe<sensor_msgs::Imu>(imuTopic, 2000, &ImuPreintegrationImpl::imuHandler, this, ros::TransportHints().tcpNoDelay());
    subOdometry = nh_.subscribe<nav_msgs::Odometry>("liosam/mapping/odometry_incremental", 5, &ImuPreintegrationImpl::odometryHandler, this,
                                                    ros::TransportHints().tcpNoDelay());

    pubImuOdometry = nh_.advertise<nav_msgs::Odometry>(odomTopic + "_incremental", 2000);

    boost::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(imuGravity);
    p->accelerometerCovariance                       = gtsam::Matrix33::Identity(3, 3) * pow(imuAccNoise, 2);  // acc white noise in continuous
    p->gyroscopeCovariance                           = gtsam::Matrix33::Identity(3, 3) * pow(imuGyrNoise, 2);  // gyro white noise in continuous
    p->integrationCovariance = gtsam::Matrix33::Identity(3, 3) * pow(1e-4, 2);                       // error committed in integrating position from velocities
    gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());  // assume zero initial bias

    priorPoseNoise        = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished());  // rad,rad,rad,m, m, m
    priorVelNoise         = gtsam::noiseModel::Isotropic::Sigma(3, 1e4);                                                               // m/s
    priorBiasNoise        = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3);  // 1e-2 ~ 1e-3 seems to be good
    correctionNoise       = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished());  // rad,rad,rad,m, m, m
    correctionNoise2      = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1, 1, 1, 1, 1, 1).finished());                 // rad,rad,rad,m, m, m
    noiseModelBetweenBias = (gtsam::Vector(6) << imuAccBiasN, imuAccBiasN, imuAccBiasN, imuGyrBiasN, imuGyrBiasN, imuGyrBiasN).finished();

    imuIntegratorImu_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias);  // setting up the IMU integration for IMU message thread
    imuIntegratorOpt_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias);  // setting up the IMU integration for optimization
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
    lastImuT_imu      = -1;
    doneFirstOpt      = false;
    systemInitialized = false;
  }
  /*//}*/

  /*//{ odometryHandler() */
  void odometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg) {
    std::lock_guard<std::mutex> lock(mtx);

    const double currentCorrectionTime = ROS_TIME(odomMsg);

    // make sure we have imu data to integrate
    if (imuQueOpt.empty()) {
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
      while (!imuQueOpt.empty()) {
        if (ROS_TIME(&imuQueOpt.front()) < currentCorrectionTime - delta_t) {
          lastImuT_opt = ROS_TIME(&imuQueOpt.front());
          imuQueOpt.pop_front();
        } else {
          break;
        }
      }
      // initial pose
      prevPose_ = lidarPose.compose(lidar2Imu);
      const gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, priorPoseNoise);
      graphFactors.add(priorPose);
      // initial velocity
      prevVel_ = gtsam::Vector3(0, 0, 0);
      const gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, priorVelNoise);
      graphFactors.add(priorVel);
      // initial bias
      prevBias_ = gtsam::imuBias::ConstantBias();
      const gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, priorBiasNoise);
      graphFactors.add(priorBias);
      // add values
      graphValues.insert(X(0), prevPose_);
      graphValues.insert(V(0), prevVel_);
      graphValues.insert(B(0), prevBias_);
      // optimize once
      optimizer.update(graphFactors, graphValues);
      graphFactors.resize(0);
      graphValues.clear();

      imuIntegratorImu_->resetIntegrationAndSetBias(prevBias_);
      imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);

      key               = 1;
      systemInitialized = true;
      return;
    }


    // reset graph for speed
    if (key == 100) {
      // get updated noise before reset
      const gtsam::noiseModel::Gaussian::shared_ptr updatedPoseNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(X(key - 1)));
      const gtsam::noiseModel::Gaussian::shared_ptr updatedVelNoise  = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(V(key - 1)));
      const gtsam::noiseModel::Gaussian::shared_ptr updatedBiasNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(B(key - 1)));
      // reset graph
      resetOptimization();
      // add pose
      const gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, updatedPoseNoise);
      graphFactors.add(priorPose);
      // add velocity
      const gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, updatedVelNoise);
      graphFactors.add(priorVel);
      // add bias
      const gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, updatedBiasNoise);
      graphFactors.add(priorBias);
      // add values
      graphValues.insert(X(0), prevPose_);
      graphValues.insert(V(0), prevVel_);
      graphValues.insert(B(0), prevBias_);
      // optimize once
      optimizer.update(graphFactors, graphValues);
      graphFactors.resize(0);
      graphValues.clear();

      key = 1;
    }


    // 1. integrate imu data and optimize
    while (!imuQueOpt.empty()) {
      // pop and integrate imu data that is between two optimizations
      const sensor_msgs::Imu* thisImu = &imuQueOpt.front();
      const double            imuTime = ROS_TIME(thisImu);
      if (imuTime < currentCorrectionTime - delta_t) {
        const double dt = (lastImuT_opt < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_opt);

        if (dt <= 0) {
          ROS_WARN_COND(dt < 0, "invalid dt (opt): (%0.2f - %0.2f) = %0.2f", imuTime, lastImuT_opt, dt);
          imuQueOpt.pop_front();
          continue;
        }

        imuIntegratorOpt_->integrateMeasurement(gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                                                gtsam::Vector3(thisImu->angular_velocity.x, thisImu->angular_velocity.y, thisImu->angular_velocity.z), dt);

        lastImuT_opt = imuTime;
        imuQueOpt.pop_front();
      } else {
        break;
      }
    }
    // add imu factor to graph
    const gtsam::PreintegratedImuMeasurements& preint_imu = dynamic_cast<const gtsam::PreintegratedImuMeasurements&>(*imuIntegratorOpt_);
    const gtsam::ImuFactor                     imu_factor(X(key - 1), V(key - 1), X(key), V(key), B(key - 1), preint_imu);
    graphFactors.add(imu_factor);
    // add imu bias between factor
    graphFactors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(
        B(key - 1), B(key), gtsam::imuBias::ConstantBias(), gtsam::noiseModel::Diagonal::Sigmas(sqrt(imuIntegratorOpt_->deltaTij()) * noiseModelBetweenBias)));
    // add pose factor
    const gtsam::Pose3                     curPose = lidarPose.compose(lidar2Imu);
    const gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key), curPose, degenerate ? correctionNoise2 : correctionNoise);
    graphFactors.add(pose_factor);
    // insert predicted values
    const gtsam::NavState propState_ = imuIntegratorOpt_->predict(prevState_, prevBias_);
    graphValues.insert(X(key), propState_.pose());
    graphValues.insert(V(key), propState_.v());
    graphValues.insert(B(key), prevBias_);
    // optimize
    optimizer.update(graphFactors, graphValues);
    optimizer.update();
    graphFactors.resize(0);
    graphValues.clear();
    // Overwrite the beginning of the preintegration for the next step.
    const gtsam::Values result = optimizer.calculateEstimate();
    prevPose_                  = result.at<gtsam::Pose3>(X(key));
    prevVel_                   = result.at<gtsam::Vector3>(V(key));
    prevState_                 = gtsam::NavState(prevPose_, prevVel_);
    prevBias_                  = result.at<gtsam::imuBias::ConstantBias>(B(key));
    // Reset the optimization preintegration object.
    imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);
    // check optimization
    if (failureDetection(prevVel_, prevBias_)) {
      resetParams();
      return;
    }


    // 2. after optiization, re-propagate imu odometry preintegration
    prevStateOdom = prevState_;
    prevBiasOdom  = prevBias_;
    // first pop imu message older than current correction data
    double lastImuQT = -1;
    while (!imuQueImu.empty() && ROS_TIME(&imuQueImu.front()) < currentCorrectionTime - delta_t) {
      lastImuQT = ROS_TIME(&imuQueImu.front());
      imuQueImu.pop_front();
    }
    // repropogate
    if (!imuQueImu.empty()) {
      // reset bias use the newly optimized bias
      imuIntegratorImu_->resetIntegrationAndSetBias(prevBiasOdom);
      // integrate imu message from the beginning of this optimization
      for (int i = 0; i < (int)imuQueImu.size(); ++i) {
        sensor_msgs::Imu* thisImu = &imuQueImu[i];
        const double      imuTime = ROS_TIME(thisImu);
        const double      dt      = (lastImuQT < 0) ? (1.0 / 500.0) : (imuTime - lastImuQT);

        if (dt <= 0) {
          ROS_WARN_COND(dt < 0, "invalid dt (QT): (%0.2f - %0.2f) = %0.2f", imuTime, lastImuQT, dt);
          continue;
        }

        imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                                                gtsam::Vector3(thisImu->angular_velocity.x, thisImu->angular_velocity.y, thisImu->angular_velocity.z), dt);
        lastImuQT = imuTime;
      }
    }

    ++key;
    doneFirstOpt = true;
  }
  /*//}*/

  /*//{ failureDetection() */
  bool failureDetection(const gtsam::Vector3& velCur, const gtsam::imuBias::ConstantBias& biasCur) {
    const Eigen::Vector3f vel(velCur.x(), velCur.y(), velCur.z());
    if (vel.norm() > 30) {
      ROS_WARN("Large velocity (%0.1f, %0.1f, %0.1f), reset IMU-preintegration!", vel.x(), vel.y(), vel.z());
      return true;
    }

    const Eigen::Vector3f ba(biasCur.accelerometer().x(), biasCur.accelerometer().y(), biasCur.accelerometer().z());
    const Eigen::Vector3f bg(biasCur.gyroscope().x(), biasCur.gyroscope().y(), biasCur.gyroscope().z());
    if (ba.norm() > 1.0 || bg.norm() > 1.0) {
      ROS_WARN("Large bias, reset IMU-preintegration!");
      return true;
    }

    return false;
  }
  /*//}*/

  /*//{ imuHandler() */
  void imuHandler(const sensor_msgs::Imu::ConstPtr& imu_raw) {
    std::lock_guard<std::mutex> lock(mtx);

    const sensor_msgs::Imu thisImu = imuConverter(*imu_raw);

    imuQueOpt.push_back(thisImu);
    imuQueImu.push_back(thisImu);

    if (doneFirstOpt == false) {
      return;
    }

    const double imuTime = ROS_TIME(&thisImu);
    const double dt      = (lastImuT_imu < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_imu);
    if (dt <= 0) {
      ROS_WARN_COND(dt < 0, "invalid dt (imu): (%0.2f - %0.2f) = %0.2f", imuTime, lastImuT_imu, dt);
      return;
    }
    lastImuT_imu = imuTime;

    // integrate this single imu message
    imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu.linear_acceleration.x, thisImu.linear_acceleration.y, thisImu.linear_acceleration.z),
                                            gtsam::Vector3(thisImu.angular_velocity.x, thisImu.angular_velocity.y, thisImu.angular_velocity.z), dt);

    // predict odometry
    const gtsam::NavState currentState = imuIntegratorImu_->predict(prevStateOdom, prevBiasOdom);

    // publish odometry
    nav_msgs::Odometry::Ptr odometry = boost::make_shared<nav_msgs::Odometry>();
    odometry->header.stamp           = thisImu.header.stamp;
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

    odometry->twist.twist.linear.x  = currentState.velocity().x();
    odometry->twist.twist.linear.y  = currentState.velocity().y();
    odometry->twist.twist.linear.z  = currentState.velocity().z();
    odometry->twist.twist.angular.x = thisImu.angular_velocity.x + prevBiasOdom.gyroscope().x();
    odometry->twist.twist.angular.y = thisImu.angular_velocity.y + prevBiasOdom.gyroscope().y();
    odometry->twist.twist.angular.z = thisImu.angular_velocity.z + prevBiasOdom.gyroscope().z();
    pubImuOdometry.publish(odometry);
  }
  /*//}*/
};
/*//}*/

/* //{ class ImuPreintegration */

class ImuPreintegration : public nodelet::Nodelet {

public:
  virtual void onInit() {
    ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();
    IP                  = std::make_unique<ImuPreintegrationImpl>(nh_);
    TF                  = std::make_unique<TransformFusion>(nh_);
    ROS_INFO("\033[1;32m----> IMU Preintegration Started.\033[0m");
  };

private:
  std::shared_ptr<ImuPreintegrationImpl> IP;
  std::shared_ptr<TransformFusion>       TF;
};

//}

}  // namespace imu_preintegration
}  // namespace liosam

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(liosam::imu_preintegration::ImuPreintegration, nodelet::Nodelet)
