#pragma once

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <opencv2/core/core.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

#include <mrs_lib/param_loader.h>

#include <mrs_msgs/Float64ArrayStamped.h>

using namespace std;

typedef pcl::PointXYZI PointType;

/*//{ class ParamServer() */
class ParamServer {
public:
  ros::NodeHandle nh;

  std::string uav_name;

  // Topics
  string pointCloudTopic;
  string imuTopic;
  string motorSpeedTopic;
  string odomTopic;
  string gpsTopic;

  // Frames
  std::string lidarFrame;
  std::string imuFrame;
  std::string baselinkFrame;
  std::string odometryFrame;
  std::string mapFrame;

  // TF
  tf::TransformListener tfListener;
  tf::StampedTransform  tfLidar2Baselink;
  tf::StampedTransform  tfLidar2Imu;

  // GPS Settings
  bool  useImuHeadingInitialization;
  bool  useGpsElevation;
  float gpsCovThreshold;
  float poseCovThreshold;

  // Save pcd
  bool   savePCD;
  string savePCDDirectory;

  // Velodyne Sensor Configuration: Velodyne
  int    N_SCAN;
  int    Horizon_SCAN;
  string timeField;
  int    downsampleRate;
  float  lidarMinRange;
  float  lidarMaxRange;

  // IMU
  string             imuType;
  float              imuAccBiasN;
  float              imuGyrBiasN;
  float              imuGravity;
  float              imuRPYWeight;
  Eigen::Matrix3d    extRot;
  Eigen::Quaterniond extQRPY;

  float              linAccNoise;
  float              angAccNoise;

  // Motor Speeds
  float mass;
  int numMotors;
  float propMass;
  float motorConstant;
  float momentConstant;

  // LOAM
  float edgeThreshold;
  float surfThreshold;
  int   edgeFeatureMinValidNum;
  int   surfFeatureMinValidNum;

  // voxel filter paprams
  float odometrySurfLeafSize;
  float mappingCornerLeafSize;
  float mappingSurfLeafSize;

  float z_tollerance;
  float rotation_tollerance;

  // CPU Params
  int    numberOfCores;
  double mappingProcessInterval;

  // Surrounding map
  float surroundingkeyframeAddingDistThreshold;
  float surroundingkeyframeAddingAngleThreshold;
  float surroundingKeyframeDensity;
  float surroundingKeyframeSearchRadius;

  // Loop closure
  bool  loopClosureEnableFlag;
  float loopClosureFrequency;
  int   surroundingKeyframeSize;
  float historyKeyframeSearchRadius;
  float historyKeyframeSearchTimeDiff;
  int   historyKeyframeSearchNum;
  float historyKeyframeFitnessScore;

  // global map visualization radius
  float globalMapVisualizationSearchRadius;
  float globalMapVisualizationPoseDensity;
  float globalMapVisualizationLeafSize;

  ParamServer() {

    mrs_lib::ParamLoader pl(nh, ros::this_node::getName());

    pl.loadParam("uav_name", uav_name);
    /* ROS_INFO("[%s]: loaded uav_name: %s", ros::this_node::getName().c_str(), uav_name.c_str()); */

    pl.loadParam("liosam/imuType", imuType);
    ROS_INFO("[%s]: loaded imuType: %s", ros::this_node::getName().c_str(), imuType.c_str());

    pl.loadParam("liosam/pointCloudTopic", pointCloudTopic);
    addNamespace("/" + uav_name, pointCloudTopic);

    /* ROS_INFO("[%s]: loading imu_topic: %s", ros::this_node::getName().c_str(), std::string("liosam/" + imuType + "/imuTopic").c_str()); */
    pl.loadParam("liosam/" + imuType + "/imuTopic", imuTopic);
    addNamespace("/" + uav_name, imuTopic);
    ROS_INFO("[%s]: loaded imu_topic: %s", ros::this_node::getName().c_str(), imuTopic.c_str());

    pl.loadParam("liosam/motorSpeedTopic", motorSpeedTopic);
    addNamespace("/" + uav_name, motorSpeedTopic);
    ROS_INFO("[%s]: loaded motor_speed_topic: %s", ros::this_node::getName().c_str(), motorSpeedTopic.c_str());

    pl.loadParam("liosam/odomTopic", odomTopic);
    addNamespace("/" + uav_name, odomTopic);

    pl.loadParam("liosam/gpsTopic", gpsTopic, std::string("odometry/gps"));
    addNamespace("/" + uav_name, gpsTopic);

    pl.loadParam("liosam/lidarFrame", lidarFrame, std::string("base_link"));
    pl.loadParam("liosam/" + imuType + "/frame", imuFrame);
    pl.loadParam("liosam/baselinkFrame", baselinkFrame, std::string("base_link"));
    pl.loadParam("liosam/odometryFrame", odometryFrame, std::string("odom"));
    pl.loadParam("liosam/mapFrame", mapFrame, std::string("map"));
    addNamespace(uav_name, lidarFrame);
    addNamespace(uav_name, imuFrame);
    addNamespace(uav_name, baselinkFrame);
    addNamespace(uav_name, odometryFrame);
    addNamespace(uav_name, mapFrame);

    pl.loadParam("liosam/useImuHeadingInitialization", useImuHeadingInitialization, false);
    pl.loadParam("liosam/useGpsElevation", useGpsElevation, false);
    pl.loadParam("liosam/gpsCovThreshold", gpsCovThreshold, 2.0f);
    pl.loadParam("liosam/poseCovThreshold", poseCovThreshold, 25.0f);

    pl.loadParam("liosam/savePCD", savePCD, false);
    pl.loadParam("liosam/savePCDDirectory", savePCDDirectory, std::string("/Downloads/LOAM/"));

    pl.loadParam("liosam/N_SCAN", N_SCAN);
    pl.loadParam("liosam/Horizon_SCAN", Horizon_SCAN);
    pl.loadParam("liosam/timeField", timeField, std::string("t"));
    pl.loadParam("liosam/downsampleRate", downsampleRate, 1);
    pl.loadParam("liosam/lidarMinRange", lidarMinRange, 0.1f);
    pl.loadParam("liosam/lidarMaxRange", lidarMaxRange, 1000.0f);

    pl.loadParam("liosam/" + imuType + "/imuAccBiasN", imuAccBiasN, 0.0002f);
    pl.loadParam("liosam/" + imuType + "/imuGyrBiasN", imuGyrBiasN, 0.00003f);
    pl.loadParam("liosam/" + imuType + "/imuRPYWeight", imuRPYWeight, 0.01f);
    pl.loadParam("liosam/imuGravity", imuGravity, 9.80511f);
    /* pl.loadMatrixStatic("liosam/" + imuType + "/extrinsicRot", extRot, 3, 3); */

    pl.loadParam("liosam/linAccNoise", linAccNoise, 0.01f);
    pl.loadParam("liosam/angAccNoise", angAccNoise, 0.001f);

    pl.loadParam("liosam/mass", mass, 3.0f);
    pl.loadParam("liosam/numMotors", numMotors, 4);
    pl.loadParam("liosam/propMass", propMass, 0.005f);
    pl.loadParam("liosam/motorConstant", motorConstant, 10.0f);
    pl.loadParam("liosam/momentConstant", momentConstant, 0.01f);

    pl.loadParam("liosam/edgeThreshold", edgeThreshold, 0.1f);
    pl.loadParam("liosam/surfThreshold", surfThreshold, 0.1f);
    pl.loadParam("liosam/edgeFeatureMinValidNum", edgeFeatureMinValidNum, 10);
    pl.loadParam("liosam/surfFeatureMinValidNum", surfFeatureMinValidNum, 100);

    pl.loadParam("liosam/odometrySurfLeafSize", odometrySurfLeafSize, 0.2f);
    pl.loadParam("liosam/mappingCornerLeafSize", mappingCornerLeafSize, 0.2f);
    pl.loadParam("liosam/mappingSurfLeafSize", mappingSurfLeafSize, 0.2f);

    pl.loadParam("liosam/z_tollerance", z_tollerance, FLT_MAX);
    pl.loadParam("liosam/rotation_tollerance", rotation_tollerance, FLT_MAX);

    pl.loadParam("liosam/numberOfCores", numberOfCores, 4);
    pl.loadParam("liosam/mappingProcessInterval", mappingProcessInterval, 0.15);

    pl.loadParam("liosam/surroundingkeyframeAddingDistThreshold", surroundingkeyframeAddingDistThreshold, 1.0f);
    pl.loadParam("liosam/surroundingkeyframeAddingAngleThreshold", surroundingkeyframeAddingAngleThreshold, 0.2f);
    pl.loadParam("liosam/surroundingKeyframeDensity", surroundingKeyframeDensity, 1.0f);
    pl.loadParam("liosam/surroundingKeyframeSearchRadius", surroundingKeyframeSearchRadius, 50.0f);

    pl.loadParam("liosam/loopClosureEnableFlag", loopClosureEnableFlag, false);
    pl.loadParam("liosam/loopClosureFrequency", loopClosureFrequency, 1.0f);
    pl.loadParam("liosam/surroundingKeyframeSize", surroundingKeyframeSize, 50);
    pl.loadParam("liosam/historyKeyframeSearchRadius", historyKeyframeSearchRadius, 10.0f);
    pl.loadParam("liosam/historyKeyframeSearchTimeDiff", historyKeyframeSearchTimeDiff, 30.0f);
    pl.loadParam("liosam/historyKeyframeSearchNum", historyKeyframeSearchNum, 25);
    pl.loadParam("liosam/historyKeyframeFitnessScore", historyKeyframeFitnessScore, 0.3f);

    pl.loadParam("liosam/globalMapVisualizationSearchRadius", globalMapVisualizationSearchRadius, 1e3f);
    pl.loadParam("liosam/globalMapVisualizationPoseDensity", globalMapVisualizationPoseDensity, 10.0f);
    pl.loadParam("liosam/globalMapVisualizationLeafSize", globalMapVisualizationLeafSize, 1.0f);

    if (!pl.loadedSuccessfully()) {
      ROS_ERROR("[LIOSAM]: Could not load all parameters!");
      ros::shutdown();
    }

    /*//{ find lidar -> base_link transform */
    if (lidarFrame != baselinkFrame) {

      bool tf_found = false;
      while (!tf_found) {
        try {
          tfListener.waitForTransform(lidarFrame, baselinkFrame, ros::Time(0), ros::Duration(3.0));
          tfListener.lookupTransform(lidarFrame, baselinkFrame, ros::Time(0), tfLidar2Baselink);
          tf_found = true;
        }
        catch (tf::TransformException ex) {
          ROS_WARN_THROTTLE(3.0, "Waiting for transform from: %s, to: %s.", lidarFrame.c_str(), baselinkFrame.c_str());
        }
      }

      ROS_INFO("Found transform from: %s, to: %s.", lidarFrame.c_str(), baselinkFrame.c_str());

    } else {

      tfLidar2Baselink.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
      tfLidar2Baselink.setRotation(tf::createQuaternionFromRPY(0.0, 0.0, 0.0));
    }
    /*//}*/

    /*//{ find lidar -> imu transform */
    if (imuFrame == baselinkFrame) {

      tfLidar2Imu = tfLidar2Baselink;
      ROS_INFO("Found transform from: %s, to: %s.", lidarFrame.c_str(), imuFrame.c_str());

    } else if (imuFrame != lidarFrame) {

      bool tf_found = false;
      while (!tf_found) {
        try {
          tfListener.waitForTransform(lidarFrame, imuFrame, ros::Time(0), ros::Duration(3.0));
          tfListener.lookupTransform(lidarFrame, imuFrame, ros::Time(0), tfLidar2Imu);
          tf_found = true;
        }
        catch (tf::TransformException ex) {
          ROS_WARN_THROTTLE(3.0, "Waiting for transform from: %s, to: %s.", lidarFrame.c_str(), imuFrame.c_str());
        }
      }

      ROS_INFO("Found transform from: %s, to: %s.", lidarFrame.c_str(), imuFrame.c_str());

    } else {

      tfLidar2Imu.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
      tfLidar2Imu.setRotation(tf::createQuaternionFromRPY(0.0, 0.0, 0.0));
    }
    /*//}*/

    extQRPY = Eigen::Quaterniond(tfLidar2Imu.getRotation().w(), tfLidar2Imu.getRotation().x(), tfLidar2Imu.getRotation().y(), tfLidar2Imu.getRotation().z());
    extRot  = Eigen::Matrix3d(extQRPY);

    ROS_INFO("Transform from: %s, to: %s.", lidarFrame.c_str(), baselinkFrame.c_str());
    ROS_INFO("   xyz: (%0.1f, %0.1f, %0.1f); xyzq: (%0.1f, %0.1f, %0.1f, %0.1f)", tfLidar2Baselink.getOrigin().x(), tfLidar2Baselink.getOrigin().y(),
             tfLidar2Baselink.getOrigin().z(), tfLidar2Baselink.getRotation().x(), tfLidar2Baselink.getRotation().y(), tfLidar2Baselink.getRotation().z(),
             tfLidar2Baselink.getRotation().w());
    ROS_INFO("Transform from: %s, to: %s.", lidarFrame.c_str(), imuFrame.c_str());
    ROS_INFO("   xyz: (%0.1f, %0.1f, %0.1f); xyzq: (%0.1f, %0.1f, %0.1f, %0.1f)", tfLidar2Imu.getOrigin().x(), tfLidar2Imu.getOrigin().y(),
             tfLidar2Imu.getOrigin().z(), tfLidar2Imu.getRotation().x(), tfLidar2Imu.getRotation().y(), tfLidar2Imu.getRotation().z(),
             tfLidar2Imu.getRotation().w());

    ros::Duration(0.1).sleep();
  }

  sensor_msgs::Imu imuConverter(const sensor_msgs::Imu &imu_in) {
    sensor_msgs::Imu imu_out = imu_in;
    // rotate acceleration
    Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
    acc                           = extRot * acc;
    imu_out.linear_acceleration.x = acc.x();
    imu_out.linear_acceleration.y = acc.y();
    imu_out.linear_acceleration.z = acc.z();
    // rotate gyroscope
    Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
    gyr                        = extRot * gyr;
    imu_out.angular_velocity.x = gyr.x();
    imu_out.angular_velocity.y = gyr.y();
    imu_out.angular_velocity.z = gyr.z();
    // rotate roll pitch yaw
    Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);
    Eigen::Quaterniond q_final = q_from * extQRPY;
    imu_out.orientation.x      = q_final.x();
    imu_out.orientation.y      = q_final.y();
    imu_out.orientation.z      = q_final.z();
    imu_out.orientation.w      = q_final.w();

    if (sqrt(q_final.x() * q_final.x() + q_final.y() * q_final.y() + q_final.z() * q_final.z() + q_final.w() * q_final.w()) < 0.1) {
      ROS_ERROR("Invalid quaternion, please use a 9-axis IMU!");
      ros::shutdown();
    }

    return imu_out;
  }

  void addNamespace(const std::string &ns, std::string &s) {
    const bool add_ns = s.rfind("/") != 0 && s.rfind(ns) != 0;
    if (add_ns)
      s = ns + "/" + s;
  }
};
/*//}*/

sensor_msgs::PointCloud2 publishCloud(ros::Publisher *thisPub, pcl::PointCloud<PointType>::Ptr thisCloud, ros::Time thisStamp, std::string thisFrame) {

  sensor_msgs::PointCloud2::Ptr tempCloud = boost::make_shared<sensor_msgs::PointCloud2>();
  pcl::toROSMsg(*thisCloud, *tempCloud);
  tempCloud->header.stamp    = thisStamp;
  tempCloud->header.frame_id = thisFrame;

  if (thisPub->getNumSubscribers() > 0) {
    try {
      thisPub->publish(tempCloud);
    }
    catch (...) {
      ROS_ERROR("[LioSam]: Exception caught during publishing topic %s.", thisPub->getTopic().c_str());
    }
  }
  return *tempCloud;
}

template <typename T>
double ROS_TIME(T msg) {
  return msg->header.stamp.toSec();
}


template <typename T>
void imuAngular2rosAngular(sensor_msgs::Imu *thisImuMsg, T *angular_x, T *angular_y, T *angular_z) {
  *angular_x = thisImuMsg->angular_velocity.x;
  *angular_y = thisImuMsg->angular_velocity.y;
  *angular_z = thisImuMsg->angular_velocity.z;
}


template <typename T>
void imuAccel2rosAccel(sensor_msgs::Imu *thisImuMsg, T *acc_x, T *acc_y, T *acc_z) {
  *acc_x = thisImuMsg->linear_acceleration.x;
  *acc_y = thisImuMsg->linear_acceleration.y;
  *acc_z = thisImuMsg->linear_acceleration.z;
}


template <typename T>
void imuRPY2rosRPY(sensor_msgs::Imu *thisImuMsg, T *rosRoll, T *rosPitch, T *rosYaw) {
  double         imuRoll, imuPitch, imuYaw;
  tf::Quaternion orientation;
  tf::quaternionMsgToTF(thisImuMsg->orientation, orientation);
  tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);

  *rosRoll  = imuRoll;
  *rosPitch = imuPitch;
  *rosYaw   = imuYaw;
}


float pointDistance(PointType p) {
  return sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}


float pointDistance(PointType p1, PointType p2) {
  return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
}
