#include "utility.h"

namespace liosam
{
namespace transform_fusion
{

/*//{ class TransformFusion() */
class TransformFusion : public nodelet::Nodelet {
public:
  /*//{ parameters*/
  std::string uavName;

  // Frames
  std::string lidarFrame;
  std::string imuFrame;
  std::string baselinkFrame;
  std::string odometryFrame;
  std::string mapFrame;

  // IMU
  string imuType;
  /*//}*/

  // TF
  geometry_msgs::TransformStamped tfLidar2Baselink;

  // IMU TF
  Eigen::Matrix3d    extRot;
  Eigen::Quaterniond extQRPY;

  std::mutex mtx;

  ros::Subscriber subImuOdometry;
  ros::Subscriber subLaserOdometry;

  ros::Publisher pubImuOdometry;
  ros::Publisher pubImuPath;

  std::shared_ptr<mrs_lib::Transformer> transformer;

  Eigen::Affine3f lidarOdomAffine;
  Eigen::Affine3f imuOdomAffineFront;
  Eigen::Affine3f imuOdomAffineBack;


  double                    lidarOdomTime = -1;
  deque<nav_msgs::Odometry> imuOdomQueue;

  bool is_initialized_ = false;

public:
  /*//{ onInit() */
  virtual void onInit() {

    ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();

    transformer = std::make_shared<mrs_lib::Transformer>(nh, "TransformFusion");

    /*//{ load parameters */
    mrs_lib::ParamLoader pl(nh, "TransformFusion");

    pl.loadParam("uavName", uavName);

    pl.loadParam("imu/frame_id", imuFrame);
    addNamespace(uavName, imuFrame);

    pl.loadParam("lidarFrame", lidarFrame);
    pl.loadParam("baselinkFrame", baselinkFrame);
    pl.loadParam("odometryFrame", odometryFrame);
    pl.loadParam("mapFrame", mapFrame);

    if (!pl.loadedSuccessfully()) {
      ROS_ERROR("[TransformFusion]: Could not load all parameters!");
      ros::shutdown();
    }
    /*//}*/

    geometry_msgs::TransformStamped tfLidar2Imu;
    findLidar2ImuTf(transformer, lidarFrame, imuFrame, baselinkFrame, extRot, extQRPY, tfLidar2Baselink, tfLidar2Imu);

    subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("liosam/fusion/odom_mapping_in", 5, &TransformFusion::lidarOdometryHandler, this, ros::TransportHints().tcpNoDelay());
    subImuOdometry =
        nh.subscribe<nav_msgs::Odometry>("liosam/fusion/odom_preintegrated_in", 10, &TransformFusion::imuOdometryHandler, this, ros::TransportHints().tcpNoDelay());

    pubImuOdometry = nh.advertise<nav_msgs::Odometry>("liosam/fusion/odometry_out", 10);
    pubImuPath     = nh.advertise<nav_msgs::Path>("liosam/fusion/path_out", 1);

    ROS_INFO("\033[1;32m----> [TransformFusion]: initialized.\033[0m");
    is_initialized_ = true;
  }
  /*//}*/

  /*//{ odom2affine() */
  Eigen::Affine3f odom2affine(nav_msgs::Odometry odom) {
    double x, y, z, roll, pitch, yaw;
    x = odom.pose.pose.position.x;
    y = odom.pose.pose.position.y;
    z = odom.pose.pose.position.z;
    tf2::Quaternion orientation;
    tf2::fromMsg(odom.pose.pose.orientation, orientation);
    tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
    return pcl::getTransformation(x, y, z, roll, pitch, yaw);
  }
  /*//}*/

  /*//{ lidarOdometryHandler() */
  void lidarOdometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg) {

    if (!is_initialized_) {
      return;
    }

    ROS_INFO_ONCE("[TransformFusion]: lidarOdometryCallback first callback");

    std::lock_guard<std::mutex> lock(mtx);

    lidarOdomAffine = odom2affine(*odomMsg);

    lidarOdomTime = odomMsg->header.stamp.toSec();
  }
  /*//}*/

  /*//{ imuOdometryHandler() */
  void imuOdometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg) {

    if (!is_initialized_) {
      return;
    }

    ROS_INFO_ONCE("[TransformFusion]: imuOdometryCallback first callback");

    // TODO: publish TFs correctly as `map -> odom -> fcu` (control has to handle this by continuously republishing the reference in map frame)

    // publish tf map->odom (inverted tf-tree)
    static tf2_ros::TransformBroadcaster tfBroadcaster;
    static tf2::Transform            tMap2Odom = tf2::Transform(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(0, 0, 0));
    geometry_msgs::TransformStamped tfMap2Odom;
    tfMap2Odom.header.stamp = odomMsg->header.stamp;
    tfMap2Odom.header.frame_id = mapFrame;
    tfMap2Odom.child_frame_id = odometryFrame;
    tfMap2Odom.transform = tf2::toMsg(tMap2Odom);
    tfBroadcaster.sendTransform(tfMap2Odom);

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

    if (imuOdomQueue.empty()) {
      return;
    }

    const Eigen::Affine3f imuOdomAffineFront = odom2affine(imuOdomQueue.front());
    const Eigen::Affine3f imuOdomAffineBack  = odom2affine(imuOdomQueue.back());
    const Eigen::Affine3f imuOdomAffineIncre = imuOdomAffineFront.inverse() * imuOdomAffineBack;
    const Eigen::Affine3f imuOdomAffineLast  = lidarOdomAffine * imuOdomAffineIncre;
    /* const Eigen::Affine3f imuOdomAffineLast  = lidarOdomAffine; */
    float                 x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(imuOdomAffineLast, x, y, z, roll, pitch, yaw);

    // publish latest odometry
    tf2::Transform tCur;
    tCur.setOrigin(tf2::Vector3(x, y, z));
    tf2::Quaternion qCur;
    qCur.setRPY(roll, pitch, yaw);
    tCur.setRotation(qCur);
    if (lidarFrame != baselinkFrame) {
      tf2::Transform tLidar2Baselink;
      tf2::fromMsg(tfLidar2Baselink.transform, tLidar2Baselink);
      tCur = tCur * tLidar2Baselink;
    }
    tCur.setRotation(tCur.getRotation().normalized());

    nav_msgs::Odometry::Ptr laserOdometry = boost::make_shared<nav_msgs::Odometry>(imuOdomQueue.back());
    laserOdometry->header.frame_id        = odometryFrame;
    laserOdometry->child_frame_id         = baselinkFrame;
    laserOdometry->pose.pose.position.x   = tCur.getOrigin().getX();
    laserOdometry->pose.pose.position.y   = tCur.getOrigin().getY();
    laserOdometry->pose.pose.position.z   = tCur.getOrigin().getZ();
    laserOdometry->pose.pose.orientation = tf2::toMsg(tCur.getRotation());

    if (std::isfinite(laserOdometry->pose.pose.orientation.x) && std::isfinite(laserOdometry->pose.pose.orientation.y) &&
        std::isfinite(laserOdometry->pose.pose.orientation.z) && std::isfinite(laserOdometry->pose.pose.orientation.w)) {
      pubImuOdometry.publish(laserOdometry);

      // publish tf odom->fcu (inverted tf-tree)
    static tf2_ros::TransformBroadcaster tfBroadcaster;
    geometry_msgs::TransformStamped tfOdom2Baselink;
    tfMap2Odom.header.stamp = odomMsg->header.stamp;
    tfMap2Odom.header.frame_id = baselinkFrame;
    tfMap2Odom.child_frame_id = odometryFrame;
    tfMap2Odom.transform = tf2::toMsg(tCur.inverse());
    tfBroadcaster.sendTransform(tfMap2Odom);

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

        const double hdg = mrs_lib::AttitudeConverter(laserOdometry->pose.pose.orientation).getHeading();
        ROS_INFO_THROTTLE(1.0, "[TransformFusion]: pos: [%.2f %.2f %.2f] vel: [%.2f %.2f %.2f] hdg: %.2f", laserOdometry->pose.pose.position.x, laserOdometry->pose.pose.position.y, laserOdometry->pose.pose.position.z, laserOdometry->twist.twist.linear.x, laserOdometry->twist.twist.linear.y, laserOdometry->twist.twist.linear.z, hdg);
    }
  }
  /*//}*/
};
/*//}*/

}  // namespace transform_fusion
}  // namespace liosam

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(liosam::transform_fusion::TransformFusion, nodelet::Nodelet)
