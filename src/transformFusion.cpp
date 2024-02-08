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
  tf::StampedTransform tfLidar2Baselink;

  // IMU TF
  Eigen::Matrix3d    extRot;
  Eigen::Quaterniond extQRPY;

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

  bool is_initialized_ = false;

public:
  /*//{ onInit() */
  virtual void onInit() {

    ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();

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

    tf::StampedTransform tfLidar2Imu;
    findLidar2ImuTf(lidarFrame, imuFrame, baselinkFrame, extRot, extQRPY, tfLidar2Baselink, tfLidar2Imu);

    subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("odom_mapping_in", 5, &TransformFusion::lidarOdometryHandler, this, ros::TransportHints().tcpNoDelay());
    subImuOdometry =
        nh.subscribe<nav_msgs::Odometry>("odom_imu_incremental_in", 2000, &TransformFusion::imuOdometryHandler, this, ros::TransportHints().tcpNoDelay());

    pubImuOdometry = nh.advertise<nav_msgs::Odometry>("fused_odometry_out", 2000);
    pubImuPath     = nh.advertise<nav_msgs::Path>("fused_path_out", 1);

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
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(odom.pose.pose.orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
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

}  // namespace transform_fusion
}  // namespace liosam

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(liosam::transform_fusion::TransformFusion, nodelet::Nodelet)
