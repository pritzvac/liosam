#include "utility.h"

struct PointXYZIRT
{
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY;
  uint32_t t;
  uint8_t  ring;
  uint32_t range;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint32_t, t, t)(std::uint8_t, ring,
                                                                                                                            ring)(std::uint32_t, range, range))

namespace liosam
{
namespace image_projection
{

const int queueLength = 2000;

/*//{ class ImageProjection() */
class ImageProjection : public nodelet::Nodelet {
private:
  std::mutex imuLock;
  std::mutex odoLock;

  ros::Subscriber subLaserCloud;
  ros::Publisher  pubLaserCloud;

  ros::Publisher pubExtractedCloud;
  ros::Publisher pubLaserCloudInfo;
  ros::Publisher pubOrigCloudInfo;

  ros::Subscriber              subImu;
  std::deque<sensor_msgs::Imu> imuQueue;

  ros::Subscriber                subOdom;
  std::deque<nav_msgs::Odometry> odomQueue;

  std::deque<sensor_msgs::PointCloud2> cloudQueue;
  sensor_msgs::PointCloud2             currentCloudMsg;

  double *imuTime = new double[queueLength];
  double *imuRotX = new double[queueLength];
  double *imuRotY = new double[queueLength];
  double *imuRotZ = new double[queueLength];

  int             imuPointerCur;
  bool            firstPointFlag;
  Eigen::Affine3f transStartInverse;

  pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
  pcl::PointCloud<PointType>::Ptr   fullCloud;
  pcl::PointCloud<PointType>::Ptr   extractedCloud;

  int     deskewFlag;
  cv::Mat rangeMat;

  bool  odomDeskewFlag;
  float odomIncreX;
  float odomIncreY;
  float odomIncreZ;

  liosam::cloud_info::Ptr cloudInfo = boost::make_shared<liosam::cloud_info>();
  double                  timeScanCur;
  double                  timeScanEnd;
  std_msgs::Header        cloudHeader;

  bool isMemoryAllocated = false;
  int  scanHeight;
  int  scanWidth;

  /*//{ parameters */

  std::string uavName;

  // Frames
  std::string lidarFrame;
  std::string imuFrame;
  std::string baselinkFrame;

  // LIDAR
  string timeField;
  int    downsampleRate;
  float  lidarMinRange;
  float  lidarMaxRange;

  // IMU
  bool   imuDeskew;
  string imuType;

  /*//}*/

  // IMU TF
  Eigen::Matrix3d    extRot;
  Eigen::Quaterniond extQRPY;

public:
  /*//{ onInit() */
  virtual void onInit() {

    ROS_INFO("[ImageProjection]: initializing");

    deskewFlag = 0;

    ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();

    /*//{ load parameters */
    mrs_lib::ParamLoader pl(nh, "ImageProjection");

    pl.loadParam("uavName", uavName);

    pl.loadParam("imu/deskew", imuDeskew);
    if (imuDeskew) {
      pl.loadParam("imu/frame_id", imuFrame);
      addNamespace(uavName, imuFrame);
    }

    pl.loadParam("lidarFrame", lidarFrame);
    addNamespace(uavName, lidarFrame);
    pl.loadParam("baselinkFrame", baselinkFrame);
    addNamespace(uavName, baselinkFrame);

    pl.loadParam("timeField", timeField, std::string("t"));
    pl.loadParam("downsampleRate", downsampleRate, 1);
    pl.loadParam("lidarMinRange", lidarMinRange, 0.1f);
    pl.loadParam("lidarMaxRange", lidarMaxRange, 1000.0f);

    if (!pl.loadedSuccessfully()) {
      ROS_ERROR("[ImageProjection]: Could not load all parameters!");
      ros::shutdown();
    }

    /*//}*/

    if (imuDeskew) {
      tf::StampedTransform tfLidar2Baselink, tfLidar2Imu;
      findLidar2ImuTf(lidarFrame, imuFrame, baselinkFrame, extRot, extQRPY, tfLidar2Baselink, tfLidar2Imu);

      subImu = nh.subscribe<sensor_msgs::Imu>("imu_in", 2000, &ImageProjection::imuHandler, this, ros::TransportHints().tcpNoDelay());
    }
    subOdom       = nh.subscribe<nav_msgs::Odometry>("odom_incremental_in", 2000, &ImageProjection::odometryHandler, this, ros::TransportHints().tcpNoDelay());
    subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("cloud_in", 5, &ImageProjection::cloudHandler, this, ros::TransportHints().tcpNoDelay());

    pubExtractedCloud = nh.advertise<sensor_msgs::PointCloud2>("liosam/deskew/deskewed_cloud_out", 1);
    pubLaserCloudInfo = nh.advertise<liosam::cloud_info>("liosam/deskew/deskewed_cloud_info_out", 1);
    pubOrigCloudInfo = nh.advertise<sensor_msgs::PointCloud2>("liosam/deskew/orig_cloud_info_out", 1);

    /* pcl::console::setVerbosityLevel(pcl::console::L_ERROR); */

    ROS_INFO("\033[1;32m----> [Image Projection]: initialized.\033[0m");
  }
  /*//}*/

  /*//{ allocateMemory() */
  void allocateMemory() {
    laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
    fullCloud.reset(new pcl::PointCloud<PointType>());
    extractedCloud.reset(new pcl::PointCloud<PointType>());

    fullCloud->points.resize(scanHeight * scanWidth);

    cloudInfo->startRingIndex.assign(scanHeight, 0);
    cloudInfo->endRingIndex.assign(scanHeight, 0);

    cloudInfo->pointColInd.assign(scanHeight * scanWidth, 0);
    cloudInfo->pointRange.assign(scanHeight * scanWidth, 0);

    resetParameters();
  }
  /*//}*/

  /*//{ resetParameters() */
  void resetParameters() {
    laserCloudIn->clear();
    extractedCloud->clear();
    // reset range matrix for range image projection
    rangeMat = cv::Mat(scanHeight, scanWidth, CV_32F, cv::Scalar::all(FLT_MAX));

    imuPointerCur  = 0;
    firstPointFlag = true;
    odomDeskewFlag = false;

    for (int i = 0; i < queueLength; ++i) {
      imuTime[i] = 0;
      imuRotX[i] = 0;
      imuRotY[i] = 0;
      imuRotZ[i] = 0;
    }
  }
  /*//}*/

  /*//{ imuHandler() */
  void imuHandler(const sensor_msgs::Imu::ConstPtr &imuMsg) {
    const sensor_msgs::Imu thisImu = imuConverter(*imuMsg, extRot, extQRPY);

    ROS_INFO_ONCE("[ImageProjection]: imuHandler first callback");

    std::lock_guard<std::mutex> lock1(imuLock);
    imuQueue.push_back(thisImu);

    // debug IMU data
    /* cout << std::setprecision(6); */
    /* cout << "IMU acc: " << endl; */
    /* cout << "x: " << thisImu.linear_acceleration.x << ", y: " << thisImu.linear_acceleration.y << ", z: " << thisImu.linear_acceleration.z << endl; */
    /* cout << "IMU gyro: " << endl; */
    /* cout << "x: " << thisImu.angular_velocity.x << ", y: " << thisImu.angular_velocity.y << ", z: " << thisImu.angular_velocity.z << endl; */
    /* double         imuRoll, imuPitch, imuYaw; */
    /* tf::Quaternion orientation; */
    /* tf::quaternionMsgToTF(thisImu.orientation, orientation); */
    /* tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw); */
    /* cout << "IMU roll pitch yaw: " << endl; */
    /* cout << "roll: " << imuRoll << ", pitch: " << imuPitch << ", yaw: " << imuYaw << endl << endl; */
  }
  /*//}*/

  /*//{ odometryHandler() */
  void odometryHandler(const nav_msgs::Odometry::ConstPtr &odometryMsg) {

    ROS_INFO_ONCE("[ImageProjection]: odometryHandler first callback");

    std::lock_guard<std::mutex> lock2(odoLock);
    odomQueue.push_back(*odometryMsg);
  }
  /*//}*/

  /*//{ cloudHandler() */
  void cloudHandler(const sensor_msgs::PointCloud2::ConstPtr &laserCloudMsg) {

    ROS_INFO_ONCE("[ImageProjection]: cloudHandler first callback");

    if (!isMemoryAllocated) {
      scanHeight = laserCloudMsg->height;
      scanWidth  = laserCloudMsg->width;
      allocateMemory();
      isMemoryAllocated = true;
      ROS_INFO("[ImageProjection]: First scan height: %d width: %d", scanHeight, scanWidth);
    }

    if (!cachePointCloud(laserCloudMsg)) {
      return;
    }

    if (!deskewInfo()) {
      return;
    }

    projectPointCloud();

    cloudExtraction();

    publishClouds();

    resetParameters();
  }
  /*//}*/

  /*//{ cachePointCloud() */
  bool cachePointCloud(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg) {
    // cache point cloud
    cloudQueue.push_back(*laserCloudMsg);
    if (cloudQueue.size() <= 2) {
      return false;
    }

    // convert cloud
    // why front of queue? this causes always the oldest point cloud to be processed i.e. delay of 200 ms?
    /* currentCloudMsg = std::move(cloudQueue.front()); */
    /* cloudQueue.pop_front(); */
    currentCloudMsg = std::move(cloudQueue.back());
    cloudQueue.pop_back();
    pcl::fromROSMsg(currentCloudMsg, *laserCloudIn);

    // get timestamp
    cloudHeader = currentCloudMsg.header;
    timeScanCur = cloudHeader.stamp.toSec();
    /* timeScanEnd = timeScanCur + laserCloudIn->points.back().time; // Velodyne */
    /* timeScanEnd = timeScanCur + (float)laserCloudIn->points.back().t / 1.0e9;  // Ouster */
    timeScanEnd = timeScanCur;  // sim

    // check dense flag
    if (!laserCloudIn->is_dense) {
      removeNaNFromPointCloud(laserCloudIn, laserCloudIn);
      /* ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!"); */
      /* ros::shutdown(); */
    }

    // check ring channel
    static int ringFlag = 0;
    if (ringFlag == 0) {
      ringFlag = -1;
      for (auto &field : currentCloudMsg.fields) {
        if (field.name == "ring") {
          ringFlag = 1;
          break;
        }
      }
      if (ringFlag == -1) {
        ROS_ERROR("Point cloud ring channel not available, please configure your point cloud data!");
        ros::shutdown();
      }
    }

    // check point time
    if (deskewFlag == 0) {
      deskewFlag = -1;
      for (auto &field : currentCloudMsg.fields) {
        if (field.name == timeField) {
          deskewFlag = 1;
          break;
        }
      }
      if (deskewFlag == -1)
        ROS_WARN("Point cloud timestamp not available, deskew function disabled, system will drift significantly!");
    }

    return true;
  }
  /*//}*/

  /*//{ deskewInfo() */
  bool deskewInfo() {

    if (imuDeskew) {
      std::lock_guard<std::mutex> lock1(imuLock);

      // make sure IMU data available for the scan
      if (imuQueue.empty() || imuQueue.front().header.stamp.toSec() > timeScanCur || imuQueue.back().header.stamp.toSec() < timeScanEnd) {
        if (imuQueue.empty()) {
          ROS_WARN("[ImageProjection]: Waiting for IMU data ... imu queue is empty");
        } else if (imuQueue.front().header.stamp.toSec() > timeScanCur) {
          ROS_WARN("[ImageProjection]: Waiting for IMU data ... imu msg time (%0.2f) > time scan cur (%0.2f)", imuQueue.back().header.stamp.toSec(),
                   timeScanCur);
        } else if (imuQueue.back().header.stamp.toSec() < timeScanEnd) {
          ROS_WARN("[ImageProjection]: Waiting for IMU data ... imu msg time (%0.2f) < time scan end time (%0.2f)", imuQueue.back().header.stamp.toSec(),
                   timeScanEnd);
        }
        return false;
      }

      imuDeskewInfo();
    }

    odomDeskewInfo();

    return true;
  }
  /*//}*/

  /*//{ imuDeskewInfo() */
  void imuDeskewInfo() {
    cloudInfo->imuAvailable = false;

    while (!imuQueue.empty()) {
      if (imuQueue.front().header.stamp.toSec() < timeScanCur - 0.01) {
        imuQueue.pop_front();
      } else {
        break;
      }
    }

    if (imuQueue.empty()) {
      return;
    }

    imuPointerCur = 0;

    for (int i = 0; i < (int)imuQueue.size(); ++i) {
      sensor_msgs::Imu thisImuMsg     = imuQueue[i];
      const double     currentImuTime = thisImuMsg.header.stamp.toSec();

      // get roll, pitch, and yaw estimation for this scan
      if (currentImuTime <= timeScanCur) {
        imuRPY2rosRPY(&thisImuMsg, &cloudInfo->imuRollInit, &cloudInfo->imuPitchInit, &cloudInfo->imuYawInit);
      }

      if (currentImuTime > timeScanEnd + 0.01) {
        break;
      }

      if (imuPointerCur == 0) {
        imuRotX[0] = 0;
        imuRotY[0] = 0;
        imuRotZ[0] = 0;
        imuTime[0] = currentImuTime;
        ++imuPointerCur;
        continue;
      }

      // get angular velocity
      double angular_x, angular_y, angular_z;
      imuAngular2rosAngular(&thisImuMsg, &angular_x, &angular_y, &angular_z);

      // integrate rotation
      const double timeDiff  = currentImuTime - imuTime[imuPointerCur - 1];
      imuRotX[imuPointerCur] = imuRotX[imuPointerCur - 1] + angular_x * timeDiff;
      imuRotY[imuPointerCur] = imuRotY[imuPointerCur - 1] + angular_y * timeDiff;
      imuRotZ[imuPointerCur] = imuRotZ[imuPointerCur - 1] + angular_z * timeDiff;
      imuTime[imuPointerCur] = currentImuTime;
      ++imuPointerCur;
    }

    --imuPointerCur;

    if (imuPointerCur <= 0) {
      return;
    }

    cloudInfo->imuAvailable = true;
  }
  /*//}*/

  /*//{ odomDeskewInfo() */
  void odomDeskewInfo() {
    std::lock_guard<std::mutex> lock2(odoLock);
    cloudInfo->odomAvailable = false;

    while (!odomQueue.empty()) {
      if (odomQueue.front().header.stamp.toSec() < timeScanCur - 0.01) {
        odomQueue.pop_front();
      } else {
        break;
      }
    }

    if (odomQueue.empty()) {
      return;
    }

    if (odomQueue.front().header.stamp.toSec() > timeScanCur) {
      return;
    }

    // get start odometry at the beinning of the scan
    nav_msgs::Odometry startOdomMsg;

    for (int i = 0; i < (int)odomQueue.size(); ++i) {
      startOdomMsg = odomQueue[i];

      if (ROS_TIME(&startOdomMsg) < timeScanCur) {
        continue;
      } else {
        break;
      }
    }

    tf::Quaternion orientation;
    tf::quaternionMsgToTF(startOdomMsg.pose.pose.orientation, orientation);

    double roll, pitch, yaw;
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

    // Initial guess used in mapOptimization
    cloudInfo->initialGuessX     = startOdomMsg.pose.pose.position.x;
    cloudInfo->initialGuessY     = startOdomMsg.pose.pose.position.y;
    cloudInfo->initialGuessZ     = startOdomMsg.pose.pose.position.z;
    cloudInfo->initialGuessRoll  = roll;
    cloudInfo->initialGuessPitch = pitch;
    cloudInfo->initialGuessYaw   = yaw;

    cloudInfo->odomAvailable = true;

    // petrlmat: The following code was commented out by me as it is not needed. Only position is extracted from the odometry to be later used in positional
    // deskewing, which was commented out by the LIOSAM authors as it makes little difference. get end odometry at the end of the scan
    /* odomDeskewFlag = false; */

    /* if (odomQueue.back().header.stamp.toSec() < timeScanEnd) { */
    /*   return; */
    /* } */

    /* nav_msgs::Odometry endOdomMsg; */

    /* for (int i = 0; i < (int)odomQueue.size(); ++i) { */
    /*   endOdomMsg = odomQueue[i]; */

    /*   if (ROS_TIME(&endOdomMsg) < timeScanEnd) { */
    /*     continue; */
    /*   } else { */
    /*     break; */
    /*   } */
    /* } */

    /* if (int(round(startOdomMsg.pose.covariance[0])) != int(round(endOdomMsg.pose.covariance[0]))) { */
    /*   return; */
    /* } */

    /* const Eigen::Affine3f transBegin = */
    /*     pcl::getTransformation(startOdomMsg.pose.pose.position.x, startOdomMsg.pose.pose.position.y, startOdomMsg.pose.pose.position.z, roll, pitch, yaw); */

    /* tf::quaternionMsgToTF(endOdomMsg.pose.pose.orientation, orientation); */
    /* tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw); */
    /* const Eigen::Affine3f transEnd = */
    /*     pcl::getTransformation(endOdomMsg.pose.pose.position.x, endOdomMsg.pose.pose.position.y, endOdomMsg.pose.pose.position.z, roll, pitch, yaw); */

    /* const Eigen::Affine3f transBt = transBegin.inverse() * transEnd; */

    /* float rollIncre, pitchIncre, yawIncre; */
    /* pcl::getTranslationAndEulerAngles(transBt, odomIncreX, odomIncreY, odomIncreZ, rollIncre, pitchIncre, yawIncre); */

    /* odomDeskewFlag = true; */
  }
  /*//}*/

  /*//{ findRotation() */
  void findRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur) {
    *rotXCur = 0;
    *rotYCur = 0;
    *rotZCur = 0;

    int imuPointerFront = 0;
    while (imuPointerFront < imuPointerCur) {
      if (pointTime < imuTime[imuPointerFront]) {
        break;
      }
      ++imuPointerFront;
    }

    if (pointTime > imuTime[imuPointerFront] || imuPointerFront == 0) {
      *rotXCur = imuRotX[imuPointerFront];
      *rotYCur = imuRotY[imuPointerFront];
      *rotZCur = imuRotZ[imuPointerFront];
    } else {
      const int    imuPointerBack = imuPointerFront - 1;
      const double ratioFront     = (pointTime - imuTime[imuPointerBack]) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
      const double ratioBack      = (imuTime[imuPointerFront] - pointTime) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
      *rotXCur                    = imuRotX[imuPointerFront] * ratioFront + imuRotX[imuPointerBack] * ratioBack;
      *rotYCur                    = imuRotY[imuPointerFront] * ratioFront + imuRotY[imuPointerBack] * ratioBack;
      *rotZCur                    = imuRotZ[imuPointerFront] * ratioFront + imuRotZ[imuPointerBack] * ratioBack;
    }
  }
  /*//}*/

  /*//{ findPosition() */
  void findPosition(double relTime, float *posXCur, float *posYCur, float *posZCur) {
    *posXCur = 0;
    *posYCur = 0;
    *posZCur = 0;

    // If the sensor moves relatively slow, like walking speed, positional deskew seems to have little benefits. Thus code below is commented.

    // if (cloudInfo->odomAvailable == false || odomDeskewFlag == false)
    //     return;

    // float ratio = relTime / (timeScanEnd - timeScanCur);

    // *posXCur = ratio * odomIncreX;
    // *posYCur = ratio * odomIncreY;
    // *posZCur = ratio * odomIncreZ;
  }
  /*//}*/

  /*//{ deskewPoint() */
  PointType deskewPoint(PointType *point, double relTime) {
    if (deskewFlag == -1 || cloudInfo->imuAvailable == false) {
      return *point;
    }

    const double pointTime = timeScanCur + relTime;

    float rotXCur, rotYCur, rotZCur;
    findRotation(pointTime, &rotXCur, &rotYCur, &rotZCur);  // petrlmat: from imu only

    float posXCur, posYCur, posZCur;
    findPosition(relTime, &posXCur, &posYCur, &posZCur);  // petrlmat: not used, always zero position

    if (firstPointFlag == true) {
      transStartInverse = (pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur)).inverse();
      firstPointFlag    = false;
    }

    // transform points to start
    const Eigen::Affine3f transFinal = pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);
    const Eigen::Affine3f transBt    = transStartInverse * transFinal;

    PointType newPoint;
    newPoint.x         = transBt(0, 0) * point->x + transBt(0, 1) * point->y + transBt(0, 2) * point->z + transBt(0, 3);
    newPoint.y         = transBt(1, 0) * point->x + transBt(1, 1) * point->y + transBt(1, 2) * point->z + transBt(1, 3);
    newPoint.z         = transBt(2, 0) * point->x + transBt(2, 1) * point->y + transBt(2, 2) * point->z + transBt(2, 3);
    newPoint.intensity = point->intensity;

    return newPoint;
  }
  /*//}*/

  /*//{ projectPointCloud() */
  // petrlmat: this functions only copies points from one point cloud to another (of another type) when deskewing is disabled (default so far) - potential
  // performance gain if we get rid of the copy
  void projectPointCloud() {
    // range image projection
    const unsigned int cloudSize = laserCloudIn->points.size();
    for (unsigned int i = 0; i < cloudSize; i++) {
      /* ROS_WARN("(%d, %d) - xyz: (%0.2f, %0.2f, %0.2f), ring: %d", j, rowIdn, laserCloudIn->at(j, rowIdn).x, laserCloudIn->at(j, rowIdn).y, */
      /*          laserCloudIn->at(j, rowIdn).z, laserCloudIn->at(j, rowIdn).ring); */

      /* const float range = pointDistance(thisPoint); */
      const float range = laserCloudIn->points.at(i).range / 1000.0f;
      if (range < lidarMinRange || range > lidarMaxRange) {
        continue;
      }

      const int rowIdn = laserCloudIn->points.at(i).ring;
      if (rowIdn < 0 || rowIdn >= scanHeight) {
        /* ROS_ERROR("Invalid ring: %d", rowIdn); */
        continue;
      }

      if (rowIdn % downsampleRate != 0) {
        /* ROS_ERROR("Downsampling. Throwing away row: %d", rowIdn); */
        continue;
      }

      PointType thisPoint;
      thisPoint.x         = laserCloudIn->points.at(i).x;
      thisPoint.y         = laserCloudIn->points.at(i).y;
      thisPoint.z         = laserCloudIn->points.at(i).z;
      thisPoint.intensity = laserCloudIn->points.at(i).intensity;

      // TODO: polish this monstrosity
      const float  horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
      static float ang_res_x    = 360.0 / float(scanWidth);
      int          columnIdn    = -round((horizonAngle - 90.0) / ang_res_x) + scanWidth / 2;
      if (columnIdn >= scanWidth) {
        columnIdn -= scanWidth;
      }

      if (columnIdn < 0 || columnIdn >= scanWidth) {
        continue;
      }

      if (rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX) {
        continue;
      }

      // petrlmat: so far, we were using liosam without deskewing
      /* thisPoint = deskewPoint(&thisPoint, laserCloudIn->at(j, rowIdn).time); // Velodyne */
      /* thisPoint = deskewPoint(&thisPoint, (float)laserCloudIn->at(j, rowIdn).t / 1000000000.0);  // Ouster */

      rangeMat.at<float>(rowIdn, columnIdn) = range;

      const int index          = columnIdn + rowIdn * scanWidth;
      fullCloud->points[index] = thisPoint;
    }
  }
  /*//}*/

  /*//{ cloudExtraction() */
  void cloudExtraction() {
    int count = 0;
    // extract segmented cloud for lidar odometry
    for (int i = 0; i < scanHeight; ++i) {
      cloudInfo->startRingIndex[i] = count - 1 + 5;

      for (int j = 0; j < scanWidth; ++j) {
        /* ROS_WARN("i: %d, j: %d, rangeMat: %0.10f, isFltMax: %d", i, j, rangeMat.at<float>(i,j), rangeMat.at<float>(i,j) == FLT_MAX); */
        if (rangeMat.at<float>(i, j) != FLT_MAX) {
          /* ROS_WARN("i: %d, j: %d, rangeMat: %0.10f, isFltMax: %d", i, j, rangeMat.at<float>(i,j), rangeMat.at<float>(i,j) == FLT_MAX); */
          // mark the points' column index for marking occlusion later
          cloudInfo->pointColInd[count] = j;
          // save range info
          cloudInfo->pointRange[count] = rangeMat.at<float>(i, j);
          // save extracted cloud
          extractedCloud->push_back(fullCloud->points[j + i * scanWidth]);
          // size of extracted cloud
          ++count;
        }
      }
      cloudInfo->endRingIndex[i] = count - 1 - 5;
    }
  }
  /*//}*/

  /*//{ publishClouds() */
  void publishClouds() {
    cloudInfo->header         = cloudHeader;
    cloudInfo->cloud_deskewed = publishCloud(&pubExtractedCloud, extractedCloud, cloudHeader.stamp, lidarFrame);
    try {
      pubLaserCloudInfo.publish(cloudInfo);
    }
    catch (...) {
      ROS_ERROR("[ImageProjection]: Exception caught during publishing topic %s.", pubLaserCloudInfo.getTopic().c_str());
    }
    
    sensor_msgs::PointCloud2 cloudInfoOrig;
    cloudInfoOrig.header = cloudHeader;
    cloudInfoOrig.height = scanHeight;
    cloudInfoOrig.width = scanWidth;
    try {
      pubOrigCloudInfo.publish(cloudInfoOrig);
    }
    catch (...) {
      ROS_ERROR("[ImageProjection]: Exception caught during publishing topic %s.", pubLaserCloudInfo.getTopic().c_str());
    }
  }
  /*//}*/

  /*//{ removeNaNFromPointCloud() */
  void removeNaNFromPointCloud(const pcl::PointCloud<PointXYZIRT>::Ptr &cloud_in, pcl::PointCloud<PointXYZIRT>::Ptr &cloud_out) {

    if (cloud_in->is_dense) {
      cloud_out = cloud_in;
      return;
    }

    unsigned int k = 0;

    cloud_out->resize(cloud_in->size());

    for (unsigned int i = 0; i < cloud_in->size(); i++) {

      if (std::isfinite(cloud_in->at(i).x) && std::isfinite(cloud_in->at(i).y) && std::isfinite(cloud_in->at(i).z)) {
        cloud_out->at(k++) = cloud_in->at(i);
      }
    }

    cloud_out->header   = cloud_in->header;
    cloud_out->is_dense = true;

    if (cloud_in->size() != k) {
      cloud_out->resize(k);
    }
  }
  /*//}*/
};
/*//}*/

}  // namespace image_projection
}  // namespace liosam

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(liosam::image_projection::ImageProjection, nodelet::Nodelet)
