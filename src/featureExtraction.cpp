#include "utility.h"
#include "liosam/cloud_info.h"

namespace liosam
{
namespace feature_extraction
{

struct smoothness_t
{
  float  value;
  size_t ind;
};

struct by_value
{
  bool operator()(smoothness_t const &left, smoothness_t const &right) {
    return left.value < right.value;
  }
};

/*//{ class FeatureExtractionImpl() */
class FeatureExtractionImpl : public ParamServer {

public:
  ros::Subscriber subLaserCloudInfo;

  ros::Publisher pubLaserCloudInfo;
  ros::Publisher pubCornerPoints;
  ros::Publisher pubSurfacePoints;

  pcl::PointCloud<PointType>::Ptr extractedCloud;
  pcl::PointCloud<PointType>::Ptr cornerCloud;
  pcl::PointCloud<PointType>::Ptr surfaceCloud;

  pcl::VoxelGrid<PointType> downSizeFilter;

  std::vector<smoothness_t> cloudSmoothness;
  float *                   cloudCurvature;
  int *                     cloudNeighborPicked;
  int *                     cloudLabel;

public:
  /*//{ FeatureExtractionImpl() */
  FeatureExtractionImpl() {
    subLaserCloudInfo = nh.subscribe<liosam::cloud_info>("liosam/deskew/cloud_info", 1, &FeatureExtractionImpl::laserCloudInfoHandler, this,
                                                         ros::TransportHints().tcpNoDelay());

    pubLaserCloudInfo = nh.advertise<liosam::cloud_info>("liosam/feature/cloud_info", 1);
    pubCornerPoints   = nh.advertise<sensor_msgs::PointCloud2>("liosam/feature/cloud_corner", 1);
    pubSurfacePoints  = nh.advertise<sensor_msgs::PointCloud2>("liosam/feature/cloud_surface", 1);

    initializationValue();
  }
  /*//}*/

  /*//{ initializationValue() */
  void initializationValue() {
    cloudSmoothness.resize(N_SCAN * Horizon_SCAN);

    downSizeFilter.setLeafSize(odometrySurfLeafSize, odometrySurfLeafSize, odometrySurfLeafSize);

    extractedCloud.reset(new pcl::PointCloud<PointType>());
    cornerCloud.reset(new pcl::PointCloud<PointType>());
    surfaceCloud.reset(new pcl::PointCloud<PointType>());

    cloudCurvature      = new float[N_SCAN * Horizon_SCAN];
    cloudNeighborPicked = new int[N_SCAN * Horizon_SCAN];
    cloudLabel          = new int[N_SCAN * Horizon_SCAN];
  }
  /*//}*/

  /*//{ laserCloudInfoHandler() */
  void laserCloudInfoHandler(const liosam::cloud_info::ConstPtr &msgIn) {
    pcl::fromROSMsg(msgIn->cloud_deskewed, *extractedCloud);  // new cloud for extraction

    calculateSmoothness(msgIn);

    markOccludedPoints(msgIn);

    extractFeatures(msgIn);

    publishFeatureCloud(msgIn);
  }
  /*//}*/

  /*//{ calculateSmoothness() */
  void calculateSmoothness(const liosam::cloud_info::ConstPtr &cloud_info) {
    const int cloudSize = extractedCloud->points.size();
    for (int i = 5; i < cloudSize - 5; i++) {
      const float diffRange = cloud_info->pointRange[i - 5] + cloud_info->pointRange[i - 4] + cloud_info->pointRange[i - 3] + cloud_info->pointRange[i - 2] +
                              cloud_info->pointRange[i - 1] - cloud_info->pointRange[i] * 10 + cloud_info->pointRange[i + 1] + cloud_info->pointRange[i + 2] +
                              cloud_info->pointRange[i + 3] + cloud_info->pointRange[i + 4] + cloud_info->pointRange[i + 5];

      cloudCurvature[i] = diffRange * diffRange;  // diffX * diffX + diffY * diffY + diffZ * diffZ;

      cloudNeighborPicked[i] = 0;
      cloudLabel[i]          = 0;
      // cloudSmoothness for sorting
      cloudSmoothness[i].value = cloudCurvature[i];
      cloudSmoothness[i].ind   = i;
    }
  }
  /*//}*/

  /*//{ markOccludedPoints() */
  void markOccludedPoints(const liosam::cloud_info::ConstPtr &cloud_info) {
    const int cloudSize = extractedCloud->points.size();
    // mark occluded points and parallel beam points
    for (int i = 5; i < cloudSize - 6; ++i) {
      // occluded points
      const float depth1     = cloud_info->pointRange[i];
      const float depth2     = cloud_info->pointRange[i + 1];
      const int   columnDiff = std::abs(int(cloud_info->pointColInd[i + 1] - cloud_info->pointColInd[i]));

      if (columnDiff < 10) {
        // 10 pixel diff in range image
        if (depth1 - depth2 > 0.3) {
          cloudNeighborPicked[i - 5] = 1;
          cloudNeighborPicked[i - 4] = 1;
          cloudNeighborPicked[i - 3] = 1;
          cloudNeighborPicked[i - 2] = 1;
          cloudNeighborPicked[i - 1] = 1;
          cloudNeighborPicked[i]     = 1;
        } else if (depth2 - depth1 > 0.3) {
          cloudNeighborPicked[i + 1] = 1;
          cloudNeighborPicked[i + 2] = 1;
          cloudNeighborPicked[i + 3] = 1;
          cloudNeighborPicked[i + 4] = 1;
          cloudNeighborPicked[i + 5] = 1;
          cloudNeighborPicked[i + 6] = 1;
        }
      }
      // parallel beam
      const float diff1 = std::abs(float(cloud_info->pointRange[i - 1] - cloud_info->pointRange[i]));
      const float diff2 = std::abs(float(cloud_info->pointRange[i + 1] - cloud_info->pointRange[i]));

      if (diff1 > 0.02 * cloud_info->pointRange[i] && diff2 > 0.02 * cloud_info->pointRange[i]) {
        cloudNeighborPicked[i] = 1;
      }
    }
  }
  /*//}*/

  /*//{ extractFeatures() */
  void extractFeatures(const liosam::cloud_info::ConstPtr &cloud_info) {
    cornerCloud->clear();
    surfaceCloud->clear();

    pcl::PointCloud<PointType>::Ptr surfaceCloudScan(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr surfaceCloudScanDS(new pcl::PointCloud<PointType>());

    for (int i = 0; i < N_SCAN; i++) {
      surfaceCloudScan->clear();

      for (int j = 0; j < 6; j++) {

        const int sp = (cloud_info->startRingIndex[i] * (6 - j) + cloud_info->endRingIndex[i] * j) / 6;
        const int ep = (cloud_info->startRingIndex[i] * (5 - j) + cloud_info->endRingIndex[i] * (j + 1)) / 6 - 1;

        if (sp >= ep) {
          continue;
        }

        std::sort(cloudSmoothness.begin() + sp, cloudSmoothness.begin() + ep, by_value());

        int largestPickedNum = 0;
        for (int k = ep; k >= sp; k--) {
          const int ind = cloudSmoothness[k].ind;
          if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > edgeThreshold) {
            largestPickedNum++;
            if (largestPickedNum <= 20) {
              cloudLabel[ind] = 1;
              cornerCloud->push_back(extractedCloud->points[ind]);
            } else {
              break;
            }

            cloudNeighborPicked[ind] = 1;
            for (int l = 1; l <= 5; l++) {
              const int columnDiff = std::abs(int(cloud_info->pointColInd[ind + l] - cloud_info->pointColInd[ind + l - 1]));
              if (columnDiff > 10) {
                break;
              }
              cloudNeighborPicked[ind + l] = 1;
            }
            for (int l = -1; l >= -5; l--) {
              const int columnDiff = std::abs(int(cloud_info->pointColInd[ind + l] - cloud_info->pointColInd[ind + l + 1]));
              if (columnDiff > 10) {
                break;
              }
              cloudNeighborPicked[ind + l] = 1;
            }
          }
        }

        for (int k = sp; k <= ep; k++) {
          const int ind = cloudSmoothness[k].ind;
          if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < surfThreshold) {

            cloudLabel[ind]          = -1;
            cloudNeighborPicked[ind] = 1;

            for (int l = 1; l <= 5; l++) {

              const int columnDiff = std::abs(int(cloud_info->pointColInd[ind + l] - cloud_info->pointColInd[ind + l - 1]));
              if (columnDiff > 10) {
                break;
              }

              cloudNeighborPicked[ind + l] = 1;
            }
            for (int l = -1; l >= -5; l--) {

              const int columnDiff = std::abs(int(cloud_info->pointColInd[ind + l] - cloud_info->pointColInd[ind + l + 1]));
              if (columnDiff > 10) {
                break;
              }

              cloudNeighborPicked[ind + l] = 1;
            }
          }
        }

        for (int k = sp; k <= ep; k++) {
          if (cloudLabel[k] <= 0) {
            surfaceCloudScan->push_back(extractedCloud->points[k]);
          }
        }
      }

      surfaceCloudScanDS->clear();
      downSizeFilter.setInputCloud(surfaceCloudScan);
      downSizeFilter.filter(*surfaceCloudScanDS);

      *surfaceCloud += *surfaceCloudScanDS;
    }
  }
  /*//}*/

  /*//{ publishFeatureCloud() */
  void publishFeatureCloud(const liosam::cloud_info::ConstPtr &msg) {

    // Copy everything except: laser data indices and ranges (no further need for this information)
    liosam::cloud_info::Ptr cloudInfo = boost::make_shared<liosam::cloud_info>();
    cloudInfo->header                 = msg->header;
    cloudInfo->imuAvailable           = msg->imuAvailable;
    cloudInfo->odomAvailable          = msg->odomAvailable;
    cloudInfo->imuRollInit            = msg->imuRollInit;
    cloudInfo->imuPitchInit           = msg->imuPitchInit;
    cloudInfo->imuYawInit             = msg->imuYawInit;
    cloudInfo->initialGuessX          = msg->initialGuessX;
    cloudInfo->initialGuessY          = msg->initialGuessY;
    cloudInfo->initialGuessZ          = msg->initialGuessZ;
    cloudInfo->initialGuessRoll       = msg->initialGuessRoll;
    cloudInfo->initialGuessPitch      = msg->initialGuessPitch;
    cloudInfo->initialGuessYaw        = msg->initialGuessYaw;
    cloudInfo->cloud_deskewed         = msg->cloud_deskewed;

    // save newly extracted features
    cloudInfo->cloud_corner  = publishCloud(&pubCornerPoints, cornerCloud, msg->header.stamp, lidarFrame);
    cloudInfo->cloud_surface = publishCloud(&pubSurfacePoints, surfaceCloud, msg->header.stamp, lidarFrame);

    // publish to mapOptimization
    try {
      pubLaserCloudInfo.publish(cloudInfo);
    }
    catch (...) {
      ROS_ERROR("[LioSam|FE]: Exception caught during publishing topic %s.", pubLaserCloudInfo.getTopic().c_str());
    }
  }
  /*//}*/
};
/*//}*/

/* //{ class FeatureExtraction */

class FeatureExtraction : public nodelet::Nodelet {

public:
  virtual void onInit() {
    ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();
    FE                  = std::make_unique<FeatureExtractionImpl>();
    ROS_INFO("\033[1;32m----> Feature Extraction Started.\033[0m");
  };

private:
  std::shared_ptr<FeatureExtractionImpl> FE;
};

//}

}  // namespace feature_extraction
}  // namespace liosam

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(liosam::feature_extraction::FeatureExtraction, nodelet::Nodelet)
