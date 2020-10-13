#include <math.h>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

string save_data_dir = "/home/henryzh47/logging/LOCAL/2020-10-09-brandy-mine";
string sensor_path_name = "trajectory.ply";
string lidar_cloud_name = "pointcloud.ply";
string filePrefix = "";

bool logSensorPath = true;
double laserCloudSkipDis = 0.02;
int maxPointNum = 30000000;

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudInNNext(
    new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudInNext(
    new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudIn(
    new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(
    new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZHSV>::Ptr sensorPose(
    new pcl::PointCloud<pcl::PointXYZHSV>());

double laserTimeIn = 0;
double laserTimeInNext = 0;
double laserTimeInNNext = 0;

const int systemDelay = 10;
int systemInitCount = 0;
bool systemInited = false;
bool systemFinished = false;
int laserCloudTotalNum = 0;
double odometryTime = 0;

double curTime[8];
float curX[8], curY[8], curZ[8];
float lastX = 0, lastY = 0, lastZ = 0;
int odomeID = -1;
float accuDis = 0;

FILE *fileVar0, *fileVar1, *fileVar2;

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloud2) {
  if (!systemInited || systemFinished) {
    return;
  }

  laserTimeIn = laserTimeInNext;
  laserTimeInNext = laserTimeInNNext;
  laserTimeInNNext = laserCloud2->header.stamp.toSec();

  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudTemp = laserCloudIn;
  laserCloudIn = laserCloudInNext;
  laserCloudInNext = laserCloudInNNext;
  laserCloudInNNext = laserCloudTemp;

  laserCloudInNNext->clear();
  pcl::fromROSMsg(*laserCloud2, *laserCloudInNNext);

  int findOdomID = -1;
  bool findOdom = false;
  for (int i = 0; i < 8; i++) {
    if (fabs(laserTimeIn - curTime[i]) < 0.15 && laserTimeIn > 0.005) {
      findOdomID = i;
      findOdom = true;
      break;
    }
  }

  bool disPass = false;
  if (findOdom) {
    if (sqrt((curX[findOdomID] - lastX) * (curX[findOdomID] - lastX) +
             (curY[findOdomID] - lastY) * (curY[findOdomID] - lastY) +
             (curZ[findOdomID] - lastZ) * (curZ[findOdomID] - lastZ)) >
        laserCloudSkipDis) {
      lastX = curX[findOdomID];
      lastY = curY[findOdomID];
      lastZ = curZ[findOdomID];
      disPass = true;
    }
  }

  if (!(disPass || laserCloudSkipDis <= 0.0)) {
    return;
  }

  /*laserCloud->clear();
  pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;
  downSizeFilter.setLeafSize(0.5, 0.5, 0.5);
  downSizeFilter.setInputCloud(laserCloudIn);
  downSizeFilter.filter(*laserCloud);*/
  laserCloud = laserCloudIn;

  int laserCloudNum = laserCloud->points.size();
  laserCloudTotalNum += laserCloudNum;

  float val;
  for (int i = 0; i < laserCloudNum; i++) {
    if (laserCloud->points[i].x > -100000 && laserCloud->points[i].x < 100000 &&
        laserCloud->points[i].y > -100000 && laserCloud->points[i].y < 100000 &&
        laserCloud->points[i].z > -100000 && laserCloud->points[i].z < 100000) {
      val = laserCloud->points[i].x;
      fwrite(&val, 4, 1, fileVar0);
      val = laserCloud->points[i].y;
      fwrite(&val, 4, 1, fileVar0);
      val = laserCloud->points[i].z;
      fwrite(&val, 4, 1, fileVar0);
      val = laserCloud->points[i].intensity;
      fwrite(&val, 4, 1, fileVar0);
    } else {
      laserCloudTotalNum--;
    }
  }

  printf("Points received: %d\n", laserCloudTotalNum);
}

void odometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry) {
  if (!systemInited) {
    systemInitCount++;
    if (systemInitCount > systemDelay) {
      systemInited = true;
    }
  }

  if (systemFinished) {
    return;
  }

  odometryTime = ros::Time::now().toSec();

  if (systemInited) {
    float disX = laserOdometry->pose.pose.position.x - curX[odomeID];
    float disY = laserOdometry->pose.pose.position.y - curY[odomeID];
    float disZ = laserOdometry->pose.pose.position.z - curZ[odomeID];
    accuDis += sqrt(disX * disX + disY * disY + disZ * disZ);
  }

  odomeID = (odomeID + 1) % 8;
  curTime[odomeID] = laserOdometry->header.stamp.toSec();
  curX[odomeID] = laserOdometry->pose.pose.position.x;
  curY[odomeID] = laserOdometry->pose.pose.position.y;
  curZ[odomeID] = laserOdometry->pose.pose.position.z;

  if (logSensorPath && systemInited) {
    double roll, pitch, yaw;
    geometry_msgs::Quaternion geoQuat = laserOdometry->pose.pose.orientation;
    tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w))
        .getRPY(roll, pitch, yaw);

    pcl::PointXYZHSV pose;
    pose.x = laserOdometry->pose.pose.position.z;
    pose.y = laserOdometry->pose.pose.position.x;
    pose.z = laserOdometry->pose.pose.position.y;
    pose.h = roll;
    pose.s = -pitch;
    pose.v = -yaw;

    sensorPose->push_back(pose);
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "velodyneCloudReceiver");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  ros::Subscriber laserCloudSub = nh.subscribe<sensor_msgs::PointCloud2>(
      "/velodyne_cloud_registered_imu", 20, laserCloudHandler);

  ros::Subscriber odometrySub = nh.subscribe<nav_msgs::Odometry>(
      "/aft_mapped_to_init_imu", 50, odometryHandler);

  nhPrivate.getParam("logSensorPath", logSensorPath);
  nhPrivate.getParam("laserCloudSkipDis", laserCloudSkipDis);
  nhPrivate.getParam("maxPointNum", maxPointNum);
  nhPrivate.getParam("filePrefix", filePrefix);

  string name0 =
      save_data_dir + "/" + filePrefix + "_" + lidar_cloud_name + ".temp";
  string name1 = save_data_dir + "/" + filePrefix + "_" + lidar_cloud_name;

  fileVar0 = fopen(name0.c_str(), "wb");

  printf("\nAwaiting for data\n\n");

  ros::Rate rate(100);
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();

    double timeDiff = ros::Time::now().toSec() - odometryTime;
    if (systemInited && timeDiff > 20.0) {
      systemFinished = true;
      break;
    }

    status = ros::ok();
    rate.sleep();
  }

  fclose(fileVar0);

  if (logSensorPath) {
    int sensorPoseNum = sensorPose->points.size();

    string name2 = save_data_dir + "/" + filePrefix + "_" + sensor_path_name;
    fileVar2 = fopen(name2.c_str(), "w");

    fprintf(fileVar2, "ply\n");
    fprintf(fileVar2, "format ascii 1.0\n");
    fprintf(fileVar2, "element vertex %d\n", sensorPoseNum);
    fprintf(fileVar2, "property float x\n");
    fprintf(fileVar2, "property float y\n");
    fprintf(fileVar2, "property float z\n");
    fprintf(fileVar2, "property float roll\n");
    fprintf(fileVar2, "property float pitch\n");
    fprintf(fileVar2, "property float yaw\n");
    fprintf(fileVar2, "end_header\n");

    for (int i = 0; i < sensorPoseNum; i++) {
      fprintf(fileVar2, "%f %f %f %f %f %f\n", sensorPose->points[i].x,
              sensorPose->points[i].y, sensorPose->points[i].z,
              sensorPose->points[i].h, sensorPose->points[i].s,
              sensorPose->points[i].v);
    }

    fclose(fileVar2);

    printf("\n%.1f meter trajectory saved\n", accuDis);
  }

  printf("\nDouble checking point cloud\n");

  bool downsizeCloud = false;
  double downsizeRate = double(maxPointNum + 1) / double(laserCloudTotalNum);
  int finalCloudTotalNum = laserCloudTotalNum;
  if (laserCloudTotalNum > maxPointNum && maxPointNum >= 0) {
    finalCloudTotalNum = maxPointNum;
    downsizeCloud = true;
  }

  fileVar0 = fopen(name0.c_str(), "rb");
  fileVar1 = fopen(name1.c_str(), "w");

  fprintf(fileVar1, "ply\n");
  fprintf(fileVar1, "format ascii 1.0\n");
  fprintf(fileVar1, "element vertex %d\n", finalCloudTotalNum);
  fprintf(fileVar1, "property float x\n");
  fprintf(fileVar1, "property float y\n");
  fprintf(fileVar1, "property float z\n");
  fprintf(fileVar1, "property float intensity\n");
  fprintf(fileVar1, "end_header\n");

  int size0, size1, size2, size3;
  float val0, val1, val2, val3;
  int finalCloudTotalCount = 0;
  for (int i = 0; i < laserCloudTotalNum; i++) {
    size0 = fread(&val0, 4, 1, fileVar0);
    size1 = fread(&val1, 4, 1, fileVar0);
    size2 = fread(&val2, 4, 1, fileVar0);
    size3 = fread(&val3, 4, 1, fileVar0);

    if (size0 != 1 || size1 != 1 || size2 != 1 || size3 != 1) {
      printf("\nPoint number error\nPoints received: %d\nPoints kept: %d\n",
             laserCloudTotalNum, i);
      break;
    }

    if (downsizeCloud) {
      double rate = double(finalCloudTotalCount) / double(i + 1);
      if (rate <= downsizeRate && finalCloudTotalCount < maxPointNum) {
        fprintf(fileVar1, "%f %f %f %f\n", val2, val0, val1, val3);
        finalCloudTotalCount++;
      }
    } else {
      fprintf(fileVar1, "%f %f %f %f\n", val2, val0, val1, val3);
      finalCloudTotalCount++;
    }
  }

  fclose(fileVar0);
  fclose(fileVar1);

  remove(name0.c_str());

  if (finalCloudTotalCount != finalCloudTotalNum) {
    printf(
        "\nPoint number error\nPoints received: %d\nPoints to save: %d\nPoints "
        "saved: %d\n",
        laserCloudTotalNum, finalCloudTotalNum, finalCloudTotalCount);
  }

  printf("\n%d points saved in point cloud\n\n", finalCloudTotalCount);

  return 0;
}
