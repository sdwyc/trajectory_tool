#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

// #include <message_filters/subscriber.h>
// #include <message_filters/synchronizer.h>
// #include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Float32.h>
// #include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_listener.h>

#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

const double PI = 3.1415926;  

float transInterval = 0.2;   
float yawInterval = 10.0;   
pcl::PointCloud<pcl::PointXYZI>::Ptr trajectory(new pcl::PointCloud<pcl::PointXYZI>());  

float vehicleYaw = 0;
float currentYaw = 0;
float vehicleX = NAN, vehicleY = NAN, vehicleZ = NAN;
float currentX = 0, currentY = 0, currentZ = 0;
float travelingDis = 0; 
float ZDiff = 0;
float timeDuration = 0; 

string RobotBase;

tf::TransformListener *poseListenerPtr= NULL;
ros::Publisher *pubTrajectoryPtr = NULL;

void Record_pose(){

    tf::StampedTransform transform;
    tf::Quaternion cur_rotation;
    double roll, pitch, yaw;
    int  temp=0;
    while (temp==0 && ros::ok())
    {
      try
          {
              temp=1;
              poseListenerPtr->lookupTransform("map",RobotBase,ros::Time(0), transform);
          }
      catch (tf::TransformException& ex)
          {
              temp=0;
              cout << "Cannot Obtain robot pose!!" << endl;
              ros::Duration(0.1).sleep();
          }
      currentX = transform.getOrigin().x();
      currentY = transform.getOrigin().y();
      currentZ = transform.getOrigin().z();
      tf::Quaternion tfQuat = transform.getRotation();
      tf::Matrix3x3(tfQuat).getRPY(roll, pitch, yaw);
    }

  if(isnan(vehicleX) && isnan(vehicleY) && isnan(vehicleZ)){
    vehicleYaw = yaw;
    vehicleX = currentX;
    vehicleY = currentY;
    vehicleZ = currentZ;
    return;
  }

  float dYaw = fabs(yaw - vehicleYaw);
  if (dYaw > PI) dYaw = 2 * PI  - dYaw;

  float dx = currentX - vehicleX;
  float dy = currentY - vehicleY;
  float dz = currentZ - vehicleZ;
  float dis = sqrt(dx * dx + dy * dy + dz * dz);

  if (dis < transInterval && dYaw < yawInterval) {
    return;
  }
  
  
  ZDiff += fabs(dz);
  travelingDis += dis;

  vehicleYaw = yaw;
  vehicleX = currentX;
  vehicleY = currentY;
  vehicleZ = currentZ;

  pcl::PointXYZI point;
  point.x = vehicleX;
  point.y = vehicleY;
  point.z = vehicleZ;
  point.intensity = travelingDis;
  trajectory->push_back(point);

  sensor_msgs::PointCloud2 trajectory2;
  pcl::toROSMsg(*trajectory, trajectory2);
  trajectory2.header.stamp = ros::Time::now();
  trajectory2.header.frame_id = "map";
  pubTrajectoryPtr->publish(trajectory2);
  std::cout << "Total trajectory length: " << travelingDis << std::endl;
  std::cout << "Total Zdiff: " << ZDiff << std::endl;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_trajectoryRecorder");

  ros::NodeHandle nh;
  // ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  ros::param::param<float>("~/transInterval",transInterval,0.5);
  ros::param::param<float>("~/yawInterval",yawInterval,5.0);
  string base_frame="base_link";
  ros::param::param<std::string>("~/robot_base_frame",RobotBase,base_frame);

  ros::Publisher pubTrajectory = nh.advertise<sensor_msgs::PointCloud2> ("/trajectory", 5);
  pubTrajectoryPtr = &pubTrajectory;
  tf::TransformListener poseListener;
  poseListenerPtr = &poseListener;

  ros::Rate rate(200);
  bool status = ros::ok();
  poseListener.waitForTransform("map",RobotBase,ros::Time::now(),ros::Duration(0.5));
  while (status) {
    Record_pose();
    status = ros::ok();
    rate.sleep();
  }
  return 0;
}
