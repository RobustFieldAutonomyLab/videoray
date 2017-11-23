#include <ros/ros.h>
#include <videoray_msgs/ScanLine.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>

typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;

ros::Publisher pub;

void callback(const videoray_msgs::ScanLineConstPtr &msg) {
  PointCloud cloud;
  cloud.header.stamp = msg->stamp.toNSec() / 1e3;
  cloud.header.frame_id = "/odom";

  int n = msg->intensities.size();
  cloud.width = n;
  cloud.height = 1;
  cloud.points.resize(n);
  double step = msg->range_max / n;
  for (int i = 0; i < cloud.points.size(); ++i) {
    cloud.points[i].x = cos(msg->angle) * (i + 1) * step;
    cloud.points[i].y = sin(msg->angle) * (i + 1) * step;
    cloud.points[i].z = 0.0;
    cloud.points[i].intensity = msg->intensities[i];
  }

  pub.publish(cloud);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "scanline_to_pointcloud");
  ros::NodeHandle nh("~");
  std::string scanlineTopic = nh.param("from", std::string("sonarscan"));
  std::string pointcloudTopic = scanlineTopic + "_pointcloud";

  ros::Subscriber sub = nh.subscribe<>(scanlineTopic, 100, callback);
  pub = nh.advertise<PointCloud>(pointcloudTopic, 100, false);

  ros::spin();
}