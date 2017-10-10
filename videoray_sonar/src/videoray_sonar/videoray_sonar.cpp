#include "ros/ros.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "tritech_micron/tritech_micron.h"
#include <iostream>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <math.h>

typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;

class TritechMicronNode
{
public:
    TritechMicronNode(ros::NodeHandle &nh, std::string frame_id, std::string topic, std::string dev, int nbins, double range, double VOS, int angle_step, int left_limit, int right_limit, bool cont):
        nh_(nh),
        scanline(new PointCloud),
        pub_(nh_.advertise<PointCloud>(topic, 10)) {
        micron_.scanLinePub = boost::bind(&TritechMicronNode::scanLinePub, this, _1);
        scanline->header.frame_id = frame_id;
        if (!micron_.connect(dev)) {
            ROS_ERROR_STREAM("Error connecting sonar USB port, exit...");
            ros::shutdown();
        }
        micron_.configure(nbins, range, VOS, angle_step, left_limit, right_limit, cont);
        micron_.start();
    }
    void scanLinePub(const tritech::mtHeadDataMsg &);
private:
    ros::NodeHandle nh_;
    PointCloud::Ptr scanline;
    ros::Publisher pub_;

    TritechMicron micron_;
};

void TritechMicronNode::scanLinePub(const tritech::mtHeadDataMsg &msg)
{
    double deg = msg.bearing_degrees / 180.0 * M_PI;
    int n = msg.scanLine.size();
    scanline->width = n;
    scanline->height = 1;
    scanline->points.resize(scanline->width * scanline->height);
    double step = msg.rangeScale / n;
    for(int i=0; i<n; ++i) {
        scanline->points[i].x = (i+1)*step*cos(deg);
        scanline->points[i].y = (i+1)*step*sin(deg);
        scanline->points[i].z = 0;
        scanline->points[i].intensity = int(msg.scanLine[i]);
    }
    scanline->header.stamp = ros::Time::now().toNSec()/1e3;
    pub_.publish(scanline);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "videoray_sonar");
    ros::NodeHandle nh("~");
    std::string frame_id;
    std::string topic;
    std::string port;
    int nbins;
    double range;
    double VOS;
    int angle_step;
    int left_limit;
    int right_limit;
    bool cont;
    nh.param<std::string>("frame_id", frame_id, "/sonar");
    nh.param<std::string>("topic", topic, "sonarscan");
    nh.param<std::string>("port", port, "/dev/ttyUSB1");
    nh.param("nbins", nbins, 200);
    nh.param("range", range, 5.0);
    nh.param("VOS", VOS, 1500.0);
    nh.param("angle_step", angle_step, 32);
    nh.param("left_limit", left_limit, 0);
    nh.param("right_limit", right_limit, 6399);
    nh.param("cont", cont, true);

    TritechMicronNode tmn(nh, frame_id, topic, port, nbins, range, VOS, angle_step, left_limit, right_limit, cont);
    ros::spin();
}
