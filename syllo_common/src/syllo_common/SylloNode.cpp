#include <iostream>
#include <fstream>
#include <string>

#include <vector>

#include "ros/ros.h"
#include "syllo_common/SylloNode.h"

using std::cout;
using std::endl;

SylloNode::SylloNode()
{
}

int SylloNode::init()
{
     namespace_ = ros::this_node::getNamespace();
     node_name_ = ros::this_node::getName();     
     
     ros::param::get("~tick_rate",tick_rate_);
     ros_tick_rate_ = new ros::Rate(tick_rate_);
     
     return 0;
}

int SylloNode::spin()
{
     ros::spinOnce();
     ros_tick_rate_->sleep();
     
     return 0;
}

int SylloNode::cleanup()
{
     if (ros_tick_rate_) {
          delete ros_tick_rate_;
     }
     return 0;
}

int SylloNode::get_param(const std::string &param, std::string &value)
{
     ros::param::get(param,value);
     return 0;
}

int SylloNode::get_param(const std::string &param, double &value)
{
     ros::param::get(param,value);
     return 0;
}
