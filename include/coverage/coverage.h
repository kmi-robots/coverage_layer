#ifndef COVERAGE_H_
#define COVERAGE_H_

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "costmap_2d/costmap_2d_ros.h"
#include "costmap_2d/costmap_2d.h"
#include "std_srvs/Empty.h"

namespace coverage {
class Coverage {
public:
    Coverage(tf::TransformListener &tf);
    bool doSomething(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
private:
    tf::TransformListener &tf_;
    costmap_2d::Costmap2DROS *costmap_ros_;
    ros::ServiceServer service_;
    ros::Publisher map_pub_;
};
};











#endif
