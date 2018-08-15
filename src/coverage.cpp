#include "coverage/coverage.h"
#include "coverage_layer/coverage_layer.h"
#include "nav_msgs/OccupancyGrid.h"


namespace coverage {
Coverage::Coverage(tf::TransformListener &tf) : tf_(tf), costmap_ros_(NULL) {
    ros::NodeHandle nh;
    service_ = nh.advertiseService("/start", &Coverage::doSomething, this);
    map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("/map_view_points", 1, true);

    costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
    costmap_ros_->start();
    ROS_INFO_STREAM("Ready");
}

bool Coverage::doSomething(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    costmap_2d::LayeredCostmap *layers;
    layers = costmap_ros_->getLayeredCostmap();

    std::set<unsigned int> result;

    for (auto const &plugin : *layers->getPlugins()) {
        ROS_INFO_STREAM(plugin->getName());
        if (plugin->getName() == "global_costmap/coverage_layer") {
            ros::Duration(10.0).sleep();
            ROS_INFO_STREAM("Sleeping done");
            boost::shared_ptr<coverage_layer::CoverageLayer> l =
                boost::static_pointer_cast<coverage_layer::CoverageLayer>(plugin);
//             result = l->generateBestViews(3.0, 5.0, 9.0, 9.0);
                std::vector<std::pair<double, double>> area = {
                    std::make_pair(0.5, 0.5),
                    std::make_pair(0.5, 9.5),
                    std::make_pair(9.5, 9.5),
                    std::make_pair(9.5, 0.5)
                };
                result = l->generateBestViews(area);
        }
    }


    nav_msgs::OccupancyGrid msg;
    std::vector<int8_t> map(200 * 200, 0);

    for (auto const &value : result) {
        map[value] = 100;
    }

    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    msg.data = map;
    msg.info.resolution = 0.05;
    msg.info.width = 200;
    msg.info.height = 200;
    msg.info.origin.orientation.w = 1;

    map_pub_.publish(msg);

    costmap_ros_->stop();

    return true;
}
};
