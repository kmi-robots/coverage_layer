#include "coverage_layer/coverage_layer.h"
#include "pluginlib/class_list_macros.h"
#include <stdexcept>

PLUGINLIB_EXPORT_CLASS(coverage_layer::CoverageLayer, costmap_2d::Layer)

namespace coverage_layer {
    
CoverageLayer::CoverageLayer() {}

void CoverageLayer::onInitialize() {
    ros::NodeHandle nh("~/" + name_);
    current_ = true;
    
    nh.param("coverage_limit", coverage_limit_, 0.6);
    nh.param("k_max", k_max_, 15);
    nh.param("v_max", v_max_, 100);
    nh.param("range", range_, 5.0);
    nh.param("field_of_view", theta_, 0.87);
    nh.param("view_threshold", view_threshold_, 100);
    
    dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = 
        boost::bind(&CoverageLayer::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
    
    CoverageLayer::matchSize();
    
    costmap_ = layered_costmap_->getCostmap()->getCharMap();
    ROS_INFO_STREAM("Coverage layer initialization complete");
}

void CoverageLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level) {
  enabled_ = config.enabled;
}

void CoverageLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y,
                                 double *max_x, double *max_y) { }

void CoverageLayer::updateCosts(costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j) { }

void CoverageLayer::activate() { }

void CoverageLayer::deactivate() { }

void CoverageLayer::reset() { }

std::set<unsigned int> CoverageLayer::generateBestViews(double wx1, double wy1, double wx2, double wy2) {  
    worldToMap(wx1, wy1, x1_, y1_);
    worldToMap(wx2, wy2, x2_, y2_);
    
    if(x1_ == x2_ || y1_ == y2_) { throw std::invalid_argument("null area"); }
    if(x1_ > x2_ || y1_ > y2_) { throw std::invalid_argument("wrong diagonal"); }
        
    std::map<unsigned int, int> *utilMap = new std::map<unsigned int, int>();
    
    ROS_INFO_STREAM("x1: "<<std::to_string(x1_)<<" y1: "<<std::to_string(y1_)
                    <<" x2: "<<std::to_string(x2_)<<" y2: "<<std::to_string(y2_));
    
    {
        std::default_random_engine generator;
        std::uniform_int_distribution<int> distribution(x1_, x2_);
        randX_ = std::bind(distribution, generator);
    }
    {
        std::default_random_engine generator;
        std::uniform_int_distribution<int> distribution(y1_, y2_);
        randY_ = std::bind(distribution, generator);
    }
    
    CellUtility ut(this, utilMap);        
    int viewArea = 0;
    for(int x = x1_; x < x2_; x++) {
        for(int y = y1_; y < y2_; y++) {
            if(getCost(x, y) == costmap_2d::FREE_SPACE) {
                viewArea++;
            }
        }
    }
    unsigned int dist = cellDistance(range_/10);
    ROS_INFO_STREAM("distance: "<<std::to_string(dist));
    for(int x = x1_ + dist; x < x2_; x += dist) {
        for(int y = y1_ + dist; y < y2_; y += dist) {
            if(getCost(x, y) == costmap_2d::FREE_SPACE) {
                ut.setStartingPoint(x, y);
                for(int k = 0; k < k_max_; k++) {
                    raytraceLine(ut, x, y, randX_(), randY_());
                }
            }
        }
    }
    
    std::vector<unsigned int> best_cells = ut.highestUtils(v_max_);
    
    ROS_INFO_STREAM("best cells: "<<best_cells.size());
    
    delete utilMap;
        
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(0, M_PI*2 - theta_);
    
    std::map<unsigned int, std::set<unsigned int>> setOfViews;
    std::map<unsigned int, int> totalView;
    std::set<unsigned int> minimalViews;
        
    for(auto const & element : best_cells) {
        unsigned int x, y;
        double wx, wy;
        indexToCells(element, x, y);
        mapToWorld(x, y, wx, wy);
        double th = distribution(generator);
        
        std::set<unsigned int> *fieldOfView = new std::set<unsigned int>();
        SensorUtility st(this, fieldOfView);
        
        for(int k = 0; k < 80; k++) {
            th = th + theta_/80;
            double pwx, pwy;
            pwx = wx + range_ * cos(th);
            pwy = wy + range_ * sin(th);
            unsigned int px, py;
            worldToMap(pwx, pwy, px, py);
            raytraceLine(st, x, y, px, py);
        }
        if(st.fieldOfViewSize() > view_threshold_) {
            minimalViews.insert(element);
            setOfViews[element] = st.getFieldOfView();
            for(auto const & sow: setOfViews[element]) {
                totalView[sow] = totalView[sow] + 1;
            }
        }
        
        delete fieldOfView;
    }
    
    ROS_INFO_STREAM("Coverage: "<<double(totalView.size())/double(viewArea));
    for(auto const & element : setOfViews) {
        std::map<unsigned int, int> result(totalView);
        
        for(auto & v : element.second) {
            result[v] = result[v] - 1;
            if(result[v] < 1)
                result.erase(v);
        }
        if(result.size() > coverage_limit_ * viewArea) {
            totalView = result;
            minimalViews.erase(element.first);
        }
    }
    
//     return minimalViews;
    ROS_INFO_STREAM("number of views: "<<minimalViews.size());
    std::set<unsigned int> coverage;
    for(auto const & tv: totalView) {
        coverage.insert(tv.first);
    }
//     return coverage;
    return minimalViews;
}

}
