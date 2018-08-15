#include "coverage_layer/coverage_layer.h"
#include "pluginlib/class_list_macros.h"
#include <stdexcept>

PLUGINLIB_EXPORT_CLASS(coverage_layer::CoverageLayer, costmap_2d::Layer)

namespace coverage_layer {
    
CoverageLayer::CoverageLayer() : views_() {
    views_ = views_.insert(views_.begin(), { {0.0, 0.0, 0.0} });
    
    std::random_device rd;
    std::default_random_engine generator(rd());
    std::uniform_real_distribution<double> distribution(0, M_PI*2);
    randTh_ = std::bind(distribution, generator);
}

void CoverageLayer::onInitialize() {
    ros::NodeHandle nh("~/" + name_);
    current_ = true;
    
    nh.param("coverage_limit", coverage_limit_, 0.6);
    nh.param("k_max", k_max_, 15);
    nh.param("v_max", v_max_, 100);
    nh.param("range", range_, 5.0);
    nh.param("field_of_view", theta_, 0.87);
    nh.param("view_threshold", view_threshold_, 100);
    
    cells_in_view_ = (range_ * theta_)/pow(getResolution(),2);
    
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
                                 double *max_x, double *max_y) {
    tree<std::array<double, 3>>::iterator robot_position = views_.begin();
    (*robot_position) = {robot_x, robot_y, robot_yaw};
}

void CoverageLayer::updateCosts(costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j) { }

void CoverageLayer::activate() { }

void CoverageLayer::deactivate() { }

void CoverageLayer::reset() { }

std::set<unsigned int> CoverageLayer::generateBestViews(std::vector<std::pair<double, double>> area) {
    
    std::vector<costmap_2d::MapLocation> cell_border;
    for(auto const & p : area) {
        costmap_2d::MapLocation ml;
        worldToMap(p.first, p.second, ml.x, ml.y);
        cell_border.push_back(ml);
    }
    std::vector<costmap_2d::MapLocation> cell_area;
    convexFillCells(cell_border, cell_area);
    
    int viewArea = 0;
    for(auto const & ca : cell_area) {
        if(getCost(ca.x, ca.y) == costmap_2d::FREE_SPACE) {
            cell_area_.insert(getIndex(ca.x, ca.y));
            viewArea++;
        }
    }
    
    tree<std::array<double, 3>>::iterator tr = views_.begin();
    
    std::array<double, 3> pos = *tr;
    cover(pos[0], pos[1], pos[2]);
    
    std::vector<double> orientations = checkViews(pos[0], pos[1]);
    for(auto const & o : orientations) {
        if(cover(pos[0], pos[1], o)) {
            views_.append_child(tr, { {pos[0], pos[1], o} });
        }
    }
    
    for(tree<std::array<double, 3>>::sibling_iterator it = views_.begin(tr); it != views_.end(tr); it++) {
        std::cout<<"depth: "<<views_.depth(it)<<" value: "<<(*it)[0]<<std::endl;
    }
    
    {
    
//     unsigned int current_x, current_y;
//     worldToMap(robot_position_[0], robot_position_[1], current_x, current_y);
//     double current_yaw_ = rotate(robot_position_[2], theta_/2);
    
//     std::set<unsigned int> *fieldOfView = new std::set<unsigned int>();
//     SensorUtility st(this, fieldOfView);
//     for(int k = 0; k < 80; k++) {
//         th = th + theta_/80;
//         double pwx, pwy;
//         pwx = wx + range_ * cos(th);
//         pwy = wy + range_ * sin(th);
//         unsigned int px, py;
//         worldToMap(pwx, pwy, px, py);
//         raytraceLine(st, x, y, px, py);
//     }
    
    /*
    if(st.fieldOfViewSize() > view_threshold_) {
        minimalViews.insert(element);
        setOfViews[element] = st.getFieldOfView();
        for(auto const & sow: setOfViews[element]) {
            totalView[sow] = totalView[sow] + 1;
        }
    }
    
    delete fieldOfView;
    
    
        
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
    return minimalViews;*/
}
}

bool CoverageLayer::cover(double x, double y, double th) {
    std::set<unsigned int> *fieldOfView = new std::set<unsigned int>();
    SensorUtility st(this, fieldOfView);
    th = rotate(th, theta_/2);

    for(int k = 0; k < 80; k++) {
        th = rotate(th, -theta_/80);
        double pwx, pwy;
        pwx = x + range_ * cos(th);
        pwy = y + range_ * sin(th);
        unsigned int px, py;
        worldToMap(pwx, pwy, px, py);
        raytraceLine(st, x, y, px, py);
    }
    
    std::set<unsigned int> v;
    if(fieldOfView->size() > cells_in_view_ * 0.5) {
        std::set_intersection(fieldOfView->begin(), fieldOfView->end(), 
                              cell_area_.begin(), cell_area_.end(), 
                              std::inserter(v, v.begin()));
        if(v.size() > cells_in_view_ * 0.5) {
            std::set<unsigned int> diff;
            std::set_difference(cell_area_.begin(), cell_area_.end(), 
                                v.begin(), v.end(), 
                                std::inserter(diff, diff.begin()));
            cell_area_ = diff;
            
            return true;
        }
    }
    
    return false;
}

std::vector<double> CoverageLayer::checkViews(double x, double y) {
    unsigned int cx, cy;
    std::vector<std::pair<int, double>> tmpr;
    std::vector<double> result;
    double threshold = (range_/sqrt(2)*getResolution())*2;
    worldToMap(x, y, cx, cy);
    for(int i = 0; i < (M_PI/theta_) * 2; i++) {
        double th = randTh_();
        int *utility = new int;
        ViewCheck vc(this, utility);
        
        for(int k = 0; k < 3; k++) {
            double pwx, pwy;
            pwx = x + range_ * cos(rotate(th, theta_*k/2));
            pwy = y + range_ * sin(rotate(th, theta_*k/2));
            unsigned int px, py;
            worldToMap(pwx, pwy, px, py);
            raytraceLine(vc, cx, cy, px, py);
        }
        
        if(*utility > threshold) {
            tmpr.push_back(std::make_pair(*utility, th));
        }
        
        delete utility;
    }
    
    sort(tmpr.begin(), tmpr.end());
    for(auto const & r: tmpr) {
        result.push_back(r.first);
    }
    
    return result;
}

double CoverageLayer::rotate(double angle, double rotation) {
    angle = std::fmod(angle, M_PI*2);
    if(angle < 0) angle = angle + M_PI*2;
    rotation = std::fmod(rotation, M_PI*2);
    
    double result = angle + rotation;
    if(result < 0) result = result + M_PI*2;
    
    return result;
}

}
