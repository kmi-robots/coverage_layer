#ifndef COVERAGE_LAYER_H_
#define COVERAGE_LAYER_H_

#include "ros/ros.h"
#include "costmap_2d/costmap_layer.h"
#include "costmap_2d/layered_costmap.h"
#include "costmap_2d/GenericPluginConfig.h"
#include "dynamic_reconfigure/server.h"
#include "tree/tree.hh"

using costmap_2d::FREE_SPACE;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::INSCRIBED_INFLATED_OBSTACLE;

namespace coverage_layer {

class CoverageLayer : public costmap_2d::CostmapLayer {
    public:
        CoverageLayer();

        virtual void onInitialize();
        virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y,
                                  double *max_x, double *max_y);
        virtual void updateCosts(costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j);

        virtual void activate();
        virtual void deactivate();
        virtual void reset();

        std::set<unsigned int> generateBestViews(std::vector<std::pair<double, double>> area);
    private:
        std::vector<double> checkViews(double x, double y);
        bool cover(double x, double y, double th);
        double rotate(double angle, double rotation);
        
        std::function<double () > randTh_;
        double coverage_limit_, range_, theta_;
        int k_max_, v_max_, view_threshold_, cells_in_view_;
        std::set<unsigned int> cell_area_;
        tree<std::array<double, 3>> views_;
        
        dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
        void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
        
        class ViewCheck {
        public:
            ViewCheck(CoverageLayer *cl, int *utility) : cl_(cl), obstacle_(false), utility_(utility) { }
            inline void operator()(unsigned int offset) {
                unsigned int x, y;
                cl_->indexToCells(offset, x, y);
                if (obstacle_ || cl_->getCost(x, y) == costmap_2d::LETHAL_OBSTACLE) {
                    obstacle_ = true;
                } else {
                    *utility_++;
                }                
            }
        private:
            CoverageLayer *cl_;
            bool obstacle_;
            int *utility_;
        };

        class CellUtility {
            public:
                CellUtility(CoverageLayer *cl, std::map<unsigned int, int> *utilMap)
                : utilMap_(utilMap), cl_(cl), obstacle_(false) { }
                    
                void setStartingPoint(unsigned int x, unsigned int y) {
                    startingPoint_ = cl_->getIndex(x, y);
                }

                inline void operator()(unsigned int offset) {
                    unsigned int x, y;
                    cl_->indexToCells(offset, x, y);
                    if (obstacle_ || cl_->getCost(x, y) == costmap_2d::LETHAL_OBSTACLE) {
                        obstacle_ = true;
                        return;
                    }
//                     if (x > cl_->x1_ && x < cl_->x2_ && y > cl_->y1_ && y < cl_->y2_)
//                     (*utilMap_)[startingPoint_] = (*utilMap_)[startingPoint_] + 1;
                    (*utilMap_)[offset] = (*utilMap_)[offset] + 1;
                        
                };
                std::vector<unsigned int> highestUtils(int n) {
                    std::vector<std::pair<int, unsigned int>> by_util;
                    for (auto const &element : (*utilMap_)) {
                        by_util.push_back(std::make_pair(element.second, element.first));
                    }
                    std::sort(by_util.begin(), by_util.end());
                    ROS_INFO_STREAM("size: "<<by_util.size());
                    
                    std::vector<unsigned int> cell_list;
                    for (auto it = by_util.rbegin(); it != by_util.rbegin() + n/2; it++) {
                        cell_list.push_back(it->second);
                    }
                    for (auto it = by_util.begin(); it != by_util.begin() + n/2; it++) {
                        cell_list.push_back(it->second);
                    }
                    return cell_list;
                };
            private:
                std::map<unsigned int, int> *utilMap_;
                unsigned int startingPoint_;
                CoverageLayer *cl_;
                bool obstacle_;
        };

        class SensorUtility {
            public:
                SensorUtility(CoverageLayer *cl, std::set<unsigned int> *fieldOfView)
                    : fieldOfView_(fieldOfView), cl_(cl), obstacle_(false) { }

                inline void operator()(unsigned int offset) {
                    unsigned int x, y;
                    cl_->indexToCells(offset, x, y);
                    if (cl_->getCost(x, y) == costmap_2d::LETHAL_OBSTACLE || obstacle_) {
                        obstacle_ = true;
                    } else {
                        fieldOfView_->insert(offset);
                    }
                }

                std::set<unsigned int> getFieldOfView() {
                    return *fieldOfView_;
                }
            private:
                std::set<unsigned int> *fieldOfView_;
                CoverageLayer *cl_;
                bool obstacle_;
        };

};
}


#endif
