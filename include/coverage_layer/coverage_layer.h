#ifndef COVERAGE_LAYER_H_
#define COVERAGE_LAYER_H_

#include "ros/ros.h"
#include "costmap_2d/costmap_layer.h"
#include "costmap_2d/layered_costmap.h"


namespace coverage_layer {

class CoverageLayer : public costmap_2d::CostmapLayer {
public:
    CoverageLayer();

    virtual ~CoverageLayer();
    virtual void onInitialize();
    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y,
                                double *max_x, double *max_y);
    virtual void updateCosts(costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j);

    virtual void activate();
    virtual void deactivate();
    virtual void reset();
    
    std::set<unsigned int> generateBestViews(double x1, double x2, double y1, double y2);
private:
    std::function<int () > rand_;
    
    class CellUtility {
    public:
        CellUtility(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2, CoverageLayer* cl) {
            x1 = x1_;
            y1 = y1_;
            x2 = x2_;
            y2 = y2_;
            cl_ = cl;
        }
        inline void operator()(unsigned int offset) {
            unsigned int x, y;
            cl_->indexToCells(offset, x, y);
            if(x >= x1_ && x <= x2_ && y >= y1_ && y <= y2_)
                utilMap_[offset] = utilMap_[offset] + 1;
        };
        std::vector<unsigned int> highestUtils(int n) {
            std::vector<std::pair<int, unsigned int>> by_util;
            for(auto const & element : utilMap_) {
                by_util.push_back(std::make_pair(element.second, element.first));
            }
            std::sort(by_util.begin(), by_util.end());
            
            std::vector<unsigned int> cell_list;
            for(auto it = by_util.end() - 1; it >= by_util.end() - n; --it) {
                cell_list.push_back(it->second);
            }
            return cell_list;
        };
    private:
        std::map<unsigned int, int> utilMap_;
        double x1_, x2_, y1_, y2_;
        CoverageLayer* cl_;
    };
    
    class SensorUtility {
    public:
        SensorUtility (unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2, CoverageLayer* cl) {
            x1 = x1_;
            y1 = y1_;
            x2 = x2_;
            y2 = y2_;
            cl_ = cl;
        }
        
        inline void operator()(unsigned int offset) {
            unsigned int x, y;
            cl_->indexToCells(offset, x, y);
            if(x >= x1_ && x <= x2_ && y >= y1_ && y <= y2_)
                fieldOfView_.insert(offset);
        }
        
        std::set<unsigned int> getFieldOfView() {
            return fieldOfView_;
        }
        
    private:
        std::set<unsigned int> fieldOfView_;
        double x1_, x2_, y1_, y2_;
        CoverageLayer* cl_;
    };
    
};
}


#endif
