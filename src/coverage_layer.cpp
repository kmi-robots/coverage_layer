#include "coverage_layer/coverage_layer.h"

// TODO params
#define KMAX 15
#define VMAX 10
#define R 5.0
#define THETA 0.87
#define DEPTH 5.0

using costmap_2d::FREE_SPACE;

namespace coverage_layer {

void CoverageLayer::onInitialize() {
    std::default_random_engine generator;
    std::uniform_int_distribution<int> distribution(-5, 5);
    rand_ = std::bind(distribution, generator);
}

void CoverageLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y,
                                 double *max_x, double *max_y) { }

void CoverageLayer::updateCosts(costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j) { }

void CoverageLayer::activate() { }

void CoverageLayer::deactivate() { }

void CoverageLayer::reset() { }

std::set<unsigned int> CoverageLayer::generateBestViews(double wx1, double wy1, double wx2, double wy2) {
    unsigned int x1, y1, x2, y2;
    worldToMap(wx1, wy1, x1, y1);
    worldToMap(wx2, wy2, x2, y2);

    CellUtility ut(x1, y1, x2, y2, this);
    
    int viewArea = 0;
    
    for(int x = x1; x <= x2; x++) {
        for(int y = y1; y <= y2; y++) {
            if(getCost(x, y) == costmap_2d::FREE_SPACE) {
                viewArea++;
                for(int k = KMAX; k > 0; k++) {
                    raytraceLine(ut, x, y, x + rand_(), y + rand_(), DEPTH);
                }
            }
        }
    }
    
    std::vector<unsigned int> best_cells = ut.highestUtils(VMAX);
    
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(0, M_PI*2 - THETA);
    
    std::map<unsigned int, std::set<unsigned int>> setOfViews;
    std::set<unsigned int> totalView;
    std::set<unsigned int> minimalViews;
        
    for(auto const & element : best_cells) {
        unsigned int x,y;
        double wx, wy;
        indexToCells(element, x, y);
        mapToWorld(x, y, wx, wy);
        double th = distribution(generator);
        
        SensorUtility st(x1, y1, x2, y2, this);
        
        for(int k = 0;k < 10;k++) {
            th = th + k * THETA/10;
            double pwx, pwy;
            pwx = R * cos(th);
            pwy = R * sin(th);
            unsigned int px, py;
            worldToMap(pwx, pwy, px, py);
            raytraceLine(st, x, y, px, py, DEPTH);
        }
        minimalViews.insert(element);
        setOfViews[element] = st.getFieldOfView();
        totalView.insert(setOfViews[element].begin(), setOfViews[element].end());
    }
    
        
    for(auto const & element : setOfViews) {
        std::set<unsigned int> result;
        std::set_difference(totalView.begin(), totalView.end(), element.second.begin(), element.second.end(),
                            std::inserter(result, result.end()));
        if(result.size() > 0.9 * viewArea) {
            totalView = result;
            minimalViews.erase(element.first);
        }
    }
    
    return minimalViews;
}

}
