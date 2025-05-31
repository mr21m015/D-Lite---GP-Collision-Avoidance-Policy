#ifndef _GRID_D_PATH_H
#define _GRID_D_PATH_H
#include<vector>
#include<global_planner/traceback.h>

namespace masterarbeit {

class GridPath : public global_planner::Traceback 
{
public:
    GridPath(global_planner::PotentialCalculator* p_calc): global_planner::Traceback(p_calc){}
    virtual ~GridPath() {}
    bool getPath(float* potential, double start_x, double start_y, double end_x, double end_y, std::vector<std::pair<float, float> >& path);
    bool getAlternativePath(float* potential, double start_x, double start_y, double end_x, double end_y, std::vector<std::pair<float, float> >& path);

private:
    bool findPath(float* potential, double start_x, double start_y, double end_x, double end_y, std::vector<std::pair<float, float>>& path, bool is_alternative);
};

} //end namespace masterarbeit
#endif