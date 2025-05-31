#ifndef _POTENTIAL_D_CALCULATOR_H
#define _POTENTIAL_D_CALCULATOR_H

#include <algorithm>
#include <vector>
#include <global_planner/potential_calculator.h>

namespace masterarbeit {

class PotentialDCalculator : public global_planner::PotentialCalculator {
public:
    PotentialDCalculator(int nx, int ny) : global_planner::PotentialCalculator(nx, ny) {}
    ~PotentialDCalculator() {}
    float calculatePotential(float* potential, unsigned char cost, int n, float prev_potential = -1){
        if (prev_potential < 0) {
            // get min of neighbors
            float min_h = std::min(potential[n - 1], potential[n + 1]),
            min_v = std::min(potential[n - nx_], potential[n + nx_]),
            min_t = std::min(potential[n - nx_ - 1], potential[n - nx_ + 1]),
            min_d = std::min(potential[n + nx_ - 1], potential[n + nx_ + 1]),
            prev_potential_hv = std::min(min_h, min_v),
            prev_potential_td = std::min(min_t, min_d);
            prev_potential = std::min(prev_potential_hv, prev_potential_td);
        }

        return prev_potential + cost;
    }
};

} // end namespace global_planner

#endif // _POTENTIAL_D_CALCULATOR_H
