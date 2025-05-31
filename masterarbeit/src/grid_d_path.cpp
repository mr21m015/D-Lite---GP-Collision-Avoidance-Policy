#include "masterarbeit/grid_d_path.h"
#include <stdio.h>
#include <limits.h>
#include <vector>
#include <utility> 
#include <algorithm> 
#include <cmath> 
#include <ros/console.h>

namespace masterarbeit
{
  
bool GridPath::getPath(float* potential, double start_x, double start_y, double end_x, double end_y, std::vector<std::pair<float, float>>& path)
{
    return findPath(potential, start_x, start_y, end_x, end_y, path, false);
}

bool GridPath::getAlternativePath(float* potential, double start_x, double start_y, double end_x, double end_y, std::vector<std::pair<float, float>>& path)
{
    return findPath(potential, start_x, start_y, end_x, end_y, path, true);
}

bool GridPath::findPath(float* potential, double start_x, double start_y, double end_x, double end_y, std::vector<std::pair<float, float>>& path, bool is_alternative)
{
    path.clear();
    std::pair<float, float> current;
    current.first = start_x;
    current.second = start_y;

    int goal_index = getIndex(end_x, end_y);
    path.push_back(current);

    int c = 0;
    int ns = xs_ * ys_;

    while (getIndex(current.first, current.second) != goal_index)
    {
        double min_val = 1e10;
        int min_x = 0, min_y = 0;
        float dist = 1e10;
        int search_radius = 1;
        bool found_valid_potential = false;

        while (search_radius < 10 && !found_valid_potential)
        {
            for (int xd = -search_radius; xd <= search_radius; xd++)
            {
                for (int yd = -search_radius; yd <= search_radius; yd++)
                {
                    if (xd == 0 && yd == 0)
                        continue;

                    int x = current.first + xd;
                    int y = current.second + yd;
                    int index = getIndex(x, y);
                    float dist_temp = std::sqrt(std::pow(end_x - x,2) + std::pow(end_y - y, 2));

                    if (index < 0 || index >= ns)
                        continue;

                    if ((is_alternative && potential[index] <= min_val && dist_temp <= dist) || (!is_alternative && potential[index] < min_val && dist_temp <= dist))
                    {
                        min_val = potential[index];
                        min_x = x;
                        min_y = y;
                        found_valid_potential = true;
                        dist = dist_temp;
                    }
                }
            }

            if (!found_valid_potential)
            {
                search_radius++;
            }
        }

        if (!found_valid_potential) 
        {
            return false;
        }

        current.first = min_x;
        current.second = min_y;
        path.push_back(current);

        if (c++ > ns / 148)
        {
            return false;
        }
    }
    return !path.empty();
}



} //end namespace masterarbeit