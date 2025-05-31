#include "masterarbeit/dstar.h"
#include "masterarbeit/planner_d_core.h"

#include <algorithm>
#include <cmath>    
#include <limits>  
#include <random>              
#include <queue>                
#include <unordered_set>        

#include <ros/ros.h>
#include <ros/console.h>
#include <costmap_2d/cost_values.h> 
#include <costmap_2d/costmap_2d.h> 

namespace masterarbeit {

    DStarExpansion::DStarExpansion(global_planner::PotentialCalculator* p_calc, int nx, int ny, unsigned char* costmap)
    : Expander(p_calc, nx, ny), k_m(0), costmap_(costmap)
{
    ros::NodeHandle nh("~");
    nh.param("robot", robot, std::string(""));
    goalNode = nullptr;
    goalNR = 0;
    init = false;
    cost_node.resize(nx*ny);
}

DStarExpansion::~DStarExpansion() {

    for (auto& entry : nodeMap)
    {
        delete entry.second;
    }
    nodeMap.clear();

    if (startNode != nullptr && nodeMap.find(startNode->i) == nodeMap.end()) {
        delete startNode;
    }
    if (goalNode != nullptr && nodeMap.find(goalNode->i) == nodeMap.end()) {
        delete goalNode;
    }

    openList.clear();
}

void DStarExpansion::initialize(unsigned char* costs, int start_i_, int goal_i_, float* potential) {

    dStarPath.clear();
    openList.clear(); 
    nodeMap.clear();
    
    goalNR++;
    
    ROS_WARN_STREAM(robot << " is approaching goal nr.: " << goalNR);
    k_m = 0.0;
    std::fill(potential, potential + ns_, POT_HIGH);
    potential[goal_i] = 0;
    for (int i = 0; i < nx_ * ny_; i++)
    {
        cost_node[i] = costs[i];
        DStarNode* node = new DStarNode(potential[i], cost_node[i], i);
        nodeMap[i] = node;
        if(i == start_i_)
        {
            startNode = node;
            lastStartNode = node;
        }
        else if(i == goal_i_)
        {
            goalNode = node;
            goalNode->rhs = 0;
        }
    }

    for(int i = 0; i < nx_ * ny_; i++)
    {
        nodeMap[i]->nbrs = getNeighbors(i);
    }

    goalNode->key = calculateKey(goalNode);
    openList.push_back(goalNode);
    
}

bool DStarExpansion::checkCostChanges(unsigned char* costs)
{
    bool changed = false;
    k_m += heuristic(lastStartNode->i, startNode->i);
    lastStartNode = deepCopyNode(startNode);
    
    for (int i = 0; i < nx_ * ny_; i++)
    {
        
        if (cost_node[i] != costmap_[i])
        {
            if(!changed)
            {
                changed = true;
            }
            updateCosts(costmap_[i], nodeMap[i]);
        }
    }
    return changed;
}


bool DStarExpansion::calculatePotentials(unsigned char* costs, double start_x, double start_y,
                                        double end_x, double end_y, int cycles, float* potential) {
    
    start_i = toIndex(start_x, start_y);
    goal_i = toIndex(end_x, end_y);
    
    if(goalNode == nullptr || goal_i != goalNode->i) 
    {
        init = false;
        init_start_x = start_x;
        init_start_y = start_y;
        initialize(costs, start_i, goal_i, potential);
        computeShortestPath(cycles);
        return true; 
    }
    else
    {
        startNode = nodeMap[start_i];
        if(!checkCostChanges(costs)) 
        { 
            return true;
        }
        else 
        {
            computeShortestPath(cycles);
            return true;
        }
    }
}

bool DStarExpansion::computeShortestPath(int cycles, bool is_alternative)
{
    DStarNode* current;
    is_alt = is_alternative;
    if(init) cycles = 2000;
    int cycle = 0;
    while (((*openList.begin())->key < calculateKey(startNode) || startNode->rhs > startNode->g) && cycle < cycles)
    {       
        current = *openList.begin();
        std::pair<float, float> k_old = current->key;
        std::pair<float, float> k_new = calculateKey(current);
        if (k_old < k_new)
        {
            current->key = k_new;
            auto it = std::find(openList.begin(), openList.end(), current);
            if (it != openList.end()) {
                openList.erase(it);
            }
            openList.push_back(current);
            std::make_heap(openList.begin(), openList.end(), DStarNodeCompare());
        }
        else if(current->g > current->rhs) 
        {
            current->g = current->rhs;
            auto it = std::find(openList.begin(), openList.end(), current);
            if (it != openList.end()) {
                openList.erase(it);
            }
            std::make_heap(openList.begin(), openList.end(), DStarNodeCompare());
            for (DStarNode* predecessor : current->nbrs)
            {
                if (predecessor->i != goalNode->i)
                { 
                    predecessor->rhs = std::min(predecessor->rhs, calculateCost(predecessor->i, current->i) + current->g);
                }
                updateVertex(predecessor);
            }
        }
        else
        {
            float g_old = current->g;
            current->g = POT_HIGH;
            float min_val = POT_HIGH;

            if(current->i != goalNode->i)
            {
                for (DStarNode* predecessor : current->nbrs)
                {
                    min_val = std::min(min_val,calculateCost(current->i, predecessor->i)+predecessor->g);
                }
                current->rhs = min_val;
            }
            updateVertex(current);

            for (DStarNode* predecessor : current->nbrs)
            {
                if(predecessor->rhs == calculateCost(current->i, predecessor->i) + g_old )
                {
                    if (predecessor->i != goalNode->i)
                    {
                        float min_val = POT_HIGH;
                        for (DStarNode* successor : predecessor->nbrs)
                        {
                            min_val = std::min(min_val,calculateCost(predecessor->i, successor->i)+successor->g);
                        }
                        predecessor->rhs = min_val;
                    }
                }
                updateVertex(predecessor);
            }
        }
        
        cycle++;
    }
    init = true;
    return true;
}

std::vector<DStarNode*> DStarExpansion::getNeighbors(int i)
{
    std::vector<DStarNode*> neighbors;
    std::vector<int> offsets = { 1, nx_ + 1, nx_, nx_ - 1, -1, -nx_ - 1, -nx_, -nx_ + 1 };

    int x = i % nx_;
    for (const auto& offset : offsets) {
        int neighborIndex = i + offset;
        int nx = neighborIndex % nx_;

        if (neighborIndex < 0 || neighborIndex >= nx_ * ny_) continue;
        if (std::abs(x - nx) > 1) continue;

        auto it = nodeMap.find(neighborIndex);
        if (it != nodeMap.end() && it->second != nullptr)
        {
            neighbors.push_back(it->second);
        }

    }
    return neighbors;
}


std::vector<DStarNode*> DStarExpansion::getNeighborsMinG(std::vector<DStarNode*> nbrs) {
    float best = POT_HIGH;
    std::vector<DStarNode*> out;

    for (auto* n : nbrs) 
    {
        if (n->rhs == n->g && n->g < best) 
        {
            best = n->g;
            out.clear();
            out.push_back(n);
        }
    }
    return out;
}


void DStarExpansion::updateVertex(DStarNode* u)
{    
    auto it = std::find_if(openList.begin(), openList.end(), [u](const DStarNode* n) { return n->i == u->i; });
    bool wasInOpenList = (it != openList.end());

    if (u->g != u->rhs)
    {
        if (wasInOpenList)
        {      
            u->key = calculateKey(u);
            std::make_heap(openList.begin(), openList.end(), DStarNodeCompare());
        }
        else
        {
            u->key = calculateKey(u);
            openList.push_back(u);
            std::push_heap(openList.begin(), openList.end(), DStarNodeCompare());
        }
    } 
    else if (wasInOpenList) 
    {
        openList.erase(it);
        std::make_heap(openList.begin(), openList.end(), DStarNodeCompare());
    }
}

std::pair<float, float> DStarExpansion::calculateKey(DStarNode* s) {
    float min_g_rhs = std::min(s->g, s->rhs);
    float h_s;
    if(s->i == startNode->i && !is_alt) h_s = heuristic(s->i, startNode->i) + 10 ;
    else if(s->i == startNode->i && !is_alt) h_s = heuristic(s->i, startNode->i) + 30 ;
    else  h_s = heuristic(s->i, startNode->i);
    return std::make_pair(min_g_rhs + h_s + k_m, min_g_rhs);
}

float DStarExpansion::heuristic(int i_u, int i_v) 
{
    int node_x = i_u % nx_;
    int node_y = i_u / nx_;
    int start_x = i_v % nx_;
    int start_y = i_v / nx_;

    return std::abs(node_x - start_x) + std::abs(node_y - start_y);
}

float DStarExpansion::calculateCost(int i_u, int i_v) {

    if ((cost_node[i_u] >= 253 && cost_node[i_u] != costmap_2d::NO_INFORMATION) ||
        (cost_node[i_v] >= 253 && cost_node[i_v] != costmap_2d::NO_INFORMATION)) {
        return POT_HIGH;
    }
    
    float dist = heuristic(i_u, i_v);

    float cost = (cost_node[i_u] + cost_node[i_v]) / 2.0f;
    float cost_weight = 1.5f; 

    return dist + cost_weight * (cost / 255.0f);
}

bool DStarExpansion::updateCosts(unsigned char cost_new, DStarNode* node)
{
    unsigned char cost_old;
    float tmp_cost_old;
    float tmp_g;
    
    cost_old = node->cost;
    for (auto v : node->nbrs)
    {
        node->cost = cost_old;
        float tmp_cost_old = calculateCost(node->i, v->i);
        node->cost = cost_new;
        float tmp_cost_new = calculateCost(node->i, v->i);
        tmp_g = v->g;

        if (tmp_cost_old > tmp_cost_new)
        {
            if (node->i != goalNode->i)
            {
                node->rhs = std::min(node->rhs,tmp_cost_new + tmp_g);
            }
        } 
        else if (node->rhs == tmp_cost_old + tmp_g)
        {
            if(node->i != goalNode->i)
            {
                float min_val = POT_HIGH;
                for (DStarNode* successor : node->nbrs)
                {
                    min_val = std::min(min_val,calculateCost(node->i, successor->i)+successor->g);
                }
                node->rhs = min_val;
            }
        }
        updateVertex(node);
    }

    //Update neighbors  
    for (auto v : node->nbrs)
    {
        node->cost = cost_old;
        float tmp_cost_old = calculateCost(node->i, v->i);
        node->cost = cost_new;
        float tmp_cost_new = calculateCost(node->i, v->i);
        tmp_g = node->g;

        if (tmp_cost_old > tmp_cost_new)
        {
            if (v->i != goalNode->i)
            {
                v->rhs = std::min(v->rhs,tmp_cost_new + tmp_g);
            }
        } 
        else if (v->rhs == tmp_cost_old + tmp_g)
        {
            if(v->i != goalNode->i)
            {
                float min_val = POT_HIGH;
                for (DStarNode* successor : v->nbrs)
                {
                    min_val = std::min(min_val,calculateCost(v->i, successor->i)+successor->g);
                }
                v->rhs = min_val;
            }
        }
        updateVertex(v);
    }
    return true;
}

bool DStarExpansion::updateStartAndGoalNodes(double start_x, double start_y, double end_x, double end_y) {
    start_i = toIndex(start_x, start_y);
    goal_i = toIndex(end_x, end_y);

    auto start_it = nodeMap.find(start_i);
    auto goal_it = nodeMap.find(goal_i);

    if (start_it == nodeMap.end() || goal_it == nodeMap.end()) {
        ROS_WARN_STREAM(robot << " Start- oder Zielknoten nicht im nodeMap gefunden!");
        return false;
    }

    startNode = start_it->second;
    goalNode = goal_it->second;
    return true;
}

float DStarExpansion::calculateEuclideanDistance(float x1, float y1, float x2, float y2)
{
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}


bool DStarExpansion::getPath(float* potential, double start_x, double start_y,
    double end_x, double end_y, std::vector<std::pair<float, float>>& path)
{
    if (!updateStartAndGoalNodes(start_x, start_y, end_x, end_y))
        return false;

    return findPath(start_x, start_y, end_x, end_y, path);
}

bool DStarExpansion::findPath(double start_x, double start_y, double end_x, double end_y,
    std::vector<std::pair<float, float>>& path, bool is_alternative, const std::pair<float,float>* evasion_vector)
{
    if (goalNode == nullptr || goalNode->rhs != 0)
        return false;

    std::vector<int> int_dStarPath;
    if (!is_alternative) dStarPath.clear();
    
    DStarNode* node = nodeMap[toIndex(start_x, start_y)];

    // Falls Start ungültig → Alternative suchen
    if (!node || node->g == POT_HIGH || node->g != node->rhs)
    {
        if(is_alternative)
        {
            node = findNextNode(node, int_dStarPath, is_alternative, evasion_vector);
        }
        else
        {
            node = findNextNode(node, int_dStarPath);
        }
        
        if (!node || node->g == POT_HIGH || node->g != node->rhs) return false;
    }

    int cycle = 0;
    int repeat_count = 0;
    int last_index = -1;
    const int repeat_threshold = 5;

    while (node != goalNode && cycle < (nx_ + ny_) * 2)
    {
        DStarNode* next_node = nullptr;
        float min_cost = POT_HIGH;

        if (node->i == last_index) 
        {
            repeat_count++;
            if (repeat_count >= repeat_threshold) 
            {
                ROS_WARN("findPath: Punkt wiederholt sich %d mal hintereinander (Index: %d) → Abbruch", repeat_count, node->i);
                break;
            }
            else
            {
                auto nbrs = node->nbrs;
                for (auto* neighbor : nbrs) 
                {
                    if (neighbor->rhs == neighbor->g && neighbor->g != POT_HIGH)
                    {
                        next_node = neighbor;
                        break;
                    }
                }

            }
        } 
        else 
        {
            int_dStarPath.push_back(node->i);
            repeat_count = 0;
            for (DStarNode* neighbor : getNeighborsMinG(node->nbrs)) 
            {
                if (neighbor->rhs == neighbor->g && neighbor->rhs != POT_HIGH)
                {
                    float cost = neighbor->g;
                    if ((is_alternative && cost <= min_cost) || (!is_alternative && cost < min_cost)) 
                    {
                        min_cost = cost;
                        next_node = neighbor;
                    }
                }
            }
        }
        last_index = node->i;

        if (!next_node) 
        {
            next_node = findNextNode(node, int_dStarPath);
            if (!next_node) break;
        }

        node = next_node;
        ++cycle;
    }

    if (node == goalNode) int_dStarPath.push_back(goalNode->i);
    
    // Pfad aufbauen
    path.clear();
    
    for (int index : int_dStarPath)
    {
        int x = index % nx_;
        int y = index / nx_;
        path.emplace_back(static_cast<float>(x), static_cast<float>(y));
    }
    if(is_alternative && path.empty()) ROS_WARN_STREAM("Kein Pfad erzeugt.");
    // Startpunkt explizit am Anfang ergänzen
    int start_i = toIndex(start_x, start_y);
    if (!int_dStarPath.empty() && int_dStarPath.front() != start_i)
        path.insert(path.begin(), std::make_pair(static_cast<float>(start_x), static_cast<float>(start_y)));

    if (!is_alternative) dStarPath = path;

    return !path.empty();
}

DStarNode* DStarExpansion::findNextNode(DStarNode* current_node, std::vector<int>& dStar, bool is_alternative, const std::pair<float, float>* evasion_vector) 
{
    std::unordered_set<int> visited;
    std::queue<DStarNode*> q;
    q.push(current_node);
    visited.insert(current_node->i);

    while (!q.empty()) {
        DStarNode* node = q.front(); q.pop();

        if (node->rhs != POT_HIGH && node->g == node->rhs &&
            std::find(dStar.begin(), dStar.end(), node->i) == dStar.end()) 
        {
            return node;
        }
        if(!is_alternative)
        {
            for (DStarNode* neighbor : getNeighbors(node->i))
            {
                if (visited.find(neighbor->i) == visited.end())
                {
                    visited.insert(neighbor->i);
                    q.push(neighbor);
                }
            }
        }
        else
        {
            for (auto* nb : getNeighbors(node->i))
            {
                if (evasion_vector!=nullptr)
                {
                    int xi = node->i % nx_;
                    int yi = node->i / nx_;
                    int xj = nb->i   % nx_;
                    int yj = nb->i   / nx_;
                    float dx = xj - xi;
                    float dy = yj - yi;
                    float dot = dx * evasion_vector->first + dy * evasion_vector->second;

                    if (dot > 0.0f)
                        continue; 
                }
                if (!visited.count(nb->i)) {
                    visited.insert(nb->i);
                    q.push(nb);
                }
            }
        }
    }
    return nullptr;
}

bool DStarExpansion::getPathObstacleInteractionInfo(
    const PathType& path, int cx_map, int cy_map, double r_cells, double global_start_x, double global_start_y, double global_goal_x,  double global_goal_y,
    std::pair<float, float>& out_H_on_path, std::pair<float, float>& out_vec_C_to_H, std::pair<float, float>& out_evasion_vector, bool* out_C_is_left_of_SG_line)
{
    if (path.empty()) {
        ROS_DEBUG("getPathObstacleInteractionInfo: Pfad leer");
        return false;
    }

    const float Sx = static_cast<float>(init_start_x),
                Sy = static_cast<float>(init_start_y);
    const float Gx = static_cast<float>(global_goal_x),
                Gy = static_cast<float>(global_goal_y);
    const float Cx = static_cast<float>(cx_map),
                Cy = static_cast<float>(cy_map);


    float SGx = Gx - Sx, SGy = Gy - Sy;
    float normSG = std::hypot(SGx, SGy);
    if (normSG < 1e-3f) {
        ROS_WARN("SG-Vektor zu kurz");
        return false;
    }
    SGx = std::round(SGx / normSG);  SGy = std::round(SGy / normSG);

    float CSGx =  SGy;
    float CSGy = -SGx;

    float best_dot = 0.0f;
    int H_idx = -1;
    
    for (size_t j = 0; j < path.size(); ++j) 
    {
        float dx = path[j].first  - Cx;
        float dy = path[j].second - Cy;
        float dot = dx * CSGx + dy * CSGy;

        if (std::fabs(dot) > std::fabs(best_dot)) 
        {
            best_dot = dot;
            H_idx    = j;
        }
    }
    
    
    out_H_on_path = path[H_idx];
    out_vec_C_to_H = {
        out_H_on_path.first  - Cx,
        out_H_on_path.second - Cy
    };
    
    

    if (best_dot >= 0.0f) 
    {
        out_evasion_vector = { std::round(CSGx), std::round(CSGy) };
    } 
    else
    {
        out_evasion_vector = {std::round(-CSGx), std::round(-CSGy) };
    }
 
    float SCx = Cx - global_start_x;
    float SCy = Cy - global_start_y;
    float SHx = out_H_on_path.first  - Sx;
    float SHy = out_H_on_path.second - Sy;


    double cross_z = SCx * SHy - SCy * SHx;

    if (out_C_is_left_of_SG_line)
    {
        *out_C_is_left_of_SG_line = (cross_z > 0.0);
    }

    return true;
}

DStarExpansion::PathType DStarExpansion::getAlternativePath(const PathType& primary_path, double start_x_map, double start_y_map, double goal_x_map,  double goal_y_map,
                                                                int intersect_cx_map, int intersect_cy_map, double intersect_r_cells, bool* C_is_globally_left)
{

    if (primary_path.empty()) 
        return {};

    std::pair<float, float> H_p1, vec_C_to_H_p1, global_evasion_dir;
    bool gotInfo = getPathObstacleInteractionInfo(primary_path, intersect_cx_map, intersect_cy_map, intersect_r_cells,
        start_x_map, start_y_map, goal_x_map, goal_y_map,
        H_p1, vec_C_to_H_p1, global_evasion_dir, C_is_globally_left);

    if (!gotInfo)
    {
        ROS_WARN("getAlternativePath: Interaktionsinfos P1 fehlgeschlagen – Fallback Orthogonale.");
        float SGx = float(goal_x_map - start_x_map), SGy = float(goal_y_map - start_y_map);
        float normSG = std::hypot(SGx, SGy);
        if (normSG < 1e-3f) return {};
        SGx /= normSG;  SGy /= normSG;

        global_evasion_dir = { SGy, -SGx };
    }

    std::vector<std::pair<int, unsigned char>> modified_costs, original_costs;
    int penalty_zone_depth = int(hypot(vec_C_to_H_p1.first, vec_C_to_H_p1.second)*5);
    int penalty_zone_width = intersect_r_cells / 3;
    std::pair<float,float> ortho ={-global_evasion_dir.second, global_evasion_dir.first};

    for (int d = 0; d < penalty_zone_depth; ++d)
    {
        for (int w = -penalty_zone_width; w <= penalty_zone_width; ++w)
        {
            int cell_x = int(std::round(intersect_cx_map + d * global_evasion_dir.first + w * ortho.first));
            int cell_y = int(std::round(intersect_cy_map + d * global_evasion_dir.second + w * ortho.second));                
            modified_costs.emplace_back(toIndex(cell_x, cell_y),costmap_2d::LETHAL_OBSTACLE);
            original_costs.emplace_back(toIndex(cell_x, cell_y),nodeMap[toIndex(cell_x, cell_y)]->cost);

        }
    }

    int N = nx_*ny_;
    std::vector<int> old_indices;
    std::vector<float> old_g, old_rhs;
    for (int i = 0; i < N; ++i) 
    {
        if(nodeMap[i]->g == nodeMap[i]->rhs && nodeMap[i]->g != POT_HIGH)
        {
            old_g.push_back(nodeMap[i]->g);
            old_rhs.push_back(nodeMap[i]->rhs);
            old_indices.push_back(i);
        }
        
    }


    std::pair<float, float> inv_global_evasion_dir = {-global_evasion_dir.first, -global_evasion_dir.second};
    PathType alternative_path = calculatePotentialAlternative(modified_costs, nx_*ny_, start_x_map, start_y_map, goal_x_map,  goal_y_map, &inv_global_evasion_dir);

    if (!alternative_path.empty())
    {
        if (arePathsIdentical(alternative_path, primary_path))
        {   
            std::vector<int> new_indices;
            std::vector<float> new_g, new_rhs;
            for (int i = 0; i < N; ++i)
            {
                if(nodeMap[i]->g == nodeMap[i]->rhs && nodeMap[i]->g != POT_HIGH)
                {
                        
                    new_g.push_back(nodeMap[i]->g);
                    new_rhs.push_back(nodeMap[i]->rhs);
                    new_indices.push_back(i);
                }
            }

            // Zum Beispiel nur die Indices des gerade erzeugten Alternativpfades:
            std::vector<int> indices;
            for (auto& p : alternative_path)
            {
                int ix = int(std::round(p.first)), iy = int(std::round(p.second));
                indices.push_back(ix + iy*nx_);
            }
            
            modified_costs = original_costs;
            ortho ={-global_evasion_dir.second, global_evasion_dir.first};

            for (int d = 0; d < penalty_zone_depth; ++d)
            {
                for (int w = -penalty_zone_width; w <= penalty_zone_width; ++w)
                {
                    int cell_x = int(std::round(intersect_cx_map + d * inv_global_evasion_dir.first + w * ortho.first));
                    int cell_y = int(std::round(intersect_cy_map + d * inv_global_evasion_dir.second + w * ortho.second));
                    if (0 <= cell_x && cell_x < nx_ && 0 <= cell_y && cell_y < ny_)
                    {   
                        DStarNode* node = nodeMap[toIndex(cell_x, cell_y)];
                        if((node->g != POT_HIGH && node->g == node->rhs) || node->rhs != POT_HIGH)
                            modified_costs.emplace_back(node->i,costmap_2d::LETHAL_OBSTACLE);
                    }
                }
            }
            // 4) Neue Potential-Berechnung
            alternative_path = calculatePotentialAlternative(modified_costs, nx_*ny_, start_x_map, start_y_map, goal_x_map,  goal_y_map, &global_evasion_dir);
            if (!alternative_path.empty())
            {
                if (arePathsIdentical(alternative_path, primary_path))
                {
                    return{};
                }
            }
            return alternative_path;
        }
        else
            return alternative_path;
        
    }

    return {};
}

DStarExpansion::PathType DStarExpansion::calculatePotentialAlternative(std::vector<std::pair<int, unsigned char>>& modified_costs_list, int cycles, double start_x, double start_y, double end_x, double end_y, const std::pair<float,float>* evasion_vector)
{

    for (auto cost : modified_costs_list)
    {
        
        if (cost_node[cost.first] != cost.second)
        {
            updateCosts(cost.second, nodeMap[cost.first]);
        }
    }

    computeShortestPath(cycles,true);
    PathType alternativePath;
    bool is_available = findPath(start_x, start_y, end_x, end_y, alternativePath, true, evasion_vector);

    return alternativePath; 
}

bool DStarExpansion::arePathsIdentical(const PathType& p1, const PathType& p2) const
{
    for (size_t i = 0; i < p1.size(); ++i) 
    {
        int dx = std::abs(int(std::round(p1[i].first))  - int(std::round(p2[i].first)));
        int dy = std::abs(int(std::round(p1[i].second)) - int(std::round(p2[i].second)));
        if (dx > 3 || dy > 3) 
        {
            return false;
        }
    }
    return true;
}

}  // end namespace masterarbeit