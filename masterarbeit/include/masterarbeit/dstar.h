#ifndef _DSTAR_H
#define _DSTAR_H

#include <global_planner/expander.h>
#include <limits>
#include <map>
#include <vector>
#include <utility>
#include <shared_mutex>
#include <fstream>
#include <sys/stat.h>

#include "masterarbeit/planner_d_core.h"




namespace masterarbeit {

class DStarNode {
public:
    DStarNode(float& g_, unsigned char& cost_, int index = -1, float rhs_ = POT_HIGH, std::pair<float, float> key_ = std::make_pair(0.0, 0.0)) :
        i(index), g(g_), rhs(rhs_), cost(cost_), key(key_)
        {}
    // Kopierkonstruktor
    DStarNode(const DStarNode& other) :
        i(other.i), g(other.g), rhs(other.rhs), cost(other.cost), key(other.key), nbrs(other.nbrs)
    {}

    // Zuweisungsoperator
    DStarNode& operator=(const DStarNode& other) {
        if (this != &other) {
            i = other.i;
            g = other.g;
            rhs = other.rhs;
            cost = other.cost;
            key = other.key;
            nbrs = other.nbrs;
            
        }
        return *this;
    }

    ~DStarNode() {}

    float& g;
    float rhs;
    unsigned char& cost;
    int i; // Koordinatenindex des Knotens im Grid
    std::pair<float, float> key; // Schlüssel für den Vergleich in der Open List
    std::vector<DStarNode*> nbrs;
};

struct DStarNodeCompare {
    bool operator()(const DStarNode* a, const DStarNode* b) const {
        if (a->key.first > b->key.first) {
            return true;
        } else if (a->key.first == b->key.first) {
            return a->key.second > b->key.second;
        } else {
            return false;
        }
    }
};


class DStarExpansion : public global_planner::Expander {
public:
    using PathType = std::vector<std::pair<float, float>>;
    DStarExpansion(global_planner::PotentialCalculator* p_calc, int nx, int ny, unsigned char* costmap);
    virtual ~DStarExpansion();

    bool calculatePotentials(unsigned char* costs, double start_x, double start_y,
    double end_x, double end_y, int cycles, float* potential);

    bool getPath(float* potential, double start_x, double start_y, double end_x, double end_y, std::vector<std::pair<float, float>>& path);
    PathType getAlternativePath(const PathType& primary_path,double start_x_map, double start_y_map, double goal_x_map, double goal_y_map,
    int intersect_cx_map, int intersect_cy_map, double intersect_r_cells, bool* out_C_is_left_of_SG_line =nullptr);
    
    std::vector<std::pair<float, float>> dStarPath;
    std::shared_mutex dStarPathMutex;
    bool init;
    

protected:
        std::vector<DStarNode*> getNeighborsMinG(std::vector<DStarNode*> nbrs);
private:
    

    void initialize(unsigned char* costs,int start_i, int goal_i, float* potential);
    DStarNode* deepCopyNode(DStarNode* original) { return new DStarNode(*original);}
    bool updateCosts(unsigned char cost, DStarNode* node);
    bool checkCostChanges(unsigned char* costs);
    float calculateCost(int i_u, int i_v);
    void updateVertex(DStarNode* u);
    bool computeShortestPath(int cycles, bool is_alternative = false);
    std::vector<DStarNode*> getNeighbors(int i);
    std::pair<float, float> calculateKey(DStarNode* s);
    float heuristic(int i_u, int i_v);
    float calculateEuclideanDistance(float x1, float y1, float x2, float y2);

    bool findPath(double start_x, double start_y, double end_x, double end_y, std::vector<std::pair<float, float>>& path, bool is_alternative = false, const std::pair<float,float>* evasion_vector = nullptr);
    DStarNode* findNextNode(DStarNode* current_node, std::vector<int>& dStar, bool is_alternative =false, const std::pair<float,float>* evasion_vector = nullptr); 
    PathType calculatePotentialAlternative(std::vector<std::pair<int, unsigned char>>& modified_costs_list, int cycles, double start_x, double start_y, double end_x, double end_y,const std::pair<float,float>* evasion_vector = nullptr);
    bool getPathObstacleInteractionInfo(const PathType& path, int cx_map, int cy_map, double r_cells, double global_start_x, double global_start_y,
                                            double global_goal_x, double global_goal_y, std::pair<float, float>& out_H_on_path,  std::pair<float, float>& out_vec_C_to_H,
                                            std::pair<float, float>& out_evasion_vector, bool* out_C_is_left_of_SG_line = nullptr);
    bool updateStartAndGoalNodes(double start_x, double start_y, double end_x, double end_y);
    bool arePathsIdentical(const PathType& p1, const PathType& p2) const;
    
    inline int toIndex(int x, int y) const { return x + y * nx_; }

    float init_start_x, init_start_y;
    float k_m;
    std::map<int, DStarNode*> nodeMap;
    DStarNode* startNode;
    DStarNode* lastStartNode;
    DStarNode* goalNode;
    std::vector<DStarNode*> openList;
    std::vector<unsigned char> cost_node;
    int goalNR;
    unsigned char* costmap_;
    bool alternativ_previous;
    std::string robot;
    int start_i;
    int goal_i;
    bool is_alt;
    std::string debug_csv_path_;
    int         dbg_nx_, dbg_ny_;
    bool debug_csv_header_written_;
};

}  // end namespace masterarbeit

#endif  // _DSTAR_H
