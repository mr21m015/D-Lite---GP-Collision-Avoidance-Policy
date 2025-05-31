#ifndef _PLANNER_D_CORE_H
#define _PLANNER_D_CORE_H

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Odometry.h>
#include <nav_core/base_global_planner.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <vector>
#include <random>

#include "masterarbeit/Intersect.h"
#include "masterarbeit/dstar.h"
#include "masterarbeit/potential_d_calculator.h"
#include "obstacle_detector/Obstacles.h"

#include <global_planner/expander.h>
#include <global_planner/orientation_filter.h>
#include <global_planner/potential_calculator.h>
#include <global_planner/GlobalPlannerConfig.h>




namespace masterarbeit {


class Expander;
class GridPath;
class DStarExpansion;

class GlobalDPlanner : public nav_core::BaseGlobalPlanner {
    public:
        GlobalDPlanner();

        GlobalDPlanner(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id);

        ~GlobalDPlanner();

        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        void initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id);

        bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                    std::vector<geometry_msgs::PoseStamped>& plan);

        bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, double tolerance,
                    std::vector<geometry_msgs::PoseStamped>& plan);

        bool getPlanFromPotential(double start_x, double start_y, double end_x, double end_y,
                                const geometry_msgs::PoseStamped& goal, const geometry_msgs::PoseStamped& start,
                                std::vector<geometry_msgs::PoseStamped>& plan);


        void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);
        void publishPlanAlternative(const std::vector<geometry_msgs::PoseStamped>& path);

        bool makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp);
        int getIndex(int x, int y, int xs) {
            return x + y * xs;
        }
        void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);
        void intersectCallback(const masterarbeit::Intersect::ConstPtr& intersect_msg); 

        void publishBaseFootprintData(const std::vector<geometry_msgs::PoseStamped>& base_footprint_plan,
                                              const std::vector<geometry_msgs::PoseStamped>& base_footprint_alternative_plan,
                                              const geometry_msgs::PoseStamped& goal_base_footprint,
                                              int random_choice,
                                              const masterarbeit::Intersect& intersect,
                                              float odom_vel, float primary_length, float alternative_length,
                                              bool direction); 
    protected:

        costmap_2d::Costmap2D* costmap_;
        std::string frame_id_;
        ros::Publisher plan_pub_;
        ros::Publisher plan_failed_pub_;
        ros::Publisher base_footprint_pub_;
        ros::Publisher plan_basefootprint_pub_;

        ros::Subscriber intersect_sub_;
        ros::Subscriber odom_sub_;
        bool initialized_, allow_unknown_, publish_potential_;
        

    private:
        void mapToWorld(double mx, double my, double& wx, double& wy);
        bool worldToMap(double wx, double wy, double& mx, double& my);
        void clearRobotCell(const geometry_msgs::PoseStamped& global_pose, unsigned int mx, unsigned int my);
        void publishPotential(float* potential);
        void transformIntersectDataToBase(const tf2::Transform& tf_map_to_base, 
                                                    std::vector<geometry_msgs::PoseStamped>& base_footprint_plan,
                                                    std::vector<geometry_msgs::PoseStamped>& base_footprint_alternative_plan,
                                                    geometry_msgs::PoseStamped& goal_base_footprint);
        void transformPlanAndGoal(const tf2::Transform& tf_map_to_base,
                                    std::vector<geometry_msgs::PoseStamped>& base_footprint_plan,
                                    std::vector<geometry_msgs::PoseStamped>& base_footprint_alternative_plan,
                                    geometry_msgs::PoseStamped& goal_base_footprint);
        double computePlanLength(const std::vector<geometry_msgs::PoseStamped>& plan) const;
        double protected_division(double numerator, double denominator);
        bool decide_action_from_phenotype( bool direction, double intersect_centers_x, double intersect_centers_y,
            double intersect_radius, double obstacle_x, double obstacle_y, double velocity_x, double velocity_y, double robot_pos_x, double robot_pos_y);
        
        double planner_window_x_, planner_window_y_, default_tolerance_;
        boost::mutex mutex_;
        ros::ServiceServer make_plan_srv_;


        global_planner::PotentialCalculator* p_calc_;
        DStarExpansion* planner_dstar_;
        global_planner::OrientationFilter* orientation_filter_;
        
        float alternative_expansion_; 
        ros::Publisher potential_pub_;
        int publish_scale_;
        std::string robot;

        std::random_device rd_;
        std::mt19937 gen_;
        geometry_msgs::Point odom_pos_;

        void outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value);

        float* potential_array_;
        unsigned int start_x_, start_y_, end_x_, end_y_;

        float convert_offset_;
        double extra_yaw_;

        bool outline_map_, found_legal;
        double realtime_duration_;
        masterarbeit::Intersect intersect_;
        float odom_vel;
        int path_choice, count;
        std::pair<int,bool> initial_side;
        dynamic_reconfigure::Server<global_planner::GlobalPlannerConfig> *dsrv_;
        void reconfigureCB(global_planner::GlobalPlannerConfig &config, uint32_t level);

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener;

};

} //end namespace global_planner

#endif // PLANNER_D_CORE_H
