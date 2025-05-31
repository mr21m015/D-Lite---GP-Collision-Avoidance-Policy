#ifndef HEATMAP_NODE_H
#define HEATMAP_NODE_H

#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <tf2_ros/message_filter.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include "obstacle_detector/Obstacles.h"
#include <memory>
#include <mutex>



namespace masterarbeit{

 static const std::map<std::string, grid_map::InterpolationMethods> interpolationMethods = {
    { "Nearest",              grid_map::InterpolationMethods::INTER_NEAREST},
    { "Linear",               grid_map::InterpolationMethods::INTER_LINEAR },
    { "Cubic_convolution",    grid_map::InterpolationMethods::INTER_CUBIC_CONVOLUTION },
    { "Cubic",                grid_map::InterpolationMethods::INTER_CUBIC } };

class HeatmapProcessor
{
public:
    explicit HeatmapProcessor(ros::NodeHandle& nh);
    virtual ~HeatmapProcessor();
    
private:
    void onInitialize();
    void heatmapCallback(const geometry_msgs::PoseArray::ConstPtr& particlecloud,
                            const obstacle_detector::Obstacles::ConstPtr& obstacles,
                            const nav_msgs::Odometry::ConstPtr& odom);
    bool combineLayers(double min_x, double max_x, double min_y, double max_y, double t);
    bool checkOverlap(double min1_x, double min1_y, double max1_x, double max1_y,
                double min2_x, double min2_y, double max2_x, double max2_y);
    void processTimeSlice(double t,
                          const geometry_msgs::PoseArray::ConstPtr& particlecloud,
                          const obstacle_detector::Obstacles::ConstPtr& obstacles,
                          double odomVelocity_,
                          std::vector<double>& center_xs,
                          std::vector<double>& center_ys,
                          std::vector<double>& radii,
                          std::vector<double>& time);
    
    ros::NodeHandle nh_;
    boost::shared_ptr<message_filters::Subscriber<geometry_msgs::PoseArray>> amcl_sub_;
    boost::shared_ptr<message_filters::Subscriber<obstacle_detector::Obstacles>> obstacles_sub_;
    boost::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;

    typedef message_filters::Subscriber<geometry_msgs::PoseArray> PoseArraySubscriber;
    typedef message_filters::Subscriber<obstacle_detector::Obstacles> ObstaclesSubscriber;
    typedef message_filters::Subscriber<nav_msgs::Odometry> OdometrySubscriber;

    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseArray, obstacle_detector::Obstacles, nav_msgs::Odometry> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync_;

    ros::Publisher gridmap_pub_;
    ros::Publisher intersect_pub_;
    
    grid_map::GridMap gridMap_;
    double mean_value_;
    std::string interpolationMethod_;
    bool intersect;

    double resolution_, size_grid_x_, size_grid_y_;
    std::vector<double> timeSlices;
    std::vector<bool> timeOverlap;
    std::string robot, global_frame_;

    std::mutex data_mutex_; 
};
} //namesapce masterarbeit
#endif // HEATMAP_NODE_H