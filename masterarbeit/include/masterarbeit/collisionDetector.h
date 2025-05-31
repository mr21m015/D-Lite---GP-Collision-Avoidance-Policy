#ifndef COLLISION_DETECTOR_H
#define COLLISION_DETECTOR_H

#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "masterarbeit/Intersect.h"
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


namespace masterarbeit{
class CollisionDetector
{
public:
    CollisionDetector(const ros::NodeHandle& nh);

private:
    void callback(const nav_msgs::OccupancyGridConstPtr& occupancy_grid,
                  const geometry_msgs::PolygonStampedConstPtr& footprint);

    void publishStretchedPolygon();

    ros::NodeHandle nh_;
    message_filters::Subscriber<nav_msgs::OccupancyGrid> occupancy_grid_sub_;
    message_filters::Subscriber<geometry_msgs::PolygonStamped> footprint_sub_;
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::OccupancyGrid, geometry_msgs::PolygonStamped> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync_;
    ros::Publisher collision_pub_;
    ros::Publisher polygon_pub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    bool checkCollision(const nav_msgs::OccupancyGrid& grid, const geometry_msgs::PolygonStamped& footprint);
   geometry_msgs::PolygonStamped transformPolygon(const geometry_msgs::PolygonStamped& input_polygon, 
                                               const std::string& target_frame);

    std::string robot;
    bool crashed_;
    geometry_msgs::PolygonStamped stretched_polygon_;
};
} //namesapce masterarbeit
#endif // COLLISION_DETECTOR_H
