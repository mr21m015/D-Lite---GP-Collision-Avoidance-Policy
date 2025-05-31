#include "masterarbeit/collisionDetector.h"
#include "masterarbeit/CollisionDetection.h"
#include <geometry_msgs/Point32.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>

namespace masterarbeit{

CollisionDetector::CollisionDetector(const ros::NodeHandle& nh)  : nh_(nh), crashed_(false), tf_listener_(tf_buffer_)
{
    ros::NodeHandle g_nh_;
    nh_.param("robot",robot,std::string(""));
    
    std::string occupancy_topic = robot.empty() ? "/move_base/local_costmap/costmap" : "move_base_" + robot + "/local_costmap/costmap";
    std::string footprint_topic = robot.empty() ? "/move_base/local_costmap/footprint" : "move_base_" + robot + "/local_costmap/footprint";

    occupancy_grid_sub_.subscribe(g_nh_, occupancy_topic, 10);
    footprint_sub_.subscribe(g_nh_, footprint_topic, 10);

    sync_.reset(new Sync(MySyncPolicy(10), occupancy_grid_sub_, footprint_sub_));
    sync_->registerCallback(boost::bind(&CollisionDetector::callback, this, _1, _2));//, _3
    
    ROS_WARN("Abonniere auf: %s, %s", occupancy_grid_sub_.getTopic().c_str(), footprint_sub_.getTopic().c_str());
    collision_pub_ = g_nh_.advertise<masterarbeit::CollisionDetection>("collision_detector", 1);
    polygon_pub_ = g_nh_.advertise<geometry_msgs::PolygonStamped>("collision_polygon", 1);

}

void CollisionDetector::callback(const nav_msgs::OccupancyGridConstPtr& occupancy_grid,
                                 const geometry_msgs::PolygonStampedConstPtr& footprint) 
{
    
    geometry_msgs::PolygonStamped transformed_footprint = transformPolygon(*footprint, robot + "_tf/base_footprint");

    crashed_ = checkCollision(*occupancy_grid, transformed_footprint);
    publishStretchedPolygon();
    
    masterarbeit::CollisionDetection msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = occupancy_grid->header.frame_id;
    msg.collision_ = crashed_;
    collision_pub_.publish(msg); 
}


bool CollisionDetector::checkCollision(const nav_msgs::OccupancyGrid& grid, const geometry_msgs::PolygonStamped& footprint)
{
    bool crashed = false;

    // Skalierungsfaktor
    double shift_factor = 0.05;

    geometry_msgs::PolygonStamped scaled_polygon = footprint;

    scaled_polygon.header.stamp = ros::Time::now();
    scaled_polygon.header.frame_id = robot + "_tf/base_footprint";

    // Durch alle Punkte im transformierten Footprint iterieren und skalieren
    for (auto& point : scaled_polygon.polygon.points)
    {
        point.x = point.x + shift_factor;
    }

    // Transformiere das skalierte Polygon zurück in das Grid-Frame
    stretched_polygon_ = transformPolygon(scaled_polygon, grid.header.frame_id);


    // Überprüfe, ob das Polygon eine Kollision verursacht
    for(const auto& point : stretched_polygon_.polygon.points)
    {
        int grid_x = static_cast<int>((point.x - grid.info.origin.position.x) / grid.info.resolution);
        int grid_y = static_cast<int>((point.y - grid.info.origin.position.y) / grid.info.resolution);

        if(grid_x >= 0 && grid_x < static_cast<int>(grid.info.width) &&
           grid_y >= 0 && grid_y < static_cast<int>(grid.info.height))
        {
            int index = grid_y * grid.info.width + grid_x;
            if(grid.data[index] > 50)
            {
                ROS_WARN_STREAM(robot << " collision detected");
                crashed = true;
                break;
            }
        }
    }

    return crashed; 
}

geometry_msgs::PolygonStamped CollisionDetector::transformPolygon(const geometry_msgs::PolygonStamped& input_polygon, 
    const std::string& target_frame)
{
    geometry_msgs::PolygonStamped transformed_polygon;
    transformed_polygon.header.frame_id = target_frame;
    transformed_polygon.header.stamp = ros::Time::now();

    // Transform nur einmal abfragen
    geometry_msgs::TransformStamped transform_msg;
    try
    {
        transform_msg = tf_buffer_.lookupTransform(target_frame, input_polygon.header.frame_id, ros::Time(0), ros::Duration(0.1));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN_STREAM("Transform lookup failed: " << ex.what());
        return transformed_polygon;
    }

    tf2::Transform transform;
    tf2::fromMsg(transform_msg.transform, transform);

    // Alle Punkte transformieren
    for (const auto& point : input_polygon.polygon.points)
    {
        tf2::Vector3 p_in(point.x, point.y, point.z);
        tf2::Vector3 p_out = transform * p_in;

        geometry_msgs::Point32 new_point;
        new_point.x = p_out.x();
        new_point.y = p_out.y();
        new_point.z = p_out.z();
        transformed_polygon.polygon.points.push_back(new_point);
    }

    return transformed_polygon;
}




void CollisionDetector::publishStretchedPolygon()
{
    if (!stretched_polygon_.polygon.points.empty())
    {
        polygon_pub_.publish(stretched_polygon_);
        // ROS_INFO("Stretched polygon published.");
    }
    // else
    // {
    //     ROS_WARN("Stretched polygon is empty, nothing to publish.");
    // }
}


} // end namespace masterarbeit

int main(int argc, char** argv)
{
    ros::init(argc, argv, "collision_detector");
    ros::NodeHandle nh("~"); 
    masterarbeit::CollisionDetector detector(nh);
    ros::spin();
    return 0;
}

