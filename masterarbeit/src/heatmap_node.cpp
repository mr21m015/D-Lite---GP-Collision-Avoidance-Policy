#include "masterarbeit/heatmap_node.h"
#include <grid_map_msgs/GridMap.h>
#include <grid_map_core/TypeDefs.hpp>
#include <grid_map_core/SubmapGeometry.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <Eigen/Dense>
#include <functional>
#include <limits>
#include <string>
#include <masterarbeit/Intersect.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <thread>
#include <vector>
#include <tuple>
#include <algorithm>

namespace masterarbeit {
HeatmapProcessor::HeatmapProcessor(ros::NodeHandle& nh): nh_(nh)
{
    onInitialize();
}

HeatmapProcessor::~HeatmapProcessor() {}

void HeatmapProcessor::onInitialize()
{
    nh_.param("mapframe", global_frame_, std::string("/map"));
    nh_.param("frame_size_X", size_grid_x_, 100.0);
    nh_.param("frame_size_Y", size_grid_y_, 100.0);
    nh_.param("Resolution", resolution_, 0.05);
    gridMap_.setFrameId(global_frame_);
    gridMap_.setGeometry(grid_map::Length(size_grid_x_*resolution_, size_grid_y_*resolution_), resolution_, grid_map::Position(0, 0));
    timeSlices = {0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0};
    timeOverlap.resize(timeSlices.size());
    gridMap_.add("odom_particles", 0.0);
    gridMap_.add("obstacle_particles", 0.0);
    gridMap_.add("fusion_layer", 0.0);

     ROS_INFO("Initialization complete: Global Frame: %s, Frame Size X: %f, Frame Size Y: %f, Resolution: %f",
             global_frame_.c_str(), size_grid_x_, size_grid_y_, resolution_);

    std::string topics_string;
    nh_.param("observation_sources", topics_string, std::string("")); // obstacles
    ROS_INFO("Subscribed to Topics: %s", topics_string.c_str());

    gridmap_pub_ = nh_.advertise<grid_map_msgs::GridMap>("GridMap", 1, true);
    intersect_pub_ = nh_.advertise<masterarbeit::Intersect>("intersect", 1, true);
    ROS_INFO("Intersect publisher initialized.");

    std::string topic;
    nh_.param("topic", topic, std::string("obstacles"));
    nh_.param("robot", robot, std::string(""));
    nh_.param("interpolation_type", interpolationMethod_, std::string("Nearest"));
    ros::NodeHandle nh;
    std::string full_topic = robot + "/" + topic;
    ROS_WARN_STREAM("Topic: " << topic << " Robot: " << robot << " Interpolation type: " << interpolationMethod_);
    obstacles_sub_ = boost::make_shared<ObstaclesSubscriber>(nh, topic, 1);
    amcl_sub_ = boost::make_shared<PoseArraySubscriber>(nh,"particlecloud", 1);
    odom_sub_ = boost::make_shared<OdometrySubscriber>(nh, "odom", 1);
    sync_ = boost::make_shared<Sync>(MySyncPolicy(10), *amcl_sub_, *obstacles_sub_, *odom_sub_);

    auto callback = std::bind(&HeatmapProcessor::heatmapCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    sync_->registerCallback(callback);
}

void HeatmapProcessor::processTimeSlice(double t,
                        const geometry_msgs::PoseArray::ConstPtr& particlecloud,
                        const obstacle_detector::Obstacles::ConstPtr& obstacles,
                        double odomVelocity_,
                        std::vector<double>& center_xs,
                        std::vector<double>& center_ys,
                        std::vector<double>& radii,
                        std::vector<double>& time) {

                      
    double min_p_x = std::numeric_limits<double>::max(), max_p_x = std::numeric_limits<double>::lowest();
    double min_p_y = std::numeric_limits<double>::max(), max_p_y = std::numeric_limits<double>::lowest();
    double tempRoll, tempPitch, yaw;
    Eigen::Vector2d position, direction_vector, projected_velocity, future_position;
    double speed_magnitude;

    for (const auto &pose : particlecloud->poses) {
        position = Eigen::Vector2d(pose.position.x, pose.position.y);
        tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
        tf2::Matrix3x3 m(q);
        m.getRPY(tempRoll, tempPitch, yaw);

        direction_vector = Eigen::Vector2d(cos(yaw), sin(yaw));
        projected_velocity = odomVelocity_ * direction_vector;
        future_position = position + t * projected_velocity;
        min_p_x = std::min(min_p_x, future_position[0]);
        max_p_x = std::max(max_p_x, future_position[0]);
        min_p_y = std::min(min_p_y, future_position[1]);
        max_p_y = std::max(max_p_y, future_position[1]);

        if (gridMap_.isInside(future_position)) {
            gridMap_.atPosition("odom_particles"  + std::to_string(t), future_position) += 0.01;
        }
    }

    double min_o_x = std::numeric_limits<double>::max();
    double max_o_x = std::numeric_limits<double>::lowest();
    double min_o_y = std::numeric_limits<double>::max();
    double max_o_y = std::numeric_limits<double>::lowest();

    for (const auto &circle : obstacles->circles) {
        speed_magnitude = std::sqrt(circle.velocity.x * circle.velocity.x + circle.velocity.y * circle.velocity.y);
        for (const auto &particle : circle.particlefilter.poses) {
            position = Eigen::Vector2d(particle.position.x, particle.position.y);
            tf2::Quaternion q(particle.orientation.x, particle.orientation.y, particle.orientation.z, particle.orientation.w);
            tf2::Matrix3x3 m(q);
            m.getRPY(tempRoll, tempPitch, yaw);

            direction_vector = Eigen::Vector2d(cos(yaw), sin(yaw));
            projected_velocity = speed_magnitude * direction_vector;
            future_position = position + t * projected_velocity;
            min_o_x = std::min(min_o_x, future_position[0]);
            max_o_x = std::max(max_o_x, future_position[0]);
            min_o_y = std::min(min_o_y, future_position[1]);
            max_o_y = std::max(max_o_y, future_position[1]);

            if (gridMap_.isInside(future_position)) {
                gridMap_.atPosition("obstacle_particles" + std::to_string(t), position) += 0.01;
            }
        }

        if (checkOverlap(min_o_x, min_o_y, max_o_x, max_o_y, min_p_x, min_p_y, max_p_x, max_p_y)) {
            double final_min_x = std::max(min_o_x, min_p_x);
            double final_max_x = std::min(max_o_x, max_p_x);
            double final_min_y = std::max(min_o_y, min_p_y);
            double final_max_y = std::min(max_o_y, max_p_y);
            double center_x = (final_min_x + final_max_x) / 2.0;
            double center_y = (final_min_y + final_max_y) / 2.0;
            double width = final_max_x - final_min_x;
            double height = final_max_y - final_min_y;
            double radius = std::min(width, height) / 2.0;
            std::lock_guard<std::mutex> lock(data_mutex_);  
            if (combineLayers(final_min_x, final_min_y, final_max_x, final_max_y, t))
            {
                for(int i=0; i < timeSlices.size();i++)
                {
                    if(t == timeSlices[i] && mean_value_ != 0.0) 
                    {
                        timeOverlap[i] = true;
                        break;
                    }
                }

                intersect = true;
                center_xs.push_back(center_x);
                center_ys.push_back(center_y);
                radii.push_back(radius);
                time.push_back(t);

                break;
            }
            else 
            {
                ROS_DEBUG_STREAM("GridMap::get(...) : No map layer of type 'fusion_layer' available.");
            }
        }
    }

    if (intersect) return;
}

void HeatmapProcessor::heatmapCallback(const geometry_msgs::PoseArray::ConstPtr& particlecloud,
                                       const obstacle_detector::Obstacles::ConstPtr& obstacles,
                                       const nav_msgs::Odometry::ConstPtr& odom) {
    // Clear existing grid map layers
    gridMap_.add("odom_particles", 0.0); 
    gridMap_.add("obstacle_particles", 0.0);
    gridMap_.add("fusion_layer", 0.0);
    
    for(double t : timeSlices)
    {
        std::string odom_layer = "odom_particles" + std::to_string(t);
        std::string obstacle_layer = "obstacle_particles" + std::to_string(t);
        std::string fusion_layer = "fusion_layer" + std::to_string(t);

        gridMap_.add(odom_layer, 0.0);
        gridMap_.add(obstacle_layer, 0.0);
        gridMap_.add(fusion_layer, 0.0);
        
    }
    std::fill(timeOverlap.begin(), timeOverlap.end(), false);
    intersect = false;
    std::vector<double> center_xs;
    std::vector<double> center_ys;
    std::vector<double> radii;
    std::vector<double> time;
    double odomVelocity_ = odom->twist.twist.linear.x;
    std::vector<std::thread> threads;
    

    for (double t : timeSlices) {
        threads.emplace_back(&HeatmapProcessor::processTimeSlice, this, t, particlecloud, obstacles, odomVelocity_,
                std::ref(center_xs), std::ref(center_ys), std::ref(radii), std::ref(time));
    }

    for (auto& t : threads) {
        t.join();
    }
    bool layerOverlap = false;

    std::vector<std::tuple<float, float, float, float>> combined;
    for (size_t i = 0; i < time.size(); ++i) {
        combined.emplace_back(time[i], center_xs[i], center_ys[i], radii[i]);
    }

    std::sort(combined.begin(), combined.end(), 
            [](const auto& a, const auto& b) {
                return std::get<0>(a) < std::get<0>(b);
            });

    for(int i=0; i < timeSlices.size();i++)
    {
        std::string odom_layer = "odom_particles" + std::to_string(timeSlices[i]);
        std::string obstacle_layer = "obstacle_particles" + std::to_string(timeSlices[i]);
        std::string fusion_layer = "fusion_layer" + std::to_string(timeSlices[i]);
        if (timeOverlap[i] && layerOverlap != false) 
        {
            
            gridMap_.add("odom_particles",gridMap_[odom_layer]);
            gridMap_.add("obstacle_particles", gridMap_[obstacle_layer]);
            gridMap_.add("fusion_layer", gridMap_[fusion_layer]);
            gridMap_.erase(odom_layer);
            gridMap_.erase(obstacle_layer);
            gridMap_.erase(fusion_layer);
            
            layerOverlap = true;
        }
        else
        {
            gridMap_.erase(odom_layer);
            gridMap_.erase(obstacle_layer);
            gridMap_.erase(fusion_layer);
        }
    }
    masterarbeit::Intersect msg;
    ros::Time t = ros::Time::now();
    msg.header.stamp = t;
    msg.header.frame_id = global_frame_;

    msg.intersect = intersect;
    if (!combined.empty()) 
    {
        float min_time = std::get<0>(combined[0]);
        for (size_t i = 0; i < combined.size(); ++i)
        {
            if(min_time == std::get<0>(combined[i]))
            {
                msg.center_x.push_back(std::get<1>(combined[i]));
                msg.center_y.push_back(std::get<2>(combined[i]));
                msg.radius.push_back(std::get<3>(combined[i]));
                msg.time.push_back(std::get<0>(combined[i]));
                msg.circles = obstacles->circles;
            }
        }
    }
    

    intersect_pub_.publish(msg);

    grid_map_msgs::GridMap message;
    grid_map::GridMapRosConverter::toMessage(gridMap_, message);
    message.info.header.stamp = t;
    gridmap_pub_.publish(message);
}

bool HeatmapProcessor::combineLayers(double min_x, double min_y, double max_x, double max_y, double t)
{
    int count_ = 0;
    mean_value_ = 0.0;
    std::string odom_layer = "odom_particles" + std::to_string(t);
    std::string obstacle_layer = "obstacle_particles" + std::to_string(t);
    std::string fusion_layer = "fusion_layer" + std::to_string(t);
    if (gridMap_.exists(odom_layer) && gridMap_.exists(obstacle_layer) && gridMap_.exists(fusion_layer))
    {
        bool intersect = false;

        grid_map::Position minPos(max_x, max_y);
        grid_map::Position maxPos(min_x, min_y);
        grid_map::Index minIndex, maxIndex;
        if (!gridMap_.isInside(minPos) || !gridMap_.isInside(maxPos))
        {
            ROS_DEBUG("One or both positions are out of map bounds.");
            return false;
        }

        if (!gridMap_.getIndex(minPos, minIndex) || !gridMap_.getIndex(maxPos, maxIndex))
        {
            ROS_DEBUG("Positions out of grid map bounds.");
            return false;
        }
        grid_map::Index sizeIndex = maxIndex - minIndex;

        grid_map::Matrix &data_odom = gridMap_[odom_layer];
        grid_map::Matrix &data_obstacle = gridMap_[obstacle_layer];
        grid_map::Matrix &data_fusion = gridMap_[fusion_layer];

        for (grid_map::SubmapIterator it(gridMap_, minIndex, sizeIndex); !it.isPastEnd(); ++it)
        {
            const grid_map::Index index(*it);

            if (!gridMap_.isValid(index, odom_layer))
            {
                continue;
            }
            if (!gridMap_.isValid(index, obstacle_layer))
            {
                continue;
            }
            if (index(0) < 0 || index(0) >= data_odom.rows() || index(1) < 0 || index(1) >= data_odom.cols())
            {
                continue;
            }
            float odom_value = data_odom(index(0), index(1));
            float obstacle_value = data_obstacle(index(0), index(1));

            if (odom_value == 0.0 || obstacle_value == 0.0)
            {
                continue;
            }

            float &fusion_value = data_fusion(index(0), index(1));
            fusion_value = odom_value + obstacle_value;
            intersect = true;
        }
        return true;
    }
    else
    {
        ROS_WARN_STREAM("combineLayers skipped: One or more layers missing for time " << t << " (odom: " << gridMap_.exists(odom_layer) << ", obstacle: " << gridMap_.exists(obstacle_layer) << ", fusion: " << gridMap_.exists(fusion_layer) << ")");
        return false;
    }
}

bool HeatmapProcessor::checkOverlap(double min1_x, double min1_y, double max1_x, double max1_y,
                                    double min2_x, double min2_y, double max2_x, double max2_y)
{
    bool x_overlap = max1_x >= min2_x && min1_x <= max2_x;
    bool y_overlap = max1_y >= min2_y && min1_y <= max2_y;

    bool overlap = x_overlap && y_overlap;

    return overlap;
}

} // end namespace masterarbeit

int main(int argc, char** argv)
{
    ros::init(argc, argv, "heatmap_processor");
    ros::NodeHandle nh("~");
    masterarbeit::HeatmapProcessor processor(nh);
    while (ros::ok())
    {  
        ros::spin();
    }
    return 0;
}
