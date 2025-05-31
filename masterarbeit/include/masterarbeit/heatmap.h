#ifndef HEATMAP_OBSTACLE_LAYER_H_
#define HEATMAP_OBSTACLE_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <pluginlib/class_loader.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_core/grid_map_core.hpp>


#include <nav_msgs/OccupancyGrid.h>
#include <Eigen/Dense>
#include <tf2_ros/message_filter.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <dynamic_reconfigure/server.h>
#include "masterarbeit/HeatmapObstaclePluginConfig.h"
#include "masterarbeit/Intersect.h"
#include <costmap_2d/footprint.h>


#include <boost/make_shared.hpp>
namespace masterarbeit{

class HeatmapObstacleLayer :  public costmap_2d::CostmapLayer
{
public:
  HeatmapObstacleLayer()
  {
    costmap_ = NULL;  // this is the unsigned char* member of parent class Costmap2D.
  }
  virtual ~HeatmapObstacleLayer(){}

  virtual void onInitialize() override;
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y) override;
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);


void heatmapCallback(const grid_map_msgs::GridMap::ConstPtr &grid_map_msg,
                    const masterarbeit::Intersect::ConstPtr &intersect_msg);


protected:
  protected:
  virtual void setupDynamicReconfigure(ros::NodeHandle& nh);

  std::vector<geometry_msgs::Point> transformed_footprint_;
  bool footprint_clearing_enabled_;
  
  boost::shared_ptr<message_filters::Subscriber<grid_map_msgs::GridMap>> grid_map_sub_;
  boost::shared_ptr<message_filters::Subscriber<masterarbeit::Intersect>> intersect_sub_;

  typedef message_filters::Subscriber<grid_map_msgs::GridMap> GridmapSubscriber;
  typedef message_filters::Subscriber<masterarbeit::Intersect> IntersectSubscriber;
  typedef message_filters::sync_policies::ApproximateTime<grid_map_msgs::GridMap, masterarbeit::Intersect> MySyncPolicy;
  typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  boost::shared_ptr<Sync> sync_;

  std::string global_frame_, robot;  

  bool rolling_window_;
  dynamic_reconfigure::Server<masterarbeit::HeatmapObstaclePluginConfig> *dsrv_;

  int combination_method_;

  unsigned int size_x_grid_;
  unsigned int size_y_grid_;
  double resolution_grid_;
  double origin_x;
  double origin_y;
  double mean_value_;
  grid_map::GridMap gridMap_;
  size_t size;

  Intersect intersect_;



private:
  void reconfigureCB(masterarbeit::HeatmapObstaclePluginConfig &config, uint32_t level);
};

} // end namespace masterarbeit

#endif // HEATMAP_OBSTACLE_LAYER_H_
