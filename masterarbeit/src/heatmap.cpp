#include "masterarbeit/heatmap.h"
#include <costmap_2d/costmap_math.h>
#include <costmap_2d/layered_costmap.h>
#include <tf2_ros/message_filter.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_core/TypeDefs.hpp>
#include <grid_map_core/SubmapGeometry.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <functional>
#include <limits>
#include <masterarbeit/Intersect.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(masterarbeit::HeatmapObstacleLayer, costmap_2d::Layer)

using costmap_2d::FREE_SPACE;
using costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace masterarbeit
{

void HeatmapObstacleLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_), g_nh;
  rolling_window_ = layered_costmap_->isRolling();

  bool track_unknown_space;
  nh.param("track_unknown_space", track_unknown_space, layered_costmap_->isTrackingUnknown());

  if (track_unknown_space)
    default_value_ = NO_INFORMATION;
  else
    default_value_ = FREE_SPACE;

  matchSize();

  size_x_grid_ = getSizeInCellsX();
  size_y_grid_ = getSizeInCellsY();
  resolution_grid_ = getResolution();
  g_nh.param("robot", robot, std::string(""));
  global_frame_ = layered_costmap_->getGlobalFrameID();

  grid_map_sub_ = boost::make_shared<GridmapSubscriber>(g_nh, "heatmap_processor/GridMap", 1); //"/" + robot + 
  intersect_sub_ = boost::make_shared<IntersectSubscriber>(g_nh, "heatmap_processor/intersect", 1); //"/" + robot + 
  
  sync_ = boost::make_shared<Sync>(MySyncPolicy(10), *grid_map_sub_, *intersect_sub_);

  auto callback = std::bind(&HeatmapObstacleLayer::heatmapCallback, this, std::placeholders::_1, std::placeholders::_2);
  sync_->registerCallback(callback);
  

  dsrv_ = NULL;
  setupDynamicReconfigure(nh);
}

void HeatmapObstacleLayer::heatmapCallback(const grid_map_msgs::GridMap::ConstPtr &grid_map_msg,
                                           const masterarbeit::Intersect::ConstPtr &intersect_msg)
{
    grid_map::GridMap gridMap;
    grid_map::GridMapRosConverter::fromMessage(*grid_map_msg, gridMap);

    gridMap_ = gridMap;
    intersect_ = *intersect_msg;
}


  void HeatmapObstacleLayer::setupDynamicReconfigure(ros::NodeHandle &nh)
  {
    dsrv_ = new dynamic_reconfigure::Server<masterarbeit::HeatmapObstaclePluginConfig>(nh);
    dynamic_reconfigure::Server<masterarbeit::HeatmapObstaclePluginConfig>::CallbackType cb =
        [this](auto &config, auto level)
    { reconfigureCB(config, level); };
    dsrv_->setCallback(cb);
  }

  void HeatmapObstacleLayer::reconfigureCB(masterarbeit::HeatmapObstaclePluginConfig &config, uint32_t level)
  {
    enabled_ = config.enabled;
    combination_method_ = config.combination_method;
  }

  void HeatmapObstacleLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y, double *max_x, double *max_y)
  {
    if (!enabled_)
      return;

    if (rolling_window_)
      updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);

    useExtraBounds(min_x, min_y, max_x, max_y);
    grid_map::GridMap gridmap = gridMap_;
    Intersect inter = intersect_;
    size = size_x_grid_ * size_y_grid_;

    std::fill_n(costmap_, size, costmap_2d::FREE_SPACE);

    unsigned int mx, my;
    const bool intersection = inter.intersect;
    
    if(intersection)
    {
      grid_map::Matrix &data_fusion = gridmap["fusion_layer"];
      const std::vector<float>& radius = inter.radius;
      const std::vector<float>& center_x = inter.center_x;
      const std::vector<float>& center_y = inter.center_y;

      for(int i = 0; i<radius.size(); i++)
      {
        for (grid_map::CircleIterator iterator(gridmap, grid_map::Position(center_x[i], center_y[i]), radius[i]/2.0); !iterator.isPastEnd(); ++iterator)
        {
          const grid_map::Index index(*iterator);
          grid_map::Position pos;
          gridmap.getPosition(index, pos);
          if (gridmap.isValid(index, "fusion_layer"))
          {
            if(data_fusion(index(0), index(1)) == 0.0)
            {
              if (!worldToMap(pos[0], pos[1], mx, my))
              {
                continue;
              }
              else
              {
                unsigned int costmap_index_ = getIndex(mx, my);
                costmap_[costmap_index_] = costmap_2d::LETHAL_OBSTACLE;
                
              }
            }
            else
            {
              if (!worldToMap(pos[0], pos[1], mx, my))
              {
                continue;
              }
              else
              {
                unsigned int costmap_index_ = getIndex(mx, my);
                costmap_[costmap_index_] = costmap_2d::LETHAL_OBSTACLE;
              }
            }
            touch(pos[0], pos[1], min_x, min_y, max_x, max_y);
          }
        }
      }
    }
  }

  void HeatmapObstacleLayer::updateCosts(costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j)
  {
    if (footprint_clearing_enabled_)
    {
      setConvexPolygonCost(transformed_footprint_, costmap_2d::FREE_SPACE);
    }

    switch (combination_method_)
    {
    case 0: // Overwrite
      updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
      break;
    case 1: // Maximum
      updateWithMax(master_grid, min_i, min_j, max_i, max_j);
      break;
    default: // Nothing
      break;
    }
  }

} // end namespace masterarbeit
