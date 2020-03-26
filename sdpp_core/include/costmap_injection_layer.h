#ifndef COSTMAP_INJECTION_LAYER_H_
#define COSTMAP_INJECTION_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>

#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>



namespace costmap_injection_namespace
{
class CostmapInjectionLayer : public costmap_2d::Layer, public costmap_2d::Costmap2D
{
public:
  CostmapInjectionLayer();

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                             double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  bool isDiscretized()
  {
    return true;
  }

  virtual void matchSize();

private:

  ros::Subscriber sub;
  nav_msgs::OccupancyGrid grid;
  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;

  void pointCallback(const geometry_msgs::Point::ConstPtr& message);
  void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& message);
};
}
#endif