#include<sdpp_navigation/costmap_injection_layer.h>
#include <pluginlib/class_list_macros.h>
#include <tf2_ros/message_filter.h>


PLUGINLIB_EXPORT_CLASS(costmap_injection_namespace::CostmapInjectionLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace costmap_injection_namespace
{

CostmapInjectionLayer::CostmapInjectionLayer() {}

void CostmapInjectionLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;
  default_value_ = NO_INFORMATION;
  //matchSize();

  //dynamic reconfigure server
  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &CostmapInjectionLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);

  sub = nh.subscribe("/test", 1000, &CostmapInjectionLayer::costmapCallback, this);
    ROS_INFO("subscriber initialized");
}


void CostmapInjectionLayer::matchSize()
{
  Costmap2D* master = layered_costmap_->getCostmap();

  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());
}


void CostmapInjectionLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

void CostmapInjectionLayer::costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& message){

    grid = *message;

    //for(std::vector<signed char>::iterator it = grid.data.begin(); it != grid.data.end(); it++)
    //    std::cout << ' ' << *it;


    ROS_INFO("%s", message->header.frame_id.c_str());
}

void CostmapInjectionLayer::pointCallback(const geometry_msgs::Point::ConstPtr& message){

    ROS_INFO("I heard it");

}

void CostmapInjectionLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;


  *min_x = 1;
  *min_y = 1;
  *max_x = 100;
  *max_y = 100;
}

void CostmapInjectionLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
    //for(std::vector<signed char>::iterator it = grid.data.begin(); it != grid.data.end(); it++)
    //    std::cout << ' ' << *it;

    //std::cout << grid.data.size() << std::endl;

  if (!enabled_)
    return;

   if( grid.data.size() == 0)
    return;


  for (int j = 0; j < 200; j++)
  {
    for (int i = 0; i < 200; i++)
    {
      int index = getIndex(i, j);
      if (grid.data[index] == NO_INFORMATION)
        continue;

      //std::cout << grid.header << std::endl;
      master_grid.setCost(i, j, grid.data[index]);
    }
  }
}

} // end namespace