#include<grid_layer.h>

#include <pluginlib/class_list_macros.h>
#include <tf2_ros/message_filter.h>


PLUGINLIB_EXPORT_CLASS(simple_layer_namespace::GridLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace simple_layer_namespace
{

GridLayer::GridLayer() {}

void GridLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;
  default_value_ = NO_INFORMATION;
  matchSize();

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &GridLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);

  //boost::shared_ptr <message_filters::Subscribers<sensor_msgs::LaserScan>
  //  > sub(new message_filters::Subscriber<sensor_msgs::LaserScan>(g_nh, topic, 50));
  sub = nh.subscribe("/test", 1000, &GridLayer::pointCallback, this);
    ROS_INFO("subscriber initialized");
   //sub(new message_filters::Subscriber<sensor_msgs::LaserScan>(nh, "test", 50));
}


void GridLayer::matchSize()
{
  Costmap2D* master = layered_costmap_->getCostmap();


  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());

}


void GridLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

void GridLayer::pointCallback(const geometry_msgs::Point::ConstPtr& message){

    ROS_INFO("I heard it");

}

void GridLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;


  //double mark_x = robot_x + cos(robot_yaw), mark_y = robot_y + sin(robot_yaw);
  double mark_x = 1, mark_y = 1;

  unsigned int mx;
  unsigned int my;
  if(worldToMap(mark_x, mark_y, mx, my)){
    setCost(mx, my, LETHAL_OBSTACLE);
  }

  *min_x = std::min(*min_x, mark_x);
  *min_y = std::min(*min_y, mark_y);
  *max_x = std::max(*max_x, mark_x);
  *max_y = std::max(*max_y, mark_y);
}

void GridLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_)
    return;

  for (int j = min_j; j < max_j; j++)
  {
    for (int i = min_i; i < max_i; i++)
    {
      int index = getIndex(i, j);
      if (costmap_[index] == NO_INFORMATION)
        continue;
      master_grid.setCost(i, j, costmap_[index]);
    }
  }
}

} // end namespace
