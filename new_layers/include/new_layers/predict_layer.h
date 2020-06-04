#ifndef COSTMAP_2D_DYNOBSTACLE_LAYER_H_
#define COSTMAP_2D_DYNOBSTACLE_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>

#include <tracking_2d_lidar/TracksArrayMsg.h>
#include <geometry_msgs/Twist.h>
#include <vector>

double det(double a[2], double b[2]);

namespace new_layers
{

class PredictLayer : public costmap_2d::Layer, public costmap_2d::Costmap2D
{
public:
  PredictLayer();

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                             double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

private:
  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);

  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
  
  // dynamic obstacles
  double prediction_time_, bound_, thick_;  // param
  ros::Subscriber sub_;
  void CB_obstacle(const tracking_2d_lidar::TracksArrayMsg::ConstPtr& msg);
  bool updateCost_done_;
  std::vector<double> obst_x0_, obst_y0_, obst_x1_, obst_y1_, last_x_, last_y_;

  // robot itself
  double vel_x_ = 0;
  double vel_y_ = 0;
  double speed_ = 0;
  ros::Subscriber sub_vel_;
  void CB_vel(const geometry_msgs::Twist::ConstPtr& msg);

  double robot_x_ = -10000;
  double robot_y_ = -10000;
  double robot_yaw_ = -10000;

  // boundry
  double min_x_, max_x_, min_y_, max_y_;
};
}
#endif
