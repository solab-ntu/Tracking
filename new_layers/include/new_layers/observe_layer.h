#ifndef NEW_LAYERS_OBSERVE_LAYER_H_
#define NEW_LAYERS_OBSERVE_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>

#include <vector>

namespace new_layers
{

class ObserveLayer : public costmap_2d::Layer
{
public:
  ObserveLayer();

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                             double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

private:
  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);

  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;

  // param
  double update_bound_width_;

  // 
  unsigned int update_count_;
  unsigned int  obstacle_cell_total_; 
  bool writeOutput();
};
}
#endif