#include <new_layers/observe_layer.h>
#include <pluginlib/class_list_macros.h>

#include <iostream>
#include <fstream>
#include <string>

PLUGINLIB_EXPORT_CLASS(new_layers::ObserveLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::INSCRIBED_INFLATED_OBSTACLE;

namespace new_layers
{

ObserveLayer::ObserveLayer() {}

void ObserveLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;

  // params
  nh.param("/robot_0/move_base/new_layers/observe_layer/update_bound_width", update_bound_width_, 5.0);

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &ObserveLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);

  //
  update_count_ = 0;
  obstacle_cell_total_ = 0;
}


void ObserveLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

void ObserveLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;

  // 
  *min_x = robot_x - update_bound_width_/2.0;
  *min_y = robot_y - update_bound_width_/2.0;
  *max_x = robot_x + update_bound_width_/2.0;
  *max_y = robot_y + update_bound_width_/2.0;
}

void ObserveLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_)
    return;
  
  unsigned char* charmap = master_grid.getCharMap();
  unsigned int x_size = master_grid.getSizeInCellsX();
  unsigned int y_size = master_grid.getSizeInCellsY();

  int obstacle_cell_num = 0;
  for (int i=0; i<x_size*y_size; i++){
    unsigned char* charmap_i = charmap + i;
    if (*charmap_i == LETHAL_OBSTACLE || *charmap_i == INSCRIBED_INFLATED_OBSTACLE){
      obstacle_cell_num ++;
    }
  }

  obstacle_cell_total_ += obstacle_cell_num;
  update_count_++;
  if (!writeOutput()){
    ROS_INFO("[observe_layer] Failed to write output to text file.");
  }
}

bool ObserveLayer::writeOutput(){
  try{
    std::ofstream myfile;
    myfile.open ("/home/wuch/catkin_ws/src/nav_tuning/batch_mode/obstacle_cell.txt");
    int obstacle_cell_avg = obstacle_cell_total_/update_count_;
    myfile << obstacle_cell_avg;
    myfile.close();
    return true;
  } catch (...) {
    return false;
  }
}

} // end namespace