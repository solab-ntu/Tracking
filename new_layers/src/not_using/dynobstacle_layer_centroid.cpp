#include <new_layers/dynobstacle_layer.h>
#include <pluginlib/class_list_macros.h>
#include <tf/tf.h>

PLUGINLIB_EXPORT_CLASS(new_layers::DynobstacleLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

namespace new_layers
{

DynobstacleLayer::DynobstacleLayer() {}

void DynobstacleLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;

  // params
  nh.param("/new_layers/dynobstacle_layer/prediction_time", prediction_time_, 3.0);

  // setup callback for ostacles
  updateCost_done_ = true;
  sub_ = nh.subscribe("/standalone_converter/costmap_obstacles", 1, &DynobstacleLayer::CB_obstacle, this);

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &DynobstacleLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

void DynobstacleLayer::CB_obstacle(const costmap_converter::ObstacleArrayMsg::ConstPtr& msg){
  
  std::vector<double> obst_x;
  std::vector<double> obst_y;

  for (size_t i=0; i < msg->obstacles.size(); ++i){
    // calculate centroid 
    double centroid_x;
    double centroid_y;
    int num_points = msg->obstacles.at(i).polygon.points.size();
    for (size_t j=0; j < num_points; ++j){
      centroid_x += msg->obstacles.at(i).polygon.points[j].x;
      centroid_y += msg->obstacles.at(i).polygon.points[j].y;
    }
    centroid_x = centroid_x/double(num_points);
    centroid_y = centroid_y/double(num_points);

    // initiate min, max
    if(i == 0){
      min_x_ = centroid_x;
      max_x_ = centroid_x;
      min_y_ = centroid_y;
      max_y_ = centroid_y;
    }

    min_x_ = std::min(min_x_, centroid_x);
    max_x_ = std::max(max_x_, centroid_x);
    min_y_ = std::min(min_y_, centroid_y);
    max_y_ = std::max(max_y_, centroid_y);

    // constant velocity prediction

    // // convert quaternion to RPY
    // tf::Quaternion q(
    //   msg->obstacles.at(i).orientation.x,
    //   msg->obstacles.at(i).orientation.y,
    //   msg->obstacles.at(i).orientation.z,
    //   msg->obstacles.at(i).orientation.w);
    // tf::Matrix3x3 m(q);
    // double roll, pitch, yaw;
    // m.getRPY(roll, pitch, yaw);

    // from robot coordinate to world coordinate 
    double x = centroid_x + msg->obstacles.at(i).velocities.twist.linear.x * prediction_time_;
    double y = centroid_y + msg->obstacles.at(i).velocities.twist.linear.y * prediction_time_;

    // store centroids
    obst_x.push_back(x);
    obst_y.push_back(y);
  }

  if(updateCost_done_){
    // assign new ones
    obst_x_ = obst_x;
    obst_y_ = obst_y;
    updateCost_done_ = false;
  }
}


void DynobstacleLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

void DynobstacleLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;

  *min_x = std::min(*min_x, min_x_);
  *min_y = std::min(*min_y, min_y_);
  *max_x = std::max(*max_x, max_x_);
  *max_y = std::max(*max_y, max_y_);
}

void DynobstacleLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_)
    return;

  // set cost 
  if (!obst_x_.empty()){
    // set the current cost
    for(int i = 0; i < obst_x_.size(); i++){
      unsigned int mx;
      unsigned int my;

      if(master_grid.worldToMap(obst_x_.at(i), obst_y_.at(i), mx, my)){
        master_grid.setCost(mx, my, LETHAL_OBSTACLE);
      }
    }

    // clear the previous obst
    for(int i = 0; i < last_x_.size(); i++){
      unsigned int mx;
      unsigned int my;

      if(master_grid.worldToMap(last_x_.at(i), last_y_.at(i), mx, my)){
        master_grid.setCost(mx, my, FREE_SPACE);
      }
    }

    // clear old ones
    last_x_ = obst_x_;
    last_y_ = obst_y_;
    obst_x_.clear();
    obst_y_.clear();
    updateCost_done_ = true;
  }
}

} // end namespace
