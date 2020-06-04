#include <new_layers/predict_layer.h>
#include <pluginlib/class_list_macros.h>
#include <tf/tf.h>

#include <base_local_planner/line_iterator.h>  // bresenham 
#include <math.h>

PLUGINLIB_EXPORT_CLASS(new_layers::PredictLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;                 // 254
using costmap_2d::INSCRIBED_INFLATED_OBSTACLE;     // 253
using costmap_2d::FREE_SPACE;                      // 0
static const unsigned char NOT_COLLISION_HIGH = 252;

namespace new_layers
{

PredictLayer::PredictLayer() {}

void PredictLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;

  // params
  nh.param<double>("/new_layers/predict_layer/prediction_time", prediction_time_, 4.0);
  nh.param<double>("/new_layers/predict_layer/bound", bound_, 5.0);
  nh.param<double>("/new_layers/predict_layer/predict_line_thickness", thick_, 8.0);
  

  // setup callback for ostacles
  updateCost_done_ = true;
  sub_ = nh.subscribe("/tracks", 1, &PredictLayer::CB_obstacle, this);

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &PredictLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

void PredictLayer::CB_obstacle(const tracking_2d_lidar::TracksArrayMsg::ConstPtr& msg){
  
  std::vector<double> obst_x0, obst_y0, obst_x1, obst_y1;

  if(msg->tracks.empty()){
    updateCost_done_ = true;
  }else{

  for (size_t i=0; i < msg->tracks.size(); ++i){
    // set start point and end point
    double x0 = msg->tracks.at(i).x.data;
    double y0 = msg->tracks.at(i).y.data;
    double x1 = x0 + prediction_time_ * msg->tracks.at(i).vx.data;
    double y1 = y0 + prediction_time_ * msg->tracks.at(i).vy.data;

    obst_x0.push_back(x0);
    obst_y0.push_back(y0);
    obst_x1.push_back(x1);
    obst_y1.push_back(y1);

    // set boundary to include every tracks
    // double min_x01 = std::min(x0, x1);
    // double max_x01 = std::max(x0, x1);
    // double min_y01 = std::min(y0, y1);
    // double max_y01 = std::max(y0, y1);
    
    // if(i == 0){  // initiate min, max
    //   min_x_ = min_x01;
    //   max_x_ = max_x01;
    //   min_y_ = min_y01;
    //   max_y_ = max_y01;
    // }

    // min_x_ = std::min(min_x_, min_x01);
    // max_x_ = std::max(max_x_, max_x01);
    // min_y_ = std::min(min_y_, min_y01);
    // max_y_ = std::max(max_y_, max_y01);
  }

  // only update obst when updateCost is done
  //if(updateCost_done_){
    // assign new ones
    obst_x0_ = obst_x0;
    obst_y0_ = obst_y0;
    obst_x1_ = obst_x1;
    obst_y1_ = obst_y1;
    updateCost_done_ = false;
  //}
  }
}


void PredictLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

void PredictLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;

  // if not set, the default bound covers the whole map
  min_x_ = robot_x - bound_;
  min_y_ = robot_y - bound_;
  max_x_ = robot_x + bound_;
  max_y_ = robot_y + bound_;

  *min_x = std::min(*min_x, min_x_);
  *min_y = std::min(*min_y, min_y_);
  *max_x = std::max(*max_x, max_x_);
  *max_y = std::max(*max_y, max_y_);

  // clears this layer's map before each update step
  // for( int i=0; i<getSizeInCellsX()*getSizeInCellsY(); i++) {
  //   costmap_[i] = FREE_SPACE;
  // }
}

void PredictLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_)
    return;
  
  // set cost 
  if (!obst_x0_.empty()){
    // set the current cost
    for(int i = 0; i < obst_x0_.size(); i++){
      // Bresenham
      unsigned int x0_map;
      unsigned int y0_map;

      // --- edit master_grid
      if(master_grid.worldToMap(obst_x0_.at(i), obst_y0_.at(i), x0_map, y0_map)){
        unsigned int x1_map;
        unsigned int y1_map;
        if(master_grid.worldToMap(obst_x1_.at(i), obst_y1_.at(i), x1_map, y1_map)){
          
          // draw a line with thickness
          // double m = -1.0 * (x1_map - x0_map)/(y1_map - y0_map);
          // double dx = thick_/2.0/sqrt(1.0+m*m);
          // double dy = dx * m;
          double theta = atan2(-1.0 * (x1_map - x0_map), (y1_map - y0_map));  // in rad.
          double dx = int(round(thick_/2.0*cos(theta)));
          double dy = int(round(thick_/2.0*sin(theta)));
          for(base_local_planner::LineIterator line_(dx, dy, -dx, -dy); line_.isValid(); line_.advance()){ 
            double x0 = x0_map+line_.getX();
            double y0 = y0_map+line_.getY();
            double x1 = x1_map+line_.getX();
            double y1 = y1_map+line_.getY();
            // make sure the two ends are within the map
            if(x0<master_grid.getSizeInCellsX() && x1<master_grid.getSizeInCellsX() && y0<master_grid.getSizeInCellsY() && y1<master_grid.getSizeInCellsY()){
              // points generated by Bresenham
              for(base_local_planner::LineIterator line(x0, y0, x1, y1); line.isValid(); line.advance()){ 
                master_grid.setCost(line.getX(), line.getY(), NOT_COLLISION_HIGH);
              }
              for(base_local_planner::LineIterator line(x0, y0+1, x1, y1+1); line.isValid(); line.advance()){ 
                master_grid.setCost(line.getX(), line.getY(), NOT_COLLISION_HIGH);
              }
            }
          }
        }
      }
    }
  

    // // clear the previous obst
    // for(int i = 0; i < last_x_.size(); i++){
    //   unsigned int mx;
    //   unsigned int my;

    //   if(master_grid.worldToMap(last_x_.at(i), last_y_.at(i), mx, my)){
    //     master_grid.setCost(mx, my, FREE_SPACE);
    //   }
    // }

    // clear old ones
    // last_x_ = obst_x_;
    // last_y_ = obst_y_;
    obst_x0_.clear();
    obst_y0_.clear();
    obst_x1_.clear();
    obst_y1_.clear();
    updateCost_done_ = true;
  }
}

} // end namespace
