define myobstacle position
(
  localization "gps"
  size [ 4.4 1.8 1.5 ]
  gui_nose 1
  drive "car"
  color "blue"
  
  # [ xmin xmax ymin ymax zmin zmax amin amax ]				
  velocity_bounds [-30 30 -1 1 -1 1 -90 90 ]					
  acceleration_bounds [-30 30 -1 1 -1 1 -90 90]
)