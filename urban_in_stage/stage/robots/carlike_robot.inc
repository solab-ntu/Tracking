define laser ranger
(
  sensor
  (
    range [ 0.05 50.0 ]
    fov 90.0
    samples 640
  )
  # generic model properties
  color "black"
  size [ 0.06 0.15 0.03 ]
)

#
# Robot model:
# footprint (counter-clockwise): [-0.1,-0.125], [0.5,-0.125], [0.5,0.125], [-0.1,0.125]
# center of rotation: [0,0]
# wheelbase: 0.4

define carlike_robot position
(
  pose [ 0.0 0.0 0.0 0.0 ]

  odom_error [0.03 0.03 999999 999999 999999 0.02]

  size [ 4.4 1.8 1.5 ] # this models the footprint (rectangular), but shifted such that the bottom-left corner is in [0,0]. The center of rotation now here at [0.3, 0.125, 0.2]
  # correct center of rotation:
  origin [ 2.2 0.9 0.0 0.0]
  gui_nose 1
  color "red"

  # kinematics
  drive "car"
  wheelbase 2.64 # distance between both axles
  
  # spawn sensors
  laser(pose [ 4.4 1.2 -0.6 0.0 ])

  # [ xmin xmax ymin ymax zmin zmax amin amax ]				
  velocity_bounds [-30 30 -1 1 -1 1 -90 90 ]					
  acceleration_bounds [-30 30 -1 1 -1 1 -90 90]
)
