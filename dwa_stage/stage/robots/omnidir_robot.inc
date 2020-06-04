define laser ranger
(
  sensor
  (
    range_max 6.5
    fov 270.0 # field of view
    samples 640
  )
  # generic model properties
  color "black"
  size [ 0.06 0.15 0.03 ]
)

define omnidir_robot position
(
  pose [ 0.0 0.0 0.0 0.0 ]

  #localization "odom"

  #odom_error [0.03 0.03 999999 999999 999999 0.02]
  #odom_error [0.03 0.03 0.03 0.03]
  #odom_error [0.0 0.0 0.0 0.0]

  size [ 0.25 0.25 0.40 ]
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1   # draw nose showing heading
  color "red"

  # kinematics
  drive "omni"

  # spawn sensors
  laser(pose [ -0.1 0.0 -0.11 0.0 ])
)
