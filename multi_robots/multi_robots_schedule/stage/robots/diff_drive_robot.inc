define laser ranger
(
  sensor
  (
    range_max 6.5
    fov 360.0
    samples 1800
  )
  # generic model properties
  color "black"
  size [ 0.06 0.15 0.03 ]
)

define diff_drive_robot position
(
  pose [ 0.0 0.0 0.0 0.0 ]

  localization "gps" # exact localization

  size [ 0.220 0.22 0.40 ]
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1
  color "red"

  # kinematics
  drive "diff"

  # spawn sensors
  laser(pose [ 0.5 0.0 -0.11 0.0 ])
)
