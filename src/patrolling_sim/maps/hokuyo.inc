define hokuyo ranger
(
  sensor(           
    range [ 0.0  5.0 ]
    fov 230
    samples 1081
  )

  # model properties
  color "orange"
  size [ 0.1 0.1 0.1 ]

  block( points 4
    point[0] [0 0]
    point[1] [0 1]
    point[2] [1 1]
    point[3] [1 0]
    z [0 1]
  )
)
