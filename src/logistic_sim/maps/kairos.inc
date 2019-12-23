define kairos position(
    size [0.847 0.663 0.3]
    origin [0 0 0 0]
    gui_nose 0
    drive "omni"

    # This block approximates a circular shape of a Robot
    block( points 4
        point[0] [ 0.4235 0.3315 ]
        point[1] [ 0.4235 -0.3315 ]
        point[2] [ -0.4235 -0.3315 ]
        point[3] [ -0.4235 0.3315 ]
        z [0 1]
    )
    
    hokuyo( pose [0.4235 0.3315 -0.15 45] )
    hokuyo( pose [-0.4235 -0.3315 -0.15 -135] )

    # Report error-free position in world coordinates
    localization "gps"
    #localization_origin [ 0 0 0 0 ]

    # Some more realistic localization error
    localization "odom"
    odom_error [ 0.01 0.01 0.0 0.1 ]
)
