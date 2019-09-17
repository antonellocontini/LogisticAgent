#pragma once

namespace nonuniformtaskplanner
{

NonUniformTaskPlanner::NonUniformTaskPlanner(ros::NodeHandle &nh_, uint t1_size, uint t2_size, uint t3_size)
    : TaskPlanner(nh_), t1_size(t1_size), t2_size(t2_size), t3_size(t3_size)
{

}

} // namespace nonuniformtaskplanner
