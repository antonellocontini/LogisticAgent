#include <string>
#include <sstream>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>

#include "color_cout.hpp"
#include "PatrolAgent.hpp"
#include "CFAgent.hpp"

using namespace std;

int main(int argc, char **argv)
{
    cfa::CFA CFA;
    CFA.init(argc, argv);
    c_print("@init",green);
    CFA.run();

    return 0;
}