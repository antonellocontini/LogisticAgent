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
#include "CycleOptAgent.hpp"

using namespace std;

//---[MAIN]---------------------------//

int main (int argc, char** argv)
{
    coa::COA COA;
    COA.init (argc, argv);
    c_print("@ Inizializzazione finita!",green);
    COA.run();
    return 0;
}