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
#include "CycleAgent.hpp"


using namespace std;

//---[MAIN]---------------------------//

int main (int argc, char** argv)
{
    cycleagent::CycleAgent CA;
    CA.init (argc, argv);
    c_print("@ Inizializzazione finita!",green);
    sleep(3);
    CA.init_agent();
    sleep(3);
    CA.run();
    return 0;
}