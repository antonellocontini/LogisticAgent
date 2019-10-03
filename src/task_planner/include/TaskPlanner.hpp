#pragma once
// #include <color_cout.hpp> //lib
// #include <partition.hpp>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <iterator>
#include <queue>
#include <set>
#include <sstream>
#include <stdexcept>
#include <vector>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <stdexcept>

#include <ros/package.h> //to get pkg path
#include <ros/ros.h>
#include <sys/stat.h>

#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Bool.h>

#include <logistic_sim/Token.h>
#include <logistic_sim/Mission.h>

#include "struct_vertex.hpp"
#include "message_types.hpp"
#include "partition.hpp"
#include "get_graph.hpp"
#include "color_cout.hpp"

namespace taskplanner
{

const std::string PS_path = ros::package::getPath("logistic_sim");

// dati relativi al singolo robot
struct MonitorData
{
    float tot_distance = 0.0f;
    int interference_num = 0;
    int completed_missions = 0;
    int completed_tasks = 0;
    float total_time = 0.0f;
    // campi ausiliari
    int last_dst = -1;
};

ostream &operator<<(ostream &os, const MonitorData &md);

ostream &operator<<(ostream &os, const vector<MonitorData> &v);

class TaskPlanner
{
public:
    TaskPlanner(ros::NodeHandle &nh_, const std::string &name = "TaskPlanner");
    ~TaskPlanner(){};

    vertex *vertex_web;
    uint dimension;

    std::string ALGORITHM;
    std::string GENERATION;
    uint TEAM_CAPACITY = 0;
    std::vector<uint> CAPACITY;
    uint TEAM_SIZE = 0;
    uint nTask = 0;
    uint id = 0;
    // uint dst_vertex[3] = {18, 23, 28};
    // per ora statici senza funzioni
    const uint p_11[8] = {6, 7, 9, 12, 11, 10, 8, 5};
    const uint p_16[12] = {6, 7, 9, 12, 14, 17, 16, 15, 13, 10, 8, 5};
    const uint p_21[16] = {6, 7, 9, 12, 14, 17, 19, 22, 21, 20, 18, 15, 13, 10, 8, 5};
    const uint p_11_16[14] = {6, 7, 9, 12, 11, 12, 14, 17, 16, 15, 13, 10, 8, 5};
    const uint p_11_21[18] = {6, 7, 9, 12, 11, 12, 14, 17, 19,
                              22, 21, 20, 18, 15, 13, 10, 8, 5};
    const uint p_16_21[18] = {6, 7, 9, 12, 14, 17, 16, 17, 19, 22,
                              21, 20, 18, 15, 13, 10, 8, 5};
    const uint p_11_16_21[20] = {6, 7, 9, 12, 11, 12, 14, 17, 16, 17,
                                 19, 22, 21, 20, 18, 15, 13, 10, 8, 5};
    //----------------------------------------------------------------------------

    vector<logistic_sim::Mission> missions;

    vector<MonitorData> robots_data;
    bool first_round = true;
    int num_robots;

    ros::Time start_time;

    std::map<std::string, uint> map_src = {{"model6", 13}, {"grid", 7}};
    std::map<std::string, std::vector<uint> > map_dsts = {{"model6", {18,23,28}}, {"grid", {16,17,18}}};
    uint src_vertex;
    std::vector<uint> dst_vertex;
    std::vector<std::vector<uint> > paths;

    int compute_cost_of_route(std::vector<uint> &route);

    void missions_generator(std::string &gen_type);

    void u_missions_generator();
    void nu_missions_generator();

    logistic_sim::Mission create_mission(uint type, int id);
    
    virtual void set_partition();

    void init(int argc, char **argv);

    virtual void token_Callback(const logistic_sim::TokenConstPtr &msg);

protected:
    // per l'inizializzazione e il token dei task
    ros::Subscriber sub_token;
    ros::Publisher pub_token;
    std::string name;
};

} // namespace taskplanner

#include "impl/TaskPlanner.i.hpp"