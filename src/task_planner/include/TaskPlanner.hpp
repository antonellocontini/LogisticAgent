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

#include "logistic_sim/RobotReady.h"

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

// override operatori per lettura e scrittura missioni su file
ostream &operator<<(ostream &os, const logistic_sim::Mission &m);
ostream &operator<<(ostream &os, const vector<logistic_sim::Mission> &v);

istream &operator>>(istream &is, logistic_sim::Mission &m);
istream &operator>>(istream &is, vector<logistic_sim::Mission> &v);

// confronto per test
bool operator==(const logistic_sim::Mission &lhs, const logistic_sim::Mission &rhs);
bool operator!=(const logistic_sim::Mission &lhs, const logistic_sim::Mission &rhs);

// override operatori per lettura e scrittura percorsi su file
ostream &operator<<(ostream &os, const logistic_sim::Path &p);
ostream &operator<<(ostream &os, const vector<logistic_sim::Path> &v);

istream &operator>>(istream &is, logistic_sim::Path &p);
istream &operator>>(istream &is, vector<logistic_sim::Path> &v);

class TaskPlanner
{
public:
    TaskPlanner(ros::NodeHandle &nh_, const std::string &name = "TaskPlanner");
    ~TaskPlanner(){};

    vertex *vertex_web;
    uint dimension;
    string mapname;

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
    bool PERMUTATIONS;
    int num_robots, robots_ready_count = 0;
    vector<bool> robots_ready_status;

    ros::Time start_time;

    std::map<std::string, std::vector<uint>> map_homes = {{"model6", {0,1,2,3,4,5}}, {"grid", {1,2,3,21,22,23}}, {{"icelab"}, {0,1,2,26,27,28}}, {"model5", {0,1,2,25,26,27}}};
    std::map<std::string, uint> map_src = {{"model6", 13}, {"grid", 7}, {"icelab", 22} , {"icelab_black", 18}, {"model5", 6}};
    std::map<std::string, std::vector<uint> > map_dsts = {{"model6", {18,23,28}}, {"grid", {16,17,18}},
                                                          {"icelab", {10,13,16}}, {"icelab_black", {26,33,42}},
                                                          {"model5", {11,16,21}}};
    uint src_vertex;
    std::vector<uint> dst_vertex;
    std::vector<uint> home_vertex;
    std::vector<std::vector<uint> > paths;

    int compute_cost_of_route(std::vector<uint> &route);

    void missions_generator(std::string &gen_type);

    void u_missions_generator();
    void nu_missions_generator();

    logistic_sim::Mission create_mission(uint type, int id);
    
    virtual void set_partition();
    virtual std::vector<logistic_sim::Path> path_partition(logistic_sim::Token &token);

    void init(int argc, char **argv);

    virtual void token_Callback(const logistic_sim::TokenConstPtr &msg);

    bool robot_ready(logistic_sim::RobotReady::Request &req,
                     logistic_sim::RobotReady::Response &res);

    virtual void allocate_memory() { }

protected:
    // per l'inizializzazione e il token dei task
    ros::Subscriber sub_token;
    ros::Publisher pub_token;
    std::string name;
};

} // namespace taskplanner

#include "impl/TaskPlanner.i.hpp"