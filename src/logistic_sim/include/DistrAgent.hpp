#include "Agent.hpp"
#include "logistic_sim/Token.h"

namespace distragent
{
using namespace agent;

class DistrAgent : public Agent
{
protected:
    ros::Publisher token_pub;
    ros::Subscriber token_sub;

    bool need_task = true;
    bool reached_pickup, go_home = false;
    logistic_sim::Task current_task;
    std::vector<std::vector<uint>> token_weight_map;

public:
    uint p_11[8] = {6, 7, 9, 12, 11, 10, 8, 5};
    uint p_16[12] = {6, 7, 9, 12, 14, 17, 16, 15, 13, 10, 8, 5};
    uint p_21[16] = {6, 7, 9, 12, 14, 17, 19, 22, 21, 20, 18, 15, 13, 10, 8, 5};

    uint p_11_16[14] = {6, 7, 9, 12, 11, 12, 14, 17, 16, 15, 13, 10, 8, 5};
    uint p_11_21[18] = {6, 7, 9, 12, 11, 12, 14, 17, 19,
                        22, 21, 20, 18, 15, 13, 10, 8, 5};
    uint p_16_21[18] = {6, 7, 9, 12, 14, 17, 16, 17, 19, 22,
                        21, 20, 18, 15, 13, 10, 8, 5};

    uint p_11_16_21[20] = {6, 7, 9, 12, 11, 12, 14, 17, 16, 17,
                           19, 22, 21, 20, 18, 15, 13, 10, 8, 5};

    virtual void init(int argc, char **argv);
    virtual void run();
    virtual void onGoalComplete();
    virtual int compute_next_vertex();
    //
    uint compute_id_path(std::vector<logistic_sim::Task> m);
    void compute_travell(uint id_path, logistic_sim::Mission m);
    int compute_cost_of_route(std::vector<unit> r);
    //
    void tp_dijkstra(uint source, uint destination, int *shortest_path, uint &elem_s_path);
    void init_tw_map();
    void token_callback(const logistic_sim::TokenConstPtr &msg);
};
} // namespace distragent

#include "impl/DistrAgent.i.hpp"