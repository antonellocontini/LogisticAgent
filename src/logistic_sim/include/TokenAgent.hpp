#include "Agent.hpp"
#include "logistic_sim/Token.h"

namespace tokenagent
{
    using namespace agent;

    class TokenAgent : public Agent
    {
        protected:
            ros::Publisher token_pub;
            ros::Subscriber token_sub;

            bool need_task = true;
            bool reached_pickup = false;
            bool go_home = false;
            bool init_wait_done = false;

            uint init_next_vertex;
           
            logistic_sim::Mission current_mission;
            std::vector< std::vector<uint> > token_weight_map;

            ros::Time goal_start_time;

        public:

            virtual void init(int argc, char **argv);
            virtual void run();
            virtual void onGoalComplete();
            virtual int compute_next_vertex();

            void tp_dijkstra(uint source, uint destination, int *shortest_path, uint &elem_s_path);
            void init_tw_map();
            void token_callback(const logistic_sim::TokenConstPtr &msg);
    };
}

#include "impl/TokenAgent.i.hpp"