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

        public:
            virtual void init(int argc, char **argv);
            virtual void run();
            virtual void onGoalComplete();
            virtual int compute_next_vertex();

            void tp_dijkstra(uint source, uint destination, int *shortest_path, uint &elem_s_path);
            // void init_tw_map();
            void token_callback(const logistic_sim::TokenConstPtr &msg);
    };
}

#include "impl/DistrAgent.i.hpp"