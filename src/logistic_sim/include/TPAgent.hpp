#include "LogicAgent.hpp"
#include "patrolling_sim/Token.h"

namespace tpagent
{
    using namespace logicagent;

    class TPAgent : public LogicAgent
    {
        protected:
            ros::Publisher token_pub;
            ros::Subscriber token_sub;

            std::vector< std::vector<uint> > token_weight_map;
            bool reached_pickup, go_home, reached_home;

            uint home_vertex;

            uint num_robots;

        public:
            virtual void init(int argc, char **argv);
            virtual void run();
            virtual void onGoalComplete();
            virtual int compute_next_vertex();

            void token_callback(const patrolling_sim::TokenConstPtr &msg);
            void init_tw_map();
            void tp_dijkstra(uint source, uint destination, int *shortest_path, uint &elem_s_path);
    };
}

#include "impl/TPAgent.i.hpp"