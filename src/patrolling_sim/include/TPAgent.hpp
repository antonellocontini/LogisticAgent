#include "PatrolAgent.hpp"
#include "patrolling_sim/Token.h"

namespace tpagent
{
    using namespace patrolagent;

    class TPAgent : public PatrolAgent
    {
        protected:
            ros::Publisher token_pub;
            ros::Subscriber token_sub;

            std::vector< std::vector<uint> > token_weight_map;
            bool reached_pickup, go_home;

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