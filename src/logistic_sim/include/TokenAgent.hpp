#include "Agent.hpp"
#include "patrolling_sim/Token.h"

namespace tokenagent
{
    using namespace agent;

    class TokenAgent : public Agent
    {
        protected:
            ros::Publisher token_pub;
            ros::Subscriber token_sub;

        public:
            virtual void init(int argc, char **argv);
            virtual void run();
            virtual void onGoalComplete();
            virtual int compute_next_vertex();

            void token_callback(const patrolling_sim::TokenConstPtr &msg);
    };
}

#include "impl/TokenAgent.i.hpp"