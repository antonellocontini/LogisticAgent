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
            bool reached_home = false;
            bool init_wait_done = false;

            uint init_next_vertex;
            uint src_vertex = 13    ;
           
            logistic_sim::Mission current_mission;
            std::vector< std::vector<uint> > token_weight_map;

            ros::Time goal_start_time;

            float total_distance = 0.0f;
            uint missions_completed = 0;
            uint tasks_completed = 0;
            uint interference_number = 0;

        public:

            virtual void init(int argc, char **argv);
            virtual void run();
            virtual void onGoalComplete();
            virtual int compute_next_vertex();
            bool check_interference_token(const logistic_sim::Token &token);
            void goalFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback) override;
            int compute_cost_of_route(std::vector<uint> r);

            void tp_dijkstra(uint source, uint destination, int *shortest_path, uint &elem_s_path);
            void init_tw_map();
            void token_callback(const logistic_sim::TokenConstPtr &msg);
    };
}

#include "impl/TokenAgent.i.hpp"