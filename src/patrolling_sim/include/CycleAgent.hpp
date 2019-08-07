#include "PatrolAgent.hpp"
/*
vector<Task> mission
vector<int>  route
uint id_vertex, id_task
uint loading[5] vertici superiori
uint dowload[5] vertici inferiori
*/
 
namespace cycleagent
{
    using namespace patrolagent;

    class CycleAgent : public PatrolAgent
    {
        protected:
            bool first =  true;
            // bool init =false;
            bool mission_complete;
            bool go_src;
            std::vector<int> route_to_src;

        public:
            // CycleAgent();
            virtual void run();
            virtual void onGoalComplete();
            virtual int compute_next_vertex();
    };
}//namespace cycleagent

#include "impl/CycleAgent.i.hpp"