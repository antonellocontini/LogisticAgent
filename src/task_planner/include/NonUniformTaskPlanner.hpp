#pragma once
#include "TaskPlanner.hpp"

namespace nonuniformtaskplanner
{

using namespace taskplanner;

// un taskplanner che genera missioni per la casistica
// nella quale i tipi di oggetti non sono in proporzioni
// uguali
class NonUniformTaskPlanner : public TaskPlanner
{
public:
    NonUniformTaskPlanner(ros::NodeHandle &nh_, uint t1_size=5, uint t2_size=3, uint t3_size=1);
    ~NonUniformTaskPlanner(){};
    
    void missions_generator() override;
    void token_Callback( const logistic_sim::TokenConstPtr &msg) override;
protected:
    uint t1_size, t2_size, t3_size;
    logistic_sim::Mission create_mission(uint type, int id);
};

} // namespace Nonuniformtaskplanner


#include "impl/NonUniformTaskPlanner.i.hpp"