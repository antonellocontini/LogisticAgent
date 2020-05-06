#include "OnlineDCOPAgent.hpp"

namespace onlinedcopagent
{

void OnlineDCOPAgent::token_callback(const logistic_sim::TokenConstPtr &msg)
{
    ROS_INFO_STREAM("TODO DCOP TOKEN CALLBACK");
    // TODO
}

} // namespace onlinedcopagent

int main(int argc, char *argv[])
{
    onlinedcopagent::OnlineDCOPAgent ODA;
    ODA.init(argc, argv);
    ROS_INFO_STREAM("@ ONLINE DCOP");
    sleep(3);
    ODA.run();
    return 0;
}