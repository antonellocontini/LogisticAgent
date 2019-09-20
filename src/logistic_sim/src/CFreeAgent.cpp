#include "CFreeAgent.hpp"

using namespace cfreeagent;

void CFreeAgent::token_callback(const logistic_sim::TokenConstPtr &msg)
{
    c_print("CFA token callback",yellow, P);
    
}

int main(int argc, char *argv[])
{
    cfreeagent::CFreeAgent CFA;
    CFA.init(argc, argv);
    c_print("@ SPART", green);
    sleep(3);
    CFA.run();
    return 0;
}