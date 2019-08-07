#include "TPAgent.hpp"

using namespace std;

int main (int argc, char** argv)
{
    tpagent::TPAgent TPA;
    TPA.init(argc, argv);
    c_print("@ Inizializzazione finita!",green);
    sleep(3);
    TPA.init_agent();
    sleep(3);
    TPA.run();
    return 0;
}