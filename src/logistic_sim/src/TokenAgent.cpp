#include "TokenAgent.hpp"

int main(int argc, char *argv[])
{
    tokenagent::TokenAgent TPA;
    TPA.init(argc, argv);
    c_print("@ Inizializzazione finita!",green);
    sleep(3);
    TPA.run();
    return 0;
}