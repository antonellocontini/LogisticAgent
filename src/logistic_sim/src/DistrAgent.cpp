#include "DistrAgent.hpp"

int main(int argc, char *argv[])
{
    distragent::DistrAgent TPA;
    TPA.init(argc, argv);
    c_print("@ Inizializzazione finita!",green);
    sleep(3);
    TPA.run();
    return 0;
}