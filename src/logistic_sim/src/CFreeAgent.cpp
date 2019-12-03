#include "CFreeAgent.hpp"

using namespace cfreeagent;


int main(int argc, char* argv[])
{
  cfreeagent::CFreeAgent CFA;
  CFA.init(argc, argv);
  c_print("@ CFREE", green);
  sleep(3);
  CFA.run();
  return 0;
}
