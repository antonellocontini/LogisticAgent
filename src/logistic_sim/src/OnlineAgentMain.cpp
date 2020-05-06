#include "OnlineAgent.hpp"

int main(int argc, char *argv[])
{
  onlineagent::OnlineAgent OA;
  OA.init(argc, argv);
  c_print("@ ONLINE", green);
  sleep(3);
  OA.run();
  return 0;
}