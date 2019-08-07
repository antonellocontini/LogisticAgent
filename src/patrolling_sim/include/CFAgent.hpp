#include "PatrolAgent.hpp"

namespace cfa
{
using namespace patrolagent;

class CFA : public PatrolAgent
{
  public:
    virtual void run();
    virtual void onGoalComplete();
    virtual int compute_next_vertex();
};
} // namespace cfa

#include "impl/CFAgent.i.hpp"