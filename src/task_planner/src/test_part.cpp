
#include <iostream>
#include <sstream>
#include <algorithm>
#include <stdexcept>

#include "partition.hpp"

#include "TaskPlanner.hpp"

  using t_coalition = std::pair<std::vector<logistic_sim::Mission>, logistic_sim::Mission>;
// std::ostream&
// operator<<(std::ostream& out, partition::iterator &it)
// {
//   out << '(';

//   if (it->size() > 1)
//     std::copy(it->begin(), it->end()-1,
//               std::ostream_iterator<unsigned>(out, " "));

//   out << *(it->end()-1) << ')';

//   return out;
// }


void print_coalition(const t_coalition &coalition)
{
    auto tasks = coalition.first;
    auto mission = coalition.second;
    c_print("Mission id: ", mission.ID, green, P);
    // std::cout << "pd: " << mission.PATH_DISTANCE << "\n";
    // std::cout << "td: " << mission.TOT_DEMAND << "\n";
    std::cout << " V: " << mission.V << "\n";
    // auto size_dsts = mission.DSTS.size();
    // std::cout << "dsts"
    //           << "\n";
    // for (int i = 0; i < size_dsts; i++)
    // {
    //     std::cout << mission.DSTS[i] << " ";
    // }
    // std::cout << "\n";

    // auto size_boh = mission.DEMANDS.size();
    // std::cout << "demands"
    //           << "\n";
    // for (int i = 0; i < size_boh; i++)
    // {
    //     std::cout << mission.DEMANDS[i] << " ";
    // }
    // std::cout << "\n";

    // auto size_route = mission.ROUTE.size();
    // std::cout << "route"
    //           << "\n";
    // for (int i = 0; i < size_route; i++)
    // {
    //     std::cout << mission.ROUTE[i] << " ";
    // }
    // std::cout << "\n";

    // c_print("tasks", magenta, P);

    // auto size_tasks = tasks.size();

    // for (auto i = 0; i < size_tasks; i++)
    // {
    //     auto t = tasks[i];
    //     c_print("task id: ", t.ID, magenta, P);
    //     std::cout << "pd: " << t.PATH_DISTANCE << "\n";
    //     std::cout << "td: " << t.TOT_DEMAND << "\n";
    //     std::cout << " V: " << t.V << "\n";
    //     auto size_dsts = t.DSTS.size();
    //     std::cout << "dsts"
    //               << "\n";
    //     for (int i = 0; i < size_dsts; i++)
    //     {
    //         std::cout << t.DSTS[i] << " ";
    //     }
    //     std::cout << "\n";

    //     auto size_boh = t.DEMANDS.size();
    //     std::cout << "demands"
    //               << "\n";
    //     for (int i = 0; i < size_boh; i++)
    //     {
    //         std::cout << t.DEMANDS[i] << " ";
    //     }
    //     std::cout << "\n";

    //     auto size_route = t.ROUTE.size();
    //     std::cout << "route"
    //               << "\n";
    //     for (int i = 0; i < size_route; i++)
    //     {
    //         std::cout << t.ROUTE[i] << " ";
    //     }
    //     std::cout << "\n";
    // }

    c_print("fine mission id: ", mission.ID, red, P);
}

int main(int argc, char **argv)
{
  using namespace taskplanner;

  // sinistra task, destra mission

  ros::init(argc, argv, "task_planner");

  ros::NodeHandle nh_; // con ~ avremmo il prefisso sui topic

  TaskPlanner TP(nh_);

  TP.missions_generator();

  int CAPACITY_ROBOT = 6;

  auto size_m = TP.missions.size();

  std::vector<t_coalition> v_coalitions;

  try
  {
    partition::iterator it(size_m);
    int id = 0;
    while (1)
    {
      std::vector<std::vector<logistic_sim::Mission>> all_part = *it[TP.missions]; // vettore di missioni
      auto subset = it.subsets();
      t_coalition candidate;
      candidate.second.ID = id;
      id++;
      // candidate.first.resize(subset);
      uint id_m = 0;
      double tmp_V = 0;
      for (auto i = 0; i < subset; i ++)
      {
        uint tmp_D = 0;
        std::vector<logistic_sim::Mission> part = all_part[i];
        candidate.first = part;
      }
      ++it;

      // print_coalition(candidate);
    }
  }
  catch (std::overflow_error &)
  {
  }
  c_print("fine preparazione!", red);

  return 0;
}