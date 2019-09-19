
#include <iostream>
#include <sstream>
#include <algorithm>
#include <stdexcept>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <iterator>
#include <queue>
#include <set>
#include <sstream>
#include <stdexcept>
#include <vector>

#include <ros/package.h> //to get pkg path
#include <ros/ros.h>
#include <sys/stat.h>

#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Bool.h>

#include <logistic_sim/Token.h>
#include <logistic_sim/Mission.h>

#include "struct_vertex.hpp"
#include "message_types.hpp"
#include "get_graph.hpp"
#include "color_cout.hpp"
// #include "partition.hpp"
#include "color_cout.hpp"
#include "TaskPlanner.hpp"

// using t_coalition = std::pair<std::vector<logistic_sim::Mission>, logistic_sim::Mission>;

// struct less_V
// {
//   inline bool operator()(const t_coalition &A, const t_coalition &B)
//   {
//     return A.second.V < B.second.V;
//   }
// };

// inline bool operator==(const t_coalition &A, const t_coalition &B)
// {
//   return A.second.ID == B.second.ID ? 1 : 0;
// }

// inline bool operator<(const t_coalition &A, const t_coalition &B)
// {
//   return A.second.V < B.second.V ? 1 : 0;
// }

int main(int argc, char **argv)
{
  using namespace taskplanner;

  // sinistra task, destra mission

  ros::init(argc, argv, "task_planner");

  ros::NodeHandle nh_; // con ~ avremmo il prefisso sui topic

  TaskPlanner TP(nh_);

  TP.missions_generator();

  int CAPACITY_ROBOT = 3;

  auto size_m = TP.missions.size();

  while (1)
  cout << "MA dio cane\n";

  // std::vector<t_coalition> v_coalitions;

  // try
  // {
  //   partition::iterator it(size_m);
  //   int id = 0;
  //   while (1)
  //   {
  //     std::vector<std::vector<logistic_sim::Mission>> all_part = *it[TP.missions]; // vettore di missioni
  //     auto a = all_part.size();
  //     cout << "size_all: " << a << "\n";
  //     auto subset = it.subsets();
  //     t_coalition candidate;
  //     candidate.second.ID = id;
  //     id++;
  //     // candidate.first.resize(subset);
  //     uint id_m = 0;
  //     double tmp_V = 0;
  //     for (auto i = 0; i < subset; i++)
  //     {
  //       uint tmp_D = 0;
  //       // std::vector<logistic_sim::Mission> part = all_part[i];
  //       // //dalla part mi creo la missione
  //       // candidate.first = part;
  //       // for (auto j = 0; j < part.size(); j++)
  //       // {
  //       //   // candidate.second.TOT_DEMAND += part[j].TOT_DEMAND;
  //       //   // copy(part[j].DSTS.begin(), part[j].DSTS.end(), back_inserter(candidate.second.DSTS));
  //       //   // copy(part[j].DEMANDS.begin(), part[j].DEMANDS.end(), back_inserter(candidate.second.DEMANDS));
  //       //   // copy(part[j].ITEM.begin(), part[j].ITEM.end(), back_inserter(candidate.second.ITEM));
  //       //   // candidate.second.PATH_DISTANCE += part[j].PATH_DISTANCE;
  //       // }

  //       // candidate.second.V = (double)candidate.second.PATH_DISTANCE / (double)candidate.second.TOT_DEMAND;
  //       // if (candidate.second.TOT_DEMAND <= CAPACITY_ROBOT)
  //       // {
  //       //   v_coalitions.push_back(candidate);
  //       //   // print_coalition(candidate);
  //       // }
  //     }
  //     ++it;
  //     c_print("DC",red);
  //   }
  // }
  // catch (std::overflow_error &)
  // {
  // }
  // // std::sort(v_coalitions.begin(), v_coalitions.end(), less_V());

  ros::AsyncSpinner spinner(2);

  spinner.start();

  ros::waitForShutdown();

  c_print("fine preparazione!", red);

  return 0;
}