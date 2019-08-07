#include "TaskPlanner.hpp"

using namespace taskplanner;

//------------------------------------------------------------------------------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "task_planner");

  ros::NodeHandle nh_("~");

  taskplanner::TaskPlanner TP(nh_);

  TP.init(argc, argv);

  //   while (!OK)
  //   {
  //     //   c_print("aspetto i robot", red);
  // //   }

  c_print("inizializzazione finita!", green);

  // TP.run();

  ros::AsyncSpinner spinner(2);

  spinner.start();

//   while (!OK)
//   {
//   }

//   // calcolo tutte le possibilita' finche' non finisco i task

//   auto nat = TP.processedTask();

//   int size = nat.size();

//   // optional second argument is the partition size (1--size)

//   try
//   {
//     partition::iterator it(size);

//     while (true)
//     {
//       auto all_part = *it[nat];

//       cout << all_part << "\n";

//       auto as = all_part.size();

//       int c = 0;

//       for (auto i = 0; i < as; i++)
//       {
//         auto part = all_part[i];
//         ProcessTask pt;
//         c++;
//         for (auto j = 0; j < part.size(); j++)
//         {
//             pt.id = c;
//             pt.mission.push_back(part[j]);
//           cout << part[j] << " ";
//         }
//         cout << " \n";

//         TaskPlanner::v_pt.push_back(pt);
//       }

//       ++it;
//     }
//   }
//   catch (std::overflow_error &)
//   {
//   }

  ros::waitForShutdown();

  return 0;
}
