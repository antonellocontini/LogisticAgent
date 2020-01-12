#include "OnlineGreedyTaskPlanner.hpp"
#include "partition.hpp"
#include "permutations.hpp"
#include "algorithms.hpp"
#include <memory>

namespace onlinegreedytaskplanner
{
OnlineGreedyTaskPlanner::OnlineGreedyTaskPlanner(ros::NodeHandle &nh_, const std::string &name) : OnlineCentralizedTaskPlanner(nh_, name)
{
}


std::vector<logistic_sim::Path> OnlineGreedyTaskPlanner::path_partition(logistic_sim::Token &token,
                                                                        std::vector<logistic_sim::Mission> &missions)
{
  ofstream stats_file("stats_file.txt");
  c_print("Calculating tasks distribution", green, P);
  // merging task paths with home paths
  std::vector<logistic_sim::Path> robot_paths;
  std::vector<std::vector<logistic_sim::Path>> best_paths(TEAM_SIZE);
  std::vector<uint> best_paths_length(TEAM_SIZE, std::numeric_limits<uint>::max());
  std::vector<bool> possible_paths_found(TEAM_SIZE, false);
  std::vector<logistic_sim::Token> missions_stats(TEAM_SIZE);
  logistic_sim::Token best_token;

  // find starting vertex for new paths (that is the last vertex in the already planned path)
  std::vector<uint> init_vertex;
  for (int i = 0; i < TEAM_SIZE; i++)
  {
    init_vertex.push_back(token.TRAILS[i].PATH.back());
  }

  // home_vertex will contain the home vertex of each robot
  std::vector<uint> home_vertex;
  ros::NodeHandle nh;
  std::vector<double> list;
  nh.getParam("initial_pos", list);

  if (list.empty())
  {
    ROS_ERROR("No initial positions given: check \"initial_pos\" parameter.");
    ros::shutdown();
    exit(-1);
  }

  // read each agent home coordinates to find their home vertex
  for (int value = 0; value < TEAM_SIZE; value++)
  {
    double home_x = list[2 * value];
    double home_y = list[2 * value + 1];
    uint v = IdentifyVertex(vertex_web, dimension, home_x, home_y);
    home_vertex.push_back(v);
    // std::cout << "Robot " << value << "\tVertice iniziale: " << v << std::endl;
  }

  for (int current_team_size = TEAM_SIZE; current_team_size >= 1; current_team_size--)
  {
    try
    {
      c_print("Missions number: ", missions.size(), green, P);
      partition::iterator it(missions.size());
      int id_partition = 0;

      std::vector<uint> waypoints;
      logistic_sim::Token temp_token;
      temp_token.MISSIONS_COMPLETED = std::vector<uint>(TEAM_SIZE, 0);
      temp_token.TASKS_COMPLETED = std::vector<uint>(TEAM_SIZE, 0);
      while (true)
      {
        std::vector<std::vector<logistic_sim::Mission>> partitions = *it[missions];
        auto n_subsets = it.subsets();
        // if (n_subsets <= TEAM_SIZE)
        if (n_subsets == current_team_size)
        {
          // std::cout << "partition for " << n_subsets << " robots" << std::endl;
          // add empty missions for the inactive robots
          if (n_subsets < TEAM_SIZE)
          {
            partitions.insert(partitions.end(), TEAM_SIZE - n_subsets, std::vector<logistic_sim::Mission>());
          }
          perm_iterator<std::vector<logistic_sim::Mission>> jt(partitions);
          try
          {
            while (true)
            {
              std::vector<std::vector<logistic_sim::Mission>> permutation = *jt;

              // try calculating paths for the current permutation
              temp_token.TRAILS = token.TRAILS;
              temp_token.HOME_TRAILS = token.HOME_TRAILS;
              robot_paths = temp_token.TRAILS;
              for (int i = 0; i < TEAM_SIZE; i++)
              {
                robot_paths[i].PATH.insert(robot_paths[i].PATH.end(), temp_token.HOME_TRAILS[i].PATH.begin(),
                                           temp_token.HOME_TRAILS[i].PATH.end());
              }

              for (int i = 0; i < TEAM_SIZE; i++)
              {
                temp_token.MISSIONS_COMPLETED[i % TEAM_SIZE] = token.MISSIONS_COMPLETED[i % TEAM_SIZE];
                temp_token.TASKS_COMPLETED[i % TEAM_SIZE] = token.TASKS_COMPLETED[i % TEAM_SIZE];
                if (!permutation[i].empty())
                {
                  // std::cout << "Robot " << i << " WAYPOINTS\n";
                  waypoints.clear();
                  waypoints.push_back(init_vertex[i % TEAM_SIZE]);
                  for (logistic_sim::Mission &m : permutation[i])
                  {
                    temp_token.MISSIONS_COMPLETED[i % TEAM_SIZE]++;
                    for (uint d : m.DEMANDS)
                    {
                      temp_token.TASKS_COMPLETED[i % TEAM_SIZE]++;
                    }
                    waypoints.push_back(src_vertex);
                    for (uint v : m.DSTS)
                    {
                      waypoints.push_back(v);
                    }
                  }
                  waypoints.push_back(home_vertex[i % TEAM_SIZE]);

                  for (auto &w : waypoints)
                  {
                    // std::cout << w << " ";
                  }
                  // std::cout << "\n" << std::endl;
                  try
                  {
                    std::vector<unsigned int> last_leg, first_leg;
                    auto f = boost::bind(astar_cmp_function, min_hops_matrix, waypoints, _1, _2);
                    std::vector<uint> path =
                        spacetime_dijkstra(robot_paths, map_graph, waypoints, i % TEAM_SIZE,
                                           temp_token.TRAILS[i % TEAM_SIZE].PATH.size() - 1, &last_leg, &first_leg, &f);

                    temp_token.TRAILS[i % TEAM_SIZE].PATH.pop_back();
                    temp_token.TRAILS[i % TEAM_SIZE].PATH.insert(temp_token.TRAILS[i % TEAM_SIZE].PATH.end(),
                                                                 first_leg.begin(), first_leg.end());
                    temp_token.HOME_TRAILS[i % TEAM_SIZE].PATH = last_leg;

                    robot_paths[i % TEAM_SIZE].PATH = temp_token.TRAILS[i % TEAM_SIZE].PATH;
                    robot_paths[i % TEAM_SIZE].PATH.insert(robot_paths[i % TEAM_SIZE].PATH.end(),
                                                           temp_token.HOME_TRAILS[i % TEAM_SIZE].PATH.begin(),
                                                           temp_token.HOME_TRAILS[i % TEAM_SIZE].PATH.end());
                  }
                  catch (std::string &e)
                  {
                    // empty path means planning has failed
                    robot_paths[i % TEAM_SIZE].PATH.clear();
                  }
                }
              }
              // std::cout << std::endl;

              // check for empty path
              int empty_paths = 0;
              for (int i = 0; i < TEAM_SIZE; i++)
              {
                if (robot_paths[i].PATH.empty())
                {
                  empty_paths++;
                }
              }

              if (empty_paths == 0)
              {
                std::cout << "Found valid paths!" << std::endl;
                //   stats_file << TEAM_SIZE << "\n\n";
                for (int i = 0; i < TEAM_SIZE; i++)
                {
                  token.MISSIONS_COMPLETED = temp_token.MISSIONS_COMPLETED;
                  token.TASKS_COMPLETED = temp_token.TASKS_COMPLETED;
                  token.TRAILS = temp_token.TRAILS;
                  token.HOME_TRAILS = temp_token.HOME_TRAILS;
                  // stats_file << missions_stats[j].MISSIONS_COMPLETED[i] << "\n";
                  // stats_file << missions_stats[j].TASKS_COMPLETED[i] << "\n\n";
                  std::cout << "Robot " << i << " path\n";
                  for (uint v : robot_paths[i].PATH)
                  {
                    std::cout << v << " ";
                  }
                  std::cout << "\n" << std::endl;
                }
                stats_file.close();
                return robot_paths;
              }
              ++jt;
            }
          }
          catch (std::overflow_error &e)
          {
          }
        }
        ++it;
      }
    }
    catch (std::overflow_error &)
    {
    }
  }

  c_print("[ERROR]Can't find solution!!!", red, P);
  return std::vector<logistic_sim::Path>(TEAM_SIZE, logistic_sim::Path());
}

}  // namespace onlinegreedytaskplanner

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "task_planner");
  ros::NodeHandle nh_;
  onlinegreedytaskplanner::OnlineGreedyTaskPlanner OGTP(nh_);
  OGTP.init(argc, argv);
  c_print("initialization completed!", green);
  ros::AsyncSpinner spinner(2);
  spinner.start();
  OGTP.run();
  return 0;
}