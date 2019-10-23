#pragma once
#include "permutations.hpp"

namespace greedytaskplanner
{
GreedyTaskPlanner::GreedyTaskPlanner(ros::NodeHandle &nh_) : SP_TaskPlanner(nh_, "GreedyTaskPlanner")
{
}

std::vector<logistic_sim::Path> GreedyTaskPlanner::path_partition(logistic_sim::Token &token)
{
  ofstream stats_file("stats_file.txt");
  c_print("Calculating tasks distribution", green, P);
  c_print("Missions number: ", missions.size(), green, P);
  partition::iterator it(missions.size());
  int id_partition = 0;

  // individuo vertici iniziali degli agenti
  ros::NodeHandle nh;
  std::vector<double> list;
  std::vector<uint> init_vertex;
  nh.getParam("initial_pos", list);

  if (list.empty())
  {
    ROS_ERROR("No initial positions given: check \"initial_pos\" parameter.");
    ros::shutdown();
    exit(-1);
  }

  // dalle coordinate degli agenti individuo il nodo del grafo
  for (int value = 0; value < TEAM_SIZE; value++)
  {
    double initial_x = list[2 * value];
    double initial_y = list[2 * value + 1];
    uint v = IdentifyVertex(vertex_web, dimension, initial_x, initial_y);
    init_vertex.push_back(v);
    // std::cout << "Robot " << value << "\tVertice iniziale: " << v << std::endl;
  }

  std::vector<uint> waypoints;
  std::vector<logistic_sim::Path> other_paths(TEAM_SIZE, logistic_sim::Path());
  for (int current_team_size = TEAM_SIZE; current_team_size >= 1; current_team_size--)
  {
    std::cout << "ricerca percorso con " << current_team_size << " robot" << std::endl;
    try
    {
      while (true)
      {
        std::vector<std::vector<logistic_sim::Mission>> partitions = *it[missions];
        auto n_subsets = it.subsets();
        if (n_subsets == current_team_size)
        {
          std::cout << "partizione da " << n_subsets << " robot" << std::endl;
          // metto missioni vuote per i robot che stanno fermi
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
              for (int j = 0; j < TEAM_SIZE; j++)
              {
                other_paths[j].PATH.clear();
              }
              token.MISSIONS_COMPLETED = std::vector<uint>(TEAM_SIZE, 0);
              token.TASKS_COMPLETED = std::vector<uint>(TEAM_SIZE, 0);
              // individuo i robot che stanno fermi
              std::vector<bool> still_robots(TEAM_SIZE, false);
              for (int i = 0; i < TEAM_SIZE; i++)
              {
                if (permutation[i].empty())
                {
                  other_paths[i].PATH.push_back(init_vertex[i]);
                  still_robots[i] = true;
                }
              }
              for (int i = 0; i < TEAM_SIZE; i++)
              {
                // se il robot ha dei task li pianifica
                if (!still_robots[i])
                {
                  waypoints.clear();
                  // std::cout << "Robot " << i << "\n";
                  waypoints.push_back(init_vertex[i % TEAM_SIZE]);
                  for (logistic_sim::Mission &m : permutation[i])
                  {
                    token.MISSIONS_COMPLETED[i % TEAM_SIZE]++;
                    for (uint d : m.DEMANDS)
                    {
                      token.TASKS_COMPLETED[i % TEAM_SIZE]++;
                    }
                    // std::cout << src_vertex << " ";
                    waypoints.push_back(src_vertex);
                    for (uint v : m.DSTS)
                    {
                      // std::cout << v << " ";
                      waypoints.push_back(v);
                    }
                  }
                  if (mapname != "model6")
                  {
                    std::cout << init_vertex[i % TEAM_SIZE] << " ";
                    waypoints.push_back(init_vertex[i % TEAM_SIZE]);
                  }
                  else
                  {
                    // std::cout << 9 << " ";
                    waypoints.push_back(9);
                  }
                  std::cout << "\n";
                  try
                  {
                    std::vector<uint> path = token_dijkstra(waypoints, other_paths, i % TEAM_SIZE);
                    if (mapname != "model6")
                    {
                      // path.insert(path.end(), 50, init_vertex[i % TEAM_SIZE]);
                    }
                    other_paths[i % TEAM_SIZE].PATH = path;
                  }
                  catch (std::string &e)
                  {
                    other_paths[i % TEAM_SIZE].PATH.clear();
                  }
                }
              }
              std::cout << std::endl;

              // check for empty path
              int empty_paths = 0;
              for (int i = 0; i < TEAM_SIZE; i++)
              {
                if (other_paths[i].PATH.empty())
                {
                  empty_paths++;
                }
              }
              if (empty_paths == 0)
              {
                // percorso corridoio per model6
                if (mapname == "model6")
                {
                  std::vector<uint> indices(TEAM_SIZE);
                  // std::cout << "Dimensione percorsi\n";
                  for (int i = 0; i < TEAM_SIZE; i++)
                  {
                    // std::cout << "robot " << i << ": " << other_paths[i].PATH.size() << std::endl;
                    indices[i] = i;
                  }
                  auto cmp_function = [&](uint lhs, uint rhs) {
                    return other_paths[lhs].PATH.size() < other_paths[rhs].PATH.size();
                  };
                  std::sort(indices.begin(), indices.end(), cmp_function);
                  for (int j = 0; j < indices.size(); j++)
                  {
                    // j indica il nodo home
                    // indices[j] indica il robot assegnato a quel nodo
                    // std::cout << "Casa robot " << indices[j] << "\n";
                    int home_vertex = j;
                    for (int i = 5; i >= home_vertex; i--)
                    {
                      std::cout << i << " ";
                      other_paths[indices[j]].PATH.push_back(i);
                    }
                    // other_paths[indices[j]].PATH.insert(other_paths[indices[j]].PATH.end(), 50, home_vertex);
                    // std::cout << std::endl;
                  }
                }

                int max_path = 0;
                for (int i = 0; i < TEAM_SIZE; i++)
                {
                  if (!still_robots[i] && other_paths[i].PATH.size() > max_path)
                  {
                    max_path = other_paths[i].PATH.size();
                  }
                }
                // riempo i percorsi dei robot fermi con il vertice iniziale
                for (int i = 0; i < TEAM_SIZE; i++)
                {
                  if (still_robots[i])
                  {
                    other_paths[i].PATH.insert(other_paths[i].PATH.end(), max_path - 1, other_paths[i].PATH.front());
                  }
                }
                std::cout << "Trovato!" << std::endl;
                stats_file << TEAM_SIZE << "\n\n";
                for (int i = 0; i < TEAM_SIZE; i++)
                {
                  stats_file << token.MISSIONS_COMPLETED[i] << "\n";
                  stats_file << token.TASKS_COMPLETED[i] << "\n\n";
                  std::cout << "Percorso robot " << i << "\n";
                  for (uint v : other_paths[i].PATH)
                  {
                    std::cout << v << " ";
                  }
                  std::cout << "\n" << std::endl;
                }
                stats_file.close();
                return other_paths;
              }
              ++jt;
            }
          }
          catch (std::overflow_error &e)
          {
            std::cout << "fine permutazioni per partizioni corrente" << std::endl;
          }
        }
        ++it;
      }
    }
    catch (std::overflow_error &)
    {
      std::cout << "fine partizioni" << std::endl;
    }
  }

  c_print("[ERROR]Can't find solution!!!", red, P);
  return std::vector<logistic_sim::Path>(TEAM_SIZE, logistic_sim::Path());
}

}  // namespace nonuniformtaskplanner