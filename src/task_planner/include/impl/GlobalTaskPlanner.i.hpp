#pragma once
#include "permutations.hpp"

namespace globaltaskplanner
{
GlobalTaskPlanner::GlobalTaskPlanner(ros::NodeHandle &nh_) : SP_TaskPlanner(nh_, "GlobalTaskPlanner")
{
}

std::vector<logistic_sim::Path> GlobalTaskPlanner::path_partition(logistic_sim::Token &token)
{
  ofstream stats_file("stats_file.txt");
  c_print("Calculating tasks distribution", green, P);
  std::vector<logistic_sim::Path> other_paths(TEAM_SIZE, logistic_sim::Path());
  std::vector<std::vector<logistic_sim::Path>> best_paths(TEAM_SIZE);
  std::vector<uint> best_paths_length(TEAM_SIZE, std::numeric_limits<uint>::max());
  std::vector<bool> possible_paths_found(TEAM_SIZE, false);
  std::vector<logistic_sim::Token> missions_stats(TEAM_SIZE);

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
          // per i robot fermi
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
              // individuo quali robot staranno fermi
              std::vector<bool> still_robots(TEAM_SIZE, false);
              for (int i = 0; i < TEAM_SIZE; i++)
              {
                if (permutation[i].empty())
                {
                  other_paths[i].PATH.push_back(init_vertex[i]);
                  still_robots[i] = true;
                }
              }
              // assegno la permutazione corrente ai robot e provo a calcolare i percorsi
              for (int i = 0; i < TEAM_SIZE; i++)
              {
                temp_token.MISSIONS_COMPLETED[i % TEAM_SIZE] = 0;
                temp_token.TASKS_COMPLETED[i % TEAM_SIZE] = 0;
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
                  if (mapname != "model6")
                  {
                    waypoints.push_back(init_vertex[i % TEAM_SIZE]);
                  }
                  else
                  {
                    waypoints.push_back(9);
                  }

                  for (auto &w : waypoints)
                  {
                    // std::cout << w << " ";
                  }
                  // std::cout << "\n" << std::endl;
                  try
                  {
                    std::vector<uint> path = token_dijkstra(waypoints, other_paths, i % TEAM_SIZE, still_robots);
                    if (mapname != "model6")
                    {
                      // path.insert(path.end(), 50, init_vertex[i % TEAM_SIZE]);
                    }
                    other_paths[i % TEAM_SIZE].PATH = path;
                  }
                  catch (std::string &e)
                  {
                    // std::cout << "can't find path, trying another one..." << std::endl;
                    other_paths[i % TEAM_SIZE].PATH.clear();
                  }
                }
              }
              // std::cout << std::endl;

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
                      // std::cout << i << " ";
                      other_paths[indices[j]].PATH.push_back(i);
                    }
                    // other_paths[indices[j]].PATH.insert(other_paths[indices[j]].PATH.end(), 50, home_vertex);
                    // std::cout << std::endl;
                  }
                }

                // controllo se i percorsi che ho trovato sono
                // più corti dei migliori trovati finora
                uint max_length = other_paths[0].PATH.size();
                for (int i = 1; i < TEAM_SIZE; i++)
                {
                  if (other_paths[i].PATH.size() > max_length)
                  {
                    max_length = other_paths[i].PATH.size();
                  }
                }

                // riempo percorso per i robot fermi
                for (int i = 0; i < TEAM_SIZE; i++)
                {
                  if (permutation[i].empty())
                  {
                    other_paths[i].PATH = std::vector<uint>(max_length, init_vertex[i]);
                  }
                }

                if (max_length < best_paths_length[n_subsets - 1])
                {
                  c_print("new best with ", n_subsets, " robots: ", max_length, green, P);
                  best_paths_length[n_subsets - 1] = max_length;
                  best_paths[n_subsets - 1] = other_paths;
                  possible_paths_found[n_subsets - 1] = true;
                  missions_stats[n_subsets - 1] = temp_token;
                }
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

    if (possible_paths_found[current_team_size - 1])
    {
      break;
    }
  }

  // se sono riuscito a trovare almeno un insieme
  // di percorsi validi lo restituisco
  for (int j = TEAM_SIZE - 1; j >= 0; j--)
  {
    if (possible_paths_found[j])
    {
      std::cout << "Trovato!" << std::endl;
      stats_file << TEAM_SIZE << "\n\n";
      for (int i = 0; i < TEAM_SIZE; i++)
      {
        token.MISSIONS_COMPLETED.push_back(missions_stats[j].MISSIONS_COMPLETED[i]);
        token.TASKS_COMPLETED.push_back(missions_stats[j].TASKS_COMPLETED[i]);
        stats_file << missions_stats[j].MISSIONS_COMPLETED[i] << "\n";
        stats_file << missions_stats[j].TASKS_COMPLETED[i] << "\n\n";
        std::cout << "Percorso robot " << i << "\n";
        for (uint v : best_paths[j][i].PATH)
        {
          std::cout << v << " ";
        }
        std::cout << "\n" << std::endl;
      }
      stats_file.close();
      return best_paths[j];
    }
  }

  c_print("[ERROR]Can't find solution!!!", red, P);
  return std::vector<logistic_sim::Path>(TEAM_SIZE, logistic_sim::Path());
}

}  // namespace nonuniformtaskplanner