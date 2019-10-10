#pragma once

namespace sp_taskplanner
{
SP_TaskPlanner::SP_TaskPlanner(ros::NodeHandle &nh_) : TaskPlanner(nh_, "SP_TaskPlanner")
{
}

uint SP_TaskPlanner::compute_cycle_dst(logistic_sim::Mission &mission)
{
  uint res = 0;
  sort(mission.DSTS.begin(), mission.DSTS.end());
  mission.DSTS.erase(unique(mission.DSTS.begin(), mission.DSTS.end()), mission.DSTS.end());
  if (mission.DSTS.size() == 1)
  {
    if (mission.DSTS[0] == dst_vertex[0])
      res = 1;
    else if (mission.DSTS[0] == dst_vertex[1])
      res = 2;
    else
      res = 3;
  }
  else if (mission.DSTS.size() == 2)
  {
    if ((mission.DSTS[0] == dst_vertex[0]) && (mission.DSTS[1] == dst_vertex[1]))
    {
      res = 4;
    }
    else if ((mission.DSTS[0] == dst_vertex[0]) && (mission.DSTS[1] == dst_vertex[2]))
    {
      res = 5;
    }
    else
    {
      res = 6;
    }
  }
  else
  {
    res = 7;
  }

  return res;
}

void SP_TaskPlanner::compute_route(uint id, logistic_sim::Mission &m)
{
  switch (id)
  {
    case 1:
    {
      std::copy(std::begin(p_11), std::end(p_11), back_inserter(m.ROUTE));
      m.PATH_DISTANCE = compute_cost_of_route(m.ROUTE);
      m.V = (double)m.PATH_DISTANCE / (double)m.TOT_DEMAND;
    }
    break;
    case 2:
    {
      std::copy(std::begin(p_16), std::end(p_16), back_inserter(m.ROUTE));
      m.PATH_DISTANCE = compute_cost_of_route(m.ROUTE);
      m.V = (double)m.PATH_DISTANCE / (double)m.TOT_DEMAND;
    }
    break;
    case 3:
    {
      std::copy(std::begin(p_21), std::end(p_21), back_inserter(m.ROUTE));
      m.PATH_DISTANCE = compute_cost_of_route(m.ROUTE);
      m.V = (double)m.PATH_DISTANCE / (double)m.TOT_DEMAND;
    }
    break;
    case 4:
    {
      std::copy(std::begin(p_11_16), std::end(p_11_16), back_inserter(m.ROUTE));
      m.PATH_DISTANCE = compute_cost_of_route(m.ROUTE);
      m.V = (double)m.PATH_DISTANCE / (double)m.TOT_DEMAND;
    }
    break;
    case 5:
    {
      std::copy(std::begin(p_11_21), std::end(p_11_21), back_inserter(m.ROUTE));
      m.PATH_DISTANCE = compute_cost_of_route(m.ROUTE);
      m.V = (double)m.PATH_DISTANCE / (double)m.TOT_DEMAND;
    }
    break;
    case 6:
    {
      std::copy(std::begin(p_16_21), std::end(p_16_21), back_inserter(m.ROUTE));
      m.PATH_DISTANCE = compute_cost_of_route(m.ROUTE);
      m.V = (double)m.PATH_DISTANCE / (double)m.TOT_DEMAND;
    }
    break;
    case 7:
    {
      std::copy(std::begin(p_11_16_21), std::end(p_11_16_21), back_inserter(m.ROUTE));
      m.PATH_DISTANCE = compute_cost_of_route(m.ROUTE);
      m.V = (double)m.PATH_DISTANCE / (double)m.TOT_DEMAND;
    }
    break;
    default:
    {
      c_print("ERR", red);
    }
    break;
  }
}

void SP_TaskPlanner::set_partition()
{
  std::vector<t_coalition> good_partition;
  try
  {
    c_print(nTask);
    partition::iterator it(nTask);
    int id_partition = 0;

    t_coalition candidate;
    while (true)
    {
      std::vector<std::vector<logistic_sim::Mission>> partitions = *it[missions];
      auto n_subsets = it.subsets();
      logistic_sim::Mission candidate_partition;
      candidate_partition.ID = id_partition;
      // c_print(id_partition);
      id_partition++;
      uint tmp_TD = 0;
      int id_subset = 0;
      double V = 0;
      candidate.second = candidate_partition;
      std::vector<logistic_sim::Mission> m;
      for (int i = 0; i < n_subsets; i++)
      {
        uint tmp_D = 0;
        std::vector<logistic_sim::Mission> subset = partitions[i];
        logistic_sim::Mission candidate_subset;
        candidate_subset.ID = id_subset;
        id_subset++;
        for (int j = 0; j < subset.size(); j++)
        {
          candidate_subset.TOT_DEMAND += subset[j].TOT_DEMAND;
          copy(subset[j].DEMANDS.begin(), subset[j].DEMANDS.end(), back_inserter(candidate_subset.DEMANDS));
          copy(subset[j].DSTS.begin(), subset[j].DSTS.end(), back_inserter(candidate_subset.DSTS));
          copy(subset[j].ITEM.begin(), subset[j].ITEM.end(), back_inserter(candidate_subset.ITEM));
        }

        uint id_path = compute_cycle_dst(candidate_subset);

        compute_route(id_path, candidate_subset);

        candidate.second.V += candidate_subset.V;

        if (candidate_subset.TOT_DEMAND > TEAM_CAPACITY)
        {
          candidate.second.GOOD++;
        }
        // uso PICKUP == true se non è buona
        // prima soglio e controllo che la partizione sia composta da sottoinsiemi processabili dai robot poi calcolo il
        // percorso e metto in ordine per V poi prendo la prima buona
        m.push_back(candidate_subset);
      }

      ++it;
      candidate.first = m;
      if (candidate.second.GOOD == 0)
      {
        // c_print("ok", green);
        // print_coalition(*it);
        good_partition.push_back(candidate);
      }
    }
  }
  catch (std::overflow_error &)
  {
  }

  std::sort(good_partition.begin(), good_partition.end(), less_V());

  auto ele = good_partition.front();

  print_coalition(ele);

  missions = ele.first;
}

std::vector<logistic_sim::Path> SP_TaskPlanner::path_partition()
{
  try
  {
    c_print(missions.size());
    partition::iterator it(missions.size());
    int id_partition = 0;

    while (true)
    {
      std::vector<std::vector<logistic_sim::Mission>> partitions = *it[missions];
      std::vector<logistic_sim::Path> other_paths(TEAM_SIZE, logistic_sim::Path());
      auto n_subsets = it.subsets();
      if (n_subsets >= TEAM_SIZE)
      {
        for (int i = 0; i < n_subsets; i++)
        {
          std::vector<uint> waypoints;
          for (logistic_sim::Mission &m : partitions[i])
          {
            waypoints.push_back(src_vertex);
            for (uint v : m.DSTS)
            {
              waypoints.push_back(v);
            }
          }
          try
          {
            std::vector<uint> path = token_dijkstra(waypoints, other_paths, i % TEAM_SIZE);
            other_paths[i % TEAM_SIZE].PATH = path;
          }
          catch (std::string &e)
          {
          }
        }

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
          return other_paths;
        }
      }
      ++it;
    }
  }
  catch (std::overflow_error &)
  {
  }

  c_print("[ERROR]Can't find solution!!!", red, P);
  return std::vector<logistic_sim::Path>(TEAM_SIZE, logistic_sim::Path());
}

using graph_type = std::vector<std::vector<unsigned int>>;
std::vector<unsigned int> SP_TaskPlanner::spacetime_dijkstra(const std::vector<std::vector<unsigned int>> &other_paths,
                                                             const graph_type &graph, unsigned int size,
                                                             const std::vector<unsigned int> &waypoints, uint ID_ROBOT)
{
  std::cout << "WAYPOINTS:";
  for (int i = 0; i < waypoints.size(); i++)
  {
    std::cout << " " << waypoints[i];
  }
  std::cout << std::endl;

  const unsigned int WHITE = 0, GRAY = 1, BLACK = 2, MAX_TIME = 512U;
  unsigned int source = waypoints.front();
  auto it_waypoints = waypoints.begin() + 1;

  std::vector<unsigned int> path;

  std::vector<std::vector<std::vector<uint> > > prev_paths(size, std::vector<std::vector<uint>>(MAX_TIME, std::vector<uint>(size, 0)));
  std::vector<std::vector<uint> > path_sizes(size, std::vector<uint>(MAX_TIME, 0));
  // unsigned int ***prev_paths = new unsigned int **[size];
  // unsigned int **path_sizes = new unsigned int *[size];
  // for (unsigned int i = 0; i < size; i++)
  // {
  //   prev_paths[i] = new unsigned int *[MAX_TIME];
  //   path_sizes[i] = new unsigned int[MAX_TIME];
  //   for (unsigned int j = 0; j < MAX_TIME; j++)
  //   {
  //     prev_paths[i][j] = new unsigned int[MAX_TIME];
  //     path_sizes[i][j] = 0;
  //   }
  // }
  prev_paths[source][0][0] = source;
  path_sizes[source][0] = 1;

  std::vector<std::vector<uint> > visited(size, std::vector<uint>(MAX_TIME, WHITE));
  // unsigned int **visited = new unsigned int *[size];
  // for (unsigned int i = 0; i < size; i++)
  // {
  //   visited[i] = new unsigned int[MAX_TIME];
  //   for (unsigned int j = 0; j < MAX_TIME; j++)
  //   {
  //     visited[i][j] = WHITE;
  //   }
  // }

  st_location st(source, 0);
  std::vector<st_location> queue(MAX_TIME, st_location(0, 0));
  // st_location *queue = new st_location[MAX_TIME];
  queue[0] = st;
  int queue_size = 1;

  while (queue_size > 0)
  {
    st_location current_st = queue[--queue_size];
    unsigned int u = current_st.vertex;
    unsigned int time = current_st.time;

    visited[u][time] = BLACK;

    if (u == *it_waypoints)
    {
      if (it_waypoints + 1 == waypoints.end())
      {
        std::vector<unsigned int> result =
            // std::vector<unsigned int>(prev_paths[u][time], prev_paths[u][time] + path_sizes[u][time]);
            std::vector<unsigned int>(prev_paths[u][time].begin(), prev_paths[u][time].begin() + path_sizes[u][time]);

        // pulizia heap
        // for (unsigned int i = 0; i < size; i++)
        // {
        //   for (unsigned int j = 0; j < MAX_TIME; j++)
        //   {
        //     delete[] prev_paths[i][j];
        //   }

        //   delete[] prev_paths[i];
        //   delete[] path_sizes[i];
        //   delete[] visited[i];
        // }
        // delete[] prev_paths;
        // delete[] path_sizes;
        // delete[] visited;
        // delete[] queue;

        return result;
      }
      else
      {
        for (int i = 0; i < queue_size; i++)
        {
          st_location temp_st = queue[i];
          unsigned int temp_v = temp_st.vertex;
          unsigned int temp_t = temp_st.time;
          visited[temp_v][temp_t] = WHITE;
        }
        queue_size = 0;
      }

      it_waypoints++;
    }

    // considero i vicini
    for (auto it = graph[u].begin(); it != graph[u].end(); it++)
    {
      const unsigned int v = *it;
      const unsigned int next_time = time + 1;

      if (visited[v][next_time] == WHITE)
      {
        bool good = true;
        for (int i = 0; i < TEAM_SIZE; i++)
        {
          if (i != ID_ROBOT)
          {
            if (next_time < other_paths[i].size())
            {
              if (other_paths[i][next_time] == v)
              {
                good = false;
                break;
              }
              if (other_paths[i][next_time] == u && other_paths[i][time] == v)
              {
                good = false;
                break;
              }
            }
            // else if (!other_paths[i].empty())
            // {
            // 	if (other_paths[i].back() == v)
            // 	{
            // 		good = false;
            // 		break;
            // 	}
            // }
          }
        }

        if (good)
        {
          st_location next_st(v, next_time);
          queue_size = insertion_sort(queue, queue_size, next_st);
          visited[v][next_time] = GRAY;

          unsigned int psize = path_sizes[u][time];
          for (unsigned int i = 0; i < psize; i++)
          {
            prev_paths[v][next_time][i] = prev_paths[u][time][i];
          }
          prev_paths[v][next_time][psize] = v;
          path_sizes[v][next_time] = path_sizes[u][time] + 1;
        }
      }
    }

    // considero l'attesa sul posto
    unsigned int next_time = time + 1;
    if (visited[u][next_time] == WHITE)
    {
      bool good = true;
      for (int i = 0; i < TEAM_SIZE; i++)
      {
        if (i != ID_ROBOT)
        {
          if (next_time < other_paths[i].size())
          {
            if (other_paths[i][next_time] == u)
            {
              good = false;
              break;
            }
          }
          // else if (!other_paths[i].empty())
          // {
          // 	if (other_paths[i].back() == u)
          // 	{
          // 		good = false;
          // 		break;
          // 	}
          // }
        }
      }

      if (good)
      {
        st_location next_st(u, next_time);
        queue_size = insertion_sort(queue, queue_size, next_st);
        visited[u][next_time] = GRAY;

        unsigned int psize = path_sizes[u][time];
        for (unsigned int i = 0; i < psize; i++)
        {
          prev_paths[u][next_time][i] = prev_paths[u][time][i];
        }
        prev_paths[u][next_time][psize] = u;
        path_sizes[u][next_time] = path_sizes[u][time] + 1;
      }
    }
  }

  throw std::string("Can't find path!!!");
}

std::vector<uint> SP_TaskPlanner::token_dijkstra(const std::vector<uint> &waypoints,
                                                 std::vector<logistic_sim::Path> &other_paths, uint ID_ROBOT)
{
  // adatto struttura dijkstra
  std::vector<std::vector<unsigned int>> graph(dimension);
  for (int i = 0; i < dimension; i++)
  {
    for (int j = 0; j < vertex_web[i].num_neigh; j++)
    {
      graph[i].push_back(vertex_web[i].id_neigh[j]);
    }
  }

  std::vector<std::vector<uint>> simple_paths(TEAM_SIZE);
  for (int i = 0; i < TEAM_SIZE; i++)
  {
    simple_paths[i] = other_paths[i].PATH;
  }
  c_print("Fine del space_dijkstra", green, P);
  return spacetime_dijkstra(simple_paths, graph, dimension, waypoints, ID_ROBOT);
}

unsigned int SP_TaskPlanner::insertion_sort(std::vector<st_location> &queue, unsigned int size, st_location loc)
{
  int i;
  for (i = size - 1; i >= 0; i--)
  {
    if (loc < queue[i])
      break;
    queue[i + 1] = queue[i];
  }
  queue[i + 1] = loc;

  return size + 1;
}

unsigned int SP_TaskPlanner::insertion_sort(st_location *queue, unsigned int size, st_location loc)
{
  int i;
  for (i = size - 1; i >= 0; i--)
  {
    if (loc < queue[i])
      break;
    queue[i + 1] = queue[i];
  }
  queue[i + 1] = loc;

  return size + 1;
}

}  // namespace nonuniformtaskplanner