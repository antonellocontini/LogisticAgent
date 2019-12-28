namespace onlinegreedytaskplanner
{
    // smaller elements goes in the back of the array
template <class T>
unsigned int OnlineGreedyTaskPlanner::insertion_sort(st_location *queue, unsigned int size, st_location loc, T *cmp_function)
{
  int i;
  for (i = size - 1; i >= 0; i--)
  {
    if (cmp_function != nullptr)
    {
      bool cmp_result = (*cmp_function)(loc, queue[i]);
      if (cmp_result)
        break;
    }
    else if (loc < queue[i])
    {
      break;
    }
    queue[i + 1] = queue[i];
  }
  queue[i + 1] = loc;
  // std::cout << std::endl;

  return size + 1;
}

template <class T>
std::vector<unsigned int> OnlineGreedyTaskPlanner::spacetime_dijkstra(const std::vector<logistic_sim::Path> &other_paths,
                                                          const std::vector<std::vector<unsigned int>> &graph,
                                                          const std::vector<unsigned int> &waypoints, int ID_ROBOT,
                                                          int start_time,
                                                          std::vector<unsigned int> *last_leg,
                                                          std::vector<unsigned int> *first_leg, T *cmp_function)
{
//   std::cout << "SPACETIME DIJKSTRA --- WAYPOINTS:";
//   for (int i = 0; i < waypoints.size(); i++)
//   {
//     std::cout << " " << waypoints[i];
//   }
//   std::cout << std::endl;

  unsigned int source = waypoints.front();
  auto it_waypoints = waypoints.begin() + 1;

  // inizializzazione strutture
  for (unsigned int i = 0; i < graph.size(); i++)
  {
    for (unsigned int j = 0; j < MAX_TIME; j++)
    {
      for (unsigned int k = 0; k < MAX_WAYPOINTS; k++)
      {
        path_sizes[i][j][k] = 0;
      }
    }
  }
  prev_paths[source][0][1][0] = source;
  path_sizes[source][0][1] = 1;

  for (unsigned int i = 0; i < graph.size(); i++)
  {
    for (unsigned int j = 0; j < MAX_TIME; j++)
    {
      for (unsigned int k = 0; k < MAX_WAYPOINTS; k++)
      {
        visited[i][j][k] = WHITE;
      }
    }
  }

  // inizializzazione coda
  for (unsigned int i = 0; i < MAX_TIME; i++)
  {
    queue[i] = st_location();
  }
  st_location st(source, start_time, 1);
  queue[0] = st;
  int queue_size = 1;

  // individuo la lunghezza del percorso maggiore
  uint max_path_size = 0;
  for (int i = 0; i < TEAM_SIZE; i++)
  {
    if (i != ID_ROBOT && other_paths[i].PATH.size() > max_path_size)
    {
      max_path_size = other_paths[i].PATH.size();
    }
  }

  while (queue_size > 0)
  {
    st_location current_st = queue[--queue_size];
    unsigned int u = current_st.vertex;
    unsigned int time = current_st.time - start_time;
    unsigned int current_waypoint = current_st.waypoint;
    unsigned int next_next_waypoint;

    visited[u][time][current_waypoint] = BLACK;

    if (u == waypoints[current_waypoint])
    {
      if (current_waypoint + 1 == waypoints.size())
      {
        if (time + 1 + start_time >= max_path_size)
        {
          std::vector<unsigned int> result =
              std::vector<unsigned int>(prev_paths[u][time][current_waypoint],
                                        prev_paths[u][time][current_waypoint] + path_sizes[u][time][current_waypoint]);

          // spezzo il percorso in due parti
          if (last_leg != nullptr && first_leg != nullptr)
          {
            last_leg->clear();
            first_leg->clear();
            bool last_part = true;
            for (auto it = result.rbegin(); it != result.rend(); it++)
            {
              if (*it == waypoints[current_waypoint - 1])
              {
                last_part = false;
              }

              if (last_part)
              {
                last_leg->insert(last_leg->begin(), *it);
              }
              else
              {
                first_leg->insert(first_leg->begin(), *it);
              }
            }
          }

          return result;
        }
        else
        {
          next_next_waypoint = current_waypoint;
        }
      }
      else
      {
        next_next_waypoint = current_waypoint + 1;
      }
    }
    else  // u non è waypoint
    {
      next_next_waypoint = current_waypoint;
    }

    // considero l'attesa sul posto
    unsigned int next_time = time + 1;
    if (next_time < MAX_TIME && visited[u][next_time][next_next_waypoint] == WHITE)
    {
      bool good = true;
      for (int i = 0; i < TEAM_SIZE; i++)
      {
        if (i != ID_ROBOT)
        {
          if (next_time + start_time < other_paths[i].PATH.size())
          {
            if (other_paths[i].PATH[next_time + start_time] == u)
            {
              // std::cout << "can't stand in " << u << " at time " << time << std::endl;
              good = false;
              break;
            }
          }
          else if (!other_paths[i].PATH.empty() /* && still_robots[i] */)
          {
            if (other_paths[i].PATH.back() == u)
            {
              // std::cout << "can't stand in " << u << " at time " << time << std::endl;
              good = false;
              break;
            }
          }
        }
      }

      if (good)
      {
        st_location next_st(u, next_time + start_time, next_next_waypoint);
        queue_size = insertion_sort(queue, queue_size, next_st, cmp_function);
        visited[u][next_time][next_next_waypoint] = GRAY;

        unsigned int psize = path_sizes[u][time][current_waypoint];
        for (unsigned int i = 0; i < psize; i++)
        {
          prev_paths[u][next_time][next_next_waypoint][i] = prev_paths[u][time][current_waypoint][i];
        }

        prev_paths[u][next_time][next_next_waypoint][psize] = u;
        path_sizes[u][next_time][next_next_waypoint] = path_sizes[u][time][current_waypoint] + 1;
      }
    }

    // considero i vicini
    for (auto it = graph[u].begin(); it != graph[u].end(); it++)
    {
      const unsigned int v = *it;
      const unsigned int next_time = time + 1;

      if (next_time < MAX_TIME && visited[v][next_time][next_next_waypoint] == WHITE)
      {
        bool good = true;
        for (int i = 0; i < TEAM_SIZE; i++)
        {
          if (i != ID_ROBOT)
          {
            if (next_time + start_time < other_paths[i].PATH.size())
            {
              if (other_paths[i].PATH[next_time + start_time] == v)
              {
                // std::cout << "can't go in " << v << " at time " << time << std::endl;
                good = false;
                break;
              }
              if (other_paths[i].PATH[next_time + start_time] == u && other_paths[i].PATH[time + start_time] == v)
              {
                // std::cout << "can't go in " << v << " at time " << time << std::endl;
                good = false;
                break;
              }
            }
            else if (!other_paths[i].PATH.empty() /* && still_robots[i] */)
            {
              if (other_paths[i].PATH.back() == v)
              {
                // std::cout << "can't go in " << v << " at time " << time << ", there is a still robot" << std::endl;
                good = false;
                break;
              }
            }
          }
        }

        if (good)
        {
          st_location next_st(v, next_time + start_time, next_next_waypoint);
          queue_size = insertion_sort(queue, queue_size, next_st, cmp_function);
          visited[v][next_time][next_next_waypoint] = GRAY;

          unsigned int psize = path_sizes[u][time][current_waypoint];
          for (unsigned int i = 0; i < psize; i++)
          {
            prev_paths[v][next_time][next_next_waypoint][i] = prev_paths[u][time][current_waypoint][i];
          }

          prev_paths[v][next_time][next_next_waypoint][psize] = v;
          path_sizes[v][next_time][next_next_waypoint] = path_sizes[u][time][current_waypoint] + 1;
        }
      }
    }
  }

  throw std::string("Can't find path!!!");
}

}