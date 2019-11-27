#pragma once
#include "permutations.hpp"

namespace sp_taskplanner
{
SP_TaskPlanner::SP_TaskPlanner(ros::NodeHandle &nh_, const std::string &name) : TaskPlanner(nh_, name)
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
  c_print("Calculating partitions", green, P);
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

void SP_TaskPlanner::allocate_memory()
{
  std::cout << "allocating memory..." << std::endl;
  std::cout << dimension << " " << MAX_TIME << std::endl;
  prev_paths = new unsigned int ***[dimension];
  path_sizes = new unsigned int **[dimension];
  for (unsigned int i = 0; i < dimension; i++)
  {
    prev_paths[i] = new unsigned int **[MAX_TIME];
    path_sizes[i] = new unsigned int *[MAX_TIME];
    for (unsigned int j = 0; j < MAX_TIME; j++)
    {
      prev_paths[i][j] = new unsigned int*[MAX_WAYPOINTS];
      path_sizes[i][j] = new unsigned int [MAX_WAYPOINTS];
      for(unsigned int k = 0; k < MAX_WAYPOINTS; k++)
      {
        prev_paths[i][j][k] = new unsigned int[MAX_TIME];
      }
    }
  }

  visited = new unsigned int **[dimension];
  next_waypoint = new unsigned int*[dimension];
  for (unsigned int i = 0; i < dimension; i++)
  {
    visited[i] = new unsigned int*[MAX_TIME];
    for(unsigned int j=0; j<MAX_TIME; j++)
    {
      visited[i][j] = new unsigned int[MAX_WAYPOINTS];
    }
    next_waypoint[i] = new unsigned int[MAX_TIME];
  }

  queue = new st_location[MAX_WAYPOINTS * MAX_TIME * dimension];
}

using graph_type = std::vector<std::vector<unsigned int>>;
std::vector<unsigned int> SP_TaskPlanner::spacetime_dijkstra(const std::vector<logistic_sim::Path> &other_paths,
                                                             const graph_type &graph, unsigned int size,
                                                             std::vector<unsigned int> &waypoints, const std::vector<bool> &still_robots, uint ID_ROBOT)
{
  // std::cout << "WAYPOINTS:";
  // for (int i = 0; i < waypoints.size(); i++)
  // {
  //   std::cout << " " << waypoints[i];
  // }
  // std::cout << std::endl;

  const unsigned int WHITE = 0, GRAY = 1, BLACK = 2;
  unsigned int source = waypoints.front();
  auto it_waypoints = waypoints.begin() + 1;
  int k = 1;

  for (unsigned int i = 0; i < size; i++)
  {
    for (unsigned int j = 0; j < MAX_TIME; j++)
    {
      for (unsigned int k=0; k<MAX_WAYPOINTS; k++)
      {
        path_sizes[i][j][k] = 0;
      }
    }
  }
  // vertice, istante, waypoint successivo, percorso
  prev_paths[source][0][1][0] = source;
  path_sizes[source][0][1] = 1;

  for (unsigned int i = 0; i < size; i++)
  {
    for (unsigned int j = 0; j < MAX_TIME; j++)
    {
      for(unsigned int k=0; k < MAX_WAYPOINTS; k++)
      {
        visited[i][j][k] = WHITE;
      }
    }
  }

  for(unsigned int i=0; i<MAX_TIME; i++)
  {
    queue[i] = st_location();
  }
  st_location st(source, 0, 1);
  queue[0] = st;
  int queue_size = 1;

  uint max_path_size = 0;
  for(int i=0; i<TEAM_SIZE; i++)
  {
    if (i != ID_ROBOT && other_paths[i].PATH.size() > max_path_size)
    {
      max_path_size = other_paths[i].PATH.size();
    }
  }
  // std::cout << "Max path size: " << max_path_size << std::endl;
  // std::cout << "waypoints number: " << waypoints.size() << std::endl;

  while (queue_size > 0)
  {
    st_location current_st = queue[--queue_size];
    unsigned int u = current_st.vertex;
    unsigned int time = current_st.time;
    unsigned int current_waypoint = current_st.waypoint;
    unsigned int next_next_waypoint;

    // if (time == MAX_TIME - 1)
    // {
    //   std::cout << "Hitting MAX_TIME ceiling" << std::endl;
    // }

    // std::cout << "current vertex: " << u << std::endl;
    // std::cout << "current time: " << time << std::endl;

    visited[u][time][current_waypoint] = BLACK;

    if (u == waypoints[current_waypoint])
    // if (u == *it_waypoints)
    {
      // std::cout << "vertex " << u << " waypoint index: " << current_waypoint << std::endl;
      if (current_waypoint+1 == waypoints.size())
      {
        if (time + 1 >= max_path_size)
        // if (it_waypoints + 1 == waypoints.end() && time + 1 >= max_path_size)
        {
          // std::vector<unsigned int> result(prev_paths[u][time].begin(), prev_paths[u][time].begin() + time + 1);
          int size = path_sizes[u][time][current_waypoint];
          // std::cout << "path size: " << size << std::endl;
          std::vector<unsigned int> result(size);
          std::copy(prev_paths[u][time][current_waypoint], prev_paths[u][time][current_waypoint] + size, result.begin());
          return result;
        }
        else /* if (time + 1 < max_path_size) */
        {
          // next_next_waypoint--;
          // std::cout << "last waypoint!!!" << std::endl;
          next_next_waypoint = current_waypoint;
        }
      }
      else
      {
        // std::cout <<"prossimo waypoint!!! " << current_waypoint+1 <<std::endl;
        next_next_waypoint = current_waypoint + 1;
        // if (next_waypoint[u][time] + 1 < waypoints.size())
        // {
        //   for (int i = 0; i < queue_size; i++)
        //   {
        //     st_location temp_st = queue[i];
        //     unsigned int temp_v = temp_st.vertex;
        //     unsigned int temp_t = temp_st.time;
        //     visited[temp_v][temp_t] = WHITE;
        //   }
        //   queue_size = 0;
        // }

        // if (next_waypoint[u][time] + 1 == waypoints.size() && time + 1 < max_path_size)
        // {
        //   // std::cout << "aspetto gli altri robot" << std::endl;
        //   // std::cout << "location corrente: " << u << std::endl;
        //   // std::cout << "istante corrente: " << time <<std::endl;
        //   // waypoints.push_back(u);
        //   next_next_waypoint--;
        // }
      }
      // it_waypoints++;
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
          if (next_time < other_paths[i].PATH.size())
          {
            // se un altro robot vuole andare dove sono io non posso stare fermo
            if (other_paths[i].PATH[next_time] == u)
            {
              good = false;
              break;
            }
          }
          // se il percorso dell'altro è più corto suppongo che lui sia rimasto nell'ultimo vertice
          else if (!other_paths[i].PATH.empty() /* && still_robots[i] */)
          {
          	if (other_paths[i].PATH.back() == u)
          	{
          		good = false;
          		break;
          	}
          }
        }
      }

      if (good)
      {
        st_location next_st(u, next_time, next_next_waypoint);
        // std::cout << "aggiungo alla coda stato(" << u << "," << next_time  << ") con next_next_w" << next_next_waypoint << std::endl;
        // next_waypoint[u][next_time] = next_next_waypoint;
        queue_size = insertion_sort(queue, queue_size, next_st);
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
            if (next_time < other_paths[i].PATH.size())
            {
              if (other_paths[i].PATH[next_time] == v)
              {
                good = false;
                break;
              }
              if (other_paths[i].PATH[next_time] == u && other_paths[i].PATH[time] == v)
              {
                good = false;
                break;
              }
            }
            else if (!other_paths[i].PATH.empty() /* && still_robots[i] */)
            {
            	if (other_paths[i].PATH.back() == v)
            	{
            		good = false;
            		break;
            	}
            }
          }
        }

        if (good)
        {
          st_location next_st(v, next_time, next_next_waypoint);
          // std::cout << "aggiungo alla coda stato(" << v << "," << next_time  << ") con next_next_w" << next_next_waypoint << std::endl;
          // next_waypoint[u][next_time] = next_next_waypoint;
          queue_size = insertion_sort(queue, queue_size, next_st);
          visited[v][next_time][next_next_waypoint] = GRAY;

          unsigned int psize = path_sizes[u][time][current_waypoint];
          for (unsigned int i = 0; i < psize; i++)
          {
            prev_paths[v][next_time][next_next_waypoint][i] = prev_paths[u][time][current_waypoint][i];
          }
          prev_paths[v][next_time][next_next_waypoint][psize] = v;
          path_sizes[v][next_time][next_next_waypoint] = path_sizes[u][time][current_waypoint] + 1;
          // st_location next_st(v, next_time);
          // std::cout << "aggiungo alla coda stato(" << v << "," << next_time  << ") con next_next_w" << next_next_waypoint << std::endl;
          // next_waypoint[v][next_time] = next_next_waypoint;
          // queue_size = insertion_sort(next_waypoint, queue, queue_size, next_st);
          // visited[v][next_time] = GRAY;

          // unsigned int psize = path_sizes[u][time];
          // for (unsigned int i = 0; i < psize; i++)
          // {
          //   prev_paths[v][next_time][i] = prev_paths[u][time][i];
          // }
          // prev_paths[v][next_time][psize] = v;
          // path_sizes[v][next_time] = path_sizes[u][time] + 1;
        }
      }
    }
  }

  throw std::string("Can't find path!!!");
}

std::vector<uint> SP_TaskPlanner::token_dijkstra(std::vector<uint> &waypoints,
                                                 std::vector<logistic_sim::Path> &other_paths, uint ID_ROBOT, const std::vector<bool> &still_robots = std::vector<bool>())
{
  // adatto struttura dijkstra
  static bool initialize = true;
  static std::vector<std::vector<unsigned int>> graph(dimension);
  if(initialize)
  {
    for (int i = 0; i < dimension; i++)
    {
      for (int j = 0; j < vertex_web[i].num_neigh; j++)
      {
        graph[i].push_back(vertex_web[i].id_neigh[j]);
      }
    }
    initialize = false;
  }

  // std::vector<std::vector<uint>> simple_paths(TEAM_SIZE);
  // for (int i = 0; i < TEAM_SIZE; i++)
  // {
  //   simple_paths[i] = other_paths[i].PATH;
  // }
  // c_print("Fine del space_dijkstra", green, P);
  if (!still_robots.empty())
  {
    return spacetime_dijkstra(other_paths, graph, dimension, waypoints, still_robots, ID_ROBOT);
  }
  else
  {
    return spacetime_dijkstra(other_paths, graph, dimension, waypoints, std::vector<bool>(TEAM_SIZE, false), ID_ROBOT);
  }
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

unsigned int SP_TaskPlanner::insertion_sort(unsigned int **next_waypoint, st_location *queue, unsigned int size, st_location loc)
{
	auto cmp_function = [&](const st_location &lhs, const st_location &rhs)
	{
		if (lhs.time < rhs.time)
		{
			return true;
		}
		else if (lhs.time == rhs.time)
		{
			if (next_waypoint[lhs.vertex][lhs.time] > next_waypoint[rhs.vertex][rhs.time])
			{
				return true;
			}
		}
		return false;
	};

    int i;
    for(i=size-1; i>=0; i--)
    {
        // if (loc < queue[i])
		if (cmp_function(loc, queue[i]))
            break;
        queue[i+1] = queue[i];
    }
    queue[i+1] = loc;

    return size+1;
}

}  // namespace nonuniformtaskplanner