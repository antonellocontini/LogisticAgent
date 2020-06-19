#pragma once

#include <ros/console.h>
#include <chrono>
#include <functional>
#include <limits>
#include <list>
#include <map>
#include <set>
#include <vector>

namespace mapf
{
struct state
{
  std::vector<uint> configuration;

  // strict weak ordering for use in map and set structures
  bool operator<(const state &other) const
  {
    for (int i = 0; i < configuration.size(); i++)
    {
      if (configuration[i] < other.configuration[i])
      {
        return true;
      }
      else if (configuration[i] > other.configuration[i])
      {
        return false;
      }
    }
    return false;
  }

  bool operator==(const state &other) const
  {
    return configuration == other.configuration;
  }
};

unsigned int max = std::numeric_limits<unsigned int>::max();
unsigned int GRAY = 1;
unsigned int BLACK = 2;

struct search_tree
{
  std::set<state, std::function<bool(const state &, const state &)>> open;
  std::map<uint, std::vector<uint>> other_paths;
  std::map<state, state> prev;
  std::map<state, uint> g_value, f_value;
  std::set<state> visited;
  std::map<state, unsigned int> dcop_visited;
  std::vector<std::vector<unsigned int>> graph, fw;
  std::vector<uint> final_configuration;
  state initial_state;

  search_tree(const std::vector<std::vector<unsigned int>> &graph, const std::vector<unsigned int> &final_configuration,
              const state &initial_state, const std::map<uint, std::vector<uint>> &other_paths)
    : other_paths(other_paths)
    , prev()
    , g_value()
    , f_value()
    , visited()
    , dcop_visited()
    , graph(graph)
    , fw(std::vector<std::vector<unsigned int>>(graph.size(), std::vector<unsigned int>(graph.size(), max)))
    , final_configuration(final_configuration)
    , initial_state(initial_state)
  {
    // define compare function for open set
    open = std::set<state, std::function<bool(const state &, const state &)>>{ [&](const state &lhs, const state &rhs) {
      uint lhs_f = f_value.find(lhs)->second;
      uint rhs_f = f_value.find(rhs)->second;

      uint lhs_g = g_value.find(lhs)->second;
      uint rhs_g = g_value.find(rhs)->second;

      if (lhs_f < rhs_f)
      {
        return true;
      }
      else if (lhs_f == rhs_f)
      {
        if (lhs_g > rhs_g)
        {
          return true;
        }
        else if (lhs_g < rhs_g)
        {
          return false;
        }
        else
        {
          for (int i = 0; i < lhs.configuration.size(); i++)
          {
            if (lhs.configuration[i] < rhs.configuration[i])
            {
              return true;
            }
            else if (lhs.configuration[i] > rhs.configuration[i])
            {
              return false;
            }
          }
        }
      }

      return false;
    } };

    // init fw matrix
    for (int i = 0; i < graph.size(); i++)
    {
      fw[i][i] = 0;
      for (uint v : graph[i])
      {
        fw[i][v] = 1;
      }
    }

    for (int k = 0; k < graph.size(); k++)
    {
      for (int i = 0; i < graph.size(); i++)
      {
        for (int j = 0; j < graph.size(); j++)
        {
          if (fw[i][k] != max && fw[k][j] != max && fw[i][j] > fw[i][k] + fw[k][j])
          {
            fw[i][j] = fw[i][k] + fw[k][j];
          }
        }
      }
    }
  }


  unsigned int heuristic_function(const state &s) const
  {
    uint result = 0;
    for (int i = 0; i < s.configuration.size(); i++)
    {
      if (other_paths.count(i) == 0)
      {
        uint robot_cost = 0;
        uint curr_pos = s.configuration[i];
        uint next_pos = final_configuration[i];

        robot_cost += fw[curr_pos][next_pos];
        if (robot_cost > result)
        {
          result = robot_cost;
        }
      }
    }

    return result;
  }

  void _near_state_impl(std::vector<state> &result, const state &s, state &temp_state, unsigned int robot_i) const
  {
    uint g_v = g_value.find(s)->second;
    unsigned int current_vertex = s.configuration[robot_i], robots_number = s.configuration.size();
    std::vector<unsigned int> near_vertices;
    auto path_it = other_paths.find(robot_i);
    if (path_it == other_paths.end())
    {
      near_vertices = { current_vertex };
      near_vertices.insert(near_vertices.end(), graph[current_vertex].begin(), graph[current_vertex].end());
    }
    else  // if robot_i has already a path its move is constrained
    {
      const std::vector<uint> &path = path_it->second;
      if (g_v + 1 < path.size())
      {
        near_vertices = { path[g_v + 1] };
      }
      else
      {
        near_vertices = { path.back() };
      }
    }

    for (unsigned int v : near_vertices)
    {
      temp_state.configuration[robot_i] = v;

      if (robot_i < robots_number - 1)
      {
        _near_state_impl(result, s, temp_state, robot_i + 1);
      }
      else
      {
        // check conflict state
        bool good = true;

        if (s == temp_state)
        {
          good = false;
        }

        for (int i = 0; i < robots_number; i++)
        {
          for (int j = i + 1; j < robots_number; j++)
          {
            if (temp_state.configuration[i] == s.configuration[j] && temp_state.configuration[j] == s.configuration[i])
            {
              good = false;
            }

            if (temp_state.configuration[i] == temp_state.configuration[j])
            {
              good = false;
            }
          }
        }

        if (good)
        {
          result.push_back(temp_state);
        }
      }
    }
  }

  std::vector<state> near_states(const state &s) const
  {
    std::vector<state> result;
    state temp_state(s);
    _near_state_impl(result, s, temp_state, 0);
    return result;
  }

  int get_dcop_near_state(state &best_state, const state &s) const;

  bool is_final_state(const state &s) const
  {
    for (int i=0; i<s.configuration.size(); i++)
    {
      if (other_paths.count(i) == 0 && s.configuration[i] != final_configuration[i])
      {
        return false;
      }
    }
    return true;
  }

  std::list<state> astar_search(std::vector<std::vector<unsigned int>> &paths, uint time_limit = 0, double* duration = nullptr)
  {
    unsigned int vertices = graph.size(), robots_number = final_configuration.size();

    uint max_timestep_reached = 0;
    open.clear();
    g_value[initial_state] = 0;
    f_value[initial_state] = heuristic_function(initial_state);
    open.insert(initial_state);

    uint count = 0;
    auto start = std::chrono::system_clock::now();
    std::chrono::duration<double> diff;
    while (!open.empty())
    {
      auto end = std::chrono::system_clock::now();
      diff = end - start;
      if (diff.count() > 10.0)
      {
        start = std::chrono::system_clock::now();
        ROS_DEBUG_STREAM("visited nodes: " << count);
        ROS_DEBUG_STREAM("queue size: " << open.size() << "\n");
      }
      count++;
      state s = *open.begin();
      open.erase(s);
      if (is_final_state(s))
      {
        ROS_DEBUG_STREAM("found solution!");
        std::list<state> result;

        while (true)
        {
          result.push_front(s);
          auto it = prev.find(s);
          if (it != prev.end())
          {
            s = it->second;
          }
          else
          {
            paths = std::vector<std::vector<unsigned int>>(s.configuration.size() - other_paths.size());
            for (const state &s : result)
            {
              std::stringstream ss;
              int count = 0;
              for (int i = 0; i < s.configuration.size(); i++)
              {
                if (other_paths.count(i) == 0)
                {
                  unsigned int x = s.configuration[i];
                  ss << std::setw(3) << x;
                  paths[count].push_back(x);
                  count++;
                }
              }

              ROS_DEBUG_STREAM(ss.str());
            }
            if (duration != nullptr)
            {
              *duration = diff.count();
            }
            return result;
          }
        }
      }

      visited.insert(s);

      uint s_g_value = g_value.find(s)->second;
      if (s_g_value > max_timestep_reached)
      {
        max_timestep_reached = s_g_value;
      }
      for (state n : near_states(s))
      {
        // std::stringstream ss;
        // for (uint x : n.configuration)
        // {
        //   ss << " " << x;
        // }
        // ROS_DEBUG_STREAM(ss.str());
        // std::cout << "neighbour:\n";
        if (visited.count(n) == 0)
        {
          uint new_f_value = s_g_value + 1 + heuristic_function(n);
          auto old_f_it = f_value.find(n);
          bool removed = false;
          if (old_f_it != f_value.end())
          {
            uint old_f_value = old_f_it->second;
            if (new_f_value < old_f_value)
            {
              open.erase(n);
              g_value.erase(n);
              f_value.erase(n);
              removed = true;
            }
          }
          // if i have removed the old value i must insert the new one
          if (old_f_it == f_value.end() || removed)
          {
            // std::cout << "\tinserted\n";
            g_value[n] = s_g_value + 1;
            f_value[n] = new_f_value;
            open.insert(n);
            prev[n] = s;
          }
        }
      }

      if (time_limit > 0 && diff.count() > time_limit)
      {
        ROS_WARN_STREAM("MAPF Time limit reached!");
        open.clear();
      }
    }

    if (duration != nullptr)
    {
      *duration = diff.count();
    }
    ROS_WARN_STREAM("fail!");
    ROS_DEBUG_STREAM("visited states: " << count);
    ROS_DEBUG_STREAM("max timestep reached: " << max_timestep_reached);
    return std::list<state>();
  }

  std::list<state> dcop_search(std::vector<std::vector<unsigned int>> &paths,
                               std::vector<std::vector<unsigned int>> &home_paths, bool home_included = true);
};

}  // namespace mapf
