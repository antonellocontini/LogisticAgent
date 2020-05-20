#pragma once

#include <vector>
#include <list>
#include <map>
#include <unordered_map>
#include <set>
#include <unordered_set>
#include <limits>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <sstream>

namespace mapd_time
{

struct state
{
  std::vector<unsigned int> configuration, waypoint_indices;
  unsigned int time, f;

  bool operator<(const state &other) const
  {
    if (f < other.f)
    {
      return true;
    }
    else if (f > other.f)
    {
      return false;
    }
    else if (time > other.time)
    {
      return true;
    }
    else if (time < other.time)
    {
      return false;
    }
    else
    {
      for (int i=0; i<configuration.size(); i++)
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

      for (int i=0; i<waypoint_indices.size(); i++)
      {
        if (waypoint_indices[i] < other.waypoint_indices[i])
        {
          return true;
        }
        else if (waypoint_indices[i] > other.waypoint_indices[i])
        {
          return false;
        }
      }

      return false;
    }
  }

  bool operator==(const state &other) const
  {
    if (time != other.time || configuration != other.configuration || waypoint_indices != other.waypoint_indices)
    {
      return false;
    }
    return true;
  }
};

unsigned int max = std::numeric_limits<unsigned int>::max();

struct search_tree
{
  std::set<state> open;
  std::map<state, state> prev;
  std::set<state> visited;
  std::vector<std::vector<unsigned int> > graph, waypoints, fw;
  state initial_state;

  search_tree(const std::vector<std::vector<unsigned int> > &graph, const std::vector<std::vector<unsigned int> > &waypoints, const state &initial_state)
    : open(), prev(), visited(), graph(graph), waypoints(waypoints), fw(std::vector<std::vector<unsigned int> >(graph.size(), std::vector<unsigned int>(graph.size(), max))), initial_state(initial_state)
  {
    for (int i=0; i<graph.size(); i++)
    {
      fw[i][i] = 0;
      for (uint v : graph[i])
      {
        fw[i][v] = 1;
      }
    }

    for (int k=0; k<graph.size(); k++)
    {
      for (int i=0; i<graph.size(); i++)
      {
        for (int j=0; j<graph.size(); j++)
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
    for (int i=0; i<s.configuration.size(); i++)
    {
      uint robot_cost = 0;
      uint curr_pos = s.configuration[i];
      for (int j=s.waypoint_indices[i]; j<=waypoints[i].size(); j++)
      {
        uint next_pos;
        if (j < waypoints[i].size())
        {
          next_pos = waypoints[i][j];
        }
        else
        {
          next_pos = waypoints[i].back();
        }
        
        robot_cost += fw[curr_pos][next_pos];
        curr_pos = next_pos;
      }

      if (robot_cost > result)
      {
        result = robot_cost;
      }
    }

    return result;
  }

  void _near_state_impl(std::vector<state> &result, const state &s, state &temp_state, unsigned int robot_i) const
  {
    unsigned int current_vertex = s.configuration[robot_i], robots_number = s.configuration.size();
    std::vector<unsigned int> near_vertices = {current_vertex};
    near_vertices.insert(near_vertices.end(), graph[current_vertex].begin(), graph[current_vertex].end());
    for (unsigned int v : near_vertices)
    {
        unsigned int next_waypoint;
        if (s.waypoint_indices[robot_i] < waypoints[robot_i].size())
        {
          next_waypoint = waypoints[robot_i][s.waypoint_indices[robot_i]];
        }
        else
        {
          next_waypoint = waypoints[robot_i].back();
        }
        
        temp_state.configuration[robot_i] = v;
        if (v == next_waypoint && s.waypoint_indices[robot_i] < waypoints[robot_i].size())
        {
          temp_state.waypoint_indices[robot_i] = s.waypoint_indices[robot_i] + 1;
        }
        else
        {
          temp_state.waypoint_indices[robot_i] = s.waypoint_indices[robot_i];
        }

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
          // std::cout << "same state" << std::endl;
          good = false;
        }

        for (int i=0; i<robots_number; i++)
        {
          for (int j=i+1; j<robots_number; j++)
          {
            if (temp_state.configuration[i] == s.configuration[j] && temp_state.configuration[j] == s.configuration[i])
            {
              // std::cout << "edge conflict" << std::endl;
              good = false;
            }

            if (temp_state.configuration[i] == temp_state.configuration[j])
            {
              // std::cout << "vertex conflict" << std::endl;
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
    // std::cout << "neighbours number: " << result.size() << std::endl;
    return result;
  }

  bool is_final_state(const state &s) const
  {
    for (int i=0; i<s.configuration.size(); i++)
    {
      if (s.configuration[i] != waypoints[i].back() || s.waypoint_indices[i] != waypoints[i].size())
      {
        return false;
      }
    }
    return true;
  }

  std::list<state> start_search(std::vector<std::vector<uint>> &paths, std::vector<std::vector<uint>> &home_paths)
  {
    unsigned int vertices = graph.size()
               , robots_number = waypoints.size();
    
    open.clear();
    initial_state.time = 0;
    initial_state.f = heuristic_function(initial_state);
    open.insert(initial_state);

    uint count = 0;
    auto start = std::chrono::system_clock::now();
    while(!open.empty())
    {
      auto end = std::chrono::system_clock::now();
      std::chrono::duration<double> diff = end - start;
      if (diff.count() > 10.0)
      {
        start = std::chrono::system_clock::now();
        std::cout << "visited nodes: " << count << std::endl;
        std::cout << "queue size: " << open.size() << std::endl;
      }
      count++;
      state s = *open.begin();
      open.erase(s);
      if (is_final_state(s))
      {
        ROS_DEBUG_STREAM("found solution!");
        std::list<state> result;

        while(true)
        {
          result.push_front(s);
          auto it = prev.find(s);
          if (it != prev.end())
          {
            s = it->second;
          }
          else
          {
            for (const state &s : result)
            {
              std::stringstream ss;
              for (int i=0; i<s.configuration.size(); i++)
              {
                unsigned int x = s.configuration[i];
                ss << std::setw(3) << x;
              }
              ROS_DEBUG_STREAM(ss.str());
            }
            return result;
          }
        }
      }

      visited.insert(s);

      for (state n : near_states(s))
      {
        // std::cout << "neighbour:\n"; 
        n.time = s.time + 1;
        n.f = n.time + heuristic_function(n);
        if (visited.count(n) == 0)
        {
          if (open.count(n) == 0 || n.time > s.time + 1)
          {
            // std::cout << "\tinserted\n";
            open.insert(n);
            prev[n] = s;
          }
          else
          {
            // std::cout << "\tworse performance\n";
          }
        }
        else
        {
          // std::cout << "\talready visited\n";
        }
        
      }
    }

    ROS_WARN_STREAM("fail!");
    ROS_DEBUG_STREAM("visited states: " << count);
    return std::list<state>();
  }
};

} // namespace mapd_time
