#pragma once
#include <vector>
#include <sstream>

using uint = unsigned int;

struct Task
{
  bool take;
  uint id;
  uint item;
  uint demand;
  uint dst;
};

// costruttore
inline Task mkTask(uint item, uint id, uint demand, uint dst)
{
  Task t;
  t.take = false;
  t.item = item;
  t.id = id;
  t.demand = demand;
  t.dst = dst;
  return t;
}

// stampa
std::ostream &operator<<(std::ostream &os, const Task &t)
{
  os << "Task id: " << t.id << "\n"
     << " - take: " << t.take << "\n"
     << " - item: " << t.item << "\n"
     << " - demand: " << t.demand << "\n"
     << " - dst: " << t.dst << "\n";
}

struct Mission
{
  bool take;
  uint id;
  std::vector<Task> mission;
  std::vector<uint> path;
  uint *dst;
  uint *total_items;
  uint total_demand;
};

std::ostream &operator<<(std::ostream &os, const Mission &m)
{
  os << "Mission id: " << m.id << "\n"
     << " - take: " << m.take << "\n"
     << " - Mission: \n";
  for (auto i = 0; m.mission.size(); i++)
  {
    os << m.mission[i] << "\n";
  }
  os << "\n";
}

struct ProcessTask
{
  uint id;
  uint tot_demand;
  uint path_distance;
  vector<Task> mission;
  vector<uint> route;
  double V;
};

inline ProcessTask mkPT(uint id, uint tot_demand, uint pd)
{
  ProcessTask pt;

  pt.id = id;
  pt.tot_demand = tot_demand;
  pt.path_distance = pd;
  pt.V = pd / tot_demand;

  return pt;
}

inline bool operator==(const ProcessTask &A, const ProcessTask &B)
{
  return A.id == B.id ? 1 : 0;
}

inline bool operator<(const ProcessTask &A, const ProcessTask &B)
{
  return A.path_distance / A.tot_demand < B.path_distance / B.tot_demand ? 1
                                                                         : 0;
}

ostream &operator<<(ostream &os, const ProcessTask &t)
{
  os << "ProcessTask id: " << t.id << " demand:" << t.tot_demand
     << " -V: " << t.V << " -Pd: " << t.path_distance;
  os << "\nMission: \n";
  for (auto i = 0; i < t.mission.size(); i++)
  {
    os << "id: " << t.mission[i].id << " dst: " << t.mission[i].dst
       << " demand: " << t.mission[i].demand << "\n";
  }
  cout << "\n";
  os << "Route: ";
  for (auto j = 0; j < t.route.size(); j++)
  {
    os << t.route[j] << " ";
  }
  os << "\n";
}

bool cmp_PT(const ProcessTask &A, const ProcessTask &B)
{
  return A.V < B.V ? 1 : 0;
}

inline bool operator==(const ProcessTask &A, const Task &B)
{
  for (auto i = 0; i < A.mission.size(); i++)
  {
    if (B.id == A.mission[i].id)
    {
      return 1;
    }
    else
    {
      return 0;
    }
  }
}

struct ProcessAgent
{
  uint ID_ROBOT;
  uint CAPACITY;
  bool take;
  vector<Task> mission; // task da concatenare
  vector<uint> route;   // vettore di vertici del path finale
  uint *dst;
  uint *total_item;
  uint total_demand;
};

ostream &operator<<(ostream &os, const ProcessAgent &pa)
{
  os << "\nProcessAgent id: " << pa.ID_ROBOT << "\n";
  for (auto i = 0; i < pa.mission.size(); i++)
  {
    os << "- id_id: " << pa.mission[i].id << "\n"
       << "-   demand: " << pa.mission[i].demand << "\n"
       << "-      dst: " << pa.mission[i].dst << "\n"
       << "\n";
  }
  for (auto k = 0; k < pa.route.size(); k++)
  {
    os << "- route: " << pa.route[k] << "\n";
  }
  os << "\n";
}

inline ProcessAgent mkPA(uint id, uint c)
{
  ProcessAgent pa;

  pa.ID_ROBOT = id;
  pa.CAPACITY = c;
  pa.take = false;
  pa.mission.clear();
  pa.route.clear(); // route definitiva
  pa.dst;
  pa.total_demand = 0;
  pa.total_item;

  return pa;
}

inline bool operator>(const ProcessAgent &A, const ProcessAgent &B)
{
  return A.CAPACITY > B.CAPACITY ? 1 : 0;
}

inline bool operator<(const ProcessAgent &A, const ProcessAgent &B)
{
  return A.CAPACITY < B.CAPACITY ? 1 : 0;
}

struct Token
{
  uint id_sender;
  uint id_receiver;
  vector<ProcessTask> mission;
  vector<ProcessTask> assigned_mission;
  uint *curr_vertex;
  uint *next_vertex;
};

ostream &operator<<(ostream &os, const Token &t)
{
  os << "\nToken - id_sender " << t.id_sender << "\n"
     << "        - id_receiver " << t.id_receiver << "\n";
  for (auto i = 0; i < t.mission.size(); i++)
  {
    os << t.mission[i] << "\n";
  }
  os << "\n";
}