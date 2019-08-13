#pragma once

namespace taskplanner
{

TaskPlanner::TaskPlanner(ros::NodeHandle &nh_)
{
  sub_token = nh_.subscribe("token", 1, &TaskPlanner::token_Callback, this);
  sub_task = nh_.subscribe("task", 10, &TaskPlanner::task_Callback, this);

  pub_task = nh_.advertise<task_planner::Task>("answer", 10);
  pub_token = nh_.advertise<std_msgs::Int16MultiArray>("token", 1);
}




void TaskPlanner::init(int argc, char **argv)
{
  srand(time(NULL));
  chdir(PS_path.c_str());
  string mapname = string(argv[1]);
  string graph_file = "/home/antonello/Tesi/LogisticAgent_ws/src/patrolling_sim/maps/" + mapname + "/" + mapname + ".graph";
  uint dimension = GetGraphDimension(graph_file.c_str());
  vertex_web = new vertex[dimension];
  GetGraphInfo(vertex_web, dimension, graph_file.c_str());
  uint nedges = GetNumberEdges(vertex_web, dimension);
  printf("Loaded graph %s with %d nodes and %d edges\n", mapname.c_str(), dimension, nedges);
  TEAM_SIZE = atoi(argv[3]);

  task_generator();

  c_print("TEAM: ", TEAM_SIZE, " nTask: ", nTask, magenta);
  init_agent = new bool[TEAM_SIZE]();
  //                           ^ figo!
  pa = new ProcessAgent[TEAM_SIZE];

  c_print("INIT", green);
}


void TaskPlanner::task_generator()
{
  uint n_item = 3;
  uint o = 0;
  uint n_demand = 3;
  uint j = 0;
  for (auto h = 0; h < 3; h++)
  {
    j = 0;
    for (auto d = 1; d <= n_demand; d++)
    {
      // for (auto i = 0; i < n_item; i++)
      // {
        tasks.push_back(mkTask(d-1, o, d, dst_vertex[j]));
        j++;
        // task_set.insert(mkTask(i,o,d,dst_vertex[j]));
        o++;
      // }
    }
  }
  nTask = tasks.size();
}


/* void TaskPlanner::compute_CF()
{
  auto max_el = *std::max_element(pa, pa + TEAM_SIZE);
  cout << "elemento massimo " << max_el.CAPACITY << "\n";

  int id_pt = 0;
  for (auto i = 0; i < tasks.size(); i++)
  {
    ProcessTask pt;
    pt.id = id_pt;
    id_pt++;
    pt.mission.push_back(tasks[i]);
    pt.tot_demand = tasks[i].demand;
    // uint id_path = compute_cycle_dst(pt.mission);
    // compute_route(id_path, &pt);
    // v_pt.push_back(pt);
  }

  ProcessTask el;
  ProcessTask el2;
  auto t = v_pt.size();
  for (auto k = 0; k < v_pt.size(); k++)
  {
    el = v_pt[k];
    for (auto q = 0; q < v_pt.size(); q++)
    {
      el2 = v_pt[q];
      if (el.id != el2.id)
      {
        auto tmp_d = el.tot_demand + el2.tot_demand;
        if (tmp_d <= max_el.CAPACITY)
        {
          ProcessTask n_pt;
          n_pt.id = t;
          t++;
          //                    ^ prende la taglia piu uno: possibili id uguali! ma non importante.
          n_pt.tot_demand = tmp_d;
          for (auto t = 0; t < el.mission.size(); t++)
            n_pt.mission.push_back(el.mission[t]);
          for (auto y = 0; y < el2.mission.size(); y++)
            n_pt.mission.push_back(el2.mission[y]);
          uint id_path = compute_cycle_dst(n_pt.mission);
          compute_route(id_path, &n_pt);

          // minimization
          if ((n_pt.V - el.V - el2.V) < 0)  // con < di zero allora accoppia
          {
            cout << "i: " << el.id << " ed: " << el2.id << "\n";
            v_pt.erase(find(v_pt.begin(), v_pt.end(), el));
            v_pt.erase(find(v_pt.begin(), v_pt.end(), el2));
            v_pt.push_back(n_pt);
            // pq_pt.push(n_pt);
            break;
          }
          else
          {
            delete &n_pt;
          }
        }
      }
    }

    // auto ele = pq_pt.top();

    // for (auto i = 0; i < ele.mission.size(); i++)
    // {
    // //   // 
    //   v_pt.erase(find(v_pt.begin(), v_pt.end(), ele.mission[i]));
    //   // v_pt.erase(find(v_pt.begin(), v_pt.end(), el2));
      
    // }

    // v_pt.push_back(ele);

    // pq_pt.clear();

    // pq_pt = priority_queue<ProcessTask> ();
    // break;
  }

  // sort(v_pt.begin(), v_pt.end(), cmp_PT);

  c_print("soluzione", red);

  for (auto j = 0; j < v_pt.size(); j++)
  {
    cout << v_pt[j] << " ";
  }
  cout << "\n";
} */


void TaskPlanner::task_Callback(const logistic_sim::TaskRequestConstPtr &tr)
{
 
}

void TaskPlanner::token_Callback(const logistic_sim::TokenConstPtr &msg)
{
  
}


}  // namespace taskplanner
