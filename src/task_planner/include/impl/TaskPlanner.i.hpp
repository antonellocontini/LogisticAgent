#pragma once

namespace taskplanner
{

TaskPlanner::TaskPlanner(ros::NodeHandle &nh_)
{
  #if DBG
  sub_init = nh_.subscribe("init", 1, &TaskPlanner::init_Callback, this);
  sub_task = nh_.subscribe("need_task", 1, &TaskPlanner::task_Callback, this);
  // sub_mission = nh_.subscribe("need_mission", 1, &TaskPlanner::mission_Callback, this);

  pub_task = nh_.advertise<task_planner::Task>("answer", 1);
  pub_results = nh_.advertise<std_msgs::Int16MultiArray>("results", 100);
  // pub_results = nh_.advertise<tcp_interface::RCOMMessage>("results", 100);
  #endif

  pub_init = nh_.advertise<std_msgs::Int16MultiArray>("init", 1);
}

void TaskPlanner::t_print(Task &t)
{
  cout << "\nTask" << t.order << ":\n"
       << " -      item: " << t.item << "\n"
       << " -    demand: " << t.demand << "\n"
       << " -       dst: " << t.dst << "\n";
}

void TaskPlanner::pa_print(ProcessAgent &pa)
{
  cout << "\nProcessAgent: \n"
       << " -  id_robot: " << pa.ID_ROBOT << "\n"
       << " -  capacity: " << pa.CAPACITY << "\n"
       << " -      flag: " << pa.flag << "\n"
       << "mission.size: " << pa.mission.size() << "\n"
       << " -     route: ";
  for (auto i = 0; i < pa.route.size(); i++)
  {
    cout << pa.route[i].id_vertex << " ";
  }
  cout << "\n";
}

void TaskPlanner::ct_print(CandidateTask &ct)
{
  cout << "\nCandidateTask: \n"
       << " -        id: " << ct.id << "\n"
       << " -    subset: " << ct.subset << "\n"
       << " - candidate: ";
  cout << "{";
  for (auto i = 0; i < ct.subset; i++)
  {
    auto el = ct.vv_t[i];
    cout << "(";
    for (auto j = 0; j < el.size() - 1; j++)
    {
      cout << el[j] << " ";
    }
    cout << el.back() << ")";
  }
  cout << "}"
       << "\n";
  cout << " -         V: " << ct.V << "\n";
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
  TEAM_t = atoi(argv[3]);

  t_generator();

  c_print("TEAM: ", TEAM_t, " nTask: ", nTask, magenta);
  init_agent = new bool[TEAM_t]();
  //                           ^ figo!
  pa = new ProcessAgent[TEAM_t];

  c_print("INIT", green);
}

void TaskPlanner::init_Callback(const std_msgs::Int16MultiArrayConstPtr &msg)
{
  /* vettore dove:
  [0] = id_robot
  [1] = type
  [2] = data
  */

  int value = msg->data[0];
  int type_msg = msg->data[1];

  if (value == -1)
  {
    value = 0;
  }

  switch (type_msg)
  {
    case (INIT_MSG):
    {
      // inizializzazione dei robot acquisisco Capacity
      init_agent[value] = true;
      auto c = msg->data[2];
      TEAM_c += c;
      c_print("TEAM_C: ", TEAM_c, red);
      pa[value] = mkPA(value, c);
      pa_print(pa[value]);
      uint T_t = TEAM_t;
      for (int i = 0; i < TEAM_t; i++)
      {
        if (init_agent[i] == true)
        {
          T_t--;
        }
      }
      if (T_t == 0)
      {
        compute_best_subtask();
        auto ele = pq_ct.top();
        ct_print(ele);

        for (auto i = 0; i < ele.subset; i++)
        {
          auto s_el = ele.vv_t[i].size();
          auto el = ele.vv_t[i];
          for (auto j = 0; j < s_el; j++)
          {
            // auto e = el[j].route;
            auto r = el[j];
            v_pt.push_back(r);
          }
        }
        // sort(v_pt.begin(), v_pt.end(), cmp_PT);

        for (auto h = 0; h < v_pt.size(); h++)
        {
          cout << v_pt[h] << " ";
        }
        cout << " ";
        c_print("fine", green);
      }
    }
    break;

    case (INIT_MSG2):
    {
      init_agent[value] = true;
      auto c = msg->data[2];
      TEAM_c += c;
      c_print("TEAM_C: ", TEAM_c, red);
      pa[value] = mkPA(value, c);
      pa_print(pa[value]);
      uint T_t = TEAM_t;
      for (int i = 0; i < TEAM_t; i++)
      {
        if (init_agent[i] == true)
        {
          T_t--;
        }
      }
      if (T_t == 0)
      {
        compute_CF();
      }
    }
    break;

    case (INIT_MSG3):
    {
      init_agent[value] = true;
      auto c = msg->data[2];
      TEAM_c += c;
      c_print("TEAM_C: ", TEAM_c, red);
      pa[value] = mkPA(value, c);
      pa_print(pa[value]);
      uint T_t = TEAM_t;
      for (int i = 0; i < TEAM_t; i++)
      {
        if (init_agent[i] == true)
        {
          T_t--;
        }
      }
      if (T_t == 0)
      {
        // posso fare qualcosa tipo avvisare i robot che possono partire
        // ma chi parte? direi in modo sequenziale... serve ricevitore del messaggio
        // pub_init.
      }
    }
    break;

    default:
      break;
  }
}

void TaskPlanner::t_generator()
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
  for (auto k = 0; k < nTask; k++)
    t_print(tasks[k]);
}

void TaskPlanner::compute_route_to_delivery(ProcessAgent *pa)
{
  // if (pa.mission.size() == 1)

  // // {
  Route step;
  auto el = pa;
  cout << el << "\n";
  cout << "pa.mission.size(): " << el->mission.size() << "\n";

  step.id_vertex = src_vertex;
  step.status = true;
  el->route.push_back(step);
  auto dst = el->mission.front().dst;
  cout << "pa.mission.size(): " << el->mission.size() << "\n";
  int i = 0;
  switch (dst)
  {
    case 11:
      i = 3;
      break;
    case 16:
      i = 5;
      break;
    case 21:
      i = 7;
      break;
    default:
      c_print("# ERR t.dst non esiste!", red);
      break;
  }
  // Route step;
  for (int j = 0; j < i; j++)
  {
    step.id_vertex = under_pass[j];
    step.status = false;
    // tmp_route.push_back(step);
    pa->route.push_back(step);
  }
  step.id_vertex = dst;
  step.status = true;
  // tmp_route.push_back(step);
  pa->route.push_back(step);
  cout << el << "\n";
}

void TaskPlanner::compute_route_to_picktask(ProcessAgent *pa)
{
  auto el = pa;
  Route step;
  int i = 0;
  cout << "pa.mission.size(): " << el->mission.size() << "\n";
  auto dst = el->mission.back().dst;
  switch (dst)
  {
    case 11:
      i = 3;
      break;
    case 16:
      i = 5;
      break;
    case 21:
      i = 7;
      break;
    default:
      c_print("# ERR t.dst non esiste!", red);
      break;
  }
  for (int j = i - 1; j >= 0; --j)
  {
    step.id_vertex = upper_pass[j];
    step.status = false;
    // tmp_route.push_back(step);
    pa->route.push_back(step);
  }
  cout << el << "\n";
}

int TaskPlanner::compute_cost_of_route(ProcessAgent *pa)
{
  auto el = pa;
  int custo_final = 0;
  // int anterior, proximo;

  cout << "pa.route.size(): " << pa->route.size() << "\n";

  for (int i = 1; i < pa->route.size(); i++)
  {
    int anterior = pa->route[i - 1].id_vertex;
    int proximo = pa->route[i].id_vertex;

    for (int j = 0; j < vertex_web[anterior].num_neigh; j++)
    {
      if (vertex_web[anterior].id_neigh[j] == proximo)
      {
        custo_final += vertex_web[anterior].cost[j];
        break;
      }
    }
  }

  c_print("costo del percorso: ", custo_final, magenta);

  return custo_final;
}

int TaskPlanner::ccor(vector<uint> route)
{
  int custo_final = 0;
  for (int i = 1; i < route.size(); i++)
  {
    int anterior = route[i - 1];
    int proximo = route[i];

    for (int j = 0; j < vertex_web[anterior].num_neigh; j++)
    {
      if (vertex_web[anterior].id_neigh[j] == proximo)
      {
        custo_final += vertex_web[anterior].cost[j];
        break;
      }
    }
  }
  return custo_final;
}

uint TaskPlanner::compute_cycle_dst(vector<Task> mission)
{
  /*
   1 =11
   2 = 16
   3 = 21
   4 = 11-16
   5 = 11-21
   6 = 16-21
   7 = 11-16-21 */
  uint res = 0;
  vector<uint> tmp_d;
  tmp_d.clear();
  for (auto i = 0; i < mission.size(); i++)
  {
    auto d = mission[i].dst;
    tmp_d.push_back(d);
  }
  sort(tmp_d.begin(), tmp_d.end());
  tmp_d.erase(unique(tmp_d.begin(), tmp_d.end()), tmp_d.end());

  if (tmp_d.size() == 1)
  {
    if (tmp_d[0] == 11)
      res = 1;
    else if (tmp_d[0] == 16)
      res = 2;
    else
      res = 3;
  }
  else if (tmp_d.size() == 2)
  {
    if ((tmp_d[0] == 11) && (tmp_d[1] == 16))
    {
      res = 4;
    }
    else if ((tmp_d[0] == 11) && (tmp_d[1] == 21))
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

void TaskPlanner::compute_route(uint id_path, ProcessTask *pt)
{
  auto ele = pt;
  /* 1 =11
     2 = 16
     3 = 21
     4 = 11-16
     5 = 11-21
     6 = 16-21
     7 = 11-16-21 */
  switch (id_path)
  {
    case 1:
    {
      for (auto i = 0; i < 8; i++)
      {
        // v_pt[j].route.push_back(p_11[i]);
        ele->route.push_back(p_11[i]);
      }
      // v_pt[j].path_distance = ccor(v_pt[j].route);
      ele->path_distance = ccor(ele->route);
      ele->V = (double)ele->path_distance / (double)ele->tot_demand;
      // c_print("V: ", ele->V, red);
    }
    break;
    case 2:
    {
      for (auto i = 0; i < 12; i++)
      {
        ele->route.push_back(p_16[i]);
      }
      ele->path_distance = ccor(ele->route);
      // ele->V = ele->path_distance / ele->tot_demand;
      ele->V = (double)ele->path_distance / (double)ele->tot_demand;
      // c_print("V: ", ele->V, red);
    }
    break;
    case 3:
    {
      for (auto i = 0; i < 16; i++)
      {
        ele->route.push_back(p_21[i]);
      }
      ele->path_distance = ccor(ele->route);
      // ele->V = ele->path_distance / ele->tot_demand;
      ele->V = (double)ele->path_distance / (double)ele->tot_demand;
      // c_print("V: ", ele->V, red);
    }
    break;
    case 4:
    {
      for (auto i = 0; i < 14; i++)
      {
        ele->route.push_back(p_11_16[i]);
      }
      ele->path_distance = ccor(ele->route);
      ele->V = (double)ele->path_distance / (double)ele->tot_demand;
      // ele->V = ele->path_distance / ele->tot_demand;
      // c_print("V: ", ele->V, red);
    }
    break;

    case 5:
    {
      for (auto i = 0; i < 18; i++)
      {
        ele->route.push_back(p_11_21[i]);
      }
      ele->path_distance = ccor(ele->route);
      ele->V = (double)ele->path_distance / (double)ele->tot_demand;
      // ele->V = ele->path_distance / ele->tot_demand;
      // c_print("V: ", ele->V, red);
    }
    break;
    case 6:
    {
      for (auto i = 0; i < 18; i++)
      {
        ele->route.push_back(p_16_21[i]);
      }
      ele->path_distance = ccor(ele->route);
      ele->V = (double)ele->path_distance / (double)ele->tot_demand;
      // ele->V = ele->path_distance / ele->tot_demand;
      // c_print("V: ", ele->V, red);
    }
    break;
    case 7:
    {
      for (auto i = 0; i < 20; i++)
      {
        ele->route.push_back(p_11_16_21[i]);
      }
      ele->path_distance = ccor(ele->route);
      ele->V = (double)ele->path_distance / (double)ele->tot_demand;
      // ele->V = ele->path_distance / ele->tot_demand;
      // c_print("V: ", ele->V, red);
    }
    break;
    default:
    {
      c_print("ERR", red);
    }
    break;
  }
}

void TaskPlanner::prepare_missions()
{
  auto max_el = *std::max_element(pa, pa + TEAM_t);
  cout << "elemento massimo " << max_el.CAPACITY << "\n";
  auto t_size = tasks.size();
  try
  {
    partition::iterator it(t_size);
    int id = 0;
    while (true)
    {
      auto all_part = *it[tasks];
      auto subset = it.subsets();
      auto as = all_part.size();
      auto ct = mkCT(id, subset);  // creo un Candidato
      id++;
      int pt_id = 0;
      double tmp_V = 0;
      for (auto k = 0; k < subset; k++)
      {
        int tmp_d = 0;
        auto part = all_part[k];
        // un processTask e' un subset di Task
        ProcessTask pt;
        pt.id = pt_id;
        pt_id++;
        for (auto j = 0; j < part.size(); j++)
        {
          tmp_d += part[j].demand;
          pt.mission.push_back(part[j]);
        }
        pt.tot_demand = tmp_d;
        if (tmp_d > max_el.CAPACITY)
          ct.good++;
        uint id_path = compute_cycle_dst(pt.mission);
        compute_route(id_path, &pt);
        tmp_V += pt.V;
        ct.vv_t[k].push_back(pt);
      }
      ++it;
      ct.V = tmp_V;
      v_ct.push_back(ct);
    }
  }
  catch (std::overflow_error &)
  {
  }
  c_print("fine preparazione!", red);
}

void TaskPlanner::compute_best_subtask()
{
  prepare_missions();

  for (vector<CandidateTask>::iterator it = v_ct.begin(); it != v_ct.end();)
  {
    if (it->good == 0)
    {
      pq_ct.push(*it);
    }
    ++it;
  }

  v_ct.clear();
}

void TaskPlanner::compute_CF()
{
  auto max_el = *std::max_element(pa, pa + TEAM_t);
  cout << "elemento massimo " << max_el.CAPACITY << "\n";

  int id_pt = 0;
  for (auto i = 0; i < tasks.size(); i++)
  {
    ProcessTask pt;
    pt.id = id_pt;
    id_pt++;
    pt.mission.push_back(tasks[i]);
    pt.tot_demand = tasks[i].demand;
    uint id_path = compute_cycle_dst(pt.mission);
    compute_route(id_path, &pt);
    v_pt.push_back(pt);
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
}

void TaskPlanner::run()
{
  // TODO
}

#if DBG
void TaskPlanner::task_Callback(const patrolling_sim::TaskRequestConstPtr &tr)
{
  bool single_task = true;
  uint id_robot = tr->ID_ROBOT;
  auto el = &pa[id_robot];
  //        ^ importante!

  task_planner::Task tm;

  if ((single_task) && (tasks.size() >= 1))
  {
    Task t = *std::min_element(tasks.begin(), tasks.end(), pop_min_element);

    el->mission.push_back(t);

    compute_route_to_delivery(el);
    compute_route_to_picktask(el);
    int path_distance = compute_cost_of_route(el);

    tm.header.stamp = ros::Time().now();
    tm.ID_ROBOT = tr->ID_ROBOT;
    tm.take = true;
    tm.go_home = false;
    tm.demand = t.demand;
    tm.item = t.item;
    tm.order = t.order;
    tm.dst = t.dst;
    tm.path_distance = path_distance;
    c_print("\nRoute:", red);
    for (auto i = 0; i < el->route.size(); i++)
    {
      tm.route.push_back(el->route[i].id_vertex);
      tm.condition.push_back(el->route[i].status);
      cout << setw(3) << el->route[i].id_vertex << "   Status: [ " << el->route[i].status << " ]"
           << "\n";
    }
    cout << "\n";
    c_print("% publish on topic mission! Task n: ", t.order, " ID_robot: ", tm.ID_ROBOT, yellow);
    pub_task.publish(tm);
    el->mission.clear();
    el->route.clear();
    tasks.erase(std::find(tasks.begin(), tasks.end(), t));
    c_print("Size tasks: ", tasks.size(), red);
    single_task = false;
  }
  else
  {
    c_print("Task finiti!", red);
    tm.header.stamp = ros::Time().now();
    tm.ID_ROBOT = tr->ID_ROBOT;
    tm.take = false;
    tm.demand = 0;
    tm.item = 3;
    tm.go_home = true;
    tm.dst = initial_position[id];
    c_print("% publish on topic mission! go_home ID_robot: ", tm.ID_ROBOT, yellow);
    id++;
    pub_task.publish(tm);
    el->mission.clear();
    el->route.clear();
  }
  ros::spinOnce();
  sleep(1);
}

// void TaskPlanner::mission_Callback(const patrolling_sim::MissionRequestConstPtr &mr)
// {
//   c_print("Request Mission! id_robot: ", mr->ID_ROBOT, green);

//   bool single_task = true;
//   uint id_robot = mr->ID_ROBOT;
//   task_planner::Task tm;

//   if ((single_task) && (v_pt.size() >= 1))
//   {
//     ProcessTask pt = v_pt.front();
//     tm.header.stamp = ros::Time().now();
//     tm.ID_ROBOT = id_robot;
//     tm.take = true;
//     tm.go_home = false;
//     tm.demand = pt.tot_demand;
//     tm.item = 0;  // per ora!
//     tm.order = pt.id;
//     tm.dst = 0;  // per ora
//     tm.path_distance = pt.path_distance;
//     for (auto i = 0; i < pt.route.size(); i++)
//     {
//       tm.route.push_back(pt.route[i]);
//       tm.condition.push_back(false);
//     }
//     pub_task.publish(tm);
//     v_pt.erase(find(v_pt.begin(), v_pt.end(), pt));
//     single_task = false;
//   }
//   else
//   {
//     tm.header.stamp = ros::Time().now();
//     tm.ID_ROBOT = mr->ID_ROBOT;
//     tm.take = false;
//     tm.demand = 0;
//     tm.item = 3;
//     tm.go_home = true;
//     tm.dst = initial_position[id];
//     c_print("% publish on topic mission! go_home ID_robot: ", tm.ID_ROBOT, yellow);
//     id++;
//     pub_task.publish(tm);
//   }
//   ros::spinOnce();
//   sleep(1);
//   c_print("fine", red);
// }
#endif

}  // namespace taskplanner
