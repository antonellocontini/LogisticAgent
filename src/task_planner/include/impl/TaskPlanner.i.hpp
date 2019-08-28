#pragma once

namespace taskplanner
{

TaskPlanner::TaskPlanner(ros::NodeHandle &nh_)
{
  sub_token = nh_.subscribe("token", 1, &TaskPlanner::token_Callback, this);
  pub_token = nh_.advertise<logistic_sim::Token>("token", 1);
}

void TaskPlanner::init(int argc, char **argv)
{
  srand(time(NULL));
  chdir(PS_path.c_str());
  string mapname = string(argv[1]);
  string graph_file = "maps/" + mapname + "/" + mapname + ".graph";
  uint dimension = GetGraphDimension(graph_file.c_str());
  vertex_web = new vertex[dimension];
  GetGraphInfo(vertex_web, dimension, graph_file.c_str());
  uint nedges = GetNumberEdges(vertex_web, dimension);
  printf("Loaded graph %s with %d nodes and %d edges\n", mapname.c_str(), dimension, nedges);
  TEAM_SIZE = atoi(argv[3]);

  missions_generator();

  c_print("TEAM: ", TEAM_SIZE, " nTask: ", nTask, magenta);
 
  // aspetto che arrivino gli agenti
  sleep(10);

  // giro di inizializzazione
  logistic_sim::Token token;
  token.ID_SENDER = TASK_PLANNER_ID;
  token.ID_RECEIVER = 0;
  token.INIT = true;

  pub_token.publish(token);
  ros::spinOnce();

  sleep(1);

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
      tasks.push_back(mkTask(d - 1, o, d, dst_vertex[j]));
      j++;
      // task_set.insert(mkTask(i,o,d,dst_vertex[j]));
      o++;
      // }
    }
  }
  nTask = tasks.size();
}


int TaskPlanner::compute_cost_of_route(std::vector<uint> route)
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


void TaskPlanner::mission_generator()
{
  int size = 10;
  int d = 1;
  for (auto i = 0; i < size; i++)
  {
    logistic_sim::Mission m;
    m.PICKUP = false;
    m.ID = i;
    m.PRIORITY = 0;
    m.ITEM.push_back(i%3);
    swicth(i%3)
    {
      case 0:
      {
        m.ROUTE = p_11;
        break;
      }
      case 1:
      {
        m.ROUTE = p_16;
        break;
      }
      case 2:
      {
        m.ROUTE = p_21;
        break;
      }
      default
      {
        c_print("[DEBUG]",yellow,P);
        break;
      }
    }
    m.DSTS.push_back(dst_vertex[i%3]);
    m.V = 0.0;
    m.TOT_DEMAND = (i%3)+1;
    m.PATH_DISTANCE = compute_cost_of_route(m.ROUTE);
  }

  missions.push_back(m);

  nTask = missions.size();
}


void TaskPlanner::token_Callback(const logistic_sim::TokenConstPtr &msg)
{
  c_print(msg->ID_SENDER, " ", msg->ID_RECEIVER, green);

  if (msg->ID_RECEIVER != TASK_PLANNER_ID)
    return;

  logistic_sim::Token token;
  token = *msg;
  token.ID_SENDER = TASK_PLANNER_ID;
  token.ID_RECEIVER = 0;
  if (msg->INIT)
  {
    CAPACITY = msg->CAPACITY;
    token.INIT = false;
    // inserire task
    token.MISSION = missions;
  }
  else
  {
    // se devo inserire altri task e monitor
  }

  pub_token.publish(token);
  ros::spinOnce();
}

} // namespace taskplanner
