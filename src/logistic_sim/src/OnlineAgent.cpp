#include "OnlineAgent.hpp"

namespace onlineagent
{

void OnlineAgent::init(int argc, char **argv)
{
    CFreeAgent::init(argc, argv);
    allocate_memory();
    map_graph = std::vector<std::vector<unsigned int>>(dimension);
    for (int i = 0; i < dimension; i++)
    {
        for (int j = 0; j < vertex_web[i].num_neigh; j++)
        {
            map_graph[i].push_back(vertex_web[i].id_neigh[j]);
        }
    }
}

void OnlineAgent::token_callback(const logistic_sim::TokenConstPtr &msg)
{
    // se non è per me termino
    if (msg->ID_RECEIVER != ID_ROBOT)
        return;

    logistic_sim::Token token;
    token = *msg;

    token.ID_SENDER = ID_ROBOT;
    if (msg->ID_RECEIVER == TEAM_SIZE - 1)
    {
        token.ID_RECEIVER = TASK_PLANNER_ID;
    }
    else
    {
        token.ID_RECEIVER = (ID_ROBOT + 1) % TEAM_SIZE;
    }

    if (msg->INIT)
    {
        c_print("Inizializzazione...", green, P);
        token.CAPACITY.push_back(CAPACITY);
        token.INIT_POS.push_back(initial_vertex);
        token.MISSION_START_TIME.push_back(ros::Time::now());
        token.MISSION_CURRENT_DISTANCE.push_back(0.0f);
        token.INTERFERENCE_COUNTER.push_back(0);
        token.MISSIONS_COMPLETED.push_back(0);
        token.TASKS_COMPLETED.push_back(0);
        token.TOTAL_DISTANCE.push_back(0.0f);
        token.X_POS.push_back(0.0);
        token.Y_POS.push_back(0.0);
        token.GOAL_STATUS.push_back(0);
        logistic_sim::Path p;
        p.PATH = std::vector<uint>(1, initial_vertex);
        token.TRAILS.push_back(p);
        token.REACHED_HOME.push_back(false);
        token.ACTIVE_ROBOTS = TEAM_SIZE;

        initialize = false;
    }

    if (msg->ALLOCATE)
    {
        c_print("Allocazione task...", green, P);
        if (token.ACTIVE_ROBOTS == 0)
        {
            c_print("Impossibile allocare missioni anche con un solo robot", red, P);
            // mando il token al taskplanner segnalando la terminazione
            token.SHUTDOWN = true;
        }
        else if (ID_ROBOT < token.ACTIVE_ROBOTS) // robot attivo
        {
            // prendo una missione tra quelle disponibili
            auto it = token.TAKEN_MISSION.begin();
            for (; it != token.TAKEN_MISSION.end(); it++)
            {
                int i = it - token.TAKEN_MISSION.begin();
                if (*it == -1)
                {
                    break;
                }
            }

            // trovata una missione non assegnata
            if (it != token.TAKEN_MISSION.end())
            {
                logistic_sim::Mission m;
                missions.push_back(m);
            }
            else if (ID_ROBOT == TEAM_SIZE - 1) // tutte le missioni sono state assegnate
            {
                c_print("Missioni assegnate", green, P);
                token.ALLOCATE = false;
                token.GOOD_PATHS = true;
                token.CALCULATE_PATHS = true;
            }
        }
    }
    else if (msg->CALCULATE_PATHS)
    {
        if (msg->GOOD_PATHS && ID_ROBOT < token.ACTIVE_ROBOTS)
        {
            uint init_pos = initial_vertex;
            std::vector<uint> waypoints = {current_vertex};
            for (logistic_sim::Mission &m : missions)
            {
                waypoints.push_back(src_vertex);
                for (uint dst : m.DSTS)
                {
                    waypoints.push_back(dst);
                }
            }
            waypoints.push_back(init_pos);
            try
            {
                std::vector<unsigned int> path = spacetime_dijkstra(token.TRAILS, map_graph, waypoints, std::max(0UL, token.TRAILS[ID_ROBOT].PATH.size() - 1));
                token.NEW_TRAILS[ID_ROBOT].PATH = path;

                // l'ultimo robot attivo pusha i nuovi percorsi
                if (ID_ROBOT == token.ACTIVE_ROBOTS - 1)
                {
                    for (int i = 0; i < TEAM_SIZE; i++)
                    {
                        token.TRAILS[i].PATH.insert(
                            token.TRAILS[i].PATH.end(),
                            token.NEW_TRAILS[i].PATH.begin(),
                            token.NEW_TRAILS[i].PATH.end());
                    }
                }
            }
            catch (const std::string &e)
            {
                c_print("Impossibile pianificare con ", token.ACTIVE_ROBOTS, red, P);
                token.GOOD_PATHS = false;
            }
        }

        if (!token.GOOD_PATHS)
        {
            missions.clear();

            if (ID_ROBOT == TEAM_SIZE - 1)
            {
                token.ACTIVE_ROBOTS--;
                token.ALLOCATE = true;
                token.CALCULATE_PATHS = false;
                token.TAKEN_MISSION = std::vector<int>(token.MISSION.size(), -1);
            }
        }
    }

    // avanzamento dei robot

    // aggiorno posizioni dei robot nel token
    token.X_POS[ID_ROBOT] = xPos[ID_ROBOT];
    token.Y_POS[ID_ROBOT] = yPos[ID_ROBOT];
    for (int i = 0; i < TEAM_SIZE; i++)
    {
        if (i != ID_ROBOT)
        {
            xPos[i] = token.X_POS[i];
            yPos[i] = token.Y_POS[i];
        }
    }

    // metto nel token quale arco sto occupando
    token.CURR_VERTEX[ID_ROBOT] = current_vertex;
    token.NEXT_VERTEX[ID_ROBOT] = next_vertex;

    // gestisco l'arrivo al goal
    if (goal_complete)
    {
        static bool first_time = true;

        if (first_time)
        {
            // aggiorno distanza percorsa nel token
            uint edge_length = 0;
            if (current_vertex < dimension)
            {
                for (int i = 0; i < vertex_web[current_vertex].num_neigh; i++)
                {
                    if (vertex_web[current_vertex].id_neigh[i] == next_vertex)
                    {
                        edge_length += vertex_web[current_vertex].cost[i];
                        break;
                    }
                }
            }
            token.TOTAL_DISTANCE[ID_ROBOT] += edge_length;

            if (token.GOAL_STATUS[ID_ROBOT] > 0 && token.TRAILS[ID_ROBOT].PATH.size() > 1)
            {
                token.TRAILS[ID_ROBOT].PATH.erase(token.TRAILS[ID_ROBOT].PATH.begin());
            }
            token.GOAL_STATUS[ID_ROBOT]++;

            // TODO: bisogna gestire condizione di terminazione
            if (token.ALL_MISSIONS_INSERTED && token.TRAILS[ID_ROBOT].PATH.size() == 1)
            {
                token.REACHED_HOME[ID_ROBOT] = true;
                bool can_exit = true;
                for (int i = 0; i < TEAM_SIZE; i++)
                {
                    if (!token.REACHED_HOME[i])
                    {
                        can_exit = false;
                        break;
                    }
                }
                if (can_exit)
                {
                    token.SHUTDOWN = true;
                }
            }
        }
        current_vertex = next_vertex;

        bool equal_status = true;
        int status = token.GOAL_STATUS[0];
        for (int i = 1; i < TEAM_SIZE; i++)
        {
            if (token.GOAL_STATUS[i] != status)
                equal_status = false;
        }

        if (equal_status)
        {
            first_time = true;
            if (token.TRAILS[ID_ROBOT].PATH.size() > 1)
            {
                goal_complete = false;
                next_vertex = token.TRAILS[ID_ROBOT].PATH[1];
            }
            else
            {
                c_print("[ WARN]Don't know what to do!!!", yellow, P);
                next_vertex = current_vertex;
            }
            c_print("before OnGoal()", magenta);
            c_print("[DEBUG]\tGoing to ", next_vertex, green, P);
            sendGoal(next_vertex);
        }
    }

    ros::Duration(0.03).sleep();
    token_pub.publish(token);
    ros::spinOnce();

    if (token.SHUTDOWN)
    {
        c_print("Segnale di shutdown, termino...", magenta, P);
        ros::shutdown();
    }
}

std::vector<unsigned int> OnlineAgent::spacetime_dijkstra(const std::vector<logistic_sim::Path> &other_paths,
                                                          const std::vector<std::vector<unsigned int>> &graph,
                                                          const std::vector<unsigned int> &waypoints,
                                                          int start_time)
{
    std::cout << "SPACETIME DIJKSTRA --- WAYPOINTS:";
    for (int i = 0; i < waypoints.size(); i++)
    {
        std::cout << " " << waypoints[i];
    }
    std::cout << std::endl;

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
        unsigned int time = current_st.time;
        unsigned int current_waypoint = current_st.waypoint;
        unsigned int next_next_waypoint;

        visited[u][time][current_waypoint] = BLACK;

        if (u == waypoints[current_waypoint])
        {
            if (current_waypoint + 1 == waypoints.size())
            {
                if (time + 1 >= max_path_size)
                {
                    std::vector<unsigned int> result =
                        std::vector<unsigned int>(prev_paths[u][time][current_waypoint],
                                                  prev_paths[u][time][current_waypoint] +
                                                      path_sizes[u][time][current_waypoint]);

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
        else // u non è waypoint
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
                        if (other_paths[i].PATH[next_time] == u)
                        {
                            std::cout << "can't stand in " << u << " at time " << time << std::endl;
                            good = false;
                            break;
                        }
                    }
                    else if (!other_paths[i].PATH.empty() /* && still_robots[i] */)
                    {
                        if (other_paths[i].PATH.back() == u)
                        {
                            std::cout << "can't stand in " << u << " at time " << time << std::endl;
                            good = false;
                            break;
                        }
                    }
                }
            }

            if (good)
            {
                st_location next_st(u, next_time, next_next_waypoint);
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
                                std::cout << "can't go in " << v << " at time " << time << std::endl;
                                good = false;
                                break;
                            }
                            if (other_paths[i].PATH[next_time] == u && other_paths[i].PATH[time] == v)
                            {
                                std::cout << "can't go in " << v << " at time " << time << std::endl;
                                good = false;
                                break;
                            }
                        }
                        else if (!other_paths[i].PATH.empty() /* && still_robots[i] */)
                        {
                            if (other_paths[i].PATH.back() == v)
                            {
                                std::cout << "can't go in " << v << " at time " << time << ", there is a still robot" << std::endl;
                                good = false;
                                break;
                            }
                        }
                    }
                }

                if (good)
                {
                    st_location next_st(v, next_time, next_next_waypoint);
                    queue_size = insertion_sort(queue, queue_size, next_st);
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

} // namespace onlineagent

int main(int argc, char *argv[])
{
    return 0;
}