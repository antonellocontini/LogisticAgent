#pragma once

using namespace cfreeagent;

bool CFreeAgent::token_check_pt(std::vector<uint> &my_path, std::vector<logistic_sim::Path> &other_paths, uint CHECK_ID, int *id_vertex_stuck)
{
	bool status = true;
	// controllo che la posizione vicina sia raggiungibile in questo turno
	for (int j = 0; j < my_path.size(); j++)
	{

		for (int i = 0; i < other_paths.size(); i++)
		{
			if (i != CHECK_ID)
			{
				const logistic_sim::Path &path = other_paths[i];
				if (j + 1 < path.PATH.size() && j + 1 < my_path.size())
				{
					int other_pos = path.PATH[j];
					int other_next_pos = path.PATH[j + 1];
					if (my_path[j + 1] == other_next_pos)
					{
						std::cout << "conflict robot id: " << i << std::endl;
						*id_vertex_stuck = my_path[j + 1];
						return false;
					}
					if (my_path[j + 1] == other_pos && my_path[j] == other_next_pos)
					{
						std::cout << "conflict robot id: " << i << std::endl;
						*id_vertex_stuck = my_path[j + 1];
						return false;
					}
				}
			}
		}
	}
	return status;
}

struct st_location
{
    unsigned int vertex;
    unsigned int time;

    st_location(unsigned int vertex = 0, unsigned int time = 0) : vertex(vertex), time(time) { }
    st_location(const st_location &ref) : vertex(ref.vertex), time(ref.time) { }
    bool operator<(const st_location &loc) const
    {
        return time < loc.time;
    }
};

unsigned int insertion_sort(unsigned int **next_waypoint, st_location *queue, unsigned int size, st_location loc)
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

using graph_type = std::vector<std::vector<unsigned int> >;
std::vector<unsigned int> CFreeAgent::spacetime_dijkstra(const std::vector<std::vector<unsigned int> > &other_paths, const graph_type &graph, unsigned int size, const std::vector<unsigned int> &waypoints)
{
	std::cout << "WAYPOINTS:";
	for(int i=0; i<waypoints.size(); i++)
	{
		std::cout << " " << waypoints[i];
	}
	std::cout << std::endl;
	
    const unsigned int WHITE=0, GRAY=1, BLACK=2, MAX_TIME=512U;
    unsigned int source = waypoints.front();
    auto it_waypoints = waypoints.begin()+1;
    
    std::vector<unsigned int> path;

    unsigned int ***prev_paths = new unsigned int**[size];
    unsigned int **path_sizes = new unsigned int*[size];
    for(unsigned int i=0; i<size; i++)
    {
        prev_paths[i] = new unsigned int*[MAX_TIME];
        path_sizes[i] = new unsigned int[MAX_TIME];
        for(unsigned int j=0; j<MAX_TIME; j++)
        {
            prev_paths[i][j] = new unsigned int[MAX_TIME];
            path_sizes[i][j] = 0;
        }
    }
    prev_paths[source][0][0] = source;
    path_sizes[source][0] = 1;

    unsigned int **visited = new unsigned int*[size];
	unsigned int **next_waypoint = new unsigned int*[size];
    for(unsigned int i=0; i<size; i++)
    {
        visited[i] = new unsigned int[MAX_TIME];
		next_waypoint[i] = new unsigned int[MAX_TIME];
        for(unsigned int j=0; j<MAX_TIME; j++)
        {
            visited[i][j] = WHITE;
			next_waypoint[i][j] = 1;
        }
    }

    st_location st(source, 0);
    st_location *queue = new st_location[MAX_TIME];
    queue[0] = st;
    int queue_size = 1;

    while(queue_size > 0)
    {
        st_location current_st = queue[--queue_size];
        unsigned int u = current_st.vertex;
        unsigned int time = current_st.time;
		unsigned int next_next_waypoint = next_waypoint[u][time];
        
        visited[u][time] = BLACK;

		if (u == waypoints[next_waypoint[u][time]])
        // if (u == *it_waypoints)
        {
			if (next_waypoint[u][time]+1 == waypoints.size())
            // if (it_waypoints+1 == waypoints.end())
            {
				std::vector<unsigned int> result = std::vector<unsigned int>(prev_paths[u][time], prev_paths[u][time] + path_sizes[u][time]);
				
				// pulizia heap
				for(unsigned int i=0; i<size; i++)
				{
					for(unsigned int j=0; j<MAX_TIME; j++)
					{
						delete[] prev_paths[i][j];
					}

					delete[] prev_paths[i];
					delete[] path_sizes[i];
					delete[] visited[i];
					delete[] next_waypoint[i];
				}
				delete[] prev_paths;
				delete[] path_sizes;
				delete[] visited;
				delete[] next_waypoint;
				delete[] queue;

                return result;
            }
            // else
            // {
            //     for(int i=0; i<queue_size; i++)
            //     {
            //         st_location temp_st = queue[i];
            //         unsigned int temp_v = temp_st.vertex;
            //         unsigned int temp_t = temp_st.time;
            //         visited[temp_v][temp_t] = WHITE;
            //     }
            //     queue_size = 0;
            // }

            // it_waypoints++;
			next_next_waypoint++;
        }

        // considero i vicini
        for(auto it = graph[u].begin(); it != graph[u].end(); it++)
        {
            const unsigned int v = *it;
            const unsigned int next_time = time + 1;

            if (visited[v][next_time] == WHITE)
            {
				bool good = true;
				for(int i=0; i<TEAM_SIZE; i++)
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
							if (other_paths[i][next_time] == u
									&& other_paths[i][time] == v)
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
					next_waypoint[v][next_time] = next_next_waypoint;
					queue_size = insertion_sort(next_waypoint, queue, queue_size, next_st);
					visited[v][next_time] = GRAY;

					unsigned int psize = path_sizes[u][time];
					for(unsigned int i=0; i<psize; i++)
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
			for(int i=0; i<TEAM_SIZE; i++)
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

			if(good)
			{
				st_location next_st(u, next_time);
				next_waypoint[u][next_time] = next_next_waypoint;
				queue_size = insertion_sort(next_waypoint, queue, queue_size, next_st);
				visited[u][next_time] = GRAY;

				unsigned int psize = path_sizes[u][time];
				for(unsigned int i=0; i<psize; i++)
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

std::vector<uint> CFreeAgent::token_dijkstra(const std::vector<uint> &waypoints, std::vector<logistic_sim::Path> &other_paths)
{
	// adatto struttura dijkstra
	std::vector<std::vector<unsigned int> > graph(dimension);
	for(int i=0; i<dimension; i++)
	{
		for(int j=0; j<vertex_web[i].num_neigh; j++)
		{
			graph[i].push_back(vertex_web[i].id_neigh[j]);
		}
	}

	std::vector<std::vector<uint> > simple_paths(TEAM_SIZE);
	for(int i=0; i<TEAM_SIZE; i++)
	{
		simple_paths[i] = other_paths[i].PATH;
	}
	c_print("Fine del space_dijkstra", green,P);
	return spacetime_dijkstra(simple_paths, graph, dimension, waypoints);
}