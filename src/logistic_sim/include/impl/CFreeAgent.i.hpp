#pragma once

using namespace cfreeagent;

int taken_missions_count(const logistic_sim::Token &token)
{
	int count = 0;
	for (int i = 0; i < token.TAKEN_MISSION.size(); i++)
	{
		if (token.TAKEN_MISSION[i] != -1)
			count++;
	}
	return count;
}

bool CFreeAgent::token_check_pt(std::vector<uint> &my_path, std::vector<logistic_sim::Path> &other_paths, uint CHECK_ID,
								int *id_vertex_stuck)
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

unsigned int insertion_sort(st_location *queue, unsigned int size, st_location loc,
							bool (*cmp_function)(const st_location&, const st_location&) = nullptr)
{
	//   auto cmp_function = [&](const st_location &lhs, const st_location &rhs) {
	// 	if (lhs.time < rhs.time)
	// 	{
	// 	  return true;
	// 	}
	// 	else if (lhs.time == rhs.time)
	// 	{
	// 	  if (next_waypoint[lhs.vertex][lhs.time] > next_waypoint[rhs.vertex][rhs.time])
	// 	  {
	// 		return true;
	// 	  }
	// 	}
	// 	return false;
	//   };

	int i;
	for (i = size - 1; i >= 0; i--)
	{
		if (cmp_function != nullptr)
		{
			if (cmp_function(loc, queue[i]))
				break;
		}
		else if (loc < queue[i])
		{
			break;
		}
		queue[i + 1] = queue[i];
	}
	queue[i + 1] = loc;

	return size + 1;
}

using graph_type = std::vector<std::vector<unsigned int>>;
std::vector<unsigned int> CFreeAgent::spacetime_dijkstra(const std::vector<std::vector<unsigned int>> &other_paths,
														 const graph_type &graph, unsigned int size,
														 const std::vector<unsigned int> &waypoints,
														 const std::vector<bool> &still_robots)
{
	std::cout << "WAYPOINTS:";
	for (int i = 0; i < waypoints.size(); i++)
	{
		std::cout << " " << waypoints[i];
	}
	std::cout << std::endl;

	//   const unsigned int WHITE = 0, GRAY = 1, BLACK = 2, MAX_TIME = 128U, MAX_WAYPOINTS = 64U;
	unsigned int source = waypoints.front();
	auto it_waypoints = waypoints.begin() + 1;

	//   std::vector<unsigned int> path;

	//   unsigned int ****prev_paths = new unsigned int ***[size];
	//   unsigned int ***path_sizes = new unsigned int **[size];
	for (unsigned int i = 0; i < size; i++)
	{
		// prev_paths[i] = new unsigned int **[MAX_TIME];
		// path_sizes[i] = new unsigned int *[MAX_TIME];
		for (unsigned int j = 0; j < MAX_TIME; j++)
		{
			//   prev_paths[i][j] = new unsigned int *[MAX_WAYPOINTS];
			//   path_sizes[i][j] = new unsigned int[MAX_WAYPOINTS];
			for (unsigned int k = 0; k < MAX_WAYPOINTS; k++)
			{
				// prev_paths[i][j][k] = new unsigned int[MAX_TIME];
				path_sizes[i][j][k] = 0;
			}
		}
	}
	prev_paths[source][0][1][0] = source;
	path_sizes[source][0][1] = 1;

	//   unsigned int ***visited = new unsigned int **[size];
	for (unsigned int i = 0; i < size; i++)
	{
		// visited[i] = new unsigned int *[MAX_TIME];
		for (unsigned int j = 0; j < MAX_TIME; j++)
		{
			//   visited[i][j] = new unsigned int[MAX_WAYPOINTS];
			for (unsigned int k = 0; k < MAX_WAYPOINTS; k++)
			{
				visited[i][j][k] = WHITE;
			}
		}
	}

	//   st_location *queue = new st_location[MAX_TIME * MAX_TIME];
	for (unsigned int i = 0; i < MAX_TIME; i++)
	{
		queue[i] = st_location();
	}
	st_location st(source, 0, 1);
	queue[0] = st;
	int queue_size = 1;

	uint max_path_size = 0;
	for (int i = 0; i < TEAM_SIZE; i++)
	{
		if (i != ID_ROBOT && other_paths[i].size() > max_path_size)
		{
			max_path_size = other_paths[i].size();
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
		// if (u == *it_waypoints)
		{
			if (current_waypoint + 1 == waypoints.size())
			// if (it_waypoints+1 == waypoints.end())
			{
				if (time + 1 >= max_path_size)
				{
					std::vector<unsigned int> result =
						std::vector<unsigned int>(prev_paths[u][time][current_waypoint], prev_paths[u][time][current_waypoint] + path_sizes[u][time][current_waypoint]);

					// pulizia heap
					//   for (unsigned int i = 0; i < size; i++)
					//   {
					// 	for (unsigned int j = 0; j < MAX_TIME; j++)
					// 	{
					// 		for(unsigned int k=0; k<MAX_TIME; k++)
					// 		{
					// 			delete[] prev_paths[i][k][j];
					// 		}
					// 	  delete[] prev_paths[i][j];
					// 	  delete[] path_sizes[i][j];
					//     delete[] visited[i][j];
					// 	}

					// 	delete[] prev_paths[i];
					// 	delete[] path_sizes[i];
					// 	delete[] visited[i];
					//   }
					//   delete[] prev_paths;
					//   delete[] path_sizes;
					//   delete[] visited;
					//   delete[] queue;

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
				// if (next_waypoint[u][time] + 1 < waypoints.size())
				// {
				//   for (int i = 0; i < queue_size; i++)
				//   {
				// 	st_location temp_st = queue[i];
				// 	unsigned int temp_v = temp_st.vertex;
				// 	unsigned int temp_t = temp_st.time;
				// 	visited[temp_v][temp_t] = WHITE;
				//   }
				//   queue_size = 0;
				// }

				// if (next_waypoint[u][time] + 1 == waypoints.size() && time + 1 < max_path_size)
				// {
				//   next_next_waypoint--;
				// }
			}

			// it_waypoints++;
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
					if (next_time < other_paths[i].size())
					{
						if (other_paths[i][next_time] == u)
						{
							std::cout << "can't stand in " << u << " at time " << time << std::endl;
							good = false;
							break;
						}
					}
					else if (!other_paths[i].empty() /* && still_robots[i] */)
					{
						if (other_paths[i].back() == u)
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
						if (next_time < other_paths[i].size())
						{
							if (other_paths[i][next_time] == v)
							{
								std::cout << "can't go in " << v << " at time " << time << std::endl;
								good = false;
								break;
							}
							if (other_paths[i][next_time] == u && other_paths[i][time] == v)
							{
								std::cout << "can't go in " << v << " at time " << time << std::endl;
								good = false;
								break;
							}
						}
						else if (!other_paths[i].empty() /* && still_robots[i] */)
						{
							if (other_paths[i].back() == v)
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

std::vector<uint> CFreeAgent::token_dijkstra(const std::vector<uint> &waypoints,
											 std::vector<logistic_sim::Path> &other_paths,
											 const std::vector<bool> &still_robots)
{
	// adatto struttura dijkstra
	std::vector<std::vector<unsigned int>> graph(dimension);
	for (int i = 0; i < dimension; i++)
	{
		for (int j = 0; j < vertex_web[i].num_neigh; j++)
		{
			graph[i].push_back(vertex_web[i].id_neigh[j]);
		}
	}

	std::vector<std::vector<uint>> simple_paths(TEAM_SIZE);
	for (int i = 0; i < TEAM_SIZE; i++)
	{
		simple_paths[i] = other_paths[i].PATH;
	}
	
	if (!still_robots.empty())
	{
		return spacetime_dijkstra(simple_paths, graph, dimension, waypoints, still_robots);
	}
	else
	{
		return spacetime_dijkstra(simple_paths, graph, dimension, waypoints, std::vector<bool>(TEAM_SIZE, false));
	}
}