#pragma once

using namespace cfreeagent;

bool CFreeAgent::token_check_pt(std::vector<uint> &my_path, std::vector<logistic_sim::Path> &other_paths, uint ID_ROBOT, int *id_vertex_stuck)
{
	bool status = true;
	// controllo che la posizione vicina sia raggiungibile in questo turno
	for (int j = 0; j < my_path.size(); j++)
	{

		for (int i = 0; i < other_paths.size(); i++)
		{
			if (i != ID_ROBOT)
			{
				const logistic_sim::Path &path = other_paths[i];
				if (j + 1 < path.PATH.size())
				{
					int other_pos = path.PATH[j];
					int other_next_pos = path.PATH[j + 1];
					if (my_path[j + 1] == other_next_pos)
					{
						*id_vertex_stuck = my_path[j + 1];
						status = false;
					}
					if (my_path[j + 1] == other_pos && my_path[j] == other_next_pos)
					{
						*id_vertex_stuck = my_path[j + 1];
						status = false;
					}
				}
			}
		}
	}
	return status;
}

void CFreeAgent::token_dijkstra(const std::vector<uint> &waypoints, std::vector<logistic_sim::Path> &other_paths)
{
	auto waypoints_it = waypoints.begin() + 1;
	uint source = waypoints.front();
	uint destination = *waypoints_it;
	uint i, j, k, x;
	int id_next_vertex = -1;
	int id_source;

	uint elem_s_path;
	s_path *tab_dijkstra = new s_path[dimension];

	// Initialization:
	for (i = 0; i < dimension; i++)
	{
		tab_dijkstra[i].id = vertex_web[i].id;
		tab_dijkstra[i].elem_path = 0;
		tab_dijkstra[i].visit = false;

		if (vertex_web[i].id == source)
		{
			tab_dijkstra[i].dist = 0;
			tab_dijkstra[i].path[0] = source;
			tab_dijkstra[i].elem_path++;
			id_next_vertex = i;
			id_source = i;
		}
		else
		{
			tab_dijkstra[i].dist = INT_MAX;
		}
	}

	int next_vertex = source;
	int minim_dist;
	bool cont;

	std::vector<uint> steps(dimension, 0);

	while (true)
	{
		//	printf("next_vertex = %i\n", next_vertex);

		if (next_vertex == destination)
		{
			if (waypoints_it + 1 != waypoints.end())
			{
				// resetto i percorsi degli altri nodi e li setto come non visitati
				for (uint i = 0; i < dimension; i++)
				{
					if (i != id_next_vertex)
					{
						tab_dijkstra[i].visit = false;
						tab_dijkstra[i].elem_path = 0;
						tab_dijkstra[i].dist = INT_MAX;
					}
				}

				waypoints_it++;
				source = destination;
				// cerco id vertice sorgente
				for (int i = 0; i < dimension; i++)
				{
					if (tab_dijkstra[i].id == source)
					{
						id_source = i;
						break;
					}
				}
				destination = *waypoints_it;
			}
			else
			{
				break;
			}
		}

		tab_dijkstra[id_next_vertex].visit = true;

		/* id_next_vertex has INDEX of next_vertex in tab_dijkstra */
		/* j has INDEX of the neighbor in tab_disjkstra */
		/* k has index of the neighbor in the neighbor table of next_vertex */

		// Go to neihgobors;
		for (k = 0; k < vertex_web[next_vertex].num_neigh; k++)
		{
			cont = false;

			// condition - cannot have been visited:
			for (j = 0; j < dimension; j++)
			{
				// cerco l'id del vertice vicino per poter controllare in tabella se l'ho già visitato
				if (tab_dijkstra[j].id == vertex_web[next_vertex].id_neigh[k] && tab_dijkstra[j].visit == false)
				{
					cont = true;
					break;
				}
			}

			// controllo che la posizione vicina sia raggiungibile in questo turno
			for (int i = 0; i < other_paths.size() && cont; i++)
			{
				if (i != ID_ROBOT)
				{
					const logistic_sim::Path &path = other_paths[i];
					int dist = tab_dijkstra[id_next_vertex].elem_path - 1;
					if (dist < path.PATH.size())
					{
						int other_pos = path.PATH[dist];
						int other_next_pos = path.PATH[dist + 1];
						if (tab_dijkstra[j].id == other_next_pos)
						{
							//   c_print("[WARN ]\tAvoiding other robot", yellow, P);
							//   c_print("\tDist: ", dist, yellow, P);
							//   c_print("\tOther_pos: ", other_pos, "\tother_next_pos", other_next_pos, yellow, P);
							cont = false;
						}
						if (tab_dijkstra[j].id == other_pos && tab_dijkstra[id_next_vertex].id == other_next_pos)
						{
							//   c_print("[WARN ]\tAvoiding edge of other robot", yellow, P);
							//   c_print("\tDist: ", dist, yellow, P);
							//   c_print("\tOther_pos: ", other_pos, "\tother_next_pos", other_next_pos, yellow, P);
							cont = false;
						}
					}
				}
			}

			// se la posizione è raggiungibile controllo se mi da un percorso migliore
			if (cont)
			{
				// calculate distance from this neighbor:
				if (tab_dijkstra[id_next_vertex].dist + vertex_web[next_vertex].cost[k] < tab_dijkstra[j].dist)
				{
					// update distance to this vertex:
					tab_dijkstra[j].dist = tab_dijkstra[id_next_vertex].dist + vertex_web[next_vertex].cost[k];

					// update path (previous path + this one):
					for (x = 0; x < tab_dijkstra[id_next_vertex].elem_path; x++)
					{
						tab_dijkstra[j].path[x] = tab_dijkstra[id_next_vertex].path[x];
					}

					tab_dijkstra[j].path[tab_dijkstra[id_next_vertex].elem_path] = tab_dijkstra[j].id;
					tab_dijkstra[j].elem_path = tab_dijkstra[id_next_vertex].elem_path + 1;
				}
			}
		}

		minim_dist = INT_MAX;

		// decide next_vertex:
		for (i = 0; i < dimension; i++)
		{
			if (tab_dijkstra[i].dist < minim_dist && tab_dijkstra[i].visit == false)
			{
				minim_dist = tab_dijkstra[i].dist;
				next_vertex = tab_dijkstra[i].id;
				id_next_vertex = i;
			}
		}

		// non posso più andare avanti, provo ad attendere un turno(autoanello)
		if (minim_dist == INT_MAX)
		{
			// controllo di poter stare fermo
			bool good = true;
			for (int i = 0; i < other_paths.size(); i++)
			{
				if (i != ID_ROBOT)
				{
					const logistic_sim::Path &path = other_paths[i];
					int dist = tab_dijkstra[id_source].elem_path - 1;
					if (dist + 1 < path.PATH.size())
					{
						int other_next_pos = path.PATH[dist + 1];
						if (tab_dijkstra[id_source].id == other_next_pos)
						{
							//   c_print("[WARN ]\tAvoiding other robot", yellow, P);
							//   c_print("\tDist: ", dist, yellow, P);
							//   c_print("\tOther_pos: ", other_pos, "\tother_next_pos", other_next_pos, yellow, P);
							good = false;
							break;
						}
					}
				}
			}

			if (good)
			{
				std::cout << "sto fermo in " << id_source << "\n";
				tab_dijkstra[id_source].path[tab_dijkstra[id_source].elem_path] = tab_dijkstra[id_source].id;
				tab_dijkstra[id_source].elem_path++;
				// resetto i percorsi degli altri nodi e li setto come non visitati
				for (uint i = 0; i < dimension; i++)
				{
					if (i != id_source)
					{
						tab_dijkstra[i].elem_path = 0;
						tab_dijkstra[i].visit = false;
						tab_dijkstra[i].dist = INT_MAX;
					}
				}
				// setto il vertice da cui proseguire
				id_next_vertex = id_source;
				next_vertex = source;
			}
			else
			{
				bool can_move = false;
				// se non posso stare fermo provo a spostarmi in un nodo libero adiacente
				for (uint neighbour : vertex_web[source].id_neigh)
				{
					// cerco l'id nella tabella dijkstra
					uint id_neighbour;
					for (int i = 0; i < dimension; i++)
					{
						if (tab_dijkstra[i].id == neighbour)
						{
							id_neighbour = i;
							break;
						}
					}

					// controllo nella tabella se sono riuscito a raggiungere il nodo
					if (tab_dijkstra[id_neighbour].elem_path == tab_dijkstra[id_source].elem_path + 1)
					{
						std::cout << "Non posso stare fermo in " << source << ", vado in " << neighbour << std::endl;
						// vado nel vicino e resetto tabella dijkstra
						can_move = true;
						id_source = id_next_vertex = id_neighbour;
						source = next_vertex = neighbour;
						for (int i = 0; i < dimension; i++)
						{
							tab_dijkstra[i].visit = false;
							if (i != id_neighbour)
							{
								tab_dijkstra[i].elem_path = 0;
								tab_dijkstra[i].dist = INT_MAX;
							}
						}
						break; // non devo controllare gli altri vicini
					}
				}

				if (!can_move)
				{
					throw std::string("Can't calculate this path!!!");
				}
			}
		}
	}

	// Save shortest_path & delete tab_dijkstra...
	elem_s_path = tab_dijkstra[id_next_vertex].elem_path; // id_next_vertex has the ID of the destination in tab_dijkstra

	for (i = 0; i < elem_s_path; i++)
	{
		int v = tab_dijkstra[id_next_vertex].path[i];
		other_paths[ID_ROBOT].PATH.push_back((uint)v);
	}
	std::cout << std::endl;

	delete[] tab_dijkstra;
}