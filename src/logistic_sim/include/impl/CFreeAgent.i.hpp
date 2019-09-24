#pragma once

using namespace cfreeagent;

void CFreeAgent::token_dijkstra(uint source, uint destination, std::vector<logistic_sim::Path> &other_paths){
  
  uint i,j,k,x;
  int id_next_vertex=-1;
  
  int shortest_path[dimension];
  uint elem_s_path;
  s_path *tab_dijkstra = new s_path[dimension];
  
  //Initialization:
  for(i=0; i<dimension; i++){
	
	tab_dijkstra[i].id = vertex_web[i].id;
	tab_dijkstra[i].elem_path = 0;
	tab_dijkstra[i].visit = false;
	
	if (vertex_web[i].id == source) {
	  tab_dijkstra[i].dist = 0;
	  tab_dijkstra[i].path[0] = source;
	  tab_dijkstra[i].elem_path++;
	  id_next_vertex = i;
	}else{
	  tab_dijkstra[i].dist = INT_MAX;
	}

  }  
  
  int next_vertex = source;
  int minim_dist;
  bool cont;
  
  std::vector<uint> steps(dimension, 0);

  while(true){	
	
//	printf("next_vertex = %i\n", next_vertex);
	
	if(next_vertex == destination){
	 break; 
	}
	
	tab_dijkstra[id_next_vertex].visit = true;
	
	/* id_next_vertex has INDEX of next_vertex in tab_dijkstra */
	/* j has INDEX of the neighbor in tab_disjkstra */
	/* k has index of the neighbor in the neighbor table of next_vertex */
	
	//Go to neihgobors;	
	for(k=0; k<vertex_web[next_vertex].num_neigh; k++){
	  
		cont = false;
		
		//condition - cannot have been visited:
		for(j=0; j<dimension; j++){
		  if(tab_dijkstra[j].id == vertex_web[next_vertex].id_neigh[k] && tab_dijkstra[j].visit == false){
			cont = true;
			break;
		  }		  
		}

        for(int i=0; i<other_paths.size(); i++)
        {
            if(i != ID_ROBOT)
            {
                const logistic_sim::Path &path = other_paths[i];
                int dist = tab_dijkstra[id_next_vertex].dist;
                if (dist < path.PATH.size())
                {
                    int other_pos = path.PATH[dist];
                    int other_next_pos = path.PATH[dist+1];
                    if (tab_dijkstra[j].id == other_next_pos)
                    {
                        c_print("[WARN ]\tAvoiding other robot", yellow, P);
                        cont = false;
                    }
                    if (tab_dijkstra[j].id == other_pos && tab_dijkstra[id_next_vertex].id == other_next_pos)
                    {
                        c_print("[WARN ]\tAvoiding edge of other robot", yellow, P);
                        cont = false;
                    }
                }
            }
        }
		
		if(cont){
		  
		  //calculate distance from this neighbor:
		  if( tab_dijkstra[id_next_vertex].dist + vertex_web[next_vertex].cost[k] < tab_dijkstra[j].dist){
			
			//update distance to this vertex:
			tab_dijkstra[j].dist = tab_dijkstra[id_next_vertex].dist + vertex_web[next_vertex].cost[k];
			
			//update path (previous path + this one):
			for (x=0; x<tab_dijkstra[id_next_vertex].elem_path; x++){			 
			  tab_dijkstra[j].path[x] = tab_dijkstra[id_next_vertex].path[x];			  
			}
			
			tab_dijkstra[j].path[tab_dijkstra[id_next_vertex].elem_path] = tab_dijkstra[j].id;
			tab_dijkstra[j].elem_path = tab_dijkstra[id_next_vertex].elem_path+1;

		  }	
		  
		}
		

 
	}
	
	minim_dist = INT_MAX;
	
	//decide next_vertex:
	for(i=0; i<dimension; i++){	  
	  
	  if(tab_dijkstra[i].dist < minim_dist && tab_dijkstra[i].visit == false){
		minim_dist = tab_dijkstra[i].dist;
		next_vertex = tab_dijkstra[i].id;
		id_next_vertex = i;
	  }	 
	  
	}
	
  }
  
  //Save shortest_path & delete tab_dijkstra... 
  elem_s_path = tab_dijkstra[id_next_vertex].elem_path; //id_next_vertex has the ID of the destination in tab_dijkstra

  for(i=0; i<elem_s_path; i++){	
	shortest_path[i] = tab_dijkstra[id_next_vertex].path[i];
    other_paths[ID_ROBOT].PATH.push_back(tab_dijkstra[id_next_vertex].path[i]);
  }
  
  delete [] tab_dijkstra;
  
}