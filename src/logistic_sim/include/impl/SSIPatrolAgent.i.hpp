#pragma once

namespace ssipatrolagent {

using namespace std;

void SSIPatrolAgent::init(int argc, char** argv) {
        
    Agent::init(argc,argv);

    //initialize structures
    next_vertex = -1; 
    next_next_vertex = -1;

    taskRequests = new int[dimension];  
    tasks = new bool[dimension];    
    bids = new bid_tuple[dimension]; 
    global_instantaneous_idleness = new double[dimension];
    selected_vertices = new bool[dimension];
    bid_tuple noBid = {BIG_NUMBER,-1}; 
    for(size_t i=0; i<dimension; i++) {
        taskRequests[i] = 0;
        tasks[i] = false;
        selected_vertices[i] = false;
        bids[i] = noBid;
        global_instantaneous_idleness[i]=dimension*2;  // start with a high value (not too high) 
    }
    nactivetasks=0;

    last_update_idl = ros::Time::now().toSec();

    first_vertex = true;	

    //initialize parameters
    timeout = cf.getDParam("timeout");
    theta_idl = cf.getDParam("theta_idleness");
    theta_cost = cf.getDParam("theta_navigation");
    theta_hop = cf.getDParam("theta_hop");	
    threshold = cf.getDParam("threshold");			
    hist = cf.getDParam("hist");

    std::stringstream paramss;
    paramss << timeout << "," << theta_idl << "," << theta_cost << "," << theta_hop << "," << threshold << "," << hist;

    ros::param::set("/algorithm_params",paramss.str());
}


void SSIPatrolAgent::onGoalComplete()
{
    // printf("DTAP onGoalComplete!!!\n");

    // std::cout << env_in << bold << red << "\t < DTAP onGoalComplete! >" << env_out<<"\n";

    if (first_vertex)
    {
        printf("computing next vertex FOR THE FIRST TIME:\n current_vertex = %d, next_vertex=%d, next_next_vertex=%d %s",current_vertex, next_vertex,next_next_vertex);
        next_vertex = compute_next_vertex(current_vertex);
        printf("DONE: current_vertex = %d, next_vertex=%d, next_next_vertex=%d\n",current_vertex, next_vertex,next_next_vertex);		
        first_vertex = false;
    } 
    else 
    {
        printf("updating next vertex :\n current_vertex = %d, next_vertex=%d, next_next_vertex=%d\n",current_vertex, next_vertex,next_next_vertex);
        
        //Update Idleness Table:
        update_global_idleness();
        //update current vertex
        current_vertex = next_vertex;
        //update next vertex based on previous decision
        next_vertex = next_next_vertex;
        //update global idleness of next vertex to avoid conflicts
        
            if (next_vertex>=0 && next_vertex< dimension){
                pthread_mutex_lock(&lock);
                global_instantaneous_idleness[next_vertex] = 0.0;   
                pthread_mutex_unlock(&lock);
            }
        printf("DONE: current_vertex = %d, next_vertex=%d, next_next_vertex=%d\n",current_vertex, next_vertex,next_next_vertex);		
    }

    /** SEND GOAL (REACHED) AND INTENTION **/
    send_goal_reached(); // Send TARGET to monitor
    send_results();  // Algorithm specific function
    
    //Send the goal to the robot (Global Map)
    ROS_INFO("Sending goal - Vertex %d (%f,%f)\n", next_vertex, vertex_web[next_vertex].x, vertex_web[next_vertex].y);
    //sendGoal(vertex_web[next_vertex].x, vertex_web[next_vertex].y);  
    sendGoal(next_vertex);  // send to move_base

    goal_complete = false; 

    //compute next next vertex
    printf("computing next_next_vertex :\n current_vertex = %d, next_vertex=%d, next_next_vertex=%d\n",current_vertex, next_vertex,next_next_vertex);
    
    next_next_vertex = compute_next_vertex(next_vertex); 
	   
    printf("<<< DONE Computed next vertices: current_vertex = %d, next_vertex=%d, next_next_vertex=%d >>>\n",current_vertex, next_vertex,next_next_vertex);		

}


double SSIPatrolAgent::utility(int cv,int nv) {
    double idl = global_instantaneous_idleness[nv];
    
    size_t hops = compute_hops(cv,nv);
    double U = theta_idl * idl + theta_hop * hops * dimension;

    // double cost = compute_cost(cv,nv); ????  1 hop = 5 m
    // double U = theta_idl * idl + theta_navigation * cost;
#if DEBUG_PRINT
    if (U>-1000)
        printf("  HOPSUtil:: node: %d --> U[%d] ( %.1f, %zu ) = %.1f\n",cv,nv,idl,hops,U);
#endif
    return U;
}

int SSIPatrolAgent::compute_next_vertex() {
	compute_next_vertex(current_vertex);
}

} // namespace ssipatrolagent

