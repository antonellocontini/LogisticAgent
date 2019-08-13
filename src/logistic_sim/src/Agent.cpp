// specialized method (virtual)
#include "Agent.hpp"

namespace agent{

void Agent::init(int argc, char** argv) {
        /*
            argv[0]=/.../patrolling_sim/bin/GBS
            argv[1]=__name:=XXXXXX
            argv[2]=grid
            argv[3]=ID_ROBOT
            argv[4]=CAPACITY
            argv[5]=TEAMSIZE
        */
    
    srand ( time(NULL) );
    
    //More than One robot (ID between 0 and 99)
    if ( atoi(argv[3])>NUM_MAX_ROBOTS || atoi(argv[3])<-1 ){
        ROS_INFO("The Robot's ID must be an integer number between 0 an 99"); //max 100 robots 
        return;
    }else{
        ID_ROBOT = atoi(argv[3]); 
        printf("ID_ROBOT = %d\n",ID_ROBOT); //-1 for 1 robot without prefix (robot_0)
    }

    CAPACITY = atoi(argv[4]);
    TEAM_SIZE = atoi(argv[5]);

    c_print("C and TS:", CAPACITY, TEAM_SIZE, green);
    
    /** D.Portugal: needed in case you "rosrun" from another folder **/     
    chdir(PS_path.c_str());
                
    std:string mapname = string(argv[2]);
    std::string graph_file = "maps/"+mapname+"/"+mapname+".graph";
    
    //Check Graph Dimension:
    dimension = GetGraphDimension(graph_file.c_str());
    
    //Create Structure to save the Graph Info;
    vertex_web = new vertex[dimension];
    
    //Get the Graph info from the Graph File
    GetGraphInfo(vertex_web, dimension, graph_file.c_str());
    
    
    uint nedges = GetNumberEdges(vertex_web,dimension);
    
    printf("Loaded graph %s with %d nodes and %d edges\n",mapname.c_str(),dimension,nedges);

#if 0
    /* Output Graph Data */   
    for (auto i=0;i<dimension;i++){
        printf ("ID= %u\n", vertex_web[i].id);
        printf ("X= %f, Y= %f\n", vertex_web[i].x, vertex_web[i].y);
        printf ("#Neigh= %u\n", vertex_web[i].num_neigh);
        
        for (auto j=0;j<vertex_web[i].num_neigh; j++){
        printf("\tID = %u, DIR = %s, COST = %u\n", vertex_web[i].id_neigh[j], vertex_web[i].dir[j], vertex_web[i].cost[j]);
        }
        
        printf("\n");   
    }
#endif

    /* Define Starting Vertex/Position (Launch File Parameters) */

    ros::init(argc, argv, "patrol_agent");  // will be replaced by __name:=XXXXXX
    ros::NodeHandle nh;
    
    // wait a random time (avoid conflicts with other robots starting at the same time...)
    double r = 3.0 * ((rand() % 1000)/1000.0);
    ros::Duration wait(r); // seconds
    wait.sleep();
    
    double initial_x, initial_y;
    std::vector<double> list;
    nh.getParam("initial_pos", list);

    // list.push_back(8.1);
    // list.push_back(4.39);
    
    if (list.empty()){
     ROS_ERROR("No initial positions given: check \"initial_pos\" parameter.");
     ros::shutdown();
     exit(-1);
    }

       
    int value = ID_ROBOT;
    if (value == -1){value = 0;}
    
    initial_x = list[2*value];
    initial_y = list[2*value+1];
    
    //   printf("initial position: x = %f, y = %f\n", initial_x, initial_y);
    current_vertex = IdentifyVertex(vertex_web, dimension, initial_x, initial_y);
    //   printf("initial vertex = %d\n\n",current_vertex);  
    
    
    //instantaneous idleness and last visit initialized with zeros:
    last_visit = new double[dimension];
    for(size_t i=0; i<dimension; i++){ 
        last_visit[i]= 0.0; 
        
        if (i==current_vertex){
            last_visit[i]= 0.1; //Avoids getting back at the initial vertex
        }
        //ROS_INFO("last_visit[%d]=%f", i, last_visit[i]);
    }
        
    //Publicar dados de "odom" para nó de posições
    positions_pub = nh.advertise<nav_msgs::Odometry>("positions", 1); //only concerned about the most recent
        
    //Subscrever posições de outros robots
    positions_sub = nh.subscribe<nav_msgs::Odometry>("positions", 10, boost::bind(&Agent::positionsCB, this, _1));  
    
    char string1[40];
    char string2[40];
    
    if(ID_ROBOT==-1){ 
        strcpy (string1,"odom"); //string = "odom"
        strcpy (string2,"cmd_vel"); //string = "cmd_vel"
        // TEAM_SIZE = 1;
    }else{ 
        sprintf(string1,"robot_%d/odom",ID_ROBOT);
        sprintf(string2,"robot_%d/cmd_vel",ID_ROBOT);
        // TEAM_SIZE = ID_ROBOT + 1;
    }   

    /* Set up listener for global coordinates of robots */
    listener = new tf::TransformListener();

    //Cmd_vel to backup:
    cmd_vel_pub  = nh.advertise<geometry_msgs::Twist>(string2, 1);
    
    //Subscrever para obter dados de "odom" do robot corrente
    odom_sub = nh.subscribe<nav_msgs::Odometry>(string1, 1, boost::bind(&Agent::odomCB, this, _1)); //size of the buffer = 1 (?)
    
    ros::spinOnce(); 
    
    //Publicar dados para "results"
    results_pub = nh.advertise<std_msgs::Int16MultiArray>("results", 100);
    // results_sub = nh.subscribe("results", 10, resultsCB); //Subscrever "results" vindo dos robots
    results_sub = nh.subscribe<std_msgs::Int16MultiArray>("results", 100, boost::bind(&Agent::resultsCB, this, _1) ); //Subscrever "results" vindo dos robots
    // last time comm delay has been applied
    last_communication_delay_time = ros::Time::now().toSec();   

    readParams();

    c_print("Fine inizializzazione e lettura dei parametri", green,P);
}

int Agent::compute_next_vertex()
{
    int vertex = 0;

    if (path.size() -1 == id_vertex)
    {
        vertex = path[id_vertex];
        id_vertex = 0;
        // need_token_mission = true;
    }
    else
    {
        vertex = path[id_vertex];
        id_vertex++;
    }
    
    return vertex;
}


void Agent::onGoalComplete()
{
    if(next_vertex>-1) {
        //Update Idleness Table:
        // update_idleness();
        current_vertex = next_vertex;       
    }
    
    //devolver proximo vertex tendo em conta apenas as idlenesses;
    next_vertex = compute_next_vertex();
    //printf("Move Robot to Vertex %d (%f,%f)\n", next_vertex, vertex_web[next_vertex].x, vertex_web[next_vertex].y);

    /** SEND GOAL (REACHED) AND INTENTION **/
    send_goal_reached(); // Send TARGET to monitor
    send_results();  // Algorithm specific function

    //Send the goal to the robot (Global Map)
    ROS_INFO("Sending goal - Vertex %d (%f,%f)\n", next_vertex, vertex_web[next_vertex].x, vertex_web[next_vertex].y);
    //sendGoal(vertex_web[next_vertex].x, vertex_web[next_vertex].y);  
    sendGoal(next_vertex);  // send to move_base
    
    goal_complete = false;    
}

void Agent::processEvents() {
    
}

void Agent::send_results() { 

}


void Agent::receive_results() {

}


void Agent::run() {
    
    // get ready
    ready();

    c_print("Ready",green,P);

    //initially clear the costmap (to make sure the robot is not trapped):
    // std_srvs::Empty srv;
    // std::string mb_string;
     
    //  if (ID_ROBOT>-1){
    //          std::ostringstream id_string;
    //          id_string << ID_ROBOT;
    //          mb_string = "robot_" + id_string.str() + "/";
    // }
    // mb_string += "move_base/clear_costmaps";
    
    // if (ros::service::call(mb_string.c_str(), srv)){
    // //if (ros::service::call("move_base/clear_costmaps", srv)){
    //     ROS_INFO("Costmap correctly cleared before patrolling task.");
    // }else{
    //     ROS_WARN("Was not able to clear costmap (%s) before patrolling...", mb_string.c_str());
    // }
    
    // Asynch spinner (non-blocking)
    ros::AsyncSpinner spinner(2); // Use n threads
    spinner.start();
//     ros::waitForShutdown();

    /* Run Algorithm */ 
    
    ros::Rate loop_rate(30); //0.033 seconds or 30Hz

    c_print("While ros::ok()",yellow,P);
    
    while(ros::ok()){
        
        if (goal_complete) {
            onGoalComplete();  // can be redefined
            resend_goal_count=0;
        }
        else { // goal not complete (active)
            if (interference) {
                do_interference_behavior();
            }       
            
            if (ResendGoal) {
                //Send the goal to the robot (Global Map)
                if (resend_goal_count<3) {
                    resend_goal_count++;
                    ROS_INFO("Re-Sending goal (%d) - Vertex %d (%f,%f)", resend_goal_count, next_vertex, vertex_web[next_vertex].x, vertex_web[next_vertex].y);
                    sendGoal(next_vertex);
                }
                else {
                    resend_goal_count=0;
                    onGoalNotComplete();
                }
                ResendGoal = false; //para nao voltar a entrar (envia goal so uma vez)
            }
            
            processEvents();
            
            if (end_simulation) {
                return;
            }   
        
        } // if (goal_complete)
        
		loop_rate.sleep(); 

    } // while ros.ok    
}

} // namespace agent