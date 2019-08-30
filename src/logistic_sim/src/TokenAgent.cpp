#include "TokenAgent.hpp"


bool tokenagent::TokenAgent::check_interference_token(const logistic_sim::Token &token)
{
    int i;
    double dist_quad;

    if (ros::Time::now().toSec() - last_interference < 10) // seconds
        return false;                                      // false if within 10 seconds from the last one

    /* Poderei usar TEAMSIZE para afinar */
    // ID_ROBOT
    for (i = 0; i < TEAM_SIZE; i++)
    { //percorrer vizinhos (assim asseguro q cada interferencia é so encontrada 1 vez)
        
        if( i == ID_ROBOT)
            continue;

        dist_quad = (xPos[i] - xPos[ID_ROBOT]) * (xPos[i] - xPos[ID_ROBOT]) + (yPos[i] - yPos[ID_ROBOT]) * (yPos[i] - yPos[ID_ROBOT]);

        if (dist_quad <= INTERFERENCE_DISTANCE * INTERFERENCE_DISTANCE)
        { //robots are ... meter or less apart
            //          ROS_INFO("Feedback: Robots are close. INTERFERENCE! Dist_Quad = %f", dist_quad);
            double x_dst = vertex_web[current_mission.DSTS[0]].x;
            double y_dst = vertex_web[current_mission.DSTS[0]].y;
            double other_x_dst = vertex_web[token.CURR_DST[i]].x;
            double other_y_dst = vertex_web[token.CURR_DST[i]].y;
            double other_distance = (xPos[i] - x_dst) * (xPos[i] - x_dst) + (yPos[i] - y_dst) * (yPos[i] - y_dst);
            double my_distance = (xPos[ID_ROBOT] - other_x_dst) * (xPos[ID_ROBOT] - other_x_dst) + (yPos[ID_ROBOT] - other_y_dst) * (yPos[ID_ROBOT] - other_y_dst);
            // c_print("my_distance ", my_distance, "\tother_distance ", other_distance, yellow);
            last_interference = ros::Time::now().toSec();
            if (my_distance > other_distance)
            {
                return true;
            }
            else
            {
                c_print("Robot ", i, " dovrebbe essere in interferenza", red);
            }
            
        }
    }
    return false;
}

// la lasciamo vuota
void tokenagent::TokenAgent::goalFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback)
{
    send_positions();  
}


int main(int argc, char *argv[])
{
    tokenagent::TokenAgent TPA;
    TPA.init(argc, argv);
    c_print("@ Inizializzazione finita!",green);
    sleep(3);
    TPA.run();
    return 0;
}