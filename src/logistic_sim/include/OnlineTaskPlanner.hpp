#pragma once

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <boost/thread.hpp>
#include "TaskPlanner.hpp"
#include<chrono>

namespace onlinetaskplanner
{

using t_coalition = std::pair<std::vector<logistic_sim::Mission>, logistic_sim::Mission>;

struct less_V
{
    inline bool operator()(const t_coalition &A, const t_coalition &B)
    {
        return A.second.V < B.second.V;
    }
};

void print_coalition(const t_coalition &coalition)
{
    auto tasks = coalition.first;
    auto mission = coalition.second;
    c_print("Mission id: ", mission.ID, green, P);
    // std::cout << "pd: " << mission.PATH_DISTANCE << "\n";
    std::cout << "td: " << mission.TOT_DEMAND << "\n";
    // std::cout << " V: " << mission.V << "\n";
    // auto size_dsts = mission.DSTS.size();
    // std::cout << "dsts"
    //           << "\n";
    // for (int i = 0; i < size_dsts; i++)
    // {
    //     std::cout << mission.DSTS[i] << " ";
    // }
    // std::cout << "\n";

    auto size_boh = mission.DEMANDS.size();
    std::cout << "demands"
              << "\n";
    for (int i = 0; i < size_boh; i++)
    {
        std::cout << mission.DEMANDS[i] << " ";
    }
    std::cout << "\n";

    // auto size_route = mission.ROUTE.size();
    // std::cout << "route"
    //           << "\n";
    // for (int i = 0; i < size_route; i++)
    // {
    // std::cout << mission.ROUTE[i] << " ";
    // }
    // std::cout << "\n";

    // c_print("tasks", magenta, P);

    auto size_tasks = tasks.size();

    for (auto i = 0; i < size_tasks; i++)
    {
        auto t = tasks[i];
        c_print("task id: ", t.ID, magenta, P);
        //     std::cout << "pd: " << t.PATH_DISTANCE << "\n";
        std::cout << "td: " << t.TOT_DEMAND << "\n";
        //     std::cout << " V: " << t.V << "\n";
            auto size_dsts = t.DSTS.size();
            std::cout << "dsts"
                      << "\n";
            for (int i = 0; i < size_dsts; i++)
            {
                std::cout << t.DSTS[i] << " ";
            }
            std::cout << "\n";

        auto size_boh = t.DEMANDS.size();
        std::cout << "demands"
                  << "\n";
        for (int i = 0; i < size_boh; i++)
        {
            std::cout << t.DEMANDS[i] << " ";
        }
        std::cout << "\n";

        //     auto size_route = t.ROUTE.size();
        //     std::cout << "route"
        //               << "\n";
        //     for (int i = 0; i < size_route; i++)
        //     {
        //         std::cout << t.ROUTE[i] << " ";
        //     }
        //     std::cout << "\n";
    }

    c_print("fine mission id: ", mission.ID, red, P);
}


const std::string PS_path = ros::package::getPath("logistic_sim");

class OnlineTaskPlanner : public taskplanner::TaskPlanner
{
public:
    OnlineTaskPlanner(ros::NodeHandle &nh_, const std::string &name = "OnlineTaskPlanner");
    // void init(int argc, char **argv);
    void run();
    void token_callback(const logistic_sim::TokenConstPtr &msg) override;
    std::vector<logistic_sim::Mission> set_partition(const std::vector<logistic_sim::Mission> &ts);
    // definito per come ros vuole il tipo della funzione, chiamo quella della base class
    bool robot_ready(logistic_sim::RobotReady::Request &req,
                     logistic_sim::RobotReady::Response &res)
    {
        return TaskPlanner::robot_ready(req, res);
    }
    bool check_paths_conflicts(const std::vector<logistic_sim::Path> &paths, bool print = true);

    // queste due funzioni scrivono/leggono una versione semplificata della struttura mission
    // in particolare andrebbero usate solo per missioni con singola destinazione
    // inoltre identificano la destinazione non con il vertice nel grafo ma con un indice
    // per garantire compatibilità con mappe diverse
    // void write_simple_missions(std::ostream &os, const std::vector<logistic_sim::Mission> &mission);
    // std::vector<logistic_sim::Mission> read_simple_missions(std::istream &is);
    void allocate_memory() override;

protected:
    bool offline_mode = false;
    int window_size = 11;
    std::list<std::vector<logistic_sim::Mission>> mission_windows;
    boost::mutex window_mutex;
    ros::Time last_goal_time;
    ros::Duration shutdown_timeout = ros::Duration(5 * 60.0), shutdown_warning = ros::Duration(4 * 60.0);
    bool warning_occured = false;
    logistic_sim::Token::_GOAL_STATUS_type last_goal_status;

    std::vector<ros::Subscriber> real_pos_sub, amcl_pos_sub;
    std::vector<nav_msgs::Odometry> last_real_pos;
    std::vector<geometry_msgs::PoseWithCovarianceStamped> last_amcl_pos;
    std::vector<std::vector<double>> robot_pos_errors;

    // callback per leggere posizioni reali e misurate dei robot
    void real_pos_callback(const nav_msgs::OdometryConstPtr &msg, int id_robot);
    void amcl_pos_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg, int id_robot);

    bool allocation_phase = false;
    std::chrono::time_point<std::chrono::system_clock> end = std::chrono::system_clock::now();
    std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();
};

} // namespace onlinetaskplanner
