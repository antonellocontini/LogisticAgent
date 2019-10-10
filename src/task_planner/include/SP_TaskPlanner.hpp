#pragma once
#include "TaskPlanner.hpp"

namespace sp_taskplanner
{
using namespace taskplanner;

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


// un taskplanner che genera missioni per la casistica
// nella quale i tipi di oggetti non sono in proporzioni
// uguali
class SP_TaskPlanner : public TaskPlanner
{
private:
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
public:
    SP_TaskPlanner(ros::NodeHandle &nh_);
    ~SP_TaskPlanner(){};

    uint compute_cycle_dst(logistic_sim::Mission &mission);
    void compute_route(uint id, logistic_sim::Mission &mission);
    void set_partition() override;
    std::vector<logistic_sim::Path> path_partition() override;
    std::vector<unsigned int> spacetime_dijkstra(const std::vector<std::vector<unsigned int> > &other_paths, const std::vector<std::vector<unsigned int> > &graph, unsigned int size, const std::vector<unsigned int> &waypoints, uint ID_ROBOT);
    std::vector<uint> token_dijkstra(const std::vector<uint> &waypoints, std::vector<logistic_sim::Path> &other_paths, uint ID_ROBOT);
    unsigned int insertion_sort(std::vector<st_location> &queue, unsigned int size, st_location loc);
    unsigned int insertion_sort(st_location *queue, unsigned int size, st_location loc);
};

} // namespace SP_TaskPlanner


#include "impl/SP_TaskPlanner.i.hpp"