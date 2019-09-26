#pragma once

namespace sp_taskplanner
{

SP_TaskPlanner::SP_TaskPlanner(ros::NodeHandle &nh_)
    : TaskPlanner(nh_, "SP_TaskPlanner")
{

}

uint SP_TaskPlanner::compute_cycle_dst(logistic_sim::Mission &mission)
{
    uint res = 0;
    sort(mission.DSTS.begin(), mission.DSTS.end());
    mission.DSTS.erase(unique(mission.DSTS.begin(), mission.DSTS.end()), mission.DSTS.end());
    if (mission.DSTS.size() == 1)
    {
        if (mission.DSTS[0] == 18)
            res = 1;
        else if (mission.DSTS[0] == 23)
            res = 2;
        else
            res = 3;
    }
    else if (mission.DSTS.size() == 2)
    {
        if ((mission.DSTS[0] == 18) && (mission.DSTS[1] == 23))
        {
            res = 4;
        }
        else if ((mission.DSTS[0] == 18) && (mission.DSTS[1] == 28))
        {
            res = 5;
        }
        else
        {
            res = 6;
        }
    }
    else
    {
        res = 7;
    }

    return res;
}

void SP_TaskPlanner::compute_route(uint id, logistic_sim::Mission &m)
{
    switch(id)
    {
        case 1:
        {
            std::copy(std::begin(p_11), std::end(p_11), back_inserter(m.ROUTE));
            m.PATH_DISTANCE = compute_cost_of_route(m.ROUTE);
            m.V = (double)m.PATH_DISTANCE / (double)m.TOT_DEMAND;
        }
        break;
         case 2:
        {
            std::copy(std::begin(p_16), std::end(p_16), back_inserter(m.ROUTE));
            m.PATH_DISTANCE = compute_cost_of_route(m.ROUTE);
            m.V = (double)m.PATH_DISTANCE / (double)m.TOT_DEMAND;
        }
        break;
         case 3:
        {
            std::copy(std::begin(p_21), std::end(p_21), back_inserter(m.ROUTE));
            m.PATH_DISTANCE = compute_cost_of_route(m.ROUTE);
            m.V = (double)m.PATH_DISTANCE / (double)m.TOT_DEMAND;
        }
        break;
         case 4:
        {
            std::copy(std::begin(p_11_16), std::end(p_11_16), back_inserter(m.ROUTE));
            m.PATH_DISTANCE = compute_cost_of_route(m.ROUTE);
            m.V = (double)m.PATH_DISTANCE / (double)m.TOT_DEMAND;
        }
        break;
         case 5:
        {
            std::copy(std::begin(p_11_21), std::end(p_11_21), back_inserter(m.ROUTE));
            m.PATH_DISTANCE = compute_cost_of_route(m.ROUTE);
            m.V = (double)m.PATH_DISTANCE / (double)m.TOT_DEMAND;
        }
        break;
         case 6:
        {
            std::copy(std::begin(p_16_21), std::end(p_16_21), back_inserter(m.ROUTE));
            m.PATH_DISTANCE = compute_cost_of_route(m.ROUTE);
            m.V = (double)m.PATH_DISTANCE / (double)m.TOT_DEMAND;
        }
        break;
         case 7:
        {
            std::copy(std::begin(p_11_16_21), std::end(p_11_16_21), back_inserter(m.ROUTE));
            m.PATH_DISTANCE = compute_cost_of_route(m.ROUTE);
            m.V = (double)m.PATH_DISTANCE / (double)m.TOT_DEMAND;
        }
        break;
         default:
        {
            c_print("ERR",red);
        }
        break;
        
    }
}


void SP_TaskPlanner::set_partition()
{
    std::vector<t_coalition> good_partition;
    try
    {

        c_print(nTask);
        partition::iterator it(nTask);
        int id_partition = 0;

        t_coalition candidate;
        while (true)
        {
            std::vector<std::vector<logistic_sim::Mission>> partitions = *it[missions];
            auto n_subsets = it.subsets();
            logistic_sim::Mission candidate_partition;
            candidate_partition.ID = id_partition;
            // c_print(id_partition);
            id_partition++;
            uint tmp_TD = 0;
            int id_subset = 0;
            double V = 0;
            candidate.second = candidate_partition;
            std::vector<logistic_sim::Mission> m;
            for (int i = 0; i < n_subsets; i++)
            {
                uint tmp_D = 0;
                std::vector<logistic_sim::Mission> subset = partitions[i];
                logistic_sim::Mission candidate_subset;
                candidate_subset.ID = id_subset;
                id_subset++;
                for (int j = 0; j < subset.size(); j++)
                {
                    candidate_subset.TOT_DEMAND += subset[j].TOT_DEMAND;
                    copy(subset[j].DEMANDS.begin(), subset[j].DEMANDS.end(), back_inserter(candidate_subset.DEMANDS));
                    copy(subset[j].DSTS.begin(), subset[j].DSTS.end(), back_inserter(candidate_subset.DSTS));
                    copy(subset[j].ITEM.begin(), subset[j].ITEM.end(), back_inserter(candidate_subset.ITEM));
                }
                
                uint id_path = compute_cycle_dst(candidate_subset);

                compute_route(id_path, candidate_subset);

                candidate.second.V += candidate_subset.V;

                if (candidate_subset.TOT_DEMAND > 3)
                {
                    candidate.second.GOOD++;
                }
                // uso PICKUP == true se non è buona
                // prima soglio e controllo che la partizione sia composta da sottoinsiemi processabili dai robot poi calcolo il percorso e metto in ordine per V poi prendo la prima buona
                m.push_back(candidate_subset);
            }

            ++it;
            candidate.first = m;
            if (candidate.second.GOOD == 0)
            {
                // c_print("ok", green);
                // print_coalition(*it);
                good_partition.push_back(candidate);
            }
        }
    }
    catch (std::overflow_error &)
    {
    }

     std::sort(good_partition.begin(), good_partition.end(), less_V());

     auto ele = good_partition.front();

     print_coalition(ele);

     missions = ele.first;

} // namespace taskplanner

} // namespace nonuniformtaskplanner