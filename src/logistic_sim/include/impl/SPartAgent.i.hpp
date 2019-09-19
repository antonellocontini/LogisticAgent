#pragma once

namespace spartagent
{

logistic_sim::Mission SPartAgent::set_partition_tasks(logistic_sim::Token &token)
{
    // size delle missioni nel token
    auto set_size = token.MISSION.size();

    // try
    // {
    //     partition::iterator it(size_m);
    //     int id = 0;
    //     while (1)
    //     {
    //         std::vector<std::vector<logistic_sim::Mission>> all_part = *it[TP.missions]; // vettore di missioni
    //         auto a = all_part.size();
    //         cout << "size_all: " << a << "\n";
    //         auto subset = it.subsets();
    //         t_coalition candidate;
    //         candidate.second.ID = id;
    //         id++;
    //         // candidate.first.resize(subset);
    //         uint id_m = 0;
    //         double tmp_V = 0;
    //         for (auto i = 0; i < subset; i++)
    //         {
    //             uint tmp_D = 0;
    //             std::vector<logistic_sim::Mission> part = all_part[i];
    //             //dalla part mi creo la missione
    //             candidate.first = part;
    //             for (auto j = 0; j < part.size(); j++)
    //             {
    //                 candidate.second.TOT_DEMAND += part[j].TOT_DEMAND;
    //                 copy(part[j].DSTS.begin(), part[j].DSTS.end(), back_inserter(candidate.second.DSTS));
    //                 copy(part[j].DEMANDS.begin(), part[j].DEMANDS.end(), back_inserter(candidate.second.DEMANDS));
    //                 copy(part[j].ITEM.begin(), part[j].ITEM.end(), back_inserter(candidate.second.ITEM));
    //                 candidate.second.PATH_DISTANCE += part[j].PATH_DISTANCE;
    //             }

    //             candidate.second.V = (double)candidate.second.PATH_DISTANCE / (double)candidate.second.TOT_DEMAND;
    //             if (candidate.second.TOT_DEMAND <= CAPACITY_ROBOT)
    //             {
    //                 v_coalitions.push_back(candidate);
    //                 // print_coalition(candidate);
    //             }
    //         }
    //         ++it;
    //     }
    // }
    // catch (std::overflow_error &)
    // {
    // }
    // // std::sort(v_coalitions.begin(), v_coalitions.end(), less_V());
    // c_print("fine preparazione!", red);
}

} // namespace spartagent