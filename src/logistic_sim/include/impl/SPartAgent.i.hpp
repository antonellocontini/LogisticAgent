#pragma once

namespace spartagent
{

logistic_sim::Mission SPartAgent::set_partition_tasks(logistic_sim::Token &token)
{
    // size delle missioni nel token
    auto set_size = token.MISSION.size();

    try
    {
        partition::iterator it(set_size);
        int id = 0;
        while (true)
        {
            auto all_part = *it[token.MISSION];
            auto subset = it.subsets();
            auto as = all_part.size();
            Candidate c;
            c.id = id;
            c.vv_MISSION.resize(subset);
            id++;
            uint pm_id = 0;
            double tmp_V = 0;
            for (auto i = 0; i < subset; i++)
            {
                uint tmp_d = 0;
                auto part = all_part[i];
                logistic_sim::Mission m;
                m.ID = pm_id;
                pm_id++;
                for (auto j = 0; j < part.size(); j++)
                {
                }
            }
        }
    }
    catch (std::overflow_error &)
    {
    }
}

} // namespace spartagent