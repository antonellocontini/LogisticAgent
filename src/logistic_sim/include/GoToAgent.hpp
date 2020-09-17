
/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, ISR University of Coimbra.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the ISR University of Coimbra nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Luca Iocchi (2014-2016)
*********************************************************************/

#pragma once

#include "Agent.hpp"
#include "logistic_sim/GoToPos.h"
#include "logistic_sim/CancelGoTo.h"

namespace gotoagent
{

using uint = unsigned int;
using MoveBaseClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;
using namespace std;

const std::string PS_path = ros::package::getPath("logistic_sim"); // D.Portugal => get pkg path

class GoToAgent : public agent::Agent
{
protected:
  ros::ServiceServer goto_pos_service, cancel_goto_service;
  bool goto_pos(logistic_sim::GoToPos::Request &msg, logistic_sim::GoToPos::Response &res);
  bool cancel_goto(logistic_sim::CancelGoTo::Request &msg, logistic_sim::CancelGoTo::Response &res);
  void goalDoneCallback(const actionlib::SimpleClientGoalState &state,
                        const move_base_msgs::MoveBaseResultConstPtr &result) override;
public:

  void init(int argc, char **argv) override;
  void run() override;
  void token_callback(const logistic_sim::TokenConstPtr &msg) override;

  void backup();
};

} // namespace agent
