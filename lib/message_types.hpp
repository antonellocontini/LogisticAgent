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

// Message types
#define NUM_MAX_ROBOTS 32
//#define INTERFERENCE_DISTANCE 2.5
#define INTERFERENCE_DISTANCE 0.5
#define SHARE_MSG 33
#define DELTA_TIME_SEQUENTIAL_START 15
#define SIMULATE_FOREVER false // WARNING: Set this to false, if you want a finishing condition.
#define P true  //print
#define DBG true

#define INITIALIZE_MSG_TYPE 10
#define TARGET_REACHED_MSG_TYPE 11
#define INTERFERENCE_MSG_TYPE 12
#define TASK_REACHED_MSG_TYPE 15
#define RESENDGOAL_MSG_TYPE 14
#define TASK_PLANNER 888
#define TASK_PLANNER_MSG_TYPE 883
#define POSITION_MSG_TYPE 13

#define Pr false

//LOGISCTIC AGENT
#define INIT_MSG 37
#define START 46
#define AT_HOME_MSG_TYPE 47
#define END_MSG_TYPE 333
//
#define GBS_MSG_TYPE 31
#define SEBS_MSG_TYPE 32
#define CBLS_MSG_TYPE 32

// Message types for DTAGreedy algorithm
#define DTAGREEDY_MSG_TYPE 40 //Idleness message: msg format: [ID_ROBOT,msg_type,global_idleness[1..dimension],next_vertex]

// Message types for DTASSI algorithm
#define DTASSI_TR 41 //Task request, msg format: [ID_ROBOT,msg_type,next_vertex_index,bid_value]
#define DTASSI_BID 42 //Bid Message, msg format: [ID_ROBOT,msg_type,next_vertex_index,bid_value]
#define NUM_MAX_ROBOTS 32
#define DEAD_ROBOT_TIME 60.0           // (seconds) time from last goal reached after which a robot is
                                        // considered dead
#define TIMEOUT_WRITE_RESULTS 180.0     // (seconds) timeout for writing results to file
// For hystograms
// #define RESOLUTION 1.0     // seconds
#define MAXIDLENESS 500.0  // seconds

#define LOG_MONITOR 0
#define SAVE_HYSTOGRAMS 0
#define EXTENDED_STAGE 0

#define SIMULATE_FOREVER false             // WARNING: Set this to false, if you want a finishing condition.
#define TIMEOUT_WRITE_RESULTS_FOREVER 900.0  // timeout for writing results to file when simulating forever
#define MAX_DIMENSION 200

#define TASK_PLANNER_ID 333