/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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
 *********************************************************************/

/* Author: Deok-Hwa Kim */

#include "joint_path_service_capability.h"
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/collision_detection/collision_tools.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/move_group/capability_names.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

move_group::MoveGroupJointPathService::MoveGroupJointPathService()
    : MoveGroupCapability("JointPathService"), display_computed_paths_(true)
{
}

void move_group::MoveGroupJointPathService::initialize()
{
    display_path_ = node_handle_.advertise<moveit_msgs::DisplayTrajectory>(
                        planning_pipeline::PlanningPipeline::DISPLAY_PATH_TOPIC, 10, true);
    joint_path_service_ = root_node_handle_.advertiseService(JOINT_PATH_SERVICE_NAME,
                          &MoveGroupJointPathService::computeService, this);
}

namespace
{
bool isStateValid(const planning_scene::PlanningScene* planning_scene,
                  const kinematic_constraints::KinematicConstraintSet* constraint_set, robot_state::RobotState* state,
                  const robot_state::JointModelGroup* group, const double* ik_solution)
{
    state->setJointGroupPositions(group, ik_solution);
    state->update();
    return (!planning_scene || !planning_scene->isStateColliding(*state, group->getName())) &&
           (!constraint_set || constraint_set->decide(*state).satisfied);
}
}

bool move_group::MoveGroupJointPathService::computeService(moveit_ros_move_group::GetJointPath::Request& req,
        moveit_ros_move_group::GetJointPath::Response& res)
{
    ROS_INFO("Received request to compute Joint path");
    context_->planning_scene_monitor_->updateFrameTransforms();
    robot_state::RobotState start_state =
        planning_scene_monitor::LockedPlanningSceneRO(context_->planning_scene_monitor_)->getCurrentState();
    robot_state::robotStateMsgToRobotState(req.start_state, start_state);

    if (const robot_model::JointModelGroup* jmg = start_state.getJointModelGroup(req.group_name)) {
        bool ok = true;
        std::vector<robot_state::RobotStatePtr> traj;

        robot_state::robotStateToRobotStateMsg(start_state, res.start_state);
        std::string target_group = req.group_name;



        const robot_model::RobotModelPtr& kmodel = context_->planning_scene_monitor_->getRobotModelLoader()->getModel();
        const std::vector<robot_model::JointModel*>& jms = kmodel->getJointModels();
        for(robot_model::JointModel* jm : jms) {
            const std::vector<std::string>& jm_variable_names = jm->getVariableNames(); 
            for(auto jm_var : jm_variable_names) {
                auto jm_var_bound = jm->getVariableBounds(jm_var);
                //jm_var_bound.position_bounded_ = false;
                //jm_var_bound.min_position_ = -3.14;
                //jm_var_bound.max_position_ = 3.14;
                //jm->setVariableBounds(jm_var, jm_var_bound);
            }
        }


        robot_state::RobotState query_robot_state = start_state;
        const robot_model::JointModelGroup* joint_model_group = query_robot_state.getJointModelGroup(target_group);

        planning_scene_monitor::LockedPlanningSceneRO ps(context_->planning_scene_monitor_);

        std::vector<double> query_joint_states;
        query_robot_state.copyJointGroupPositions(joint_model_group, query_joint_states);

        const std::vector<std::string>& query_joint_names = joint_model_group->getActiveJointModelNames();
        const std::vector<std::string>& req_joint_names = req.waypoints.joint_names;

        std::vector<robot_state::RobotState> nc_waypoints;
        std::vector<robot_state::RobotState> nc_waypoints_time;
        nc_waypoints.push_back(start_state);


        
        std::size_t last_waypoint_index = 0;

        for(std::size_t i = 0; i < req.waypoints.points.size(); ++i){
            trajectory_msgs::JointTrajectoryPoint waypoint = req.waypoints.points[i];
            for(std::size_t j = 0; j < query_joint_names.size(); ++j){
                for(std::size_t k = 0; k < req_joint_names.size(); ++k){
                    if(req_joint_names[k] == query_joint_names[j]){
                        query_joint_states[j] = waypoint.positions[k];
                        break;
                    }
                }
            }
            query_robot_state.setJointGroupPositions(joint_model_group, query_joint_states);
            if(ps->isStateColliding(query_robot_state, target_group, true)) {
                std::cout << "This state is colliding " << i << " / " << req.waypoints.points.size()  << std::endl;

                float p_last = (float)(req.waypoints.points.size()-1 - last_waypoint_index)/(float)req.waypoints.points.size();

                if(i ==  req.waypoints.points.size() - 1 && p_last > 0.1)
                {
                    std::cout << "!!!!!!!! this trajectory could not solved !!" << std::endl;
                    //nc_waypoints.push_back(query_robot_state);
                    //res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
                    //return false;
                }
            }
            else{
                nc_waypoints.push_back(query_robot_state);
                last_waypoint_index = i;
                //std::cout << "This state is not colliding " << i << " / " << req.waypoints.points.size()  << std::endl;
            }
        }

        robot_state::RobotState prev_state = nc_waypoints[0];

        std::vector<robot_trajectory::RobotTrajectory> computed_traj;
        for(std::size_t i = 1; i < nc_waypoints.size(); ++i){
            const robot_state::RobotState& next_state = nc_waypoints[i];

            std::vector<double> prev_joint_states;
            std::vector<double> next_joint_states;
            prev_state.copyJointGroupPositions(joint_model_group, prev_joint_states);
            next_state.copyJointGroupPositions(joint_model_group, next_joint_states);

            const double min_diff = 3.0*3.14/180.0;
            double diff = 0.0;
            for(std::size_t j = 0; j < prev_joint_states.size(); ++j){
                diff += std::abs(prev_joint_states[j] - next_joint_states[j]);
            }
            diff /= prev_joint_states.size();

            //std::cout << "[DIFF] : " << diff << std::endl;
            if(diff < min_diff && i != (nc_waypoints.size()-1)){
                continue;
            }


            planning_interface::MotionPlanRequest mp_req;
            planning_interface::MotionPlanResponse mp_res;

            mp_req.group_name = target_group;

            robot_state::robotStateToRobotStateMsg(prev_state, mp_req.start_state);
            mp_req.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(next_state, joint_model_group));



            bool solved = context_->planning_pipeline_->generatePlan(ps, mp_req, mp_res);
            if(solved){
                prev_state = mp_res.trajectory_->getLastWayPoint();

                for(std::size_t j = 0; j < mp_res.trajectory_->getWayPointCount(); ++j) {
                    //traj.push_back(mp_res.trajectory_->getWayPointPtr(j));
                }

                computed_traj.push_back(*mp_res.trajectory_);
            }
        }

        robot_trajectory::RobotTrajectory rt(context_->planning_scene_monitor_->getRobotModel(), req.group_name);
        //std::cout << rt.getWayPointCount() <<std::endl;

        std::vector<double> prev_joint_states;
        computed_traj[0].getFirstWayPoint().copyJointGroupPositions(joint_model_group, prev_joint_states);
        std::vector<double> zero_velocities(prev_joint_states.size(), 0.0);
        for(std::size_t i = 0; i < computed_traj.size(); ++i){
            const robot_trajectory::RobotTrajectory& traj = computed_traj[i];
            for(std::size_t j = 1; j < traj.getWayPointCount(); ++j){

                std::vector<double> next_joint_states;
                traj.getWayPoint(j).copyJointGroupPositions(joint_model_group, next_joint_states);

                double diff = 0.0;
                for(std::size_t j = 0; j < prev_joint_states.size(); ++j){
                    diff += std::abs(prev_joint_states[j] - next_joint_states[j]);
                }
                diff /= prev_joint_states.size();


                //double dt = diff /joint_speed;
                double dt = traj.getWayPointDurationFromPrevious(j);

                if(j == 0){
                    //dt = 0.1;
                }
                rt.addSuffixWayPoint(traj.getWayPoint(j), dt);
                rt.getLastWayPointPtr()->setJointGroupVelocities(joint_model_group, zero_velocities);


                prev_joint_states = next_joint_states;
            }
        }

        //trajectory_processing::IterativeParabolicTimeParameterization time_param;
        //time_param.computeTimeStamps(rt, 0.1);
       // std::cout << rt.getWayPointCount() <<std::endl;

        /*
        robot_trajectory::RobotTrajectory rt(context_->planning_scene_monitor_->getRobotModel(), req.group_name);
        for (std::size_t i = 0; i < traj.size(); ++i){
            rt.addSuffixWayPoint(traj[i], 0.0);
        }


        */

        rt.getRobotTrajectoryMsg(res.solution);
        res.solution.joint_trajectory.header.stamp = ros::Time::now()+ros::Duration(1.0);
        //std::cout <<res.solution <<std::endl;


        ROS_INFO("Computed Joint path with %u points",
                (unsigned int)traj.size());
        if (display_computed_paths_ && rt.getWayPointCount() > 0)
        {
            moveit_msgs::DisplayTrajectory disp;
            disp.model_id = context_->planning_scene_monitor_->getRobotModel()->getName();
            disp.trajectory.resize(1, res.solution);
            robot_state::robotStateToRobotStateMsg(rt.getFirstWayPoint(), disp.trajectory_start);
            display_path_.publish(disp);
        }

        res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

    }
    else {
        res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
    }

    return true;
}

#include <class_loader/class_loader.h>
CLASS_LOADER_REGISTER_CLASS(move_group::MoveGroupJointPathService, move_group::MoveGroupCapability)
