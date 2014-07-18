#include "prace_moveit_interface/MoveItInterface.h"

#include <moveit/robot_state/conversions.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/common_planning_interface_objects/common_objects.h>

#include <trajectory_msgs/JointTrajectory.h>

#include <stdio.h>
#include <termios.h>    	//termios, TCSANOW, ECHO, ICANON
#include <unistd.h> 	//STDIN_FILENO

void pressKey()
{
    //the struct termios stores all kinds of flags which can manipulate the I/O Interface
    //I have an old one to save the old settings and a new
    static struct termios oldt, newt;
    printf("Press key to continue....\n");

    //tcgetattr gets the parameters of the current terminal
    //STDIN_FILENO will tell tcgetattr that it should write the settings
    // of stdin to oldt
    tcgetattr( STDIN_FILENO, &oldt);
    //now the settings will be copied
    newt = oldt;

    //two of the c_lflag will be turned off
    //ECHO which is responsible for displaying the input of the user in the terminal
    //ICANON is the essential one! Normally this takes care that one line at a time will be processed
    //that means it will return if it sees a "\n" or an EOF or an EOL
    newt.c_lflag &= ~(ICANON | ECHO );

    //Those new settings will be set to STDIN
    //TCSANOW tells tcsetattr to change attributes immediately.
    tcsetattr( STDIN_FILENO, TCSANOW, &newt);

    //now the char wil be requested
    getchar();

    //the old settings will be written back to STDIN
    tcsetattr( STDIN_FILENO, TCSANOW, &oldt);

}


MoveItInterface::MoveItInterface(ros::NodeHandle &nodeHandle) :
	_nh(nodeHandle), _chain_plans(true)
{
	transformer = new tf::TransformListener();
	broadcaster = new tf::TransformBroadcaster();

	ros::NodeHandle nh("~");
	std::string action_ns;
	nh.param<std::string>("action_ns", action_ns, "arm_controller");

	_moveLinNs = _planLinNs = action_ns;
	_movePTPNs = _planPTPNs = action_ns;
	_moveJointSpaceNs = _planJointSpaceNs = action_ns;
	_moveLinNs.append("/MoveLin");
	_movePTPNs.append("/MovePTP");
	_moveJointSpaceNs.append("/MoveJointSpace");

	ROS_DEBUG("movelin_ns: %s",_moveLinNs.c_str());
	ROS_DEBUG("moveptp_ns: %s",_movePTPNs.c_str());
	ROS_DEBUG("movejointspace_ns: %s",_moveJointSpaceNs.c_str());

	_pubPose = _nh.advertise<geometry_msgs::PoseStamped>("test_pose",1);
	_pubAco = _nh.advertise<moveit_msgs::AttachedCollisionObject>("/attached_collision_object",10);
	_displayPathPublisher = _nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 10, true);

	_asMoveLin = new SimpleActionLin(_nh, _moveLinNs, boost::bind(&MoveItInterface::moveLinCallback, this, _1),false);
	_asMovePTP = new SimpleActionPtp(_nh, _movePTPNs, boost::bind(&MoveItInterface::movePTPCallback, this, _1),false);
	_asMoveJointSpace = new SimpleActionJointSpace(_nh, _moveJointSpaceNs, boost::bind(&MoveItInterface::moveJointSpaceCallback, this, _1),false);

	_asPlanLin = new SimpleActionLin(_nh, _planLinNs.append("/PlanLin"), boost::bind(&MoveItInterface::planLinCallback, this, _1),false);
	_asPlanPTP = new SimpleActionPtp(_nh, _planPTPNs.append("/PlanPTP"), boost::bind(&MoveItInterface::planPTPCallback, this, _1),false);
	_asPlanJointSpace = new SimpleActionJointSpace(_nh, _planJointSpaceNs.append("/PlanJointSpace"), boost::bind(&MoveItInterface::planJointSpaceCallback, this, _1),false);

	_asExecPlan = new SimpleActionExecPlan(_nh, action_ns + "/ExecPlan", boost::bind(&MoveItInterface::executePlansCallback, this, _1),false);
	_asExecQueue = new SimpleActionExecPlan(_nh, action_ns + "/ExecQueue", boost::bind(&MoveItInterface::executeQueue, this, _1),false);

	_asSyncMove = new ActionSyncMove(_nh, action_ns + "/SyncMove", false);
	_asSyncMove->registerGoalCallback(boost::bind(&MoveItInterface::syncGoalCallback, this, _1));
	_asSyncMove->registerCancelCallback(boost::bind(&MoveItInterface::syncCancelCallback, this, _1));

	_asQueueLin = new ActionLin(_nh, action_ns + "/QueueLin", false);
	_asQueueLin->registerGoalCallback(boost::bind(&MoveItInterface::queueLinGoalCallback, this, _1));
	_asQueuePTP = new ActionPtp(_nh, action_ns + "/QueuePtp", false);
	_asQueuePTP->registerGoalCallback(boost::bind(&MoveItInterface::queuePTPGoalCallback, this, _1));
	_asQueueJointSpace = new ActionJointSpace(_nh, action_ns + "/QueueJointSpace", false);
	_asQueueJointSpace->registerGoalCallback(boost::bind(&MoveItInterface::queueJointSpaceGoalCallback, this, _1));

	_asMoveLin->start();
	_asMovePTP->start();
	_asMoveJointSpace->start();

	_asPlanLin->start();
	_asPlanPTP->start();
	_asPlanJointSpace->start();

	_asQueueLin->start();
	_asQueuePTP->start();
	_asQueueJointSpace->start();

	_asExecPlan->start();
	_asExecQueue->start();
	_asSyncMove->start();

	addGround();
	addTable();
	//addTable2();
	//addPallet();

	getMoveGroup("left_arm_gripper");
	getMoveGroup("right_arm_gripper");
	getMoveGroup("both_arms_gripper");

	_lookupMirrorMoveGroup.insert(std::pair<std::string, std::string>("left_arm_gripper","right_arm_gripper"));
	_lookupMirrorMoveGroup.insert(std::pair<std::string, std::string>("right_arm_gripper","left_arm_gripper"));
	_lookupMirrorMoveGroup.insert(std::pair<std::string, std::string>("both_arms_gripper","null"));

	ROS_INFO_STREAM("left_arm_gripper is: "<<_lookupMirrorMoveGroup["left_arm_gripper"]);
	ROS_INFO_STREAM("right_arm_gripper is: "<<_lookupMirrorMoveGroup["right_arm_gripper"]);
	ROS_INFO_STREAM("both_arms_gripper is: "<<_lookupMirrorMoveGroup["both_arms_gripper"]);

}

MoveItInterface::~MoveItInterface()
{
	delete transformer;
	delete broadcaster;

	delete _asMoveLin;
	delete _asMovePTP;
	delete _asMoveJointSpace;

	delete _asPlanLin;
	delete _asPlanPTP;
	delete _asPlanJointSpace;

	delete _asQueueLin;
	delete _asQueuePTP;
	delete _asQueueJointSpace;

	delete _asExecPlan;
	delete _asExecQueue;
	delete _asSyncMove;
}

void MoveItInterface::moveLinCallback(const prace_moveit_interface::LinGoalConstPtr &goal)
{
	ROS_DEBUG("moveLinCallback");
	move_group_interface::MoveGroup* group = getMoveGroup(goal->move_group);
	std::vector<geometry_msgs::Pose> poses;
	move_group_interface::MoveGroup::Plan plan;

	_plansToExecute.clear();

	_pubPose.publish(goal->goal_pose);

	group->setEndEffectorLink(goal->target_frame);
	group->setPoseReferenceFrame(goal->goal_pose.header.frame_id);

	poses.push_back(goal->goal_pose.pose);

	robot_state::RobotStatePtr rstate;
	rstate = group->getCurrentState();
	group->setStartState(*rstate);

	double fraction = group->computeCartesianPath(poses, 0.02, 4, plan.trajectory_, true);

	if(fraction == 1.0)
	{
		if(group->execute(plan))
			_asMoveLin->setSucceeded();
		else
			_asMoveLin->setAborted();
	}
	else
		_asMoveLin->setAborted();
}

void MoveItInterface::movePTPCallback(const prace_moveit_interface::PtpGoalConstPtr &goal)
{
	ROS_DEBUG("movePTPCallback");
	move_group_interface::MoveGroup* group = getMoveGroup(goal->move_group);
	_pubPose.publish(goal->goal_pose);

	_plansToExecute.clear();

	group->setEndEffectorLink(goal->target_frame.c_str());
	group->setPoseReferenceFrame(goal->goal_pose.header.frame_id);
	group->setPoseTarget(goal->goal_pose, goal->target_frame.c_str());

	robot_state::RobotStatePtr rstate;
	rstate = group->getCurrentState();
	group->setStartState(*rstate);

	if(group->move())
		_asMovePTP->setSucceeded();
	else
		_asMovePTP->setAborted();
}

void MoveItInterface::moveJointSpaceCallback(const prace_moveit_interface::JointSpaceGoalConstPtr &goal)
{
	ROS_DEBUG("moveJointSpaceCallback");
	move_group_interface::MoveGroup* group = getMoveGroup(goal->move_group);
	_plansToExecute.clear();

	robot_state::RobotStatePtr rstate;
	rstate = group->getCurrentState();

	group->setStartState(*rstate);
	group->setJointValueTarget(goal->joint_state);

	if(group->move())
		_asMoveJointSpace->setSucceeded();
	else
		_asMoveJointSpace->setAborted();
}

void MoveItInterface::planLinCallback(const prace_moveit_interface::LinGoalConstPtr &goal)
{
	ROS_DEBUG("planLinCallback");
	move_group_interface::MoveGroup* group = getMoveGroup(goal->move_group);
	std::vector<geometry_msgs::Pose> poses;
	move_group_interface::MoveGroup::Plan plan;

	waitfortransform(goal->goal_pose.header.frame_id, goal->target_frame);

	_pubPose.publish(goal->goal_pose);

	group->setEndEffectorLink(goal->target_frame);
	group->setPoseReferenceFrame(goal->goal_pose.header.frame_id);

	poses.push_back(goal->goal_pose.pose);

	group->setStartState(*(getStartState(goal->move_group)));
	double fraction = group->computeCartesianPath(poses, 0.02, 4, plan.trajectory_, true);

	if(fraction == 1.0)
	{
		speedup(plan, goal->linVel);
		_plansToExecute.push_back(GroupPlanPair(group, plan));
		updateStartState(plan);
		_asPlanLin->setSucceeded();
	}
	else
	{
		_asPlanLin->setAborted();
		_plansToExecute.clear();
	}
}

void MoveItInterface::planPTPCallback(const prace_moveit_interface::PtpGoalConstPtr &goal)
{
	ROS_DEBUG("planPTPCallback");

	move_group_interface::MoveGroup* group = getMoveGroup(goal->move_group);
	move_group_interface::MoveGroup::Plan plan;

	waitfortransform(goal->goal_pose.header.frame_id, goal->target_frame);
	_pubPose.publish(goal->goal_pose);

	group->setStartState(*(getStartState(goal->move_group)));

	//group->setEndEffectorLink(goal->target_frame.c_str());
	group->setPoseReferenceFrame(goal->goal_pose.header.frame_id);
	group->setPoseTarget(goal->goal_pose, goal->target_frame.c_str());


	if(group->plan(plan))
	{
		speedup(plan, goal->vel_factor);
		_plansToExecute.push_back(GroupPlanPair(group, plan));
		updateStartState(plan);
		_asPlanPTP->setSucceeded();
	}
	else
	{
		_asPlanPTP->setAborted();
		_plansToExecute.clear();
	}
}

void MoveItInterface::planJointSpaceCallback(const prace_moveit_interface::JointSpaceGoalConstPtr &goal)
{
	ROS_DEBUG("planJointSpaceCallback");

	move_group_interface::MoveGroup* group = getMoveGroup(goal->move_group);
	move_group_interface::MoveGroup::Plan plan;

	group->setStartState(*(getStartState(goal->move_group)));
	group->setJointValueTarget(goal->joint_state);

	if(group->plan(plan))
	{
		speedup(plan, goal->vel_factor);
		_plansToExecute.push_back(GroupPlanPair(group, plan));
		updateStartState(plan);
		_asPlanJointSpace->setSucceeded();
	}
	else
	{
		_asPlanJointSpace->setAborted();
		_plansToExecute.clear();
	}
}

void MoveItInterface::queueLinGoalCallback(ActionLin::GoalHandle gh)
{
	ROS_INFO("queueLinGoalCallback");
	prace_moveit_interface::LinGoal goal;
	gh.setAccepted();
	goal = *gh.getGoal();
	queuePlanType newGoal;
	newGoal.type = LIN;
	newGoal.linGoal = goal;

	_queuePlans.push_back(newGoal);
	gh.setSucceeded();

}

void MoveItInterface::queuePTPGoalCallback(ActionPtp::GoalHandle gh)
{
	ROS_INFO("queuePTPGoalCallback");
	prace_moveit_interface::PtpGoal goal;
	gh.setAccepted();
	goal = *gh.getGoal();
	queuePlanType newGoal;
	newGoal.type = PTP;
	newGoal.ptpGoal = goal;

	_queuePlans.push_back(newGoal);
	gh.setSucceeded();
}

void MoveItInterface::queueJointSpaceGoalCallback(ActionJointSpace::GoalHandle gh)
{
	ROS_INFO("queueJointSpaceGoalCallback");
	prace_moveit_interface::JointSpaceGoal goal;
	gh.setAccepted();
	goal = *gh.getGoal();
	queuePlanType newGoal;
	newGoal.type = JS;
	newGoal.jsGoal = goal;

	_queuePlans.push_back(newGoal);
	gh.setSucceeded();
}

std::vector<MoveItInterface::SortedQueueType> MoveItInterface::sortQueue(std::list<queuePlanType> queue)
{
	ROS_INFO("sortQueue");
	std::vector<SortedQueueType> sortedQueue;

	ROS_INFO_STREAM("queue Size Start: "<<queue.size());

	while(!queue.empty())
	{
		//get first element and search the next corresponding element
		queuePlanType elemFirst = queue.front();
		queuePlanType elemSecond;
		elemSecond.type = -1;
		queue.pop_front();

		for(std::list<queuePlanType>::iterator it = queue.begin(); it != queue.end(); ++it)
		{
			ROS_INFO_STREAM("queue Size: "<<queue.size());

			if((*it).type == elemFirst.type)
			{
				if(elemFirst.type == LIN)
				{
					if(_lookupMirrorMoveGroup[elemFirst.linGoal.move_group] == (*it).linGoal.move_group)
					{
						elemSecond = *it;
						queue.erase(it);
						break;
					}
				}
				if(elemFirst.type == PTP)
				{
					if(_lookupMirrorMoveGroup[elemFirst.ptpGoal.move_group] == (*it).ptpGoal.move_group)
					{
						elemSecond = *it;
						queue.erase(it);
						elemFirst.ptpGoal.move_group = "both_arms_gripper";
						elemSecond.ptpGoal.move_group = "both_arms_gripper";
						break;
					}
				}
				if(elemFirst.type == JS)
				{
					if(_lookupMirrorMoveGroup[elemFirst.jsGoal.move_group] == (*it).jsGoal.move_group)
					{
						elemSecond = *it;
						queue.erase(it);
						elemFirst.jsGoal.move_group = "both_arms_gripper";
						elemSecond.jsGoal.move_group = "both_arms_gripper";
						break;
					}
				}
			}
		}
		sortedQueue.push_back(SortedQueueType(elemFirst, elemSecond));
	}
	return sortedQueue;
}

void MoveItInterface::executeQueue(const prace_moveit_interface::ExecPlanGoalConstPtr &goal)
{
	// First sort queue
	std::vector<SortedQueueType> sortedQueue;
	sortedQueue = sortQueue(_queuePlans);

	//get first entry
	for(size_t i = 0; i < sortedQueue.size(); ++i)
	{
		switch(sortedQueue[i].first.type)
		{
			case PTP:
			{
				ROS_INFO("Planning PTP");
				move_group_interface::MoveGroup* group = getMoveGroup(sortedQueue[i].first.ptpGoal.move_group);
				move_group_interface::MoveGroup::Plan plan;

				waitfortransform(sortedQueue[i].first.ptpGoal.goal_pose.header.frame_id, sortedQueue[i].first.ptpGoal.target_frame);
				if(sortedQueue[i].second.type != -1)
					waitfortransform(sortedQueue[i].second.ptpGoal.goal_pose.header.frame_id, sortedQueue[i].second.ptpGoal.target_frame);

				group->setStartState(*(getStartState(sortedQueue[i].first.ptpGoal.move_group)));

				group->setPoseReferenceFrame(sortedQueue[i].first.ptpGoal.goal_pose.header.frame_id);

				group->setPoseTarget(sortedQueue[i].first.ptpGoal.goal_pose, sortedQueue[i].first.ptpGoal.target_frame.c_str());
				if(sortedQueue[i].second.type != -1)
					group->setPoseTarget(sortedQueue[i].second.ptpGoal.goal_pose, sortedQueue[i].second.ptpGoal.target_frame.c_str());

				if(group->plan(plan))
				{
					speedup(plan, sortedQueue[i].first.ptpGoal.vel_factor);
					_plansToExecute.push_back(GroupPlanPair(group, plan));
					updateStartState(plan);
				}
				else
				{
					ROS_WARN("planing ptp failed");
					_plansToExecute.clear();
					_queuePlans.clear();
					_asExecQueue->setAborted();
					return;
				}
			}
			break;
			case LIN:
			{
				if(sortedQueue[i].second.type != -1)
					ROS_INFO("Planning LIN Dual");
				else
					ROS_INFO("Planning LIN");
				double fraction1=1.0;
				double fraction2=1.0;

				move_group_interface::MoveGroup* group = getMoveGroup(sortedQueue[i].first.linGoal.move_group);
				std::vector<geometry_msgs::Pose> poses;
				move_group_interface::MoveGroup::Plan mergedPlan, plan1, plan2;

				waitfortransform(sortedQueue[i].first.linGoal.goal_pose.header.frame_id, sortedQueue[i].first.linGoal.target_frame);
				if(sortedQueue[i].second.type != -1)
					waitfortransform(sortedQueue[i].second.linGoal.goal_pose.header.frame_id, sortedQueue[i].second.linGoal.target_frame);

				//Calculate first CartesianPath
				group->setEndEffectorLink(sortedQueue[i].first.linGoal.target_frame);
				group->setPoseReferenceFrame(sortedQueue[i].first.linGoal.goal_pose.header.frame_id);
				poses.push_back(sortedQueue[i].first.linGoal.goal_pose.pose);
				group->setStartState(*(getStartState(sortedQueue[i].first.linGoal.move_group)));
				fraction1 = group->computeCartesianPath(poses, 0.02, 4, plan1.trajectory_, true);
				speedup(plan1, sortedQueue[i].first.linGoal.linVel);
				mergedPlan = plan1;

				if(sortedQueue[i].second.type != -1)
				{
					if(fraction1 == 1.0)
					{
						//Update RobotState with last plan
						//updateStartState(plan1);
						group = getMoveGroup(sortedQueue[i].second.linGoal.move_group);

						//Calculate second CartesianPath
						group->setEndEffectorLink(sortedQueue[i].second.linGoal.target_frame);
						group->setPoseReferenceFrame(sortedQueue[i].second.linGoal.goal_pose.header.frame_id);
						poses.clear();
						poses.push_back(sortedQueue[i].second.linGoal.goal_pose.pose);
						group->setStartState(*(getStartState(sortedQueue[i].second.linGoal.move_group)));
						fraction2 = group->computeCartesianPath(poses, 0.02, 4, plan2.trajectory_, true);
						speedup(plan2, sortedQueue[i].second.linGoal.linVel);
						mergedPlan = mergePlans(plan1, plan2);
						group = getMoveGroup("both_arms_gripper");
					}
				}
				if(fraction1 == 1.0 && fraction2 == 1.0)
				{
					_plansToExecute.push_back(GroupPlanPair(group, mergedPlan));
					updateStartState(mergedPlan);
				}
				else
				{
					ROS_WARN("planing lin failed");
					_plansToExecute.clear();
					_asExecQueue->setAborted();
					_queuePlans.clear();
					return;
				}
			}
			break;
			case JS:
			{
				ROS_INFO("Planning PTP");
				move_group_interface::MoveGroup* group = getMoveGroup(sortedQueue[i].first.jsGoal.move_group);
				move_group_interface::MoveGroup::Plan plan;

				if(sortedQueue[i].second.type != -1)
				{
					//merge goal
					sortedQueue[i].first.jsGoal.joint_state.name.insert(
							sortedQueue[i].first.jsGoal.joint_state.name.end(),
							sortedQueue[i].second.jsGoal.joint_state.name.begin(),sortedQueue[i].second.jsGoal.joint_state.name.end());
					sortedQueue[i].first.jsGoal.joint_state.position.insert(
							sortedQueue[i].first.jsGoal.joint_state.position.end(),
							sortedQueue[i].second.jsGoal.joint_state.position.begin(),sortedQueue[i].second.jsGoal.joint_state.position.end());
				}

				group->setStartState(*(getStartState(sortedQueue[i].first.jsGoal.move_group)));
				group->setJointValueTarget(sortedQueue[i].first.jsGoal.joint_state);

				if(group->plan(plan))
				{
					speedup(plan, sortedQueue[i].first.jsGoal.vel_factor);
					_plansToExecute.push_back(GroupPlanPair(group, plan));
					updateStartState(plan);
				}
				else
				{
					ROS_WARN("planing js failed");
					_plansToExecute.clear();
					_asExecQueue->setAborted();
					_queuePlans.clear();
					return;
				}
			}
		}
	}

	_queuePlans.clear();
	bool success = executePlans();

	if(success)
		_asExecQueue->setSucceeded();
	else
	{
		_asExecQueue->setAborted();
		ROS_INFO("execute queue failed");
	}
}

void MoveItInterface::executePlansCallback(const prace_moveit_interface::ExecPlanGoalConstPtr &goal)
{
	bool success = executePlans();

	if(success)
		_asExecPlan->setSucceeded();
	else
		_asExecPlan->setAborted();
}

bool MoveItInterface::executePlans()
{
	ROS_INFO("executePlans");

	bool success = true;
	if(_chain_plans)
	{
		std::vector<GroupPlanPair> chainedPlans = chainPlans(_plansToExecute);
		for(size_t i = 0; i < chainedPlans.size(); i++)
		{
			ROS_INFO("Executing Plan %d", (int)i);
			success = success && chainedPlans[i].first->execute(chainedPlans[i].second);
		}
		ROS_INFO("Executing finished");
		_plansToExecute.clear();
	}
	else
	{
		for(size_t i = 0; i < _plansToExecute.size(); i++)
			success = success && _plansToExecute[i].first->execute(_plansToExecute[i].second);
		_plansToExecute.clear();
	}
	return success;
}

void MoveItInterface::syncGoalCallback(ActionSyncMove::GoalHandle gh)
{
	prace_moveit_interface::SyncMoveGoal goal;
	gh.setAccepted();
	goal = *gh.getGoal();
	_syncGoals[goal.sync_id].push_back(gh);
	size_t activeGoals = _syncGoals[goal.sync_id].size();
	if(activeGoals == (size_t)goal.num_sync_partners)
	{
		//execute queued goals
		for(size_t i = 0; i < activeGoals; ++i)
		{
			_syncGoals[goal.sync_id][i].setSucceeded();
		}
		_syncGoals[goal.sync_id].clear();
	}
}

void MoveItInterface::syncCancelCallback(ActionSyncMove::GoalHandle gh)
{

}



bool MoveItInterface::waitfortransform(std::string frame_id, std::string target)
{
	ros::Time now = ros::Time::now();
	try{
		this->transformer->waitForTransform(target, frame_id, now, ros::Duration(8));
	}
	catch (tf::TransformException ex) {
		ROS_ERROR("%s", ex.what());
		return false;
	}
	return true;
}

bool MoveItInterface::transformLink(geometry_msgs::PoseStamped &goal_trans, geometry_msgs::PoseStamped goal, std::string target)
{
	if(goal.header.frame_id != target)
	{
		ros::Time now = ros::Time::now();
		goal.header.stamp = now;
		try{
			this->transformer->waitForTransform(target, goal.header.frame_id, now, ros::Duration(8));
			this->transformer->transformPose(target, goal, goal_trans);
			_pubPose.publish(goal_trans);
		}
		catch (tf::TransformException ex) {
			ROS_ERROR("%s", ex.what());
			return false;
		}
	}
	else
	{
		ROS_INFO("conversion not required");
		goal_trans = goal;
	}
	return true;
}

robot_state::RobotStatePtr MoveItInterface::getStartState(std::string move_group)
{
	move_group_interface::MoveGroup* group = getMoveGroup(move_group);
	if(_plansToExecute.size() == 0)
	{
		_robotStartState.reset(new robot_state::RobotState(*(group->getCurrentState())));
	}
	return _robotStartState;
}

void MoveItInterface::updateStartState(move_group_interface::MoveGroup::Plan plan)
{
	std::map<std::string, double> jointmap;

	for(size_t i = 0; i < plan.trajectory_.joint_trajectory.joint_names.size(); i++)
	{
		std::string tmpstr = plan.trajectory_.joint_trajectory.joint_names[i];

		jointmap.insert(std::pair<std::string, double>(tmpstr,
				plan.trajectory_.joint_trajectory.points.back().positions[i]));
	}
	_robotStartState->setVariablePositions(jointmap);
	_robotStartState->updateLinkTransforms();

}

robot_state::RobotStatePtr MoveItInterface::setStartState(std::string move_group)
{
	move_group_interface::MoveGroup* group = getMoveGroup(move_group);
	if(_plansToExecute.size() == 0)
	{
		group->setStartStateToCurrentState();
		return group->getCurrentState();
	}
	else
	{
		std::vector<std::string> jointnames = group->getJoints();
		robot_state::RobotState state(*(group->getCurrentState()));
		std::map<std::string, double> jointmap;
		bool allJointsFound = false;
		int i;
		for(i = _plansToExecute.size()-1; (i >= 0) && (allJointsFound == false); i--)
		{
			for(size_t j = 0; j < jointnames.size(); j++)
			{
				allJointsFound = (std::find(_plansToExecute[i].second.trajectory_.joint_trajectory.joint_names.begin(),
					_plansToExecute[i].second.trajectory_.joint_trajectory.joint_names.end(),jointnames[j])!=
							_plansToExecute[i].second.trajectory_.joint_trajectory.joint_names.end());
			}
		}
		if(allJointsFound)
		{
			for(size_t j = 0; j < _plansToExecute[i].second.trajectory_.joint_trajectory.joint_names.size(); j++)
			{
				std::string tmpstr = _plansToExecute[i].second.trajectory_.joint_trajectory.joint_names[j];

				jointmap.insert(std::pair<std::string, double>(tmpstr,
						_plansToExecute[i].second.trajectory_.joint_trajectory.points.back().positions[j]));
			}

			state.setVariablePositions(jointmap);
			state.updateLinkTransforms();
			group->setStartState(state);
			_robotStartState.reset(new robot_state::RobotState(state));
			return _robotStartState;
		}
		else
		{
			group->setStartStateToCurrentState();
			return group->getCurrentState();
		}
	}
}

std::vector<MoveItInterface::GroupPlanPair> MoveItInterface::chainPlans(std::vector<GroupPlanPair> plans)
{
	ROS_INFO("chaining");
	std::vector<GroupPlanPair> chainedPlans;
	move_group_interface::MoveGroup::Plan plan;
	if(plans.size() > 0)
	{
		if(plans.size() == 1)
		{
			chainedPlans = plans;
		}
		else
		{
			plan = plans[0].second;

			for(size_t i = 1; i < plans.size(); i++)
			{
				if((plans[i].first != plans[i-1].first))
				{
					ROS_INFO("Found new Group in Plan. Generation new Chain");
					chainedPlans.push_back(GroupPlanPair(plans[i-1].first, plan));
					plan = plans[i].second;
					if(i == plans.size()-1)
						chainedPlans.push_back(GroupPlanPair(plans[i].first, plan));
					continue;
				}
				ros::Duration time_last_plan_point;
				time_last_plan_point = plan.trajectory_.joint_trajectory.points.back().time_from_start;
				for(size_t j = 0; j < plans[i].second.trajectory_.joint_trajectory.points.size(); j++)
				{
					plans[i].second.trajectory_.joint_trajectory.points[j].time_from_start += time_last_plan_point;
					if(plans[i].second.trajectory_.joint_trajectory.points[j].time_from_start != time_last_plan_point)
						plan.trajectory_.joint_trajectory.points.push_back(plans[i].second.trajectory_.joint_trajectory.points[j]);
				}
				if(i == plans.size()-1)
					chainedPlans.push_back(GroupPlanPair(plans[i].first, plan));
			}
		}
	}
	ROS_INFO("chained");
	return chainedPlans;
}

move_group_interface::MoveGroup::Plan MoveItInterface::mergePlans(move_group_interface::MoveGroup::Plan plan1, move_group_interface::MoveGroup::Plan plan2)
{
	move_group_interface::MoveGroup::Plan mergedPlan;
	mergedPlan = plan1;
	mergedPlan.trajectory_.joint_trajectory.joint_names.insert(mergedPlan.trajectory_.joint_trajectory.joint_names.end(),
			plan2.trajectory_.joint_trajectory.joint_names.begin(),
			plan2.trajectory_.joint_trajectory.joint_names.end());

	size_t i;
	for(i = 0; (i < mergedPlan.trajectory_.joint_trajectory.points.size())
				&& (i < plan2.trajectory_.joint_trajectory.points.size()); i++)
	{
		mergedPlan.trajectory_.joint_trajectory.points[i].accelerations.insert(
				mergedPlan.trajectory_.joint_trajectory.points[i].accelerations.end(),
				plan2.trajectory_.joint_trajectory.points[i].accelerations.begin(),
				plan2.trajectory_.joint_trajectory.points[i].accelerations.end());

		mergedPlan.trajectory_.joint_trajectory.points[i].positions.insert(
						mergedPlan.trajectory_.joint_trajectory.points[i].positions.end(),
						plan2.trajectory_.joint_trajectory.points[i].positions.begin(),
						plan2.trajectory_.joint_trajectory.points[i].positions.end());

		mergedPlan.trajectory_.joint_trajectory.points[i].velocities.insert(
						mergedPlan.trajectory_.joint_trajectory.points[i].velocities.end(),
						plan2.trajectory_.joint_trajectory.points[i].velocities.begin(),
						plan2.trajectory_.joint_trajectory.points[i].velocities.end());
	}
	if(plan1.trajectory_.joint_trajectory.points.size() > plan2.trajectory_.joint_trajectory.points.size())
	{
		for(size_t j = i; j < plan1.trajectory_.joint_trajectory.points.size(); j++)
		{
			mergedPlan.trajectory_.joint_trajectory.points[j].accelerations.insert(
							mergedPlan.trajectory_.joint_trajectory.points[j].accelerations.end(),
							plan2.trajectory_.joint_trajectory.points.back().accelerations.begin(),
							plan2.trajectory_.joint_trajectory.points.back().accelerations.end());

			mergedPlan.trajectory_.joint_trajectory.points[j].positions.insert(
							mergedPlan.trajectory_.joint_trajectory.points[j].positions.end(),
							plan2.trajectory_.joint_trajectory.points.back().positions.begin(),
							plan2.trajectory_.joint_trajectory.points.back().positions.end());

			mergedPlan.trajectory_.joint_trajectory.points[j].velocities.insert(
							mergedPlan.trajectory_.joint_trajectory.points[j].velocities.end(),
							plan2.trajectory_.joint_trajectory.points.back().velocities.begin(),
							plan2.trajectory_.joint_trajectory.points.back().velocities.end());
		}
	}
	if(plan1.trajectory_.joint_trajectory.points.size() < plan2.trajectory_.joint_trajectory.points.size())
	{
		trajectory_msgs::JointTrajectoryPoint point;
		for(size_t j = i; j < plan2.trajectory_.joint_trajectory.points.size(); j++)
		{
			point  = mergedPlan.trajectory_.joint_trajectory.points.back();

			point.accelerations.insert(
					point.accelerations.end(),
					plan2.trajectory_.joint_trajectory.points[j].accelerations.begin(),
					plan2.trajectory_.joint_trajectory.points[j].accelerations.end());

			point.positions.insert(
					point.positions.end(),
					plan2.trajectory_.joint_trajectory.points[j].positions.begin(),
					plan2.trajectory_.joint_trajectory.points[j].positions.end());

			point.velocities.insert(
					point.velocities.end(),
					plan2.trajectory_.joint_trajectory.points[j].velocities.begin(),
					plan2.trajectory_.joint_trajectory.points[j].velocities.end());
			point.time_from_start = plan2.trajectory_.joint_trajectory.points[j].time_from_start;

			mergedPlan.trajectory_.joint_trajectory.points.push_back(point);
		}
	}

	return mergedPlan;
}

void MoveItInterface::speedup(move_group_interface::MoveGroup::Plan &plan, double factor)
{
	for(size_t i = 0; i < plan.trajectory_.joint_trajectory.points.size(); i++)
	{
		plan.trajectory_.joint_trajectory.points[i].time_from_start *= 1/factor;
	}
}

move_group_interface::MoveGroup* MoveItInterface::getMoveGroup(std::string groupName)
{
	move_group_interface::MoveGroup* group;
	if(_moveGroups.find(groupName) == _moveGroups.end())
	{
		group = new move_group_interface::MoveGroup(groupName);
		_moveGroups.insert(std::pair<std::string, move_group_interface::MoveGroup*>(groupName, group));
	}
	else
	{
		group = _moveGroups[groupName];
	}
	return group;
}

void MoveItInterface::addGround()
{
	ROS_INFO("adding ground collision");

	geometry_msgs::PoseStamped pose;
	pose.pose.position.x = 0;
	pose.pose.position.y = 0;
	pose.pose.position.z = -0.0501;
	pose.pose.orientation.x = 0;
	pose.pose.orientation.y = 0;
	pose.pose.orientation.z = 0;
	pose.pose.orientation.w = 1;
	pose.header.stamp = ros::Time().now();
	pose.header.frame_id = "base_link";

	moveit_msgs::CollisionObject co;
	co.operation = moveit_msgs::CollisionObject::ADD;
	co.id = "ground";
	co.header.frame_id = "base_link";
	co.header.stamp = ros::Time().now();
	shape_msgs::SolidPrimitive box;
	box.type = shape_msgs::SolidPrimitive::BOX;
	box.dimensions.push_back(3.0);
	box.dimensions.push_back(3.0);
	box.dimensions.push_back(0.1);
	co.primitives.push_back(box);
	co.primitive_poses.push_back(pose.pose);

	moveit_msgs::AttachedCollisionObject aco;
	aco.link_name = "base_link";
	aco.touch_links.push_back("base_link");
	aco.object = co;
	_pubAco.publish(aco);
}

void MoveItInterface::addTable()
{
	ROS_INFO("adding table collision");
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0.85;
        pose.pose.position.y = 0.0;
        pose.pose.position.z = 0.3;
        pose.pose.orientation.x = 0;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = 0;
        pose.pose.orientation.w = 1;
        pose.header.stamp = ros::Time().now();
        pose.header.frame_id = "base_link";

        moveit_msgs::CollisionObject co;
        co.operation = moveit_msgs::CollisionObject::ADD;
        co.id = "table";
        co.header.frame_id = "base_link";
        co.header.stamp = ros::Time().now();
        shape_msgs::SolidPrimitive box;
        box.type = shape_msgs::SolidPrimitive::BOX;
        box.dimensions.push_back(0.86);
        box.dimensions.push_back(1.30);
        box.dimensions.push_back(0.6);
        co.primitives.push_back(box);
        co.primitive_poses.push_back(pose.pose);

        moveit_msgs::AttachedCollisionObject aco;
        aco.link_name = "base_link";
        aco.touch_links.push_back("base_link");
        aco.object = co;
        _pubAco.publish(aco);

}

void MoveItInterface::addTable2()
{
	ROS_INFO("adding table collision");
	geometry_msgs::PoseStamped pose;
	pose.pose.position.x = 0.720;
	pose.pose.position.y = -1.412;
	pose.pose.position.z = 0.38;
	pose.pose.orientation.x = 0;
	pose.pose.orientation.y = 0;
	pose.pose.orientation.z = 0;
	pose.pose.orientation.w = 1;
	pose.header.stamp = ros::Time().now();
	pose.header.frame_id = "base_link";

	moveit_msgs::CollisionObject co;
	co.operation = moveit_msgs::CollisionObject::ADD;
	co.id = "table2";
	co.header.frame_id = "base_link";
	co.header.stamp = ros::Time().now();
	shape_msgs::SolidPrimitive box;
	box.type = shape_msgs::SolidPrimitive::BOX;
	box.dimensions.push_back(0.8);
	box.dimensions.push_back(2.0);
	box.dimensions.push_back(0.76);
	co.primitives.push_back(box);
	co.primitive_poses.push_back(pose.pose);

	moveit_msgs::AttachedCollisionObject aco;
	aco.link_name = "base_link";
	aco.touch_links.push_back("base_link");
	aco.object = co;
	_pubAco.publish(aco);
}

void MoveItInterface::addPallet()
{
	ROS_INFO("adding pallet collision");
	geometry_msgs::PoseStamped pose;
	pose.pose.position.x = 0.735;
	pose.pose.position.y = -0.004;
	pose.pose.position.z = 0.7575;
	pose.pose.orientation.x = 0;
	pose.pose.orientation.y = 0;
	pose.pose.orientation.z = 0;
	pose.pose.orientation.w = 1;
	pose.header.stamp = ros::Time().now();
	pose.header.frame_id = "base_link";

	moveit_msgs::CollisionObject co;
	co.operation = moveit_msgs::CollisionObject::ADD;
	co.id = "pallet";
	co.header.frame_id = "base_link";
	co.header.stamp = ros::Time().now();
	shape_msgs::SolidPrimitive box;
	box.type = shape_msgs::SolidPrimitive::BOX;
	box.dimensions.push_back(0.2);
	box.dimensions.push_back(0.4);
	box.dimensions.push_back(0.045);
	co.primitives.push_back(box);
	co.primitive_poses.push_back(pose.pose);

	moveit_msgs::AttachedCollisionObject aco;
	aco.link_name = "base_link";
	aco.touch_links.push_back("base_link");
	aco.object = co;
	_pubAco.publish(aco);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "prace_moveit_interface");
	ros::NodeHandle node;

	MoveItInterface* moveItInterface = new MoveItInterface(node);

	ros::spin();

	delete moveItInterface;

	return 0;
}
