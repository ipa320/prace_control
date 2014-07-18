#ifndef MOVEITINTERFACE_H
#define MOVEITINTERFACE_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/server/action_server.h>

#include <prace_moveit_interface/LinAction.h>
#include <prace_moveit_interface/PtpAction.h>
#include <prace_moveit_interface/JointSpaceAction.h>
#include <prace_moveit_interface/ExecPlanAction.h>
#include <prace_moveit_interface/SyncMoveAction.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <list>


class MoveItInterface
{
public:
	MoveItInterface(ros::NodeHandle &nodeHandle);
	~MoveItInterface();

private:

	typedef actionlib::SimpleActionServer<prace_moveit_interface::LinAction> SimpleActionLin;
	typedef actionlib::SimpleActionServer<prace_moveit_interface::PtpAction> SimpleActionPtp;
	typedef actionlib::SimpleActionServer<prace_moveit_interface::JointSpaceAction> SimpleActionJointSpace;

	typedef actionlib::ActionServer<prace_moveit_interface::LinAction> ActionLin;
	typedef actionlib::ActionServer<prace_moveit_interface::PtpAction> ActionPtp;
	typedef actionlib::ActionServer<prace_moveit_interface::JointSpaceAction> ActionJointSpace;

	typedef actionlib::SimpleActionServer<prace_moveit_interface::ExecPlanAction> SimpleActionExecPlan;
	typedef actionlib::ActionServer<prace_moveit_interface::SyncMoveAction> ActionSyncMove;

	typedef std::pair<move_group_interface::MoveGroup*, move_group_interface::MoveGroup::Plan> GroupPlanPair;

	enum PlanType {LIN, PTP, JS};

	struct queuePlanType{
		int type;
		prace_moveit_interface::LinGoal linGoal;
		prace_moveit_interface::PtpGoal ptpGoal;
		prace_moveit_interface::JointSpaceGoal jsGoal;
	};

	typedef std::pair<queuePlanType, queuePlanType> SortedQueueType;

	ros::NodeHandle _nh;

	SimpleActionLin* _asMoveLin;
	SimpleActionPtp* _asMovePTP;
	SimpleActionJointSpace* _asMoveJointSpace;

	SimpleActionLin* _asPlanLin;
	SimpleActionPtp* _asPlanPTP;
	SimpleActionJointSpace* _asPlanJointSpace;

	ActionLin* _asQueueLin;
	ActionPtp* _asQueuePTP;
	ActionJointSpace* _asQueueJointSpace;

	SimpleActionExecPlan* _asExecPlan;
	SimpleActionExecPlan* _asExecQueue;
	ActionSyncMove* _asSyncMove;

	tf::TransformListener* transformer;
	tf::TransformBroadcaster* broadcaster;
	tf::Transform _needleTransform;
	tf::Transform _dropTransform;

	ros::Publisher _pubPose;
	ros::Publisher _pubAco;
	ros::Publisher _displayPathPublisher;

	std::string _moveLinNs, _planLinNs;
	std::string _movePTPNs, _planPTPNs;
	std::string _moveJointSpaceNs, _planJointSpaceNs;

	std::vector<GroupPlanPair> _plansToExecute;
	robot_state::RobotStatePtr _robotStartState;
	bool _chain_plans;

	std::list<queuePlanType> _queuePlans;

	std::map<std::string, std::string> _lookupMirrorMoveGroup;

	std::map<std::string, move_group_interface::MoveGroup*> _moveGroups;
	std::map<int, std::vector<ActionSyncMove::GoalHandle> > _syncGoals;

	void moveLinCallback(const prace_moveit_interface::LinGoalConstPtr &goal);

	void movePTPCallback(const prace_moveit_interface::PtpGoalConstPtr &goal);

	void moveJointSpaceCallback(const prace_moveit_interface::JointSpaceGoalConstPtr &goal);

	void planLinCallback(const prace_moveit_interface::LinGoalConstPtr &goal);

	void planPTPCallback(const prace_moveit_interface::PtpGoalConstPtr &goal);

	void planJointSpaceCallback(const prace_moveit_interface::JointSpaceGoalConstPtr &goal);

	void queueLinGoalCallback(ActionLin::GoalHandle gh);

	void queuePTPGoalCallback(ActionPtp::GoalHandle gh);

	void queueJointSpaceGoalCallback(ActionJointSpace::GoalHandle gh);

	void queueGoal();

	void executePlansCallback(const prace_moveit_interface::ExecPlanGoalConstPtr &goal);
	bool executePlans();

	void executeQueue(const prace_moveit_interface::ExecPlanGoalConstPtr &goal);

	std::vector<SortedQueueType> sortQueue(std::list<queuePlanType> queue);

	void syncGoalCallback(ActionSyncMove::GoalHandle gh);

	void syncCancelCallback(ActionSyncMove::GoalHandle gh);

	bool waitfortransform(std::string frame_id, std::string target);

	bool transformLink(geometry_msgs::PoseStamped &goal_trans, geometry_msgs::PoseStamped goal, std::string target = "base_link");

	robot_state::RobotStatePtr getStartState(std::string move_group);

	void updateStartState(move_group_interface::MoveGroup::Plan plan);

	robot_state::RobotStatePtr setStartState(std::string move_group);

	std::vector<GroupPlanPair> chainPlans(std::vector<GroupPlanPair> plans);

	move_group_interface::MoveGroup::Plan mergePlans(move_group_interface::MoveGroup::Plan plan1, move_group_interface::MoveGroup::Plan plan2);

	void speedup(move_group_interface::MoveGroup::Plan &plan, double factor);

	move_group_interface::MoveGroup* getMoveGroup(std::string groupName);

	void addGround();

	void addTable();

	void addTable2();

	void addPallet();
};
#endif
