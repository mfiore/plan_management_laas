#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <libhatp/hatpPlan.h>
#include <hatp_msgs/Plan.h>
#include <hatp_msgs/PlanningRequest.h>

#include <plan_management_msgs/ManageSequentialPlanAction.h>
#include <plan_management_msgs/SequentialTaskPlan.h>
#include <plan_management_msgs/ManageGoalAction.h>

#include <boost/thread/locks.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/shared_ptr.hpp>

#include <vector>
#include <string>


using namespace std;


typedef actionlib::SimpleActionServer<plan_management_msgs::ManageGoalAction> Server;
typedef actionlib::SimpleActionClient<plan_management_msgs::ManageSequentialPlanAction> Client;


boost::mutex mutex_sequential_plan_done_;
bool sequential_plan_done_;
map<string,vector<string> > hatp_parameter_name_;
map<string,string> hatp_to_shary_action_name_;

ros::ServiceClient hatp_client_; 

void buildParameterMap(ros::NodeHandle node_handle) {
    vector<string> hatp_actions;
    node_handle.getParam("/hatp_goal_manager/action_list",hatp_actions);
    ROS_INFO("%ld",hatp_actions.size());
    for (int i=0;i<hatp_actions.size();i++) {
    	vector<string> parameters;
    	string name="/hatp_goal_manager/"+hatp_actions[i]+"/parameters";
    	node_handle.getParam(name,parameters);
    	hatp_parameter_name_[hatp_actions[i]]=parameters;

    	ROS_INFO("HATP_GOAL_MANAGER Action %s parameters are:",hatp_actions[i].c_str());
    	for (int j=0; j<parameters.size(); j++) {
    		ROS_INFO("HATP_GOAL_MANAGER %s",parameters[j].c_str());
    	}
    	string shary_name;
    	node_handle.getParam("/hatp_goal_manager/"+hatp_actions[i]+"/shary_name",shary_name);
    	hatp_to_shary_action_name_[hatp_actions[i]]=shary_name;

    	ROS_INFO("HATP_GOAL_MANAGER shary name is %s",shary_name.c_str());
    }

}


plan_management_msgs::SequentialTaskPlan convertPlan(hatpPlan *hatp_plan) {
	plan_management_msgs::SequentialTaskPlan sequential_plan;
	sequential_plan.plan_id=0;
	
	vector<string> actors=hatp_plan->getAgentsList();
	sequential_plan.actors=actors;
	
	hatpStream *streams=hatp_plan->getStreams();
	map<string,vector<hatpTask*> > agent_streams=streams->getAgentsStreams();

	for (int i=0;i<actors.size();i++) {
		plan_management_msgs::SequentialPlanActionList action_list; //actions of the agent
		action_list.main_actor=actors[i];

		vector<hatpTask*> agent_tasks=agent_streams[actors[i]]; //we take the stream of the agent and convert it to the shary3 format
		ROS_INFO("HATP_GOAL_MANAGER agent %s",actors[i].c_str());
		for (int j=0; j<agent_tasks.size();j++) {
			plan_management_msgs::SequentialPlanAction plan_action;
			
			plan_action.main_actor=actors[i];
			ROS_INFO("HATP_GOAL_MANAGER action links are");
			vector<unsigned int> stream_predecessors=agent_tasks[j]->getStreamPredecessorsIds();
			for (int k=0; k<stream_predecessors.size();k++) {
				plan_action.action_links.push_back(stream_predecessors[k]);
				ROS_INFO("HATP_GOAL_MANAGER %d",stream_predecessors[k]);
			}
			
			plan_action.action.name=agent_tasks[j]->getName();
			plan_action.action.id=agent_tasks[j]->getID();
			
			ROS_INFO("HATP_GOAL_MANAGER action name and id are %s %d",plan_action.action.name.c_str(),
				plan_action.action.id);

			vector<common_msgs::Parameter> parameters;
			vector<string> task_parameters=agent_tasks[j]->getParameters();
			ROS_INFO("HATP_GOAL_MANAGER parameters are");
			for (int k=0;k<task_parameters.size();k++) {
				common_msgs::Parameter p;
				p.name=hatp_parameter_name_[plan_action.action.name][k];
				p.value=task_parameters[k];
				ROS_INFO("HATP_GOAL_MANAGER %s %s",p.name.c_str(),p.value.c_str());
			} 
			action_list.actions.push_back(plan_action);
		}
		sequential_plan.actions.push_back(action_list);
	}
	return sequential_plan;
}


map<string,string> getParametersMap(vector<common_msgs::Parameter> parameters_msg) {
	map<string,string> parameter_map;
	for (int i=0;i<parameters_msg.size();i++) {
		parameter_map[parameters_msg[i].name]=parameters_msg[i].value;
	}
	return parameter_map;
}

hatp_msgs::PlanningRequest convertToHatpRequest(const plan_management_msgs::ManageGoalGoalConstPtr& goal) {
	hatp_msgs::PlanningRequest hatp_srv;
	hatp_srv.request.request.type="first-plan";
	hatp_srv.request.request.id = 0;
	
	hatp_srv.request.request.task=goal->goal;;
	map<string,string> parameter_map=getParametersMap(goal->parameters);
	vector<string> hatp_action_parameters=hatp_parameter_name_[goal->goal];
	for (int i=0;i<hatp_action_parameters.size();i++) {
		hatp_srv.request.request.parameters.push_back(parameter_map.at(hatp_action_parameters[i]));
	}
	return hatp_srv;
}

bool isSequentialPlanDone() {
	boost::lock_guard<boost::mutex> lock(mutex_sequential_plan_done_);
	return sequential_plan_done_;
}


void sequentialPlanDoneCallback(const actionlib::SimpleClientGoalState& state,
	const plan_management_msgs::ManageSequentialPlanResultConstPtr& result) {
	boost::lock_guard<boost::mutex> lock(mutex_sequential_plan_done_);
	sequential_plan_done_=true;
}

void manageGoal(const plan_management_msgs::ManageGoalGoalConstPtr& goal, Server *goal_server, Client *action_client) {
	plan_management_msgs::ManageGoalFeedback feedback;
	plan_management_msgs::ManageGoalResult result;


	//ask for plan

	hatp_msgs::PlanningRequest hatp_srv=convertToHatpRequest(goal);		

	if(!hatp_client_.call(hatp_srv)){
		ROS_ERROR("HATP_GOAL_MANAGER Failed to call service 'Planner'");
		common_msgs::Report report;
		report.status="FAILED";
		report.details="couldn't contact hatp";
		result.report=report;
		goal_server->setAborted(result);
		return;		
	}
	hatpPlan hatp_plan(hatp_srv.response.solution);
	
	//Convert plan to our structures
	if (hatp_plan.isReportOK()) {
		common_msgs::Report report;
		report.status="RUNNING";
		report.details="received plan";
		feedback.report=report;
		goal_server->publishFeedback(feedback);

		ROS_INFO("HATP_GOAL_MANAGER Received plan");

		plan_management_msgs::SequentialTaskPlan sequential_plan=convertPlan(&hatp_plan);
		plan_management_msgs::ManageSequentialPlanGoal sequential_plan_goal;
		sequential_plan_goal.task_plan=sequential_plan;

		ROS_INFO("HATP_GOAL_MANAGER Sending goal to sequential_plan_manager");



		action_client->sendGoal(sequential_plan_goal,
			&sequentialPlanDoneCallback,
			Client::SimpleActiveCallback(),
			Client::SimpleFeedbackCallback());
		ROS_INFO("HATP_GOAL_MANAGER Goal sent, waiting for results");
		ros::Rate r(3);
		while (!isSequentialPlanDone() && ros::ok() && !goal_server->isPreemptRequested()) {
			r.sleep();
		}
		if (!ros::ok()) {
			goal_server->setAborted();
			return;
		}
		if (goal_server->isPreemptRequested()) {
			action_client->cancelGoal();
			ROS_INFO("HATP_GOAL_MANAGER server preempted");
			goal_server->setPreempted();
			return;
		}
		plan_management_msgs::ManageSequentialPlanResultConstPtr sequential_plan_result=action_client->getResult();
		ROS_INFO("HATP_GOAL_MANAGER Received result %s",sequential_plan_result->plan_report.report.status.c_str());
		if (sequential_plan_result->plan_report.report.status=="COMPLETED") {
			common_msgs::Report report;
			report.status="SUCCEDED";
			result.report=report;
			ROS_INFO("HATP_GOAL_MANAGER goal completed");
			goal_server->setSucceeded(result);
		}
		else {
			common_msgs::Report report;
			report.status="FAILED";
			report.details=sequential_plan_result->plan_report.report.details;
			ROS_INFO("HATP_GOAL_MANAGER goal failed");
			goal_server->setAborted(result);
		}
	}
	else {
		common_msgs::Report report;
		report.status="FAILED";
		report.details=hatp_plan.getReport();
		result.report=report;

		ROS_INFO("HATP_GOAL_MANAGER Goal failed: status %s details %s",report.status.c_str(),report.details.c_str());
		goal_server->setAborted(result);
	}
}

int main (int argc, char **argv) {
	ros::init(argc,argv,"hatp_goal_manager");
	ros::NodeHandle node_handle;

	ROS_INFO("HATP_GOAL_MANAGER Started hatp_goal_manager");

	string hatp_actions;
	node_handle.getParam("/hatp_goal_manager/action_list",hatp_actions);
	ROS_INFO("%s",hatp_actions.c_str());
	buildParameterMap(node_handle);
	
	string robot_name;
	node_handle.getParam("/situation_assessment/robot_name",robot_name);
	ROS_INFO("HATP_GOAL_MANAGER robot_name %s",robot_name.c_str());

	ROS_INFO("HATP_GOAL_MANAGER Waiting for manage_sequential_plan");
	Client action_client("/plan_management/manage_sequential_plan",true);
	ROS_INFO("HATP_GOAL_MANAGER Connected to manage_sequential_plan");

	ROS_INFO("HATP_GOAL_MANAGER Connecting to hatp planning service");
	hatp_client_= node_handle.serviceClient<hatp_msgs::PlanningRequest>("Planner");
	hatp_client_.waitForExistence();
	ROS_INFO("HATP_GOAL_MANAGER Connected");


	Server goal_server("shary3/manage_goal",boost::bind(&manageGoal,_1,&goal_server,&action_client),false);
	goal_server.start();
	ROS_INFO("HATP_GOAL_MANAGER Started goal server");

	ROS_INFO("HATP_GOAL_MANAGER Ready");

	ros::spin();

	return 0;
} 