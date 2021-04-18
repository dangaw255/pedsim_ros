/*
 * @name	 	scene_services.cpp
 * @brief	 	Provides services to spawn and remove pedestrians dynamically. 
 *          The spawned agents are forwarded to flatland
 * @author 	Ronja Gueldenring
 * @date 		2019/04/05
 **/

#include <pedsim_simulator/element/agent.h>
#include <pedsim_simulator/element/agentcluster.h>
#include <pedsim_simulator/element/areawaypoint.h>
#include <pedsim_simulator/scene_services.h>
#include <geometry_msgs/Point.h>
#include <std_srvs/SetBool.h>
#include <flatland_msgs/SpawnModels.h>
#include <flatland_msgs/DeleteModels.h>
#include <iostream>
#include <ros/package.h>

int SceneServices::agents_index_ = 0;

SceneServices::SceneServices() {
  //Pedsim service
  spawn_peds_service_ = nh_.advertiseService("pedsim_simulator/spawn_peds", &SceneServices::spawnPeds, this);
  reset_peds_service_ = nh_.advertiseService("pedsim_simulator/reset_all_peds", &SceneServices::resetPeds, this);
  
  //flatland service clients
  spawn_models_service_name_ = ros::this_node::getNamespace() + "/spawn_models";
  spawn_models_client_ = nh_.serviceClient<flatland_msgs::SpawnModels>(spawn_models_service_name_, true);
  delete_models_service_name_ = ros::this_node::getNamespace() + "/delete_models";
  delete_models_client_ = nh_.serviceClient<flatland_msgs::DeleteModels>(delete_models_service_name_, true);
}


bool SceneServices::spawnPeds(pedsim_srvs::SpawnPeds::Request &request, pedsim_srvs::SpawnPeds::Response &response) {
  flatland_msgs::SpawnModels srv;

  for (int ped_i = 0; ped_i < (int) request.peds.size(); ped_i++) {
    // add ped to pedsim
    pedsim_msgs::Ped ped = request.peds[ped_i];
    AgentCluster* agentCluster = addAgentClusterToPedsim(ped);

    // add flatland models to spawn_models service request
    std::vector<flatland_msgs::Model> new_models = getFlatlandModelsFromAgentCluster(agentCluster, ped.yaml_file);
    srv.request.models.insert(srv.request.models.end(), new_models.begin(), new_models.end());
  }

  // make sure client is valid
  while (!spawn_models_client_.isValid()) {
    ROS_WARN("Reconnecting to flatland spawn_models service...");
    spawn_models_client_.waitForExistence(ros::Duration(5.0));
    spawn_models_client_ = nh_.serviceClient<flatland_msgs::SpawnModels>(spawn_models_service_name_, true);
  }

  // call spawn_models service
  spawn_models_client_.call(srv);
  if (srv.response.success) {
    response.success = true;
    ROS_INFO("Successfully called flatland spawn_models service");
  } else {
    response.success = false;
    ROS_ERROR("Failed to spawn all %ld agents", request.peds.size());
  }
  
  return true;
}


bool SceneServices::resetPeds(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response) {
  if (request.data)
  {
    QList<Agent*> agents = SCENE.getAgents();
    for (Agent* a : agents)
    {
      a->reset();
    }
  }

  response.success = true;
  return true;
}


AgentCluster* SceneServices::addAgentClusterToPedsim(pedsim_msgs::Ped ped) {
  // create agentcluster
  const double x = ped.pos.x;
  const double y = ped.pos.y;
  const int n = ped.number_of_peds;
  AgentCluster* agentCluster = new AgentCluster(x, y, n);

  // set distribution
  const double dx = 2;
  const double dy = 2;
  agentCluster->setDistribution(dx, dy);

  // set type
  const int type = ped.type;
  agentCluster->setType(static_cast<Ped::Tagent::AgentType>(type));

  // set vmax
  agentCluster->vmax = ped.vmax;

  // set action probabilities
  agentCluster->chattingProbability = ped.chatting_probability;
  agentCluster->tellStoryProbability = ped.tell_story_probability;
  agentCluster->groupTalkingProbability = ped.group_talking_probability;
  agentCluster->talkingAndWalkingProbability = ped.talking_and_walking_probability;
  agentCluster->stateTalkingBaseTime = ped.talking_base_time;
  agentCluster->stateTellStoryBaseTime = ped.tell_story_base_time;
  agentCluster->stateGroupTalkingBaseTime = ped.group_talking_base_time;
  agentCluster->stateTalkingAndWalkingBaseTime = ped.talking_and_walking_base_time;

  // set max talking distance
  agentCluster->maxTalkingDistance = ped.max_talking_distance;

  // set waypoint mode
  int waypoint_mode = ped.waypoint_mode;
  agentCluster->waypoint_mode = static_cast<Agent::WaypointMode>(waypoint_mode);

  // set force factors
  agentCluster->forceFactorDesired = ped.force_factor_desired;
  agentCluster->forceFactorObstacle = ped.force_factor_obstacle;
  agentCluster->forceFactorSocial = ped.force_factor_social;

  // add waypoints to agentcluster and scene
  for(int i = 0; i < (int) ped.waypoints.size(); i++){
    QString id;
    id.sprintf("%d_%d", ped.id, i);
    const double x = ped.waypoints[i].x;
    const double y = ped.waypoints[i].y;
    const double r = ped.waypoints[i].z;
    AreaWaypoint* w = new AreaWaypoint(id, x, y, r);
    w->setBehavior(static_cast<Ped::Twaypoint::Behavior>(0));
    SCENE.addWaypoint(w);
    agentCluster->addWaypoint(w);
  }

  // add agentcluster to scene
  SCENE.addAgentCluster(agentCluster);

  return agentCluster;
}


std::vector<flatland_msgs::Model> SceneServices::getFlatlandModelsFromAgentCluster(AgentCluster* agentCluster, std::string yaml_file){
  std::vector<flatland_msgs::Model> flatland_msg;

  std::vector<std::string> names = agentCluster->generate_agent_names();

  for (int i = 0; i < agentCluster->getCount(); i++){
    flatland_msgs::Model model;
    model.yaml_path = yaml_file;
    model.name = names[i];
    model.ns = "pedsim_agent_" +  std::to_string(agents_index_);
    agents_index_++;
    model.pose.x = agentCluster->getPosition().x;
    model.pose.y = agentCluster->getPosition().y;
    flatland_msg.push_back(model);
  }  

  return flatland_msg;
}
