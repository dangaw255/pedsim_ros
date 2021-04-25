/*
 * @name	 	scene_services.cpp
 * @brief	 	Provides services to spawn and remove pedestrians dynamically. 
 *          The spawned agents are forwarded to flatland
 * @author 	Ronja Gueldenring
 * @date 		2019/04/05
 **/

#ifndef _scene_service_h_
#define _scene_service_h_

#include <ros/ros.h>
#include <pedsim_simulator/scene.h>
#include <pedsim_srvs/SpawnPeds.h>
#include <flatland_msgs/Model.h>
#include <pedsim_msgs/Ped.h>
#include <std_srvs/SetBool.h>

  /**
   * This class provides services to spawn and remove pedestrians dynamically.
   */
class SceneServices {
  // Constructor and Destructor
 public:
  SceneServices();
  virtual ~SceneServices() = default;

  static int agents_index_;

    /**
    * spawn_ped_service_ + spawnPed
    * @brief Spawns pedestrian in pedsim and flatland.
    */
  bool spawnPeds(pedsim_srvs::SpawnPeds::Request &request, pedsim_srvs::SpawnPeds::Response &response);
  bool resetPeds(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response);

 protected:
   ros::NodeHandle nh_;

 private:

  bool spawnStaticObstacles(AgentCluster* cluster);

    /**
    * @brief Adding pedestrian to pedsim.
    * @return added agentcluster
    */
  AgentCluster* addAgentClusterToPedsim(pedsim_msgs::Ped ped);

    /**
    * @brief Get flatland model names from AgentCluster
    * @return added AgentCluster
    */
  std::vector<flatland_msgs::Model> getFlatlandModelsFromAgentCluster(AgentCluster* agentCluster, std::string yaml_file);

  ros::ServiceServer spawn_peds_service_;
  ros::ServiceServer reset_peds_service_;

  std::string spawn_models_service_name_;
  ros::ServiceClient spawn_models_client_;             //Service client to spawn models in flatland

  std::string respawn_models_service_name_;
  ros::ServiceClient respawn_models_client_;            //Service client to respawn models in flatland

  std::string delete_models_service_name_;
  ros::ServiceClient delete_models_client_;            // Service client to remove models in flatland.
};

#endif /* _scene_service_h_ */
