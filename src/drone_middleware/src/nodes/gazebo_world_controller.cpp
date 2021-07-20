//#region [include]

#include "drone_middleware/WorldController.h"

//#endregion

//#region [main]

int main(int _argc, char **_argv)
{

  namespace dm = drone_middleware;

  //boilerplate necessario do ros
  ros::init(_argc, _argv, "gazebo_world_controller");

  //boilerplate necessario do gazebo
  gazebo::client::setup(_argc, _argv);

  //construtor
  dm::WorldController * gazeboController = 
    new dm::WorldController("gazebo_world_controller", 
    "pause", "unpause", "restart", "get_sim_info", "multi_step",
    "~/world_stats", "~/world_control", "~/physics", 
    100, 1000);

  //aguardar conexao com o gazebo
  gazeboController->waitConnectionOnPublishers();

  //pause e reset
  gazeboController->restart();

  //fisica colocada para rodar o mais rapido possivel
  gazeboController->setRealTimeUpdateRate(0.0);

  int NODES_ONLINE;
  //pegando parametro do numero de drones
  if(!ros::param::get("/drone_middleware/nodes_online", NODES_ONLINE)) NODES_ONLINE = 0;
  ros::param::set("/drone_middleware/nodes_online", NODES_ONLINE+1);
  
  //mensagens
  ROS_INFO("[READY] gazebo_world_controller is ready.");
  
  //loop do ROS
  ros::spin();

  //parar e limpar simulacao do gazebo
  gazebo::client::shutdown();
  
  return 0;
}

//#endregion
