//#region [include]

#include "drone_middleware/PoseListener.h"

//#endregion

//#region [main]

int main(int argc, char **argv)
{ 
  //prestar atencao na ordem da chamada das funções

  namespace dm = drone_middleware;

  bool heatmap_active;
  double heatmap_x_min, heatmap_y_min, heatmap_width;
  int heatmap_n_cells;

  ros::init(argc, argv, "pose_listener");

  if(dm::PoseListener::setAndGetNDrones("/drone_middleware/n_drones") == -1)
    ros::shutdown();

  //dm::PoseListener::resetDroneArray();

  if(!ros::param::get("/drone_middleware/heatmap/active", heatmap_active))
  {
    ROS_WARN("[PARAM] Unable to get the ROS parameter %s", "/drone_middleware/heatmap/active");
    heatmap_active = false;
  }
  else if(heatmap_active)
  {
    if(!ros::param::get("/drone_middleware/heatmap/x_min", heatmap_x_min))
    {
      ROS_ERROR("[PARAM] Unable to get the ROS parameter %s", "/drone_middleware/heatmap/x_min");
      heatmap_active = false;
    }
    if(!ros::param::get("/drone_middleware/heatmap/y_min", heatmap_y_min))
    {
      ROS_ERROR("[PARAM] Unable to get the ROS parameter %s", "/drone_middleware/heatmap/y_min");
      heatmap_active = false;
    }
    if(!ros::param::get("/drone_middleware/heatmap/width", heatmap_width))
    {
      ROS_ERROR("[PARAM] Unable to get the ROS parameter %s", "/drone_middleware/heatmap/width");
      heatmap_active = false;
    }
    if(!ros::param::get("/drone_middleware/heatmap/n_cells", heatmap_n_cells))
    {
      ROS_ERROR("[PARAM] Unable to get the ROS parameter %s", "/drone_middleware/heatmap/n_cells");
      heatmap_active = false;
    }
  }

  dm::PoseListener * listener = 
    new dm::PoseListener("pose_listener", "pose_request", "drone", 
    "/ground_truth_to_tf/pose", 100, "get_heatmap_data", (uint8_t) heatmap_active, 
    heatmap_x_min, heatmap_y_min, heatmap_width, (uint16_t) heatmap_n_cells);
    
  int NODES_ONLINE;
  //pegando parametro do numero de drones
  if(!ros::param::get("/drone_middleware/nodes_online", NODES_ONLINE)) NODES_ONLINE = 0;
    ros::param::set("/drone_middleware/nodes_online", NODES_ONLINE+1);

  ROS_INFO("[READY] pose_listener is ready.");

  //loop do ROS
  ros::spin();

  return 0;
}

//#endregion