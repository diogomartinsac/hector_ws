//#region [include]

#include "drone_middleware/TeleoperatorCSV.h"

//#endregion

namespace drone_middleware
{

  //#region [class]

  TeleoperatorCSV::TeleoperatorCSV(std::string serverName, std::string serviceName, std::string path) :
    Teleoperator(serverName, serviceName)
  {

    this->path = path;

    //abertura do arquivo
    csv_file.open(path);
    if(!csv_file.is_open())
    {
      ROS_ERROR("[FILE] Unable to open CSV file.");
      ros::shutdown();
    }

    std::string temp;
    
    //descartando a primeira linha
    std::getline(csv_file, temp);

  }

  //#endregion

  //#region [func]

  std::string TeleoperatorCSV::getPath(int argc, char **argv, std::string paramName)
  {
    std::string path = "";

    //captacao do nome do arquivo
    if(argc < 2)
    {

      if(ros::param::get("/drone_middleware/csv_path", path));
      else
      {
        ROS_INFO("[FILE] Inform path to CSV file: ");
        std::cin >> path;
      }
      
    } 
    else 
    {
      path = argv[1];
    }

    return path;
  }

  bool TeleoperatorCSV::checkCSVGood()
  {
    return this->csv_file.good();
  }

  bool TeleoperatorCSV::getOneInput(drone_middleware::ScheduleMovement &srv_msg)
  {

    std::string temp;

    getline(this->csv_file, temp, ',');
    srv_msg.request.startAt.sec = ros::Time(std::atof(temp.c_str())).sec;
    srv_msg.request.startAt.nsec = ros::Time(std::atof(temp.c_str())).nsec;
    getline(this->csv_file, temp, ',');
    srv_msg.request.droneNumber = std::atoi(temp.c_str());
    getline(this->csv_file, temp, ',');
    srv_msg.request.movement.linear.x = std::atof(temp.c_str());
    getline(this->csv_file, temp, ',');
    srv_msg.request.movement.linear.y = std::atof(temp.c_str());
    getline(this->csv_file, temp, ',');
    srv_msg.request.movement.linear.z = std::atof(temp.c_str());
    getline(this->csv_file, temp, ',');
    srv_msg.request.movement.angular.x = std::atof(temp.c_str());
    getline(this->csv_file, temp, ',');
    srv_msg.request.movement.angular.y = std::atof(temp.c_str());
    getline(this->csv_file, temp);
    srv_msg.request.movement.angular.z = std::atof(temp.c_str());

    return this->checkCSVGood();
  }

  int TeleoperatorCSV::enterLoop()
  {
    //leitura e insercao na fila
    drone_middleware::ScheduleMovement srv_msg;
    
    ROS_INFO("[FILE] Reading CSV file.");
    
    while(this->getOneInput(srv_msg))
    {

      this->callSchedulerServer(srv_msg);
      //loop do ROS
      ros::spinOnce();
    }
    
    ROS_INFO("[FILE] CSV file end reached.");

    return 0;
  }

  //#endregion

}