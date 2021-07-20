//#region [include]

#include "drone_middleware/MobilityHeatMap.h"

//#endregion

namespace drone_middleware
{

  //#region [class]

  //construtor
  MobilityHeatMap::MobilityHeatMap(bool active, double x_min, double y_min, double width, 
    uint16_t n_cells, uint16_t numberDrones)
  {
    this->active = active;
    this->x_min = x_min;
    this->y_min = y_min;
    this->width = width;
    this->n_cells = n_cells;
    this->numberDrones = numberDrones;

    //ROS_INFO("[DEBUG] %i %f %f %f %i", active, x_min, y_min, width, n_cells);

    this->mul_factor = (double) n_cells/width;

    for(int i = 0; i < numberDrones; i++)
    {
      //criando last cells e preenchendo com maior valor possível para evitar problemas na primeira checagem
      drone_middleware::HeatMapCell temp;
      temp.i = std::numeric_limits<unsigned short>::max();
      temp.j = temp.i;
      this->dronesLastCells.push_back(temp);
    }

    //criando o heatmap vazio
    for(int i = 0; i < n_cells; i++)
    {   
      std::vector<uint16_t> temp;
      for(int j = 0; j < n_cells; j++)
      {
          temp.push_back(0);
      }
      this->heatMap.push_back(temp);
    }
  }

  //#endregion

  //#region [func]

  void MobilityHeatMap::incrementCell(drone_middleware::HeatMapCell cell)
  {
    if(cell.i < this->n_cells && cell.j < this->n_cells)
      this->heatMap[cell.i][cell.j]++;
    else
      ROS_WARN("Tried to modify values out of heat map bounds, check your heatmap parameters!");
  }

  drone_middleware::HeatMapCell MobilityHeatMap::getCellForPoint(double x, double y)
  {
    drone_middleware::HeatMapCell temp;

    //j = math.floor((x - self.x_min) * self.mul_factor)
    //i = math.floor((y - self.y_min) * self.mul_factor)

    temp.j = (uint16_t) ((double) (x - this->x_min) * this->mul_factor);
    temp.i = (uint16_t) ((double) (y - this->y_min) * this->mul_factor);

    return temp;
  }

  std::vector<uint16_t> MobilityHeatMap::getHeatMapAsVector()
  {
    std::vector<uint16_t> temp;

    //ROS_INFO("[DEBUG] %i %f %f %f %i", this->active, this->x_min, this->y_min, this->width, this->n_cells);

    for(int i = 0; i < n_cells; i++)
    {   
      for(int j = 0; j < n_cells; j++)
      {
        temp.push_back(this->heatMap[i][j]);
      }
    }

    return temp;
  }

  //! Getter da variável active.
  uint8_t MobilityHeatMap::is_active() 
  {
    return this->active;
  }
  
  //! Getter da variável x_min.
  double MobilityHeatMap::get_x_min() 
  {
    return this->x_min;
  }

  //! Getter da variável y_min.
  double MobilityHeatMap::get_y_min() 
  {
    return this->y_min;
  }

  //! Getter da variável width.
  double MobilityHeatMap::get_width() 
  {
    return this->width;
  }

  //! Getter da variável n_cells.
  uint16_t MobilityHeatMap::get_n_cells() 
  {
    return this->n_cells;
  }

  //! Getter das last cells.
  drone_middleware::HeatMapCell MobilityHeatMap::getDroneLastCell(uint16_t droneNumber) 
  {
    return this->dronesLastCells[droneNumber];
  }

  //! Setter das last cells
  void MobilityHeatMap::setDroneLastCell(uint16_t droneNumber, drone_middleware::HeatMapCell cell)
  {
    this->dronesLastCells[droneNumber] = cell;
  }

  //#endregion

  //#region [callback]

  //#endregion

}