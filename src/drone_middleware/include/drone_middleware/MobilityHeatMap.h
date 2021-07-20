#ifndef CLASS_MOBILITYHEATMAP_H
#define CLASS_MOBILITYHEATMAP_H

//#region [include]

#include <cstdint>
#include <limits>
#include <vector>

#include "ros/ros.h"

//#endregion

namespace drone_middleware
{
	
  //#region [class]

  typedef struct HeatMapCell
  {
      uint16_t i;
      uint16_t j;
  } HeatMapCell;

  //! Classe que armazena dados sobre o heatmap de mobilidade.
  /*!
  *   *	Possui dados do heatmap e funções úteis para sua manipulação.
  *   * O heatmap considera uma área quadrada.
  */
  class MobilityHeatMap
  {
    
    private:
    
    //! Booleano que indica se está ativo.
    /*!
    *   *	Se for falso o heatmap não está sendo usado na simulação e não será preenchido.
    */
    uint8_t active;

    //! Posição x do canto superior esquerdo da área mapeada.
    /*!
    *   *	Indica a posição x do canto superior esquerdo da área considerada para o heatmap.
    */
    double x_min;

    //! Posição y do canto superior esquerdo da área mapeada.
    /*!
    *   *	Indica a posição y do canto superior esquerdo da área considerada para o heatmap.
    */
    double y_min;

    //! Largura do heatmap.
    /*!
    *   *	Indica a largura da área considerada para o heatmap.
    */
    double width;

    //! Tamanho do lado da área em células.
    /*!
    *   *	Indica quantas células formam um lado da área quadrada do heatmap.
    */
    uint16_t n_cells;

    //! Número de drones.
    /*!
    *   *	Indica o número de drones sendo simulados.
    */
    uint16_t numberDrones;

    //! Constante determinada pelas dimensões do heatmap.
    /*!
    *   *	É usada nos cálculos.
    */
    double mul_factor;

    //! Estrutura que armazena os valores de cada célula.
    /*!
    *   *	Armazena os dados do heatmap em si.
    */
    std::vector<std::vector<uint16_t>> heatMap;

    //! Armazena as últimas células em que cada drone foi avistado.
    /*!
    *   *	Armazena uma célula para cada drone, a última onde foi confirmado que ele estava.
    */
    std::vector<drone_middleware::HeatMapCell> dronesLastCells;

    public:

    //! Construtor da classe.
    /*!
    *   *	Os valores são exatamente os mesmos dos atributos.
    */
    MobilityHeatMap(bool active, double x_min, double y_min, double width, 
        uint16_t n_cells, uint16_t numberDrones);

    //! Incrementa o valor de uma célula do heatmap.
    /*!
    *   *	i e j informam linha e coluna, a função incrementa o valor da matriz nesse local em 1.
    */
    void incrementCell(drone_middleware::HeatMapCell cell);

    //! Retorna em qual célula um ponto está.
    /*!
    *   *	Calcula qual a célula correspondente ao ponto informado e retorna na forma de strutura HeatmapCell.
    */
    drone_middleware::HeatMapCell getCellForPoint(double x, double y);

    //! Retorna o mapa como um vetor unidimensional.
    /*!
    *   *	Utilizado para poder enviar em resposta de mensagem de serviço.
    */
    std::vector<uint16_t> getHeatMapAsVector();

    //! Getter da variável active.
    uint8_t is_active();
    //! Getter da variável x_min.
    double get_x_min();
    //! Getter da variável y_min.
    double get_y_min();
    //! Getter da variável width.
    double get_width();
    //! Getter da variável n_cells.
    uint16_t get_n_cells();

    //! Getter das last cells.
    drone_middleware::HeatMapCell getDroneLastCell(uint16_t droneNumber);
    //! Setter das last cells
    void setDroneLastCell(uint16_t droneNumber, drone_middleware::HeatMapCell cell);

  };

  //#endregion

}

#endif
