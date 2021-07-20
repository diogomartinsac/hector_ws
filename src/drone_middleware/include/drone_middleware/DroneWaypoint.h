//#region [include]

#include "geometry_msgs/Pose.h"

#include <cstdint>

//#endregion

/*
uint16_t droneNumber
uint32_t seqNum
geometry_msgs::Pose pose
uint8_t all
uint8_t block
uint8_t takeoff
uint8_t land
uint8_t landAt
*/

namespace drone_middleware
{
  
  //#region [class]
  //! Classe que armazena dados de um movimento de posição de drone.
  /*!
  *   *	Em seu estado atual a classe contém o número do drone, uma informação de ordem e um atributo da classe Pose, que faz parte das mensagens de geometria do ROS.
  *   *	Esta classe é utilizada pelos nós drone_waypoint_* para manipular os comandos de posição serem enviados para os drones.
  * 	*	A ideia é que o drone "droneNumber" receba o comando "waypoint", o nó drone_waypoint_core garante que o este comando seja executado.
  */
  class DroneWaypoint
  {
    public:

    
    //! Número atribuído ao drone.
    /*!
    *   *	A numeração é positiva e inteira, em geral começa a partir do 0 e segue até N-1 onde N é o número total de drones na simulação.
    */
    uint16_t droneNumber;
    
    //! Numero de sequência da posição.
    /*!
    *   *	Os comandos de posição serão executados conforme ordem definida por esse parâmetro, que é um número positivo e inteiro.
    *	*	Quanto menor o número, será executado antes.
    *	*	Se 0 for informado, esse comando será colocado ao fim da fila atualmente existente e seu valor será alterado para o valor do último somado a 1.
    *	*	Não informe números de sequência duplicados, exceto o zero, caso contrário os comandos serão ignorados.
    */
    uint32_t seqNum;
    
    //! Movimento que o drone deve realizar, expresso em termos de posição e orientação.
    /*!
    *   *	Este tipo representa pose dividida em suas componentes linear e angular, sendo a primeira um vetor tridimensional.
    *	*	A orientação é representada por um vetor quadridimensional.
    * 	*	Para ver mais sobre geometry_msgs/Pose consulte: \<http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Pose.html\>
    */
    geometry_msgs::Pose pose;

    //! Booleano que indica decolagem.
    /*!
    *   *	Se verdadeiro, as outras informações são ignoradas, apenas decola o drone.
    */
    uint8_t takeoff;

    //! Booleano que indica pouso.
    /*!
    *   *	Se verdadeiro, as outras informações são ignoradas (exceto se landAt for verdadeiro), apenas pousa o drone diretamente abaixo.
    */
    uint8_t land;

    //! Booleano que indica se a pose deve ser considerada.
    /*!
    *   *	Se verdadeiro, e apenas se land também for verdadeiro, pousa o drone com uma pose especifica.
    */
    uint8_t landAt;

    //#endregion

    //region [func]

    //! Definição de operador que institui a ordem parcial "<" sobre Dronewaypoint
    /*!
     * 	*	Esta ordenação é necessária para guardar os comandos de posição numa estrutura ordenada e determinar quando eles devem ocorrer.
     * 	*	A ordenação é feita com base em seqNum.
     */
    friend bool operator<(const DroneWaypoint& left, const DroneWaypoint& right)
    {
      
      return (left.seqNum < right.seqNum);
    }

    //#endregion
    
    //#region [class]

  };

  //#endregion
	
}
