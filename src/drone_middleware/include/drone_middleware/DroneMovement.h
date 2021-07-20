//#region [include]

#include "geometry_msgs/Twist.h"

//#endregion

namespace drone_middleware
{
	
  //#region [class]
  //! Classe que armazena dados de um movimento de drone.
  /*!
  *   *	Em seu estado atual a classe contém o número do drone, uma informação de tempo e um atributo da classe Twist, que faz parte das mensagens de geometria do ROS.
  *   *	Esta classe é utilizada pelos nós drone_teleop_* para manipular os comandos de movimento a serem enviados para os drones.
  * 	*	A ideia é que o drone "droneNumber" receba o comando "movement" em "startAt", o nó drone_teleop_core garante que o este continue descrevendo tal movimento até que receba um novo comando.
  */
  class DroneMovement
  {
    public:
    
    //! Número atribuído ao drone.
    /*!
    *   *	A numeração é positiva e inteira, em geral começa a partir do 0 e segue até N-1 onde N é o número total de drones na simulação.
    */
    unsigned int droneNumber;
    
    //! Tempo em que o movimento deve começar.
    /*!
    *   *	Este tipo possui dois subcampos, sec e nsec, que armazenam o tempo em quantidade de segundos e de nanossegundos.
    * 	*	Para fazer esta conversão de forma fácil basta usar o construtor da classe ros::Time. Ex: ros::Time(12.5).sec retorna 12 e ros::Time(12.5).nsec retorna 500000000.
    * 	*	Para ver mais sobre ros::Time consulte: \<http://docs.ros.org/kinetic/api/rostime/html/classros_1_1Time.html\>
    */
    ros::Time startAt;
    
    //! Movimento que o drone deve realizar, expresso em termos de velocidade.
    /*!
    *   *	Este tipo representa velocidade do espaço livre dividida em suas componentes linear e angular, sendo cada uma um vetor tridimensional.
    * 	*	Com ela pode ser especificada uma velocidade nos 3 eixos (x, y, z) do espaço e rotações (row, pitch, yaw).
    * 	*	Para ver mais sobre geometry_msgs/Twist consulte: \<http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Twist.html\>
    */
    geometry_msgs::Twist movement;
    
    //! Definição de operador que institui a ordem parcial ">" sobre DroneMovement
    /*!
    * 	*	Esta ordenação é necessária para guardar os comandos de movimento numa fila de prioridade e determinar quando eles devem ocorrer de acordo com o tempo de simulação atual.
    * 	*	A ordenação é feita com base em startAt.
    */

    //#endregion

    //region [func]

    friend bool operator>(const DroneMovement& left, const DroneMovement& right)
    {
      
      return ((left.startAt.sec > right.startAt.sec) || 
        ((left.startAt.sec == right.startAt.sec) && 
        (left.startAt.nsec > right.startAt.nsec)));
    }

    //#endregion
    
    //#region [class]

  };

  //#endregion
	
}
