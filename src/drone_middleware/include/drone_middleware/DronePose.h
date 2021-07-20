//#region [include]

#include "geometry_msgs/PoseStamped.h"

//#endregion

//TODO: talvez transformar essa classe em uma estrutura de dados simples
//		ou então deixar como um espaço reservado pra ter member functions no futuro

namespace drone_middleware
{
	
  //#region [class]

  //! Classe que armazena posição e orientação do drone.
  /*!
  *   *	Em seu estado atual a classe contém o número do drone e um atributo da classe PoseStamped que faz parte das mensagens de geometria do ROS, ambos públicos.
  *   *	Esta classe é utilizada pelo nó pose_listener para armazenar as informações atuais de postura dos drones.
  *   *	Poderia ter sido implementada simplesmente como uma estrutura, mas como classe permite inclusão futura de funções membro.
  */
  class DronePose
  {
    
    public:
    
    //! Número atribuído ao drone.
    /*!
    *   *	A numeração é positiva e inteira, em geral começa a partir do 0 e segue até N-1 onde N é o número total de drones na simulação.
    */
    unsigned int droneNumber;
    
    //! Dados da posição e orientação do drone.
    /*!
    * 	*	Além da postura, contém também um header que possui informações de ID sequencial e tempo.
    *   *	Para ver mais sobre geometry_msgs/PoseStamped consulte: \<http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/PoseStamped.html\>
    */
    geometry_msgs::PoseStamped dronePose;

  };

  //#endregion

}
