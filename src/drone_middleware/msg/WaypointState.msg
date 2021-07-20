#Esta mensagem é utilizada no tópico que publica mudanças de estado no CoreWaypoint
#É publicada toda vez que um comando se torna ativo, falha, é cancelado ou completado
#droneNumber --> para qual drone era esse comando
#seqNum --> número de sequência que possuía
#pose --> pose do comando
#takeoff --> booleano, se verdadeiro, é um comando de decolagem
#land --> booleano, se verdadeiro, é um comando de pouso
#landAt --> booleano, se verdadeiro, considera pose para pousar, se falso, pousa diretamente abaixo
#actionState --> estado do comando, conforme definido em CoreWaypoint.h
uint16 droneNumber
uint32 seqNum
geometry_msgs/Pose pose
uint8 takeoff
uint8 land
uint8 landAt
uint8 actionState