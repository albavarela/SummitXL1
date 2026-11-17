#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import math
import rospy          
import smach        
import threading   
from movimiento import Movimiento
from piernas.msg import Alba  # Mensaje que contiene la información de la persona detectada por Alba
from geometry_msgs.msg import Twist   # Mensaje para enviar velocidades (lineal y angular) a los motores
from std_msgs.msg import Float32

T_OBSOLETO = 1   # s: si alba no publica en este tiempo, se ignora el último dato
D_STOP = 1.0       # m: a esta distancia o menos la persona está cerca (STOP); a más, lejos (FOLLOWING)
SENTIDO_BUSQUEDA = 1   # sentido de giro al buscar: 1 = izquierda, -1 = derecha

# dist_deseada menor que D_STOP para que follow siga avanzando hasta cruzar el umbral de STOP
mv = Movimiento(dist_deseada=0.8)

    
class AlbaData(object):
    """
    Contenedor de datos de la detección 2D.
    """
    def __init__(self):
        self.msg_2d = None
        self.t_2d = None
        self._lock = threading.Lock()

    def update_2d(self, msg):
        with self._lock:
            self.msg_2d = msg
            self.t_2d = rospy.get_time()

    def lectura_vigente(self):
        with self._lock:
            if (self.msg_2d is not None and
                    (rospy.get_time() - self.t_2d) <= T_OBSOLETO and
                    self.msg_2d.persona_detectada):
                return self.msg_2d
            return None

    def lectura_persona(self):
        return self.lectura_vigente()

    def distancia_persona(self):
        msg = self.lectura_persona()
        if msg is None:
            return None
        return math.hypot(msg.x, msg.y)

    @property
    def persona_detectada(self):
        return self.lectura_persona() is not None

    @property
    def angulo(self):
        msg = self.lectura_persona()
        return None if msg is None else msg.angulo

class ObstacleData(object):
    def __init__(self):
        self._distance = None
        self._received_at = None
        self._lock = threading.Lock()

    def update(self, msg):
        with self._lock:
            self._distance = msg.data
            self._received_at = rospy.get_time()

    def distance(self):
        with self._lock:
            distance = self._distance
            received_at = self._received_at

        if (distance is None or received_at is None
                or rospy.get_time() - received_at > T_OBSOLETO
                or distance < 0.0):
            return None
        return distance


class Stop(smach.State):

    """

    Estado en el que el robot permanece parado con una persona cerca.

    Mientras la persona esté a `D_STOP` metros o menos, publica en cada ciclo la velocidad
    que devuelve `mv.stop()`. Sale del estado si la persona se aleja, si deja
    de detectarse o si se solicita la interrupción de la máquina.

    Outcomes:
        
        persona_perdida: Alba no detecta persona (o sus datos están obsoletos).
        persona_lejos: La persona está a más de `D_STOP` metros del robot.
        preempted: Se solicitó la interrupción o se apagó el nodo.
    
    """

    def __init__(self, pub, data):

        """
        Inicializa el estado y guarda las referencias compartidas.

        Args:

            pub (rospy.Publisher): Publicador del topic `/cmd_vel` (mensajes Twist).
            data (AlbaData): Objeto compartido con los datos más recientes de alba.

        """
                
        smach.State.__init__(self, outcomes=['persona_perdida', 'persona_lejos', 'preempted'])

        self.pub = pub
        self.data = data

    def execute(self, userdata):

        """
        Mantiene el robot parado mientras la persona esté cerca.

        Args:

            userdata: No se utiliza; los datos de alba llegan por `self.data`.

        Returns:

            str: Outcome del estado: 'persona_perdida', 'persona_lejos' o 'preempted'.

        """

        rospy.loginfo('STOP')

        move = Twist()
        rate = rospy.Rate(10)
  
        # Se ejecutará mientras no se solicite interrumpir la máquina y mientras el nodo no se apague
        while not rospy.is_shutdown():

            # Por cada ciclo, se obtiene y actualiza la distancia de la persona respecto al robot
            distancia = self.data.distancia_persona()

            # Se comprueba si se ha solicitado la interrupción de la máquina
            if self.preempt_requested():
            
                self.service_preempt()
                
                move.linear.x = 0.0
                move.angular.z = 0.0
                
                self.pub.publish(move)

                return 'preempted'

            # Si no hay distancia daremos por hecho que no se detecta persona 
            if distancia is None:

                rospy.loginfo("No se detecta persona")

                # Cambiamos de estado
                return 'persona_perdida'

            # Si la distancia es mayor de D_STOP significa que la persona está lejos 
            elif distancia > D_STOP:
                
                rospy.loginfo("Persona lejos")

                # Cambiamos de estado
                return 'persona_lejos'

            # Si la distancia es menor o igual a D_STOP significa que la persona está cerca
            else:

                rospy.loginfo_throttle(2.0, 'Persona cerca')

                # mv.stop() devuelve (x, y, z): usamos el primero (lineal) y el último (angular)
                velocidades = mv.stop()

                # Asignamos al mensaje Twist los valores de velocidad y giro obtenidos
                move.linear.x = velocidades[0]
                move.angular.z = velocidades[-1]

                # Publicamos el mensaje en el topic `/cmd_vel` para que el robot se mueva según los valores obtenidos
                self.pub.publish(move)

            # Esperamos un tiempo para mantener la frecuencia de publicación y evitar saturar el topic
            rate.sleep()

        # Si se sale del bucle por interrupción del nodo, se detiene el robot y se indica que se ha preemptado la máquina
        move.linear.x = 0.0
        move.angular.z = 0.0
                
        self.pub.publish(move)

        return 'preempted'
    
class Search(smach.State):

    """
    Estado en el que el robot busca a una persona.

    Publica en cada ciclo la velocidad que devuelve `mv.search()` hasta que
    alba detecta a alguien. Según la distancia, pasa a seguir o a parar.

    Outcomes:
    
        persona_lejos: Persona detectada a más de `D_STOP` metros (ir a FOLLOWING).
        persona_cerca: Persona detectada a `D_STOP` metros o menos (ir a STOP).
        preempted: Se solicitó la interrupción o se apagó el nodo.
    
    """
    def __init__(self, pub, data):

        """
        Inicializa el estado y guarda las referencias compartidas.

        Args:

            pub (rospy.Publisher): Publicador del topic `/cmd_vel` (mensajes Twist).
            data (AlbaData): Objeto compartido con los datos más recientes de alba.

        """
                
        smach.State.__init__(self, outcomes=['preempted','persona_lejos', 'persona_cerca'],)
        
        self.pub = pub
        self.data = data
    
    def execute(self, userdata):

        """
        Busca a la persona hasta que alba la detecte.

        Args:

            userdata: No se utiliza; los datos de alba llegan por `self.data`.

        Returns:

            str: Outcome del estado: 'persona_lejos', 'persona_cerca' o 'preempted'.

        """

        rospy.loginfo('SEARCHING')

        move = Twist()
        rate = rospy.Rate(10)
  
        # Se ejecutará mientras no se solicite interrumpir la máquina y mientras el nodo no se apague
        while not rospy.is_shutdown():

            # Por cada ciclo, se obtiene y actualiza la distancia de la persona respecto al robot
            distancia = self.data.distancia_persona()

            # Se comprueba si se ha solicitado la interrupción de la máquina
            if self.preempt_requested():
            
                self.service_preempt()
                
                move.linear.x = 0.0
                move.angular.z = 0.0
                
                self.pub.publish(move)

                return 'preempted'

            # Si no hay distancia significa que no se detecta persona, seguimos buscando
            if distancia is None:

                rospy.loginfo_throttle(2.0, "Buscando persona...")
                
                # mv.search() devuelve (x, y, z): usamos el primero (lineal) y el último (angular)
                velocidades = mv.search(SENTIDO_BUSQUEDA)
                
                move.linear.x = velocidades[0]
                move.angular.z = velocidades[-1]
                
                self.pub.publish(move)
                
            # Si la distancia es menor o igual a D_STOP significa que la persona está cerca
            elif distancia <= D_STOP:
                
                rospy.loginfo("Persona cerca")

                # Cambiamos de estado
                return 'persona_cerca'

            # Si la distancia es mayor de D_STOP significa que la persona está lejos
            else:

                rospy.loginfo("Persona lejos")

                # Cambiamos de estado
                return 'persona_lejos'
                
            # Esperamos un tiempo para mantener la frecuencia de publicación y evitar saturar el topic
            rate.sleep()

        # Si se sale del bucle por interrupción del nodo, se detiene el robot y se indica que se ha preemptado la máquina
        move.linear.x = 0.0
        move.angular.z = 0.0
                
        self.pub.publish(move)

        return 'preempted'

class Follow(smach.State):

    """
    Estado en el que el robot sigue a una persona que está lejos.

    Publica en cada ciclo la velocidad que devuelve `mv.follow()`, que recibe
    la posición (x, y) de la persona respecto al robot y calcula por su cuenta
    la distancia y el rumbo. Sale del estado si la persona se acerca a `D_STOP`
    metros o menos, si deja de detectarse o si se solicita la interrupción de
    la máquina.

    Outcomes:

        persona_cerca: Persona a `D_STOP` metros o menos (ir a STOP).
        persona_perdida: Alba no detecta persona (ir a SEARCHING).
        preempted: Se solicitó la interrupción o se apagó el nodo.

    """

    def __init__(self, pub, data, obstacles):

        """
        Inicializa el estado y guarda las referencias compartidas.

        Args:

            pub (rospy.Publisher): Publicador del topic `/cmd_vel` (mensajes Twist).
            data (AlbaData): Objeto compartido con los datos más recientes de alba.

        """
                
        smach.State.__init__(self, outcomes=['persona_cerca', 'persona_perdida', 'preempted'])
        
        self.pub = pub
        self.data = data
        self.obstacles = obstacles

    def execute(self, userdata):

        """
        Sigue a la persona hasta que se acerque o se pierda.

        Args:

            userdata: No se utiliza; los datos de alba llegan por `self.data`.

        Returns:

            str: Outcome del estado: 'persona_cerca', 'persona_perdida' o 'preempted'.

        """

        rospy.loginfo('FOLLOWING')

        move = Twist()
        rate = rospy.Rate(10)
  
        # Se ejecutará mientras no se solicite interrumpir la máquina y mientras el nodo no se apague
        while not rospy.is_shutdown():

            # Por cada ciclo, se obtiene y actualiza el mensaje con la persona detectada
            msg = self.data.lectura_persona()
            
            # Se comprueba si se ha solicitado la interrupción de la máquina
            if self.preempt_requested():
            
                self.service_preempt()
                
                move.linear.x = 0.0
                move.angular.z = 0.0
                
                self.pub.publish(move)

                return 'preempted'

            # Si no hay mensaje daremos por hecho que no se detecta persona 
            if msg is None:

                rospy.loginfo("Persona perdida")

                # Cambiamos de estado
                return 'persona_perdida'

            # Solo ahora es seguro calcular la distancia, porque msg existe
            distancia = math.hypot(msg.x, msg.y)

            # Si la distancia es menor o igual a D_STOP significa que la persona está cerca
            if distancia <= D_STOP:
                
                rospy.loginfo("Persona alcanzada")

                # Cambiamos de estado
                return 'persona_cerca'

            # Si la distancia es mayor de D_STOP significa que la persona está lejos
            else:

                rospy.loginfo_throttle(2.0, 'Siguiendo, distancia: %.2f m', distancia)

                # mv.follow recibe la posición (x, y) y devuelve (lineal, angular)
                velocidad_lineal, velocidad_angular = mv.follow(
                    (msg.x, msg.y),
                    obstacle_distance=self.obstacles.distance(),
                )

                move.linear.x = velocidad_lineal
                move.angular.z = velocidad_angular

                self.pub.publish(move)
                
            # Esperamos un tiempo para mantener la frecuencia de publicación y evitar saturar el topic
            rate.sleep()

        # Si se sale del bucle por interrupción del nodo, se detiene el robot y se indica que se ha preemptado la máquina
        move.linear.x = 0.0
        move.angular.z = 0.0
                
        self.pub.publish(move)

        return 'preempted'

rospy.init_node('SummitXl')

machine = smach.StateMachine(outcomes=['abortado'])

data = AlbaData()
obstacles = ObstacleData()

rospy.Subscriber('/datos_piernas_2d', Alba, data.update_2d, queue_size=1)

rospy.Subscriber(
    rospy.get_param('~wall_distance_topic', '/pared_distancia'),
    Float32,
    obstacles.update,
    queue_size=1,
)
cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/robot/cmd_vel')
pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)

stop_state = Stop(pub, data)
searching_state = Search(pub, data)
following_state = Follow(pub, data, obstacles)

with machine:
    smach.StateMachine.add('STOP', stop_state, transitions={'persona_lejos':'FOLLOWING', 'persona_perdida':'SEARCHING', 'preempted':'abortado'})
    smach.StateMachine.add('SEARCHING', searching_state, transitions={'persona_lejos':'FOLLOWING', 'persona_cerca':'STOP', 'preempted':'abortado'})
    smach.StateMachine.add('FOLLOWING', following_state, transitions={'persona_cerca':'STOP', 'persona_perdida':'SEARCHING', 'preempted':'abortado'})

rospy.sleep(1.0)
machine.execute()
pub.publish(Twist())