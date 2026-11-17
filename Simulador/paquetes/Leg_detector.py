#!/usr/bin/env python
import math

import rospy
from geometry_msgs.msg import PoseStamped
from piernas.msg import Alba
from people_msgs.msg import PositionMeasurementArray
from visualization_msgs.msg import Marker


class LectorLegDetector(object):
    def __init__(self):
        rospy.init_node('nodo_lector_leg_detector')

        self.sub = rospy.Subscriber(
            '/people_tracker_measurements',
            PositionMeasurementArray,
            self.callback,
        )
        self.marker_pub = rospy.Publisher(
            '/visualization_marker', Marker, queue_size=1)
        self.pose_pub = rospy.Publisher(
            '/person_pose', PoseStamped, queue_size=1)
        self.datos_pub = rospy.Publisher(
            rospy.get_param('~people_topic', '/datos_piernas_2d'),
            Alba,
            queue_size=1,
        )

        rospy.loginfo("Adaptador de leg_detector iniciado.")

    def callback(self, msg):
        if rospy.is_shutdown():
            return

        if msg.people:
            person = msg.people[0]
            px, py = person.x, person.y
            frame_id = (
                person.header.frame_id
                or msg.header.frame_id
                or 'base_laser'
            )
            self.publicar_resultado(True, (px, py), frame_id)
            self.publish_visuals(px, py, frame_id, detected=True)
        else:
            frame_id = msg.header.frame_id or 'base_laser'
            self.publicar_resultado(False, (0.0, 0.0), frame_id)
            self.publish_visuals(0.0, 0.0, frame_id, detected=False)

    def publicar_resultado(self, detectado, posicion, frame_id):
        px, py = posicion
        distancia = math.hypot(px, py) if detectado else 0.0
        angulo = math.atan2(py, px) if detectado else 0.0

        if detectado:
            rospy.loginfo_throttle(
                2.0,
                "Persona detectada en X: %.2f, Y: %.2f | Angulo: %.2f",
                px,
                py,
                angulo,
            )
        else:
            rospy.loginfo_throttle(2.0, "No se detecta ninguna persona.")

        # This is the message consumed by state_machine.py.
        resultado = Alba()
        resultado.persona_detectada = detectado
        resultado.x = px
        resultado.y = py
        resultado.angulo = angulo

        if not rospy.is_shutdown():
            self.datos_pub.publish(resultado)

        if detectado:
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = frame_id
            pose.pose.position.x = px
            pose.pose.position.y = py
            pose.pose.orientation.z = math.sin(angulo / 2.0)
            pose.pose.orientation.w = math.cos(angulo / 2.0)
            if not rospy.is_shutdown():
                self.pose_pub.publish(pose)

        return detectado, distancia, angulo

    def publish_visuals(self, x, y, frame_id, detected):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = frame_id
        marker.ns = "legs"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD if detected else Marker.DELETE
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.2
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        if not rospy.is_shutdown():
            self.marker_pub.publish(marker)


if __name__ == '__main__':
    try:
        LectorLegDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
