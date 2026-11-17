#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
from collections import namedtuple

import rospy
import tf
from geometry_msgs.msg import PointStamped, PoseStamped
from piernas.msg import Alba
from people_msgs.msg import PositionMeasurementArray
from visualization_msgs.msg import Marker

PersonDetection = namedtuple('PersonDetection', 'id x y world_x world_y')


class PersonTrack(object):
    def __init__(self, detection, now):
        self.id = detection.id
        self.anchor = (detection.world_x, detection.world_y)
        self.stationary_since = now
        self.last_seen = now
        self.ignored = False


class PersonSelector(object):
    def __init__(self):
        self.tracks = []
        self.target = None

    def update(self, detections, now):
        matched = {}
        remaining = list(detections)

        for track in self.tracks:
            detection = next(
                (item for item in remaining if item.id == track.id), None)
            if detection is not None:
                matched[track] = detection
                remaining.remove(detection)

        for detection in remaining:
            track = PersonTrack(detection, now)
            self.tracks.append(track)
            matched[track] = detection

        for track, detection in matched.items():
            position = (detection.world_x, detection.world_y)
            if (now < track.last_seen or now - track.last_seen > 2.0
                    or math.hypot(position[0] - track.anchor[0],
                                  position[1] - track.anchor[1]) >= 0.2):
                track.anchor = position
                track.stationary_since = now
            elif now - track.stationary_since >= 8.0:
                if not track.ignored:
                    rospy.loginfo("2D: persona %s ignorada por permanecer quieta", detection.id)
                track.ignored = True
            track.last_seen = now

        if self.target is not None:
            if self.target.ignored:
                self.target = None
            elif self.target in matched:
                return matched[self.target]
            else:
                self.target = None

        candidates = [(track, detection) for track, detection in matched.items()
                      if not track.ignored]
        if not candidates:
            return None
        self.target, detection = min(
            candidates, key=lambda pair: math.hypot(pair[1].x, pair[1].y))
        return detection


class LectorLegDetector(object):
    def __init__(self):
        rospy.init_node('nodo_lector_leg_detector')

        # Parámetros configurables
        self.reliability_threshold = rospy.get_param('~reliability_threshold', 0.1)
        self.target_frame = rospy.get_param('~target_frame', 'robot_base_link')
        self.tracking_frame = rospy.get_param('~tracking_frame', 'robot_odom')
        self.selector = PersonSelector()

        # Inicializar el listener de TF
        self.tf_listener = tf.TransformListener()

        self.marker_pub = rospy.Publisher(
            '/visualization_marker', Marker, queue_size=1)
        self.pose_pub = rospy.Publisher(
            '/person_pose', PoseStamped, queue_size=1)
        self.datos_pub = rospy.Publisher(
            rospy.get_param('~people_topic', '/datos_piernas_2d'),
            Alba,
            queue_size=1,
        )
        self.sub = rospy.Subscriber(
            '/people_tracker_measurements',
            PositionMeasurementArray,
            self.callback,
            queue_size=1,
        )

        rospy.loginfo(
            "Adaptador leg_detector iniciado: umbral=%.2f, frame_destino=%s, frame_seguimiento=%s",
            self.reliability_threshold,
            self.target_frame,
            self.tracking_frame,
        )

    def callback(self, msg):
        if rospy.is_shutdown():
            return

        valid_people = [p for p in msg.people if p.reliability >= self.reliability_threshold]
        detections = []
        seen_ids = set()
        for person in valid_people:
            if not person.object_id or person.object_id in seen_ids:
                rospy.logwarn_throttle(5.0, "Leg_detector: ID vacia o duplicada; deteccion descartada")
                continue
            seen_ids.add(person.object_id)
            point = PointStamped()
            point.header = person.header if person.header.frame_id else msg.header
            
            point.point.x = person.pos.x
            point.point.y = person.pos.y
            point.point.z = person.pos.z if hasattr(person.pos, 'z') else 0.0

            frame_origen = point.header.frame_id
            if not frame_origen:
                rospy.logwarn_throttle(5.0, "Leg_detector: deteccion sin frame_id")
                continue
            try:
                robot_point = (point if frame_origen == self.target_frame else
                               self.tf_listener.transformPoint(self.target_frame, point))
                world_point = (point if frame_origen == self.tracking_frame else
                               self.tf_listener.transformPoint(self.tracking_frame, point))
            except (tf.LookupException, tf.ConnectivityException,
                    tf.ExtrapolationException) as exc:
                rospy.logwarn_throttle(
                    5.0, "Leg_detector: No se puede transformar %s: %s",
                    frame_origen, exc)
                continue

            values = (robot_point.point.x, robot_point.point.y,
                      world_point.point.x, world_point.point.y)
            if any(math.isnan(value) or math.isinf(value) for value in values):
                continue
            detections.append(PersonDetection(person.object_id, *values))

        selected = self.selector.update(detections, rospy.get_time())
        if selected is None:
            self.publicar_resultado(False, (0.0, 0.0), self.target_frame)
            self.publish_visuals(0.0, 0.0, self.target_frame, detected=False)
            return

        self.publicar_resultado(True, (selected.x, selected.y), self.target_frame)
        self.publish_visuals(selected.x, selected.y, self.target_frame, detected=True)

    def publicar_resultado(self, detectado, posicion, frame_id):
        px, py = posicion
        distancia = math.hypot(px, py) if detectado else 0.0
        angulo = math.atan2(py, px) if detectado else 0.0

        if detectado:
            rospy.loginfo_throttle(
                2.0,
                "2D Leg -> X: %.2f, Y: %.2f | Angulo: %.2f",
                px,
                py,
                angulo,
            )
        else:
            rospy.loginfo_throttle(2.0, "2D Leg -> No se detecta persona confiable.")

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
        marker.ns = "legs_2d"
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