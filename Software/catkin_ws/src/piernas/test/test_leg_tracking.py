#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import os
import sys
import unittest
import ast
import math
import threading

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
from Leg_detector import LectorLegDetector, PersonDetection, PersonSelector
from geometry_msgs.msg import PointStamped
from piernas.msg import Alba
from people_msgs.msg import PositionMeasurement, PositionMeasurementArray
import rospy


def load_alba_data():
    path = os.path.join(os.path.dirname(__file__), '..', 'scripts', 'state_machine.py')
    with open(path) as source:
        module = ast.parse(source.read(), filename=path)
    statements = [node for node in module.body
                  if (isinstance(node, ast.ClassDef) and node.name == 'AlbaData')
                  or (isinstance(node, ast.Assign)
                      and any(isinstance(target, ast.Name) and target.id == 'T_OBSOLETO'
                              for target in node.targets))]
    namespace = {'math': math, 'threading': threading, 'rospy': rospy}
    exec(compile(ast.Module(body=statements), path, 'exec'), namespace)
    return namespace['AlbaData']


AlbaData = load_alba_data()


def person(identifier, x, world_x=None):
    return PersonDetection(identifier, x, 0.0, x if world_x is None else world_x, 0.0)


class PersonSelectorTest(unittest.TestCase):
    def setUp(self):
        self.selector = PersonSelector()

    def test_stationary_person_is_excluded_and_next_person_selected(self):
        for time in range(8):
            selected = self.selector.update([person('a', 1), person('b', 3 + time)], time)
            self.assertEqual(selected.id, 'a')
        selected = self.selector.update([person('a', 1), person('b', 11)], 8)
        self.assertEqual(selected.id, 'b')
        selected = self.selector.update([person('a', 2), person('b', 12)], 9)
        self.assertEqual(selected.id, 'b')

    def test_stationary_time_resets_after_moving_20_cm(self):
        for time in (0, 1, 2, 3):
            self.assertIsNotNone(self.selector.update([person('a', 1.0)], time))
        for time in range(4, 12):
            self.assertIsNotNone(self.selector.update([person('a', 1.25)], time))
        self.assertIsNone(self.selector.update([person('a', 1.25)], 12))

    def test_robot_motion_does_not_count_as_person_motion(self):
        for time in range(8):
            self.assertIsNotNone(self.selector.update([person('a', 8 - time, 10)], time))
        self.assertIsNone(self.selector.update([person('a', 0, 10)], 8))

    def test_new_id_is_selected_without_waiting_or_inheriting_history(self):
        self.selector.update([person('a', 2)], 0)
        selected = self.selector.update([person('new', 2.1), person('b', 5)], 1)
        self.assertEqual(selected.id, 'new')
        self.assertEqual(self.selector.target.id, 'new')
        self.assertEqual(self.selector.target.stationary_since, 1)

    def test_ignored_id_stays_ignored_but_new_id_is_eligible(self):
        for time in range(9):
            self.selector.update([person('a', 1)], time)
        self.assertIsNone(self.selector.update([person('a', 1)], 9))
        self.assertEqual(self.selector.update([person('new', 1.1)], 10).id, 'new')
        self.assertIsNone(self.selector.update([person('a', 2)], 11))

    def test_missing_person_does_not_accrue_stationary_time_indefinitely(self):
        self.selector.update([person('a', 1)], 0)
        self.selector.update([], 1)
        self.assertIsNotNone(self.selector.update([person('a', 1)], 4))
        self.assertIsNotNone(self.selector.update([person('a', 1)], 5))

class AlbaDataTest(unittest.TestCase):
    def setUp(self):
        self.get_time = rospy.get_time
        rospy.get_time = lambda: 10.0

    def tearDown(self):
        rospy.get_time = self.get_time

    def test_2d_detection_is_returned(self):
        data = AlbaData()
        detected_2d = Alba(persona_detectada=True, x=2.0, y=0.0)
        data.update_2d(detected_2d)
        self.assertIs(data.lectura_persona(), detected_2d)


class AdapterTest(unittest.TestCase):
    def test_callback_filters_unreliable_person(self):
        node = LectorLegDetector.__new__(LectorLegDetector)
        node.reliability_threshold = 0.8
        node.target_frame = 'robot_odom'
        node.tracking_frame = 'robot_odom'
        node.selector = PersonSelector()
        results = []
        node.publicar_resultado = lambda detected, position, frame: results.append(
            (detected, position))
        node.publish_visuals = lambda *args, **kwargs: None
        measurement = PositionMeasurement()
        measurement.header.frame_id = 'robot_odom'
        measurement.object_id = 'person_a'
        measurement.reliability = 0.75
        measurement.pos.x = 1.2
        message = PositionMeasurementArray()
        message.people = [measurement]
        original_time = rospy.get_time
        try:
            rospy.get_time = lambda: 0.0
            node.callback(message)
            self.assertEqual(results[-1], (False, (0.0, 0.0)))
            measurement.reliability = 0.9
            node.callback(message)
            self.assertTrue(results[-1][0])
            self.assertAlmostEqual(results[-1][1][0], 1.2)
        finally:
            rospy.get_time = original_time

    def test_stationarity_uses_world_frame_in_callback(self):
        class FakeTF(object):
            offset = 0.0

            def transformPoint(self, frame, point):
                result = PointStamped()
                result.header.frame_id = frame
                result.point.x = point.point.x - self.offset
                result.point.y = point.point.y
                return result

        node = LectorLegDetector.__new__(LectorLegDetector)
        node.reliability_threshold = 0.7
        node.target_frame = 'robot_base_link'
        node.tracking_frame = 'robot_odom'
        node.selector = PersonSelector()
        node.tf_listener = FakeTF()
        results = []
        node.publicar_resultado = lambda detected, position, frame: results.append(
            (detected, position, frame))
        node.publish_visuals = lambda *args, **kwargs: None
        measurement = PositionMeasurement()
        measurement.header.frame_id = 'robot_odom'
        measurement.object_id = 'person_a'
        measurement.reliability = 1.0
        measurement.pos.x = 5.0
        message = PositionMeasurementArray()
        message.people = [measurement]
        original_time = rospy.get_time
        try:
            for time in range(9):
                rospy.get_time = lambda: time
                node.tf_listener.offset = time * 0.5
                node.callback(message)
        finally:
            rospy.get_time = original_time
        self.assertEqual(results[0], (True, (5.0, 0.0), 'robot_base_link'))
        self.assertEqual(results[7], (True, (1.5, 0.0), 'robot_base_link'))
        self.assertEqual(results[8], (False, (0.0, 0.0), 'robot_base_link'))


if __name__ == '__main__':
    unittest.main()
