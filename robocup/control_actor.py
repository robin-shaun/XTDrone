#!/usr/bin/python2
# -*- coding:utf-8 -*-
import rospy
import random
from ros_actor_cmd_pose_plugin_msgs.msg import ActorMotion
from geometry_msgs.msg import Point
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetModelState
from std_msgs.msg import String, Float32
from nav_msgs.msg import Odometry
from ObstacleAvoid import ObstacleAvoid
import sys
import numpy
import copy
import math
import ast
import numpy as np
import os
import heapq



class ControlActor:
    def __init__(self, actor_id):
        self.id = actor_id
        rospy.init_node('actor_' + self.id)
        self.count = 0
        self.shooting_count = 0
        self.uav_num = 6
        self.actor_num = 6
        self.vehicle_type = 'typhoon_h480'
        self.f = 10
        self.spon_dis = 5
        self.flag = True
        self.distance_flag = True
        self.suitable_point = True
        # Enable the legacy stall recovery by default.  It can be disabled
        # with the private ROS parameter when a scenario intentionally pauses
        # actors for a long time.
        self.get_moving = bool(rospy.get_param('~enable_stall_recovery', True))
        # A command can be rejected by Gazebo when an actor is close to a
        # wall or another actor.  Recover after a short, configurable period
        # instead of waiting for the old 100 second timeout.
        self.stall_limit_cycles = max(5, int(rospy.get_param(
            '~stall_limit_cycles', 30)))
        self.x = 0.0
        self.y = 0.0
        # self.x_max = 50.0
        # self.x_min = -10.0
        # self.y_max = -20.0
        # self.y_min = -30.0
        self.x_max = 130.0
        self.x_min = -50.0
        self.y_max = 60.0
        self.y_min = -60.0
        self.normal_speed = 1.0
        self.escape_speed = 2.0
        self.wall_clearance = 1.0
        self.actor_avoidance_radius = 4.0
        self.actor_min_distance = 2.0
        self.actor_spawn_distance = 6.0
        self.uav_spawn_distance = 8.0
        self.uav_takeoff_points = [(0.0, -3.0), (3.0, -3.0),
                                   (0.0, 0.0), (3.0, 0.0),
                                   (0.0, 3.0), (3.0, 3.0)]
        self.actor_positions = {}
        self.uav_positions = {}
        self.avoid = ActorMotion()
        self.last_pose = Point()
        self.current_pose = Point()
        self.target_motion = Point()
        self.avoid.v = self.normal_speed
        self.escape_distance = 20.0
        self.escape_trigger_speed = 1.0
        self.uav_push_radius = 7.0
        self.reported = False
        self.escape_triggered = False
        self.escape_requested = False
        self.escape_active = False
        self.actor_pose_ready = False
        self.route_waiting = False
        self.uav_odom = {}
        self.base_dir = os.path.dirname(os.path.abspath(__file__))
        self.black_box_path = self._find_data_file(
            [os.path.expanduser('~/XTDrone/robocup/black_box.txt'),
             os.path.join(self.base_dir, 'black_box.txt')])

        # obstacle avoidance:
        self.left_actors = range(self.actor_num)
        self.avoid_finish_flag = True
        self.subtarget_count = 0
        self.subtarget_length = 0
        self.subtarget_pos = []
        self.arrive_count = 0
        self.escape_suce_flag = False
        self.gazebo_actor_pose = Point()
        self.Obstacleavoid = None
        #self.black_box = numpy.array([[[-34, -19], [16, 34]], [[5, 20], [10, 28]], [[53, 68], [13, 31]], [[70, 84], [8, 20]], [[86, 102], [10, 18]], [[77, 96], [22, 35]], [[52, 71], [-34, -25]], [[-6, 6], [-35, -20]], [[12, 40], [-20, -8]], [[-7, 8], [-21, -9]], [[-29, -22], [-16, -27]], [[-37, -30], [-27, -12]], [[-38, -24], [-36, -29]]])
        with open(self.black_box_path, "r") as content:
            line = content.readline()
        self.black_box = ast.literal_eval(line)
        self.box_num = len(self.black_box)

        # Load obstacle data from a path independent of the working directory.
        obstacle_path = self._find_data_file(
            [os.path.join(self.base_dir, '2024.txt'),
             os.path.join(self.base_dir, 'obstacle.txt'),
             '2024.txt'])
        with open(obstacle_path, 'r') as file:
            self.obstacle_data = []
            for line in file:
                fields = line.strip().split()
                if len(fields) < 2:
                    continue
                try:
                    self.obstacle_data.append((float(fields[0]), float(fields[1])))
                except ValueError:
                    continue
        self.obstacle_path = obstacle_path
        try:
            self.Obstacleavoid = ObstacleAvoid(self.obstacle_path)
        except (IOError, OSError, ValueError) as exc:
            # The local grid planner below can still operate without the
            # legacy planner, so do not stop the actor node at startup.
            print('actor_' + self.id + ' legacy obstacle planner unavailable: ' +
                  str(exc))
            self.Obstacleavoid = None
        self.obstacle_grid_size = 2.0
        self.obstacle_grid = {}
        for obstacle_x, obstacle_y in self.obstacle_data:
            cell = (int(math.floor(obstacle_x / self.obstacle_grid_size)),
                    int(math.floor(obstacle_y / self.obstacle_grid_size)))
            self.obstacle_grid.setdefault(cell, []).append((obstacle_x,
                                                             obstacle_y))
        self.cmd_pub = rospy.Publisher('/actor_' + self.id + '/cmd_motion', ActorMotion, queue_size=10)
        self.gazeboModelstate = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
        print('actor_' + self.id + ": " + "communication initialized")
        self.left_actors_sub = rospy.Subscriber("/left_actors",String,self.left_actors_callback,queue_size=1)
        self.find_actor_sub = rospy.Subscriber("/find_actor_%s"%self.id, Float32, self.actor_broadcast_callback, queue_size=1)
        self.actor_states_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.actor_states_callback, queue_size=1)
        self.uav_odom_subs = []
        for uav_id in range(self.uav_num):
            topic = '/xtdrone/{0}_{1}/ground_truth/odom'.format(
                self.vehicle_type, uav_id)
            self.uav_odom_subs.append(rospy.Subscriber(
                topic, Odometry, self.uav_odom_callback,
                callback_args=uav_id, queue_size=1))

    @staticmethod
    def _find_data_file(candidates):
        for path in candidates:
            if path and os.path.isfile(path):
                return path
        raise IOError('None of the data files were found: %s' %
                      ', '.join(str(path) for path in candidates if path))
        

    def actor_broadcast_callback(self, msg):
        """Arm escape after the scoring system reports this actor.

        A report is only permission to escape.  Proximity to a UAV is checked
        separately from its ground-truth odometry, so a slow or stationary UAV
        does not make the actor run.
        """
        if self.reported:
            return
        self.reported = True
        print('actor_' + self.id + ' was reported; waiting for a fast UAV')


    def uav_odom_callback(self, msg, uav_id):
        """Store UAV ground truth and arm escape when its trigger is met."""
        pose = msg.pose.pose.position
        velocity = msg.twist.twist.linear
        speed = math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)
        self.uav_odom[uav_id] = (pose.x, pose.y, speed)

        if (self.reported and not self.escape_triggered and
                not self.escape_active and
                self.actor_pose_ready and
                math.hypot(self.current_pose.x - pose.x,
                           self.current_pose.y - pose.y) <= self.escape_distance and
                speed > self.escape_trigger_speed):
            self.escape_requested = True
            self.escape_triggered = True
            print('actor_' + self.id + ' escape triggered by typhoon_h480_' +
                  str(uav_id))

    @staticmethod
    def _valid_position(position):
        try:
            return (math.isfinite(float(position.x)) and
                    math.isfinite(float(position.y)) and
                    math.isfinite(float(getattr(position, 'z', 0.0))))
        except (AttributeError, TypeError, ValueError):
            return False

    def _update_actor_pose(self):
        """Read the world pose without replacing a valid pose on failure."""
        try:
            state = self.gazeboModelstate('actor_' + self.id, 'world')
            if not getattr(state, 'success', True):
                self.actor_pose_ready = False
                return False
            position = state.pose.position
            if not self._valid_position(position):
                self.actor_pose_ready = False
                return False
        except (rospy.ServiceException, AttributeError, TypeError, ValueError):
            self.actor_pose_ready = False
            return False

        self.last_pose = copy.deepcopy(self.current_pose)
        self.current_pose = copy.deepcopy(position)
        self.gazebo_actor_pose = copy.deepcopy(position)
        self.actor_pose_ready = True
        return True


    def _box_indices_at(self, x, y, clearance=0.0):
        """Return map obstacles containing a point, including clearance."""
        if not (math.isfinite(x) and math.isfinite(y) and
                math.isfinite(clearance) and clearance >= 0.0):
            return []
        result = []
        for index, box in enumerate(self.black_box):
            if (box[0][0] - clearance <= x <= box[0][1] + clearance and
                    box[1][0] - clearance <= y <= box[1][1] + clearance):
                result.append(index)
        return result


    def _near_obstacle_point(self, x, y, clearance=1.5):
        if not (math.isfinite(x) and math.isfinite(y) and
                math.isfinite(clearance) and clearance >= 0.0):
            return True
        clearance_squared = clearance * clearance
        grid_size = float(getattr(self, 'obstacle_grid_size', 2.0))
        grid = getattr(self, 'obstacle_grid', {})
        cell_x = int(math.floor(x / grid_size))
        cell_y = int(math.floor(y / grid_size))
        cell_radius = int(math.ceil(clearance / grid_size))
        for offset_x in range(-cell_radius, cell_radius + 1):
            for offset_y in range(-cell_radius, cell_radius + 1):
                for obstacle_x, obstacle_y in grid.get(
                        (cell_x + offset_x, cell_y + offset_y), []):
                    if ((x - obstacle_x) ** 2 + (y - obstacle_y) ** 2 <=
                            clearance_squared):
                        return True
        return False


    def _point_is_safe(self, x, y, box_clearance=1.5,
                       obstacle_clearance=1.5):
        if not (math.isfinite(x) and math.isfinite(y)):
            return False
        if (x < self.x_min or x > self.x_max or
                y < self.y_min or y > self.y_max):
            return False
        if self._box_indices_at(x, y, box_clearance):
            return False
        return not self._near_obstacle_point(x, y, obstacle_clearance)


    @staticmethod
    def _distance_to_segment(start_x, start_y, target_x, target_y,
                             point_x, point_y):
        delta_x = target_x - start_x
        delta_y = target_y - start_y
        length_squared = delta_x * delta_x + delta_y * delta_y
        if length_squared <= 1e-12:
            return math.hypot(point_x - start_x, point_y - start_y)
        ratio = ((point_x - start_x) * delta_x +
                 (point_y - start_y) * delta_y) / length_squared
        ratio = max(0.0, min(1.0, ratio))
        nearest_x = start_x + ratio * delta_x
        nearest_y = start_y + ratio * delta_y
        return math.hypot(point_x - nearest_x, point_y - nearest_y)

    def _segment_is_safe(self, start_x, start_y, target_x, target_y,
                         clearance=1.0, allow_recovery=False):
        """Check the complete swept walking segment against map geometry.

        Checking only the two endpoints permits a command received between
        controller iterations to jump through a thin wall.  Rectangle
        intersection is evaluated analytically and point obstacles use the
        exact distance to the segment.
        """
        values = (start_x, start_y, target_x, target_y, clearance)
        if (not all(math.isfinite(value) for value in values) or
                clearance < 0.0):
            return False
        if (start_x < self.x_min or start_x > self.x_max or
                start_y < self.y_min or start_y > self.y_max or
                target_x < self.x_min or target_x > self.x_max or
                target_y < self.y_min or target_y > self.y_max):
            return False

        for box in self.black_box:
            try:
                xmin, xmax = sorted((float(box[0][0]), float(box[0][1])))
                ymin, ymax = sorted((float(box[1][0]), float(box[1][1])))
            except (IndexError, TypeError, ValueError):
                return False
            if not all(math.isfinite(value) for value in
                       (xmin, xmax, ymin, ymax)):
                return False

            start_inside = (xmin <= start_x <= xmax and
                            ymin <= start_y <= ymax)
            # Segment/rectangle intersection using a slab test.  Boundary
            # contact counts as blocked, matching Gazebo's collision guard.
            expanded_xmin = xmin - clearance
            expanded_xmax = xmax + clearance
            expanded_ymin = ymin - clearance
            expanded_ymax = ymax + clearance
            dx = target_x - start_x
            dy = target_y - start_y
            entry, exit = 0.0, 1.0
            for start, delta, low, high in (
                    (start_x, dx, expanded_xmin, expanded_xmax),
                    (start_y, dy, expanded_ymin, expanded_ymax)):
                if abs(delta) < 1e-12:
                    if low <= start <= high:
                        continue
                    break
                first = (low - start) / delta
                last = (high - start) / delta
                if first > last:
                    first, last = last, first
                entry = max(entry, first)
                exit = min(exit, last)
                if entry > exit:
                    break
            else:
                intersects_solid = self._segment_intersects_rectangle(
                    start_x, start_y, target_x, target_y,
                    xmin, xmax, ymin, ymax)
                if intersects_solid:
                    if not (allow_recovery and
                            self._bounded_outward_recovery(
                                start_x, start_y, target_x, target_y,
                                xmin, xmax, ymin, ymax)):
                        return False
                elif entry <= exit:
                    # Starting in the expanded safety margin is only allowed
                    # to move tangentially or away from the solid box.
                    if not self._moves_away_from_rectangle(
                            start_x, start_y, target_x, target_y,
                            xmin, xmax, ymin, ymax):
                        return False

        for obstacle_x, obstacle_y in getattr(self, 'obstacle_data', []):
            try:
                obstacle_x = float(obstacle_x)
                obstacle_y = float(obstacle_y)
            except (TypeError, ValueError):
                return False
            if not math.isfinite(obstacle_x) or not math.isfinite(obstacle_y):
                return False
            segment_distance = self._distance_to_segment(
                start_x, start_y, target_x, target_y, obstacle_x, obstacle_y)
            if segment_distance <= clearance:
                start_distance = math.hypot(start_x - obstacle_x,
                                            start_y - obstacle_y)
                target_distance = math.hypot(target_x - obstacle_x,
                                             target_y - obstacle_y)
                direction_x = target_x - start_x
                direction_y = target_y - start_y
                outward_x = start_x - obstacle_x
                outward_y = start_y - obstacle_y
                moving_outward = (outward_x * direction_x +
                                  outward_y * direction_y) >= -1e-9
                if not (start_distance < clearance and
                        target_distance > start_distance and moving_outward):
                    return False
        return True

    @staticmethod
    def _segment_intersects_rectangle(start_x, start_y, target_x, target_y,
                                      xmin, xmax, ymin, ymax):
        dx = target_x - start_x
        dy = target_y - start_y
        entry, exit = 0.0, 1.0
        for start, delta, low, high in (
                (start_x, dx, xmin, xmax), (start_y, dy, ymin, ymax)):
            if abs(delta) < 1e-12:
                if start < low or start > high:
                    return False
                continue
            first = (low - start) / delta
            last = (high - start) / delta
            if first > last:
                first, last = last, first
            entry = max(entry, first)
            exit = min(exit, last)
            if entry > exit:
                return False
        return exit >= 0.0 and entry <= 1.0

    @staticmethod
    def _moves_away_from_rectangle(start_x, start_y, target_x, target_y,
                                   xmin, xmax, ymin, ymax):
        nearest_x = min(max(start_x, xmin), xmax)
        nearest_y = min(max(start_y, ymin), ymax)
        away_x = start_x - nearest_x
        away_y = start_y - nearest_y
        return (away_x * (target_x - start_x) +
                away_y * (target_y - start_y)) >= -1e-9

    @staticmethod
    def _bounded_outward_recovery(start_x, start_y, target_x, target_y,
                                  xmin, xmax, ymin, ymax):
        if not (xmin <= start_x <= xmax and ymin <= start_y <= ymax):
            return False
        dx = target_x - start_x
        dy = target_y - start_y
        distance = math.hypot(dx, dy)
        if distance <= 1e-9 or distance > 4.0:
            return False
        distances = (start_x - xmin, xmax - start_x,
                     start_y - ymin, ymax - start_y)
        nearest = min(distances)
        if nearest > 4.0:
            return False
        progress = False
        epsilon = 1e-9
        if distances[0] <= nearest + epsilon and dx < -epsilon:
            progress = True
        if distances[1] <= nearest + epsilon and dx > epsilon:
            progress = True
        if distances[2] <= nearest + epsilon and dy < -epsilon:
            progress = True
        if distances[3] <= nearest + epsilon and dy > epsilon:
            progress = True
        return progress


    def _segment_is_safe_from_actors(self, start_x, start_y, target_x,
                                     target_y):
        """Reject routes that sweep through another actor's safety disc."""
        own_name = 'actor_' + self.id
        try:
            radius = max(0.0, float(getattr(self, 'actor_min_distance', 2.0)))
        except (TypeError, ValueError):
            radius = 2.0
        for actor_name, actor_pose in getattr(self, 'actor_positions', {}).items():
            if actor_name == own_name:
                continue
            try:
                actor_x = float(actor_pose.x)
                actor_y = float(actor_pose.y)
            except (AttributeError, TypeError, ValueError):
                continue
            if (not math.isfinite(actor_x) or not math.isfinite(actor_y) or
                    self._distance_to_segment(start_x, start_y, target_x,
                                              target_y, actor_x,
                                              actor_y) < radius):
                return False
        return True

    def _route_is_safe(self, start_pose, waypoints, clearance=1.0):
        previous_x = start_pose.x
        previous_y = start_pose.y
        for waypoint in waypoints:
            if not (self._valid_position(waypoint) and
                    math.isfinite(float(previous_x)) and
                    math.isfinite(float(previous_y))):
                return False
            if not self._point_is_safe(waypoint.x, waypoint.y,
                                       clearance, clearance):
                return False
            allow_recovery = bool(self._box_indices_at(previous_x,
                                                       previous_y, 0.0))
            if not self._segment_is_safe(previous_x, previous_y,
                                         waypoint.x, waypoint.y, clearance,
                                         allow_recovery=allow_recovery):
                return False
            if not self._segment_is_safe_from_actors(
                    previous_x, previous_y, waypoint.x, waypoint.y):
                return False
            previous_x = waypoint.x
            previous_y = waypoint.y
        return True

    def _obstacle_exit_targets(self, clearance=1.0):
        """Return short, safe exits when Gazebo spawned an actor in a box."""
        exits = []
        x, y = self.current_pose.x, self.current_pose.y
        for box in self.black_box:
            try:
                xmin, xmax = sorted((float(box[0][0]), float(box[0][1])))
                ymin, ymax = sorted((float(box[1][0]), float(box[1][1])))
            except (IndexError, TypeError, ValueError):
                continue
            if not (xmin <= x <= xmax and ymin <= y <= ymax):
                continue
            candidates = [
                (xmin - clearance - 0.35, y),
                (xmax + clearance + 0.35, y),
                (x, ymin - clearance - 0.35),
                (x, ymax + clearance + 0.35)]
            distances = [math.hypot(px - x, py - y) for px, py in candidates]
            for px, py in sorted(zip(candidates, distances), key=lambda item: item[1]):
                target_x, target_y = px
                if distances[candidates.index(px)] > 4.0:
                    continue
                if (self.x_min <= target_x <= self.x_max and
                        self.y_min <= target_y <= self.y_max and
                        self._segment_is_safe(x, y, target_x, target_y,
                                              clearance, allow_recovery=True)):
                    point = Point()
                    point.x, point.y = target_x, target_y
                    exits.append(point)
        return exits


    def _plan_safe_route(self, start_pose, target, clearance=1.5):
        """Find a collision-free route using visibility around box corners."""
        if not self._point_is_safe(target.x, target.y, clearance, clearance):
            return []

        if (self._segment_is_safe(start_pose.x, start_pose.y,
                                  target.x, target.y, clearance) and
                self._segment_is_safe_from_actors(
                    start_pose.x, start_pose.y, target.x, target.y)):
            return [target]

        # If a map was regenerated while Gazebo kept an actor in a building,
        # first move to the nearest verified exit.  The C++ plugin applies the
        # same bounded outward-recovery rule.
        if self._box_indices_at(start_pose.x, start_pose.y, 0.0):
            exits = self._obstacle_exit_targets(clearance)
            if exits:
                for exit_point in exits:
                    remaining = self._plan_safe_route(exit_point, target,
                                                      clearance)
                    if remaining:
                        return [exit_point] + remaining
            return []

        # Expanded corners provide deterministic detours around rectangular
        # buildings.  A small extra margin avoids the inclusive box boundary.
        corner_margin = clearance + 0.25
        candidates = []
        for box in self.black_box:
            xmin, xmax = box[0]
            ymin, ymax = box[1]
            for x, y in ((xmin - corner_margin, ymin - corner_margin),
                         (xmin - corner_margin, ymax + corner_margin),
                         (xmax + corner_margin, ymin - corner_margin),
                         (xmax + corner_margin, ymax + corner_margin)):
                if (self.x_min <= x <= self.x_max and
                        self.y_min <= y <= self.y_max and
                        self._point_is_safe(x, y, clearance, clearance)):
                    candidates.append((x, y))

        # Remove duplicate corners produced by overlapping obstacle boxes.
        candidates = list(set(candidates))
        nodes = [(start_pose.x, start_pose.y)] + candidates + [
            (target.x, target.y)]
        target_index = len(nodes) - 1
        distances = [float('inf')] * len(nodes)
        previous = [None] * len(nodes)
        distances[0] = 0.0
        queue = [(0.0, 0)]

        while queue:
            distance, node_index = heapq.heappop(queue)
            if distance != distances[node_index]:
                continue
            if node_index == target_index:
                break

            start_x, start_y = nodes[node_index]
            for next_index, (next_x, next_y) in enumerate(nodes):
                if next_index == node_index:
                    continue
                edge_distance = math.hypot(next_x - start_x,
                                           next_y - start_y)
                candidate_distance = distance + edge_distance
                if candidate_distance >= distances[next_index]:
                    continue
                if (not self._segment_is_safe(start_x, start_y,
                                              next_x, next_y, clearance) or
                        not self._segment_is_safe_from_actors(
                            start_x, start_y, next_x, next_y)):
                    continue
                distances[next_index] = candidate_distance
                previous[next_index] = node_index
                heapq.heappush(queue, (candidate_distance, next_index))

        if previous[target_index] is None:
            return []

        route = []
        index = target_index
        while index != 0:
            x, y = nodes[index]
            waypoint = Point()
            waypoint.x = x
            waypoint.y = y
            route.append(waypoint)
            index = previous[index]
        route.reverse()
        return route


    def _build_safe_route(self, target, clearance):
        """Use the legacy planner, then the local verified planner as fallback."""
        # Every command path uses at least the configured wall clearance.  A
        # smaller caller value must not let normal walking erode the margin
        # used by escape and collision recovery.
        clearance = max(float(clearance), float(getattr(
            self, 'wall_clearance', 1.0)))
        planned_route = []
        if self.Obstacleavoid is not None:
            try:
                planned_route = self.Obstacleavoid.GetPointList(
                    self.current_pose, target, clearance)
            except Exception as exc:
                print('actor_' + self.id + ' legacy route planning failed: ' +
                      str(exc))

        if planned_route and self._route_is_safe(self.current_pose,
                                                 planned_route, clearance):
            return copy.deepcopy(planned_route)

        planned_route = self._plan_safe_route(self.current_pose, target,
                                              clearance)
        if planned_route and self._route_is_safe(self.current_pose,
                                                 planned_route, clearance):
            return planned_route
        return []


    def choose_safe_target(self):
        """Choose a random target that has a verified collision-free route."""
        for unused in range(250):
            target_x = random.uniform(self.x_min + self.spon_dis,
                                      self.x_max - self.spon_dis)
            target_y = random.uniform(self.y_min + self.spon_dis,
                                      self.y_max - self.spon_dis)
            if not self._point_is_safe(target_x, target_y, self.spon_dis):
                continue
            target = Point()
            target.x = target_x
            target.y = target_y
            if self._plan_safe_route(self.current_pose, target):
                return target_x, target_y

        # A local fallback prevents the actor from stalling if the map is
        # temporarily crowded or the obstacle data is denser than expected.
        fallback_distance = 8.0
        candidates = [
            (self.current_pose.x + fallback_distance, self.current_pose.y),
            (self.current_pose.x - fallback_distance, self.current_pose.y),
            (self.current_pose.x, self.current_pose.y + fallback_distance),
            (self.current_pose.x, self.current_pose.y - fallback_distance),
        ]
        for target_x, target_y in candidates:
            target_x = max(self.x_min + self.spon_dis,
                           min(self.x_max - self.spon_dis, target_x))
            target_y = max(self.y_min + self.spon_dis,
                           min(self.y_max - self.spon_dis, target_y))
            target = Point()
            target.x = target_x
            target.y = target_y
            if (self._point_is_safe(target_x, target_y, self.spon_dis) and
                    self._plan_safe_route(self.current_pose, target)):
                return target_x, target_y
        return self.current_pose.x, self.current_pose.y


    def _keep_command_outside_obstacles(self, command):
        if (not math.isfinite(getattr(command, 'v', 0.0)) or
                command.v <= 0.0):
            command.x = self.current_pose.x
            command.y = self.current_pose.y
            command.v = 0.0
            return command

        if (self._point_is_safe(command.x, command.y, 1.0, 1.0) and
                self._segment_is_safe(self.current_pose.x, self.current_pose.y,
                                      command.x, command.y, 1.0) and
                self._segment_is_safe_from_actors(
                    self.current_pose.x, self.current_pose.y,
                    command.x, command.y)):
            return command

        for radius in (1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 8.0):
            for step in range(24):
                angle = 2.0 * math.pi * step / 24.0
                target_x = self.current_pose.x + radius * math.cos(angle)
                target_y = self.current_pose.y + radius * math.sin(angle)
                if (self._point_is_safe(target_x, target_y, 1.0, 1.0) and
                        self._segment_is_safe(self.current_pose.x,
                                              self.current_pose.y,
                                              target_x, target_y, 1.0) and
                        self._segment_is_safe_from_actors(
                            self.current_pose.x, self.current_pose.y,
                            target_x, target_y)):
                    command.x = target_x
                    command.y = target_y
                    return command

        # Stopping at the current pose is safer than issuing a command into a
        # building when no nearby safe point is available.
        command.x = self.current_pose.x
        command.y = self.current_pose.y
        command.v = 0.0
        return command


    def create_human_point(self):
        count = 1
        spon_dis = 5
        while count < 1e5:
            a = random.uniform(-40, 110)
            b = random.uniform(-40, 40)
            candidate_x = int(a)
            candidate_y = int(b)
            unsafe = False

            if not self._point_is_safe(candidate_x, candidate_y, spon_dis):
                unsafe = True

            if unsafe:
                count += 1
                continue

            own_name = 'actor_' + self.id
            for actor_name, actor_pose in self.actor_positions.items():
                if actor_name == own_name:
                    continue
                if math.hypot(candidate_x - actor_pose.x,
                              candidate_y - actor_pose.y) < self.actor_spawn_distance:
                    unsafe = True
                    break

            if unsafe:
                count += 1
                continue

            uav_points = list(self.uav_takeoff_points)
            uav_points.extend((pose.x, pose.y) for pose in self.uav_positions.values())
            for uav_x, uav_y in uav_points:
                if math.hypot(candidate_x - uav_x,
                              candidate_y - uav_y) < self.uav_spawn_distance:
                    unsafe = True
                    break

            if not unsafe:
                return candidate_x, candidate_y
            
            count += 1
        
        print('ERROR: Actor generation failed after 100,000 attempts!')
        return None


    def left_actors_callback(self, msg):
        try:
            values = ast.literal_eval(msg.data)
            if not isinstance(values, (list, tuple)):
                raise ValueError('expected a list of actor ids')
            self.left_actors = [int(value) for value in values]
        except (ValueError, SyntaxError, TypeError) as exc:
            print('actor_' + self.id + ' invalid /left_actors message: ' +
                  str(exc))

    def actor_states_callback(self, msg):
        actor_positions = {}
        uav_positions = {}
        names = getattr(msg, 'name', None)
        poses = getattr(msg, 'pose', None)
        if not isinstance(names, (list, tuple)) or not isinstance(poses, (list, tuple)):
            self.actor_positions = actor_positions
            self.uav_positions = uav_positions
            return
        for index, model_name in enumerate(names):
            if index >= len(poses):
                continue
            position = getattr(poses[index], 'position', None)
            if not self._valid_position(position):
                continue
            try:
                if model_name.startswith('actor_'):
                    actor_positions[model_name] = position
                elif model_name.startswith(self.vehicle_type + '_'):
                    uav_positions[model_name] = position
            except AttributeError:
                continue
        self.actor_positions = actor_positions
        self.uav_positions = uav_positions

    def choose_escape_target(self):
        """Choose a distant point away from the nearest UAV after broadcast."""
        margin = 5.0
        candidates = [
            (self.x_min + margin, self.y_min + margin),
            (self.x_min + margin, self.y_max - margin),
            (self.x_max - margin, self.y_min + margin),
            (self.x_max - margin, self.y_max - margin),
        ]

        if self.uav_positions:
            nearest_uav = min(
                self.uav_positions.values(),
                key=lambda pose: ((self.current_pose.x - pose.x) ** 2 +
                                  (self.current_pose.y - pose.y) ** 2))
            target_x, target_y = max(
                candidates,
                key=lambda point: ((point[0] - nearest_uav.x) ** 2 +
                                   (point[1] - nearest_uav.y) ** 2 +
                                   0.1 * ((point[0] - self.current_pose.x) ** 2 +
                                          (point[1] - self.current_pose.y) ** 2)))
        else:
            target_x, target_y = candidates[int(self.id) % len(candidates)]

        target = Point()
        target.x = target_x
        target.y = target_y
        return target

    def start_escape_route(self):
        self.target_motion = self.choose_escape_target()
        self.route_waiting = False
        self.subtarget_pos = self._build_safe_route(self.target_motion, 1.0)
        if not self.subtarget_pos:
            print('actor_' + self.id + ' escape route unavailable; choosing a local target')
            target_x, target_y = self.choose_safe_target()
            self.target_motion.x = target_x
            self.target_motion.y = target_y
            self.subtarget_pos = self._build_safe_route(self.target_motion, 1.0)

        if not self.subtarget_pos:
            # Do not issue an unverified long-range command.  Hold only until
            # the next stall-recovery cycle can try a bounded local exit.
            self.subtarget_pos = [copy.deepcopy(self.current_pose)]
            self.route_waiting = True
        self.subtarget_length = len(self.subtarget_pos)
        self.subtarget_count = 0
        self.avoid.x = self.subtarget_pos[0].x
        self.avoid.y = self.subtarget_pos[0].y
        self.avoid_finish_flag = False
        self.distance_flag = False
        self.arrive_count = 0

    def _recover_from_stall(self):
        """Select a short verified sidestep and resume the current route."""
        speed = self.escape_speed if self.escape_active else self.normal_speed
        current_x, current_y = self.current_pose.x, self.current_pose.y
        desired_x = self.avoid.x - current_x
        desired_y = self.avoid.y - current_y
        desired_angle = math.atan2(desired_y, desired_x)
        candidates = []
        for radius in (2.0, 3.0, 4.0):
            for offset in (0.0, math.pi / 2.0, -math.pi / 2.0,
                           math.pi, math.pi / 4.0, -math.pi / 4.0):
                angle = desired_angle + offset
                point = Point()
                point.x = max(self.x_min, min(self.x_max,
                                              current_x + radius * math.cos(angle)))
                point.y = max(self.y_min, min(self.y_max,
                                              current_y + radius * math.sin(angle)))
                if not self._point_is_safe(point.x, point.y, 1.0, 1.0):
                    continue
                if not self._segment_is_safe(current_x, current_y,
                                             point.x, point.y, 1.0,
                                             allow_recovery=True):
                    continue
                if not self._segment_is_safe_from_actors(
                        current_x, current_y, point.x, point.y):
                    continue
                score = math.hypot(point.x - self.avoid.x,
                                   point.y - self.avoid.y)
                candidates.append((score, point))

        exits = self._obstacle_exit_targets(1.0)
        for point in exits:
            candidates.append((math.hypot(point.x - self.avoid.x,
                                          point.y - self.avoid.y), point))

        if candidates:
            candidates.sort(key=lambda item: item[0])
            first = candidates[0][1]
            self.subtarget_pos = [first]
            self.subtarget_length = 1
            self.subtarget_count = 0
            self.avoid.x = first.x
            self.avoid.y = first.y
            self.avoid.v = speed
            self.avoid_finish_flag = False
            self.distance_flag = False
            self.arrive_count = 0
            self.route_waiting = False
        else:
            self.route_waiting = True
            self.avoid.x = current_x
            self.avoid.y = current_y
            self.avoid.v = 0.0
        self.shooting_count = 0

    def update_escape_state(self):
        if self.escape_requested:
            self.escape_requested = False
            self.escape_active = True
            self.start_escape_route()

    def build_motion_command(self):
        """Steer away from nearby actors without changing the planned waypoint."""
        command = ActorMotion()
        command.x = self.avoid.x
        command.y = self.avoid.y
        command.v = self.avoid.v

        desired_x = command.x - self.current_pose.x
        desired_y = command.y - self.current_pose.y
        desired_length = math.sqrt(desired_x ** 2 + desired_y ** 2)
        if desired_length > 1e-6:
            steer_x = desired_x / desired_length
            steer_y = desired_y / desired_length
        else:
            steer_x = 0.0
            steer_y = 0.0

        separation_x = 0.0
        separation_y = 0.0
        nearby_actor = False
        nearby_uav = False
        emergency_separation = False
        own_name = 'actor_' + self.id

        for actor_name, actor_pose in self.actor_positions.items():
            if actor_name == own_name:
                continue

            try:
                actor_x = float(actor_pose.x)
                actor_y = float(actor_pose.y)
            except (AttributeError, TypeError, ValueError):
                continue
            if not (math.isfinite(actor_x) and math.isfinite(actor_y)):
                continue
            away_x = self.current_pose.x - actor_x
            away_y = self.current_pose.y - actor_y
            distance = math.sqrt(away_x ** 2 + away_y ** 2)
            if distance >= self.actor_avoidance_radius:
                continue

            nearby_actor = True
            if distance < 1e-6:
                # Give exactly overlapping actors deterministic, different exits.
                angle = (int(self.id) + 1) * 2.0 * math.pi / (self.actor_num + 1)
                unit_x = math.cos(angle)
                unit_y = math.sin(angle)
            else:
                unit_x = away_x / distance
                unit_y = away_y / distance

            strength = 2.0 * (self.actor_avoidance_radius - distance) / self.actor_avoidance_radius
            steer_x += unit_x * strength
            steer_y += unit_y * strength
            separation_x += unit_x
            separation_y += unit_y
            if distance < self.actor_min_distance:
                emergency_separation = True

        # A UAV inside 7 m pushes the pedestrian away, but this does not arm
        # or start the escape behavior.  Escape is armed by a report and
        # triggered independently by the 20 m / 1 m/s odometry rule.
        for uav_state in self.uav_odom.values():
            try:
                uav_x, uav_y, unused_speed = uav_state
                uav_x, uav_y = float(uav_x), float(uav_y)
            except (TypeError, ValueError):
                continue
            if not (math.isfinite(uav_x) and math.isfinite(uav_y)):
                continue
            away_x = self.current_pose.x - uav_x
            away_y = self.current_pose.y - uav_y
            distance = math.sqrt(away_x ** 2 + away_y ** 2)
            if distance >= self.uav_push_radius:
                continue

            nearby_actor = True
            nearby_uav = True
            if distance < 1e-6:
                angle = (int(self.id) + 1) * 2.0 * math.pi / (self.actor_num + 1)
                unit_x = math.cos(angle)
                unit_y = math.sin(angle)
            else:
                unit_x = away_x / distance
                unit_y = away_y / distance

            strength = 4.0 * (self.uav_push_radius - distance) / self.uav_push_radius
            steer_x += unit_x * strength
            steer_y += unit_y * strength
            separation_x += unit_x * 2.0
            separation_y += unit_y * 2.0
            if distance < self.actor_min_distance:
                emergency_separation = True

        if not nearby_actor:
            return self._keep_command_outside_obstacles(command)

        if emergency_separation:
            steer_x = separation_x
            steer_y = separation_y

        steer_length = math.sqrt(steer_x ** 2 + steer_y ** 2)
        if steer_length < 1e-6:
            angle = (int(self.id) + 1) * 2.0 * math.pi / (self.actor_num + 1)
            steer_x = math.cos(angle)
            steer_y = math.sin(angle)
            steer_length = 1.0

        push_radius = self.uav_push_radius if nearby_uav else self.actor_avoidance_radius
        command.x = self.current_pose.x + push_radius * steer_x / steer_length
        command.y = self.current_pose.y + push_radius * steer_y / steer_length
        command.x = max(self.x_min, min(self.x_max, command.x))
        command.y = max(self.y_min, min(self.y_max, command.y))
        return self._keep_command_outside_obstacles(command)

    def loop(self):
        rate = rospy.Rate(self.f)
        
        while not rospy.is_shutdown():
            self.count = self.count + 1
            # get the pose of uav and actor
            if not int(self.id) in self.left_actors:
                print('actor_' + self.id + ' has been deleted')
                break
            if not self._update_actor_pose():
                # Publishing the last valid pose with zero speed prevents a
                # stale target from carrying the actor through a wall while
                # Gazebo's state service is temporarily unavailable.
                hold = ActorMotion()
                hold.x = self.current_pose.x
                hold.y = self.current_pose.y
                hold.v = 0.0
                self.cmd_pub.publish(hold)
                rate.sleep()
                continue
            self.update_escape_state()
            # collosion: if the actor is in the black box, then go backward and update a new target position
            # update new random target position
            if (self.avoid_finish_flag and self.distance_flag and
                    not self.escape_active):
                self.suitable_point = False
                self.x, self.y = self.choose_safe_target()
                self.target_motion.x = self.x
                self.target_motion.y = self.y
                self.flag = False
                self.distance_flag = False
                self.suitable_point = True
                self.escape_suce_flag = False
                self.subtarget_pos = self._build_safe_route(
                    self.target_motion, 0.5)
                if not self.subtarget_pos:
                    print('actor_' + self.id +
                          ' route unavailable; holding position')
                    self.subtarget_pos = [copy.deepcopy(self.current_pose)]
                    self.route_waiting = True
                else:
                    self.route_waiting = False

                self.subtarget_length = len(self.subtarget_pos)
                self.subtarget_count = 0
                self.arrive_count = 0
                self.avoid.x = self.subtarget_pos[0].x
                self.avoid.y = self.subtarget_pos[0].y
                self.avoid_finish_flag = False
                if self.id == '5' or self.id == '4':
                    print(self.id + ' general change position')
                    print('current_position:   ' + self.id + '      ', self.current_pose)
                    print('subtarget_pos: ' + self.id + '      ', self.subtarget_pos)
                    print('\n')

            distance = (self.current_pose.x - self.avoid.x) ** 2 + (
                            self.current_pose.y - self.avoid.y) ** 2
            if distance < 0.01:
                self.arrive_count += 1
                if self.arrive_count > 5:
                    self.distance_flag = True
                    self.arrive_count = 0
                else:
                    self.distance_flag = False
            else:
                self.arrive_count = 0
                self.distance_flag = False            

            # check if the actor is shot:
            if self.get_moving:
                distance_change = (self.last_pose.x - self.current_pose.x)**2 + (self.last_pose.y - self.current_pose.y)**2
                if distance_change < 0.00001:
                    self.shooting_count = self.shooting_count+1
                    if self.shooting_count > self.stall_limit_cycles:
                        print('actor_' + self.id + ' stalled; selecting a local recovery route')
                        self._recover_from_stall()
                else:
                    self.shooting_count = 0

            

            if not self.avoid_finish_flag:
                if self.route_waiting:
                    # The current pose is only a temporary placeholder while
                    # route generation is retried.  Never consume it as a
                    # completed waypoint or end an active escape here.
                    self.distance_flag = False
                elif self.distance_flag:
                    self.subtarget_count += 1
                    self.distance_flag = False
                    #print self.id+ ': I am avoiding'
                    #self.distance_flag = False
                    if self.subtarget_count >= self.subtarget_length:
                        self.avoid_finish_flag = True
                        self.subtarget_count = 0
                        if self.escape_active:
                            self.escape_active = False
                            print('actor_' + self.id + ' finished escape route')
                    else:
                        self.avoid.x = self.subtarget_pos[self.subtarget_count].x
                        self.avoid.y = self.subtarget_pos[self.subtarget_count].y
            
            if self.escape_active:
                self.avoid.v = self.escape_speed
            else:
                self.avoid.v = self.normal_speed
            # if self.id == 5:
            #     print('self.avoid:', self.avoid)
            self.cmd_pub.publish(self.build_motion_command())
            rate.sleep()


    def random_move(self):
        dis_list = [0,0,0,0]
        for i in range(len(self.black_box)):
            if (self.current_pose.x > self.black_box[i][0][0]-self.spon_dis) and (self.current_pose.x < self.black_box[i][0][1]+1.0):
                if (self.current_pose.y > self.black_box[i][1][0]-1.0) and (self.current_pose.y < self.black_box[i][1][1]+1.0):
                    dis_list[0] = abs(self.current_pose.x - (self.black_box[i][0][0] - 1.0))
                    dis_list[1] = abs(self.current_pose.x - (self.black_box[i][0][1] + 1.0))
                    dis_list[2] = abs(self.current_pose.y - (self.black_box[i][1][0] - 1.0))
                    dis_list[3] = abs(self.current_pose.y - (self.black_box[i][1][1] + 1.0))
                    dis_min = dis_list.index(min(dis_list))
                    self.subtarget_length = 1
                    if dis_min == 0:
                        self.avoid.x = self.black_box[i][0][0] - 1.0
                        self.avoid.y = self.current_pose.y
                    elif dis_min == 1:
                        self.avoid.x = self.black_box[i][0][1] + 1.0
                        self.avoid.y = self.current_pose.y
                    elif dis_min == 2:
                        
                        self.avoid.y = self.black_box[i][1][0] - 1.0
                        self.avoid.x = self.current_pose.x
                    else:
                        self.avoid.y = self.black_box[i][1][1] + 1.0
                        self.avoid.x = self.current_pose.x
                    break



    def pos2ang(self, deltax, deltay):   #([xb,yb] to [xa, ya])
        if not deltax == 0:
            angle = math.atan2(deltay,deltax)
            if (deltay > 0) and (angle < 0):
                angle = angle + math.pi
            elif (deltay < 0) and (angle > 0):
                angle = angle - math.pi
            elif deltay == 0:
                if deltax > 0:
                    angle = 0.0
                else:
                    angle = math.pi
        else:
            if deltay > 0:
                angle = math.pi / 2
            elif deltay <0:
                angle = -math.pi / 2
            else:
                angle = 0.0
        if angle < 0:
            angle = angle + 2 * math.pi   # 0 to 2pi
        return angle


if __name__=="__main__":
    controlactors = ControlActor(sys.argv[1])
    controlactors.loop()


