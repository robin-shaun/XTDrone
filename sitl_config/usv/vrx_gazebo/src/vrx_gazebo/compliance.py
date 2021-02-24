#!/usr/bin/env python
import rospy
import rospkg
import numpy as np
import os
import yaml

from vrx_gazebo.utils import get_macros


class SensorCompliance:
    def __init__(self):

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('vrx_gazebo')
        self.config_dir = os.path.join(pkg_path, 'config', 'wamv_config')

        # open sensor_compliance/bounding_boxes.yaml and all the boxes defined
        self.boxes = find_boxes(os.path.join(self.config_dir,
            'sensor_compliance', 'bounding_boxes.yaml'))
        # look at all sensors in sensors directory and get the default params
        self.sensors_dir = rospy.get_param('sensors_dir') + '/'
        self.default_parameters = get_macros(self.sensors_dir)
        self.numeric = yaml.safe_load(open(os.path.join(self.config_dir,
            'sensor_compliance', 'numeric.yaml')))
        return

    def param_compliance(self, sensor_type, params={}):
        # ie: given an instance of sensor_type = 'wamv_camera'
        # with parameters = params, is this camera in compliance
        # check if the sensor is allowed
        params = params.copy()
        if sensor_type not in self.default_parameters:
            rospy.logerr('%s is not defined anywhere under %s' %
                         (sensor_type, self.config_dir))
        assert sensor_type in self.default_parameters,\
            '%s is not defined anywhere under %s' % (sensor_type, self.config_dir)
        for i in params:
            if i not in self.numeric[sensor_type]['allowed_params']:
                rospy.logerr('%s parameter specification of %s not permitted' %
                             (i, sensor_type))

        # add the default params to params if not specified
        for i in self.default_parameters[sensor_type]:
            if i not in params:
                params[i] = self.default_parameters[sensor_type][i]
        if 'x' and 'y' and 'z' in params:
            xyz = np.array([float(params['x']),
                            float(params['y']),
                            float(params['z'])])
            for box in self.boxes:
                if box.fit(xyz):
                    return True
            rospy.logerr('%s %s is out of bounds' %
                         (sensor_type, params['name']))
            rospy.logerr('%s %s is at xyz=(%s, %s, %s), %s' %
                         (sensor_type, params['name'],
                          xyz[0], xyz[1], xyz[2],
                          'must fit in at least one of the following boxes ' +
                          'with remaining space:'))
            for box in self.boxes:
                rospy.logerr('  %s' % str(box))
            return False
        else:
            return True

    def number_compliance(self, sensor_type, n):
        # ie: are n wamv_cameras allowed?
        if n > self.numeric[sensor_type]['num']:
            rospy.logerr('Too many %s requested' % sensor_type)
            rospy.logerr('  maximum of %s %s allowed' %
                         (self.numeric[sensor_type]['num'], sensor_type))
            return False
        return True


class ThrusterCompliance:
    def __init__(self):
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('vrx_gazebo')
        self.config_dir = os.path.join(pkg_path, 'config', 'wamv_config')

        # open thruster_compliance/bounding_boxes.yaml and the boxes defined
        self.boxes = find_boxes(os.path.join(self.config_dir,
            'thruster_compliance', 'bounding_boxes.yaml'))
        # look at all sensors in sensors directory and get the default params
        self.thrusters_dir = rospy.get_param('thrusters_dir') + '/'
        self.default_parameters = get_macros(self.thrusters_dir)
        self.numeric = yaml.safe_load(open(os.path.join(self.config_dir,
            'thruster_compliance', 'numeric.yaml')))
        return

    def param_compliance(self, thruster_type, params={}):
        # ie: given an instance of thruster_type = 'engine'
        # with parameters = params, is this engine in compliance
        # check if the thruster is allowed

        params = params.copy()
        if thruster_type not in self.default_parameters:
            rospy.logerr('%s is not defined anywhere under %s' %
                         (thruster_type, self.config_dir))
        assert thruster_type in self.default_parameters,\
            '%s is not defined anywhere under %s' % \
            (thruster_type, self.config_dir)
        for i in params:
            if i not in self.numeric[thruster_type]['allowed_params']:
                rospy.logerr('%s parameter specification of not permitted' %
                             (i, thruster_type))
                assert False

        # add the default params to params if not specified
        for i in self.default_parameters[thruster_type]:
            if i not in params:
                params[i] = self.default_parameters[thruster_type][i]
        # right now the ONLY compliance check we have is to make sure that
        # the sensors are in at least one of the boxes
        xyz = np.array([float(j) for j in [i for i in
                                           params['position'].split(' ')
                                           if i != '']])
        for box in self.boxes:
            if box.fit(xyz):
                return True
        rospy.logerr('%s %s is out of bounds' %
                     (thruster_type, params['prefix']))
        rospy.logerr('%s %s is at xyz=(%s, %s, %s), %s' %
                     (thruster_type, params['prefix'],
                      xyz[0], xyz[1], xyz[2],
                      'it must fit in at least one of the following boxes ' +
                      'with remaining space:'))
        for box in self.boxes:
            rospy.logerr('  %s' % str(box))
        return False

    def number_compliance(self, thruster_type, n):
        if n > self.numeric[thruster_type]['num']:
            rospy.logerr('Too many %s requested' % thruster_type)
            rospy.logerr('  maximum of %s %s allowed' %
                         (self.numeric[thruster_type]['num'], thruster_type))
            return False
        return True


class Box:
    def __init__(self, name, pose, size, space):
        self.name = name

        self.pose = np.array([float(j) for j in [i for i in pose.split(' ')
                              if i != '']])
        self.size = np.array([float(j) for j in [i for i in size.split(' ')
                              if i != '']])
        self.space = int(space)
        return

    def fit(self, pose):
        pose = pose-self.pose[:3]
        for idx, i in enumerate(pose):
            if abs(i) > self.size[idx]/2:
                return False
        # if space is -1, unlimited things
        if self.space == -1:
            return True
        elif self.space == 0:
            return False
        elif self.space > 0:
            self.space -= 1
            return True

    def __str__(self):
        return '<Box name:%s x:[%s, %s] y:[%s,%s] z:[%s,%s]\
                remaining_space:%s>' % \
                (self.name,
                    (self.pose[0] + self.size[0]/2),
                    (self.pose[0] - self.size[0]/2),
                    (self.pose[1] + self.size[1]/2),
                    (self.pose[1] - self.size[1]/2),
                    (self.pose[2] + self.size[2]/2),
                    (self.pose[2] - self.size[2]/2),
                    self.space)


def find_boxes(box_yaml):
    box_def = yaml.safe_load(open(box_yaml))

    boxes = []

    for name, properties in box_def.iteritems():
        boxes.append(Box(str(name),
                         properties['pose'],
                         properties['size'],
                         properties['capacity']))
    return boxes
