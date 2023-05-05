#!/usr/bin/env python
import re
import rospy
import os
from copy import deepcopy
import numpy as np
import math

from qrotor_ground.arena import Arena
from qrotor_ground.dynamics.quadrotor import Quadrotor, EVENT, MODE

from std_msgs.msg import String
from geometry_msgs.msg import Vector3Stamped, TwistStamped
from qrotor_firmware.msg import Log
from nav_msgs.msg import Odometry

from std_srvs.srv import Trigger, TriggerRequest, SetBool, SetBoolRequest
from qrotor_firmware.srv import Setpoint, SetpointRequest, FlatTrajectory, FlatTrajectoryRequest

from qrotor_ground.cfg import DroneCommandsConfig, TrajectoriesConfig
from dynamic_reconfigure.server import Server as DynamicReconfigureServer


class QuadrotorManager(object):
    def __init__(self, name, time_step):
        # - - - - - - - -
        # parameters
        self.name = name

        self.rosservice_timeout = 2
        self.arena = Arena()
        # - - - - - -
        # dynamics and control model
        self._mdl = Quadrotor(h=time_step, mass=0.775, CTRL='pid')

        # - - - - - -
        # variables
        self.log = Log()
        self.odom = Odometry()

        # - - - - - - - - - - -
        # rosservice requests
        self.move_req = SetpointRequest()
        self.setpoint_req = SetpointRequest()
        self.prev_setpoint_mode = 0
        self.traj_req = FlatTrajectoryRequest()
        self._yaw_sp = 0  # radians

        self._in_offboard_mode = False

        # - - - - - - - - - -
        self.sub_log = rospy.Subscriber(
            '/'+name+'/log', Log, self.callback_sub_log, queue_size=1)
        self.sub_odom_est = rospy.Subscriber(
            '/'+name+'/odom/estimate', Odometry, self.callback_sub_odom_est, queue_size=1)
        self.sub_odom_mocap = rospy.Subscriber(
            '/'+name+'/odometry/mocap', Odometry, self.callback_sub_odom_mocap, queue_size=1)
        self.thrustPublisher = rospy.Publisher(
            '/' + name + '/thrust_force', TwistStamped, queue_size=1)
        self.setpointPublisher = rospy.Publisher(
            '/' + name + '/offboard/setpoint', Vector3Stamped, queue_size=1)

        self.server = DynamicReconfigureServer(
            DroneCommandsConfig, self.reconfigure_cmds, namespace=name)
        self.server = DynamicReconfigureServer(
            TrajectoriesConfig, self.reconfigure_trajs, namespace=name+'/trajectory')

        self.start_time = rospy.get_time()
        # - - - - - - - - - -

    def voltage(self):
        return self.log.voltage

    def callback_sub_log(self, msg):
        self.log = deepcopy(msg)

    def odom2mdl(self, msg):
        self._mdl.position = [msg.pose.pose.position.x,
                              msg.pose.pose.position.y, msg.pose.pose.position.z]
        (x, y, z, w) = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        self._mdl.orientation = np.array([[1-2*y*y-2*z*z, 2*x*y-2*z*w, 2*x*z+2*y*w],
                                          [2*x*y+2*z*w, 1-2*x*x -
                                              2*z*z, 2*y*z-2*x*w],
                                          [2*x*z-2*y*w, 2*y*z+2*x*w, 1-2*x*x-2*y*y]])
        self._mdl.velocity = [msg.twist.twist.linear.x,
                              msg.twist.twist.linear.y, msg.twist.twist.linear.z]
        self._mdl.ang_vel = [msg.twist.twist.angular.x,
                             msg.twist.twist.angular.y, msg.twist.twist.angular.z]

    def callback_sub_odom_mocap(self, msg):
        self.odom2mdl(msg)

    def callback_sub_odom_est(self, msg):
        self.odom2mdl(msg)
        # self.mdl.velocity, self.mdl.ang_vel  = p.getBaseVelocity(self.mdlId)
        # self.mdl.orientation = np.reshape(np.array(p.getMatrixFromQuaternion(quat)),(3,3))
        # self.odom_ = deepcopy(msg)
        # self.position = np.array([self.odom_.pose.pose.position.x, self.odom_.pose.pose.position.y, self.odom_.pose.pose.position.z])

    @property
    def position(self):
        return self._mdl.position

    @property
    def velocity(self):
        return self._mdl.velocity

    @property
    def setpoint(self):
        return self._mdl.setpoint

    def run(self):
        # self.state_machine.run()
        self.runControl()

    def update_ff_force(self, ff):
        self._mdl.feedforward_force = ff

    def runControl(self, send=False):
        t = rospy.get_time() - self.start_time
        # print("inside", t)
        thrust_force, (f, M), (V1, V2) = self._mdl.compute_control(t)
        yaw_sp = self._mdl.yaw_sp(t)
        if self._mdl.send:
            self.publish_thrust(thrust_force, yaw_sp)
        else:
            self.publish_thrust()
        self.publish_setpoint()

    def publish_setpoint(self):
        msg = Vector3Stamped()
        sp = self._mdl.setpoint
        msg.header.stamp = rospy.Time.now()
        msg.vector.x, msg.vector.y, msg.vector.z = sp[0], sp[1], sp[2]
        self.setpointPublisher.publish(msg)

    def publish_thrust(self, force=np.zeros(3), yaw=0.):
        # create ros message
        msg = TwistStamped()
        msg.header.stamp = rospy.Time.now()
        msg.twist.linear.x = force[0]  # Nm
        msg.twist.linear.y = force[1]  # Nm
        msg.twist.linear.z = force[2]  # Nm
        msg.twist.angular.z = yaw  # radians
        self.thrustPublisher.publish(msg)

    def verify_arena_bounds(self):
        return self.arena.is_valid_position(self.position)

    def request_arm(self):
        rospy.logwarn('requesting arming for ' + self.name)
        service_name = '/'+self.name+'/arm'
        try:
            rospy.wait_for_service(
                service_name, timeout=self.rosservice_timeout)
            resp = rospy.ServiceProxy(service_name, Trigger)
            resp(TriggerRequest())
        except rospy.exceptions.ROSException as e:
            rospy.logerr("Service call failed: %s" % e)
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s" % e)

    def request_disarm(self):
        rospy.logwarn('requesting dis-arming for ' + self.name)
        service_name = '/'+self.name+'/disarm'
        try:
            rospy.wait_for_service(
                service_name, timeout=self.rosservice_timeout)
            resp = rospy.ServiceProxy(service_name, Trigger)
            resp(TriggerRequest())
        except rospy.exceptions.ROSException as e:
            rospy.logerr("Service call failed: %s" % e)
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s" % e)

    def request_kill_vehicle(self):
        if self.log.motors_state:
            rospy.logerr('requesting killing ' + self.name)
            service_name = '/'+self.name+'/kill'
            try:
                rospy.wait_for_service(
                    service_name, timeout=self.rosservice_timeout)
                resp = rospy.ServiceProxy(service_name, Trigger)
                resp(TriggerRequest())
            except rospy.exceptions.ROSException as e:
                rospy.logerr("Service call failed: %s" % e)
            except rospy.ServiceException as e:
                rospy.logwarn("Service call failed: %s" % e)

    def request_a_move(self):
        if not self._in_offboard_mode:
            rospy.logwarn('requesting ' + self.name + 'to move by ')
            service_name = '/'+self.name+'/move'
            try:
                rospy.wait_for_service(
                    service_name, timeout=self.rosservice_timeout)
                resp = rospy.ServiceProxy(service_name, Setpoint)
                resp(self.move_req)
            except rospy.exceptions.ROSException as e:
                rospy.logerr("Service call failed: %s" % e)
            except rospy.ServiceException as e:
                rospy.logwarn("Service call failed: %s" % e)
        else:
            rospy.loginfo("changing offboard setpoint by %f, %f, %f",
                          self.move_req.x, self.move_req.y, self.move_req.z)
            self._mdl.setpoint = self._mdl.position + np.array(
                [self.move_req.x, self.move_req.y, self.move_req.z])
            # self._mdl._yaw_sp = self._yaw_sp

    def request_setpoint(self):
        if not self._in_offboard_mode:
            rospy.logwarn('requesting ' + self.name + ' to move to')
            service_name = '/'+self.name+'/move_to'
            try:
                rospy.wait_for_service(
                    service_name, timeout=self.rosservice_timeout)
                resp = rospy.ServiceProxy(service_name, Setpoint)
                resp(self.setpoint_req)
            except rospy.exceptions.ROSException as e:
                rospy.logerr("Service call failed: %s" % e)
            except rospy.ServiceException as e:
                rospy.logwarn("Service call failed: %s" % e)
        else:
            rospy.loginfo("changing offboard setpoint to %f, %f, %f",
                          self.setpoint_req.x, self.setpoint_req.y, self.setpoint_req.z)
            self._mdl.setpoint = np.array(
                [self.setpoint_req.x, self.setpoint_req.y, self.setpoint_req.z])
            self._mdl._yaw_sp = self._yaw_sp

    def request_takeoff(self):
        if not self._in_offboard_mode:
            rospy.logwarn('requesting takeoff for ' + self.name)
            service_name = '/'+self.name+'/takeoff'
            try:
                rospy.wait_for_service(
                    service_name, timeout=self.rosservice_timeout)
                resp = rospy.ServiceProxy(service_name, Trigger)
                resp(TriggerRequest())
            except rospy.exceptions.ROSException as e:
                rospy.logerr("Service call failed: %s" % e)
            except rospy.ServiceException as e:
                rospy.logwarn("Service call failed: %s" % e)
        else:
            self._mdl.set_event(
                (rospy.get_time()-self.start_time), EVENT.REQUEST_LAND)

    def request_offboard(self, flag=False):
        if (flag and not self._in_offboard_mode) or (not flag and self._in_offboard_mode):
            rospy.logwarn('requesting offboard control for ' + self.name)
            service_name = '/'+self.name+'/offboard'
            try:
                rospy.wait_for_service(
                    service_name, timeout=self.rosservice_timeout)
                resp = rospy.ServiceProxy(service_name, SetBool)
                req = SetBoolRequest()
                req.data = flag
                resp(req)
                self._in_offboard_mode = flag
            except rospy.exceptions.ROSException as e:
                rospy.logerr("Service call failed: %s" % e)
            except rospy.ServiceException as e:
                rospy.logwarn("Service call failed: %s" % e)

    def request_landing(self):
        if not self._in_offboard_mode:
            rospy.logwarn('requesting landing for ' + self.name)
            service_name = '/'+self.name+'/land'
            try:
                rospy.wait_for_service(
                    service_name, timeout=self.rosservice_timeout)
                resp = rospy.ServiceProxy(service_name, Trigger)
                resp(TriggerRequest())
            except rospy.exceptions.ROSException as e:
                rospy.logerr("Service call failed: %s" % e)
            except rospy.ServiceException as e:
                rospy.logwarn("Service call failed: %s" % e)
        else:
            self._mdl.set_event(
                (rospy.get_time()-self.start_time), EVENT.REQUEST_TAKEOFF)

    def request_activate_mission(self):
        rospy.logwarn('requesting landing for ' + self.name)
        service_name = '/'+self.name+'/activate_mission'
        try:
            rospy.wait_for_service(
                service_name, timeout=self.rosservice_timeout)
            resp = rospy.ServiceProxy(service_name, Trigger)
            resp(TriggerRequest())
        except rospy.exceptions.ROSException as e:
            rospy.logerr("Service call failed: %s" % e)
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s" % e)

    def request_kill_mission(self):
        rospy.logwarn('requesting landing for ' + self.name)
        service_name = '/'+self.name+'/kill_mission'
        try:
            rospy.wait_for_service(
                service_name, timeout=self.rosservice_timeout)
            resp = rospy.ServiceProxy(service_name, Trigger)
            resp(TriggerRequest())
        except rospy.exceptions.ROSException as e:
            rospy.logerr("Service call failed: %s" % e)
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s" % e)

    def request_trajectory(self):
        rospy.logwarn('requesting trajectory to ' + self.name)
        service_name = '/'+self.name+'/flat_trajectory'
        try:
            rospy.wait_for_service(
                service_name, timeout=self.rosservice_timeout)
            resp = rospy.ServiceProxy(service_name, FlatTrajectory)
            resp(self.traj_req)
        except rospy.exceptions.ROSException as e:
            rospy.logerr("Service call failed: %s" % e)
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s" % e)

    def reconfigure_cmds(self, config, level):
        rospy.loginfo(
            """Reconfigure Request: {mode}, {x}, {y}, {z}""".format(**config))
        if config['mode'] == 0:
            if (self.prev_setpoint_mode == 1):
                self.prev_setpoint_mode = 0
                config['x'], config['y'], config['z'] = 0, 0, 0
                return config
            self.move_req.x, self.move_req.y, self.move_req.z = config['x'], config['y'], config['z']
        if config['mode'] == 1:
            if (self.prev_setpoint_mode == 0):
                self.prev_setpoint_mode = 1
                config['x'], config['y'], config['z'] = 0, 0, 0
                return config
            self.setpoint_req.x, self.setpoint_req.y, self.setpoint_req.z = config[
                'x'], config['y'], config['z']
            self._yaw_sp = config['yaw']*math.pi/180.
        if config['send']:
            if config['mode'] == 0:
                config['x'], config['y'], config['z'] = 0, 0, 0
                self.request_a_move()
            if config['mode'] == 1:
                self.request_setpoint()
            config['send'] = False

        if config['arm']:
            self.request_arm()
            config['arm'] = False
        if config['disarm']:
            self.request_disarm()
            config['disarm'] = False
        if config['kill']:
            self.request_kill_vehicle()
            config['kill'] = False
        if config['takeoff']:
            self.request_takeoff()
            config['takeoff'] = False
        if config['land']:
            self.request_landing()
            config['land'] = False
        # if config['activate_mission']:
        #     self.request_activate_mission()
        #     config['activate_mission'] = False
        # if config['kill_mission']:
        #     self.request_kill_mission()
        #     config['kill_mission'] = False

        self.request_offboard(config['OFFBOARD'])

        return config

    def reconfigure_trajs(self, config, level):
        rospy.loginfo(
            """Circle center: {cx}, {cy}, {cz}, Radius: {radius}, Phase: {phase}""".format(**config))

        if config['trajectory'] == 0:
            self.traj_req.type = FlatTrajectoryRequest.TRAJECTORY_STRAIGHT_LINE
            rospy.logerr("Not implemented")

        if config['trajectory'] == 1:
            self.traj_req.type = FlatTrajectoryRequest.TRAJECTORY_CIRCLE_2D
            self.traj_req.center.x = config['cx']
            self.traj_req.center.y = config['cy']
            self.traj_req.center.z = config['cz']
            self.traj_req.radius.x = config['radius']
            self.traj_req.radius.y = config['radius']
            self.traj_req.radius.z = 0
            self.traj_req.phase.x = config['phase']
            self.traj_req.phase.y = config['phase']
            self.traj_req.phase.z = 0

        if config['trajectory'] == 2:
            self.traj_req.type = FlatTrajectoryRequest.TRAJECTORY_CIRCLE_3D
            rospy.logerr("Not implemented")

        if config['trajectory'] == 3:
            self.traj_req.type = FlatTrajectoryRequest.TRAJECTORY_ELLIPSE_3D
            rospy.logerr("Not implemented")

        if config['send']:
            self.request_trajectory()
            config['send'] = False
        return config
