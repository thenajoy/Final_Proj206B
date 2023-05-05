import rospy, os
from copy import deepcopy
import numpy as np

from qrotor_ground.arena import Arena

from qrotor_firmware.msg import Log
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger, TriggerRequest
from qrotor_firmware.srv import Setpoint, SetpointRequest, FlatTrajectory, FlatTrajectoryRequest
from qrotor_ground.cfg import DroneCommandsConfig, TrajectoriesConfig
from dynamic_reconfigure.server import Server as DynamicReconfigureServer

class DroneManager(object):
    def __init__(self, name):
        # - - - - - - - -
        # parameters
        self.name = name

        self.rosservice_timeout = 2
        self.arena = Arena()

        # - - - - - -
        # variables
        self.log = Log()
        self.odom = Odometry()
        self._position = np.array([0., 0., 0.])
        self._velocity = np.array([0., 0., 0.])

        # - - - - - - - - - - -
        # rosservice requests
        self.move_req = SetpointRequest()
        self.setpoint_req = SetpointRequest()
        self.prev_setpoint_mode = 0

        self.traj_req = FlatTrajectoryRequest()

        # - - - - - - - - - -
        self.sub_log = rospy.Subscriber('/'+name+'/log', Log, self.callback_sub_log, queue_size=1)
        self.sub_odom = rospy.Subscriber('/'+name+'/odometry/mocap', Odometry, self.callback_sub_odom, queue_size=1)
        self.server = DynamicReconfigureServer(DroneCommandsConfig, self.reconfigure_cmds, namespace=name)
        self.server = DynamicReconfigureServer(TrajectoriesConfig, self.reconfigure_trajs, namespace=name+'/trajectory')


    def voltage(self):
        return self.log.voltage

    @property
    def position(self):
        return self._position
    
    @position.setter
    def position(self, value):
        self._position = np.array(value)

    @property
    def velocity(self):
        return self._velocity
    
    @velocity.setter
    def velocity(self, value):
        self._velocity = np.array(value)

    def callback_sub_log(self, msg):
        self.log = deepcopy(msg)

    def callback_sub_odom(self, msg):
        self.odom_ = deepcopy(msg)
        self.position = np.array([self.odom_.pose.pose.position.x, self.odom_.pose.pose.position.y, self.odom_.pose.pose.position.z])
        # print(self.position)

    def verify_arena_bounds(self):
        return self.arena.is_valid_position(self.position)

    def request_arm(self):
        rospy.logwarn('requesting arming for ' + self.name)
        service_name = '/'+self.name+'/arm'
        try:
            rospy.wait_for_service(service_name, timeout=self.rosservice_timeout)
            resp = rospy.ServiceProxy(service_name, Trigger)
            resp(TriggerRequest())
        except rospy.exceptions.ROSException as e:
            rospy.logerr("Service call failed: %s"%e)
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s"%e)

    def request_disarm(self):
        rospy.logwarn('requesting dis-arming for ' + self.name)
        service_name = '/'+self.name+'/disarm'
        try:
            rospy.wait_for_service(service_name, timeout=self.rosservice_timeout)
            resp = rospy.ServiceProxy(service_name, Trigger)
            resp(TriggerRequest())
        except rospy.exceptions.ROSException as e:
            rospy.logerr("Service call failed: %s"%e)
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s"%e)

    def request_kill_vehicle(self):
        if self.log.motors_state:
            rospy.logerr('requesting killing ' + self.name)
            service_name = '/'+self.name+'/kill'
            try:
                rospy.wait_for_service(service_name, timeout=self.rosservice_timeout)
                resp = rospy.ServiceProxy(service_name, Trigger)
                resp(TriggerRequest())
            except rospy.exceptions.ROSException as e:
                rospy.logerr("Service call failed: %s"%e)
            except rospy.ServiceException as e:
                rospy.logwarn("Service call failed: %s"%e)

    def request_a_move(self):
        rospy.logwarn('requesting ' + self.name + 'to move by ')
        service_name = '/'+self.name+'/move'
        try:
            rospy.wait_for_service(service_name, timeout=self.rosservice_timeout)
            resp = rospy.ServiceProxy(service_name, Setpoint)
            resp(self.move_req)
        except rospy.exceptions.ROSException as e:
            rospy.logerr("Service call failed: %s"%e)
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s"%e)

    def request_setpoint(self):
        rospy.logwarn('requesting ' + self.name + ' to move to')
        service_name = '/'+self.name+'/move_to'
        try:
            rospy.wait_for_service(service_name, timeout=self.rosservice_timeout)
            resp = rospy.ServiceProxy(service_name, Setpoint)
            resp(self.setpoint_req)
        except rospy.exceptions.ROSException as e:
            rospy.logerr("Service call failed: %s"%e)
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s"%e)

    def request_takeoff(self):
        rospy.logwarn('requesting takeoff for ' + self.name)
        service_name = '/'+self.name+'/takeoff'
        try:
            rospy.wait_for_service(service_name, timeout=self.rosservice_timeout)
            resp = rospy.ServiceProxy(service_name, Trigger)
            resp(TriggerRequest())
        except rospy.exceptions.ROSException as e:
            rospy.logerr("Service call failed: %s"%e)
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s"%e)

    def request_landing(self):
        rospy.logwarn('requesting landing for ' + self.name)
        service_name = '/'+self.name+'/land'
        try:
            rospy.wait_for_service(service_name, timeout=self.rosservice_timeout)
            resp = rospy.ServiceProxy(service_name, Trigger)
            resp(TriggerRequest())
        except rospy.exceptions.ROSException as e:
            rospy.logerr("Service call failed: %s"%e)
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s"%e)

    def request_activate_mission(self):
        rospy.logwarn('requesting landing for ' + self.name)
        service_name = '/'+self.name+'/activate_mission'
        try:
            rospy.wait_for_service(service_name, timeout=self.rosservice_timeout)
            resp = rospy.ServiceProxy(service_name, Trigger)
            resp(TriggerRequest())
        except rospy.exceptions.ROSException as e:
            rospy.logerr("Service call failed: %s"%e)
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s"%e)


    def request_kill_mission(self):
        rospy.logwarn('requesting landing for ' + self.name)
        service_name = '/'+self.name+'/kill_mission'
        try:
            rospy.wait_for_service(service_name, timeout=self.rosservice_timeout)
            resp = rospy.ServiceProxy(service_name, Trigger)
            resp(TriggerRequest())
        except rospy.exceptions.ROSException as e:
            rospy.logerr("Service call failed: %s"%e)
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s"%e)


    def request_trajectory(self):
        rospy.logwarn('requesting trajectory to ' + self.name)
        service_name = '/'+self.name+'/flat_trajectory'
        try:
            rospy.wait_for_service(service_name, timeout=self.rosservice_timeout)
            resp = rospy.ServiceProxy(service_name, FlatTrajectory)
            resp(self.traj_req)
        except rospy.exceptions.ROSException as e:
            rospy.logerr("Service call failed: %s"%e)
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s"%e)

    def reconfigure_cmds(self, config, level):
        rospy.loginfo("""Reconfigure Request: {mode}, {x}, {y}, {z}""".format(**config))
        if config['mode']==0:
            if (self.prev_setpoint_mode == 1):
                self.prev_setpoint_mode = 0
                config['x'], config['y'], config['z'] = 0, 0, 0
                return config
            self.move_req.x, self.move_req.y, self.move_req.z = config['x'], config['y'], config['z']
        if config['mode']==1:
            if (self.prev_setpoint_mode == 0):
                self.prev_setpoint_mode = 1
                config['x'], config['y'], config['z'] = 0, 0, 0
                return config
            self.setpoint_req.x, self.setpoint_req.y, self.setpoint_req.z = config['x'], config['y'], config['z']
        if config['send']:
            if config['mode'] == 0:
                config['x'], config['y'], config['z'] = 0, 0, 0
                self.request_a_move()
            if config['mode'] == 1:
                self.request_setpoint()
            config['send'] = False 

        if config['arm']:
            self.request_arm()
            config['arm']=False
        if config['disarm']:
            self.request_disarm()
            config['disarm']=False
        if config['kill']:
            self.request_kill_vehicle()
            config['kill']=False
        if config['takeoff']:
            self.request_takeoff()
            config['takeoff'] = False
        if config['land']:
            self.request_landing()
            config['land'] = False
        if config['activate_mission']:
            self.request_activate_mission()
            config['activate_mission'] = False
        if config['kill_mission']:
            self.request_kill_mission()
            config['kill_mission'] = False



        return config
    
    def reconfigure_trajs(self, config, level):
        rospy.loginfo("""Circle center: {cx}, {cy}, {cz}, Radius: {radius}, Phase: {phase}""".format(**config))

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

