import rospy, os
import numpy as np
import threading

from qrotor_ground.drone_manager import DroneManager
from qrotor_ground.arena import Arena

from qrotor_firmware.srv import Setpoint, SetpointRequest
from rospy.core import logwarn
from std_srvs.srv import Trigger, TriggerRequest
from qrotor_ground.cfg import DroneCommandsConfig
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from std_msgs.msg import Bool


class StationManager(object):
    def __init__(self, drones):
        self.drones = drones
        self.N = len(self.drones)

        self.drone_distance_limit = 0.4  # meters
        self.arena = Arena()
        self.move_req = SetpointRequest()
        self.setpoint_req = SetpointRequest()

        # - - - - - - - - - -
        self.takeoff_all_pub = rospy.Publisher('takeoffall', Bool, queue_size=1)
        self.kill_all_pub = rospy.Publisher('killall', Bool, queue_size=1)
        self.land_all_pub = rospy.Publisher('landall', Bool, queue_size=1)
        self.server = DynamicReconfigureServer(DroneCommandsConfig, self.reconfigure, namespace="all")

    def verify_drone_position_bounds(self):
        for i in range(self.N):
            # print('TODO!')
            for j in range(i + 1, self.N):
                if np.linalg.norm(self.drones[i].position - self.drones[j].position) < self.drone_distance_limit:
                    rospy.logwarn(self.drones[i].name + ' and ' + self.drones[j].name + ' are too close, killing them!')
                    self.kill_all_vehicles()

            if not self.drones[i].verify_arena_bounds():
                rospy.logwarn(self.drones[i].name + " is out of position bounds")
                self.drones[i].request_kill_vehicle()

    # def kill_vehicles(self, indices):

    # threads = []
    # for i in indices:
    #     self.drones[i].move_req = self.move_req
    #     t = threading.Thread(target=self.drones[i].request_kill_vehicle)
    #     threads.append(t)
    #     t.start()

    def kill_all_vehicles(self):
        msg = Bool()
        msg.data = True
        self.kill_all_pub.publish(msg)

    # rospy.logwarn("killing all vehicles")
    # threads = []
    # for i in range(self.N):
    #     self.drones[i].move_req = self.move_req
    #     t = threading.Thread(target=self.drones[i].request_kill_vehicle)
    #     threads.append(t)
    #     t.start()

    def arm_all_vehicles(self):
        rospy.logwarn("arming all vehicles")
        threads = []
        for i in range(self.N):
            self.drones[i].move_req = self.move_req
            t = threading.Thread(target=self.drones[i].request_arm)
            threads.append(t)
            t.start()

    def disarm_all_vehicles(self):
        rospy.logwarn("disarming all vehicles")
        threads = []
        for i in range(self.N):
            self.drones[i].move_req = self.move_req
            t = threading.Thread(target=self.drones[i].request_disarm)
            threads.append(t)
            t.start()

    def request_move(self):
        rospy.logwarn("moving all vehicles")
        threads = []
        for i in range(self.N):
            self.drones[i].move_req = self.move_req
            t = threading.Thread(target=self.drones[i].request_a_move)
            threads.append(t)
            t.start()

    def request_takeoff(self):
        rospy.logwarn("takeoff all vehicles")
        msg = Bool()
        msg.data = True
        self.takeoff_all_pub.publish(msg)
        # threads = []
        # for i in range(self.N):
        #     self.drones[i].move_req = self.move_req
        #     t = threading.Thread(target=self.drones[i].request_takeoff)
        #     threads.append(t)
        #     t.start()

    def request_landing(self):
        rospy.logwarn("landing all vehicles")
        msg = Bool()
        msg.data = True
        self.land_all_pub.publish(msg)
        # threads = []
        # for i in range(self.N):
        #     self.drones[i].move_req = self.move_req
        #     t = threading.Thread(target=self.drones[i].request_landing)
        #     threads.append(t)
        #     t.start()

    def request_activate_mission(self):
        rospy.logwarn("activate mission for all vehicles")
        threads = []
        for i in range(self.N):
            self.drones[i].move_req = self.move_req
            t = threading.Thread(target=self.drones[i].request_activate_mission)
            threads.append(t)
            t.start()

    def request_kill_mission(self):
        rospy.logwarn("kill mission for all vehicles")
        threads = []
        for i in range(self.N):
            self.drones[i].move_req = self.move_req
            t = threading.Thread(target=self.drones[i].request_kill_mission)
            threads.append(t)
            t.start()

    def reconfigure(self, config, level):
        if config['mode'] == 0:
            self.move_req.x, self.move_req.y, self.move_req.z = config['x'], config['y'], config['z']
        if config['mode'] == 1:
            self.setpoint_req.x, self.setpoint_req.y, self.setpoint_req.z = config['x'], config['y'], config['z']
        if config['send']:
            if config['mode'] == 0:
                self.request_move()
            if config['mode'] == 1:
                rospy.logerr("Cannot request the same setpoint for all the vehicles")
            config['send'] = False

        if config['arm']:
            self.arm_all_vehicles()
            config['arm'] = False
        if config['disarm']:
            self.disarm_all_vehicles()
            config['disarm'] = False
        if config['kill']:
            self.kill_all_vehicles()
            config['kill'] = False

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

    def run(self):
        self.verify_drone_position_bounds()
