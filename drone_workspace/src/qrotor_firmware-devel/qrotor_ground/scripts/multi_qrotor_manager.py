#!/usr/bin/env python
from time import sleep
import rospy, os
import numpy as np
import threading
import time
from matplotlib import pyplot as plt

from qrotor_ground.vehicle_manager.quadrotor import QuadrotorManager, EVENT, MODE
from qrotor_ground.utils.catenary_tension import *

from qrotor_ground.cfg import MultiDroneManagerConfig, DroneCommandsConfig, CatenaryConfig
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3

class MultiQrotorManager(object):
    """
    Multi Quadrotor Manager including computing position and control
    """
    def __init__(self, names=None):
        """
        initialization
        """
        # - - - - - - - - - - - - - - -
        # time variables
        self.loop_rate_ = 200 # Hz
        self.prev_time_ = rospy.get_time()
        self.curr_time_ = rospy.get_time()
        self.dt_        = 0.
        self.start_time = rospy.get_time()

        # - - - - - - - - - - - - - - -
        # create drone managers
        self.names = names
        self.drones = []
        for name in names:
            self.drones.append(QuadrotorManager(name, time_step=(1/self.loop_rate_)))
        self.n = len(self.drones)

        # - - - - - - - - - - - - - - -
        self.setpoints = [ np.array([0., -1.0, 1.0]),
                                np.array([0., 0.0, 1.0]),
                                np.array([0., 1.0, 1.0])]
        self.update_setpoints()
        self._seperation_distance = 0.5
        self._send_cmds = False
        self._in_offboard_mode = False
        self.move_req = np.zeros(3)

        # - - - - - - - - - - - - - - -
        # cable related info
        self.cable_length = 2.0 # TODO update these
        self.cable_linear_density = 0.35 # TODO update these values
        self.compute_catenary_force = False

        # - - - - - - - - - - - - - - -
        self.takeoff_all_pub = rospy.Publisher('/takeoffall', Bool, queue_size=1)
        self.kill_all_pub = rospy.Publisher('/killall', Bool, queue_size=1)
        self.land_all_pub = rospy.Publisher('/landall', Bool, queue_size=1)
        self.mission_all_pub = rospy.Publisher('/missionall', Bool, queue_size=1)
        self.move_all_pub = rospy.Publisher('/moveall', Vector3, queue_size=1)
        self.offb_server = DynamicReconfigureServer(MultiDroneManagerConfig, self.offb_reconfigCb, namespace="all/offboard")
        self.cmds_server = DynamicReconfigureServer(DroneCommandsConfig, self.cmd_reconfigCb, namespace="all")
        self.catenary_server = DynamicReconfigureServer(CatenaryConfig, self.catenary_reconfigCb, namespace="all/catenary")

        # - - - - - - - - - - - - - - -
        # debug
        self.live_plot_thread = None


        # - - - - - - - - - - - - - - -

    def catenary_reconfigCb(self, config, level):
        self.cable_linear_density = config['rho']
        self.cable_length = config['length']
        self.compute_catenary_force = config['use_catenary_tension']
        return config

    def cmd_reconfigCb(self, config, level):
        if config['mode'] == 0:
            self.move_req = np.array([config['x'], config['y'], config['z']])
        if config['mode'] == 1:
            pass
        if config['send']:
            if config['mode'] == 0:
                self.request_move()
                config['x'] = 0
                config['y'] = 0
                config['z'] = 0
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
            self.request_takeoff(offboard=False)
            config['takeoff'] = False
        if config['land']:
            self.request_landing(offboard=False)
            config['land'] = False
        if config['activate_mission']:
            self.request_activate_mission()
            config['activate_mission'] = False
        if config['kill_mission']:
            self.request_kill_mission()
            config['kill_mission'] = False
        return config

    def offb_reconfigCb(self, config, level):
        if self.n == 1:
            self.setpoints[0] = np.array([config['x'], config['y'], config['z']])
        elif self.n == 2:
            self.setpoints[0] = np.array([config['x'], config['y']-config['distance'], config['z']])
            self.setpoints[1] = np.array([config['x'], config['y']+config['distance'], config['z']])                
        elif self.n == 3:
            self.setpoints[1] = np.array([config['x'], config['y'], config['z']])
            self.setpoints[0] = np.array([config['x'], config['y']-config['distance'], config['z']])
            self.setpoints[2] = np.array([config['x'], config['y']+config['distance'], config['z']])
        else:
            rospy.logerr('More than 3 drones are not supported at this time!')
        print(self.setpoints)

        if config['set']:
            self.update_setpoints()
            config['set'] = False

        # self._send_cmds = config['send_cmds']
        if config['set_offboard'] and not self._in_offboard_mode:
            self._in_offboard_mode = config['set_offboard']
            self.request_offb_control()
        if not config['set_offboard'] and self._in_offboard_mode:
            self._in_offboard_mode = config['set_offboard']
            self.request_offb_control()
        
        if config['activate_mission']:
            self.request_activate_mission()
            config['activate_mission'] = False
        # if config['kill_mission']:
        #     self.request_kill_mission()
        #     config['kill_mission'] = False

        if config['takeoff']:
            self.request_takeoff(offboard=True)
            config['takeoff'] = False
        if config['landall']:
            self.request_landing(offboard=True)
            config['landall'] = False
        if config['killall']:
            self.kill_all_vehicles()
            config['killall'] = False


        # rospy.loginfo("l: %f, yaw: %f", config['distance'], config['yaw'])
        return config

    def request_move(self):
        rospy.logwarn("moving all vehicles")
        msg = Vector3()
        msg.x = self.move_req[0]
        msg.y = self.move_req[1]
        msg.z = self.move_req[2]
        self.move_all_pub.publish(msg)


    def arm_all_vehicles(self):
        rospy.logwarn("arming all vehicles")
        threads = []
        for i in range(self.n):
            t = threading.Thread(target=self.drones[i].request_arm)
            threads.append(t)
            t.start()

    def disarm_all_vehicles(self):
        rospy.logwarn("disarming all vehicles")
        threads = []
        for i in range(self.n):
            t = threading.Thread(target=self.drones[i].request_disarm)
            threads.append(t)
            t.start()

    def request_takeoff(self, offboard=False):
        if (offboard):
            threads = []
            t = rospy.get_time()-self.start_time
            for i in range(self.n):
                thrd = threading.Thread(target=self.drones[i]._mdl.set_event, args=(t, EVENT.REQUEST_TAKEOFF,))
                threads.append(thrd)
                thrd.start()
        else:
            rospy.logwarn("takeoff all vehicles")
            msg = Bool()
            msg.data = True
            self.takeoff_all_pub.publish(msg)


    def kill_all_vehicles(self):
        msg = Bool()
        msg.data = True
        self.kill_all_pub.publish(msg)

    def request_landing(self, offboard=False):
        if offboard:
            threads = []
            t = rospy.get_time()-self.start_time
            for i in range(self.n):
                thrd = threading.Thread(target=self.drones[i]._mdl.set_event, args=(t, EVENT.REQUEST_LAND,))
                threads.append(thrd)
                thrd.start()
        else:
            rospy.logwarn("landing all vehicles")
            msg = Bool()
            msg.data = True
            self.land_all_pub.publish(msg)            

    def request_activate_mission(self):
        rospy.logwarn("starting mission for all vehicles")
        msg = Bool()
        msg.data = True
        self.mission_all_pub.publish(msg)

    def update_setpoints(self):
        for i in range(self.n):
            self.drones[i]._mdl.setpoint = self.setpoints[i]

    def update_time(self):
        """
        time update
        """
        self.curr_time_ = rospy.get_time()
        self.dt_        = self.curr_time_ - self.prev_time_
        self.prev_time_ = self.curr_time_
        # print("------------------")
        # print(self.start_time, self.curr_time_, self.prev_time_)
        # rospy.loginfo('Frequency: %f Hz',1/self.dt_)

    def publish_cmd_thrust(self):
        pass

    def request_offb_control(self):
        rospy.logwarn("activate offboard control for all vehicles")
        threads = []
        for i in range(self.n):
            t = threading.Thread(target=self.drones[i].request_offboard, args=(self._in_offboard_mode,))
            threads.append(t)
            t.start()

    def start(self):
        """
        Ground Station Node Run
        """
        # self.live_plot_thread = threading.Thread(target=self.live_update_demo, args=(True,))

        self.start_time = rospy.get_time()
        gnd_loop_rate = rospy.Rate(self.loop_rate_)
        self.update_time()
        ind = 0
        # running ros loop
        while (not rospy.is_shutdown()):
            self.update_time()
            self.run_tasks()
            # loop
            gnd_loop_rate.sleep()

    def run_tasks(self):
        # run individual drones
        if self.n >= 2 and self.compute_catenary_force:
            self.computeCatenaryforces()

        for drone in self.drones:
                drone.runControl(self._send_cmds)

    def computeCatenaryforces(self):
        print(self.drones[0].setpoint)
        print(self.drones[1].setpoint)
        t0, t1 = catenary_in_3d(self.drones[0].position, self.drones[1].position, self.cable_length, self.cable_linear_density)
        print(t0, t1)
        self.drones[0].update_ff_force(t0)
        self.drones[1].update_ff_force(t1)


    # def live_update_demo(blit = False):
    #     x = np.linspace(0,50., num=100)
    #     X,Y = np.meshgrid(x,x)
    #     fig = plt.figure()
    #     ax1 = fig.add_subplot(2, 1, 1)
    #     ax2 = fig.add_subplot(2, 1, 2)

    #     img = ax1.imshow(X, vmin=-1, vmax=1, interpolation="None", cmap="RdBu")


    #     line, = ax2.plot([], lw=3)
    #     text = ax2.text(0.8,0.5, "")

    #     ax2.set_xlim(x.min(), x.max())
    #     ax2.set_ylim([-1.1, 1.1])

    #     fig.canvas.draw()   # note that the first draw comes before setting data 


    #     if blit:
    #         # cache the background
    #         axbackground = fig.canvas.copy_from_bbox(ax1.bbox)
    #         ax2background = fig.canvas.copy_from_bbox(ax2.bbox)

    #     plt.show(block=False)


    #     t_start = time.time()
    #     k=0.

    #     for i in np.arange(1000):
    #         img.set_data(np.sin(X/3.+k)*np.cos(Y/3.+k))
    #         line.set_data(x, np.sin(x/3.+k))
    #         tx = 'Mean Frame Rate:\n {fps:.3f}FPS'.format(fps= ((i+1) / (time.time() - t_start)) ) 
    #         text.set_text(tx)
    #         #print tx
    #         k+=0.11
    #         if blit:
    #             # restore background
    #             fig.canvas.restore_region(axbackground)
    #             fig.canvas.restore_region(ax2background)

    #             # redraw just the points
    #             ax1.draw_artist(img)
    #             ax2.draw_artist(line)
    #             ax2.draw_artist(text)

    #             # fill in the axes rectangle
    #             fig.canvas.blit(ax1.bbox)
    #             fig.canvas.blit(ax2.bbox)

    #             # in this post http://bastibe.de/2013-05-30-speeding-up-matplotlib.html
    #             # it is mentionned that blit causes strong memory leakage. 
    #             # however, I did not observe that.

    #         else:
    #             # redraw everything
    #             fig.canvas.draw()

    #         fig.canvas.flush_events()
    #         #alternatively you could use
    #         #plt.pause(0.000000000001) 
    #         # however plt.pause calls canvas.draw(), as can be read here:
    #         #http://bastibe.de/2013-05-30-speeding-up-matplotlib.html


# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
if __name__ == '__main__':
    node_name = os.path.splitext(os.path.basename(__file__))[0]
    rospy.init_node(node_name)
    rospy.loginfo('Starting [%s] node' % node_name)

    vehicles_ = rospy.get_param("/vehicles")
    vehicle_names = vehicles_.split()
    print(vehicle_names)

    ground_state_ = MultiQrotorManager(names=vehicle_names)
    ground_state_.start()
    
    rospy.loginfo('Shutting down [%s] node' % node_name)
