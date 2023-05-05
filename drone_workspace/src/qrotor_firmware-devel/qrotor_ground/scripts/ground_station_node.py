#!/usr/bin/env python
import rospy, os

from qrotor_ground.drone_manager import DroneManager
from qrotor_ground.station_manager import StationManager
from qrotor_ground.ground_station_gui import *

class GroundControlNode(object):
    """
    Ground Control Node
    """
    def __init__(self, names=None):
        """
        initialization
        """
        # - - - - - - - - - - - - - - -
        # create drone managers
        self.names = names
        self.drones = []
        for name in names:
            self.drones.append(DroneManager(name))

        # - - - - - - - - - - - - - - - 
        # station manager
        self.station_manager = StationManager(self.drones)

        # - - - - - - - - - - - - - - -
        # ground station GUI
        self.gui = GroundStationGUI(names, self.drones, self.station_manager)

        # - - - - - - - - - - - - - - -
        # time variables
        self.loop_rate_ = 100 # Hz
        self.prev_time_ = rospy.get_time()
        self.curr_time_ = rospy.get_time()
        self.dt_        = 0

    def update_time(self):
        """
        time update
        """
        self.curr_time_ = rospy.get_time()
        self.dt_        = self.curr_time_ - self.prev_time_
        self.prev_time_ = self.curr_time_
        # rospy.loginfo('Frequency: %f Hz',1/self.dt_)

    def run(self):
        """
        Ground Station Node Run
        """
        gnd_loop_rate = rospy.Rate(self.loop_rate_)
        self.update_time()
        self.gui_is_good = True
        ind = 0
        # running ros loop
        while (not rospy.is_shutdown()) and self.gui_is_good:
            self.update_time()
            self.station_manager.run()

            # run display at 20Hz
            if ind == 5:
                self.gui_is_good = self.gui.run()
                ind = 0
            ind += 1

            # loop
            gnd_loop_rate.sleep()

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
if __name__ == '__main__':
    node_name = os.path.splitext(os.path.basename(__file__))[0]
    rospy.init_node(node_name)
    rospy.loginfo('Starting [%s] node' % node_name)

    vehicles_ = rospy.get_param("/vehicles")
    vehicle_names = vehicles_.split()
    print(vehicle_names)

    gnd_ctrl_node_ = GroundControlNode(names=vehicle_names)
    gnd_ctrl_node_.run()
    
    rospy.loginfo('Shutting down [%s] node' % node_name)
