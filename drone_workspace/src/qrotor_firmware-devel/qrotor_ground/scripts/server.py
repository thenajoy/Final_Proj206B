#!/usr/bin/env python
import rospy

from dynamic_reconfigure.server import Server
import dynamic_reconfigure.client
from qrotor_ground.cfg import DroneCommandsConfig
import IPython as ip

x = 0
# client = dynamic_reconfigure.client.Client("dynamic_tutorials", timeout=30, config_callback=callback1)

def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {mode}, {x},\ 
          {y}, {z}""".format(**config))
    global x
    x = x+1
    config["x"] = x    
    return config

if __name__ == "__main__":
    rospy.init_node("dynamic_tutorials", anonymous = False)

    srv = Server(DroneCommandsConfig, callback)
    rospy.spin()