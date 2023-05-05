#!/usr/bin/env python
#ros dependencies
import rospy
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu, Image
from nav_msgs.msg import Odometry
import numpy as np
#our dependencies
from trajectory import *
from clfpdcontrollers import *
from clfpdlyapunov import *
from state_estimation import *
from gazebo_msgs.msg import ModelStates

class OffboardDrone:
    """
    Offboard control class
    """
    def __init__(self):
        """
        Init function for an offboard drone control object
        Inputs:
        controller: controller object
        """
       
        #define the thrust publisher
        self.pub = rospy.Publisher('/white_falcon/thrust_force', TwistStamped, queue_size=1)

        #store an observer (gets updated in callbacks)
        self.observer = QuadObserver()
        
        # #define trajectory and desired states
        self.des_state = [np.array([[0, 0, 0.75]]).T, np.zeros((3, 1)), np.eye(3), np.zeros((3, 1))]
        self.T = 10 #time for trajectory
        self.finalend = self.observer.get_pos() + self.des_state[0].reshape((3,1))
        self.traj = Trajectory(self.observer.get_pos(), self.observer.get_pos() + self.des_state[0].reshape((3,1)), self.T)        
        
        self.ar_tag_name = "ar_2020"
        #Initialize the subscriber and callbacks to run in the background (Only need to do this once!)
        self.state_sub()

        #store a lyapunov object - don't reset this :)
        self.lyapunov = LyapunovWrotor(3, 3, self.observer, self.traj)

        #store two controllers - pass in self.observer into these controllers
        self.controllerVel = PlanarQrotorLyapunov(self.observer, self.lyapunov, self.traj)
        self.controllerForce = PlanarQrotorPD(self.observer, self.lyapunov, self.traj)
        
        #Initialize time parameters
        self.prev_time = rospy.get_time()
        self.start_time = rospy.get_time()
        self.hz = 100
        self.dt = 1/self.hz #curr - prev 0.02 works well
        self.t = 0 #curr time - start time
    
    def imu_callback(self, data):
        """
        Callback function using IMU data - obtains angular velocity, linear acceleration, and orientation (only store accel)
        """
        #ensure gravity is subtracted from the IMU data
        self.q_acc = np.array([[data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z]]).T - np.array([[0, 0, 9.81]]).T
        
        #update observer
        self.observer.set_accel(self.q_acc)


    def odom_callback(self, data):
        """
        Odometry estimate callback function - gets x, v, R, omega
        """
        q_x = np.array([[data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z]]).T#extract position data
        q_v = np.array([[data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.linear.z]]).T #linear velocity

        #update the observer params
        self.observer.set_pos_vel(q_x, q_v)

    
    def state_sub(self):
        """
        Subscriber node to the IMU and Depth topics
        """
        rospy.init_node("imu_sub", anonymous = True)
        rospy.Subscriber("/white_falcon/imu", Imu, self.imu_callback, queue_size=10, tcp_nodelay=True) #subscribe to IMU data
        rospy.Subscriber("/white_falcon/odometry/mocap", Odometry, self.odom_callback, queue_size=10, tcp_nodelay=True) #subscribe to odometry estimate data
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)
    
    def get_input(self, save = False):
        """
        Function to get control input depending on the state
        """
        
        #first, call the trajectory to get the desired state
        if self.traj.published_counter == 0:
            print("RESETTING TRAJECTORY")
            self.traj.published_counter += 1
            self.traj.first_step = True
        
        #self.traj.reset(self.observer.get_pos(), self.observer.get_vel(), self.observer.get_AR_pos() - (np.array([0.5, 0, 0])).reshape((3,1)), self.T - self.t)

        #define trajectory and desired states
        # self.des_state = [np.array([[0, 0, 0.75]]).T, np.zeros((3, 1)), np.eye(3), np.zeros((3, 1))]
        # self.T = 20 #time for trajectory
        # self.traj = Trajectory(self.observer.get_pos(), np.array([[0, 0, self.observer.get_AR_pos()[2,0]]]).reshape((3,1)), self.T)

        #get desired state
        x_d, v_d, a_d = self.traj.get_state(self.t)

        #get the velocity vector
        x = self.observer.get_pos()
        xD = x_d
        vD = v_d

        # #store a lyapunov object - don't reset this :)
        # self.lyapunov = LyapunovWrotor(3, 3, self.observer, self.traj)

        # #store two controllers - pass in self.observer into these controllers
        # self.controllerVel = PlanarQrotorLyapunov(self.observer, self.lyapunov, self.traj)
        # self.controllerForce = PlanarQrotorPD(self.observer, self.lyapunov, self.traj)
        vel2track = self.controllerVel.eval_vel_vec(self.t, vD, self.finalend + np.array([0.5, 0, 0]).reshape((3,1)))
        print("Position:", self.observer.get_pos().T)
        print("desired pos: ", self.traj.pos(self.t))

        #get the force vector from the controller -> either pass in vel2Track or leave it as none
        force = self.controllerForce.eval_force_vec(self.t, vel2track)

        # print("CASADI: ", force.T)
                
        #convert force vector to a Twist object
        # print("Force: ", force.T)
        msg = TwistStamped()
        msg.header.stamp = rospy.Time.now()
        if self.t < self.T:
            msg.twist.linear.x = force[0, 0]
            msg.twist.linear.y = force[1, 0]
            msg.twist.linear.z = force[2, 0]
            print('time', self.t)
        else:
            print("END")
            msg.twist.linear.x = 0
            msg.twist.linear.y = 0
            msg.twist.linear.z = 9.2214
            print('time 2 ', self.t)

        # print('msg', msg)
        self.pub.publish(msg) #Uncomment to publish commands
        # print(msg)
        return msg
    
    def clock(self):
        """
        Clock function to update system time
        """
        curr_time = rospy.get_time()
        self.t = curr_time - self.start_time
        self.dt = curr_time - self.prev_time
        self.prev_time = curr_time
    
    def run(self):
        """
        Master run function, enforces a rate when calling controller
        Inputs:
        r: rate in Hz, 50 by default
        """
        rate = rospy.Rate(self.hz)
        while not rospy.is_shutdown():
            self.get_input()
            self.clock()
            print("clock freq: ", 1/(self.dt+0.00001)) #add a buffer to prevent zero division
            rate.sleep()

    def callback(self, data):
        index = data.name.index("ar_2020")

        pose = data.pose[index]
        x, y, z = pose.position.x, pose.position.y, pose.position.z
        #roll, pitch, yaw = tf.transformation.euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])

        #print('AR tag position ({}, {}, {})'.format(x,y,z))
        #might need later
        #print('AR tag orientation (rpy): ({}, {}, {})'.format(roll, pitch, yaw))
        self.observer.set_AR_pos(x, y, z)


if __name__ == '__main__':    
    #create an offboard drone object
    offboard_drone = OffboardDrone()
    #run the controller procedure
    offboard_drone.run()

    self.graph()