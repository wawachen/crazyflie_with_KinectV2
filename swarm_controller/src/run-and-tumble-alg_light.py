#! /usr/bin/env python
import rospy
import rospy_crazyflie.client as crazyflie_client
from rospy_crazyflie.client import Client
from rospy_crazyflie.srv import *
from rospy_crazyflie.msg import *
import time
from collections import deque
import sys
import random
import numpy as np
# from crazyflie_driver.msg import *
from std_msgs.msg import Int32

import tf
from geometry_msgs.msg import PoseStamped


num_samples = 10
takeoff_height = 0.2 # m
forward_vel = 0.1       # m/s
strafe_vel = 0.1        # m/s
turn_deg = 25           # deg
max_intensity = 800   	# lux
obst_dist_thresh = 150  # mm
start_time = 5          # sec
run_time = 0.5            # sec
tumble_time = 0.5       # sec
strafe_time = 2	        # sec
ao_time = 0.5           # sec

def rotate_z(x,y,theta):
    #theta is radians
    x1 = np.cos(theta)*x-np.sin(theta)*y
    y1 = np.sin(theta)*x+np.cos(theta)*y

    return [x1,y1]

class bcfController:
    def __init__(self):
        # Init node
        rospy.init_node('bcf_Controller')

        # Connect to the crazyflie
        self.crazyflies = crazyflie_client.get_crazyflies(server='/crazyflie_server')
        self.client = crazyflie_client.Client(self.crazyflies[0])

        # Create deque for calculating average intensity over last num_samples
        self.d_intensity = deque(maxlen=num_samples)

        # Publishers
        self.action_publisher = rospy.Publisher('bcf_action',
                                                ActionPub, queue_size=10)

        self.speed_max = 0.1

        #--initialization
        self.velocity_x = 0
        self.velocity_y = 0
        self.velocity_z = 0
        self.velocity = np.zeros(4)
        
        #--flocking controller--
        self.far_distance = 0.50
        self.near_distance = 0.10 
        self.k_sep = 1.5 #1.5
        self.k_coh = 1.2
        self.k_frict = 0.1

        self.x = 0
        self.y = 0
        self.z = 0

        self.rangeLeft = 0
        self.rangeRight = 0
        self.rangeFront = 0
        self.rangeBack = 0

        #--bacterium_controller--
        self.T = 0
        self.T0 = 60
        self.T_vary = 0
        self.threshold_value = 30
        self.Tm = 1
        self.alpha = 1000 #300 6000
        self.kd = 2 #default 30

        self.dP_dt = 0
        self.dC_dt = 0
        self.dP_dt_weight = 0
        self.current_concentration = 0
        self.previous_concentration = 0
        self.random_bearing = np.random.uniform(0,360)

        self.memory_capacity = 4
        self.concentration_record = np.zeros(self.memory_capacity)
        self.p_rate_record = np.zeros(self.memory_capacity)
        self.estimated_position = np.zeros(3)

        self.counter = 0
        self.queque_num = 0

        #-------------
        self.current_action = ActionPub()
        self.other_pos = []
        self.pos = np.zeros(2)
        self.end = 0
        self.last_pos_x = 0
        self.last_pos_y = 0
        self.obj_num = 0
        self.obj_signal = 0
        self.yaw_sf = 0

        # Create subscribers
        rospy.Subscriber('/crazyflie/z_range', zRangeData, callback=self.concentration_callback)
        rospy.Subscriber('/crazyflie/bcf_state', KalmanPositionEst, callback=self.state_callback)
        rospy.Subscriber('/crazyflie/multi_range', RangeData, callback=self.range_callback)
        rospy.Subscriber('/crazyflie/drone_pose_green', PoseStamped,callback=self.pos_callback_g)
        rospy.Subscriber('/crazyflie/drone_pose_orange', PoseStamped,callback=self.pos_callback_o)
        rospy.Subscriber('/detector/found_num', Int32, callback=self.found_callback)


    def found_callback(self, data):
        self.obj_num = data.data
        self.obj_signal = 1

        # rospy.spin()
    def pos_callback_g(self,data):
        quaternion = (
        data.pose.orientation.x,
        data.pose.orientation.y,
        data.pose.orientation.z,
        data.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw_g = euler[2]

        self.pos[0] = data.pose.position.x
        self.pos[1] = data.pose.position.y
        self.yaw_sf = yaw_g
    
    def pos_callback_o(self,data):
        self.other_pos = np.array([[data.pose.position.x,data.pose.position.y]])

    def concentration_callback(self, data):
        self.current_concentration = data.down
        return

    def state_callback(self, data):
        self.x = data.stateX
        self.y = data.stateY
        self.z = data.stateZ
        # print("x",self.x)
        return

    def range_callback(self, data):
        self.rangeLeft = data.left
        self.rangeRight = data.right
        self.rangeFront = data.front
        self.rangeBack = data.back
        # print("range",self.rangeLeft)
        return

    def start(self):
        self.client.take_off(takeoff_height)
        self.client.wait()
        rospy.sleep(start_time)
        # self.client.start_forward(forward_vel)
        # self.client.wait()
        # rospy.sleep(run_time)
        print('Start routine complete!')
        return None

    def flocking(self):
        o_count = 0

        start = time.time()
        dt = start - self.end
        if dt == 0.0:
            return

        # initialize the vectors
        v_coh = np.zeros(2)
        v_sep = np.zeros(2)
        v_mig = np.zeros(2)
        
        # sep_sum.clear()
        sep_sum = np.zeros(2)
        
        # coh_sum.clear()
        coh_sum = np.zeros(2)

        # assumption_pos.clear()
        assumption_pos = np.zeros(2)

        # relative_velocity.clear()
        relative_velocity = np.zeros(2)

        # v_frict.clear()
        v_frict = np.zeros(2)


        if self.obj_signal:
            pos_xy = np.zeros([self.obj_num-1,2]) #drone has little phi and can be ignored

            for i in range((self.obj_num-1)):
                # get the coordinates of the objects in initial frame
                pos_xy[i][0] = self.other_pos[i][0]-self.pos[0]
                pos_xy[i][1] = self.other_pos[i][1]-self.pos[1]
                
                distance = np.sqrt(pos_xy[i][0]**2+pos_xy[i][1]**2)

                # determine the effection area and set the angula
                if distance <= self.far_distance and distance >= self.near_distance:
                    o_count+=1
                    # separation speed
                    sep_sum[0] = sep_sum[0] + pos_xy[i][0] / distance
                    sep_sum[1] = sep_sum[1] + pos_xy[i][1] / distance

                    # aggregation speed
                    coh_sum[0] = coh_sum[0] + pos_xy[i][0]
                    coh_sum[1] = coh_sum[1] + pos_xy[i][1]

                    # assume the other drones as one drone in order to calculate the velocity in total
                    assumption_pos[0] = assumption_pos[0] + pos_xy[i][0]
                    assumption_pos[1] = assumption_pos[1] + pos_xy[i][1]

            if o_count > 0:
                # calculate the assumption drone relative velocity and friction
                relative_velocity[0] = (assumption_pos[0] - self.last_pos_x)/dt
                relative_velocity[1] = (assumption_pos[1] - self.last_pos_y)/dt

                v_frict[0] =  self.k_frict * relative_velocity[0]/ o_count
                v_frict[1] =  self.k_frict * relative_velocity[1]/ o_count

                # record the last pos and time
                self.last_pos_x = assumption_pos[0]
                self.last_pos_y = assumption_pos[1]

                self.end = time.time()

                v_sep[0] = - self.k_sep / o_count * sep_sum[0]
                v_sep[1] = - self.k_sep / o_count * sep_sum[1]

                # velocity of cohision
                v_coh[0] = self.k_coh / o_count * coh_sum[0]
                v_coh[1] = self.k_coh / o_count * coh_sum[1]
            else:
                v_frict[0] = 0
                v_frict[1] = 0

                v_sep[0] = 0
                v_sep[1] = 0

                # velocity of cohision
                v_coh[0] = 0
                v_coh[1] = 0

            # velocity of migration for decentralized model
            v_mig[0] = v_coh[0] + v_sep[0] + v_frict[0]
            v_mig[1] = v_coh[1] + v_sep[1] + v_frict[1]

            # print(v_coh, v_sep, v_frict)
            # print(v_mig)
            # set the maximum speed for flocking
            if v_mig[0]>0.7071*self.speed_max:
                v_mig[0] = 0.7071*self.speed_max
            elif v_mig[0]<-0.7071*self.speed_max:
                v_mig[0] = -self.speed_max
                # print(v_mig[0])
            
            if v_mig[1]>0.7071*self.speed_max:
                v_mig[1] = 0.7071*self.speed_max
            elif v_mig[1]<-0.7071*self.speed_max:
                v_mig[1] = -self.speed_max

            vt = rotate_z(v_mig[0],v_mig[1],self.yaw_sf)#velocity in body coordinate
            
            print("drone velocity: ",vt,v_mig)
            # self.client.start_linear_motion(vt[0], vt[1],0.0)
            rospy.sleep(run_time)
        else: 
            print("no neighbour found")


    def RandomVelocityGenerator1(self):
        random_seed = np.random.randint(1000)
        np.random.seed(random_seed)

        angle = (59.0 / 1.0) + (np.random.random() * 9.0)
        angle = angle + self.random_bearing

        if angle > 360:
            angle = angle - 360

        #  get a random bearing for the agent and bearing belongs to [0,360]
        # self.random_bearing = np.random.uniform(0,360) #(59.0 / 1.0) + (np.random.random() * 9.0) 
        # convert into radian
        self.random_bearing = angle
        self.random_bearing_r = np.radians(self.random_bearing)
        # generate velocity cmd
        self.velocity_x = self.speed * np.cos(self.random_bearing_r)
        self.velocity_y = self.speed * np.sin(self.random_bearing_r)
        
        self.velocity[0] = self.velocity_x
        self.velocity[1] = self.velocity_y
        self.velocity[2] = self.velocity_z
        self.velocity[3] = self.speed


    def bacterium_controller_laser(self):
        self.counter += 1

        print("counter",self.counter)
        print("concentration",self.current_concentration)

        self.dC_dt = self.current_concentration - self.previous_concentration

        if self.dC_dt < 0.1: #to avoid small swing
            self.dC_dt = 0 

        # print(self.current_concentration)
        self.dP_dt = self.kd/((self.kd + self.current_concentration)*(self.kd + self.current_concentration)) * (self.dC_dt)
        
        if self.queque_num < self.memory_capacity:
            self.p_rate_record[self.queque_num] = self.dP_dt
        else:
            for i in range(self.memory_capacity):
                if i == (self.memory_capacity-1):
                    self.p_rate_record[i] = self.dP_dt
                else:
                    self.p_rate_record[i] = self.p_rate_record[i+1]
        
        for i in range(self.memory_capacity):
            self.dP_dt_weight = self.dP_dt_weight + self.p_rate_record[i] * np.exp((i-3)/self.Tm)

        # print(self.current_concentration)
        if self.current_concentration == 0:
            self.speed = self.speed_max
        else:
            self.speed = self.speed_max/(self.current_concentration*40)
        
        if self.speed > self.speed_max:
            self.speed = self.speed_max
        
        # print(self.speed)
        
        self.dP_dt_weight = self.dP_dt_weight/self.Tm
        # print(self.dP_dt_weight)
        if self.current_concentration == 0:
            self.T_vary += 0.1
            if self.T_vary > 300:
                self.T_vary = 300
            self.T0 = 60
        elif self.current_concentration>0:
            self.T_vary = 0
            self.T0 = 180

        self.T = (self.T0+self.T_vary) * np.exp(self.alpha * self.dP_dt_weight)
        print("T",self.T)
        if self.T < 20:
            self.T = 20
        # print(self.alpha * self.dP_dt_weight)
        
        self.dP_dt_weight = 0

        self.previous_concentration = self.current_concentration
 
        if self.counter > self.T:
            self.RandomVelocityGenerator1()       
            self.counter = 0
        else:
            self.velocity[0] = self.speed * np.cos(np.radians(self.random_bearing))
            self.velocity[1] = self.speed * np.sin(np.radians(self.random_bearing))
            self.velocity[3] = self.speed

        self.queque_num += 1

        # print(self.velocity)
        vt_b = rotate_z(self.velocity[0],self.velocity[1],self.yaw_sf)#velocity in body coordinate
        
        return vt_b

       
    def run(self):
        self.current_action.action = 1
        self.action_publisher.publish(self.current_action)
        self.client.forward(forward_vel)
        print('Running!')
        rospy.sleep(run_time)
        return None

    def tumble(self):
        self.current_action.action = 2
        self.action_publisher.publish(self.current_action)
        self.client.wait()
        c = random.random()
        print('Tumbling! c = ', c)
        if c <= 0.5:
            pass
            # turns CF in the range 0-180 deg
            self.client.turn_left(180*random.random())
            self.client.wait()
        else:
            pass
            self.client.turn_right(180*random.random())
            self.client.wait()
        rospy.sleep(tumble_time)
        self.client.forward(forward_vel)
        return None

    def avoid_obst(self):
        self.current_action.action = 3
        self.action_publisher.publish(self.current_action)
        dist_arr = np.array([self.rangeLeft, self.rangeFront,
                             self.rangeRight, self.rangeBack])
        smallest_dist = np.argmin(dist_arr)

        if smallest_dist == 0:
            print('Obstacle to left, moving right')
            self.client.wait()
            self.client.right(strafe_vel)
            rospy.sleep(strafe_time)
            self.client.wait()
            self.client.turn_right(turn_deg)
            self.client.wait()
            self.client.forward(forward_vel)
        elif smallest_dist == 1:
            print('Obstacle to front, moving back')
            self.client.wait()
            self.client.back(strafe_vel)
            rospy.sleep(strafe_time)
            self.client.wait()
            self.client.turn_left(turn_deg)
            self.client.wait()
            self.client.forward(forward_vel)
        elif smallest_dist == 2:
            print('Obstacle to right, moving left')
            self.client.wait()
            self.client.left(strafe_vel)
            rospy.sleep(strafe_time)
            self.client.wait()
            self.client.turn_left(turn_deg)
            self.client.wait()
            self.client.forward(forward_vel)
        elif smallest_dist == 3:
            print('Obstacle to back, moving forward')
            self.client.wait()
            self.client.forward(forward_vel)
        rospy.sleep(ao_time)
        return None

    def stop(self):
        self.client.stop()
        self.client.wait()
        self.client.land()
        self.client.wait()
        del self.client
        sys.exit(0)
        return None


if __name__ == '__main__':
    bcrazy = bcfController()

    # Takeoff and hold
    bcrazy.start()

    # Start high level control loop
    while 1:
        # print("range",bcrazy.rangeLeft,bcrazy.rangeRight,bcrazy.rangeFront,bcrazy.rangeBack)
        if (bcrazy.rangeLeft > obst_dist_thresh and
        bcrazy.rangeRight > obst_dist_thresh and
        bcrazy.rangeFront > obst_dist_thresh and
        bcrazy.rangeBack > obst_dist_thresh):
            print("running now")
            # bcrazy.run()
            bcrazy.flocking()
            # print(bcrazy.intensity, bcrazy.last_intensity)
            # if bcrazy.intensity > bcrazy.last_intensity:
            #     bcrazy.run()
            # else:
            #     bcrazy.tumble()
            # bcrazy.run()
        else:
            bcrazy.avoid_obst()
        # print("flocking now")
        # bcrazy.flocking()
    
    # end while loop
    bcrazy.stop()
