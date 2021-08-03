#! /usr/bin/env python
import rospy
import rospy_crazyflie.client as crazyflie_client
from rospy_crazyflie.client import Client
from rospy_crazyflie.srv import *
from rospy_crazyflie.msg import *
import sys


class SensorPublisher:
    def __init__(self):
        # Init node
        rospy.init_node('cf_sensor_publisher')

        # Log variables
        self.multiRangeLogvars = [LogVariable('range.left', 'float'),
                              LogVariable('range.front', 'float'),
                              LogVariable('range.right', 'float'),
                              LogVariable('range.back', 'float')]
        self.RangeLogvars = [LogVariable('range.zrange', 'float')]
        self.stateLogvars = [LogVariable('stateEstimate.x', 'float'),
                             LogVariable('stateEstimate.y', 'float'),
                             LogVariable('stateEstimate.z', 'float')]
        #self.light_rangeLogvars = [LogVariable('BH1750.intensity', 'float'),
        #                           LogVariable('range.left', 'float'),
        #                           LogVariable('range.front', 'float'),
        #                           LogVariable('range.right', 'float'),
        #                           LogVariable('range.back', 'float'),
        #                           LogVariable('range.up', 'float')]
        # self.tempLogvar = [LogVariable('HDC2010.temp', 'float')]

        # Publishers
        self.state_publisher = rospy.Publisher('/crazyflie/bcf_state', KalmanPositionEst, queue_size=10)
        #self.intensity_publisher = rospy.Publisher('bcf_intensity', LightData, queue_size=10)
        # self.range_publisher = rospy.Publisher('bcf_range', RangeData, queue_size=10)
        # self.temp_publisher = rospy.Publisher('bcf_temp', TempData, queue_size=10)
        self.multiRange_publisher = rospy.Publisher('/crazyflie/multi_range', RangeData, queue_size=10)
        self.zRange_publisher = rospy.Publisher('/crazyflie/z_range', zRangeData, queue_size=10)

        # Get all crazyflies on the /crazyflie_server
        self.crazyflies = crazyflie_client.get_crazyflies(server='/crazyflie_server')

        # Connect to the first crazyflie
        self.client = Client(self.crazyflies[0])

        # Initialize variables to be published with message type
        self.state = KalmanPositionEst()
        #self.intensity = LightData()
        self.zrange = zRangeData()
        self.mrange = RangeData()

        # Additional stuff
        # Configures the crazyflie to log the data
        self.client.add_log_config('stateEstimateConfig',
                                   self.stateLogvars,
                                   10, callback=self.stateUpdate)
        #self.client.add_log_config('light_rangeConfig',
        #                           self.light_rangeLogvars,
        #                           500, callback=self.light_rangeUpdate)
        self.client.add_log_config('zrangeConfig', self.RangeLogvars, 200, callback=self.zrangeUpdate)
        self.client.add_log_config('multirangeConfig', self.multiRangeLogvars, 200, callback=self.mrangeUpdate)

        # Keep the node running until it is killed
        rospy.spin()

    def stateUpdate(self, data, timestamp):
        # Called when crazyflie state is updated
        self.state.stateX = data['stateEstimate.x']
        self.state.stateY = data['stateEstimate.y']
        self.state.stateZ = data['stateEstimate.z']
        self.state.cfstamp = timestamp
        # print("x={},y={},z={}".format(self.state.stateX,
        #                               self.state.stateY, self.state.stateZ))
        self.state_publisher.publish(self.state)

    def zrangeUpdate(self, data, timestamp):
        self.zrange.down = data['range.zrange']
        self.zrange.cfstamp = timestamp

        self.zRange_publisher.publish(self.zrange)
        print("z={}".format(self.zrange.down))

    def mrangeUpdate(self, data, timestamp):
        self.mrange.right = data['range.right']
        self.mrange.left = data['range.left']
        self.mrange.front = data['range.front']
        self.mrange.back = data['range.back']
        self.mrange.cfstamp = timestamp

        print("right={},left={},front={},back={}".format(self.mrange.right,
                                     self.mrange.left, self.mrange.front,self.mrange.back))
        
        self.multiRange_publisher.publish(self.mrange)
    # def tempUpdate(self, data, timestamp):
    #     self.temp.temp = data['HDC2010.temp']
    #     self.temp_publisher.publish(self.temp)

    # def light_rangeUpdate(self, data, timestamp):
    #     # Called when light intensity and multiranger data are updated
    #     self.intensity.intensity = data['BH1750.intensity']
    #     self.dists.left = data['range.left']
    #     self.dists.front = data['range.front']
    #     self.dists.right = data['range.right']
    #     self.dists.back = data['range.back']
    #     # print("int={},l={},f={},r={},b={}".format(self.intensity,
    #     #                                           self.rangeLeft,
    #     #                                           self.rangeFront,
    #     #                                           self.rangeRight,
    #     #                                           self.rangeBack))
    #     self.intensity_publisher.publish(self.intensity)
    #     self.range_publisher.publish(self.dists)


if __name__ == '__main__':
    cf = SensorPublisher()
