#!/usr/bin/env python
import time
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist


class astra_rover_ackerman(object):
    def __init__(self):
        rospy.loginfo("astra_rover_ackerman initialising...")

        # TODO: Ackerman stuff
        self.distance_axis = 0.3
        self.distance_front_center = 0.5
        self.distance_back_center = 0.5

        self.publishers_astra_rover_d = {}
        self.controller_ns = "astra_rover"
        self.controller_command = "command"
        self.controllers_list = [   "wheel_left_1_joint_velocity_controller",
                                    "wheel_left_2_joint_velocity_controller",
                                    "wheel_left_3_joint_velocity_controller",
                                    "wheel_right_1_joint_velocity_controller",
                                    "wheel_right_2_joint_velocity_controller",
                                    "wheel_right_3_joint_velocity_controller",
                                    "rocker_hinge_left_position_controller",
                                    "bogey_hinge_left_position_controller",
                                    "rocker_hinge_right_position_controller",
                                    "bogey_hinge_right_position_controller",
                                   
                                    "bogey_steering_left_joint_position_controller",
                                    "rocker_steering_left_joint_position_controller",
                                    "bogey_steering_right_joint_position_controller",
                                    "rocker_steering_right_joint_position_controller"
                                ]

        for controller_name in self.controllers_list:
            topic_name = "/"+self.controller_ns+"/"+controller_name+"/"+self.controller_command
            self.publishers_astra_rover_d[controller_name] = rospy.Publisher(
                topic_name,
                Float64,
                queue_size=1)

        self.wait_publishers_to_be_ready()
        self.init_publisher_variables()
        self.init_state()

        self.cmd_vel_msg = Twist()
        cmd_vel_topic = "/cmd_vel"
        rospy.Subscriber(cmd_vel_topic, Twist, self.cmd_vel_callback)

        rospy.logwarn("astra_rover_ackerman...READY")

    def cmd_vel_callback(self, msg):
        self.cmd_vel_msg = msg

    def wait_publishers_to_be_ready(self):

        rate_wait = rospy.Rate(10)
        for controller_name, publisher_obj in self.publishers_astra_rover_d.iteritems():
            publisher_ready = False
            while not publisher_ready:
                rospy.logwarn("Checking Publisher for ==>"+str(controller_name))
                pub_num = publisher_obj.get_num_connections()
                publisher_ready = (pub_num > 0)
                rate_wait.sleep()
            rospy.loginfo("Publisher ==>" + str(controller_name) + "...READY")

    def init_publisher_variables(self):
        """
        We create variables for more pythonic access access to publishers
        and not need to access any more
        :return:
        """
        # Get the publishers for wheel speed
        self.wheel_left_1 = self.publishers_astra_rover_d[self.controllers_list[0]]
        self.wheel_left_2 = self.publishers_astra_rover_d[self.controllers_list[1]]
        self.wheel_left_3 = self.publishers_astra_rover_d[self.controllers_list[2]]
        self.wheel_right_1 = self.publishers_astra_rover_d[self.controllers_list[3]]
        self.wheel_right_2 = self.publishers_astra_rover_d[self.controllers_list[4]]
        self.wheel_right_3 = self.publishers_astra_rover_d[self.controllers_list[5]]
        # Get the publishers for suspension
        self.rocker_hinge_left= self.publishers_astra_rover_d[self.controllers_list[6]]
        self.bogey_hinge_left= self.publishers_astra_rover_d[self.controllers_list[7]]
        self.rocker_hinge_right= self.publishers_astra_rover_d[self.controllers_list[8]]
        self.bogey_hinge_right= self.publishers_astra_rover_d[self.controllers_list[9]]
        # Get the publishers for steering
        self.bogey_steering_left = self.publishers_astra_rover_d[self.controllers_list[10]]
        self.rocker_steering_left = self.publishers_astra_rover_d[self.controllers_list[11]]
        self.bogey_steering_right = self.publishers_astra_rover_d[self.controllers_list[12]]
        self.rocker_steering_right = self.publishers_astra_rover_d[self.controllers_list[13]]

        # Init Messages
        self.wheel_left_1_velocity_msg = Float64()
        self.wheel_left_2_velocity_msg = Float64()
        self.wheel_left_3_velocity_msg = Float64()
        self.wheel_right_1_velocity_msg = Float64()
        self.wheel_right_2_velocity_msg = Float64()
        self.wheel_right_3_velocity_msg = Float64()

        self.rocker_hinge_left_pos_msg = Float64()
        self.bogey_hinge_left_pos_msg = Float64()
        self.rocker_hinge_right_pos_msg = Float64()
        self.bogey_hinge_right_pos_msg = Float64()

        self.bogey_steering_left_pos_msg = Float64()
        self.rocker_steering_left_pos_msg = Float64()
        self.bogey_steering_right_pos_msg = Float64()
        self.rocker_steering_right_pos_msg = Float64()



    def init_state(self):
        self.set_suspension_mode("standard")
        self.set_turning_radius(None)
        self.set_wheels_speed(0.0)

    def set_suspension_mode(self, mode_name):

        if mode_name == "standard":

            self.rocker_hinge_left_pos_msg.data = 0.0
            self.bogey_hinge_left_pos_msg.data = 0.0
            self.rocker_hinge_right_pos_msg.data = 0.0
            self.bogey_hinge_right_pos_msg.data = 0.0

            self.rocker_hinge_left.publish(self.rocker_hinge_left_pos_msg)
            self.bogey_hinge_left.publish(self.bogey_hinge_left_pos_msg)
            self.rocker_hinge_right.publish(self.rocker_hinge_right_pos_msg)
            self.bogey_hinge_right.publish(self.bogey_hinge_right_pos_msg)

    def set_turning_radius(self, turn_radius):

        if not turn_radius:
            # We dont need Ackerman calculations, its not turn.
            self.bogey_steering_left_pos_msg.data = 0.0
            self.rocker_steering_left_pos_msg.data = 0.0
            self.bogey_steering_right_pos_msg.data = 0.0
            self.rocker_steering_right_pos_msg.data = 0.0
        else:
            # TODO: Ackerman needed here
            if turn_radius > 0.0:
                self.bogey_steering_left_pos_msg.data = -0.3
                self.rocker_steering_left_pos_msg.data = -0.3
                self.bogey_steering_right_pos_msg.data = 0.3
                self.rocker_steering_right_pos_msg.data = 0.3
            else:
                self.bogey_steering_left_pos_msg.data = 0.3
                self.rocker_steering_left_pos_msg.data = 0.3
                self.bogey_steering_right_pos_msg.data = -0.3
                self.rocker_steering_right_pos_msg.data = -0.3

        self.bogey_steering_left.publish(self.bogey_steering_left_pos_msg)
        self.rocker_steering_left.publish(self.rocker_steering_left_pos_msg)
        self.bogey_steering_right.publish(self.bogey_steering_right_pos_msg)
        self.rocker_steering_right.publish(self.rocker_steering_right_pos_msg)

    def set_wheels_speed(self, turning_speed):
        """
        Sets the turning speed in radians per second
        :param turning_speed: In radians per second
        :return:
        """
        # TODO: turning_speed for each wheel should change based on ackerman.

        self.wheel_left_1_velocity_msg.data = turning_speed
        self.wheel_left_2_velocity_msg.data = -1*turning_speed
        self.wheel_left_3_velocity_msg.data = turning_speed
        self.wheel_right_1_velocity_msg.data = -1*turning_speed
        self.wheel_right_2_velocity_msg.data = turning_speed
        self.wheel_right_3_velocity_msg.data = -1*turning_speed

        self.wheel_left_1.publish(self.wheel_left_1_velocity_msg)
        self.wheel_left_2.publish(self.wheel_left_2_velocity_msg)
        self.wheel_left_3.publish(self.wheel_left_3_velocity_msg)
        self.wheel_right_1.publish(self.wheel_right_1_velocity_msg)
        self.wheel_right_2.publish(self.wheel_right_2_velocity_msg)
        self.wheel_right_3.publish(self.wheel_right_3_velocity_msg)

    def move_forwards(self):
        self.set_wheels_speed(10.0)
        self.set_turning_radius(None)

    def move_backwards(self):
        self.set_wheels_speed(-10.0)
        self.set_turning_radius(None)

    def move_turn_left(self):
        self.set_wheels_speed(10.0)
        self.set_turning_radius(1.0)

    def move_turn_right(self):
        self.set_wheels_speed(10.0)
        self.set_turning_radius(-1.0)

    def move_turn_stop(self):
        self.set_wheels_speed(0.0)
        self.set_turning_radius(None)


    def move_with_cmd_vel(self):
        wheel_speed = self.cmd_vel_msg.linear.x
        turning_radius = self.cmd_vel_msg.angular.z
        if turning_radius == 0.0:
            turning_radius = None

        rospy.logdebug("turning_radius="+str(turning_radius)+",wheel_speed="+str(wheel_speed))
        self.set_turning_radius(turning_radius)
        self.set_wheels_speed(wheel_speed)



if __name__ == "__main__":
    rospy.init_node("astra_rover_ackerman_node", log_level=rospy.INFO)
    astra_rover_ackerman_object = astra_rover_ackerman()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        astra_rover_ackerman_object.move_with_cmd_vel()
        rate.sleep()



