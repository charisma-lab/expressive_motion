#!/usr/bin/env python

'''
This node will generate the higher level trajectory for different behaviors such as "Happy", Grumpy", and "Sleepy".
'''

import rospy
import numpy as np
import math
import time, argparse
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseGoal
from std_msgs.msg import Header, Bool, Float64MultiArray, Float64
from tf.transformations import quaternion_from_euler
import matplotlib.pyplot as plt

def create_header(frame_id):
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id
    return header


def quat_heading(yaw):
    return Quaternion(*quaternion_from_euler(0, 0, yaw))


class BehaviorGenerator:
    def __init__(self):
        self.waypoint_publisher = rospy.Publisher('/neato01/social_global_plan', Path, queue_size=1)
        self.goal_publisher = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=1)
        self.changed_goal_publisher = rospy.Publisher('/neato01/changed_goal', Bool, queue_size=1)
        self.vel_factor_publisher = rospy.Publisher('/velocity_factor', Float64, queue_size=1)
        rospy.Subscriber('/neato05/pose', PoseStamped, self.goal_callback, queue_size=1) # Use of service could be more efficient
        rospy.Subscriber('/neato01/pose', PoseStamped, self.start_callback, queue_size=1)
        rospy.Subscriber('/obstacle', PoseStamped, self.obstacle_callback, queue_size=1)
        rospy.Subscriber('/TSI_values', Float64MultiArray, self.TSI_callback, queue_size=1)
        self.start = None
        self.goal = None
        self.obstacle = None

        self.last_goal = None
        self.goal_change_threshold = 0.3
        self.happy_section_length = 1.5
        self.sine_amplitude = 0.35
        self.change_goal_flag = True

        self.start_goal_flag = True

        self.goal_to_publish = None  # TODO: redundant if i use path_to_publish for same purpose
        self.path_to_publish = None


    def goal_callback(self, goal_msg):
        # extract the pose and update
        self.goal = [goal_msg.pose.position.x, goal_msg.pose.position.y]
        self.goal_stamped = goal_msg

        if self.start_goal_flag:
            self.last_goal = self.goal

        #print('self.goal : ', self.goal)

    def start_callback(self, start_msg):
        self.start = [start_msg.pose.position.x, start_msg.pose.position.y]
        #print('self.start : ', self.start)

    def obstacle_callback(self, obstacle_msg):
        self.obstacle = obstacle_msg

    def populate_path_msg(self, rotated_path, heading):
        poses_waypoints = []
        for i, point in enumerate(rotated_path):
            # print('rotated_path : ', rotated_path)
            # print('point : ', point)
            header = create_header('map')
            waypoint = Pose(Point(float(point[0]), float(point[1]), 0), quat_heading(float(heading[i])))
            poses_waypoints.append(PoseStamped(header, waypoint))
        return poses_waypoints

    def get_dist_theta_to_goal(self):
        y_diff = self.goal[1] - self.start[1]
        x_diff = self.goal[0] - self.start[0]
        theta = math.atan2(y_diff, x_diff)
        dist = math.sqrt(x_diff ** 2 + y_diff ** 2)
        return x_diff, y_diff, dist, theta

    def get_change_goal_dist(self):
        y_diff = self.goal[1] - self.last_goal[1]
        x_diff = self.goal[0] - self.last_goal[0]
        dist = math.sqrt(x_diff ** 2 + y_diff ** 2)
        return dist

    def sample_poses(self, dist, theta, intermediate_steps):
        # fix equidistant points on x-axis
        sample_x = np.array(np.linspace(0, dist, num=intermediate_steps))
        sample_y = np.random.uniform(-1.0, 1.0, intermediate_steps)
        return sample_x, sample_y

    def gen_heading(self, path_rotated):
        """Take path, and generate heading (in euler) based on the slope of point[i] to point[i+1]"""
        # path_rotated has the start and end points
        heading = np.zeros([path_rotated.shape[0], 1])
        # Find heading for each point
        for i in range(len(path_rotated) - 1): # since we are calculating dx, dy wrt i, and i+1
            dx = path_rotated[i + 1, 0] - path_rotated[i, 0]
            dy = path_rotated[i + 1, 1] - path_rotated[i, 1]
            heading[i] = math.atan2(dy, dx)
        heading[-1] = heading[-2]  # TODO: Or make it zero if you want
        return heading

    def gen_traj_section(self, wavelength, amp, wave_formed, num_waypt_count):
        freq = 1.0 / wavelength
        section_path = np.linspace(wave_formed, wave_formed+wavelength, num=wavelength*num_waypt_count)
        omega = 2*np.pi*freq
        section_path = np.array([[L, amp * np.sin(omega * L)] for L in section_path])
        return section_path

    def gen_traj_section_last(self, wavelength, amp, wave_formed, num_waypt_count):
        freq = 1.0 / wavelength
        omega = 2 * np.pi * freq
        #section_path = np.linspace(wave_formed, wave_formed+wavelength, num=wavelength*50)
        section_path = np.linspace(0, wavelength, num=wavelength * num_waypt_count)
        section_path = np.array([[L+wave_formed, amp * np.sin(omega * L)] for L in section_path])
        return section_path

    def check_goal_change(self):
        goal_change_dist = self.get_change_goal_dist()
        has_changed = (abs(goal_change_dist) > self.goal_change_threshold)
        return has_changed

    def TSI_callback(self, val):
        self.time_val = val.data[0]
        self.space_val = val.data[1]
        self.interaction_val = val.data[2]
        # when space_val = -1, amplitude is 0 and when space_val is +1 then amplitude is 0.35. 
        amplitude = rospy.get_param(space_val_min) * self.space_val + rospy.get_param(space_val_max)
        # when time_val = -1, then vel_factor = 0.5 and when time_val = +1 then vel_factor = 1

        velocity_factor = rospy.get_param(velcoity_val_min)*self.time_val + rospy.get_param(velocity_val_max)
        self.vel_factor_publisher.publish(velocity_factor)
        """<-----------------------HAPPY OR SLEEPY---------------------------->
        Happy and Sleepy have the same high level path, but the lower level controller will
        impart distinguishing behavior. Grumpy has a a different high level path"""

        if self.time_val != 0 and self.space_val != 0 and self.interaction_val != 9: 
            # generate sinusoidal path
            x_diff, y_diff, dist, theta = self.get_dist_theta_to_goal()  # this could be one scope higher
            # we add noise to change interaction_val assuming at -1 interaction val, noise is 0 and at +1 it is 50
            noise = rospy.get_param(interaction_val_min) * self.interaction_val + rospy.get_param(interaction_val_max)
            theta = theta + noise
            # when interaction_val = -1 then num_waypt = 25 and for +1 it is 75. This is when space_val = 1 (undulating/sine wave)
            num_waypt_count = -rospy.get_param(points_val_min)*self.interaction_val + rospy.get_param(points_val_max)
            
            # # Check we need to generate new trajectory, based on goal_change_threshold
            # goal_change_dist = self.get_change_goal_dist()

            # if abs(goal_change_dist) > self.goal_change_threshold:
            if self.check_goal_change() or self.start_goal_flag:
                self.last_goal = self.goal

                path = []

                if dist < self.happy_section_length:
                    wavelength = dist
                    path = self.gen_traj_section(wavelength, amplitude*0.5, 0, num_waypt_count)  # decrease amplitude by half

                if dist > self.happy_section_length:
                    remaining = dist  # start with total length and then keep reducing
                    wave_formed = 0.0   # start with zero, and keep adding

                    while wave_formed != dist:
                        remaining = remaining - self.happy_section_length
                        if remaining >=self.happy_section_length:
                            path.append(self.gen_traj_section(self.happy_section_length, amplitude, wave_formed, num_waypt_count))  # generate path for 1.5 meters
                            wave_formed += self.happy_section_length
                        else:
                            path.append(self.gen_traj_section_last(self.happy_section_length+remaining, amplitude, wave_formed, num_waypt_count))
                            wave_formed += (self.happy_section_length + remaining)
                    path = np.concatenate(path, axis=0)
                # Now we have a path (in x axis)

                # Define rotation matrix to rotate the whole line
                rot = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])

                path_rotated = np.zeros([path.shape[0], path.shape[1]])
                heading = np.zeros([path.shape[0],1])

                for i, point in enumerate(path):
                    result = np.matmul(rot, point)
                    path_rotated[i] = [result[0] + self.start[0], result[1] + self.start[1]]
                    # Now, path is from start point to goal point
                path_rotated[0] = self.start # TODO: check if this is coming out correctly
                path_rotated[-1] = self.goal

                # Find heading for each point #TODO: replace this with gen_heading?
                for i in range(len(path_rotated)-1):
                    dx = path_rotated[i+1, 0] - path_rotated[i, 0]
                    dy = path_rotated[i+1, 1] - path_rotated[i, 1]
                    heading[i] = math.atan2(dy, dx)
                heading[-1] = heading[-2]

                # Now, populate the Path message #TODO: move to separate function
                path_to_publish = Path()
                path_to_publish.header = create_header('map')
                path_to_publish.poses = self.populate_path_msg(path_rotated, heading)
                self.waypoint_publisher.publish(path_to_publish)
                
                # goal_to_publish = MoveBaseActionGoal()
                # goal_to_publish.header = path_to_publish.header
                # goal_to_publish.goal.target_pose = self.goal_stamped
                
                self.path_to_publish = path_to_publish  # save it
                # self.goal_publisher.publish(self.goal_to_publish)
                print("changed goal")
                self.start_goal_flag = False

            else:
                self.waypoint_publisher.publish(self.path_to_publish)
                print("publishing old path")

        
#------------------------------------------------------------------------------------               
if __name__ == "__main__":
    rospy.init_node("waypoint_publisher")
    print("Now going to work on trajectory generation")
    generator = BehaviorGenerator() 
    r = rospy.Rate(20)
    start_time = time.time()
    r.sleep()
    rospy.spin()
