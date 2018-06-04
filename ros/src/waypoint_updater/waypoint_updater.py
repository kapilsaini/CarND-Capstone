#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.pose = None
        self.base_wps = None
        self.wp_2d = None
        self.wp_tree = None
        self.loop()

        rospy.spin()

    def loop(self):
    	rate = rospy.Rate(50)
    	while not rospy.is_Shutdown():
    	    if self.pose and self.base_wps:
    	        self.publish_waypoints(self.get_closest_wp_ndx())
    	     rate.sleep()
    	     
    def get_closest_wp_ndx(self):
        x = self.pose.pose.poistion.x
        y = self.pose.pose.poistion.y
        closest_ndx = self.wp_tree.query([x, y], 1)[1]
        closest_coord = self.wp_2d[closest_ndx]
        prev_coord = self.wp_2d[closest_ndx -1]
        
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])
        
        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)
        if val > 0:
          closest_ndx = (closest_ndx + 1) % len(self.wp_2d)
       return closest_ndx
    	
    def publish_waypoints(self, closest_ndx):
        lane = Lane()
        lane.header = self.base_wps.header
        lane.waypoints = self.base_wps.waypoints[closest_ndx: closest_ndx+LOOKAHEAD_WPS]
        self.final_waypoints_pub.publish(lane)
    
    
    def pose_cb(self, msg):
        # TODO: Implement
        pass

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_wps = waypoints
        if not self.wp_2d:
            self.wp_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoint
            self.wy_tree = KDTree(self.wp_2d)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
