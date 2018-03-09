#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial.distance import cdist
import math
from std_msgs.msg import Int32

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
        rospy.loginfo("Gauss - Started Waypoint Updater")

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # @done: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        # Removed for now, as those raise warnings
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        # @done: Add other member variables you need below
        self.base_waypoints_msg = None
        self.lightidx = -1          # Waypoint of last set traffic light to stop at (-1 for none)
        self.obstacleidx = -1       # Waypoint of last set obstacle detected (-1 for none)

        self.targetvel = self.kmph2mps(rospy.get_param("/waypoint_loader/velocity"))

        self.position = None

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.update_final_waypoints()
            rate.sleep()

    def update_final_waypoints(self):
        if (not self.position):
            return

        if self.base_waypoints_msg:
            index = self.closest_waypoint_index(self.position)

        highval = 99999999
        lightconv = highval if self.lightidx == -1 else self.lightidx
        obstcconv = highval if self.obstacleidx == -1 else self.obstacleidx
        stopidx = min(lightconv, obstcconv)

        # TODO: More intelligent waypoints
        if stopidx == highval:
            rospy.logdebug("Gauss - Generated forward waypoints")
            for wpt in range(index, index+LOOKAHEAD_WPS):
                self.set_waypoint_velocity(self.base_waypoints_msg.waypoints, wpt, self.targetvel)
        else:
            rospy.logdebug("Gauss - Generated stop waypoints") #(Not really)
            for wpt in range(index, index+LOOKAHEAD_WPS):
                self.set_waypoint_velocity(self.base_waypoints_msg.waypoints, wpt, self.targetvel)

        waypoints_sliced = self.base_waypoints_msg.waypoints[index:index+LOOKAHEAD_WPS]
        output_msg = Lane()
        output_msg.header = self.base_waypoints_msg.header
        output_msg.waypoints = waypoints_sliced

        rospy.loginfo("Gauss - Publishing Waypoints of length: " + str(len(output_msg.waypoints)))
        self.final_waypoints_pub.publish(output_msg)

    def pose_cb(self, msg):
        # @done: Implement
        self.position = msg.pose.position
        x = self.position.x
        y = self.position.y
        rospy.loginfo("Gauss - Got Pose (x, y): " + str(x) + ", " + str(y))

    def waypoints_cb(self, waypoints):
        # @done: Implement
        rospy.loginfo("Gauss - Got Waypoints")
        self.base_waypoints_msg = waypoints
        self.waypoints_positions = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]

    def closest_waypoint_index(self, position, waypoints=None):
        if (not waypoints):
            waypoints = self.waypoints_positions
        return cdist([[position.x, position.y]], waypoints).argmin()

    def traffic_cb(self, msg):
        if self.obstacleidx != msg.data:
            self.obstacleidx = msg.data 
        
    def obstacle_cb(self, msg):
        if self.obstacleidx != msg.data:
            self.obstacleidx = msg.data 

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

    def kmph2mps(self, velocity_kmph):
        return (velocity_kmph * 1000.) / (60. * 60.)

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

