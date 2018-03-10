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
        self.waypt_count = 0
        
        self.lightidx = -1          # Waypoint of last set traffic light to stop at (-1 for none)
        self.obstacleidx = -1       # Waypoint of last set obstacle detected (-1 for none)

        self.targetvel = self.kmph2mps(rospy.get_param("/waypoint_loader/velocity"))

        self.position = None

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.update_final_waypoints()
            rate.sleep()

    def update_final_waypoints(self):
        if not self.position or not self.base_waypoints_msg:
            return

        index = self.closest_waypoint_index(self.position)

        highval = 99999999
        lightconv = highval if self.lightidx == -1 else self.lightidx
        obstcconv = highval if self.obstacleidx == -1 else self.obstacleidx
        stopidx = min(lightconv, obstcconv)

        # TODO: More intelligent waypoints
        if stopidx == highval:
            rospy.logdebug("Gauss - Generated forward waypoints")
            for wpt in range(index, index+LOOKAHEAD_WPS):
                # Depending on how curvy the road is ahead, slow down. 
                # It's usually subtle, but it's dramatically noticeable
                # on the sharper turns.
                #
                # We may want to find an intelligent way to calculate
                # the skip and ct parameters- such as based off the 
                # current speed or even PID error.
                #
                # It would also need a strategy on how to be combined with 
                # other velocity-considerations, such as 
                wptMod = wpt % self.waypt_count
                straight = self.check_waypoint_straight_factor(self.waypoints_positions, index, 20, 10)
                straight = math.pow(straight, 1.5) # Exaggerate/curve it
                self.set_waypoint_velocity(self.base_waypoints_msg.waypoints, wptMod, self.targetvel * straight)
        else:
            rospy.logdebug("Gauss - Generated stop waypoints") #(Not really)
            for wpt in range(index, index+LOOKAHEAD_WPS):
                wptMod = wpt % self.waypt_count
                straight = self.check_waypoint_straight_factor(self.waypoints_positions, index, 20, 10)
                straight = math.pow(straight, 1.5) # Exagerate/curve it
                self.set_waypoint_velocity(self.base_waypoints_msg.waypoints, wptMod, self.targetvel * straight)

        waypoints_sliced = self.base_waypoints_msg.waypoints[index:index+LOOKAHEAD_WPS]
        output_msg = Lane()
        output_msg.header = self.base_waypoints_msg.header
        output_msg.waypoints = waypoints_sliced

        rospy.loginfo("Gauss - Publishing Waypoints of length: " + str(len(output_msg.waypoints)))
        self.final_waypoints_pub.publish(output_msg)
		
        straight = self.check_waypoint_straight_factor(self.waypoints_positions, index, 20, 10)
        
        
        #rospy.logerr(straight)

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
        self.waypt_count = len(self.waypoints_positions)

    def closest_waypoint_index(self, position, waypoints=None):
        if (not waypoints):
            waypoints = self.waypoints_positions
        return cdist([[position.x, position.y]], waypoints).argmin()

    def traffic_cb(self, msg):
        if self.lightidx != msg.data:
            self.lightidx = msg.data
        
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
		
    '''
    Calculate a "straightness factor" by sparsely checking some 
    uniformly spaced gradients ahead of a specified waypoint and
    combining their various values.
    '''
    def check_waypoint_straight_factor(self, waypoint_points, start, skip, ct):
        factor = 1.0
        wpt = start
        for i in range(ct):
            wptNext = (wpt + skip + 1)%self.waypt_count
            # get sample points
            ptprv = waypoint_points[((wpt - skip - 1) + self.waypt_count) % self.waypt_count]
            ptcur = waypoint_points[wpt]
            ptnxt = waypoint_points[wptNext]
            # get vectors
            veccur = [ptcur[0] - ptprv[0], ptcur[1] - ptprv[1]]
            vecnxt = [ptnxt[0] - ptcur[0], ptnxt[1] - ptcur[1]]
            #normalize and dot product
            cn = math.sqrt(veccur[0] * veccur[0] + veccur[1] * veccur[1])
            nn = math.sqrt(vecnxt[0] * vecnxt[0] + vecnxt[1] * vecnxt[1])
            veccur = [veccur[0] / cn, veccur[1]/cn]
            vecnxt = [vecnxt[0] / nn, vecnxt[1]/nn]
            dot = veccur[0] * vecnxt[0] + veccur[1] * vecnxt[1] # Dotproduct to get the cos of their angles
            # remap to [0.0, 1.0]
            # We use asin instead of acos because we want a high value if
            # the angles are the same, similar to the original dot product
            # value - we can't use the original dot product value though
            # because it's too sensitive in the section that we don't want
            # it to be (actual angles are more effective than cos values)
            subfactor = math.asin(min(max(dot, 0.0), 1.0))/math.pi * 2      # Check min and max or else risk a function domain exception
            #apply - note that tuning parameters skip and ct to get a sane
            # calculated factor value is kind of an art.
            factor *= subfactor 
            wpt = wptNext
        return factor

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

