#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
import tf

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

LOOKAHEAD_WPS = 100 # Number of waypoints we will publish. You can change this number
MAX_VELOCITY = 10.0 # In miles per hour


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size = 1)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size = 1)
        rospy.Subscriber('/traffic_light', Int32, self.tf_cb, queue_size = 1)

        # TODO: Uncomment when traffic light detection node and/or obstacle detection node is implemented
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        #rospy.Subscriber('/obstacle_waypoint', PoseStamped, self.obstacle_cb)


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # Container for current pose received via subscribtion to '/current_pose'
        self.pose = None

        # Container for current base waypoints received via subscribtion to '/base_waypoints'
        self.base_waypoints = None

        self.last_waypoint_id = None

        self.wps_ahead = None

        self.tf_waypoint = None

        self.need = 0

        self.save_velocity = 0

        self.drive = 0 # driving = 0, start_brake = 1, braking = 2

        self.tf_state = -1

        self.prev_tf_state = -1

        rospy.spin()

    def publish(self):
        # Define container for waypoints ahead
        wps_ahead = Lane()

        # Determine length of input waypoints
        waypoints_len = len(self.base_waypoints)

        # Determine first waypoint ahead of the car
        idx_wp_closest = self.get_idx_closest_waypoint()
        idx_wp_ahead = self.get_idx_ahead_waypoint(idx_wp_closest)
        self.last_waypoint_id = idx_wp_ahead

        # Determine waypoints ahead to be published
        idx_cur = idx_wp_ahead
        #rospy.loginfo(idx_cur)
        for i in range(LOOKAHEAD_WPS):
            wp = self.base_waypoints[idx_cur]
            next_wp = Waypoint()
            next_wp.pose = wp.pose
            next_wp.twist = wp.twist
            #if (self.need > 200) & (self.drive > 0):
            if (self.need > 0) & (self.drive > 0):
                next_wp.twist.twist.linear.x = self.save_velocity
            wps_ahead.waypoints.append(next_wp)
            idx_cur = (idx_cur + 1) % waypoints_len
        #if (self.need > 200) & (self.drive > 0):
        if (self.need > 0) & (self.drive > 0):
            self.drive = 0
        #self.stopping(400, idx_wp_ahead, wps_ahead)
        #self.stopping(753, idx_wp_ahead, wps_ahead)
        #rospy.loginfo(self.tf_state)
        #rospy.loginfo(self.need)
        if self.tf_state > 300:
            #rospy.loginfo(self.tf_state)
            self.stopping(self.tf_state, idx_wp_ahead, wps_ahead)
        #rospy.loginfo(self.need)
    
        # Publish waypoints ahead
        self.wps_ahead = wps_ahead
        self.final_waypoints_pub.publish(wps_ahead)

        #rospy.loginfo(self.tf_waypoint)


    def pose_cb(self, msg):
        # TODO: Implement
        """
        Updates the position of the car (Step 1) and publishes the waypoints ahead (Step 2)
        """

        self.pose = msg.pose
        '''if (self.base_waypoints and self.pose):
            self.publish()
            rospy.loginfo('WaypointUpdater: Updated pose - x: %.2f - y: %.2f', self.pose.position.x, self.pose.position.y)
            rospy.loginfo('WaypointUpdater: Published waypoints ahead, first waypoint - x: %.2f - y: %.2f', self.wps_ahead.waypoints[0].pose.pose.position.x, self.wps_ahead.waypoints[0].pose.pose.position.y)'''


    def waypoints_cb(self, waypoints):
        """
        Stores the waypoints initially received by subscription in the class variable 'base_waypoints'
        -> waypoints: waypoints subscribed in the 'Lane'-datatype
        """

        self.base_waypoints = waypoints.waypoints
        #rospy.loginfo('WaypointUpdater: Updated with current waypoints')

    def tf_cb(self, waypoint):
        self.tf_waypoint = waypoint


    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        #rospy.loginfo(msg.data)
        self.prev_tf_state = self.tf_state
        self.tf_state = msg.data - 20 #TODO: Fix this in tl_detector
       
        #Check if the signal changed from red to non-red(green, yellow, no traffic light).
        #if ((self.prev_tf_state > -1) and (self.tf_state == -1)):
        if ((self.prev_tf_state > -1) and (self.tf_state == -21)): #TODO: Fix this hack
            self.need = 1
        else:
            self.need = 0
        
        if (self.base_waypoints and self.pose):
            self.publish()
        #pass

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


    def get_idx_closest_waypoint(self):
        """
        Identifies closest waypoint to the current position of the car
        <- idx_wp_closest: Closest waypoint to the current position of the car
        """
        # If condition validates wheter there are base_waypoints and the position of the car available
        if (self.base_waypoints and self.pose):
            min_wp_dist = 1000000
            idx_wp_closest = None

            if self.last_waypoint_id == None:
                for i in range(len(self.base_waypoints)):
                    dist = self.get_eucl_distance(self.base_waypoints[i].pose.pose.position.x, self.base_waypoints[i].pose.pose.position.y,self.pose.position.x,self.pose.position.y)
                    if dist < min_wp_dist:
                        min_wp_dist = dist
                        idx_wp_closest = i

            else:
                for i in range(self.last_waypoint_id, self.last_waypoint_id+50):
                    j = i-3
                    if j < 0:
                        j = j + len(self.base_waypoints)
                    dist = self.get_eucl_distance(self.base_waypoints[j].pose.pose.position.x, self.base_waypoints[j].pose.pose.position.y,self.pose.position.x,self.pose.position.y)
                    if dist < min_wp_dist:
                        min_wp_dist = dist
                        idx_wp_closest = j

            return idx_wp_closest

        else:
            return None


    def get_idx_ahead_waypoint(self, idx_wp_closest):
        """
        Checks whether closest waypoint is ahead of the car and based on the returns the first waypoint ahead
        -> idx_wp_closest: Position of the closest waypoint of the car in 'self.base_waypoints'
        <- return: Position of the closest waypoint ahead of the car in 'self.base_waypoints'

        """
        wp_x_local,_,_ = self.transform_wp_to_local(self.base_waypoints[idx_wp_closest])
        if wp_x_local > 0.0:
            return idx_wp_closest
        else:
            return idx_wp_closest + 1
        
    def transform_wp_to_local(self, wp):
        """
        -> wp: Single waypoint to be transformed into local car coordinate system
        <- waypoint transformed into local car coordinate system
        """
        wx = wp.pose.pose.position.x
        wy = wp.pose.pose.position.y
        # Get yaw
        _,_,yaw = tf.transformations.euler_from_quaternion([self.pose.orientation.x,self.pose.orientation.y,self.pose.orientation.z,self.pose.orientation.w])

        dx = wx - self.pose.position.x
        dy = wy - self.pose.position.y
        wx_local = math.cos(-yaw) * dx - math.sin(-yaw) * dy
        wy_local = math.sin(-yaw) * dx - math.cos(-yaw) * dy
        return wx_local, wy_local, math.atan2(wy_local, wx_local)


    def get_eucl_distance(self,x1,y1,x2,y2):
        """
        Calculates euclidian distance between two points based on their x and y coordinates
        """
        return math.sqrt((x2-x1)**2 + (y2-y1)**2)

    def stopping(self, idx, idx_wp_ahead, wps_ahead):
        if (idx - LOOKAHEAD_WPS < idx_wp_ahead < idx):
            if (self.drive == 0):
                self.save_velocity = self.get_waypoint_velocity(self.base_waypoints[idx])
                self.drive = 1
            wp_until_tl = idx - idx_wp_ahead - 6
            if wp_until_tl < 0:
                wp_until_tl = 0
            if wp_until_tl <= LOOKAHEAD_WPS:
                parting_wp = wps_ahead.waypoints[wp_until_tl]
            for i in range(wp_until_tl, LOOKAHEAD_WPS):
                self.set_waypoint_velocity(wps_ahead.waypoints, i, 0.)
            for i in range(wp_until_tl-1, -1, -1):
                wp = wps_ahead.waypoints[i]
                d = self.distance(wps_ahead.waypoints, i, i + 1)
                prev_vel = self.get_waypoint_velocity(parting_wp)
                vel = math.sqrt(0.2*d) + prev_vel
                if vel <= 0.25:
                    vel = 0
                old_vel = self.get_waypoint_velocity(wp)
                if vel > old_vel:
                    break
                self.set_waypoint_velocity(wps_ahead.waypoints, i, vel)
                parting_wp = wp
        #if (idx_wp_ahead >= idx):
            #self.need = self.need+1


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
