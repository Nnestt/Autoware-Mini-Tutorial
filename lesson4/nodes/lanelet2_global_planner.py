#!/usr/bin/env python3

import numpy as np
import rospy
import shapely
from threading import Lock

from geometry_msgs.msg import PoseStamped
from autoware_mini.msg import Path, Waypoint

import lanelet2
from lanelet2.io import Origin, load
from lanelet2.projection import UtmProjector
from lanelet2.core import BasicPoint2d
from lanelet2.geometry import findNearest


class GlobalPlanner:
    def __init__(self):

        # Parameters
        lanelet2_map_path = rospy.get_param("~lanelet2_map_path")
        self.speed_limit = float(rospy.get_param("~speed_limit"))

        coordinate_transformer = rospy.get_param("/localization/coordinate_transformer")
        use_custom_origin = rospy.get_param("/localization/use_custom_origin")
        utm_origin_lat = rospy.get_param("/localization/utm_origin_lat")
        utm_origin_lon = rospy.get_param("/localization/utm_origin_lon")

        self.output_frame = rospy.get_param("lanelet2_global_planner/output_frame")
        self.distance_to_goal_limit = rospy.get_param("lanelet2_global_planner/distance_to_goal_limit")

        # Load Lanelet2 map
        if coordinate_transformer == "utm":
            projector = UtmProjector(Origin(utm_origin_lat, utm_origin_lon), use_custom_origin, False)
        else:
            raise RuntimeError('Only "utm" is supported for lanelet2 map loading')
        self.lanelet2_map = load(lanelet2_map_path, projector)

        # Create routing graph
        traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                lanelet2.traffic_rules.Participants.VehicleTaxi)
        self.graph = lanelet2.routing.RoutingGraph(self.lanelet2_map, traffic_rules)

        # Internal variables
        self.lock = Lock()
        self.current_location = None
        self.goal_point = None

        # Publishers
        self.global_path_pub = rospy.Publisher('global_path', Path, latch=True, queue_size=1, tcp_nodelay=True)

        # Subscribers
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback, queue_size=1)
        rospy.Subscriber('/localization/current_pose', PoseStamped, self.current_pose_callback, queue_size=1)

    def goal_callback(self, msg):
        with self.lock:
            self.goal_point = BasicPoint2d(msg.pose.position.x, msg.pose.position.y)

        if self.current_location is None:
            return

        rospy.loginfo("%s - Goal position (%f, %f, %f) in %s frame", rospy.get_name(),
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
            msg.header.frame_id)

        # Get the start lanelet; find the goal lanelet the same way using self.goal_point
        start_lanelet = findNearest(self.lanelet2_map.laneletLayer, self.current_location, 1)[0][1]
        goal_lanelet = findNearest(self.lanelet2_map.laneletLayer, self.goal_point, 1)[0][1]

        # Find route (the third argument is the routing cost id, the last argument disables lane changes)
        route = self.graph.getRoute(start_lanelet, goal_lanelet, 0, False)
        if route is None:
            rospy.logwarn("%s - No route found to goal position", rospy.get_name())
            return

        # Find shortest path; check it for None with a logwarn the same way as above
        path = route.shortestPath()
        if path is None:
            rospy.logwarn("%s - No shortest path found", rospy.get_name())
            return

        # Get path without lane changes
        path_no_lane_change = path.getRemainingLane(start_lanelet)

        # Convert the lanelet sequence to a list of waypoints and publish it
        waypoints = self.convert_laneletseq_to_waypoints_list(path_no_lane_change)
        self.publish_lane_from_waypoints_list(waypoints)


    def current_pose_callback(self, msg):
        with self.lock:
            self.current_location = BasicPoint2d(msg.pose.position.x, msg.pose.position.y)

        if self.goal_point is None:
            return

        # Check if the vehicle is close enough to the goal point
        distance_to_goal = np.sqrt((self.current_location.x - self.goal_point.x) ** 2 +
                                    (self.current_location.y - self.goal_point.y) ** 2)
        if distance_to_goal <= self.distance_to_goal_limit:
            goal_x = self.goal_point.x
            goal_y = self.goal_point.y
            self.publish_lane_from_waypoints_list([])
            with self.lock:
                self.goal_point = None
            rospy.loginfo("%s - Goal reached at (%f, %f)", rospy.get_name(), goal_x, goal_y)


    def convert_laneletseq_to_waypoints_list(self, laneletseq):
        waypoints = []
        last_lanelet_waypoint_start = 0

        for j, lanelet in enumerate(laneletseq):
            if j == len(laneletseq) - 1:
                last_lanelet_waypoint_start = len(waypoints)

            # Get speed from lanelet attribute or use global speed limit. The speed limit is in km/h, convert to m/s for the Waypoint message.
            if 'speed_ref' in lanelet.attributes:
                speed = min(float(lanelet.attributes['speed_ref']) / 3.6, self.speed_limit / 3.6)
            else:
                speed = self.speed_limit / 3.6

            # Iterate through the centerline points and create waypoints.
            for i, point in enumerate(lanelet.centerline):
                # Skip first point of every lanelet except the very first (endpoints overlap)
                if i == 0 and j != 0:
                    continue
                waypoint = Waypoint()
                waypoint.position.x = point.x
                waypoint.position.y = point.y
                waypoint.position.z = point.z
                waypoint.speed = speed
                waypoints.append(waypoint)

        if waypoints:
            last_lanelet = laneletseq[-1]
            last_lanelet_line = shapely.LineString([(point.x, point.y, point.z)
                                                    for point in last_lanelet.centerline])
            clicked_goal = shapely.Point(self.goal_point.x, self.goal_point.y)
            projected_goal = last_lanelet_line.interpolate(last_lanelet_line.project(clicked_goal))
            projected_x, projected_y, projected_z = projected_goal.coords[0]

            closest_waypoint_index = min(
                range(last_lanelet_waypoint_start, len(waypoints)),
                key=lambda i: np.sqrt((waypoints[i].position.x - projected_x) ** 2 +
                                      (waypoints[i].position.y - projected_y) ** 2))
            closest_waypoint = waypoints[closest_waypoint_index]
            closest_waypoint.position.x = projected_x
            closest_waypoint.position.y = projected_y
            closest_waypoint.position.z = projected_z
            waypoints = waypoints[:closest_waypoint_index + 1]

            with self.lock:
                self.goal_point = BasicPoint2d(projected_x, projected_y)

        return waypoints

    def publish_lane_from_waypoints_list(self, waypoints):
        lane = Path()
        lane.header.frame_id = self.output_frame
        lane.header.stamp = rospy.Time.now()
        lane.waypoints = waypoints
        self.global_path_pub.publish(lane)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('global_planner')
    node = GlobalPlanner()
    node.run()
