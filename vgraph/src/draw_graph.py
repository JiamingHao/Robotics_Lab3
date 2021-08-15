#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import utils
import create_map

rospy.init_node('line')
pub_line_min_dist = rospy.Publisher('/vgraph_markers', Marker, queue_size=1)
rospy.loginfo('Publishing line')

edges, shortest_path = utils.get_waypoints()
map_edges = []
for img_edge in edges:
	map_edge = create_map.img2map(img_edge)
	map_edges.append(map_edge.tolist())
edges = map_edges
shortest_path = create_map.img2map(shortest_path).tolist()

resolution = 100.
id_count = 0

while not rospy.is_shutdown():
	# ================== Initialize marker ==================
	marker = Marker()
	marker.header.frame_id = "/map"
	marker.type = marker.LINE_STRIP
	marker.action = marker.ADD

	marker.pose.orientation.x = 0.0
	marker.pose.orientation.y = 0.0
	marker.pose.orientation.z = 0.0
	marker.pose.orientation.w = 1.0

	marker.pose.position.x = 0.0
	marker.pose.position.y = 0.0
	marker.pose.position.z = 0.0

	# ================== Draw graph ==================
	marker.scale.x = 0.02
	marker.scale.y = 0.02
	marker.scale.z = 0.02

	marker.color.a = 1.0
	marker.color.r = 1.0
	marker.color.g = 0.0
	marker.color.b = 0.0

	for edge in edges:
		id_count += 1
		marker.id = id_count
		marker.points = []

		start = Point()
		start.x = float(edge[0][0])/resolution
		start.y = float(edge[0][1])/resolution
		marker.points.append(start)

		end = Point()
		end.x = float(edge[1][0])/resolution
		end.y = float(edge[1][1])/resolution
		marker.points.append(end)

		# Duplicate points in the marker.points list will cause some anomalies,
		# must publish the marker and empty the list at each iteration
		pub_line_min_dist.publish(marker)
		rospy.sleep(0.01)

	# ================== Draw shortest path ==================
	marker.scale.x = 0.05
	marker.scale.y = 0.05
	marker.scale.z = 0.05

	marker.color.a = 1.0
	marker.color.r = 1.0
	marker.color.g = 1.0
	marker.color.b = 0.0

	id_count += 1
	marker.id = id_count
	marker.points = []

	for point in shortest_path:
		pt = Point()
		pt.x = (float)(point[0])/resolution
		pt.y = (float)(point[1])/resolution
		marker.points.append(pt)

	pub_line_min_dist.publish(marker)
	rospy.sleep(0.01)

	break
