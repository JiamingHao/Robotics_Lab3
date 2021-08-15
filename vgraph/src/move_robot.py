#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from rbx1_nav.transform_utils import quat_to_angle
import math
import utils
import create_map


class MoveRobot():
	def __init__(self, path):
		rospy.init_node('move_robot', anonymous=False)
		rospy.on_shutdown(self.shutdown)

		# Publisher to control the robot's speed
		self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

		self.tf_listener = tf.TransformListener() # Initialize the tf listener
		rospy.sleep(2) # Give tf some time to fill its buffer

		self.odom_frame = '/odom' # Set the odom frame

		# Find out if the robot uses /base_link or /base_footprint
		try:
			self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
			self.base_frame = '/base_footprint'
		except (tf.Exception, tf.ConnectivityException, tf.LookupException):
			print "Exception occurred"
			try:
				self.tf_listener.waitForTransform(self.odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
				self.base_frame = '/base_link'
			except (tf.Exception, tf.ConnectivityException, tf.LookupException):
				rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
				rospy.signal_shutdown("tf Exception")

		self.points_list = []
		self.offset = 100

		for point in path:
			pt = Point()
			pt.x = float(point[0])/self.offset
			pt.y = float(point[1])/self.offset
			pt.z = 0.0
			self.points_list.append(pt)

		# Set speeds
		self.rate = rospy.Rate(20)
		self.linear_speed = 0.15
		self.angular_speed = 0.2

		# stage num = the length of path, the starting point is stage 1
		self.cur_stage = 1
		(self.cur_pos, self.rotation) = self.get_odom()
		self.has_corrected_yaw = False
		self.cur_action = 'gf' # 'tl': turn left; 'tr': turn right; 'fw': go foward
		self.next_action_time = rospy.Time.now()
		self.goal_tolarence = 0.1

		while not rospy.is_shutdown():
			# if the robot has not arrived the current goal
			dist2next_vertex = self.get_distance_between(self.cur_pos, self.points_list[self.cur_stage])
			if dist2next_vertex > self.goal_tolarence:
				# if the robot has finished previous action
				if rospy.Time.now() > self.next_action_time:
					if self.has_corrected_yaw:
						self.cur_action = 'gf'
						self.next_action_time = rospy.Time.now() + rospy.Duration(dist2next_vertex / self.linear_speed)
						self.has_corrected_yaw = False
					else:
						self.get_goal_direction(self.points_list[self.cur_stage])
						self.has_corrected_yaw = True

			else:
				# this is the last point in the path
				if self.cur_stage == len(self.points_list) - 1:
					print "Reached Goal!"
					break
				else:
					# not the last point, set the next point as the goal
					self.cur_stage += 1
					self.has_corrected_yaw = False

			# Execute the action chosen above
			twist = Twist()
			if self.cur_action == 'tl':
				twist.angular.z = self.angular_speed
			elif self.cur_action == 'tr':
				twist.angular.z = -self.angular_speed
			elif self.cur_action == 'gf':
				twist.linear.x = self.linear_speed
			self.cmd_vel_pub.publish(twist)
			self.rate.sleep()

			(self.cur_pos, self.rotation) = self.get_odom()

		# Stop the robot for good
		self.cmd_vel_pub.publish(Twist())
		#========================== END of __init__ ==========================

	def get_goal_direction(self, cur_goal):
		""" Make the robot rotate towards the goal point """
		rho_robot = math.atan2(cur_goal.y - self.cur_pos.y, cur_goal.x - self.cur_pos.x)

		yaw_err = rho_robot - self.rotation
		if yaw_err < 0:
			self.cur_action = 'tr'
		else:
			self.cur_action = 'tl'
		self.next_action_time = rospy.Time.now() + rospy.Duration(abs(yaw_err) / self.angular_speed)


	def get_distance_between(self, p1, p2):
		""" Calculate the distance between the two points, p1 and p2 """
		return math.sqrt(math.pow((p1.x - p2.x), 2) + math.pow((p1.y - p2.y), 2))

	def get_odom(self):
		# Get the current transform between the odom and base frames
		try:
			(trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
		except (tf.Exception, tf.ConnectivityException, tf.LookupException):
			rospy.loginfo("TF Exception")
			return
		return (Point(*trans), quat_to_angle(Quaternion(*rot)))

	def shutdown(self):
		# Always stop the robot when shutting down the node.
		rospy.loginfo("Stopping the robot...")
		self.cmd_vel_pub.publish(Twist())
		rospy.sleep(1)


if __name__ == '__main__':
	# first get the shortest path
	edges, shortest_path = utils.get_waypoints()
	map_edges = []
	for img_edge in edges:
		map_edge = create_map.img2map(img_edge)
		map_edges.append(map_edge.tolist())
	edges = map_edges
	shortest_path = create_map.img2map(shortest_path).tolist()

	try:
		MoveRobot(shortest_path)
	except:
		rospy.loginfo("move_robot node terminated.")
