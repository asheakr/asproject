#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point

def callback(data):
	"""Callback function that processes incoming Point messages from Unity."""
    rospy.loginfo(f"Received position from Unity: x={data.x}, y={data.y}, z={data.z}")

def main():
	# Initialize the ROS node.
	rospy.init_node('unity_position_listener', anonymous=True)

	# Create a subscriber to the Unity position topic
	rospy.Subscriber("/unity_position", Point, callback)

	rospy.loginfo("Unity position listener started. Waiting for messages...")

	# Keep the node running until Shutdown
	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("Shutting down Unity position listener node.")

if __name__ == '__main__':
	main()
