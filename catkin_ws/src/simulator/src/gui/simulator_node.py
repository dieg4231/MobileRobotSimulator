#!/usr/bin/env python

from MobileRobotSimulator import *
from simulator.srv import *
from simulator.msg import Parameters
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion 
import tf
import time
import rospy
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

gui=MobileRobotSimulator()

def turtle_odometry(msg):
	quaternion = (
    msg.pose.pose.orientation.x,
    msg.pose.pose.orientation.y,
    msg.pose.pose.orientation.z,
    msg.pose.pose.orientation.w)

	euler = euler_from_quaternion(quaternion)
	roll = euler[0]
	pitch = euler[1]
	yaw = euler[2]
	gui.handle_turtle(msg.pose.pose.position.x,msg.pose.pose.position.y,yaw)
	#msg.pose.pose.position
	
def handle_simulator_object_interaction(req):
	print(req)
	resp = simulator_object_interactionResponse()
	resp.done = gui.handle_simulator_object_interaction(req.grasp,req.name)
	print(gui.objects_data)
	return resp

def update_value(msg):
	gui.handle_hokuyo(msg.ranges)
	ranges=msg.ranges

def handle_simulator_set_light_position(req):

	resp = simulator_set_light_positionResponse()
	gui.set_light_position(req.light_x,req.light_y)
	return resp

def handle_simulator_stop(req):

	resp = simulator_stopResponse()
	gui.s_t_simulation(False)
	return resp


def handle_robot_step(req):

	resp = simulator_robot_stepResponse()
	gui.sensors_values_aux = req.sensors;
	gui.handle_service(req.theta,req.distance)
	parameters = gui.get_parameters()
	resp.robot_x = parameters[0]
	resp.robot_y = parameters[1]
	resp.theta = parameters[2]
	return resp

def handle_print_graph(req):

	resp = simulator_algorithm_resultResponse()
	gui.handle_print_graph(req.nodes_algorithm)
	resp.success=1;
	return resp

def ros():

	rospy.init_node('simulator_gui_node')
	a = rospy.Service('simulator_robot_step', simulator_robot_step, handle_robot_step)
	b = rospy.Service('simulator_print_graph', simulator_algorithm_result, handle_print_graph)
	c = rospy.Service('simulator_stop', simulator_stop, handle_simulator_stop)
	d = rospy.Service('simulator_set_light_position', simulator_set_light_position, handle_simulator_set_light_position)
	e = rospy.Service('simulator_object_interaction', simulator_object_interaction, handle_simulator_object_interaction)
	
	#rospy.Subscriber('/scan',LaserScan,update_value,queue_size=1)
	#rospy.Subscriber('/odom',Odometry, turtle_odometry ,queue_size=1)

	odom_pub = rospy.Publisher("/odom_simul", Odometry, queue_size=50)
	odom_broadcaster = tf.TransformBroadcaster()

	x = 0.0
	y = 0.0
	th = 0.0

	vx = 0.1
	vy = -0.1
	vth = 0.1
	current_time = rospy.Time.now()
	last_time = rospy.Time.now()


	pub_params = rospy.Publisher('simulator_parameters_pub', Parameters, queue_size = 0)
	#rospy.Subscriber("simulator_laser_pub", Laser_values, callback)

	msg_params = Parameters()

	rate = rospy.Rate(100)

	while not gui.stopped:
		parameters = gui.get_parameters()
		msg_params.robot_x = parameters[0]
		msg_params.robot_y = parameters[1]
		msg_params.robot_theta = parameters[2]
		msg_params.robot_radio = parameters[3]
		msg_params.robot_max_advance = parameters[4]
		msg_params.robot_turn_angle = parameters[5]
		msg_params.laser_num_sensors = parameters[6]
		msg_params.laser_origin = parameters[7]
		msg_params.laser_range = parameters[8]
		msg_params.laser_value = parameters[9]
		msg_params.world_name = parameters[10]
		msg_params.noise = parameters[11]
		msg_params.light_x = parameters[12]
		msg_params.light_y = parameters[13]
		msg_params.run = parameters[14]
		msg_params.behavior = parameters[15]
		msg_params.steps = parameters[16]
		msg_params.turtle = parameters[16]
		pub_params.publish(msg_params)

		x = parameters[0]
		y = parameters[1]
		th = parameters[2]
		odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

		odom_broadcaster.sendTransform(
			(x, y, 0.),
			odom_quat,
			current_time,
			"base_link_rob2w",
			"map"
			
		)

		odom = Odometry()
		odom.header.stamp = current_time
		odom.header.frame_id = "map"
		odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))
		odom.child_frame_id = "base_link_rob2w"
		odom.twist.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
		odom_pub.publish(odom)

		rate.sleep()

		#print(gui.stopped)

	for _ in range(20):
		msg_params.run = False
		pub_params.publish(msg_params)
		rate.sleep()

if __name__ == "__main__":
	time.sleep(5) #
	ros()