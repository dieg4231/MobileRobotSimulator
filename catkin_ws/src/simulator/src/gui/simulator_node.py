#!/usr/bin/env python

from MobileRobotSimulator import *
from simulator.srv import *
from minibot_msgs.srv import *
from simulator.msg import Parameters
from simulator.msg import PosesArray
from simulator.msg import poseCustom
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion 
import tf
import time
import rospy
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped
from std_msgs.msg import Int8MultiArray

gui=MobileRobotSimulator()

battery_low = False
battery_charging = False

def handlePoseByAruco(msg):
	quaternion = (
    msg.pose.orientation.x,
    msg.pose.orientation.y,
    msg.pose.orientation.z,
    msg.pose.orientation.w)

	euler = euler_from_quaternion(quaternion)
	roll = euler[0]
	pitch = euler[1]
	yaw = euler[2]
	gui.handlePoseByAruco(msg.pose.position.y,msg.pose.position.x,-yaw+1.5707)


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

def convertArray2Pose(objects_data):
	arrayPosesObjs = PosesArray();
	for obj in objects_data:
		tmp = poseCustom()
		tmp.name = obj[0]
		tmp.x = obj[1]
		tmp.y = obj[2]
		arrayPosesObjs.posesArray.append(tmp)
	return arrayPosesObjs

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

def turn_lights(lights_array):

	lights = Int8MultiArray()
	lights.data = lights_array
	lights_pub.publish(lights)

def publish_movement(move):
	rate = rospy.Rate(1.2)
	move_cmd = Twist()

	try:
		if move[0] == 1:
			move_cmd.linear.x  = 0.5
			move_cmd.angular.z = 0
			move_pub.publish(move_cmd)
			rate.sleep()

		elif move[1] == 1:
			move_cmd.linear.x  = -0.5
			move_cmd.angular.z = 0
			move_pub.publish(move_cmd)
			rate.sleep()

		elif move[2] == 1:
			move_cmd.linear.x  = 0
			move_cmd.angular.z = 2.4
			move_pub.publish(move_cmd)
			rate.sleep()

		elif move[3] == 1:
			move_cmd.linear.x  = 0
			move_cmd.angular.z = -2.4
			move_pub.publish(move_cmd)
			rate.sleep()

		else:
			move_cmd.linear.x  = 0
			move_cmd.angular.z = 0
			move_pub.publish(move_cmd)
	except:
			move_cmd.linear.x  = 0
			move_cmd.angular.z = 0
			move_pub.publish(move_cmd)

def battery_charge():
	try:
		get_batt_perc = rospy.ServiceProxy('/battery_perc', GetBattPerc)
		gui.batteryBar['value'] = get_batt_perc().batt_percentage
	except rospy.ServiceException as e:
		gui.batteryBar['value'] = 0
	
def battery_advertise(message, color):
	gui.labelBattAdvertise.grid_forget()
	gui.labelBattAdvertise.config( text = message , bg = color )
	gui.labelBattAdvertise.grid(column = 4 ,row = 22 ,sticky = (N, W) ,padx = (10,5))
	
def get_params():
	global battery_low, battery_charging
	if rospy.has_param('/battery_low'):
		battery_low = rospy.get_param("/battery_low")
	if rospy.has_param('/battery_charging'):
		battery_charging = rospy.get_param("/battery_charging")

def ros():
	global lights_pub, move_pub
	rospy.init_node('simulator_gui_node')
	a = rospy.Service('simulator_robot_step', simulator_robot_step, handle_robot_step)
	b = rospy.Service('simulator_print_graph', simulator_algorithm_result, handle_print_graph)
	c = rospy.Service('simulator_stop', simulator_stop, handle_simulator_stop)
	d = rospy.Service('simulator_set_light_position', simulator_set_light_position, handle_simulator_set_light_position)
	e = rospy.Service('simulator_object_interaction', simulator_object_interaction, handle_simulator_object_interaction)
	
	#rospy.Subscriber('/scan',LaserScan,update_value,queue_size=1)
	#rospy.Subscriber('/odom',Odometry, turtle_odometry ,queue_size=1)
	rospy.Subscriber('/robotPoseByAruco',PoseStamped, handlePoseByAruco ,queue_size=1)
	odom_pub = rospy.Publisher("/odom_simul", Odometry, queue_size=50)
	objPose_pub = rospy.Publisher("/objectsPose", PosesArray, queue_size=5)
	odom_broadcaster = tf.TransformBroadcaster()
	lights_pub = rospy.Publisher("/turn_lights", Int8MultiArray, queue_size=5)
	move_pub   = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

	
	x = 0.0
	y = 0.0
	th = 0.0

	vx = 0.1
	vy = -0.1
	vth = 0.1
	current_time = rospy.Time.now()
	last_time = rospy.Time.now()

	lights_array = [0, 0]
	last_movement = []

	pub_params = rospy.Publisher('simulator_parameters_pub', Parameters, queue_size = 0)
	#rospy.Subscriber("simulator_laser_pub", Laser_values, callback)

	get_params()
	battery_charge()

	msg_params = Parameters()
	rate = rospy.Rate(100)
	

	start = time.time()
	labelColor = gui.warnLightColor

	robot_selected = 'No selected'

	if rospy.has_param('/robot_selected'):
		robot_selected = rospy.get_param("/robot_selected")

	gui.labelRobot = Label(gui.rightMenu ,text = "Robot: " + robot_selected, background = gui.backgroundColor, foreground = gui.titlesColor ,font = gui.headLineFont )
	gui.labelRobot.grid(column = 4 ,row = 0 ,sticky = (N, W) ,padx = (5,5))    

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
		msg_params.useRealRobot = parameters[16]
		msg_params.useLidar = parameters[17]
		msg_params.useSArray = parameters[18]
		msg_params.realLights = [parameters[19], parameters[20], 0, 0, 0, 0]

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


		objPose_pub.publish(convertArray2Pose(gui.objects_data))		   
		
		if lights_array != [parameters[19], parameters[20]]:
			lights_array = [parameters[19], parameters[20]]
			turn_lights(lights_array)
		
		if last_movement != parameters[21]:
			publish_movement(parameters[21])
			last_movement = parameters[21]
			gui.movement = [0, 0, 0, 0]
		
		if(gui.isRunning is False):
			battery_charge()

		rate.sleep()


		get_params()
		end = time.time()
		
		if(end - start > 0.5):
			start = time.time()	
			if battery_charging:
				if labelColor == gui.warnLightColor:
					labelColor = gui.warnStrongColor
				else:
					labelColor = gui.warnLightColor

				battery_advertise('Battery charging...', labelColor)
		
			elif battery_low:
				if labelColor == gui.errorLightColor:
					labelColor = gui.errorStrongColor
				else:
					labelColor = gui.errorLightColor

				battery_advertise('Battery low :,O', labelColor)
			else: 
				gui.labelBattAdvertise.grid_forget()

		#print(gui.stopped)
	
	for _ in range(20):
		msg_params.run = False
		pub_params.publish(msg_params)
		rate.sleep()


if __name__ == "__main__":
	time.sleep(5) #
	ros()
	turn_lights([0, 0, 0, 0, 0, 0])
