#!/usr/bin/env python

import roslib; roslib.load_manifest('um6')
import rosbag, rospy

import datetime
from tf.msg import tfMessage
from argparse import ArgumentParser
from geometry_msgs.msg import Quaternion
from numpy import mean, array, hypot, diff, convolve, arange, sin, cos, ones, pi, matrix
from tf.transformations import euler_from_quaternion,quaternion_from_euler,quaternion_multiply,quaternion_matrix
import tf


# Prompt user if scipy is missing.
try:
  from scipy import optimize
except ImportError:
  rospy.logfatal("This script requires scipy be available.")
  rospy.logfatal("On Ubuntu: sudo apt-get install python-scipy")
  exit(1)

# Plots are optional
try:
  from matplotlib import pyplot
  from mpl_toolkits.mplot3d import Axes3D
except ImportError:
  pyplot = None

parser = ArgumentParser(description='Process UM6 bag file for compass calibration. Pass a bag containing /imu/rpy and /imu/mag topics, with the UM6 compass facing upright, being slowly rotated in a clockwise direction for 30-120 seconds.')
parser.add_argument('bag', metavar='FILE', type=str, help='input bag file')
parser.add_argument('outfile', metavar='OUTFILE', type=str, help='output yaml file',
                    nargs="?", default="/tmp/um6_calibration.yaml")
parser.add_argument('--plots', type=bool, help='Show plots if matplotlib available.')
args = parser.parse_args()

if not args.plots:
    pyplot = None

bag = rosbag.Bag(args.bag)
imu_rot = Quaternion(0,0,0,1) 
transform_found = False
for topic, msg, time in bag.read_messages(topics=("/tf", "/tf")):
	for c_trans in msg.transforms:
		if (c_trans.header.frame_id == "/base_link" and c_trans.child_frame_id == "/imu_link"):
			imu_rot = c_trans.transform.rotation
			transform_found = True
			break
	if (transform_found):
		break

temp_q = (imu_rot.x, imu_rot.y, imu_rot.z, imu_rot.w)
imu_rot = temp_q
t_array = quaternion_matrix(imu_rot)
t_mat = matrix(t_array)
t_mat = t_mat.I

if (not transform_found):
	rospy.logwarn("Transform between base_link and imu_link not found, assuming unity")

time_yaw_tuples = []
for topic, msg, time in bag.read_messages(topics=("/imu/rpy", "imu/rpy")):	
	o_msg = matrix([msg.vector.x, msg.vector.y, msg.vector.z, 0])
	o_msg = o_msg.T
	transformed_msg = t_mat*o_msg
	time_yaw_tuples.append((time.to_sec(), float(transformed_msg[2])))
	

if len(time_yaw_tuples) < 100:
  rospy.logfatal("Insufficient data or no data in bag file. Looking for topic /imu/rpy.")
  exit(1)


time_yaw = zip(*time_yaw_tuples)
time_yaw.append(diff(time_yaw[-1]))
time_yaw_tuples_filtered = [tup for tup in zip(*time_yaw) if abs(tup[-1]) < 3.0]
time_yaw = zip(*time_yaw_tuples_filtered)
# apply smoothing as a new column
w = [1.0 / 30.0] * 30
time_yaw.append(convolve(time_yaw[-1], w, 'same'))

# remove sections of no movement.
time_yaw_tuples_movement = [tup for tup in zip(*time_yaw)] # if abs(tup[-1]) > 0.01]
time_start = time_yaw_tuples_movement[50][0]
time_end = time_yaw_tuples_movement[-50][0]

if pyplot:
  fig = pyplot.figure()
  ax1 = fig.add_subplot(211)
  ax1.scatter(array(time_yaw[0]), time_yaw[-1])
  lines = pyplot.axvline(time_start)
  pyplot.axvline(time_end)
  fig.gca().add_artist(lines)

vecs = []
for topic, msg, time in bag.read_messages(topics=("/imu/mag", "imu/mag")):
  if time.to_sec() > time_start and time.to_sec() < time_end:
	o_msg = matrix([msg.vector.x, msg.vector.y, msg.vector.z, 0])
	o_msg = o_msg.T
	t_msg = t_mat*o_msg
	vecs.append((float(t_msg[0]), float(t_msg[1]), float(t_msg[2])))

rospy.loginfo("Using data from %d to %d (%d seconds), or %d samples.", time_start, time_end, time_end - time_start, len(vecs))

def calc_R(xc, yc):
    """ calculate the distance of each 2D points from the center (xc, yc) """
    return hypot(x-xc, y-yc)

def f_2(c):
    """ calculate the algebraic distance between the 2D points and the mean circle centered at c=(xc, yc) """
    Ri = calc_R(*c)
    return Ri - Ri.mean()


x,y,z = zip(*vecs)
center_estimate = mean(x), mean(y)
center, ier = optimize.leastsq(f_2, center_estimate)
radius = calc_R(*center).mean()
center = (center[0], center[1], mean(z))

a = arange(0, 2*pi + pi/50, pi/50)
circle_points = (center[0] + cos(a) * radius, 
                 center[1] + sin(a) * radius, 
                 center[2] * ones(len(a)))

rospy.loginfo("Magnetic circle centered at " + str(center) + ", with radius " + str(radius))

with open(args.outfile, "w") as f:
  f.write("# Generated from %s on %s.\n" % (args.bag, datetime.date.today()))
  f.write("mag_zero_x: %f\n" % center[0])
  f.write("mag_zero_y: %f\n" % center[1])
  f.write("mag_zero_z: %f\n" % center[2])
  f.write("mag_zero_radius: %f\n" % radius)

rospy.loginfo("Calibration file written to %s", args.outfile)

if pyplot:
  ax2 = fig.add_subplot(212, projection='3d')
  ax2.view_init(elev=80, azim=5)
  ax2.scatter(x, y, z)
  ax2.plot(*circle_points, c="red")
  pyplot.show()
