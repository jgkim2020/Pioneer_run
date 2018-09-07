#!/usr/bin/env python
import rosbag
import tf
import tf.transformations as tft
import matplotlib.pyplot as plt
from argparse import ArgumentParser
import numpy as np
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
import rospy
from scipy.interpolate import interp1d
from scipy.signal import savgol_filter

global t_true,x_true,y_true,isClear
global callback_ckpro



def true_pos_callback(data):
    global x_true, y_true, t_true


    x_true.append(data.x)
    y_true.append(data.y)
    now = rospy.get_rostime()
    t_now = now.secs + now.nsecs * 1e-9
    t_true.append(t_now)



if __name__ == "__main__":

    ref_path = Path()
    ref_path.header.frame_id = "/world"

    rospy.init_node("ref_talker")
    callback_ckp = rospy.get_rostime()

    t_ref = []
    x_ref = []
    y_ref = []

    t_true = []
    x_true = []
    y_true = []


    ref_traj_topic_name = rospy.get_param("~ref_traj_topic_name")
    bagfile_name = rospy.get_param('~bagfile_name')
    target_name = rospy.get_param('~target_name')
    print (bagfile_name)

    bag = rosbag.Bag(bagfile_name)
    for topic, msg, t in bag.read_messages(topics=['/gazebo/model_states']):
        target_idx = np.where(np.array(msg.name) == target_name)[0][0]
        target_pos = msg.pose[target_idx]
        x_ref.append(target_pos.position.x)
        y_ref.append(target_pos.position.y)
        t_ref.append(t.secs + t.nsecs * 1e-9)
        # poseStamped = PoseStamped()
        # poseStamped.header.frame_id = "/world"
        # poseStamped.pose.position
        # ref_path.poses

    x_ref = savgol_filter(x_ref,5001,0) -2
    y_ref = savgol_filter(y_ref, 5001, 0)

    fx = interp1d(t_ref, x_ref)
    fy = interp1d(t_ref, y_ref)

    t_duration = t_ref[-1] - t_ref[0]

    ref_pub = rospy.Publisher(ref_traj_topic_name,Point, queue_size = 1)
    path_pub = rospy.Publisher("ref_path",Path,queue_size=1)
    true_sub = rospy.Subscriber('pioneer_controller/target_position',Point,true_pos_callback)

    t_init_ref = t_ref[0]
    now = rospy.get_rostime()
    t_init=now.secs + now.nsecs * 1e-9
    t_now = t_init
    rate = rospy.Rate(200)

    plt.figure(1)
    plt.subplot(211)
    plt.plot(t_ref, x_ref, 'r', label='x_ref')
    plt.title('target x')
    plt.legend()
    plt.subplot(212)
    plt.plot(t_ref, y_ref, 'r', label='y_ref')
    plt.title('target y')
    plt.legend()
    plt.ion()
    plt.show()
    plt.pause(0.00001)

    print("t_now : %f / t_duration %f" % (t_now,t_duration))


    while (not rospy.is_shutdown()) and (t_now - t_init)< t_duration:

        now = rospy.get_rostime()
        t_now= now.secs + now.nsecs * 1e-9

        t_eval = t_now - t_init + t_init_ref


        goal_point = Point()
        goal_point.x = fx(t_eval)
        goal_point.y = fy(t_eval)


        ref_pub.publish(goal_point)
        # print("publish goal_topic [ %f , %f ]" % (goal_point.x,goal_point.y))
        #
        #
        # print("t_now: %f / t_init: %f " % (t_now,t_init))


        plt.figure(1)
        plt.subplot(211)
        plt.plot(t_eval,goal_point.x,'r*')
        try :
            plt.plot(np.array(t_true) - t_init + t_init_ref ,x_true,'k')
        except:
            print('length error')
        plt.legend()
        plt.subplot(212)
        plt.plot(t_eval,goal_point.y,'r*')
        try :
            plt.plot(np.array(t_true) - t_init + t_init_ref ,y_true,'k')
        except:
            print('length error')

        plt.ion()
        plt.show()
        plt.pause(0.00001)
        rate.sleep()






