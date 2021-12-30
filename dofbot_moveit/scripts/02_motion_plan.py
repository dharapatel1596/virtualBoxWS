#!/usr/bin/env python
# coding: utf-8
import rospy
from Arm_Lib import Arm_Device
from math import pi
from time import sleep
import moveit_commander
from geometry_msgs.msg import Pose, PoseStamped
from moveit_commander import MoveGroupCommander,PlanningSceneInterface
from tf.transformations import quaternion_from_euler
import sqlite3
from sqlite3 import Error


DE2RA = pi / 180

def create_connection(db_file):
    """ create a database connection to the SQLite database
        specified by the db_file
    :param db_file: database file
    :return: Connection object or None
    """
    conn = None
    try:
        conn = sqlite3.connect(db_file)
    except Error as e:
        print(e)

    return conn

if __name__ == '__main__':

    rospy.init_node("dofbot_motion_plan_py")

    dofbot = MoveGroupCommander("dofbot")

    Arm = Arm_Device()
    dofbot.set_named_target("up")
    dofbot.go()
    sleep(2)
    Arm.Arm_serial_servo_write(6,60,200)
    sleep(2)

    dofbot.allow_replanning(True)
    dofbot.set_planning_time(5)

    dofbot.set_num_planning_attempts(10)

    dofbot.set_goal_position_tolerance(0.01)

    dofbot.set_goal_orientation_tolerance(0.01)

    dofbot.set_goal_tolerance(0.01)

    dofbot.set_max_velocity_scaling_factor(1.0)

    dofbot.set_max_acceleration_scaling_factor(1.0)
    #brown spot
    database = r"/home/jetson/dofbotDB.db"
    conn = create_connection(database)
    with conn:
        print("connection received\n")
	cur = conn.cursor()
	cur.execute("select * from tasks where spotname=brown")
	raw = cur.fetchall()
        print(raw)

    #j1 = (88 - 90) * DE2RA
    #j2 = (54 - 90) * DE2RA
    #j3 = (51 - 90) * DE2RA
    #j4 = (13 - 90) * DE2RA
    #j5 = (90 - 90) * DE2RA
    #target_joints1 = [j1, j2, j3, j4, j5]
    #red spot
    #j1 = (115 - 90) * DE2RA
    #j2 = (28 - 90) * DE2RA
    #j3 = (75 - 90) * DE2RA
    #j4 = (33 - 90) * DE2RA
    #j5 = (90 - 90) * DE2RA
    #target_joints2 = [j1, j2, j3, j4, j5]
    #tool_size = [0.03,0.03,0.03]
    #end_effector_link = dofbot.get_end_effector_link()
    
    #p = PoseStamped()
    #p.header.frame_id = end_effector_link
    #p.pose.position.x = 0
    #p.pose.position.y = 0
    #p.pose.position.z = 0.10
    #p.pose.orientation.x = 0
    #p.pose.orientation.y = 0
    #p.pose.orientation.z = 0
    #p.pose.orientation.w = 1

    #while 1:
    #    dofbot.set_joint_value_target(target_joints1)
    #    dofbot.go()
    #    sleep(2)
    #    Arm.Arm_serial_servo_write(6,135,200)
    #    sleep(2)    
    #    dofbot.set_named_target("up")
    #    dofbot.go()
    #    sleep(2)

    #    dofbot.set_joint_value_target(target_joints2)
    #    dofbot.go()
    #    Arm.Arm_serial_servo_write(6,60,200)
    #    sleep(2)

    #    dofbot.set_named_target("up")
    #    dofbot.go()
    #    sleep(2)

