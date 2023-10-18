#!/usr/bin/env python
# coding: utf-8
import rospy
from math import pi
from time import sleep
import moveit_commander
from geometry_msgs.msg import PoseStamped
from moveit_commander.move_group import MoveGroupCommander
from tf.transformations import quaternion_from_euler

# 角度转弧度
DE2RA = pi / 180

if __name__ == '__main__':
    # 初始化节点
    rospy.init_node("set_pose_py")
    # 初始化机械臂
    yahboomcar = MoveGroupCommander("arm_group")
    # 当运动规划失败后，允许重新规划
    yahboomcar.allow_replanning(True)
    yahboomcar.set_planning_time(5)
    # 尝试规划的次数
    yahboomcar.set_num_planning_attempts(10)
    # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
    yahboomcar.set_goal_position_tolerance(0.01)
    yahboomcar.set_goal_orientation_tolerance(0.01)
    # 设置允许目标误差
    yahboomcar.set_goal_tolerance(0.01)
    # 设置允许的最大速度和加速度
    yahboomcar.set_max_velocity_scaling_factor(1.0)
    yahboomcar.set_max_acceleration_scaling_factor(1.0)
    # 设置"down"为目标点
    yahboomcar.set_named_target("down")
    yahboomcar.go()
    sleep(0.5)
    # 创建位姿实例
    pos = PoseStamped()
    # 设置具体的位置
    pos.header.frame_id = "base_footprint"#0.2
    pos.pose.position.x = 0.01#0.2
    pos.pose.position.y = 0.0#0.0
    pos.pose.position.z = 0.178#0.2178
    # RPY的单位是角度值
    #roll = 90.0#-180
    #pitch = 45.0#45
    #yaw = 90.0#-180
    # RPY转四元素
    #q = quaternion_from_euler(roll * DE2RA, pitch * DE2RA, yaw * DE2RA)
    pos.pose.orientation.x = 0
    pos.pose.orientation.y = 0
    pos.pose.orientation.z = -0.02
    pos.pose.orientation.w = 0.0999
    # 设置目标点
    yahboomcar.set_pose_target(pos)
    # 多次执行,提高成功率
    for i in range(5):
        # 运动规划
        plan = yahboomcar.plan()
        if len(plan.joint_trajectory.points) != 0:
            print ("plan success")
            # 规划成功后运行
            yahboomcar.execute(plan)
            break
        else: print ("plan error")
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)
