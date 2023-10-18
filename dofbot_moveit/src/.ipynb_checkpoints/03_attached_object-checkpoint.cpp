#include <iostream>
#include "ros/ros.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/LinearMath/Quaternion.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

using namespace std;

int main(int argc, char **argv) {
    //ROS节点初始化
    ros::init(argc, argv, "dofbot_attached_object_cpp");
    //创建节点句柄
    ros::NodeHandle n;
    // 设置线程
    ros::AsyncSpinner spinner(1);
    // 开启线程
    spinner.start();
    // 初始化机械臂
    moveit::planning_interface::MoveGroupInterface dofbot("dofbot");
    // 获取机器人正在计划的框架的名称
    string frame = dofbot.getPlanningFrame();
    // 创建场景实例
    moveit::planning_interface::PlanningSceneInterface scene;
    /////////////////////////添加障碍物///////////////////////
    vector<string> object_ids;
    scene.removeCollisionObjects(object_ids);
    // 创建检测对象容器
    vector<moveit_msgs::CollisionObject> objects;
    // 创建碰撞检测对象
    moveit_msgs::CollisionObject obj;
    // 设置障碍物的id
    obj.id = "obj";
    object_ids.push_back(obj.id);
    // 障碍物的状态
    obj.operation = obj.ADD;
    // 设置障碍物的头信息
    obj.header.frame_id = frame;
    shape_msgs::SolidPrimitive primitive;
    // 设置障碍物类型
    primitive.type = primitive.BOX;
    // 设置障碍物维度
    primitive.dimensions.resize(3);
    // 设置障碍物的长宽高
    primitive.dimensions[0] = 0.1;
    primitive.dimensions[1] = 0.2;
    primitive.dimensions[2] = 0.1;
    obj.primitives.push_back(primitive);
    geometry_msgs::Pose pose;
    // 设置障碍物的位置信息[x,y,z]
    pose.position.x = 0;
    pose.position.y = 0.2;
    pose.position.z = 0.3;
    // 设置障碍物的姿态信息
    tf::Quaternion quaternion;
    // R,P,Y的单位是角度
    double Roll = 0.0;
    double Pitch = 0.0;
    double Yaw = 90.0;
    // 将RPY转换成四元数
    quaternion.setRPY(Roll * M_PI / 180, Pitch * M_PI / 180, Yaw * M_PI / 180);
    pose.orientation.x = quaternion.x();
    pose.orientation.y = quaternion.y();
    pose.orientation.z = quaternion.z();
    pose.orientation.w = quaternion.w();
    // 设置障碍物的位姿信息
    obj.primitive_poses.push_back(pose);
    objects.push_back(obj);
    /////////////////////////设置障碍物的颜色///////////////////////
    // 创建检测对象的颜色容器
    std::vector<moveit_msgs::ObjectColor> colors;
    // 创建颜色实例
    moveit_msgs::ObjectColor color;
    // 添加需要设置颜色的id
    color.id = "obj";
    // 设置RGBA值,范围[0~1]
    color.color.r = 0;
    color.color.g = 1.0;
    color.color.b = 0;
    color.color.a = 0.5;
    colors.push_back(color);
    // 将设置的信息添加到场景中
    scene.applyCollisionObjects(objects, colors);
    // 阻塞进程
    ros::waitForShutdown();
    return 0;
}

