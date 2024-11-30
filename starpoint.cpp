#include <ros/ros.h>

//topic 头文件
#include <iostream>
#include <px4_command/command.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <stdlib.h>
#include <math_utils.h>
#define inf 0x3f3f3f3f

using namespace std;

enum Command
{
    Move_ENU,
    Move_Body,
    Hold,
    Takeoff,
    Land,
    Arm,
    Disarm,
    Failsafe_land,
    Idle
};

px4_command::command Command_now;
sensor_msgs::LaserScan Laser;                                   //激光雷达点云数据
geometry_msgs::PoseStamped pos_drone;                                  //无人机当前位置
Eigen::Quaterniond q_fcu;
Eigen::Vector3d Euler_fcu;
float size_square;   // 五角星大小（直径）
float height_square; // 飞行高度
float sleep_time;

void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    pos_drone = *msg;
    // Read the Quaternion from the Mavros Package [Frame: ENU]
    Eigen::Quaterniond q_fcu_enu(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    q_fcu = q_fcu_enu;
    //Transform the Quaternion to Euler Angles
    Euler_fcu = quaternion_to_euler(q_fcu);
}
float cal_dis(float x1, float y1, float x2, float y2)
{
    return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}

// 五角星顶点坐标计算函数
void calculate_star_points(float radius, float z, vector<pair<float, float>> &points)
{
    float angle_step = M_PI / 5 * 3; // 每个顶点间隔角度36° * 3
    for (int i = 0; i < 5; i++)
    {
        // float r = (i % 2 == 0) ? radius : radius / 2.6; // 外顶点和内顶点的半径比例
        // float theta = i * angle_step;                  // 当前点的角度
        float x = r * cos(angle_step);
        float y = r * sin(angle_step);
        points.push_back(make_pair(x, y));
    }
    points.push_back(make_pair(0, 0));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "starpoint");
    ros::NodeHandle nh("~");
    ros::Rate rate(1.0);
    ros::Publisher move_pub = nh.advertise<px4_command::command>("/px4/command", 10);

    nh.param<float>("size_square", size_square, 1.0);
    nh.param<float>("height_square", height_square, 0.5);
    nh.param<float>("sleep_time", sleep_time, 10.0);

    int check_flag;
    cout << "size_square: " << size_square << "[m]" << endl;
    cout << "height_square: " << height_square << "[m]" << endl;
    cout << "Please check the parameter and setting, 1 for go on, else for quit: " << endl;
    cin >> check_flag;
    if (check_flag != 1)
    {
        return -1;
    }

    int Arm_flag;
    cout << "Whether choose to Arm? 1 for Arm, 0 for quit" << endl;
    cin >> Arm_flag;
    if (Arm_flag == 1)
    {
        Command_now.command = Arm;
        move_pub.publish(Command_now);
    }
    else
        return -1;

    int takeoff_flag;
    cout << "Whether choose to Takeoff? 1 for Takeoff, 0 for quit" << endl;
    cin >> takeoff_flag;
    if (takeoff_flag != 1)
    {
        return -1;
    }

    vector<pair<float, float>> star_points;
    calculate_star_points(size_square / 2.0, height_square, star_points);

    int comid = 0;
    int i = 0;
    float absdis = inf;

    // 起飞到指定高度
    
    do{
        ros::spinOnce();
        Command_now.command = Move_ENU;
        Command_now.sub_mode = 0;
        Command_now.pos_sp[0] = 0;
        Command_now.pos_sp[1] = 0;
        Command_now.pos_sp[2] = height_square;
        Command_now.yaw_sp = 0;
        Command_now.comid = comid++;
        move_pub.publish(Command_now);
        rate.sleep();
        cout << "Takeoff to height_square: " << height_square << "m" << endl;
        absdis = cal_dis(pos_drone.pose.position.x, pos_drone.pose.position.y, Command_now.pos_sp[0], Command_now.pos_sp[1]);
        i++;
    }while (absdis > 0.3);

    // 按顺序飞向五角星的每个顶点
    for (int point_idx = 0; point_idx < star_points.size(); ++point_idx)
    {
        do
        {
            ros::spinOnce();
            Command_now.command = Move_ENU;
            Command_now.sub_mode = 0;
            Command_now.pos_sp[0] = star_points[point_idx].first;
            Command_now.pos_sp[1] = star_points[point_idx].second;
            Command_now.pos_sp[2] = height_square;
            Command_now.yaw_sp = 0;
            Command_now.comid = comid++;
            move_pub.publish(Command_now);
            rate.sleep();
            cout << "Moving to Point " << point_idx + 1 << ": (" << star_points[point_idx].first << ", " << star_points[point_idx].second << ", " << height_square << ")" << endl;
            cal_dis(pos_drone.pose.position.x, pos_drone.pose.position.y, Command_now.pos_sp[0], Command_now.pos_sp[1]);
            i++;
        } while(absdis > 0.3);
    }

    // 降落
    Command_now.command = Land;
    while (ros::ok())
    {
        move_pub.publish(Command_now);
        rate.sleep();
        cout << "Landing..." << endl;
    }

    cout << "Mission complete, exiting..." << endl;
    return 0;
}