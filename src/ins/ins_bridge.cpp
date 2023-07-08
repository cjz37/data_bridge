#include <iostream>
#include <iomanip>
#include <fstream>
#include <cmath>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <nav_msgs/Odometry.h>

#include <Eigen/Core>
#include <Eigen/Dense>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "ins_bridge_node");

    rosbag::Bag bag;
    bag.open(argv[1], rosbag::bagmode::Read);
    rosbag::View view(bag);

    std::ofstream fout1(argv[2]);
    std::ofstream fout2(argv[3]);

    if (!fout1 || !fout2)
    {
        ROS_WARN("Cannot open file to save data!");
        exit(-1);
    }

    nav_msgs::Odometry::ConstPtr odom_msg_cur = nullptr;

    // 记录起始点坐标
    Eigen::Translation3d odom_base_position;
    bool isFirst = true;
    for (const rosbag::MessageInstance& msg : view)
    {
        odom_msg_cur = msg.instantiate<nav_msgs::Odometry>();

        if (odom_msg_cur != nullptr)
        {
            // 惯导转90度
            // Eigen::Quaterniond q(odom_msg_cur->pose.pose.orientation.w, odom_msg_cur->pose.pose.orientation.x, odom_msg_cur->pose.pose.orientation.y, odom_msg_cur->pose.pose.orientation.z);
            // Eigen::Matrix3d R, tempR;
            // R = q.normalized().toRotationMatrix();
            // tempR << 1, 0, 0,
            //          0, 0, -1,
            //          0, 1, 0;
            // R = R * tempR;
            // q = Eigen::Quaterniond(R);
            // q.normalized();

            // fout1 << std::fixed << std::setprecision(9);
            // fout1 << odom_msg_cur->header.stamp.toSec() << "\t"
            //      << odom_msg_cur->pose.pose.position.x << "\t" << odom_msg_cur->pose.pose.position.y << "\t" << odom_msg_cur->pose.pose.position.z << "\t"
            //      << q.x() << "\t" << q.y() << "\t" << q.z() << "\t" << q.w() << "\n";
            
            // std::cout << std::fixed << std::setprecision(9);
            // std::cout << odom_msg_cur->header.stamp.toSec() << "\t"
            //          << odom_msg_cur->pose.pose.position.x << "\t" << odom_msg_cur->pose.pose.position.y << "\t" << odom_msg_cur->pose.pose.position.z << "\t"
            //          << q.x() << "\t" << q.y() << "\t" << q.z() << "\t" << q.w() << "\n";

            // 用于pose_align ORIGINAL
            fout1 << std::fixed << std::setprecision(9);
            fout1 << odom_msg_cur->header.stamp.toSec() << "\t"
                << odom_msg_cur->pose.pose.position.x << "\t" << odom_msg_cur->pose.pose.position.y << "\t" << odom_msg_cur->pose.pose.position.z << "\t"
                << odom_msg_cur->pose.pose.orientation.x << "\t" << odom_msg_cur->pose.pose.orientation.y << "\t" << odom_msg_cur->pose.pose.orientation.z << "\t" << odom_msg_cur->pose.pose.orientation.w << "\n";
            
            std::cout << std::fixed << std::setprecision(9);
            std::cout << odom_msg_cur->header.stamp.toSec() << "\t"
                    << odom_msg_cur->pose.pose.position.x << "\t" << odom_msg_cur->pose.pose.position.y << "\t" << odom_msg_cur->pose.pose.position.z << "\t"
                    << odom_msg_cur->pose.pose.orientation.x << "\t" << odom_msg_cur->pose.pose.orientation.y << "\t" << odom_msg_cur->pose.pose.orientation.z << "\t" << odom_msg_cur->pose.pose.orientation.w << "\n";

            // 用于OpenCalib
            Eigen::Quaterniond q(odom_msg_cur->pose.pose.orientation.w, odom_msg_cur->pose.pose.orientation.x, odom_msg_cur->pose.pose.orientation.y, odom_msg_cur->pose.pose.orientation.z);
            Eigen::Matrix3d R;
            R = q.normalized().toRotationMatrix();

            fout2 << std::fixed << std::setprecision(9);
            fout2 << odom_msg_cur->header.stamp.toSec() << "\t"
                << R(0, 0) << "\t" << R(0, 1) << "\t" << R(0, 2) << "\t" << odom_msg_cur->pose.pose.position.x << "\t"
                << R(1, 0) << "\t" << R(1, 1) << "\t" << R(1, 2) << "\t" << odom_msg_cur->pose.pose.position.y << "\t"
                << R(2, 0) << "\t" << R(2, 1) << "\t" << R(2, 2) << "\t" << odom_msg_cur->pose.pose.position.z << "\n";
        }
    }

    bag.close();
    fout1.close();
    fout2.close();

    ROS_INFO("Done.");
}