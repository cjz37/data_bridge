#include <iostream>
#include <iomanip>
#include <fstream>
#include <cmath>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Core>
#include <Eigen/Dense>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "ins_synce_node");

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

    nav_msgs::Odometry::ConstPtr odom_msg_pre = nullptr, odom_msg_cur = nullptr;
    sensor_msgs::PointCloud2::ConstPtr cloud_msg_pre = nullptr, cloud_msg_cur = nullptr;

    // 记录起始点坐标
    Eigen::Translation3d odom_base_position;
    bool isFirst = true;
    bool getNewCloud = false;
    bool preOdom = false;
    for (const rosbag::MessageInstance& msg : view)
    {
        odom_msg_cur = msg.instantiate<nav_msgs::Odometry>();
        cloud_msg_cur = msg.instantiate<sensor_msgs::PointCloud2>();
        
        if (odom_msg_cur != nullptr)
        {
            // 插值
            if (getNewCloud && preOdom)
            {
                getNewCloud = false;
                double time_cloud = cloud_msg_pre->header.stamp.toSec();
                double time_odom_cur = odom_msg_cur->header.stamp.toSec();
                double time_odom_pre = odom_msg_pre->header.stamp.toSec();

                double k = (time_cloud - time_odom_pre) / (time_odom_cur - time_odom_pre);

                Eigen::Translation3d p0_trans(Eigen::Vector3d(odom_msg_pre->pose.pose.position.x, odom_msg_pre->pose.pose.position.y, odom_msg_pre->pose.pose.position.z));
                Eigen::Quaterniond p0_quat(odom_msg_pre->pose.pose.orientation.w, odom_msg_pre->pose.pose.orientation.x, odom_msg_pre->pose.pose.orientation.y, odom_msg_pre->pose.pose.orientation.z);
                
                Eigen::Translation3d p1_trans(Eigen::Vector3d(odom_msg_cur->pose.pose.position.x, odom_msg_cur->pose.pose.position.y, odom_msg_cur->pose.pose.position.z));
                Eigen::Quaterniond p1_quat(odom_msg_cur->pose.pose.orientation.w, odom_msg_cur->pose.pose.orientation.x, odom_msg_cur->pose.pose.orientation.y, odom_msg_cur->pose.pose.orientation.z);

                // 四元数插值
                Eigen::Quaterniond slerp_quat = p0_quat.slerp(1 - k, p1_quat);

                // 3D位置坐标插值
                Eigen::Translation3d interp_trans;
                interp_trans.x() = p1_trans.x() * k + p0_trans.x() * (1 - k);
                interp_trans.y() = p1_trans.y() * k + p0_trans.y() * (1 - k);
                interp_trans.z() = p1_trans.z() * k + p0_trans.z() * (1 - k);

                // 设置基准点，之后的position均为相对于该点的平移量
                if (isFirst)
                {
                    isFirst = false;
                    odom_base_position.x() = interp_trans.x();
                    odom_base_position.y() = interp_trans.y();
                    odom_base_position.z() = interp_trans.z();
                }

                // 用于pose_align
                fout1 << std::fixed << std::setprecision(9);
                fout1 << time_cloud << "\t"
                     << interp_trans.x() - odom_base_position.x() << "\t" << interp_trans.y() - odom_base_position.y() << "\t" << interp_trans.z() - odom_base_position.z() << "\t"
                     << slerp_quat.x() << "\t" << slerp_quat.y() << "\t" << slerp_quat.z() << "\t" << slerp_quat.w() << "\n";
                
                std::cout << std::fixed << std::setprecision(9);
                std::cout << time_cloud << "\t"
                          << interp_trans.x() - odom_base_position.x() << "\t" << interp_trans.y() - odom_base_position.y() << "\t" << interp_trans.z() - odom_base_position.z() << "\t"
                          << slerp_quat.x() << "\t" << slerp_quat.y() << "\t" << slerp_quat.z() << "\t" << slerp_quat.w() << "\n";

                // 用于OpenCalib
                Eigen::Matrix3d R;
                R = slerp_quat.normalized().toRotationMatrix();

                fout2 << std::fixed << std::setprecision(9);
                fout2 << time_cloud << "\t"
                    << R(0, 0) << "\t" << R(0, 1) << "\t" << R(0, 2) << "\t" << interp_trans.x() - odom_base_position.x() << "\t"
                    << R(1, 0) << "\t" << R(1, 1) << "\t" << R(1, 2) << "\t" << interp_trans.y() - odom_base_position.y() << "\t"
                    << R(2, 0) << "\t" << R(2, 1) << "\t" << R(2, 2) << "\t" << interp_trans.z() - odom_base_position.z() << "\n";
            }

            odom_msg_pre = odom_msg_cur;
            preOdom = true;
        }

        if (cloud_msg_cur != nullptr)
        {
            getNewCloud = true;
            cloud_msg_pre = cloud_msg_cur;
        }
    }

    bag.close();
    fout1.close();
    fout2.close();

    ROS_INFO("Done.");
}