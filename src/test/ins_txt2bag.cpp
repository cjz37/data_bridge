#include <ros/ros.h>
#include <ros/param.h>
#include <rosbag/bag.h>
#include <nav_msgs/Odometry.h>

#include <iostream>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

#include <Eigen/Core>
#include <Eigen/Dense>


struct ins_data
{
    double time;
    double r11;
    double r12;
    double r13;
    double t1;
    double r21;
    double r22;
    double r23;
    double t2;
    double r31;
    double r32;
    double r33;
    double t3;
};

std::vector<ins_data> datas;

void load_data(std::string);


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ins_txt2bag");

    ros::NodeHandle n;

    ros::Rate loop_rate(100);

    load_data(argv[1]);
    int lines = datas.size();
    std::vector<ins_data>::iterator it = datas.begin();

    rosbag::Bag bag;
    bag.open(argv[2], 1U);

    int count = 0;
    std::cout << std::endl;
    while (ros::ok())
    {
        Eigen::Matrix3d rotationMatrix;
        rotationMatrix << it->r11, it->r12, it->r13,
                          it->r21, it->r22, it->r23,
                          it->r31, it->r32, it->r33;

        // std::cout << "Matrix:\n" << rotationMatrix << std::endl;

        Eigen::Quaterniond quaternion(rotationMatrix);

        // std::cout << "q = " << quaternion.w() << " " << quaternion.x() << " " << quaternion.y() << " " << quaternion.z() << std::endl;

        nav_msgs::Odometry ins_msg;
        ins_msg.header.stamp = ros::Time(it->time);
        ros::param::get("frame_id", ins_msg.header.frame_id);
        ins_msg.pose.pose.orientation.x = quaternion.x();
        ins_msg.pose.pose.orientation.y = quaternion.y();
        ins_msg.pose.pose.orientation.z = quaternion.z();
        ins_msg.pose.pose.orientation.w = quaternion.w();
        
        bag.write("ins", ins_msg.header.stamp, ins_msg);

        double remain_sum_seconds = 1.0 * (lines-count) / 100;
        int remain_minutes = (int)remain_sum_seconds / 60;
        int remain_seconds = remain_sum_seconds - remain_minutes * 60;
        count++;

        std::cout << "\033[0G\033[K";
        std::cout << "<" << remain_minutes << "m" << remain_seconds << "s remaining>"
                //   << "time=" << ins_msg.header.stamp << ", frame_id=" << ins_msg.header.frame_id << ", q=(" << ins_msg.pose.pose.orientation.x << "," << ins_msg.pose.pose.orientation.y << "," << ins_msg.pose.pose.orientation.z << "," << ins_msg.pose.pose.orientation.w << ")"
                  << std::flush;

        it++;
        if (it == datas.end())
            break;
        loop_rate.sleep();
    }
    std::cout << std::endl;

    bag.close();
}


void load_data(std::string path)
{
    std::string line;
    int num[10];
    int i = 0;
    ins_data ins;
    std::ifstream infile;
    infile.open(path);
    if (!infile)
    {
        std::cout << "Cannot open file: " << path << std::endl;
        exit(1);
    }
    ins_data output_ins;
    while (std::getline(infile, line))
    {
        std::stringstream ss(line);
        std::string word;
        i = 0;
        while (ss >> word)
        {
            ++i;
            if (i > 13)
                break;
            switch (i)
            {
            case 1:
                output_ins.time = std::stod(word);
                break;
            
            case 2:
                output_ins.r11 = std::stod(word);
                break;
            
            case 3:
                output_ins.r12 = std::stod(word);
                break;

            case 4:
                output_ins.r13 = std::stod(word);
                break;

            case 5:
                output_ins.t1 = std::stod(word);
                break;

            case 6:
                output_ins.r21 = std::stod(word);
                break;
            
            case 7:
                output_ins.r22 = std::stod(word);
                break;

            case 8:
                output_ins.r23 = std::stod(word);
                break;

            case 9:
                output_ins.t2 = std::stod(word);
                break;

            case 10:
                output_ins.r31 = std::stod(word);
                break;

            case 11:
                output_ins.r32 = std::stod(word);
                break;

            case 12:
                output_ins.r33 = std::stod(word);
                break;

            case 13:
                output_ins.t3 = std::stod(word);
                break;

            default:
                break;
            }
        }
        datas.push_back(output_ins);
    }
    infile.close();
}
