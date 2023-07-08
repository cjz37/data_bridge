#include <iostream>
#include <iomanip>
#include <fstream>
#include <cmath>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <GeographicLib/UTMUPS.hpp>
#include <GeographicLib/Geocentric.hpp>
using namespace GeographicLib; 

// WGS84
// 基准椭球体长半径
const double a = 6378137.0;
// 基准椭球体极扁率
const double f = 1 / 298.257223565;

bool isSetBase = false;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bag2txt__nav_msgs_Odometry");

    rosbag::Bag bag;
    bag.open(argv[1], rosbag::bagmode::Read);
    rosbag::View view(bag);

    std::ofstream fout(argv[2]);

    if (!fout)
    {
        ROS_WARN("Cannot open file to save data!");
        exit(-1);
    }

    std::string zonestr = "50n";
    int zone;
    bool northp;
    UTMUPS::DecodeZone(zonestr, zone, northp);

    double X0, Y0, Z0;
    double lon0, lat0;
    nav_msgs::Odometry::ConstPtr odom_msg_pre = nullptr, odom_msg_cur = nullptr;
    sensor_msgs::PointCloud2::ConstPtr cloud_msg = nullptr;
    double time, time0, time1; // cloud  pre_gps  cur_gps
    double pre_x, pre_y, pre_z, cur_x, cur_y, cur_z;
    double pre_qx, pre_qy, pre_qz, pre_qw, cur_qx, cur_qy, cur_qz, cur_qw;
    for (const rosbag::MessageInstance& m : view)
    {
        /*
        插值同步
        */
        // odom_msg_cur = m.instantiate<nav_msgs::Odometry>();
        // cloud_msg = m.instantiate<sensor_msgs::PointCloud2>();

        // if (odom_msg_cur != nullptr)
        // {
        //     time1 = odom_msg_cur->header.stamp.toSec();

        //     cur_x = odom_msg_cur->pose.pose.position.x;
        //     cur_y = odom_msg_cur->pose.pose.position.y;
        //     cur_z = odom_msg_cur->pose.pose.position.z;

        //     cur_qx = odom_msg_cur->pose.pose.orientation.x;
        //     cur_qy = odom_msg_cur->pose.pose.orientation.y;
        //     cur_qz = odom_msg_cur->pose.pose.orientation.z;
        //     cur_qw = odom_msg_cur->pose.pose.orientation.w;
        // }


        // if (cloud_msg != nullptr)
        // {
        //     double k = (time - time0) / (time1 - time0);

        //     double time = cloud_msg->header.stamp.toSec();
        //     // double time0 = odom_msg_pre->header.stamp.toSec();
        //     // double time1 = odom_msg_cur->header.stamp.toSec();


        //     // synced position
        //     // double synced_x = (odom_msg_cur->pose.pose.position.x - odom_msg_pre->pose.pose.position.x) * k + odom_msg_pre->pose.pose.position.x;
        //     // double synced_y = (odom_msg_cur->pose.pose.position.y - odom_msg_pre->pose.pose.position.y) * k + odom_msg_pre->pose.pose.position.y;
        //     // double synced_z = (odom_msg_cur->pose.pose.position.z - odom_msg_pre->pose.pose.position.z) * k + odom_msg_pre->pose.pose.position.z;

        //     // synced orientation

        //     fout << std::fixed << std::setprecision(9);
        //     fout << time << "\n";
        //         // << synced_x << "\t" << synced_y << "\t" << synced_z << "\n";
        //         // << odom_msg->pose.pose.orientation.x << "\t" << odom_msg->pose.pose.orientation.y << "\t" << odom_msg->pose.pose.orientation.z << "\t" << odom_msg->pose.pose.orientation.w << std::endl;

        // }


        // time0 = time1;

        // pre_x = cur_x;
        // pre_y = cur_y;
        // pre_x = cur_z;

        // pre_qx = cur_qx;
        // pre_qy = cur_qy;
        // pre_qz = cur_qz;
        // pre_qw = cur_qw;




        /*
        get UTM zone: 50
        */
        // if (gps_msg != nullptr)
        // {
        //     std::cout << std::fixed << std::setprecision(9);
        //     int zone = (gps_msg->longitude + 180.0) / 6 + 1;
        //     std::cout << gps_msg->longitude << "\tzone: " << zone << std::endl;
        // }

        odom_msg_cur = m.instantiate<nav_msgs::Odometry>();
        /*
        bag -> txt (LLA)
        */
        // if (odom_msg_cur != nullptr)
        // {
        //     double timestamp = odom_msg_cur->header.stamp.toSec();

        //     // odom_msg_cur->pose.pose.position: UTM
        //     double easting = odom_msg_cur->pose.pose.position.x;  // lat
        //     double northing = odom_msg_cur->pose.pose.position.y;  // lon
        //     double altitude = odom_msg_cur->pose.pose.position.z;  // alt
        //     bool northernHemishpere = true;

        //     // to LLA
        //     // double lat, lon, alt = 0.0;
        //     // UTMUPS::Reverse(zone, northp, easting, northing, lat, lon);

        //     fout << std::fixed << std::setprecision(9);
        //     fout << timestamp << "\t"
        //          << easting << "\t" << northing << "\t" << altitude << "\t"
        //          << odom_msg_cur->pose.pose.orientation.x << "\t" << odom_msg_cur->pose.pose.orientation.y << "\t" << odom_msg_cur->pose.pose.orientation.z << "\t" << odom_msg_cur->pose.pose.orientation.w << std::endl;

        //     std::cout << std::fixed << std::setprecision(9);
        //     std::cout << timestamp << "\t"
        //          << easting << "\t" << northing << "\t" << altitude << "\t"
        //          << odom_msg_cur->pose.pose.orientation.x << "\t" << odom_msg_cur->pose.pose.orientation.y << "\t" << odom_msg_cur->pose.pose.orientation.z << "\t" << odom_msg_cur->pose.pose.orientation.w << std::endl;

        //     // to ECEF
        //     // double N = a / sqrt(1 - f * (2 - f) * sin(lat) * sin(lat));
        //     // double X = (N + alt) * cos(lat) * cos(lon);
        //     // double Y = (N + alt) * cos(lat) * sin(lon);
        //     // double Z = (N * (1 - f) * (1 - f) + alt) * sin(lat);

        //     // if (!isSetBase) 
        //     // {
        //     //     X0 = X;
        //     //     Y0 = Y;
        //     //     Z0 = Z;
        //     //     lon0 = lon;
        //     //     lat0 = lat;
        //     //     isSetBase = true;
        //     //     continue;
        //     // }

        //     // double dX = X - X0;
        //     // double dY = Y - Y0;
        //     // double dZ = Z - Z0;

        //     // // to ENU
        //     // double e = cos(lon0) * dY - sin(lon0) * dX;
        //     // double n = cos(lat0) * dZ - sin(lat0) * cos(lon0) * dX - sin(lat0) * sin(lon0) * dY;
        //     // // double u = cos(lat0) * cos(lon0) * dX + cos(lat0) * sin(lon0) * dY + sin(lat0) * dZ;
        //     // double u = 0.0;

            
            
        //     // fout << std::fixed << std::setprecision(9);
        //     // fout << timestamp << "\t"
        //     //           << e << "\t" << n << "\t" << u << "\t"
        //     //           << odom_msg->pose.pose.orientation.x << "\t" << odom_msg->pose.pose.orientation.y << "\t" << odom_msg->pose.pose.orientation.z << "\t" << odom_msg->pose.pose.orientation.w << std::endl;

        //     // std::cout << std::fixed << std::setprecision(9);
        //     // std::cout << timestamp << "\t"
        //     //           << e << "\t" << n << "\t" << u << "\t"
        //     //           << odom_msg->pose.pose.orientation.x << "\t" << odom_msg->pose.pose.orientation.y << "\t" << odom_msg->pose.pose.orientation.z << "\t" << odom_msg->pose.pose.orientation.w << std::endl;

        //     // X0 = X;
        //     // Y0 = Y;
        //     // Z0 = Z;
        //     // lat0 = lat;
        //     // lon0 = lon;
        // }

        /*
        1057 bag -> txt(OpenCalib)
        */
        if (odom_msg_cur != nullptr)
        {
            double time = odom_msg_cur->header.stamp.toSec();

            double t1 = odom_msg_cur->pose.pose.position.x;
            double t2 = odom_msg_cur->pose.pose.position.y;
            double t3 = odom_msg_cur->pose.pose.position.z;

            Eigen::Quaterniond q;

            q.w() = odom_msg_cur->pose.pose.orientation.w;
            q.x() = odom_msg_cur->pose.pose.orientation.x;
            q.y() = odom_msg_cur->pose.pose.orientation.y;
            q.z() = odom_msg_cur->pose.pose.orientation.z;

            Eigen::Matrix3d R;
            R = q.normalized().toRotationMatrix();

            fout << std::fixed << std::setprecision(9);
            fout << time << "\t"
                 << R(0, 0) << "\t" << R(0, 1) << "\t" << R(0, 2) << "\t" << t1 << "\t"
                 << R(1, 0) << "\t" << R(1, 1) << "\t" << R(1, 2) << "\t" << t2 << "\t"
                 << R(2, 0) << "\t" << R(2, 1) << "\t" << R(2, 2) << "\t" << t3 << "\n";

        }

    }

    bag.close();
    fout.close();
}