#include <iostream>
#include <iomanip>
#include <fstream>
#include <cmath>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>


void convertIntensityToFloat(const sensor_msgs::PointCloud2::ConstPtr &msg, rosbag::Bag &output_bag, const ros::Time time);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointCloud_bridge_node");
    ros::NodeHandle nh;

    // Create the input and output bag filenames
    std::string input_bag_filename = argv[1];
    std::string output_bag_filename = argv[2];

    // Open the input bag for reading
    rosbag::Bag input_bag;
    input_bag.open(input_bag_filename, rosbag::bagmode::Read);

    // Create the output bag for writing
    rosbag::Bag output_bag;
    output_bag.open(output_bag_filename, rosbag::bagmode::Write);

    // Get the view of the input bag and iterate over the messages
    rosbag::View view(input_bag);
    for (const rosbag::MessageInstance &msg : view)
    {
        sensor_msgs::PointCloud2::ConstPtr point_cloud_msg = msg.instantiate<sensor_msgs::PointCloud2>();
        if (point_cloud_msg != nullptr)
        {
            // Convert intensity to float and write to the output bag
            convertIntensityToFloat(point_cloud_msg, output_bag, msg.getTime());
        }
        else
        {
            // If it's not a point cloud message, write it directly to the output bag
            output_bag.write(msg.getTopic(), msg.getTime(), msg, msg.getConnectionHeader());
        }
    }

    // Close the bags
    input_bag.close();
    output_bag.close();

    return 0;
}


void convertIntensityToFloat(const sensor_msgs::PointCloud2::ConstPtr &msg, rosbag::Bag &output_bag, const ros::Time time)
{
    // Create a new point cloud message with float intensity
    sensor_msgs::PointCloud2 new_msg;
    new_msg.header = msg->header;
    new_msg.height = msg->height;
    new_msg.width = msg->width;
    new_msg.is_bigendian = msg->is_bigendian;
    new_msg.point_step = msg->point_step;
    new_msg.row_step = msg->row_step;
    new_msg.is_dense = msg->is_dense;

    // Calculate the offset for the intensity field
    int intensity_offset = -1;
    for (int i = 0; i < msg->fields.size(); i++)
    {
        if (msg->fields[i].name == "intensity")
        {
            intensity_offset = msg->fields[i].offset;
            break;
        }
    }

    if (intensity_offset == -1)
    {
        ROS_WARN("Field 'intensity' not found in point cloud message!");
        return;
    }

    // Copy the data from the original message and convert the intensity values
    new_msg.data = msg->data;
    int float_offset = 0;

    for (int i = 0; i < msg->width * msg->height; i++)
    {
        uint8_t *intensity_data = &new_msg.data[float_offset + intensity_offset];
        uint8_t *ring_data_pre = &new_msg.data[float_offset + intensity_offset + 2];
        uint8_t *ring_data_cur = &new_msg.data[float_offset + intensity_offset + 4];

        uint16_t ring = *ring_data_pre;
        memcpy(ring_data_cur, &ring, sizeof(uint16_t));

        float float_intensity = static_cast<float>(*intensity_data);
        memcpy(intensity_data, &float_intensity, sizeof(float));
        
        
        float_offset += msg->point_step;
    }

    new_msg.fields = msg->fields;
    new_msg.fields[3].datatype = 7;
    new_msg.fields[4].offset = 20;

    // Write the converted message to the output bag
    output_bag.write("/velodyne_points", time, new_msg);
}