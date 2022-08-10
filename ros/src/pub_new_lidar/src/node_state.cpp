#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

#include <iostream>

#include <Eigen/Dense>

#include <math.h>

using namespace std;

float intensity_0 = 1;

sensor_msgs::PointCloud2 lidar_msg;

class Syn{
public:
    Syn() {}
    ~Syn() {}

    void initSyn(ros::NodeHandle& nh);

    

    typedef std::shared_ptr<Syn> Ptr;
private:
    void lidarCallback(sensor_msgs::PointCloud2 msg_);
    void newLidarCallback(const ros::TimerEvent& /*event*/);

    float distance(float x, float y, float z)
    {
        return  sqrt(x*x + y*y + z*z);
    }

    inline bool convertPointCloudToPointCloud2x(const sensor_msgs::PointCloud &input, sensor_msgs::PointCloud2 &output);
    ros::NodeHandle node;
    ros::Subscriber lidar_data;
    ros::Publisher new_lidar_pub_;
    ros::Timer state_timer_;
};


inline bool Syn::convertPointCloudToPointCloud2x(const sensor_msgs::PointCloud &input, sensor_msgs::PointCloud2 &output){
    output.header = input.header;
    output.width  = input.points.size ();
    output.height = 1;
    output.fields.resize (4 + input.channels.size ());
    // Convert x/y/z to fields
    output.fields[0].name = "x"; 
    output.fields[1].name = "y"; 
    output.fields[2].name = "z";
    output.fields[3].name = "intensity";
    int offset = 0;
    // All offsets are *4, as all field data types are float32
    for (size_t d = 0; d < output.fields.size ()-1; ++d, offset += 4)
    {
      output.fields[d].offset = offset;
      output.fields[d].datatype = sensor_msgs::PointField::FLOAT32;
      output.fields[d].count  = 1;
    }

    output.fields[3].offset = 16;
    output.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
    output.fields[3].count  = 1;

    output.point_step = 32;
    output.row_step   = output.point_step * output.width;
    // Convert the remaining of the channels to fields
    for (size_t d = 0; d < input.channels.size (); ++d)
      output.fields[3 + d].name = input.channels[d].name;
    output.data.resize (input.points.size () * output.point_step);
    output.is_bigendian = false;  // @todo ?
    output.is_dense     = false; 
    // Copy the data points
    for (size_t cp = 0; cp < input.points.size (); ++cp)
    {
      memcpy (&output.data[cp * output.point_step + output.fields[0].offset], &input.points[cp].x, sizeof (float));
      memcpy (&output.data[cp * output.point_step + output.fields[1].offset], &input.points[cp].y, sizeof (float));
      memcpy (&output.data[cp * output.point_step + output.fields[2].offset], &input.points[cp].z, sizeof (float));
      float it = intensity_0 * exp((-0.004)*(distance(input.points[cp].x,input.points[cp].y, input.points[cp].z)));
      memcpy (&output.data[cp * output.point_step + output.fields[3].offset], &it, sizeof (float));
      for (size_t d = 0; d < input.channels.size (); ++d)
      {
        if (input.channels[d].values.size() == input.points.size())
        {
          memcpy (&output.data[cp * output.point_step + output.fields[3 + d].offset], &input.channels[d].values[cp], sizeof (float));
        }
      }
    }
    return (true);
}

void Syn::initSyn(ros::NodeHandle& nh){
    node = nh;

    lidar_data = node.subscribe<sensor_msgs::PointCloud2>("/airsim_node/PX4/lidar/Lidar", 10, &Syn::lidarCallback, this);

    new_lidar_pub_ = node.advertise<sensor_msgs::PointCloud2>("/newLidar", 10);

    state_timer_ = node.createTimer(ros::Duration(0.05), &Syn::newLidarCallback, this);
}

void Syn::lidarCallback(sensor_msgs::PointCloud2 msg_) {
    std::cout << "lidar: " << msg_.header.stamp << std::endl;

    //Convert PointClound2 to PointCloud: get x y z
    sensor_msgs::PointCloud output;
    sensor_msgs::convertPointCloud2ToPointCloud(msg_, output);
    cout << " msg_.channels.size() = " <<  output.channels.size() << endl;
    cout << "x = " << output.points[0].x << endl;
    cout << "y = " << output.points[0].y << endl;
    cout << "z = " << output.points[0].z << endl;

    //list_intensity;
    
    // for(uint8_t i=0; i< output.points.size(); i++)
    // {
    //     list_intensity.push_back(intensity_0 * exp((-0.004)*(distance(output.points[i].x,output.points[i].y, output.points[i].z))));
    // }

    //sensor_msgs::convertPointCloudToPointCloud2(output, lidar_msg);
    convertPointCloudToPointCloud2x(output, lidar_msg);

    // lidar_msg.header.stamp = msg_.header.stamp;
    // lidar_msg.header.frame_id = msg_.header.frame_id;

    // if (msg_.data.size() > 3) {
    //     lidar_msg.height = 1;
    //     lidar_msg.width = msg_.data.size() / 3;

    //     lidar_msg.fields.resize(8);
    //     lidar_msg.fields[0].name = "x";
    //     lidar_msg.fields[1].name = "y";
    //     lidar_msg.fields[2].name = "z";
    //     lidar_msg.fields[4].name = "intensity";

    //     int offset = 0;

    //     for (size_t d = 0; d < lidar_msg.fields.size(); ++d, offset += 4) {
    //         lidar_msg.fields[d].offset = offset;
    //         lidar_msg.fields[d].datatype = sensor_msgs::PointField::FLOAT32;
    //         lidar_msg.fields[d].count = 1;
    //     }

    //     lidar_msg.is_bigendian = false;
    //     lidar_msg.point_step = offset; // 4 * num fields
    //     lidar_msg.row_step = lidar_msg.point_step * lidar_msg.width;

    //     lidar_msg.is_dense = true; // todo

    //     for(uint8_t i=0; i<msg_.data.size(); i++)
    //     {
    //         for(int j=0; j<4; j++)
    //         {

    //         }
    //         if(i%12 == 0)
    //         {
    //             for(int j=0; j<4; j++)
    //             {
    //                 msg_.data.insert((msg_.data.begin()+i+j, 0.0));
    //             }
    //         } 
    //     }

    //     int count = 0;

    //     for(uint8_t i=0; i<msg_.data.size(); i++)
    //     {
    //         if(i%16 == 0)
    //         {
    //             for(int j=0; j<4; j++)
    //             {
    //                 msg_.data.insert((msg_.data.begin()+i+j, list_intensity.at(count)));
    //                 count++;
    //             }
    //         } 
    //     }

    //     for(uint8_t i=0; i<msg_.data.size(); i++)
    //     {
    //         if(i%20 == 0)
    //         {
    //             for(int j=0; j<12; j++)
    //             {
    //                 msg_.data.insert((msg_.data.begin()+i+j, 0));
    //             }
    //         }
    //     }

    //     const unsigned char* bytes = reinterpret_cast<const unsigned char*>(data_std.data());
    //     vector<unsigned char> lidar_msg_data(bytes, bytes + sizeof(float) * data_std.size());  //luu tru mang phan tu sang vector vua tao
    //     lidar_msg.data = std::move(lidar_msg_data);   //lidar_msg.data: Actual point data, size is (row_step*height)

    //     if (isENU_) {
    //         try {
    //             sensor_msgs::PointCloud2 lidar_msg_enu;
    //             auto transformStampedENU = tf_buffer_.lookupTransform(AIRSIM_FRAME_ID, vehicle_name, ros::Time(0), ros::Duration(1));
    //             tf2::doTransform(lidar_msg, lidar_msg_enu, transformStampedENU);

    //             lidar_msg_enu.header.stamp = lidar_msg.header.stamp;
    //             lidar_msg_enu.header.frame_id = lidar_msg.header.frame_id;

    //             lidar_msg = std::move(lidar_msg_enu);
    //         }
    //         catch (tf2::TransformException& ex) {
    //             ROS_WARN("%s", ex.what());
    //             ros::Duration(1.0).sleep();
    //         }
    //     }
    // }
    // else {
    //     // msg = []
    // }
}

void Syn::newLidarCallback(const ros::TimerEvent& /*event*/){
    
    sensor_msgs::PointCloud2 lidar_msg_new;
    lidar_msg_new = lidar_msg;
    new_lidar_pub_.publish(lidar_msg_new);
    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "node_newLidar");
    ros::NodeHandle node("~");

    //sensor_msgs::PointCloud2 input;
    
    
    std::cout << "Starting ...." << std::endl;
    Syn::Ptr syn_map;

    syn_map.reset(new Syn);
    syn_map->initSyn(node);

    ros::Duration(1.0).sleep();
    ros::spin();

    return 0;
}
