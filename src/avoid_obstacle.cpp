#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "laser_geometry/laser_geometry.h"
#include "sensor_msgs/PointCloud.h"


#include <string>
#include <iostream>
#include <vector>
#include <functional>
#include <algorithm>
#include <numeric>


std::vector<float> dist;
bool flag = false;
float angle_inc{};
/*
    Debug purpose 
*/
std::ostream& operator<<(std::ostream& os, const std::vector<float>& data)
{
    os << "[";
    for(const auto& val : data)
    {
        os << val << ' ';
    }
    os << "]\n";
    return os;
}


void SensorFB(const sensor_msgs::LaserScan& msg)
{  
    flag = true;
    angle_inc = msg.angle_increment;
    dist.resize(msg.ranges.size());
    transform(msg.ranges.cbegin(), msg.ranges.cend(), dist.begin(), [&msg](const auto& val) {return std::isinf(val) ? msg.range_max : val;});
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "avoid_obstacle");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

    ros::Subscriber sub = n.subscribe("/scan", 1000, SensorFB);
    
    ros::Rate loop_rate(10);

    geometry_msgs::Twist left;
    left.angular.z = 0.1;
    left.linear.x = 0.1;

    geometry_msgs::Twist right;
    right.angular.z = -0.1;
    right.linear.x = 0.1;

    geometry_msgs::Twist forward;
    forward.linear.x = 0.1; 
    forward.angular.z = 0;
    
    
    std::vector<float> min_mean_dist_vec(3,0);
    
    while (ros::ok())
    {
        if(flag){ // waits until the first LaserScan data subscription
            size_t max_idx = M_PI/angle_inc + 1;
            for(size_t i = 0; i<3; ++i) 
            {
                min_mean_dist_vec[i] = std::accumulate(dist.cbegin() + static_cast<size_t>(i*(max_idx/3)), dist.cbegin() + static_cast<size_t>((i+1)*(max_idx/3)), 0);
            }   
        }
        
        switch (std::distance(min_mean_dist_vec.cbegin(), std::max_element(min_mean_dist_vec.cbegin(), min_mean_dist_vec.cend())))
        {
        case 0:
            pub.publish(forward);
            break;
        
        case 1:
            pub.publish(left);
            break;

        case 2:
            pub.publish(right);
            break;

        default:
            break;
        } 
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}