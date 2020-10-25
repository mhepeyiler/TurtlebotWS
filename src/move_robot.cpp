#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"

#include <map>
#include <array>
#include <utility>
#include <string>


class RobotControl
{
public:
    RobotControl(const ros::Publisher& pub) : mpub{pub} 
    {
        mtwist.linear.x = 0;
        mtwist.angular.z = 0;
    }
    
    void KeyFB(const std_msgs::String::ConstPtr& msg)
    {
        mtwist.angular.z = key_mapping.at(msg->data)[0];
        mtwist.linear.x = key_mapping.at(msg->data)[1];
        mpub.publish(mtwist);
    }

private:

    const ros::Publisher& mpub;
    geometry_msgs::Twist mtwist;

    using map_str_arr = std::map<std::string, std::array<double,2>>;
    using pair_str_arr = std::pair<std::string, std::array<double,2>>;

    const map_str_arr key_mapping{pair_str_arr{"w",{0,1}},
                                  pair_str_arr{"x",{0,-1}},
                                  pair_str_arr{"a",{-1,0}},
                                  pair_str_arr{"d",{1,0}},
                                  pair_str_arr{"s",{0,0}}};   
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "control_robot");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    RobotControl rob(pub);
    ros::Subscriber sub = n.subscribe<std_msgs::String>("/key_stroke", 1000, &RobotControl::KeyFB, &rob);
    ros::spin();
}