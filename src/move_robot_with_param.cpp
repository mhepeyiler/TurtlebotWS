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
    RobotControl(const ros::Publisher& pub, const ros::Rate& rate) : mpub{pub}, mrate{rate} 
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

    void run()const
    {
        ros::spinOnce();
        mrate.sleep();
    }

private:

    const ros::Publisher& mpub;
    const ros::Rate& mrate;
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
    const ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    ros::Rate sleep_rate(10);
    RobotControl rob(pub, sleep_rate);
    const ros::Subscriber sub = n.subscribe<std_msgs::String>("/key_stroke", 1000, &RobotControl::KeyFB, &rob);
    
    while(ros::ok())
    {
        rob.run();
    }
}