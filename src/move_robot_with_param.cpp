#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"

#include <map>
#include <array>
#include <utility>
#include <string>



struct ScaleVars
{
    double linear_scale{0.1};
    double angular_scale{0.1};
};

class RobotControl
{
public:
    RobotControl(const ros::Publisher& pub, ros::Rate& rate, ScaleVars var) : mpub{pub}, mrate{rate}, mscale{var} 
    {
        mtwist.linear.x = 0;
        mtwist.angular.z = 0;
    }
    
    void KeyFB(const std_msgs::String::ConstPtr& msg)
    {
        mtwist.angular.z = key_mapping.at(msg->data)[0] * mscale.angular_scale;
        mtwist.linear.x = key_mapping.at(msg->data)[1] * mscale.linear_scale; 
        mpub.publish(mtwist);
    }

    void run()const
    {
        ros::spinOnce();
        mrate.sleep();
    }

private:

    const ros::Publisher& mpub;
    ros::Rate& mrate;
    geometry_msgs::Twist mtwist;
    const ScaleVars mscale;

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
    ros::NodeHandle n("~");
    const ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    ros::Rate sleep_rate(10);
    ScaleVars vars;
    
    if(n.hasParam("linear"))
        n.getParam("linear", vars.linear_scale);
    else
        ROS_WARN("Linear scale variable has been set to 0.1");
    
    if(n.hasParam("angular"))
        n.getParam("angular", vars.angular_scale);
    else
        ROS_WARN("Angular scale variable has been set to 0.1");

    ROS_INFO("%.2f %.2f", vars.angular_scale, vars.linear_scale);

    RobotControl rob(pub, sleep_rate, vars);
    const ros::Subscriber sub = n.subscribe<std_msgs::String>("/key_stroke", 1000, &RobotControl::KeyFB, &rob);
    
    while(ros::ok())
    {
        rob.run();
    }
}