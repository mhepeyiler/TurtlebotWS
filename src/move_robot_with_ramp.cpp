#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "time.h"

#include <map>
#include <array>
#include <utility>
#include <string>
#include <cstdlib>
#include <cmath>


struct ScaleVars
{
    
    double linear_scale{0.1};
    double angular_scale{0.1};
    using ScaleVars_t = double;
};


void MakeTwistZero(geometry_msgs::Twist& tw)
{
    tw.linear.x = 0;
    tw.angular.z = 0;
}

template<typename T>
T RampedVelocity(T v_prev, T v_target, ros::Time t_prev, ros::Time t_now, ScaleVars::ScaleVars_t ramp_rate)
{
    auto step = ramp_rate * static_cast<ScaleVars::ScaleVars_t>((t_now - t_prev).toSec());
    auto sign = (v_target > v_prev) ? 1.0 : -1.0;
    auto error = fabs(v_target - v_prev);
    return error < step  ? v_target : (v_prev + sign * step);
}


class RobotControl
{
public:
    RobotControl(const ros::Publisher& pub, ros::Rate& rate, ScaleVars vel_scale, ScaleVars ramps_scale) : mpub{pub}, mrate{rate}, 
                                                                                                            mvel_scale{vel_scale}, mramps_scale{ramps_scale}  
    {
        MakeTwistZero(mtarget_twist);
        MakeTwistZero(mlast_twist);   
    }
    
    void KeyFB(const std_msgs::String::ConstPtr& msg)
    {
        mtarget_twist.angular.z = key_mapping.at(msg->data)[0] * mvel_scale.angular_scale;
        mtarget_twist.linear.x = key_mapping.at(msg->data)[1] * mvel_scale.linear_scale; 
    }

    void run()
    {
        RampedTwist();
        mpub.publish(mlast_twist);
        ros::spinOnce();
        mrate.sleep();
    }

private:

    const ros::Publisher& mpub;
    ros::Rate& mrate;
    geometry_msgs::Twist mtarget_twist, mlast_twist;
    ros::Time mlast_time;

    const ScaleVars mvel_scale, mramps_scale;
    
    using map_str_arr = std::map<std::string, std::array<double,2>>;
    using pair_str_arr = std::pair<std::string, std::array<double,2>>;

    const map_str_arr key_mapping{pair_str_arr{"w",{0,1}},
                                  pair_str_arr{"x",{0,-1}},
                                  pair_str_arr{"a",{-1,0}},
                                  pair_str_arr{"d",{1,0}},
                                  pair_str_arr{"s",{0,0}}};   

    void RampedTwist()
    {
        auto t_now = ros::Time::now();
        mlast_twist.angular.z = RampedVelocity(mlast_twist.angular.z, mtarget_twist.angular.z, mlast_time, t_now, mramps_scale.angular_scale); 
        mlast_twist.linear.x = RampedVelocity(mlast_twist.linear.x, mtarget_twist.linear.x, mlast_time, t_now, mramps_scale.angular_scale); 
        mlast_time = t_now;
    }


};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "control_robot");
    ros::NodeHandle n("~");
    const ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    ros::Rate sleep_rate(10);

    ScaleVars vel_scale, ramps_scale;
    
    if(n.hasParam("vel_linear"))
        n.getParam("vel_linear", vel_scale.linear_scale);
    else
        ROS_WARN("Linear scale variable has been set to 0.1");
    
    if(n.hasParam("vel_angular"))
        n.getParam("vel_angular", vel_scale.angular_scale);
    else
        ROS_WARN("Angular scale variable has been set to 0.1");


    if(n.hasParam("ramps_linear"))
        n.getParam("ramps_linear", ramps_scale.linear_scale);
    else
        ROS_WARN("Linear scale variable has been set to 0.1");
    
    if(n.hasParam("ramps_angular"))
        n.getParam("ramps_angular", ramps_scale.angular_scale);
    else
        ROS_WARN("Angular scale variable has been set to 0.1");


    RobotControl rob(pub, sleep_rate, vel_scale, ramps_scale);
    const ros::Subscriber sub = n.subscribe<std_msgs::String>("/key_stroke", 1000, &RobotControl::KeyFB, &rob);
    
    while(ros::ok())
    {
        rob.run();
    }
}