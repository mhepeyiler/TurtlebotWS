#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream> 
#include <cstdio>
#include <string>
#include <cstring>

int main(int argc, char** argv) { 

    
    ros::init(argc, argv, "key_stroke");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<std_msgs::String>("/key_stroke", 1000);

    ros::Rate loop_rate(10);

    while(ros::ok()){
         
        // Set terminal to raw mode 
        system("stty raw"); 
        char input = getchar(); 
        system("stty cooked");

        if(input == 'i')
            return 0;
            
        std_msgs::String msg;
        msg.data = input;
        pub.publish(msg);
        
        ros::spinOnce();
        loop_rate.sleep();  
    }
    return 0; 
}