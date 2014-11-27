#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include <rosutil/rosutil.hpp>
#include <math.h>
#include <sstream>
#include <iostream>



class DifferentialModel {
public:
    DifferentialModel(int argc, char *argv[]);

private:
    float robotBase;  //Base (Track width)
    float wheelRadius; //Wheel radius
    int ticks_per_rev;
    float control_frequency; //Hz
    float control_time; //Ts
    
  
    /*
    ros::Publisher chatter_pub; 
    ros::Subscriber sub_sensor_feedback;
    */
  
    //functions
    void runNodeDifferentialModel();
};
