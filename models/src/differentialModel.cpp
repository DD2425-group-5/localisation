#include "differentialModel.hpp"

DifferentialModel::DifferentialModel(int argc, char *argv[]){
    ros::init(argc, argv, "differentail_model");
    ros::NodeHandle n;
    
    //get params
    ROSUtil::getParam(n, "/robot_info/ticks_per_rev", ticks_per_rev);
    ROSUtil::getParam(n, "/robot_info/wheel_baseline", robotBase);
    ROSUtil::getParam(n, "/robot_info/wheel_radius", wheelRadius);
    
    //while (ros::ok()){
        runNodeDifferentialModel();
        
    //}
}



void DifferentialModel::runNodeDifferentialModel(){
    double xNew = 0.0;
    double yNew= 0.0;
    double thetaNew = 0.0;
    double xOld = 0.0;
    double yOld= 0.0;
    double thetaOld = M_PI/2;
    double timeStep = 0.1;
    double velocityR = 0.2;
    double velocityL = 0.2;
    
    
    if(velocityR-velocityL < 0.0001){
        xNew = xOld + timeStep*cos(thetaOld)*(velocityR+velocityL)/2;
        yNew = yOld + timeStep*sin(thetaOld)*(velocityR+velocityL)/2;
        thetaNew = thetaOld;
    }
    else{  
        xNew = xOld+\
        robotBase*((velocityR+velocityL)/(2.0*(velocityR-velocityL)))*\
        ( sin( (velocityR-velocityL)*timeStep/robotBase + thetaOld) -\
          sin( thetaOld) );
         
        yNew = yOld-\
        robotBase*((velocityR+velocityL)/(2.0*(velocityR-velocityL)))*\
        ( cos( (velocityR-velocityL)*timeStep/robotBase + thetaOld) -\
          cos( thetaOld) );
        
        thetaNew = thetaOld+((velocityR-velocityL)*timeStep)/robotBase;
    }
    ROS_INFO("\n\n\n\n\n\n\n\n\n\nIf you start @\nX=%f \nY=%f \nTheta=%f \nyou end up @\nX=%f \nY=%f \nTheta=%f\n\n",\
    xOld, yOld, thetaOld, xNew, yNew, thetaNew);
}



int main(int argc, char **argv)
{
    DifferentialModel mainDifferentialModel(argc, argv);
}

