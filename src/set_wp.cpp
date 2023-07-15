// Contains all necessary functions for drone set-up 
#include <gnc_functions.hpp>

int main(int argc, char** argv)
{
    //initialize ros 
    ros::init(argc, argv, "gnc_node");
    ros::NodeHandle gnc_node("~");
    init_publisher_subscriber(gnc_node);
  
    //specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish
    ros::Rate rate(2.0);

    // Waitng for connection from FCU
    wait4connect();

    // Setting to mode guided
    set_mode("GUIDED");

    //create local reference frame 
    initialize_local_frame();

    // Take off attitude in meters 
    takeoff(10);


    // Setting the hoizontal speed in meters/sec
    set_speed(10);


    while(ros::ok())
    {   
        ros::spinOnce();
        rate.sleep();

        // Check for position tolerance 
        if(check_waypoint_reached(.3) == 0)
        {
            set_destination(0,30,10, 0);
        }
  
    }    
} 
