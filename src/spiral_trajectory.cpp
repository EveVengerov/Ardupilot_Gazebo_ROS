#include <gnc_functions.hpp>
//include API
#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/Waypoint.h>
#include <sensor_msgs/NavSatFix.h>

#define PI 3.14159265359

double current_lat = 0.0;
double current_lon = 0.0;

void globalPositionCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    current_lat = msg->latitude;
    current_lon = msg->longitude;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "gnc_node");
    ros::NodeHandle gnc_node("~");

    // Service clients for MAVROS commands
    ros::ServiceClient arming_client = gnc_node.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = gnc_node.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    ros::ServiceClient waypoint_push_client = gnc_node.serviceClient<mavros_msgs::WaypointPush>("/mavros/mission/push");
    ros::ServiceClient waypoint_clear_client = gnc_node.serviceClient<mavros_msgs::WaypointClear>("/mavros/mission/clear");
    ros::ServiceClient command_long_client = gnc_node.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
    
    //initialize control publisher/subscribers
    init_publisher_subscriber(gnc_node);

    // wait for FCU connection
    wait4connect();

    //wait for used to switch to mode GUIDED
    wait4start();

    double takeoff_alt = 20.0;
    takeoff(takeoff_alt);

    // Subscribe to global position updates
    ros::Subscriber global_position_sub = gnc_node.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 10, globalPositionCallback);

    // Wait for the initial latitude and longitude to be received
    while (ros::ok() && (current_lat == 0.0 || current_lon == 0.0)) {
        ROS_INFO("Waiting for current latitude and longitude...");
        ros::spinOnce();
        ros::Duration(0.5).sleep();
    }

    // Clear existing waypoints
    mavros_msgs::WaypointClear waypoint_clear;
    waypoint_clear_client.call(waypoint_clear);

    // Generate spiral waypoints
    double radius = 3.0;  // Radius of the spiral in meters
    double vertical_speed = 1.0;  // Vertical speed in meters/second
    double horizontal_speed = 3.0; // Horizontal speed in meters/second
    double angular_speed = 1.0;  // Angular speed of rotation in radians/second
    double max_height = 50.0;  // Maximum height of the spiral in meters

    std::vector<mavros_msgs::Waypoint> waypoints;
    mavros_msgs::Waypoint waypoint;
    waypoint.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
    waypoint.command = mavros_msgs::CommandCode::NAV_WAYPOINT;
    waypoint.autocontinue = true;

    double current_height = takeoff_alt;
    double current_time = 0.0;
    double current_position = 0.0;
    double max_position = 70.0;

    
    // // Spiral parallel to z axis
    // while (current_height < max_height) 
    // {
    //     mavros_msgs::Waypoint waypoint;
    //     waypoint.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
    //     waypoint.command = mavros_msgs::CommandCode::NAV_WAYPOINT;
    //     waypoint.is_current = false;
    //     waypoint.autocontinue = true;
    //     waypoint.x_lat = current_lat;
    //     waypoint.y_long = current_lon;
    //     waypoint.z_alt = current_height;
    //     waypoint.param1 = 0.0;
    //     waypoint.param2 = 0.0;
    //     waypoint.param3 = 0.0;
    //     waypoint.param4 = 0.0;

    //     // Generate setpoint coordinates for the spiral trajectory
    //     double x = radius * cos(angular_speed * current_time);
    //     double y = radius * sin(angular_speed * current_time);

    //     waypoint.x_lat += x / 111111.0;  // Convert x-coordinate to latitude
    //     waypoint.y_long += y / (111111.0 * cos(current_lat * PI / 180.0));  // Convert y-coordinate to longitude


    //     waypoints.push_back(waypoint);

    //     current_height += vertical_speed * 0.1;
    //     current_time += 0.1;  // Increase time
    // }


    // Spiral parallel to x axis (same for y axis)
    while (current_position < max_position) 
    {
        mavros_msgs::Waypoint waypoint;
        waypoint.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
        waypoint.command = mavros_msgs::CommandCode::NAV_WAYPOINT;
        waypoint.is_current = false;
        waypoint.autocontinue = true;
        waypoint.x_lat = current_lat;
        waypoint.y_long = current_lon;
        waypoint.z_alt = current_height;
        waypoint.param1 = 0.0;
        waypoint.param2 = 0.0;
        waypoint.param3 = 0.0;
        waypoint.param4 = 0.0;

        // Generate setpoint coordinates for the spiral trajectory
        double x = horizontal_speed*current_time;
        double y = radius * cos(angular_speed * current_time);
        double z = radius * sin(angular_speed * current_time);

        waypoint.x_lat += x / 111111.0;  // Convert x-coordinate to latitude
        waypoint.y_long += y / (111111.0 * cos(current_lat * PI / 180.0));  // Convert y-coordinate to longitude
        waypoint.z_alt += z;

        waypoints.push_back(waypoint);

        current_position+= horizontal_speed * 0.1;
        current_time += 0.1;  // Increase time
    }


    // Push waypoints to the vehicle
    mavros_msgs::WaypointPush waypoint_push;
    waypoint_push.request.waypoints = waypoints;
    waypoint_push_client.call(waypoint_push);


    //wait for user to switch to mode AUTO
    wait4start_auto();

    return 0;
}