#include <gnc_functions.hpp>
//include API 

int main(int argc, char** argv)
{
	//initialize ros 
	ros::init(argc, argv, "gnc_node");
	ros::NodeHandle gnc_node("~");
	
	//initialize control publisher/subscribers
	init_publisher_subscriber(gnc_node);

  	// wait for FCU connection
	wait4connect();

	//wait for used to switch to mode GUIDED
	wait4start();
	

	//create local reference frame 
	initialize_local_frame();

	set_speed(7);

	// TakeOff
	int takeoff_alt = 10;
	takeoff(takeoff_alt);



	//specify some waypoints 
	std::vector<gnc_api_waypoint> waypointList;
	gnc_api_waypoint nextWayPoint;


	int num_samples = 10;
	float pi = 3.14;
	int vf = 30;
	double r = 3.0;
	int n  = 5;

	// start circle 
	nextWayPoint.x = 0;
	nextWayPoint.y = r;
	nextWayPoint.z = takeoff_alt;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);


	for(float t = 0.0; t<=1.0; t = t + 1.0/num_samples)
	{
		nextWayPoint.x = r * sin(2*pi*n*t);
		nextWayPoint.y = vf*t;
		nextWayPoint.z = takeoff_alt + r * cos(2*pi*n*t);
		nextWayPoint.psi = 0;
		// nextWayPoint.psi = -360*t;
		waypointList.push_back(nextWayPoint);
	}



	// return to origin
	nextWayPoint.x = 0;
	nextWayPoint.y = 0;
	nextWayPoint.z = takeoff_alt;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);


	//specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish
	ros::Rate rate(4.0);
	int counter = 0;
	while(ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
		if(check_waypoint_reached(.3) == 1)
		{
			if (counter < waypointList.size())
			{
				set_destination(waypointList[counter].x,waypointList[counter].y,waypointList[counter].z, waypointList[counter].psi);
				counter++;	
			}else{
				//land after all waypoints are reached
				land();
			}	
		}	
		
	}
	return 0;
}