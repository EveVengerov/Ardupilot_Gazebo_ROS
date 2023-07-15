// #include <gnc_functions.hpp>
// #include <ros/ros.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <mavros_msgs/CommandBool.h>
// #include <mavros_msgs/SetMode.h>
// #include <mavros_msgs/State.h>
// #include <sensor_msgs/Image.h>
// #include <sensor_msgs/CameraInfo.h>
// #include <cv_bridge/cv_bridge.h>
// #include <opencv2/opencv.hpp>
// #include <opencv2/aruco.hpp>

// cv::Ptr<cv::aruco::Dictionary> dictionary;
// cv::Mat cameraMatrix, distCoeffs;
// bool markerDetected = false;
// double markerSize = 2; // Size of the marker in meters

// void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg) {
//     cameraMatrix = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->K.data())).clone();
//     distCoeffs = cv::Mat(1, 5, CV_64F, const_cast<double*>(msg->D.data())).clone();
// }

// void imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
//     ROS_INFO("Marker Detecting...");
//     if (!markerDetected) {
//         try {
//             cv_bridge::CvImagePtr cvImage = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//             cv::Mat frame = cvImage->image;

//             cv::imshow("ArUco Marker Detection", frame);
//             cv::waitKey(1);

//             std::vector<int> markerIds;
//             std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
//             cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();


//             // Detect markers
//             cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

//             // Draw marker outlines and IDs
//             if (markerIds.size() > 0) {
//                 ROS_INFO("Marker Detected");
//                 cv::aruco::drawDetectedMarkers(frame, markerCorners, markerIds);
//                 markerDetected = true;

//                 // Get the center of the first detected marker
//                 cv::Point2f markerCenter = (markerCorners[0][0] + markerCorners[0][1] + markerCorners[0][2] + markerCorners[0][3]) * 0.25;

//                 // Calculate the 3D pose of the marker
//                 cv::Vec3d rvec, tvec;
//                 cv::aruco::estimatePoseSingleMarkers(markerCorners, markerSize, cameraMatrix, distCoeffs, rvec, tvec);

//                 // Set the desired pose for precision landing
//                 geometry_msgs::PoseStamped pose;
//                 pose.header.stamp = ros::Time::now();
//                 pose.header.frame_id = "base_link";
//                 pose.pose.position.x = tvec[0];
//                 pose.pose.position.y = tvec[1];
//                 pose.pose.position.z = tvec[2];
//                 pose.pose.orientation.x = rvec[0];
//                 pose.pose.orientation.y = rvec[1];
//                 pose.pose.orientation.z = rvec[2];
//                 pose.pose.orientation.w = 1.0; // Assuming identity orientation for simplicity

//                 // Publish the desired pose
//                 ros::NodeHandle nh;
//                 ros::Publisher posePublisher = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
//                 posePublisher.publish(pose);

//                 // Land the drone if the marker is close enough
//                 if (tvec[2] < 0.2) 
//                 {
//                     land();
//                 }
//             }

//         } catch (cv_bridge::Exception& e) {
//             ROS_ERROR("cv_bridge exception: %s", e.what());
//         }
//     }

//     else 
//     {
//         ROS_INFO("Marker Not Detected");
//     }   
// }

// int main(int argc, char** argv) {
//     //initialize ros 
//     ros::init(argc, argv, "gnc_node");
//     ros::NodeHandle gnc_node("~");
    
//     //initialize control publisher/subscribers
//     init_publisher_subscriber(gnc_node);

//     dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);


//     wait4connect();
//     wait4start();
//     takeoff(5); 

//     ros::Subscriber cameraInfoSub = gnc_node.subscribe<sensor_msgs::CameraInfo>("webcam/camera_info", 10, cameraInfoCallback);
//     ros::Subscriber imageSub = gnc_node.subscribe<sensor_msgs::Image>("webcam/image_raw", 10, imageCallback);
//     ROS_INFO("Subscribing to webcam");
//     ros::Rate rate(2); // Set the loop rate in Hz
    


//     while (ros::ok()) {
//         ros::spinOnce();
//         rate.sleep();
//     }

//     return 0;
// }


// Include required opencv and mat libraries for aruco detection
#include <gnc_functions.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core/mat.hpp>


//specify waypoint
std::vector<gnc_api_waypoint> waypointList;
gnc_api_waypoint nextWayPoint;

cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
cv::Mat cameraMatrix, distCoeffs;
bool markerDetected = false;
double markerSize = 0.5; // Size of the marker in meters
bool isCameraInfo = false;

void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg) {
    ROS_INFO("CameraInfo");
    cameraMatrix = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->K.data())).clone();
    distCoeffs = cv::Mat(1, 5, CV_64F, const_cast<double*>(msg->D.data())).clone();
    isCameraInfo = true;
}

void imageCallback(const sensor_msgs::Image::ConstPtr& msg) 
{
    
    if(!markerDetected)
    {   ROS_INFO("Detecting Marker");
        try 
        {
            cv_bridge::CvImagePtr cvImage = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

            if (cvImage->image.empty()) 
            {
            ROS_ERROR("Empty image received");
            return;
            }

            cv::Mat frame = cvImage->image;     
            cv::imshow("Camera Feed", frame);
            cv::waitKey(1);

            std::vector<int> markerIds;
            std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
            cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();



            // Detect markers
            cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

            // Draw marker outlines and IDs
            if (markerIds.size() > 0) 
            {
                ROS_INFO("Marker Detected");
                cv::aruco::drawDetectedMarkers(frame, markerCorners, markerIds);
                ROS_INFO("Marker Drawn");

                // Get the center of the first detected marker
                cv::Point2f markerCenter = (markerCorners[0][0] + markerCorners[0][1] + markerCorners[0][2] + markerCorners[0][3]) * 0.25;
                ROS_INFO("Marker Center");


                if(isCameraInfo){
                markerDetected = true;
                // Calculate the 3D pose of the marker
                std::vector<cv::Vec3d> rvecs; std::vector<cv::Vec3d> tvecs; 
                cv::aruco::estimatePoseSingleMarkers(markerCorners, markerSize, cameraMatrix, distCoeffs, rvecs, tvecs);

                ROS_INFO("Pose estimated");

                // Get the pose of the marker
                // cv::Vec3d tvec = (tvecs[0] + tvecs[1] + tvecs[2] + tvecs[3]) * 0.25;

                cv::Vec3d rvec = rvecs[0];
                cv::Vec3d tvec = tvecs[0];
                // Set the desired pose for precision landing

                float rx = -rvec[1];
                float ry = -rvec[0];
                float rz = -rvec[2];
                // receive yaw angle by converting the rotation vector
                nextWayPoint.psi =  rotationVector2eulerAngles(rx,ry,rz) * 180/3.14;    


                // Save desired waypoint, note: the camera's coordinate frae is different from the drone's  coordinate frame
                nextWayPoint.x = tvec[1];
                nextWayPoint.y = -tvec[0];
                nextWayPoint.z = -tvec[2];


                ROS_INFO("Waypoint send");
                ROS_INFO("Aruco Marker pose wrt Camera : %f y: %f z: %f psi: %f", nextWayPoint.x, nextWayPoint
                    .y, nextWayPoint.z, nextWayPoint.psi);

                // Converting the pose of the marker from the camera frame to local reference frame
                set_destination_camera2local_frame(nextWayPoint.x, nextWayPoint.y, nextWayPoint.z, nextWayPoint.psi);

                // if the marker is close enough on detection, directly land
                if (tvec[2]<0.2)
                {
                    land();
                }
            
            }
            }
        } 
        catch (cv_bridge::Exception& e) 
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }
}

int main(int argc, char** argv) {
    // Initialise ros node
    ros::init(argc, argv, "gnc_node");
    ros::NodeHandle gnc_node("~");

    init_publisher_subscriber(gnc_node);


    // Waiting for FCU to get a connection
    wait4connect();
    // Waiting for guided mode
    set_mode("GUIDED");

    //create local reference frame 
    initialize_local_frame();

    // take off to 3 meters  
    takeoff(3); 
    set_speed(1);

    // recieve the  camera parameters information for calibration 
    ros::Subscriber cameraInfoSub = gnc_node.subscribe<sensor_msgs::CameraInfo>("/webcam/camera_info", 10, cameraInfoCallback);
    // receive the frames as a bgr image
    ros::Subscriber imageSub = gnc_node.subscribe<sensor_msgs::Image>("/webcam/image_raw", 1, imageCallback);
    ROS_INFO("Subscribing to webcam");
    

    ros::Rate rate(2.0);

    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();  
    }

    cv::destroyAllWindows();

    return 0;
}