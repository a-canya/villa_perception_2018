#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "villa_surface_detectors/DetectTable.h"

ros::ServiceClient table_detector_client;

// Cloud for processing
sensor_msgs::PointCloud2 cloud_in;

// Blocking call for user input
void pressEnter(std::string message){
	std::cout << message;
	while (true){
		char c = std::cin.get();
		if (c == '\n')
			break;
		else if (c == 'q'){
			ros::shutdown();
			exit(1);
		}
		else {
			std::cout <<  message;
		}
	}
}


void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input){	
	// Store Cloud
	cloud_in = (*input);
	ROS_INFO("Pointcloud is ready.");	

	// Prepare service request
	villa_surface_detectors::DetectTable srv; 
	srv.request.cloud_input = cloud_in;

	pressEnter("Press enter");
	// Perform Service Call
	if (table_detector_client.call(srv)){
		ROS_INFO("Call was successful!\n");
	}else{
		ROS_ERROR("Failed to call service table_detector_service");
		ROS_INFO("Attempting service call again when cloud becomes available");
	}

}

int main(int argc, char **argv) {
	// Intialize ROS with this node name
	ros::init(argc, argv, "table_detector_tester_node");
	ros::NodeHandle n;

	std::string param_topic = "/hsrb/head_rgbd_sensor/depth_registered/rectified_points";	
//	std::string param_topic = "/octomap_point_cloud_centers";	
	ros::Subscriber sub = n.subscribe (param_topic, 10, cloud_cb);

	table_detector_client = n.serviceClient<villa_surface_detectors::DetectTable>("detect_table");

	ros::spin();

	return 0;
}
