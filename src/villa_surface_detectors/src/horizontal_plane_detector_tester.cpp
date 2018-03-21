#include <signal.h>
#include <vector>
#include <Eigen/Dense>
#include <string>
#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <pcl_ros/impl/transforms.hpp>

// PCL specific includes

#include <pcl/console/parse.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/common/time.h>
#include <pcl/common/common.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/kdtree/kdtree.h>
#include "villa_surface_detectors/DetectHorizontalPlanes.h"

#include <tf/transform_listener.h>
#include <tf/tf.h>


/* define what kind of point clouds we're using */
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

ros::ServiceClient horizontal_plane_detector_client;

// Mutex: //
//boost::mutex cloud_mutex;
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
	//cloud_mutex.lock ();
	cloud_in = (*input);
	ROS_INFO("Received cloud.");
	//cloud_mutex.unlock ();
	ROS_INFO("Pointcloud is ready.");

	// Prepare service request
	villa_surface_detectors::DetectHorizontalPlanes srv;
	srv.request.cloud_input = cloud_in;

	pressEnter("Press enter");
	// Perform Service Call
	if (horizontal_plane_detector_client.call(srv)){
		ROS_INFO("Call was successful!\n");
	}else{
		ROS_ERROR("Failed to call service horizontal_plane_detector_server");
		ROS_INFO("Attempting service call again when cloud becomes available");
	}

}

int main(int argc, char **argv) {
	// Intialize ROS with this node name
	ros::init(argc, argv, "horizontal_plane_server_tester_node");
	ros::NodeHandle n;

	horizontal_plane_detector_client = n.serviceClient<villa_surface_detectors::DetectHorizontalPlanes>("detect_horizontal_planes");
	villa_surface_detectors::DetectHorizontalPlanes srv; 

	std::string param_topic = "/hsrb/head_rgbd_sensor/depth_registered/rectified_points";
	//std::string param_topic = "/octomap_point_cloud_centers";
	ros::Subscriber sub = n.subscribe (param_topic, 10, cloud_cb);

	ros::spin();

	return 0;
}
