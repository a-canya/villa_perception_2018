#include <ros/ros.h>

#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// PCL specific includes
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>


#include "villa_surface_detectors/DetectHorizontalPlanes.h"
#include "villa_surface_detectors/DetectTable.h"

#include <tf/transform_listener.h>
#include "pcl_angle_filter.h"

#define TABLE_MARKER_NAMESPACE "table_marker"

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

using namespace std;

ros::ServiceClient horizontal_plane_detector_client;
ros::ServiceServer table_detect_serv;

ros::Publisher detected_table_marker_pub;
ros::Publisher table_plane_points_pub;

#define VISUALIZE false

// This service returns the most likely location of a cupboard based on number of overlapping bounding boxes of planes in the x-y plane
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

bool compare_cloud_size(const sensor_msgs::PointCloud2 &lhs, const sensor_msgs::PointCloud2 &rhs) {
    return lhs.data.size() < rhs.data.size();
}

bool compare_by_second(const pair<int, float> &lhs, const pair<int, float> &rhs) {
    return lhs.second < rhs.second;
}

bool find_table(villa_surface_detectors::DetectTable::Request &req, villa_surface_detectors::DetectTable::Response &res){
	ROS_INFO("Extracting Table(s) from given parameters...");
	// Find Horizontal Planes from the given point cloud
    // Ideally this cloud is pass-through filtered so that we only consider points where we expect a table to be
    sensor_msgs::PointCloud2 &input_cloud = req.cloud_input;
    float start_angle = req.ignore_start_angle.data;
    float end_angle = req.ignore_end_angle.data;
    if (start_angle != end_angle) {
        PointCloudT::Ptr converted_cloud (new PointCloudT);
        converted_cloud->header.frame_id = input_cloud.header.frame_id;
        pcl::fromROSMsg(input_cloud, *converted_cloud);

        PointCloudT::Ptr filtered_cloud (new PointCloudT);
        filtered_cloud->header.frame_id = input_cloud.header.frame_id;
        Eigen::Vector2f origin(req.ignore_origin.x, req.ignore_origin.y);
        filter_points_between(converted_cloud, filtered_cloud, origin, req.ignore_start_angle.data, req.ignore_end_angle.data);
        pcl::toROSMsg(*filtered_cloud, input_cloud);
    }

	// Prepare service request
	villa_surface_detectors::DetectHorizontalPlanes srv;
	srv.request.cloud_input = input_cloud;

	ROS_INFO("Calling horizontal_plane_detector_server");
	// Perform Service Call
	if (horizontal_plane_detector_client.call(srv)){
		ROS_INFO("Call was successful!");
		if (!srv.response.horizontal_planes.empty()){
			ROS_INFO("Detected %zu planes", srv.response.horizontal_planes.size());
		}else{
			ROS_INFO("No Planes Detected. Table cannot be extracted");
			return false;
		}

	}else{
		ROS_ERROR("Failed to call service detect_horizontal_planes");
		return false;
	}

    // Sort by the surface area (XY) of the planes
    // Keep the indices around so we can get the other metadata we need
    vector< pair<int, float> > index_with_size;
    for (int i = 0; i < srv.response.horizontal_plane_bounding_boxes.size(); i++) {
        const visualization_msgs::Marker &box = srv.response.horizontal_plane_bounding_boxes[i];
        index_with_size.push_back(pair<int, float>(i, box.scale.x * box.scale.y));
    }

    const pair<int, float> largest = *std::max_element(index_with_size.begin(), index_with_size.end(), compare_by_second);
    const int largest_index = largest.first;
    const visualization_msgs::Marker &largest_marker = srv.response.horizontal_plane_bounding_boxes[largest_index];

    ROS_INFO("Largest Plane has index %i and has size of %f", largest_index, (float)largest.first);

    if (VISUALIZE) {
        // Publish Marker and Table Plane Points
        visualization_msgs::Marker table_plane_marker;
        table_plane_marker.ns = TABLE_MARKER_NAMESPACE;
        table_plane_marker = largest_marker;

        ROS_INFO("Publishing table plane marker and points");
        detected_table_marker_pub.publish(table_plane_marker);
        table_plane_points_pub.publish(srv.response.horizontal_planes[largest_index]);
    }

	// Return service message
    ROS_INFO("Returning Table Extraction Response");
    res.table_points = srv.response.horizontal_planes[largest_index];
    res.table_plane_coefs = srv.response.horizontal_plane_coefs[largest_index];
    res.table_bounding_box = largest_marker;

	return true;
}


int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "table_detector_server_node");

	ros::NodeHandle nh;
	ROS_INFO("table_detector_service_node started");

	horizontal_plane_detector_client = nh.serviceClient<villa_surface_detectors::DetectHorizontalPlanes>("detect_horizontal_planes");
	table_detect_serv = nh.advertiseService("detect_table", find_table);

    // Topics that show information on the internals of this node
    if (VISUALIZE) {
        ros::NodeHandle pnh("~");
        detected_table_marker_pub = pnh.advertise<visualization_msgs::Marker>("table/marker", 1, true);
        table_plane_points_pub = pnh.advertise<sensor_msgs::PointCloud2>("table/points", 1, true);
    }
	ros::spin();
}
