#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <pcl_ros/impl/transforms.hpp>

// PCL specific includes
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/time.h>
#include <pcl/common/common.h>

#include "villa_surface_detectors/SegmentTableObjects.h"
#include "villa_surface_detectors/DetectTable.h"


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

using namespace std;

ros::ServiceServer table_object_segmentation_server;

ros::Publisher table_object_pub;
ros::Publisher table_object_boundingbox_pub;
ros::Publisher table_remainder;

tf::TransformListener *tf_listener;

#define TARGET_FRAME "map" //target frame name DONT CHANGE!
#define VOXEL_LEAF_SIZE 0.01 //size of voxel leaf for processing


#define VISUALIZE true
#define DEBUG_ENTER true // if true, you have to press enter to continue the process


// This service returns the most likely location of a cupboard based on number of overlapping bounding boxes of planes in the x-y plane
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


void pressEnter(string message){
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

bool compare_cloud_size(const sensor_msgs::PointCloud2 &lhs, const sensor_msgs::PointCloud2 &rhs) {
    return lhs.data.size() < rhs.data.size();
}

void move_to_frame(const PointCloudT::Ptr &input, const string &target_frame, PointCloudT::Ptr &output) {
    ROS_INFO("Transforming Input Point Cloud to %s frame...",  target_frame.c_str() );
    ROS_INFO("    Input Cloud Size: %zu", input->size());
    if (input->header.frame_id == target_frame) {
        output = input;
        return;
    }
    while (ros::ok()){
        tf::StampedTransform stamped_transform;
        try{
            // Look up transform
            tf_listener->lookupTransform(target_frame, input->header.frame_id, ros::Time(0), stamped_transform);
						ROS_INFO("Transfoming from (%s) to (%s)", input->header.frame_id.c_str(), target_frame.c_str());
            // Apply transform
            pcl_ros::transformPointCloud(*input, *output, stamped_transform);

            // Store Header Details
            output->header.frame_id = target_frame;
            pcl_conversions::toPCL(ros::Time::now(), output->header.stamp);

            break;
        }
            //keep trying until we get the transform
        catch (tf::TransformException &ex){
            ROS_ERROR_THROTTLE(1, "%s", ex.what());
            ROS_WARN_THROTTLE(1,"    Waiting for transform from cloud frame (%s) to %s frame. Trying again", input->header.frame_id.c_str(), target_frame.c_str());
            continue;
        }
    }
}

bool segment_objects(villa_surface_detectors::SegmentTableObjects::Request &req, villa_surface_detectors::SegmentTableObjects::Response &res){

  PointCloudT::Ptr world_cloud(new PointCloudT);
  pcl::fromROSMsg(req.world_points, *world_cloud);
  world_cloud->header.frame_id = req.world_points.header.frame_id;

  if (world_cloud->points.empty()){
		ROS_ERROR("The input cloud is empty. Cannot process service");
		return false;
	}

  PointCloudT::Ptr world_downsampled(new PointCloudT);
  pcl::VoxelGrid<PointT> vg;
	vg.setInputCloud (world_cloud);
	// TODO: Do we need as much resolution in XY as Z?
	vg.setLeafSize (VOXEL_LEAF_SIZE, VOXEL_LEAF_SIZE, VOXEL_LEAF_SIZE);
	vg.filter (*world_downsampled);
  world_downsampled->header.frame_id = req.world_points.header.frame_id;

  string target_frame (TARGET_FRAME);
  PointCloudT::Ptr map_cloud(new PointCloudT);
  move_to_frame(world_downsampled, TARGET_FRAME, map_cloud);
  ROS_INFO("Input cloud downsampled and transformed...");

  if (DEBUG_ENTER) {
    if (VISUALIZE) {
      sensor_msgs::PointCloud2 ros_remainder;
      pcl::toROSMsg(*map_cloud, ros_remainder);
      ros_remainder.header.frame_id = map_cloud->header.frame_id;
      table_remainder.publish(ros_remainder);
    }
    pressEnter("    Press Enter to continue...");
  }

  Eigen::Vector3f rotation, translation;
  rotation[0] = req.table_bounding_box.pose.orientation.x;
  rotation[1] = req.table_bounding_box.pose.orientation.y;
  rotation[2] = req.table_bounding_box.pose.orientation.z;
  translation[0] = req.table_bounding_box.pose.position.x;
  translation[1] = req.table_bounding_box.pose.position.y;
  translation[2] = req.table_bounding_box.pose.position.z;
  Eigen::Vector4f minPoint, maxPoint;
  maxPoint[0] = req.table_bounding_box.scale.x / 2;
  maxPoint[1] = req.table_bounding_box.scale.y / 2;
  maxPoint[2] = req.table_bounding_box.scale.z / 2;
  minPoint[0] = (-1) * maxPoint[0];
  minPoint[1] = (-1) * maxPoint[1];
  minPoint[2] = (-1) * maxPoint[2];

  PointCloudT::Ptr table_cloud(new PointCloudT);
  pcl::CropBox<PointT> crop;
  crop.setMin(minPoint);
  crop.setMax(maxPoint);
  crop.setRotation(rotation);
  crop.setTranslation(translation);
  crop.setInputCloud(map_cloud);
  crop.filter(*table_cloud);
  ROS_INFO("Crop Box executed...");

  if (DEBUG_ENTER) {
    if (VISUALIZE) {
      sensor_msgs::PointCloud2 ros_remainder;
      pcl::toROSMsg(*map_cloud, ros_remainder);
      ros_remainder.header.frame_id = map_cloud->header.frame_id;
      table_remainder.publish(ros_remainder);
    }
    pressEnter("    Press Enter to begin extracting objects...");
  }

  return true;
}


int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "table_object_segmentation_server");

	ros::NodeHandle nh;
  tf_listener = new tf::TransformListener(nh);
	ROS_INFO("table_object_segmentation_server started");

	table_object_segmentation_server = nh.advertiseService("segment_objects", segment_objects);

    // Topics that show information on the internals of this node
    if (VISUALIZE) {
      ros::NodeHandle pnh("~");
      table_object_boundingbox_pub = pnh.advertise<visualization_msgs::Marker>("objects/points", 1, true);
      table_object_pub = pnh.advertise<sensor_msgs::PointCloud2>("objects/marker", 1, true);
      table_remainder = pnh.advertise<sensor_msgs::PointCloud2>("objects/remainder", 1, true);
    }

	ros::spin();
}
