#include "../include/horizontal_plane_detector_server.h"
#include <signal.h>
#include <vector>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <pcl_ros/impl/transforms.hpp>

// PCL specific includes

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/console/parse.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/common/time.h>
#include <pcl/common/common.h>

#include <pcl/features/pfh.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/vfh.h>
#include <pcl/features/cvfh.h>
#include <pcl/features/moment_of_inertia_estimation.h>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>


#include "villa_surface_detectors/DetectHorizontalPlanes.h"


#include "../include/BoundingBox.h"
#include <visualization_msgs/MarkerArray.h>


#define BOUNDING_BOX_MARKER_NAMESPACE "horizontal_planes_marker"
#define Z_AXIS_REFERENCE_FRAME "base_link" // Find planes perpendicular to the z-axis of this frame
#define TARGET_FRAME "map" //target frame name DONT CHANGE!
#define EPS_ANGLE 0.05 //epsilon angle for segmenting, value in radians
#define VOXEL_LEAF_SIZE 0.02 //size of voxel leaf for processing
#define RANSAC_MAX_ITERATIONS 10000
#define PLANE_DIST_TRESH 0.025 //maximum distance from plane
#define CLUSTER_TOL 0.05 //clustering tolerance for largest plane extraction
#define MIN_NUMBER_PLANE_POINTS 500
#define MIN_PLANE_DENSITY 30000
#define STOPPING_PERCENTAGE 0.25 // Stop once we've processed all but X percentage of the cloud
#define IGNORE_FLOOR true // If the input cloud doesn't already have z filtered, we can do it
#define MIN_Z 0.05 // minimum z-value of point cloud in map frame to be considered
#define MAX_Z 2.0 // maximum z-value of point cloud in map frame to be considered

#define VISUALIZE true
#define DEBUG_ENTER true // if true, you have to press enter to continue the process

/* define what kind of point clouds we're using */
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

using namespace std;

//service for horizontal_plane segmenting
ros::ServiceServer horizontal_plane_serv;
ros::Publisher voxel_cloud_pub;
ros::Publisher horizontal_plane_marker_pub;
ros::Publisher current_plane_cloud_pub;
ros::Publisher remaining_cloud_pub;
//create listener for transforms
tf::TransformListener *tf_listener;


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

visualization_msgs::Marker createBoundingBoxMarker(const BoundingBox &box,
												   const int &marker_index){
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = TARGET_FRAME;
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = BOUNDING_BOX_MARKER_NAMESPACE;
    marker.id = marker_index;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::CUBE;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header

    marker.pose.position.x = box.position[0];
    marker.pose.position.y = box.position[1];
    marker.pose.position.z = box.position[2];
	marker.pose.orientation.x = (double)box.orientation.x();
	marker.pose.orientation.y = (double)box.orientation.y();
	marker.pose.orientation.z = (double)box.orientation.z();
	marker.pose.orientation.w = (double)box.orientation.w();

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = abs(box.max[0] - box.min[0]);
    marker.scale.y = abs(box.max[1] - box.min[1]);
    marker.scale.z = abs(box.max[2] - box.min[2]);

    // Set the color -- be sure to set alpha to something non-zero!
    //int	apply_color = marker_index % colormap.size();

    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 0.25;

    marker.lifetime = ros::Duration();

    return marker;
}

BoundingBox extract_aa_bounding_box_params(const PointCloudT::Ptr &plane_cloud){
    Eigen::Vector4f centroid = Eigen::Vector4f::Zero();
    pcl::compute3DCentroid(*plane_cloud, centroid);
    Eigen::Vector4f min, max = Eigen::Vector4f::Zero();
    pcl::getMinMax3D(*plane_cloud, min, max);

	Eigen::Vector4f position((max[0] - min[0]) / 2, (max[1] - min[2]) / 2, (max[3] - min[3]) / 2, 1);
	return BoundingBox(min, max, centroid, Eigen::Quaternionf(0,0,0,1), position, 0);
}

BoundingBox extract_oriented_bounding_box_params(const PointCloudT::Ptr &plane_cloud){
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*plane_cloud, centroid);

	pcl::MomentOfInertiaEstimation <PointT> feature_extractor;
	feature_extractor.setInputCloud (plane_cloud);
	feature_extractor.compute();

	PointT min, max, position;
	Eigen::Matrix3f rotational_matrix;
	feature_extractor.getOBB(min, max, position, rotational_matrix);
	Eigen::Quaternionf quat(rotational_matrix);
	// Homogeneous coordinates
	Eigen::Vector4f min_vec(min.x, min.y, min.z, 1);
	Eigen::Vector4f max_vec(max.x, max.y, max.z, 1);
	Eigen::Vector4f position_vec(position.x, position.y, position.z, 1);

    return BoundingBox(min_vec, max_vec, centroid, quat, position_vec, 0);

}

bool compare_cluster_size(const pcl::PointIndices &lhs, const pcl::PointIndices &rhs) {
    return lhs.indices.size() > rhs.indices.size();
}

/*Function for finding the largest plane from the segmented "table"
 * removes noise*/
std::vector<PointCloudT::Ptr> isolate_surfaces(const PointCloudT::Ptr &in, double tolerance){
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (in);
    ROS_INFO("point cloud size of 'plane cloud' : %ld", in->size());

    //use euclidean cluster extraction to eliminate noise and get largest plane
    vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (tolerance);
    ec.setMinClusterSize (MIN_NUMBER_PLANE_POINTS);
    ec.setSearchMethod (tree);
    ec.setInputCloud (in);
    ec.extract (cluster_indices);

    ROS_INFO("number of 'plane clouds' : %ld", cluster_indices.size());

    if (cluster_indices.empty()) {
        throw std::exception();
    }
		ROS_INFO("    Total number of surfaces from plane: %zu", cluster_indices.size());

    //pcl::PointIndices largest_cluster_indices = *std::max_element(cluster_indices.begin(), cluster_indices.end(), compare_cluster_size);
		std::vector<PointCloudT::Ptr> surfaces;

		//Optional
		//std::sort(cluster_indices, compare_cluster_size);
		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it < cluster_indices.end(); it++) {
			PointCloudT::Ptr cluster(new PointCloudT(*in, (*it).indices));
	    surfaces.push_back(cluster);
		}
		return surfaces;
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

double calculate_density(const PointCloudT::Ptr &cloud, const BoundingBox &box) {
	// TODO: Calculate true volume
	// If the cloud is one point thick in some dimension, we'll assign that dimension a magnitude of 1cm
	double x_dim = max(abs(box.max[0] - box.min[0]), 0.01f);
	double y_dim = max(abs(box.max[1] - box.min[1]), 0.01f);
	double z_dim = max(abs(box.max[2] - box.min[2]), 0.01f);
	double volume = x_dim * y_dim * z_dim;
	return (double)cloud->size() / volume;
}


void vector_to_frame(const geometry_msgs::Vector3Stamped &vector, const string &target_frame, geometry_msgs::Vector3Stamped &out_vector) {
    if (vector.header.frame_id == target_frame) {
        out_vector = vector;
        return;
    }
	string z_axis_reference_frame (Z_AXIS_REFERENCE_FRAME);
    //transform into the camera's frame
    while (ros::ok()){
        tf::StampedTransform transform;
        try{
            tf_listener->transformVector(target_frame, ros::Time(0), vector, z_axis_reference_frame, out_vector);
            break;
        }
            //keep trying until we get the transform
        catch (tf::TransformException ex){
            ROS_ERROR_THROTTLE(1, "%s", ex.what());
            ROS_WARN_THROTTLE(1, "   Waiting for tf to transform desired SAC axis to point cloud frame. trying again");
            continue;
        }
    }
}

bool compare_by_second(const pair<int, float> &lhs, const pair<int, float> &rhs) {
	return lhs.second > rhs.second;
}

bool find_horizontal_planes(villa_surface_detectors::DetectHorizontalPlanes::Request &req, villa_surface_detectors::DetectHorizontalPlanes::Response &res){

    PointCloudT::Ptr cloud(new PointCloudT);
	//convert to PCL format
	pcl::fromROSMsg (req.cloud_input, *cloud);
    cloud->header.frame_id = req.cloud_input.header.frame_id;


	if (cloud->points.empty()){
		ROS_ERROR("The input cloud is empty. Cannot process service");
		return false;
	}

	// Convert cloud to map frame:

	// Define Frame Parameters
	string z_axis_reference_frame (Z_AXIS_REFERENCE_FRAME);
	string target_frame (TARGET_FRAME);

    PointCloudT::Ptr map_cloud(new PointCloudT);
    move_to_frame(cloud, TARGET_FRAME, map_cloud);

	// If the floor is to be ignored, use the filtered map instead
	if (IGNORE_FLOOR){

        // Create filtered point cloud
        PointCloudT::Ptr map_cloud_pass_through (new PointCloudT);

        // Create the filtering object
        pcl::PassThrough<PointT> pass;
        pass.setInputCloud (map_cloud);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (MIN_Z, MAX_Z);
        //pass.setFilterLimitsNegative (true);
        pass.filter (*map_cloud_pass_through);
        map_cloud = map_cloud_pass_through;
	}


	// Create the filtering object: downsample the dataset using a leaf size of 1cm
	PointCloudT::Ptr cloud_filtered (new PointCloudT);
	pcl::VoxelGrid<PointT> vg;
	vg.setInputCloud (map_cloud);
	// TODO: Do we need as much resolution in XY as Z?
	vg.setLeafSize (VOXEL_LEAF_SIZE, VOXEL_LEAF_SIZE, VOXEL_LEAF_SIZE);
	vg.filter (*cloud_filtered);

	if (VISUALIZE) {
		//debugging publishing
		sensor_msgs::PointCloud2 ros_cloud;
		pcl::toROSMsg(*cloud_filtered, ros_cloud);
		ros_cloud.header.frame_id = map_cloud->header.frame_id;
		ROS_INFO("Visualizing pointcloud to extract horizontal planes from...");
		voxel_cloud_pub.publish(ros_cloud);
	}
	if (DEBUG_ENTER){
        pressEnter("    Press ENTER to continue");
	}

	//create onjects for use in segmenting
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
	PointCloudT::Ptr cloud_plane (new PointCloudT);
	PointCloudT::Ptr cloud_remainder (new PointCloudT);

	// Create the segmentation object
	pcl::SACSegmentation<PointT> seg;
	seg.setOptimizeCoefficients (true);

	//look for a plane perpendicular to a given axis
	seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (RANSAC_MAX_ITERATIONS);
	seg.setDistanceThreshold (PLANE_DIST_TRESH);
	seg.setEpsAngle(EPS_ANGLE);

	//create the axis to use
	ROS_INFO("Finding planes perpendicular to z-axis of %s frame",  z_axis_reference_frame.c_str());
	geometry_msgs::Vector3Stamped ros_vec;
	ros_vec.header.frame_id = z_axis_reference_frame;
	ros_vec.vector.x = 0.0;
	ros_vec.vector.y = 0.0;
	ros_vec.vector.z = 1.0;

    geometry_msgs::Vector3Stamped out_vec;
    vector_to_frame(ros_vec, map_cloud->header.frame_id, out_vec);

	//set the axis to the transformed vector
	Eigen::Vector3f axis = Eigen::Vector3f(out_vec.vector.x, out_vec.vector.y , out_vec.vector.z);
	seg.setAxis(axis);
	ROS_INFO("SAC axis value: %f, %f, %f", seg.getAxis()[0], seg.getAxis()[1], seg.getAxis()[2]);

	// Create the filtering object
	pcl::ExtractIndices<PointT> extract;

	// Prepare object containers for responses
	vector<sensor_msgs::PointCloud2> horizontal_planes;
	vector<Eigen::Vector4f> horizontal_plane_coefs;
	vector<visualization_msgs::Marker> horizontal_plane_bounding_boxes_markers;
	vector<visualization_msgs::Marker> horizontal_plane_AA_bounding_boxes_markers;
	vector<pair<int, float> > indices_with_densities;
	int num_planes = 0;
	int ransac_iter = 0;
	size_t num_start_points = cloud_filtered->points.size();

	while (cloud_filtered->points.size() >  STOPPING_PERCENTAGE * num_start_points) {
		// Segment the largest planar component from the remaining cloud
		ROS_INFO("Extracting a horizontal plane %d", ransac_iter + 1);
		seg.setInputCloud (cloud_filtered);
		ROS_INFO("    Number of Points to Process: %zu", cloud_filtered->size());

		seg.segment (*inliers, *coefficients);
		if (inliers->indices.empty()){
		  ROS_WARN("    Could not estimate a planar model for the given dataset.");
		  break;
		}

		ROS_INFO("    Found a horizontal plane!");
		// Extract the inliers
		extract.setInputCloud (cloud_filtered);
		extract.setIndices (inliers);
		extract.setNegative (false);
		extract.filter (*cloud_plane);


		// Create the filtering object to extract everything else
		extract.setNegative (true);
		extract.filter (*cloud_remainder);
		cloud_filtered.swap (cloud_remainder);

		if (VISUALIZE) {
			//debugging publishing
			sensor_msgs::PointCloud2 ros_remainder;
			pcl::toROSMsg(*cloud_remainder, ros_remainder);
			ros_remainder.header.frame_id = map_cloud->header.frame_id;
			remaining_cloud_pub.publish(ros_remainder);

			sensor_msgs::PointCloud2 ros_cloud;
			pcl::toROSMsg(*cloud_plane, ros_cloud);
			ros_cloud.header.frame_id = map_cloud->header.frame_id;
			current_plane_cloud_pub.publish(ros_cloud);

			ROS_INFO("Entire Plane Published");
			if (DEBUG_ENTER) {
				pressEnter("		Press Enter to begin showing surfaces");
			}
		}

		// Perform clustering on this plane.
		std::vector<PointCloudT::Ptr> surfaces;

		// if no clusters are found, this is an invalid plane extraction
		try{
			double cluster_extraction_tolerance = CLUSTER_TOL;
			surfaces = isolate_surfaces(cloud_plane, cluster_extraction_tolerance);

			// if (cloud_plane->size() < MIN_NUMBER_PLANE_POINTS){
			// 	ROS_WARN("Plane contains insufficient points. Discarding");
			// 	continue;
			// }

		}
		catch(std::exception &e){
			ROS_WARN("No clusters were found. Invalid plane points exist in this iteration.");
			continue;
		}

		// if (VISUALIZE) {
		// 	sensor_msgs::PointCloud2 horizontal_plane_cloud_ros;
		// 	pcl::toROSMsg(*cloud_plane, horizontal_plane_cloud_ros);
		// 	horizontal_plane_cloud_ros.header.frame_id = map_cloud->header.frame_id;
		// 	// Now that we've pulled out the largest cluster, lets visualize
		// 	current_plane_cloud_pub.publish(horizontal_plane_cloud_ros);
		// }

		for (std::vector<PointCloudT::Ptr>::const_iterator it = surfaces.begin(); it < surfaces.end(); it++) {
			PointCloudT::Ptr current = *it;

			//get the plane coefficients
			Eigen::Vector4f plane_coefficients;
			plane_coefficients(0) = coefficients->values[0];
			plane_coefficients(1) = coefficients->values[1];
			plane_coefficients(2) = coefficients->values[2];
			plane_coefficients(3) = coefficients->values[3];
			horizontal_plane_coefs.push_back(plane_coefficients);

			// Extract the bonding box parameters of this plane

	    const BoundingBox &oriented_bbox_params = extract_oriented_bounding_box_params(current);

			// Use the oriented bounding box for a better estimate of density. Non oriented box
			// penalizes shelves that don't happen to be perfectly aligned with the map frame
		 	double plane_bounding_box_density = calculate_density(current,oriented_bbox_params);


		 	// Create Marker to represent bounding box
		 	visualization_msgs::Marker plane_bounding_box_marker;
		 	plane_bounding_box_marker = createBoundingBoxMarker(oriented_bbox_params, 0);
			horizontal_plane_bounding_boxes_markers.push_back(plane_bounding_box_marker);

		 	//store each "horizontal_plane" found
			sensor_msgs::PointCloud2 horizontal_plane_cloud_ros;
			pcl::toROSMsg(*current, horizontal_plane_cloud_ros);
			horizontal_plane_cloud_ros.header.frame_id = map_cloud->header.frame_id;
			horizontal_planes.push_back(horizontal_plane_cloud_ros);

			const BoundingBox &aa_bbbox_params = extract_aa_bounding_box_params(current);
		 	visualization_msgs::Marker plane_AA_bounding_box_marker = createBoundingBoxMarker(aa_bbbox_params, num_planes);
			horizontal_plane_AA_bounding_boxes_markers.push_back(plane_AA_bounding_box_marker);

			if (VISUALIZE) {
				// Visualize plane bounding box marker
				current_plane_cloud_pub.publish(horizontal_plane_cloud_ros);
				horizontal_plane_marker_pub.publish(plane_bounding_box_marker);
			}

			indices_with_densities.push_back(pair<int, float>(num_planes, (float) plane_bounding_box_density));
			num_planes += 1;

			if (DEBUG_ENTER){
				ROS_INFO("%d planes total", num_planes);
				pressEnter("    Press ENTER to show next surface in plane");
			}
		}
		ransac_iter++;
		if (DEBUG_ENTER) {
			pressEnter("		Press Enter to show next entire plane");
		}
	}

  std::sort(indices_with_densities.begin(), indices_with_densities.end(), compare_by_second);

	// Populate the response with planes sorted by density
	for (vector< pair<int, float> >::const_iterator it = indices_with_densities.begin(); it < indices_with_densities.end(); ++it){
		pair<int, float> pair = *it;
		int i = pair.first;
		float density = pair.second;

        if (density < MIN_PLANE_DENSITY){
            ROS_INFO("Rejecting candidate plane with low density (%f)", density);
			continue;
        };

		res.horizontal_planes.push_back(horizontal_planes.at(i));
		geometry_msgs::Quaternion coef;
		Eigen::Vector4f current = horizontal_plane_coefs[i];
		coef.x = current(0);
		coef.y = current(1);
		coef.z = current(2);
		coef.w = current(3);

		ROS_INFO("Plane %i ori: x:%f, y:%f, z:%f, w:%f, density: %f", i, coef.x, coef.y, coef.z, coef.w, density);

		res.horizontal_plane_coefs.push_back(coef);
		res.horizontal_plane_bounding_boxes.push_back(horizontal_plane_bounding_boxes_markers.at(i));
		res.horizontal_plane_AA_bounding_boxes.push_back(horizontal_plane_AA_bounding_boxes_markers.at(i));
	}


	return true;
}


int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "horizontal_plane_detector_server");
	ros::NodeHandle nh;
    tf_listener = new tf::TransformListener(nh);

	horizontal_plane_serv = nh.advertiseService("detect_horizontal_planes", find_horizontal_planes);

    if (VISUALIZE) {
        ros::NodeHandle pnh("~");
        voxel_cloud_pub = pnh.advertise<sensor_msgs::PointCloud2>("downsampled_input_cloud", 1, true);
        current_plane_cloud_pub = pnh.advertise<sensor_msgs::PointCloud2>("current_plane_cloud", 1, true);
        remaining_cloud_pub = pnh.advertise<sensor_msgs::PointCloud2>("remaining_cloud", 1, true);
        horizontal_plane_marker_pub = pnh.advertise<visualization_msgs::Marker>("current_plane_box", 1, true);
    }
    ros::spin();
};
