#include <string>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>


// Convert to x, y, theta
tf::Transform &flatten_transform(tf::Transform &transform){

	transform.getOrigin().setZ(0);

	double yaw, pitch, roll;
	transform.getBasis().getEulerYPR(yaw, pitch, roll);
	transform.getBasis().setEulerYPR(yaw, 0, 0);

	return transform;
}


int main(int argc, char** argv){
	ros::init(argc, argv, "drift_broadcaster");

	std::string aruco_basefootprint_frame = "/camera_position";
	std::string map_frame = "/map";
	std::string odom_frame = "/odom";
	std::string basefootprint_frame = "/softkinetic_camera_rgb_optical_frame";

	ros::NodeHandle node("~");


	node.getParam("aruco_frame", aruco_basefootprint_frame);
	node.getParam("lizi_frame", basefootprint_frame);

	// std::cout << "======================================" << std::endl;
	// std::cout << aruco_basefootprint_frame << std::endl;
	// std::cout << basefootprint_frame << std::endl;
	// std::cout << "======================================" << std::endl;

	tf::TransformListener listener;
	static tf::TransformBroadcaster broadcaster;

	tf::StampedTransform map_to_aruco_basefootprint_tf;
	tf::StampedTransform odom_to_basefootprint_tf;
	tf::Transform map_to_odom_tf;


	try{
		listener.waitForTransform(odom_frame, basefootprint_frame, ros::Time(0), ros::Duration(10.0) );
		listener.waitForTransform(map_frame, aruco_basefootprint_frame, ros::Time(0), ros::Duration(10.0) );
	}
	catch(tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
	}


	ros::Rate rate(50.0);
	while(node.ok()){

		try{
			listener.lookupTransform(basefootprint_frame, odom_frame, ros::Time(0), odom_to_basefootprint_tf);
			listener.lookupTransform(aruco_basefootprint_frame, map_frame, ros::Time(0), map_to_aruco_basefootprint_tf);

			map_to_odom_tf = map_to_aruco_basefootprint_tf.inverse() * odom_to_basefootprint_tf;
			map_to_odom_tf = flatten_transform(map_to_odom_tf);
		}
		catch(tf::TransformException ex){
			ROS_ERROR_THROTTLE(10, "%s",ex.what());
		}

		broadcaster.sendTransform(tf::StampedTransform(map_to_odom_tf, ros::Time::now(), map_frame, odom_frame));

		rate.sleep();
	}

	ros::spin();
	return 0;
};
