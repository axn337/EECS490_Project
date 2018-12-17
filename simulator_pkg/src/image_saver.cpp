
#include <ros/ros.h>
#include <cmath>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Pose.h>

#include <gazebo_msgs/SetModelState.h>
#include <reconstruction/ImageStamped.h>
#include <cstdlib>

#include <Eigen/Geometry>
#include <Eigen/Dense>

using namespace Eigen;

sensor_msgs::Image image_sub_;
geometry_msgs::Pose image_pose_;

void imageCb(const sensor_msgs::Image msg){
        image_sub_=msg;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "image_saver");
	ros::NodeHandle n; //        
	ros::Duration timer(0.1);

	//inst
	reconstruction::ImageStamped msg;

	ros::Subscriber my_sub_ = n.subscribe("simple_camera/image_raw", 1,imageCb);
	ros::Publisher my_pub_ = n.advertise<reconstruction::ImageStamped>("image_stamped", 1);
	ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state"); 

	//geometry_msgs::Pose start_pose;
	image_pose_.position.x = 0;
	image_pose_.position.x = 0;
	image_pose_.position.z = 0;
	image_pose_.orientation.x = 0.0;
	image_pose_.orientation.y = 0.0;
	image_pose_.orientation.z = 0.0;
	image_pose_.orientation.w = 0.0;

	geometry_msgs::Twist start_twist;
	start_twist.linear.x = 0.0;
	start_twist.linear.y = 0.0;
	start_twist.linear.z = 0.0;
	start_twist.angular.x = 0.0;
	start_twist.angular.y = 0.0;
	start_twist.angular.z = 0.0;

	gazebo_msgs::SetModelState setmodelstate;
	gazebo_msgs::ModelState modelstate;
	
	modelstate.model_name = "simple_camera_model"; 
	modelstate.reference_frame = "world";
	modelstate.twist = start_twist;
	modelstate.pose = image_pose_;
	setmodelstate.request.model_state = modelstate;

	float alpha=0.0;//orientation in x,y plane
	float theta;//the angle of the victor pointing towards the camera
	float d; //the length of segment from origin to camera origin
	float dxy=0.5; // the radios projection in xy-plane
	float hight=0.3; //hight of camera
	float x,y,z,roll,pitch,yaw;

	d=sqrt(pow(hight,2)+pow(dxy,2));
	theta=atan2(hight,dxy);
	
	Quaternionf q;

	while (ros::ok()) {

		alpha=alpha+0.174533;

		x=dxy*cos(alpha);
		y=dxy*sin(alpha);
		z=hight;
		roll=(cos(alpha)*theta)-0.234;
		pitch=(sin(alpha)*theta)+0.958;
		yaw=alpha+(1.5708)+1.586;

	
		q = AngleAxisf(roll, Vector3f::UnitX())
		    * AngleAxisf(pitch, Vector3f::UnitY())
		    * AngleAxisf(yaw, Vector3f::UnitZ());

		image_pose_.position.x = x;
		image_pose_.position.y = y;
		image_pose_.position.z = z;

		image_pose_.orientation.x = q.x();
		image_pose_.orientation.y = q.y();
		image_pose_.orientation.z = q.z();
		image_pose_.orientation.w = q.w();

		modelstate.twist = start_twist;
		modelstate.pose = image_pose_;
		setmodelstate.request.model_state = modelstate;

		if (client.call(setmodelstate)){
			ROS_INFO("New Camera Pose!!!");
			ROS_INFO("Position: \n %f, %f, %f", modelstate.pose.position.x, modelstate.pose.position.y, modelstate.pose.position.z);
			ROS_INFO("Orientation: \n %f, %f, %f, %f", modelstate.pose.orientation.x, modelstate.pose.orientation.y, modelstate.pose.orientation.z, modelstate.pose.orientation.w);
		}else{
			ROS_ERROR("Failed to call service ");
		}
		ros::spinOnce();

		msg.header.stamp = ros::Time::now();
		msg.header.frame_id = "/world";
		msg.image=image_sub_;
		msg.pose=image_pose_;
		timer.sleep();
	}
	return 0;
}
