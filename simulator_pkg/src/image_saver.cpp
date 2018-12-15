
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Pose.h>

#include <gazebo_msgs/SetModelState.h>
#include <reconstruction/ImageStamped.h>
#include <cstdlib>

using namespace std;

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

	geometry_msgs::Pose start_pose;
	start_pose.position.x = 0;
	start_pose.position.x = 0;
	start_pose.position.z = 0;
	start_pose.orientation.x = 0.0;
	start_pose.orientation.y = 0.0;
	start_pose.orientation.z = 0.0;
	start_pose.orientation.w = 0.0;

	geometry_msgs::Twist start_twist;
	start_twist.linear.x = 0.0;
	start_twist.linear.y = 0.0;
	start_twist.linear.z = 0.0;
	start_twist.angular.x = 0.0;
	start_twist.angular.y = 0.0;
	start_twist.angular.z = 0.0;

	gazebo_msgs::SetModelState setmodelstate;
	gazebo_msgs::ModelState modelstate;
	
	modelstate.model_name = "my_object"; 
	modelstate.reference_frame = "world";
	modelstate.twist = start_twist;
	modelstate.pose = start_pose;
	setmodelstate.request.model_state = modelstate;

	float alpha;//orientation in x,y plane
	float theta;
	float radios;
	float hight;
	hight=0.3;
	radios=0.5;
	//theta=atan2(radios/hight);
	alpha=0.0;
	
	//x=sin(alpha)
	//y=cos(alpha)
	while (ros::ok()) {

		modelstate.twist = start_twist;
		modelstate.pose = start_pose;
		setmodelstate.request.model_state = modelstate;
		image_pose_=start_pose;
		if (client.call(setmodelstate)){
			ROS_INFO("New Camera Pose!!!");
			ROS_INFO("%f, %f, %f", modelstate.pose.position.x,modelstate.pose.position.y,modelstate.pose.position.y);
		}else{
			ROS_ERROR("Failed to call service ");
		}
		ros::spinOnce();
		image_pose_=start_pose;
		msg.header.stamp = ros::Time::now();
		msg.header.frame_id = "/world";
		msg.image=image_sub_;
		msg.pose=image_pose_;
		timer.sleep();
	}
	return 0;
}
