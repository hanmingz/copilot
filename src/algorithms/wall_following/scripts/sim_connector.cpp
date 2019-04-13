#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <race/drive_param.h>

namespace sim_connector{
	ros::Publisher ackermann_pub_;
	ros::Subscriber driveparam_sub_;

	void drive_callback(const race::msg::drive_param::ConstPtr& data){
		if(ros::ok()){
			ackermann_msgs::msg::AckermannDriveStamped msg;
			msg.header.stamp = ros::Time::now();
			msg.header.frame_id = "base_link";

			msg.drive.speed = data->velocity;
			msg.drive.acceleration = 3;
			msg.drive.jerk = 3;
			msg.drive.steering_angle = data.angle;
			msg.drive.steering_angle_velocity = 1;

			ackermann_pub_.publish(msg);
		}
	}
}

int main(int argc, char **argv){
	ros::init(argc, argv, "sim_connect", ros::init_options::AnonymousName);
	ros::NodeHandle n;

	sim_connector::ackermann_pub_ = n.advertise<ackermann_msgs::msg::AckermannDriveStamped>
	("/vesc/ackermann_cmd_mux/input/teleop", 5);
	sim_connector::driveparam_sub_ = n.subscribe("drive_parameters", 2, sim_connector::drive_callback);
	ROS_INFO("Start sim connector node cpp");
	ros::spin()
	return 0;
}