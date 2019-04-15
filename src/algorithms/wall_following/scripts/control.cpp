#include <ros/ros.h>
#include <math.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

namespace control {
	enum decision {CURRENT, FOLLOWL, FOLLOWR, STOP};


	ros::Publisher ackermann_pub_;
	ros::Subscriber laser_sub_;
	ros::Subscriber joy_sub_;

	float servo_offset = 0.0;
	int deadman_butt = 0;
	float throttle = 0.0;
	float turn = 0.0;
	const float VMAX = 2.00; // meters per second
	const int ANGLE_RANGE = 270; // Hokuyo 10LX has 270 degrees scan
	const float HALF_WIDTH = 0.2;
	float angle_left = 114.0;
	float angle_right = 66.0;
	const float RMAX = 1.0;

	float trigger_L_d = 0.0;
	float trigger_R_d = 0.0;
	float trigger_L_theta = 0.0;
	float trigger_R_theta = 0.0;

	float get_min_dist(const sensor_msgs::LaserScan::ConstPtr& data);
	bool leftOk(float angle, float dist);
	bool rightOk(float angle, float dist);

	void laser_callback(const sensor_msgs::LaserScan::ConstPtr& data) {
		if(ros::ok()) {
			ackermann_msgs::AckermannDriveStamped msg;
			decision dec = get_decision(data);

			//TODO add follow L, R, CURRENT, STOP

			float velocity_ratio = 0.5 * min_dist;
			msg.drive.steering_angle = turn * 24 * M_PI / 180 + servo_offset;
			if(min_dist < 2) {
				velocity_ratio =  min_dist * min_dist * min_dist / 8.0;
			}
			if(velocity_ratio > 1) {
				velocity_ratio = 1;
			}

			if(throttle < 0) {
				ROS_INFO("velocity ratio: %f", velocity_ratio);
				msg.drive.speed = velocity_ratio * throttle * VMAX;
			} else {
				msg.drive.speed = throttle;
			}
			if(deadman_butt == 0) {
				msg.drive.speed = 0;
				msg.drive.steering_angle = 0;
			}
			

			if(min_dist < 1 && throttle < 0){
				msg.drive.acceleration = 3;
				msg.drive.jerk = 3;
			} else {
				msg.drive.acceleration = 1;
				msg.drive.jerk = 1;
			}
			msg.drive.acceleration = 1;
			msg.drive.jerk = 1;
			/*if(abs(msg.drive.speed) < 0.1){
				msg.drive.speed = 0;			
			}*/

			msg.drive.steering_angle_velocity = 1;
			msg.header.stamp = ros::Time::now();
			msg.header.frame_id = "base_link";
			ackermann_pub_.publish(msg);
			//ROS_INFO("Velocity: %f", msg.drive.speed);
			//ROS_INFO("Angles in Degrees: %f", msg.drive.steering_angle*180/M_PI);
		}
	}

	void joy_callback(const sensor_msgs::Joy::ConstPtr& data) {
		std::vector<int> buttons = data->buttons;
		deadman_butt = buttons[4];
		std::vector<float> axes = data->axes;
		throttle = axes[1] * -1.0;
		turn = axes[2];
		//ROS_INFO("turn: %f", turn);
	}


	decision get_decision(const sensor_msgs::LaserScan::ConstPtr& data) {
		float angle_min = data->angle_min;
		float angle_max = data->angle_max;
		float angle_inc = data->angle_increment;
		std::vector<float> ranges = data->ranges;
		std::vector<float> filtered;
		int num = ranges.size();
		for(int i = 0; i < num-3; i++) {
			filtered.push_back((ranges[i] + ranges[i+1] + ranges[i+2] + ranges[i+3]) * 0.25);
		}
		float min_dist = 30.0;
		float angle = 0.0;
		int length = num - 3;
		bool current = true;
		bool followL = true;
		bool followR = true;
		trigger_L_d = 0.0;
		trigger_R_d = 0.0;
		trigger_L_theta = 0.0;
		trigger_R_theta = 0.0;
		for(int i = 0; i < 11; i++){
			int index = (int)(length * (turn * 24 + 90 + 45 + (i - 5) * 15) / ANGLE_RANGE);
			float dist = filtered[index];
			if(sin(abs((i-5)*15) * M_PI / 180) < (HALF_WIDTH / dist)){
				min_dist = std::min(min_dist, dist);	
				angle = turn*24 + (i-5) * 15; //angle relative to front, right is negative
				if(dist < 1.5){
					current = false;
				}
				
				followL &= leftOk(angle, dist, &trigger_L_d, &trigger_L_theta);
				followR &= rightOk(angle, dist, &trigger_R_d, &trigger_R_theta);	
			}		
		}
		// Determin follow_right or follow_left or stop
		if(current) {
			return CURRENT;
		} else if(followL && followR) {
			if(turn < 0){ //user turning right
				return FOLLOWL;			
			} else { //user turning left
				return FOLLOWR;
			}
		} else if(followL){return FOLLOWL;}
		else if(followR) {return FOLLOWR;}
		else {return STOP;}
	}

	bool leftOk(float angle, float dist, float* trigger_d, float* trigger_theta){
		float a = RMAX - HALF_WIDTH + dist * sin(angle);
		float b = dist * cos(angle);
		if(dist < *trigger_d){
			*trigger_d = dist;
			*trigger_theta = angle;
		}
		return pow(a, 2) + pow(b, 2) > pow(RMAX, 2);
	}

	bool rightOk(float angle, float dist, float* trigger_d, float* trigger_theta){
		angle = -1.0 * angle;
		float a = RMAX - HALF_WIDTH + dist * sin(angle);
		float b = dist * cos(angle);
		if(dist < *trigger_d){
			*trigger_d = dist;
			*trigger_theta = angle;
		}
		return pow(a, 2) + pow(b, 2) > pow(RMAX, 2);
	}

	//TODO: add follow left
	float followLeft()
	//TODO: add follow right

}

int main(int argc, char** argv) {
	ros::init(argc, argv, "control", ros::init_options::AnonymousName);
	ros::NodeHandle n;

	control::ackermann_pub_ = n.advertise<ackermann_msgs::AckermannDriveStamped>
	("/vesc/ackermann_cmd_mux/input/teleop", 5);
	control::laser_sub_ = n.subscribe("scan", 2, control::laser_callback);
	control::joy_sub_ = n.subscribe("vesc/joy", 2, control::joy_callback);
	ROS_INFO("Start control node CPP!");
	ros::spin();
	return 0;
}
