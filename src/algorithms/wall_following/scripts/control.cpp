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
	const float CAR_LENGTH = 0.50; //0.5 meter
	const float BUFFER_ZONE = 0.2;
	const float RMAX = 1.0;
	float angle_left = 114.0;
	float angle_right = 66.0;
	
	const float kp = 100.0;
	const float kd = 0.01;
	const float ki = 0.0;

	float trigger_L_d = 0.0;
	float trigger_R_d = 0.0;
	float trigger_L_theta = 0.0;
	float trigger_R_theta = 0.0;

	float old_min_dist = 0.0;
	ros::Time old_time;

	float get_min_dist(const sensor_msgs::LaserScan::ConstPtr& data);
	decision get_decision(const sensor_msgs::LaserScan::ConstPtr& data);
	bool leftOk(float angle, float dist, float* trigger_d, float* trigger_theta);
	bool rightOk(float angle, float dist, float* trigger_d, float* trigger_theta);
	float followLeft();
	float followRight();

	void laser_callback(const sensor_msgs::LaserScan::ConstPtr& data) {
		if(ros::ok()) {
			ackermann_msgs::AckermannDriveStamped msg;
			decision dec = get_decision(data);
			
			//TODO add follow L, R, CURRENT, STOP
			if(dec == CURRENT) {
				// TODO: fix
				ROS_INFO("CURRENT");
				msg.drive.steering_angle = turn * 24 * M_PI / 180 + servo_offset;

				if(throttle < 0) {
					msg.drive.speed = throttle * VMAX;
				} else {
					msg.drive.speed = throttle * 0.5;
				}
				msg.drive.acceleration = 1;
				msg.drive.jerk = 1;
			} else if(dec == FOLLOWL) {
				ROS_INFO("LEFT");
				msg.drive.steering_angle = followLeft();
				msg.drive.speed = throttle * VMAX;
				msg.drive.acceleration = 1;
				msg.drive.jerk = 1;
				ROS_INFO("%f", msg.drive.steering_angle);
			} else if(dec == FOLLOWR) {
				ROS_INFO("RIGHT");
				msg.drive.steering_angle = followRight();
				msg.drive.speed = throttle * VMAX;
				msg.drive.acceleration = 1;
				msg.drive.jerk = 1;
				ROS_INFO("%f", msg.drive.steering_angle);
			} else {
				// STOP
				ROS_INFO("STOP");
				msg.drive.speed = 0;
				msg.drive.steering_angle = 0;
				msg.drive.acceleration = 3;
				msg.drive.jerk = 3;
			}

			if (msg.drive.steering_angle > 24 * M_PI / 180.0) {
				msg.drive.steering_angle = 24 * M_PI / 180.0;
			}
			if(msg.drive.steering_angle < -24 * M_PI / 180.0) {
				msg.drive.steering_angle = -24 * M_PI / 180.0;
			}
			msg.drive.steering_angle_velocity = 1;
			msg.header.stamp = ros::Time::now();
			msg.header.frame_id = "base_link";
			if(deadman_butt == 0) {
				msg.drive.speed = 0;
				msg.drive.steering_angle = 0;
			}
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
				followL &= leftOk(angle, dist, &trigger_L_d, &trigger_L_theta);
				followR &= rightOk(angle, dist, &trigger_R_d, &trigger_R_theta);	
			}		
		}
		ros::Duration delta_t = data->header.stamp - old_time;
		float delta_d = old_min_dist - min_dist;
		float appr_v = delta_d / ((float)delta_t.nsec / (double)1000000000L);
		if(appr_v > min_dist && min_dist < 1.5){ //velocity and min_dist function to determine current
			current = false;
		}

		// Determin follow_right or follow_left or stop
		old_time = data->header.stamp;
		old_min_dist = min_dist;
		if(current || throttle > 0) { //or in reverse
			return CURRENT;
		} else if(followL && followR) {
			if(turn < 0){ //user turning right
				return FOLLOWL;			
			} else if(turn > 0){ //user turning left
				return FOLLOWR;
			} else {
				return STOP;
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


	float followLeft() {
		float theta = (90-trigger_L_theta) * M_PI / 180.0;
		float d = trigger_L_d - BUFFER_ZONE;
		float alpha = atan(d*sin(theta)/(HALF_WIDTH-d*cos(theta)));
		return (alpha - M_PI/2.0);
	}

	float followRight() {
		float theta = (90 + trigger_R_theta) * M_PI / 180.0;
		float d = trigger_R_d - BUFFER_ZONE;
		float alpha = atan(d*sin(theta)/(HALF_WIDTH-d*cos(theta)));

		return (M_PI/2.0 - alpha);
	}

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
