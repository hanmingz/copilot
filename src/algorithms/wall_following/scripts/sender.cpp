#include<ros/ros.h>
#include<sys/types.h>
#include<sys/socket.h>
#include<arpa/inet.h>
#include<netinet/in.h>
#include<stdio.h>
#include<string.h>
#include<math.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

#define PORT 5005
#define format "%d,%d,%d,%d,%d"

namespace UDPSender {
    int fd;
    struct sockaddr_in servaddr;
    char buffer[100];
    int arr[5];
    ros::Subscriber control_sub_;
    ros::Subscriber joy_sub_;
    ros::Subscriber mode_sub_;

    int deadman_butt = 0;
    float throttle = 0.0;
    float turn = 0.0;
    float speed = 0.0;
    float steering_angle = 0.0;
    char mode = 0;

    void control_callback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& data) {
        //scale speed to 100, steering_angle to -100 to 100
        speed = data->drive.speed * 12.50;
        steering_angle = data->drive.steering_angle * 180.0 / M_PI * 4;
        mode = data->mode;
    }

    void joy_callback(const sensor_msgs::Joy::ConstPtr& data) {
        std::vector<int> buttons = data->buttons;
        deadman_butt = buttons[4];
        std::vector<float> axes = data->axes;
        throttle = axes[1] * -1.0 * 100;
        turn = axes[2] * 100;

        sprintf(buffer, format, (int)steering_angle, (int)speed, (int)throttle, (int)turn, (int)mode);
        sendto(fd, (char*)buffer, strlen(buffer), MSG_CONFIRM, (struct sockaddr*) &servaddr, sizeof(servaddr));
    }
}

int main(int argc, char** argv) {
    int fd;
    struct sockaddr_in servaddr;
    
    if((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        ROS_INFO("Error: open socket.");
        return -1;
    }
    
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(PORT);
    servaddr.sin_addr.s_addr = inet_addr("192.168.137.1");

    ros::init(argc, argv, "sender", ros::init_options::AnonymousName);
    ros::NodeHandle n;

    UDPSender::control_sub_ = n.subscribe("vesc/ackermann_cmd_mux/input/teleop", 2, UDPSender::control_callback);
    UDPSender::joy_sub_ = n.subscribe("vesc/joy", 2, UDPSender::joy_callback);
    UDPSender::fd = fd;
    UDPSender::servaddr = servaddr;

    ROS_INFO("Start control node CPP!");
    ros::spin();
    return 0;
}
