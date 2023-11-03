#pragma once

#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>

#include "ros/ros.h"

#include <sensor_msgs/LaserScan.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>

#include "rsl400_udp_ros/rsl400/stateimage.h"
#include "rsl400_udp_ros/rsl400/udpstateimage.h"

diagnostic_msgs::KeyValue make_entry(std::string key, int value) {
    diagnostic_msgs::KeyValue key_value;
    key_value.key = key;
    key_value.value = std::to_string(value);
    return key_value;
}

float decidegree_to_radians(int decidegrees) {
    return (float)(decidegrees) * 0.1 * M_PI / 180.0;
}

class Rsl400UdpNode
{
public:
    Rsl400UdpNode(ros::NodeHandle* nodehandle);
    ~Rsl400UdpNode();
    int run();
private:
    ros::NodeHandle nh;  // ROS node handle

    const int BUFFER_LEN = 2048000;

    int _port;
    struct addrinfo *_addrinfo;
    int _socket;
    std::string _frame_id;

    char *_receive_buffer;
    
    std::vector<float> _ranges;
    std::vector<float> _intensities;

    int _scan_count = 0;
    ros::Time _prev_scan_time;

    int _beam_count = 0;
    float _start_angle;
    float _stop_angle;
    float _angle_increment;
    float _time_increment;

    double _poll_rate;

    ros::Publisher _scan_pub;
    ros::Publisher _diagnostics_pub;

    int open_udp_socket(const std::string& addr, int port, struct addrinfo *addrinfo);
    int recv(char *msg, size_t max_size);
    int timed_recv(char *msg, size_t max_size, int max_wait_ms);
    void publish_scan();
};
