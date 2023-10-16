#pragma once

#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>

#include "ros/ros.h"

#include <sensor_msgs/LaserScan.h>

#include "rsl400_udp_ros/rsl400/stateimage.h"
#include "rsl400_udp_ros/rsl400/udpstateimage.h"

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
    char *_receive_buffer;

    double _poll_rate;

    ros::Publisher _scan_pub;

    int open_udp_socket(const std::string& addr, int port, struct addrinfo *addrinfo);
    int recv(char *msg, size_t max_size);
    int timed_recv(char *msg, size_t max_size, int max_wait_ms);
    void publishScanDatagram();
};
