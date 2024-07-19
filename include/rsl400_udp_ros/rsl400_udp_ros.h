#pragma once

#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>

#include "ros/ros.h"

#include <sensor_msgs/LaserScan.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>

#include "rsl400_udp_ros/rsl400/stateimage.h"
#include "rsl400_udp_ros/rsl400/udpstateimage.h"

namespace LeuzePacketId
{
    const int BEAM_DESCRIPTION = 1;
    const int BEAM_STRENGTH_ID = 3;
    const int BEAM_ID = 6;
} // namespace LeuzePacketId

class Rsl400UdpNode
{
public:
    Rsl400UdpNode(ros::NodeHandle *nodehandle);
    ~Rsl400UdpNode();
    int run();

private:
    ros::NodeHandle nh; // ROS node handle

    std::string _address;       // IP address of the data source. The sensor must be configured to send data to this address.
    std::string _diagnostics_name;  // Name of the diagnostics node
    std::string _diagnostics_id;    // Hardware ID of the diagnostics node. Defaults frame_id.
    int _port;                  // Port number of the data source. The sensor must be configured to send data to this port.
    struct addrinfo *_addrinfo; // Address information of the data source.
    int _socket;                // UDP socket file descriptor

    const int BUFFER_LEN = 2048000; // 2 MB UDP socket receive buffer
    char *_receive_buffer;          // buffer for received UDP packets

    /**
     * Leuze imposes a limit of 1460 bytes on the payload of its UDP packets.
     * This is used in expected number of beam calculations.
     */
    const int LEUZE_PREFERRED_PAYLOAD_LIMIT = 1460;

    sensor_msgs::LaserScan _scan_msg; // ROS message for publishing scans

    /**
     * Remembers which packets have been received since last beam description
     * packet. This is used to determine when a full scan has been received.
     * Each bit in the mask encodes whether a packet of that block number has been received.
     */
    unsigned int _received_bitmask = 0;

    int _scan_count = 0;       // Number of scans received since the sensor woke up
    ros::Time _prev_scan_time; // Time of the previous scan

    int _beam_count = 0;    // Number of beams in the scan according to the beam description
    double _poll_rate;      // Rate at which the UDP socket is polled for data
    double _socket_timeout; // Timeout for the UDP socket, seconds

    // ROS publishers
    ros::Publisher _scan_pub;
    ros::Publisher _diagnostics_pub;

    int open_udp_socket(const std::string &addr, int port, struct addrinfo *addrinfo);
    int recv(char *msg, size_t max_size);
    void publish_scan();
    bool get_assignment_range(int *block_start, RSL400::PUdpTelegramType udpTelegramType, size_t data_type_size, int num_beams);
    int get_num_beams(RSL400::PUdpTelegramType udpTelegramType, size_t data_type_size);
    bool handle_beam_description(char *receive_buffer, int length);
    bool handle_beam_data(char *receive_buffer, int length);
    bool handle_beam_strength_data(char *receive_buffer, int length);
};

/**
 * Converts a decidegree to radians.
 * Leuze reports angles in decidegrees, which are 1/10 of a degree.
 * @param decidegrees The angle in decidegrees.
 * @return The angle in radians.
 */
float decidegree_to_radians(int decidegrees)
{
    return (float)(decidegrees) * 0.1 * M_PI / 180.0;
}

diagnostic_msgs::KeyValue make_entry(std::string key, int value)
{
    diagnostic_msgs::KeyValue key_value;
    key_value.key = key;
    key_value.value = std::to_string(value);
    return key_value;
}

diagnostic_msgs::KeyValue make_buf_entry(std::string key, char *buf, int len)
{
    diagnostic_msgs::KeyValue key_value;
    key_value.key = key;
    key_value.value = std::string(buf, len);
    return key_value;
}
