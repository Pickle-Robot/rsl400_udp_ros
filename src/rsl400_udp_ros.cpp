#include "rsl400_udp_ros/rsl400_udp_ros.h"

Rsl400UdpNode::Rsl400UdpNode(ros::NodeHandle* nodehandle) :
    nh(*nodehandle)
{
    ros::param::param<int>("~port", _port, 9090);
    ros::param::param<double>("~poll_rate", _poll_rate, 100.0);

    _receive_buffer = new char[BUFFER_LEN];

    _scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 50);
    
}

Rsl400UdpNode::~Rsl400UdpNode()
{
    freeaddrinfo(_addrinfo);
    close(_socket);
    delete[] _receive_buffer;
}


int Rsl400UdpNode::open_udp_socket(const std::string& addr, int port, struct addrinfo *addrinfo)
{
    char decimal_port[16];
    snprintf(decimal_port, sizeof(decimal_port), "%d", port);
    decimal_port[sizeof(decimal_port) / sizeof(decimal_port[0]) - 1] = '\0';
    struct addrinfo hints;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_protocol = IPPROTO_UDP;
    int error_code(getaddrinfo(addr.c_str(), decimal_port, &hints, &addrinfo));
    if(error_code != 0 || addrinfo == NULL)
    {
        throw std::runtime_error(("invalid address or port for UDP socket: \"" + addr + ":" + decimal_port + "\"").c_str());
    }
    int f_socket = socket(addrinfo->ai_family, SOCK_DGRAM | SOCK_CLOEXEC, IPPROTO_UDP);
    if(f_socket == -1)
    {
        freeaddrinfo(addrinfo);
        throw std::runtime_error(("could not create UDP socket for: \"" + addr + ":" + decimal_port + "\"").c_str());
    }
    error_code = bind(f_socket, addrinfo->ai_addr, addrinfo->ai_addrlen);
    if(error_code != 0)
    {
        freeaddrinfo(addrinfo);
        close(f_socket);
        throw std::runtime_error(("could not bind UDP socket with: \"" + addr + ":" + decimal_port + "\"").c_str());
    }
    return f_socket;
}

void Rsl400UdpNode::publishScanDatagram() {
    recv(_receive_buffer, BUFFER_LEN);

    RSL400::PUdpTelegramType udpTelegramType = (RSL400::PUdpTelegramType)_receive_buffer;

    ROS_INFO("--> Telegram Type: %d\r\n", udpTelegramType->Id);
    if (udpTelegramType->Id == 1) {
        int test = sizeof( RSL400::UdpExtStateImageType1);
        RSL400::PUdpExtStateImageType1 udpExtStateImageType1 = (RSL400::PUdpExtStateImageType1)_receive_buffer;

        // output some data
        ROS_INFO("ScanNo : %d\r\n", udpExtStateImageType1->StateImage1.ScanNo);
        ROS_INFO("IsOssdA: %d\r\n", udpExtStateImageType1->StateImage1.IsOssdA);
        ROS_INFO("IsEA1  : %d\r\n", udpExtStateImageType1->StateImage1.IsEA1);
        ROS_INFO("IsEA2  : %d\r\n", udpExtStateImageType1->StateImage1.IsEA2);
        ROS_INFO("IsEA3  : %d\r\n", udpExtStateImageType1->StateImage1.IsEA3);
        ROS_INFO("IsEA4  : %d\r\n", udpExtStateImageType1->StateImage1.IsEA4);
    }
    else if (udpTelegramType->Id == 6) {
        RSL400::PUdpBeamPacket t3 = (RSL400::PUdpBeamPacket)_receive_buffer;

        // output some data
        ROS_INFO("BlockNo: %d\r\n", t3->BlockNo);
    }
}

int Rsl400UdpNode::recv(char *msg, size_t max_size)
{
    return ::recv(_socket, msg, max_size, 0);
}

int Rsl400UdpNode::timed_recv(char *msg, size_t max_size, int max_wait_ms)
{
    fd_set s;
    FD_ZERO(&s);
    FD_SET(_socket, &s);
    struct timeval timeout;
    timeout.tv_sec = max_wait_ms / 1000;
    timeout.tv_usec = (max_wait_ms % 1000) * 1000;
    int retval = select(_socket + 1, &s, &s, &s, &timeout);
    if(retval == -1)
    {
        // select() set errno accordingly
        return -1;
    }
    if(retval > 0)
    {
        // our socket has data
        return ::recv(_socket, msg, max_size, 0);
    }

    // our socket has no data
    errno = EAGAIN;
    return -1;
}

int Rsl400UdpNode::run()
{
    ros::Rate clock_rate(_poll_rate);  // Hz
    while (ros::ok())
    {
        clock_rate.sleep();
        publishScanDatagram();
        ros::spinOnce();
    }
    return 0;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rsl400_udp_ros");
    ros::NodeHandle nh;
    Rsl400UdpNode node(&nh);
    return node.run();
}
