#include "rsl400_udp_ros/rsl400_udp_ros.h"

Rsl400UdpNode::Rsl400UdpNode(ros::NodeHandle* nodehandle) :
    nh(*nodehandle)
{
    ros::param::param<std::string>("~address", _address, "0.0.0.0");
    ros::param::param<int>("~port", _port, 9999);
    ros::param::param<double>("~poll_rate", _poll_rate, 100.0);
    ros::param::param<std::string>("~frame_id", _frame_id, "laser");

    _receive_buffer = new char[BUFFER_LEN];
    _prev_scan_time = ros::Time::now();

    _scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 50);
    _diagnostics_pub = nh.advertise<diagnostic_msgs::DiagnosticStatus>("rsl400_diagnostics", 50);

    _socket = 0;
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
    const int enable = 1;
    if (setsockopt(f_socket, SOL_SOCKET, SO_REUSEPORT, &enable, sizeof(int)) < 0) {
        throw std::runtime_error("Failed to set socket option to reuse port");

    }
    if (setsockopt(f_socket, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0) {
        throw std::runtime_error("Failed to set socket option to reuse address");
    }
    error_code = bind(f_socket, addrinfo->ai_addr, addrinfo->ai_addrlen);
    if(error_code != 0)
    {
        freeaddrinfo(addrinfo);
        close(f_socket);
        throw std::runtime_error(("could not bind UDP socket with: \"" + addr + ":" + decimal_port + "\"").c_str());
    }
    ROS_INFO("Opened UDP socket");
    return f_socket;
}

void Rsl400UdpNode::publish_scan() {
    int recv_length = recv(_receive_buffer, BUFFER_LEN);
    if (recv_length <= 0) {
        return;
    }
    ros::Time now = ros::Time::now();

    RSL400::PUdpTelegramType udpTelegramType = (RSL400::PUdpTelegramType)_receive_buffer;

    bool publish_laser = false;

    if (udpTelegramType->Id == 1)
    {
        RSL400::PUdpExtStateImageType1 udpExtStateImageType1 = (RSL400::PUdpExtStateImageType1)_receive_buffer;

        int total_size = udpExtStateImageType1->H1.TotalSize;
        if (total_size != recv_length) {
            ROS_WARN("Received packet size does not match header size");
            return;
        }

        diagnostic_msgs::DiagnosticStatus diagnostics;

        if (udpExtStateImageType1->StateImage1.IsOssdB ||
            udpExtStateImageType1->StateImage1.IsOssdA ||
            udpExtStateImageType1->StateImage1.IsEStopError ||
            udpExtStateImageType1->StateImage1.IsFieldPairError ||
            udpExtStateImageType1->StateImage1.IsEdm ||
            udpExtStateImageType1->StateImage1.IsScreen ||
            udpExtStateImageType1->StateImage1.IsAlarm ||
            udpExtStateImageType1->StateImage1.IsError) {
            diagnostics.level = diagnostic_msgs::DiagnosticStatus::ERROR;
        }
        else {
            diagnostics.level = diagnostic_msgs::DiagnosticStatus::OK;
        }
        diagnostics.name = "rsl400";
        diagnostics.message = "";
        diagnostics.hardware_id = "";

        diagnostics.values.push_back(make_entry("StateImage1/ScanNo", udpExtStateImageType1->StateImage1.ScanNo));
        diagnostics.values.push_back(make_entry("StateImage1/IsOssdB", udpExtStateImageType1->StateImage1.IsOssdB));
        diagnostics.values.push_back(make_entry("StateImage1/IsOssdA", udpExtStateImageType1->StateImage1.IsOssdA));
        diagnostics.values.push_back(make_entry("StateImage1/IsEStopError", udpExtStateImageType1->StateImage1.IsEStopError));
        diagnostics.values.push_back(make_entry("StateImage1/IsFieldPairError", udpExtStateImageType1->StateImage1.IsFieldPairError));
        diagnostics.values.push_back(make_entry("StateImage1/IsEdm", udpExtStateImageType1->StateImage1.IsEdm));
        diagnostics.values.push_back(make_entry("StateImage1/IsScreen", udpExtStateImageType1->StateImage1.IsScreen));
        diagnostics.values.push_back(make_entry("StateImage1/IsAlarm", udpExtStateImageType1->StateImage1.IsAlarm));
        diagnostics.values.push_back(make_entry("StateImage1/IsError", udpExtStateImageType1->StateImage1.IsError));

        diagnostics.values.push_back(make_entry("BeamDesc/Start", udpExtStateImageType1->BeamDesc.Start));
        diagnostics.values.push_back(make_entry("BeamDesc/Stop", udpExtStateImageType1->BeamDesc.Stop));
        diagnostics.values.push_back(make_entry("BeamDesc/Resolution", udpExtStateImageType1->BeamDesc.Resolution));

        _diagnostics_pub.publish(diagnostics);

        int beam_count = RSL400::getBeamCount(&udpExtStateImageType1->BeamDesc);
        _beam_count = beam_count;
        _received_bitmask = 0;
        int scan_count = udpExtStateImageType1->StateImage1.ScanNo;
        if (_scan_count == 0) {
            _scan_count = scan_count;
            _time_increment = 0.0;
        }
        else {
            int delta_count = scan_count - _scan_count;
            ros::Duration time_delta = now - _prev_scan_time;
            _time_increment = time_delta.toSec() / (delta_count * _beam_count);
        }

        _prev_scan_time = now;

        _scan_count = scan_count;
        if (_ranges.size() < beam_count) {
            _ranges.resize(beam_count);
            _intensities.resize(beam_count);
        }
        _start_angle = decidegree_to_radians(udpExtStateImageType1->BeamDesc.Start);
        _stop_angle = decidegree_to_radians(udpExtStateImageType1->BeamDesc.Stop);
        _angle_increment = decidegree_to_radians(udpExtStateImageType1->BeamDesc.Resolution);
    }
    else if (udpTelegramType->Id == 3)
    {
        if (_beam_count == 0) {
            ROS_DEBUG("Status message not received. Not publishing scan with intensities");
            return;
        }

        RSL400::PUdpBeamStrengthPacket t3 = (RSL400::PUdpBeamStrengthPacket)_receive_buffer;

        int total_size = t3->H1.TotalSize;
        int block = (int)t3->BlockNo;
        int num_beams, block_start, completion_bitmask;
        
        if (!get_assignment_range(block, total_size, sizeof(RSL400::BeamStrength), _beam_count, num_beams, block_start, completion_bitmask)) {
            return;
        }

        for (int index = 0; index < num_beams; index++) {
            _ranges[index + block_start] = (float)(t3->Beams[index].Distance) * 0.001;
            _intensities[index + block_start] = (float)(t3->Beams[index].Strength);
        }
        if (completion_bitmask == _received_bitmask) {
            publish_laser = true;
            _beam_count = 0;
        }
    }
    else if (udpTelegramType->Id == 6)
    {
        if (_beam_count == 0) {
            ROS_DEBUG("Status message not received. Not publishing scan");
            return;
        }

        RSL400::PUdpBeamPacket t6 = (RSL400::PUdpBeamPacket)_receive_buffer;

        int total_size = t6->H1.TotalSize;
        int block = (int)t6->BlockNo;
        int num_beams, block_start, completion_bitmask;
        if (!get_assignment_range(block, total_size, sizeof(RSL400::Beam), _beam_count, num_beams, block_start, completion_bitmask)) {
            return;
        }

        for (int index = 0; index < _beam_count; index++) {
            _ranges[index + block_start] = (float)(t6->Beams[index].Distance) * 0.001;
        }

        if (completion_bitmask == _received_bitmask) {
            publish_laser = true;
            _beam_count = 0;
        }
    }

    if (publish_laser) {
        sensor_msgs::LaserScan scan_msg;
        scan_msg.header.frame_id = _frame_id;
        scan_msg.header.stamp = now;
        scan_msg.ranges = _ranges;
        scan_msg.intensities = _intensities;
        scan_msg.angle_min = _start_angle;
        scan_msg.angle_max = _stop_angle;
        scan_msg.angle_increment = _angle_increment;
        scan_msg.time_increment = _time_increment;
        scan_msg.range_min = 0.0;
        scan_msg.range_max = 0x10000 * 0.001;
        _scan_pub.publish(scan_msg);
    }
}

bool Rsl400UdpNode::get_assignment_range(int block, int total_size, size_t data_type_size, int beam_count, int &num_beams, int &block_start, int &completion_bitmask)
{
    int max_beams_per_packet = (LEUZE_PREFERRED_PAYLOAD_LIMIT - sizeof(RSL400::UdpTelegramType)) / data_type_size;
    block_start = block * max_beams_per_packet;
    num_beams = (total_size - sizeof(RSL400::UdpTelegramType)) / data_type_size;
    
    int expected_num_packets = ceil((double)beam_count / max_beams_per_packet);
    completion_bitmask = (1 << expected_num_packets) - 1;
    _received_bitmask |= 1 << block;
    
    if (expected_num_packets > 1 && block == 0 && num_beams != max_beams_per_packet) {
        ROS_WARN("Received packet size does not match beam count");
        return false;
    }
    else {
        return true;
    }
}

int Rsl400UdpNode::recv(char *msg, size_t max_size)
{
    return ::recv(_socket, msg, max_size, 0);
}

int Rsl400UdpNode::run()
{
    ros::Rate clock_rate(_poll_rate);  // Hz
    struct addrinfo info;
    while (_socket == 0) {
        try
        {
            _socket = open_udp_socket(_address, _port, &info);
        }
        catch (const std::exception& e)
        {
            ROS_ERROR("Failed to connect to device: %s", e.what());
            ros::spinOnce();
            ros::Duration(1.0).sleep();
            continue;
        }
    }
    while (ros::ok())
    {
        clock_rate.sleep();
        publish_scan();
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
