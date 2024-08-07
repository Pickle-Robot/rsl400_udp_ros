#include "rsl400_udp_ros/rsl400_udp_ros.h"

Rsl400UdpNode::Rsl400UdpNode(ros::NodeHandle *nodehandle) : nh(*nodehandle)
{
    ros::param::param<std::string>("~address", _address, "0.0.0.0");
    ros::param::param<int>("~port", _port, 9999);
    ros::param::param<double>("~poll_rate", _poll_rate, 1000.0);
    ros::param::param<double>("~socket_timeout", _socket_timeout, 3.0);
    ros::param::param<std::string>("~diagnostics_name", _diagnostics_name, "rsl400");
    ros::param::param<std::string>("~diagnostics_id", _diagnostics_id, "");

    double min_range, max_range;
    ros::param::param<double>("~min_range", min_range, 0.0);
    ros::param::param<double>("~max_range", max_range, 0x10000 * 0.001 - 1.0); // no data measurement == 65.536 meters

    std::string frame_id;
    ros::param::param<std::string>("~frame_id", frame_id, "laser");

    if (_diagnostics_id.empty())
    {
        _diagnostics_id = frame_id;
    }

    _receive_buffer = new char[BUFFER_LEN];
    _prev_scan_time = ros::Time::now();

    _socket = 0;

    _scan_msg.header.frame_id = frame_id;
    _scan_msg.range_min = min_range;
    _scan_msg.range_max = max_range;

    _scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 50);
    _diagnostics_pub = nh.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 50);
}

Rsl400UdpNode::~Rsl400UdpNode()
{
    close(_socket);
    delete[] _receive_buffer;
}

int Rsl400UdpNode::open_udp_socket(const std::string &addr, int port, struct addrinfo *addrinfo)
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
    if (error_code != 0 || addrinfo == NULL)
    {
        ROS_ERROR("Invalid address or port for UDP socket: \"%s:%s\"", addr.c_str(), decimal_port);
        throw std::runtime_error("Invalid address or port for UDP socket");
    }
    int f_socket = socket(addrinfo->ai_family, SOCK_DGRAM | SOCK_CLOEXEC, IPPROTO_UDP);
    if (f_socket == -1)
    {
        freeaddrinfo(addrinfo);
        ROS_ERROR("Could not create UDP socket with: \"%s:%s\"", addr.c_str(), decimal_port);
        throw std::runtime_error("Could not create UDP socket");
    }
    int reuse_flag = 1;
    if (setsockopt(f_socket, SOL_SOCKET, SO_REUSEADDR, &reuse_flag, sizeof(reuse_flag)) < 0)
    {
        throw std::runtime_error("failed to set reuseaddr");
    }
    error_code = bind(f_socket, addrinfo->ai_addr, addrinfo->ai_addrlen);
    if (error_code != 0)
    {
        freeaddrinfo(addrinfo);
        close(f_socket);
        ROS_ERROR("Error code: %d. Could not bind UDP socket with: \"%s:%s\"", error_code, addr.c_str(), decimal_port);
        throw std::runtime_error("Could not bind UDP socket");
    }

    struct timeval timeout;
    timeout.tv_sec = (int)_socket_timeout;
    timeout.tv_usec = (int)((_socket_timeout - (int)_socket_timeout) * 1000000);

    if (setsockopt(f_socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0)
    {
        throw std::runtime_error("failed to set socket timeout");
    }

    ROS_INFO("Opened UDP socket");
    return f_socket;
}

void Rsl400UdpNode::publish_scan()
{
    int recv_length = recv(_receive_buffer, BUFFER_LEN);
    if (recv_length <= 0)
    {
        return;
    }

    RSL400::PUdpTelegramType udpTelegramType = (RSL400::PUdpTelegramType)_receive_buffer;

    bool publish_laser = false;
    switch (udpTelegramType->Id)
    {
    case LeuzePacketId::BEAM_DESCRIPTION:
        publish_laser = handle_beam_description(_receive_buffer, recv_length);
        break;
    case LeuzePacketId::BEAM_STRENGTH_ID:
        publish_laser = handle_beam_strength_data(_receive_buffer, recv_length);
        break;
    case LeuzePacketId::BEAM_ID:
        publish_laser = handle_beam_data(_receive_buffer, recv_length);
        break;
    default:
        break;
    }

    if (publish_laser)
    {
        std::reverse(_scan_msg.ranges.begin(), _scan_msg.ranges.end());
        std::reverse(_scan_msg.intensities.begin(), _scan_msg.intensities.end());
        _scan_pub.publish(_scan_msg);
    }
}

int Rsl400UdpNode::get_num_beams(RSL400::PUdpTelegramType udpTelegramType, size_t data_type_size)
{
    int total_size = udpTelegramType->H1.TotalSize;

    // Compute the actual number of beams in this packet
    int num_beams = (total_size - sizeof(RSL400::UdpTelegramType)) / data_type_size;

    return num_beams;
}

/**
 * Computes the starting beam index for a packet and whether the scan is fully received.
 *
 * @param block_start Result parameter for the starting beam index for this packet.
 * @param udpTelegramType Header of the received packet.
 * @param data_type_size Size of the data type contained in the packet (Beam or BeamStrength).
 * @return True if the scan is fully received, false otherwise.
 */
bool Rsl400UdpNode::get_assignment_range(int *block_start, RSL400::PUdpTelegramType udpTelegramType, size_t data_type_size, int num_beams)
{
    // A single scan will be split into multiple packets if it exceeds the LEUZE_PREFERRED_PAYLOAD_LIMIT.
    // Using block number and _beam_count extracted from a previous description packet, we can determine
    // where to start assigning beams from this packet.

    // Extract current data packet info
    int block = udpTelegramType->BlockNo;

    // A beam == information from a single ray. Can be distance or distance with strength/intensity.
    // max_beams_per_packet contains how many beams we expect to find in this packet if it's full.
    int max_beams_per_packet = (LEUZE_PREFERRED_PAYLOAD_LIMIT - sizeof(RSL400::UdpTelegramType)) / data_type_size;
    *block_start = block * max_beams_per_packet;

    // Compute the expected number of packets for this scan
    int expected_num_packets = ceil((double)_beam_count / max_beams_per_packet);

    // If we expect to receive multiple packets, all packets except the last one should be full.
    if (block < expected_num_packets - 1 && num_beams != max_beams_per_packet)
    {
        ROS_WARN("Received packet size does not match beam count");
        return false;
    }

    // Given the expected number of packets compute the bitmask for a fully received scan
    int completion_bitmask = (1 << expected_num_packets) - 1;

    // Update the bitmask with the current packet
    _received_bitmask |= 1 << block;

    // Return true indicating the scan is complete if the bitmask is complete
    return completion_bitmask == _received_bitmask;
}

bool Rsl400UdpNode::handle_beam_description(char *receive_buffer, int length)
{
    ros::Time now = ros::Time::now();

    RSL400::PUdpExtStateImageType1 udpExtStateImageType1 = (RSL400::PUdpExtStateImageType1)receive_buffer;

    int total_size = udpExtStateImageType1->H1.TotalSize;
    if (total_size != length)
    {
        ROS_WARN("Received packet size does not match header size");
        return false;
    }

    diagnostic_msgs::DiagnosticStatus diagnostics;

    if (udpExtStateImageType1->StateImage1.IsEStopError ||
        udpExtStateImageType1->StateImage1.IsFieldPairError ||
        udpExtStateImageType1->StateImage1.IsEdm ||
        udpExtStateImageType1->StateImage1.IsScreen ||
        udpExtStateImageType1->StateImage1.IsAlarm ||
        udpExtStateImageType1->StateImage1.IsError)
    {
        diagnostics.level = diagnostic_msgs::DiagnosticStatus::ERROR;
    }
    else
    {
        diagnostics.level = diagnostic_msgs::DiagnosticStatus::OK;
    }
    diagnostics.name = _diagnostics_name;
    diagnostics.message = "";
    diagnostics.hardware_id = _diagnostics_id;

    diagnostics.values.push_back(make_entry("StatusProfile/IsOssdB", udpExtStateImageType1->StateImage1.IsOssdB));
    diagnostics.values.push_back(make_entry("StatusProfile/IsOssdA", udpExtStateImageType1->StateImage1.IsOssdA));
    diagnostics.values.push_back(make_entry("StatusProfile/IsEStopError", udpExtStateImageType1->StateImage1.IsEStopError));
    diagnostics.values.push_back(make_entry("StatusProfile/IsFieldPairError", udpExtStateImageType1->StateImage1.IsFieldPairError));
    diagnostics.values.push_back(make_entry("StatusProfile/IsEdm", udpExtStateImageType1->StateImage1.IsEdm));
    diagnostics.values.push_back(make_entry("StatusProfile/IsScreen", udpExtStateImageType1->StateImage1.IsScreen));
    diagnostics.values.push_back(make_entry("StatusProfile/IsAlarm", udpExtStateImageType1->StateImage1.IsAlarm));
    diagnostics.values.push_back(make_entry("StatusProfile/IsError", udpExtStateImageType1->StateImage1.IsError));

    diagnostics.values.push_back(make_entry("StatusProfile/b1_Res1", udpExtStateImageType1->StateImage1.b1_Res1));
    diagnostics.values.push_back(make_entry("StatusProfile/IsRflSeg4", udpExtStateImageType1->StateImage1.IsRflSeg4));
    diagnostics.values.push_back(make_entry("StatusProfile/IsRflSeg3", udpExtStateImageType1->StateImage1.IsRflSeg3));
    diagnostics.values.push_back(make_entry("StatusProfile/IsRflSeg2", udpExtStateImageType1->StateImage1.IsRflSeg2));
    diagnostics.values.push_back(make_entry("StatusProfile/IsRflSeg1", udpExtStateImageType1->StateImage1.IsRflSeg1));
    diagnostics.values.push_back(make_entry("StatusProfile/IsModePark", udpExtStateImageType1->StateImage1.IsModePark));
    diagnostics.values.push_back(make_entry("StatusProfile/IsStatusInputSeOn", udpExtStateImageType1->StateImage1.IsStatusInputSeOn));

    diagnostics.values.push_back(make_entry("StatusProfile/IsF8", udpExtStateImageType1->StateImage1.IsF8));
    diagnostics.values.push_back(make_entry("StatusProfile/IsF7", udpExtStateImageType1->StateImage1.IsF7));
    diagnostics.values.push_back(make_entry("StatusProfile/IsF6", udpExtStateImageType1->StateImage1.IsF6));
    diagnostics.values.push_back(make_entry("StatusProfile/IsF5", udpExtStateImageType1->StateImage1.IsF5));
    diagnostics.values.push_back(make_entry("StatusProfile/IsF4", udpExtStateImageType1->StateImage1.IsF4));
    diagnostics.values.push_back(make_entry("StatusProfile/IsF3", udpExtStateImageType1->StateImage1.IsF3));
    diagnostics.values.push_back(make_entry("StatusProfile/IsF2", udpExtStateImageType1->StateImage1.IsF2));
    diagnostics.values.push_back(make_entry("StatusProfile/IsF1", udpExtStateImageType1->StateImage1.IsF1));

    diagnostics.values.push_back(make_entry("StatusProfile/IsEA4", udpExtStateImageType1->StateImage1.IsEA4));
    diagnostics.values.push_back(make_entry("StatusProfile/IsEA3", udpExtStateImageType1->StateImage1.IsEA3));
    diagnostics.values.push_back(make_entry("StatusProfile/IsEA2", udpExtStateImageType1->StateImage1.IsEA2));
    diagnostics.values.push_back(make_entry("StatusProfile/IsEA1", udpExtStateImageType1->StateImage1.IsEA1));
    diagnostics.values.push_back(make_entry("StatusProfile/IsRes2", udpExtStateImageType1->StateImage1.IsRes2));
    diagnostics.values.push_back(make_entry("StatusProfile/IsRes1", udpExtStateImageType1->StateImage1.IsRes1));
    diagnostics.values.push_back(make_entry("StatusProfile/IsF10", udpExtStateImageType1->StateImage1.IsF10));
    diagnostics.values.push_back(make_entry("StatusProfile/IsF9", udpExtStateImageType1->StateImage1.IsF9));

    diagnostics.values.push_back(make_entry("StatusProfile/IsMELD", udpExtStateImageType1->StateImage1.IsMELD));
    diagnostics.values.push_back(make_entry("StatusProfile/IsA4", udpExtStateImageType1->StateImage1.IsA4));
    diagnostics.values.push_back(make_entry("StatusProfile/IsA3", udpExtStateImageType1->StateImage1.IsA3));
    diagnostics.values.push_back(make_entry("StatusProfile/IsA2", udpExtStateImageType1->StateImage1.IsA2));
    diagnostics.values.push_back(make_entry("StatusProfile/IsA1", udpExtStateImageType1->StateImage1.IsA1));
    diagnostics.values.push_back(make_entry("StatusProfile/IsNpnPnP", udpExtStateImageType1->StateImage1.IsNpnPnP));
    diagnostics.values.push_back(make_entry("StatusProfile/IsSE2", udpExtStateImageType1->StateImage1.IsSE2));
    diagnostics.values.push_back(make_entry("StatusProfile/IsSE1", udpExtStateImageType1->StateImage1.IsSE1));

    diagnostics.values.push_back(make_entry("StatusProfile/b5_Res1", udpExtStateImageType1->StateImage1.b5_Res1));

    diagnostics.values.push_back(make_entry("StatusProfile/ProtFuncA/b0_Res1", udpExtStateImageType1->StateImage1.ProtFuncA.b0_Res1));
    diagnostics.values.push_back(make_entry("StatusProfile/ProtFuncA/IsFFOutActive", udpExtStateImageType1->StateImage1.ProtFuncA.IsFFOutActive));
    diagnostics.values.push_back(make_entry("StatusProfile/ProtFuncA/IsRestartActive", udpExtStateImageType1->StateImage1.ProtFuncA.IsRestartActive));
    diagnostics.values.push_back(make_entry("StatusProfile/ProtFuncA/IsPfFree", udpExtStateImageType1->StateImage1.ProtFuncA.IsPfFree));
    diagnostics.values.push_back(make_entry("StatusProfile/ProtFuncA/IsWfFree", udpExtStateImageType1->StateImage1.ProtFuncA.IsWfFree));
    diagnostics.values.push_back(make_entry("StatusProfile/ProtFuncA/IsActive", udpExtStateImageType1->StateImage1.ProtFuncA.IsActive));

    diagnostics.values.push_back(make_entry("StatusProfile/ProtFuncA/SelFieldPair1", udpExtStateImageType1->StateImage1.ProtFuncA.SelFieldPair1));
    diagnostics.values.push_back(make_entry("StatusProfile/ProtFuncA/SelBank", udpExtStateImageType1->StateImage1.ProtFuncA.SelBank));

    diagnostics.values.push_back(make_entry("StatusProfile/ProtFuncA/SelFieldPair3", udpExtStateImageType1->StateImage1.ProtFuncA.SelFieldPair3));
    diagnostics.values.push_back(make_entry("StatusProfile/ProtFuncA/SelFieldPair2", udpExtStateImageType1->StateImage1.ProtFuncA.SelFieldPair2));

    diagnostics.values.push_back(make_entry("StatusProfile/ProtFuncA/b3_Res1", udpExtStateImageType1->StateImage1.ProtFuncA.b3_Res1));
    diagnostics.values.push_back(make_entry("StatusProfile/ProtFuncA/IsFpSel2", udpExtStateImageType1->StateImage1.ProtFuncA.IsFpSel2));
    diagnostics.values.push_back(make_entry("StatusProfile/ProtFuncA/IsFpSel1", udpExtStateImageType1->StateImage1.ProtFuncA.IsFpSel1));
    diagnostics.values.push_back(make_entry("StatusProfile/ProtFuncA/IsPfSeg2Free", udpExtStateImageType1->StateImage1.ProtFuncA.IsPfSeg2Free));
    diagnostics.values.push_back(make_entry("StatusProfile/ProtFuncA/IsPfSeg1Free", udpExtStateImageType1->StateImage1.ProtFuncA.IsPfSeg1Free));
    diagnostics.values.push_back(make_entry("StatusProfile/ProtFuncA/IsWfSeg2Free", udpExtStateImageType1->StateImage1.ProtFuncA.IsWfSeg2Free));
    diagnostics.values.push_back(make_entry("StatusProfile/ProtFuncA/IsWfSeg1Free", udpExtStateImageType1->StateImage1.ProtFuncA.IsWfSeg1Free));

    diagnostics.values.push_back(make_entry("StatusProfile/ProtFuncB/b0_Res1", udpExtStateImageType1->StateImage1.ProtFuncB.b0_Res1));
    diagnostics.values.push_back(make_entry("StatusProfile/ProtFuncB/IsFFOutActive", udpExtStateImageType1->StateImage1.ProtFuncB.IsFFOutActive));
    diagnostics.values.push_back(make_entry("StatusProfile/ProtFuncB/IsRestartActive", udpExtStateImageType1->StateImage1.ProtFuncB.IsRestartActive));
    diagnostics.values.push_back(make_entry("StatusProfile/ProtFuncB/IsPfFree", udpExtStateImageType1->StateImage1.ProtFuncB.IsPfFree));
    diagnostics.values.push_back(make_entry("StatusProfile/ProtFuncB/IsWfFree", udpExtStateImageType1->StateImage1.ProtFuncB.IsWfFree));
    diagnostics.values.push_back(make_entry("StatusProfile/ProtFuncB/IsActive", udpExtStateImageType1->StateImage1.ProtFuncB.IsActive));

    diagnostics.values.push_back(make_entry("StatusProfile/ProtFuncB/SelFieldPair1", udpExtStateImageType1->StateImage1.ProtFuncB.SelFieldPair1));
    diagnostics.values.push_back(make_entry("StatusProfile/ProtFuncB/SelBank", udpExtStateImageType1->StateImage1.ProtFuncB.SelBank));

    diagnostics.values.push_back(make_entry("StatusProfile/ProtFuncB/SelFieldPair3", udpExtStateImageType1->StateImage1.ProtFuncB.SelFieldPair3));
    diagnostics.values.push_back(make_entry("StatusProfile/ProtFuncB/SelFieldPair2", udpExtStateImageType1->StateImage1.ProtFuncB.SelFieldPair2));

    diagnostics.values.push_back(make_entry("StatusProfile/ProtFuncB/b3_Res1", udpExtStateImageType1->StateImage1.ProtFuncB.b3_Res1));
    diagnostics.values.push_back(make_entry("StatusProfile/ProtFuncB/IsFpSel2", udpExtStateImageType1->StateImage1.ProtFuncB.IsFpSel2));
    diagnostics.values.push_back(make_entry("StatusProfile/ProtFuncB/IsFpSel1", udpExtStateImageType1->StateImage1.ProtFuncB.IsFpSel1));
    diagnostics.values.push_back(make_entry("StatusProfile/ProtFuncB/IsPfSeg2Free", udpExtStateImageType1->StateImage1.ProtFuncB.IsPfSeg2Free));
    diagnostics.values.push_back(make_entry("StatusProfile/ProtFuncB/IsPfSeg1Free", udpExtStateImageType1->StateImage1.ProtFuncB.IsPfSeg1Free));
    diagnostics.values.push_back(make_entry("StatusProfile/ProtFuncB/IsWfSeg2Free", udpExtStateImageType1->StateImage1.ProtFuncB.IsWfSeg2Free));
    diagnostics.values.push_back(make_entry("StatusProfile/ProtFuncB/IsWfSeg1Free", udpExtStateImageType1->StateImage1.ProtFuncB.IsWfSeg1Free));

    diagnostics.values.push_back(make_entry("BeamDesc/Start", udpExtStateImageType1->BeamDesc.Start));
    diagnostics.values.push_back(make_entry("BeamDesc/Stop", udpExtStateImageType1->BeamDesc.Stop));
    diagnostics.values.push_back(make_entry("BeamDesc/Resolution", udpExtStateImageType1->BeamDesc.Resolution));
    diagnostics.values.push_back(make_entry("BeamDesc/Res", udpExtStateImageType1->BeamDesc.Res));

    if (length >= sizeof(RSL400::SignaturePacket))
    {
        RSL400::PSignaturePacket signaturePacket = (RSL400::PSignaturePacket)receive_buffer;
        diagnostics.values.push_back(make_entry("Signature/SignatureId", signaturePacket->Sig.SignatureId));
        diagnostics.values.push_back(make_entry("Signature/Length", signaturePacket->Sig.Length));
        diagnostics.values.push_back(make_buf_entry("Signature/Description", signaturePacket->Sig.Description, sizeof(signaturePacket->Sig.Description)));
    }
    diagnostic_msgs::DiagnosticArray diagnostics_array;
    diagnostics_array.status.push_back(diagnostics);

    _beam_count = RSL400::getBeamCount(&udpExtStateImageType1->BeamDesc);
    _received_bitmask = 0;
    int scan_count = udpExtStateImageType1->StateImage1.ScanNo;
    if (_scan_count == 0)
    {
        _scan_count = scan_count;
        _scan_msg.time_increment = 0.0;
    }
    else
    {
        int delta_count = scan_count - _scan_count;
        ros::Duration time_delta = now - _prev_scan_time;
        _scan_msg.time_increment = time_delta.toSec() / (delta_count * _beam_count);
    }

    _prev_scan_time = now;

    _scan_count = scan_count;
    if (_scan_msg.ranges.size() != _beam_count)
    {
        _scan_msg.ranges.resize(_beam_count);
        _scan_msg.intensities.resize(_beam_count);
    }
    double angle_start = decidegree_to_radians(udpExtStateImageType1->BeamDesc.Start);
    double angle_stop = decidegree_to_radians(udpExtStateImageType1->BeamDesc.Stop);
    double angle_delta = angle_stop - angle_start;
    _scan_msg.angle_min = -angle_delta / 2.0;
    _scan_msg.angle_max = angle_delta / 2.0;
    _scan_msg.angle_increment = decidegree_to_radians(udpExtStateImageType1->BeamDesc.Resolution);
    _scan_msg.header.stamp = now;

    diagnostics_array.header = _scan_msg.header;
    _diagnostics_pub.publish(diagnostics_array);

    return false;
}

bool Rsl400UdpNode::handle_beam_strength_data(char *receive_buffer, int length)
{
    if (_beam_count == 0)
    {
        ROS_DEBUG("Status message not received. Not publishing scan with intensities");
        return false;
    }

    RSL400::PUdpBeamStrengthPacket t3 = (RSL400::PUdpBeamStrengthPacket)_receive_buffer;
    RSL400::PUdpTelegramType udpTelegramType = (RSL400::PUdpTelegramType)_receive_buffer;

    int block_start;
    int num_beams = get_num_beams(udpTelegramType, sizeof(RSL400::BeamStrength));
    bool publish_laser = get_assignment_range(&block_start, udpTelegramType, sizeof(RSL400::BeamStrength), num_beams);

    for (int index = 0; index < num_beams; index++)
    {
        int block_index = index + block_start;
        _scan_msg.ranges[block_index] = (float)(t3->Beams[index].Distance) * 0.001;
        _scan_msg.intensities[block_index] = (float)(t3->Beams[index].Strength);
    }

    if (publish_laser)
    {
        _beam_count = 0;
    }
    return publish_laser;
}

bool Rsl400UdpNode::handle_beam_data(char *receive_buffer, int length)
{
    if (_beam_count == 0)
    {
        ROS_DEBUG("Status message not received. Not publishing scan");
        return false;
    }

    RSL400::PUdpBeamPacket t6 = (RSL400::PUdpBeamPacket)_receive_buffer;
    RSL400::PUdpTelegramType udpTelegramType = (RSL400::PUdpTelegramType)_receive_buffer;

    int block_start;
    int num_beams = get_num_beams(udpTelegramType, sizeof(RSL400::Beam));
    bool publish_laser = get_assignment_range(&block_start, udpTelegramType, sizeof(RSL400::Beam), num_beams);

    for (int index = 0; index < num_beams; index++)
    {
        _scan_msg.ranges[index + block_start] = (float)(t6->Beams[index].Distance) * 0.001;
    }

    if (publish_laser)
    {
        _beam_count = 0;
    }
    return publish_laser;
}

int Rsl400UdpNode::recv(char *msg, size_t max_size)
{
    return ::recv(_socket, msg, max_size, 0);
}

int Rsl400UdpNode::run()
{
    ros::Rate clock_rate(_poll_rate); // Hz
    struct addrinfo info;
    while (_socket == 0)
    {
        try
        {
            _socket = open_udp_socket(_address, _port, &info);
        }
        catch (const std::exception &e)
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rsl400_udp_ros");
    ros::NodeHandle nh;
    Rsl400UdpNode node(&nh);
    return node.run();
}
