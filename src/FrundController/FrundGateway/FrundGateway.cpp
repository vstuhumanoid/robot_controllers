//
// Created by humanoid on 04.05.18.
//

#include "FrundGateway.h"

FrundGateway::FrundGateway()
{

}

bool FrundGateway::Init(uint16_t frund_port, uint16_t frund_runner_port, std::string frund_runner_address)
{
    port_data_ = frund_port;
    port_commands_ = frund_runner_port;
    address_commands_.toIPv4Address();
    address_commands_.setAddress(QString::fromStdString(frund_runner_address));

    ros::NodeHandle nh;
    if(!socket_data_.bind(port_data_, QUdpSocket::ShareAddress))
    {
        ROS_ERROR("Frund socket not binded!");
        return false;
    }

    socket_commands_.connectToHost(QString::fromStdString(frund_runner_address), port_commands_);
    socket_commands_.waitForConnected(200);

    return true;
}

FrundGateway::~FrundGateway()
{
    socket_data_.close();
    socket_commands_.disconnect();
}

bool FrundGateway::ReceivePacket(char* frund_packet, int packet_size)
{
    if(!socket_data_.hasPendingDatagrams())
        return false;
    QByteArray datagram;
    if(socket_data_.pendingDatagramSize() < packet_size)
        return false;
    datagram.resize(socket_data_.pendingDatagramSize());
    socket_data_.readDatagram(datagram.data(), datagram.size(), &frund_host_, &frund_port_);
    memcpy(frund_packet, datagram.data(), packet_size);
    return true;
}

void FrundGateway::SendPacket(char *frund_packet, int packet_size)
{
    QByteArray datagram;
    datagram.resize(packet_size);
    memcpy(datagram.data(), frund_packet, packet_size);
    //datagram.fromRawData(frund_packet, packet_size);
    socket_data_.writeDatagram(datagram, frund_host_, frund_port_);
}

bool FrundGateway::RunModel(std::string model_name)
{
    std::lock_guard<std::mutex> lock(locker_);
    sendToRunner("1", 1);
    sendToRunner(model_name.data(), model_name.size());
    //socket_commands_.writeDatagram("1", 2, address_commands_, port_commands_);
    //socket_commands_.writeDatagram(model_name.data(), model_name.size(), address_commands_, port_commands_);
    sleep(1);
    if(!socket_commands_.hasPendingDatagrams())
        return false;
    QByteArray datagram;
    datagram.resize(socket_commands_.pendingDatagramSize());
    QHostAddress host;
    uint16_t port;
    socket_commands_.readDatagram(datagram.data(), datagram.size(), &host, &port);
    if(datagram.data()[0] == '1')
        return true;
    return false;
}

void FrundGateway::StopModel()
{
    std::lock_guard<std::mutex> lock(locker_);
    //socket_commands_.writeDatagram("2", 2, address_commands_, port_commands_);
    sendToRunner("2", 1);
}

void FrundGateway::SendParams(std::string model_name, std::string params)
{
    std::lock_guard<std::mutex> lock(locker_);
    //socket_commands_.writeDatagram("3", 2, address_commands_, port_commands_);
    //socket_commands_.writeDatagram(model_name.data(), model_name.size(), address_commands_, port_commands_);
    //socket_commands_.writeDatagram(params.data(), params.size(), address_commands_, port_commands_);
    sendToRunner("3", 1);
    sendToRunner(model_name.data(), model_name.size());
    sendToRunner(params.data(), params.size());
}

void FrundGateway::sendToRunner(const char *buffer, const int size)
{
    char message[size + 2] = { 0 };
    strcat(strcpy(message, buffer), "!");
    socket_commands_.writeDatagram(message, size+2, address_commands_, port_commands_);
}
