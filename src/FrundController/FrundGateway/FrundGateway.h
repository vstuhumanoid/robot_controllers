//
// Created by humanoid on 04.05.18.
//

#ifndef ROBOT_CONTROLLERS_FRUNDGATEWAY_H
#define ROBOT_CONTROLLERS_FRUNDGATEWAY_H

#include <ros/ros.h>
#include <QUdpSocket>
#include <mutex>

class FrundGateway
{
public:
    FrundGateway();
    bool init(uint16_t frund_port, uint16_t frund_runner_port, std::string frund_runner_address);
    ~FrundGateway();
    bool ReceivePacket(char* frund_packet, int packet_size);
    void SendPacket(char* frund_packet, int packet_size);
    bool RunModel(std::string model_name);
    void StopModel();
    void SendParams(std::string model_name, std::string params);

private:
    QUdpSocket socket_data_;
    QUdpSocket socket_commands_;
    uint16_t port_data_;
    uint16_t port_commands_;
    QHostAddress frund_host_;
    quint16 frund_port_;
    QHostAddress address_commands_;

    std::mutex locker_;
};


#endif //ROBOT_CONTROLLERS_FRUNDGATEWAY_H
