#ifndef MOCAP_NATNET_HANDLER_H
#define MOCAP_NATNET_HANDLER_H

#include <vector>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include <NatNetTypes.h>
#include <mocap_interfaces/msg/rigid_bodies.hpp>

class NatnetDriverNode: public rclcpp::Node
{
public:
    // Constructor
    NatnetDriverNode();

    // Functions to store and send rigid body data
    void storeRigidBodyMessage(double currentSecsSinceEpoch, sRigidBodyData* bodies_ptr, int nRigidBodies);
    void sendRigidBodyMessage(double currentSecsSinceEpoch, sRigidBodyData* bodies_ptr, int nRigidBodies);
    void sendDummyPos();

    // Getter methods
    std::string getServerAddress();
    std::string getMulticastAddress();
    int getConnectionType();
    uint16_t getServerCommandPort();
    uint16_t getServerDataPort();
    bool isRecordingRequested();
    std::string getRecordName();
    bool isDummyHandler();

private:
    // Publisher for RigidBodies messages
    rclcpp::Publisher<mocap_interfaces::msg::RigidBodies>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Variables to store rigid bodies data temporarily
    double currentSecsSinceEpoch_;
    sRigidBodyData* bodies_ptr_;
    int nRigidBodies_;

    // Internal methods
    void timer_callback();
    void declare_parameters();
    void initialize_publisher();
    void setup_timer();
};
 
#endif
