#include <string>
#include <chrono>
#include <vector>
#include <algorithm>

#include <NatNetHandlerInterface.h>
#include <NatNetClientInterface.h>

// Constructor
NatnetDriverNode::NatnetDriverNode() : Node("natnet_client")
{
    this->declare_parameters();
    this->initialize_publisher();
    this->setup_timer();
}

// Declare parameters
void NatnetDriverNode::declare_parameters()
{
    this->declare_parameter<bool>("dummy_send", true);
    this->declare_parameter<int>("send_rate", 120);
    this->declare_parameter<int>("number_of_bodies", 1);
    this->declare_parameter<float>("dummy_x", 0.0);
    this->declare_parameter<float>("dummy_y", 0.0);
    this->declare_parameter<float>("dummy_z", 0.0);
    this->declare_parameter<float>("dummy_qx", 0.0);
    this->declare_parameter<float>("dummy_qy", 0.0);
    this->declare_parameter<float>("dummy_qz", 0.0);
    this->declare_parameter<float>("dummy_qw", 1.0);
    this->declare_parameter<std::string>("server_address", "0.0.0.0");
    this->declare_parameter<std::string>("multicast_address", "239.255.42.99");
    this->declare_parameter<int>("connection_type", 0);
    this->declare_parameter<uint16_t>("server_command_port", 1510);
    this->declare_parameter<uint16_t>("server_data_port", 1511);
    this->declare_parameter<std::string>("pub_topic", "rigid_body_topic");
    this->declare_parameter<bool>("record", false);
    this->declare_parameter<std::string>("record_file_name", "");
}

// Initialize publisher
void NatnetDriverNode::initialize_publisher()
{
    std::string topic;
    this->get_parameter("pub_topic", topic);
    this->publisher_ = this->create_publisher<mocap_interfaces::msg::RigidBodies>(topic.c_str(), 10);
}

// Timer to control publish rate
void NatnetDriverNode::setup_timer()
{
    int rate;
    this->get_parameter("send_rate", rate);
    std::chrono::milliseconds durationInMilliseconds(static_cast<int>(1000.0 / rate));
    //auto durationInMilliseconds = std::chrono::milliseconds(static_cast<int>(1000.0 / rate));
    this->timer_ = this->create_wall_timer(durationInMilliseconds, std::bind(&NatnetDriverNode::timer_callback, this));
}

// ID sorting
bool SortById(const sRigidBodyData body1, const sRigidBodyData body2)
{
    return body1.ID < body2.ID;
}

// Store Rigid body info
void NatnetDriverNode::storeRigidBodyMessage(double currentSecsSinceEpoch, sRigidBodyData* bodies_ptr, int nRigidBodies)
{
  currentSecsSinceEpoch_ = currentSecsSinceEpoch;
  nRigidBodies_ = nRigidBodies;
  bodies_ptr_ = bodies_ptr;
}
  
// Send Rigid body info
void NatnetDriverNode::sendRigidBodyMessage(double currentSecsSinceEpoch, sRigidBodyData* bodies_ptr, int nRigidBodies)
{
  // Sort bodies by ID
  std::vector<sRigidBodyData> bodies;
  for(int i=0; i < nRigidBodies; i++) 
  {
    bodies.push_back(bodies_ptr[i]);
  }
  std::sort(bodies.begin(), bodies.end(), SortById);
  
  // Convert seconds since epoch for time stamp
  if(this->isDummyHandler()) // update the time for dummy in case it is enabled
  {
    currentSecsSinceEpoch = this->get_clock()->now().seconds();
  }
  int64_t currentNanoSecsSinceEpoch = int64_t(currentSecsSinceEpoch * 1e9);
  rclcpp::Time currentTime = rclcpp::Time(currentNanoSecsSinceEpoch);

  mocap_interfaces::msg::RigidBodies msg;
  
  for(int i=0; i < nRigidBodies; i++)
  {      
    mocap_interfaces::msg::RigidBody rb;

    rb.id = bodies[i].ID;  
    rb.pose_stamped.pose.position.x = bodies[i].x;
    rb.pose_stamped.pose.position.y = bodies[i].y;
    rb.pose_stamped.pose.position.z = bodies[i].z;
    rb.pose_stamped.pose.orientation.x = bodies[i].qx;
    rb.pose_stamped.pose.orientation.y = bodies[i].qy;
    rb.pose_stamped.pose.orientation.z = bodies[i].qz;
    rb.pose_stamped.pose.orientation.w = bodies[i].qw;
    rb.mean_error = bodies[i].MeanError;
    rb.tracking = bodies[i].params & 0x01;
    rb.pose_stamped.header.stamp = currentTime;
    rb.pose_stamped.header.frame_id = "mocap";
    
    msg.rigid_bodies.push_back(rb);
  }
  // Publish
  publisher_->publish(msg);
}

// Timer callback
void NatnetDriverNode::timer_callback()
{
  this->sendRigidBodyMessage(currentSecsSinceEpoch_, bodies_ptr_, nRigidBodies_);
}

// Send dummy data for debugging
void NatnetDriverNode::sendDummyPos()
{
    int nRigidBodies_;
    this->get_parameter("number_of_bodies", nRigidBodies_);

    float dummy_x_, dummy_y_, dummy_z_, dummy_qx_, dummy_qy_, dummy_qz_, dummy_qw_;
    this->get_parameter("dummy_x", dummy_x_);
    this->get_parameter("dummy_y", dummy_y_);
    this->get_parameter("dummy_z", dummy_z_);
    this->get_parameter("dummy_qx", dummy_qx_);
    this->get_parameter("dummy_qy", dummy_qy_);
    this->get_parameter("dummy_qz", dummy_qz_);
    this->get_parameter("dummy_qw", dummy_qw_);

    sRigidBodyData* bodies = new sRigidBodyData[nRigidBodies_];

    for (int i = 0; i < nRigidBodies_; i++)
    {
      bodies[i].x = dummy_x_;
      bodies[i].y = dummy_y_;
      bodies[i].z = dummy_z_;
      bodies[i].qx = dummy_qx_;
      bodies[i].qy = dummy_qy_;
      bodies[i].qz = dummy_qz_;
      bodies[i].qw = dummy_qw_;
      bodies[i].MeanError = 0.0;
      bodies[i].ID = static_cast<unsigned short>(i);
      bodies[i].params = 1;
    }
    // Send data
    this->storeRigidBodyMessage(this->get_clock()->now().seconds(), bodies, nRigidBodies_);
}

// Handler for dummy data
bool NatnetDriverNode::isDummyHandler()
{
  bool dummy_ = false;
  this->get_parameter("dummy_send", dummy_);
  return dummy_;
}

// Get server address
std::string NatnetDriverNode::getServerAddress()
{
  std::string addr_;
  this->get_parameter("server_address", addr_);
  return addr_;
}

// Get multicast address
std::string NatnetDriverNode::getMulticastAddress()
{
  std::string addr_;
  this->get_parameter("multicast_address", addr_);
  return addr_;
}

// Get connection type
int NatnetDriverNode::getConnectionType()
{
  int type_ = 0;
  this->get_parameter("connection_type", type_);
  return type_;
}

// Get server command port
uint16_t NatnetDriverNode::getServerCommandPort()
{
  uint16_t port_;
  this->get_parameter("server_command_port", port_);
  return port_;
}

// Get server data port
uint16_t NatnetDriverNode::getServerDataPort()
{
  uint16_t port_;
  this->get_parameter("server_data_port", port_);
  return port_;
}

// Check if recording requested
bool NatnetDriverNode::isRecordingRequested()
{
  bool record_ = false;
  this->get_parameter("record", record_);
  return record_;
}

// Get record filename
std::string NatnetDriverNode::getRecordName()
{
  std::string recordName;
  this->get_parameter("record_file_name", recordName);

  std::string file_name;

  if (recordName.empty())
  {
    recordName = "record";
  }

  auto now = std::chrono::system_clock::now();
  auto nowTime = std::chrono::system_clock::to_time_t(now);

  std::stringstream ss;    
  ss << recordName << "_" << std::put_time(std::localtime(&nowTime), "%Y%m%d_%H%M%S");
  
  return ss.str();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Create the publisher node
  auto mocapPub = std::make_shared<NatnetDriverNode>();
  
  if(mocapPub->isDummyHandler())
    {
      RCLCPP_INFO(mocapPub->get_logger(), "########### Dummy handler ##############");
      mocapPub->sendDummyPos();
      rclcpp::spin(mocapPub);
    }
    else
    {
      RCLCPP_INFO(mocapPub->get_logger(), "########### Natnet handler #############");

      // Create the motion capture client
      MotionCaptureClient* c = new MotionCaptureClient(mocapPub.get());

      // Connect to server
      int retCode = c->connectToServer();
      if (retCode != 0)
      {
        return retCode;
      }

      rclcpp::spin(mocapPub);
      
      // Disconnect
      c->disconnectFromServer();
    }

  rclcpp::shutdown();
  return 0;
}
