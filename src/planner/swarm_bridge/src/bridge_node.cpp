#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <iostream>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/empty.hpp>
#include <traj_utils/msg/poly_traj.hpp>

#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#define UDP_PORT 8081
#define BUF_LEN 1048576    // 1MB
#define BUF_LEN_SHORT 1024 // 1KB

using namespace std;

// ROS1: node_.param< T >("a/b", v, def) — ROS2: declare-then-get helper, name '/'→'.'
template <typename T>
T get_param(rclcpp::Node::SharedPtr node, const std::string &name, const T &def)
{
  T val = def;
  if (!node->has_parameter(name))
    node->declare_parameter<T>(name, def);
  node->get_parameter(name, val);
  return val;
}

rclcpp::Node::SharedPtr node_;

int udp_server_fd_, udp_send_fd_;
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr other_odoms_sub_;
rclcpp::Subscription<traj_utils::msg::PolyTraj>::SharedPtr one_traj_sub_;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr other_odoms_pub_;
rclcpp::Publisher<traj_utils::msg::PolyTraj>::SharedPtr one_traj_pub_;
string udp_ip_;
int drone_id_;
double odom_broadcast_freq_;
char udp_recv_buf_[BUF_LEN], udp_send_buf_[BUF_LEN];
struct sockaddr_in addr_udp_send_;
nav_msgs::msg::Odometry odom_msg_;
traj_utils::msg::PolyTraj polytraj_msg_;

enum MESSAGE_TYPE
{
  ODOM = 100,
  MULTI_TRAJ,
  ONE_TRAJ
} massage_type_;

int init_broadcast(const char *ip, const int port)
{
  int fd;

  if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) <= 0)
  {
    RCLCPP_ERROR(node_->get_logger(), "[bridge_node]Socket sender creation error!");
    exit(EXIT_FAILURE);
  }

  int so_broadcast = 1;
  if (setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &so_broadcast, sizeof(so_broadcast)) < 0)
  {
    cout << "Error in setting Broadcast option";
    exit(EXIT_FAILURE);
  }

  addr_udp_send_.sin_family = AF_INET;
  addr_udp_send_.sin_port = htons(port);

  if (inet_pton(AF_INET, ip, &addr_udp_send_.sin_addr) <= 0)
  {
    printf("\nInvalid address/ Address not supported \n");
    return -1;
  }

  return fd;
}

int udp_bind_to_port(const int port, int &server_fd)
{
  struct sockaddr_in address;
  int opt = 1;

  // Creating socket file descriptor
  if ((server_fd = socket(AF_INET, SOCK_DGRAM, 0)) == 0)
  {
    perror("socket failed");
    exit(EXIT_FAILURE);
  }

  // Forcefully attaching socket to the port
  if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
                 &opt, sizeof(opt)))
  {
    perror("setsockopt");
    exit(EXIT_FAILURE);
  }
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = INADDR_ANY;
  address.sin_port = htons(port);

  // Forcefully attaching socket to the port
  if (bind(server_fd, (struct sockaddr *)&address,
           sizeof(address)) < 0)
  {
    perror("bind failed");
    exit(EXIT_FAILURE);
  }

  return server_fd;
}

// ROS1: ros::serialization::serialize/deserialize — ROS2: rclcpp::Serialization.
// rmw_serialize does NOT grow the buffer, so the SerializedMessage is created
// with the required capacity upfront (deserialize also needs buffer_length set).
template <typename T>
int serializeTopic(const MESSAGE_TYPE msg_type, const T &msg)
{
  auto ptr = (uint8_t *)(udp_send_buf_);

  *((MESSAGE_TYPE *)ptr) = msg_type;
  ptr += sizeof(MESSAGE_TYPE);

  rclcpp::Serialization<T> serializer;
  // ROS1 serialized via zero-alloc stack streams; keep one buffer per sending
  // thread instead of allocating 1MB on every message
  static thread_local rclcpp::SerializedMessage smsg(BUF_LEN);
  serializer.serialize_message(&msg, &smsg);
  uint32_t msg_size = static_cast<uint32_t>(smsg.size());

  *((uint32_t *)ptr) = msg_size;
  ptr += sizeof(uint32_t);

  memcpy(ptr, smsg.get_rcl_serialized_message().buffer, msg_size);

  return msg_size + sizeof(MESSAGE_TYPE) + sizeof(uint32_t);
}

template <typename T>
int deserializeTopic(T &msg)
{
  auto ptr = (uint8_t *)(udp_recv_buf_ + sizeof(MESSAGE_TYPE));

  uint32_t msg_size = *((uint32_t *)ptr);
  ptr += sizeof(uint32_t);

  rclcpp::Serialization<T> serializer;
  rclcpp::SerializedMessage smsg(msg_size);
  memcpy(smsg.get_rcl_serialized_message().buffer, ptr, msg_size);
  smsg.get_rcl_serialized_message().buffer_length = msg_size;
  serializer.deserialize_message(&smsg, &msg);

  return msg_size + sizeof(MESSAGE_TYPE) + sizeof(uint32_t);
}

void odom_sub_udp_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
{

  // must match the node clock's RCL_ROS_TIME; default-constructed rclcpp::Time
  // is RCL_SYSTEM_TIME and subtracting the two throws
  static rclcpp::Time t_last(0, 0, RCL_ROS_TIME);
  rclcpp::Time t_now = node_->now();
  if ((t_now - t_last).seconds() * odom_broadcast_freq_ < 1.0)
  {
    return;
  }
  t_last = t_now;

  msg->child_frame_id = string("drone_") + std::to_string(drone_id_);

  int len = serializeTopic(MESSAGE_TYPE::ODOM, *msg);

  if (sendto(udp_send_fd_, udp_send_buf_, len, 0, (struct sockaddr *)&addr_udp_send_, sizeof(addr_udp_send_)) <= 0)
  {
    RCLCPP_ERROR(node_->get_logger(), "UDP SEND ERROR (1)!!!");
  }
}

void one_traj_sub_udp_cb(const traj_utils::msg::PolyTraj::SharedPtr msg)
{

  int len = serializeTopic(MESSAGE_TYPE::ONE_TRAJ, *msg);

  if (sendto(udp_send_fd_, udp_send_buf_, len, 0, (struct sockaddr *)&addr_udp_send_, sizeof(addr_udp_send_)) <= 0)
  {
    RCLCPP_ERROR(node_->get_logger(), "UDP SEND ERROR (3)!!!");
  }
}

void udp_recv_fun()
{
  int valread;
  struct sockaddr_in addr_client;
  socklen_t addr_len;

  // Connect
  if (udp_bind_to_port(UDP_PORT, udp_server_fd_) < 0)
  {
    RCLCPP_ERROR(node_->get_logger(), "[bridge_node]Socket recever creation error!");
    exit(EXIT_FAILURE);
  }

  while (true)
  {
    if ((valread = recvfrom(udp_server_fd_, udp_recv_buf_, BUF_LEN, 0, (struct sockaddr *)&addr_client, (socklen_t *)&addr_len)) < 0)
    {
      perror("recvfrom() < 0, error:");
      exit(EXIT_FAILURE);
    }

    char *ptr = udp_recv_buf_;
    switch (*((MESSAGE_TYPE *)ptr))
    {

    case MESSAGE_TYPE::ODOM:
    {
      if (valread == deserializeTopic(odom_msg_))
      {
        other_odoms_pub_->publish(odom_msg_);
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), "Received message length not matches the sent one (2)!!!");
        continue;
      }

      break;
    }

    case MESSAGE_TYPE::ONE_TRAJ:
    {

      if (valread == deserializeTopic(polytraj_msg_))
      {
        one_traj_pub_->publish(polytraj_msg_);
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), "Received message length not matches the sent one (3)!!!");
        continue;
      }

      break;
    }

    default:

      RCLCPP_ERROR(node_->get_logger(), "Unknown received message type???");

      break;
    }
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("swarm_bridge");
  node_ = nh;

  udp_ip_ = get_param(nh, "broadcast_ip", string("127.0.0.255"));
  drone_id_ = get_param(nh, "drone_id", -1);
  odom_broadcast_freq_ = get_param(nh, "odom_max_freq", 1000.0);

  if (drone_id_ == -1)
  {
    RCLCPP_WARN(nh->get_logger(), "[swarm bridge] Wrong drone_id!");
    exit(EXIT_FAILURE);
  }

  // ROS1: TransportHints().tcpNoDelay() — TCPROS-only concept, no ROS2 equivalent
  other_odoms_sub_ = nh->create_subscription<nav_msgs::msg::Odometry>("my_odom", 10, odom_sub_udp_cb);
  other_odoms_pub_ = nh->create_publisher<nav_msgs::msg::Odometry>("/others_odom", 10);

  one_traj_sub_ = nh->create_subscription<traj_utils::msg::PolyTraj>("/broadcast_traj_from_planner", 100, one_traj_sub_udp_cb);
  one_traj_pub_ = nh->create_publisher<traj_utils::msg::PolyTraj>("/broadcast_traj_to_planner", 100);

  std::thread udp_recv_thd(udp_recv_fun);
  udp_recv_thd.detach();
  rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.1)));

  // UDP connect
  udp_send_fd_ = init_broadcast(udp_ip_.c_str(), UDP_PORT);

  cout << "[rosmsg_tcp_bridge] start running" << endl;

  rclcpp::spin(nh);
  rclcpp::shutdown();

  close(udp_server_fd_);
  close(udp_send_fd_);

  return 0;
}
