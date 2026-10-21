#include <nav_msgs/msg/odometry.hpp>
#include <traj_utils/msg/poly_traj.hpp>
#include <optimizer/poly_traj_utils.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <std_msgs/msg/empty.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <chrono>

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
rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr pos_cmd_pub;

quadrotor_msgs::msg::PositionCommand cmd;
double pos_gain[3] = {0, 0, 0};
double vel_gain[3] = {0, 0, 0};

bool receive_traj_ = false;
std::shared_ptr<poly_traj::Trajectory> traj_;
double traj_duration_;
rclcpp::Time start_time_{0, 0}; // RCL_ROS_TIME，与 node_->now() 同源可作差
int traj_id_;

// yaw control
double last_yaw_, last_yaw_dot_;
double time_forward_;

Eigen::Vector3d last_vel(Eigen::Vector3d::Zero()), last_acc(Eigen::Vector3d::Zero()), last_jerk(Eigen::Vector3d::Zero());
bool flag = false;
double jerk2_inter = 0, acc2_inter = 0;
int cnt = 0;
rclcpp::Time global_start_time{0, 0}; // RCL_ROS_TIME，与 node_->now() 同源可作差
int drone_id_;
std::string result_dir = "/home/zuzu/Documents/report/";
std::fstream result_file;
std::vector<Eigen::Vector3d> pos_vec_, vel_vec_, acc_vec_, jerk_vec_;
std::vector<double> time_vec_;

const std::vector<std::string> explode(const std::string& s, const char& c)
{
  std::string buff{""};
  std::vector<std::string> v;

  for(auto n:s)
  {
    if(n != c) buff+=n; else
    if(n == c && buff != "") { v.push_back(buff); buff = ""; }
  }
  if(buff != "") v.push_back(buff);

  return v;
}

void polyTrajCallback(const traj_utils::msg::PolyTraj::ConstSharedPtr &msg)
{
  if (msg->order != 5)
  {
    RCLCPP_ERROR(node_->get_logger(), "[traj_server] Only support trajectory order equals 5 now!");
    return;
  }
  if (msg->duration.size() * (msg->order + 1) != msg->coef_x.size())
  {
    RCLCPP_ERROR(node_->get_logger(), "[traj_server] WRONG trajectory parameters, ");
    return;
  }

  int piece_nums = msg->duration.size();
  std::vector<double> dura(piece_nums);
  std::vector<poly_traj::CoefficientMat> cMats(piece_nums);
  for (int i = 0; i < piece_nums; ++i)
  {
    int i6 = i * 6;
    cMats[i].row(0) << msg->coef_x[i6 + 0], msg->coef_x[i6 + 1], msg->coef_x[i6 + 2],
        msg->coef_x[i6 + 3], msg->coef_x[i6 + 4], msg->coef_x[i6 + 5];
    cMats[i].row(1) << msg->coef_y[i6 + 0], msg->coef_y[i6 + 1], msg->coef_y[i6 + 2],
        msg->coef_y[i6 + 3], msg->coef_y[i6 + 4], msg->coef_y[i6 + 5];
    cMats[i].row(2) << msg->coef_z[i6 + 0], msg->coef_z[i6 + 1], msg->coef_z[i6 + 2],
        msg->coef_z[i6 + 3], msg->coef_z[i6 + 4], msg->coef_z[i6 + 5];

    dura[i] = msg->duration[i];
  }

  traj_.reset(new poly_traj::Trajectory(dura, cMats));

  start_time_ = rclcpp::Time(msg->start_time);
  traj_duration_ = traj_->getTotalDuration();
  traj_id_ = msg->traj_id;

  receive_traj_ = true;
}

void finishCallback(const std_msgs::msg::Bool::ConstSharedPtr& msg) {
  if (msg->data == true) {
    RCLCPP_WARN(node_->get_logger(), "total_time, cnt, acc2_inter, jerk2_inter = %lf \t %d \t %lf \t %lf", (node_->now() - global_start_time).seconds(), cnt, acc2_inter, jerk2_inter);
    // RCLCPP_WARN(node_->get_logger(), "time.size = %d, %d, %d, %d", time_vec_.size(), vel_vec_.size(), acc_vec_.size(), jerk_vec_.size());
    auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    //转为字符串
    std::stringstream ss;
    ss << std::put_time(std::localtime(&t), "%Y-%m-%d-%H-%M-%S");
    std::string str_time = ss.str();
    result_file << str_time << "\n";
    double max_vel2 = 0;
    for (int i = 0; i < time_vec_.size(); i++) {
      double tmp_vel2 = (vel_vec_[i](0))*(vel_vec_[i](0)) + (vel_vec_[i](1))*(vel_vec_[i](1)) + (vel_vec_[i](2))*(vel_vec_[i](2));
      max_vel2 = (tmp_vel2 > max_vel2) ? tmp_vel2 : max_vel2;
    }
    result_file << "max_vel = " << sqrt(max_vel2) << "\n";
    result_file << "\n";
  }
}

void startCallback(const std_msgs::msg::Bool::ConstSharedPtr& msg) {
  if (msg->data == true) {
    RCLCPP_WARN(node_->get_logger(), "START!!!!");
    global_start_time = node_->now();
  }
}

std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, rclcpp::Time &time_now, rclcpp::Time &time_last)
{
  constexpr double PI = 3.1415926;
  constexpr double YAW_DOT_MAX_PER_SEC = PI;
  // constexpr double YAW_DOT_DOT_MAX_PER_SEC = PI;
  std::pair<double, double> yaw_yawdot(0, 0);
  double yaw = 0;
  double yawdot = 0;

  Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_
                            ? traj_->getPos(t_cur + time_forward_) - pos
                            : traj_->getPos(traj_duration_) - pos;
  double yaw_temp = dir.norm() > 0.1
                        ? atan2(dir(1), dir(0))
                        : last_yaw_;
  double max_yaw_change = YAW_DOT_MAX_PER_SEC * (time_now - time_last).seconds();
  if (yaw_temp - last_yaw_ > PI)
  {
    if (yaw_temp - last_yaw_ - 2 * PI < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).seconds();
    }
  }
  else if (yaw_temp - last_yaw_ < -PI)
  {
    if (yaw_temp - last_yaw_ + 2 * PI > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).seconds();
    }
  }
  else
  {
    if (yaw_temp - last_yaw_ < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else if (yaw_temp - last_yaw_ > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).seconds();
    }
  }

  if (fabs(yaw - last_yaw_) <= max_yaw_change)
    yaw = 0.5 * last_yaw_ + 0.5 * yaw; // nieve LPF
  yawdot = 0.5 * last_yaw_dot_ + 0.5 * yawdot;
  last_yaw_ = yaw;
  last_yaw_dot_ = yawdot;

  yaw_yawdot.first = yaw;
  yaw_yawdot.second = yawdot;

  return yaw_yawdot;
}

void cmdCallback()
{
  /* no publishing before receive traj_ */
  if (!receive_traj_)
    return;

  rclcpp::Time time_now = node_->now();
  double t_cur = (time_now - start_time_).seconds();


  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), jerk(Eigen::Vector3d::Zero()), pos_f;
  std::pair<double, double> yaw_yawdot(0, 0);

  static rclcpp::Time time_last = node_->now();
  if (flag == false) {
    flag = true;
  } else {
    cnt++;
    acc2_inter += last_acc.norm()*last_acc.norm()*(time_now-time_last).seconds();
    jerk2_inter += last_jerk.norm()*last_jerk.norm()*(time_now-time_last).seconds();

  }
  if (t_cur < traj_duration_ && t_cur >= 0.0)
  {
    pos = traj_->getPos(t_cur);
    vel = traj_->getVel(t_cur);
    acc = traj_->getAcc(t_cur);
    jerk = traj_->getJer(t_cur);

    /*** calculate yaw ***/
    yaw_yawdot = calculate_yaw(t_cur, pos, time_now, time_last);
    /*** calculate yaw ***/

    double tf = std::min(traj_duration_, t_cur + 2.0);
    pos_f = traj_->getPos(tf);
  }
  else if (t_cur >= traj_duration_)
  {
    /* hover when finish traj_ */
    pos = traj_->getPos(traj_duration_);
    vel.setZero();
    acc.setZero();

    yaw_yawdot.first = last_yaw_;
    yaw_yawdot.second = 0;

    pos_f = pos;
  }
  else
  {
    // std::cout << "[Traj server]: invalid time." << std::endl;
  }
  time_last = time_now;
  time_vec_.push_back((node_->now() - global_start_time).seconds());
  pos_vec_.push_back(pos);
  vel_vec_.push_back(vel);
  acc_vec_.push_back(acc);
  jerk_vec_.push_back(jerk);
  last_vel = vel;
  last_acc = acc;
  last_jerk = jerk;

  cmd.header.stamp = time_now;
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;

  cmd.position.x = pos(0);
  cmd.position.y = pos(1);
  cmd.position.z = pos(2);

  cmd.velocity.x = vel(0);
  cmd.velocity.y = vel(1);
  cmd.velocity.z = vel(2);

  cmd.acceleration.x = acc(0);
  cmd.acceleration.y = acc(1);
  cmd.acceleration.z = acc(2);

  cmd.yaw = yaw_yawdot.first;
  cmd.yaw_dot = yaw_yawdot.second;

  last_yaw_ = cmd.yaw;

  pos_cmd_pub->publish(cmd);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("traj_server");
  node_ = nh;
  // get drone num
  std::string name_drone = nh->get_fully_qualified_name();
  std::vector<std::string> v{explode(name_drone, '_')};
  // drone_id_ = std::stoi(v[1]);
  result_file.open(result_dir+v[1]+"_vaj.txt", std::ios::out);

  rclcpp::Subscription<traj_utils::msg::PolyTraj>::SharedPtr poly_traj_sub =
      nh->create_subscription<traj_utils::msg::PolyTraj>("planning/trajectory", 10, polyTrajCallback);
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reached_sub =
      nh->create_subscription<std_msgs::msg::Bool>("planning/finish", 10, finishCallback);
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_sub =
      nh->create_subscription<std_msgs::msg::Bool>("planning/start", 10, startCallback);
  pos_cmd_pub = nh->create_publisher<quadrotor_msgs::msg::PositionCommand>("/position_cmd", 50);

  rclcpp::TimerBase::SharedPtr cmd_timer =
      nh->create_wall_timer(std::chrono::duration<double>(0.01), cmdCallback);

  /* control parameter */
  cmd.kx[0] = pos_gain[0];
  cmd.kx[1] = pos_gain[1];
  cmd.kx[2] = pos_gain[2];

  cmd.kv[0] = vel_gain[0];
  cmd.kv[1] = vel_gain[1];
  cmd.kv[2] = vel_gain[2];

  time_forward_ = get_param(nh, "traj_server.time_forward", -1.0);
  last_yaw_ = 0.0;
  last_yaw_dot_ = 0.0;

  rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0)));

  RCLCPP_WARN(nh->get_logger(), "[Traj server]: ready.");

  // 单线程执行器（与 ROS1 ros::spin() 等价）
  rclcpp::spin(nh);
  rclcpp::shutdown();

  return 0;
}
