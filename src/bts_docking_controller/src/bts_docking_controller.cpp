#include<bts_docking_controller/bts_docking_controller.hpp>

Eigen::Vector3d quaternion_to_euler_ned(const Eigen::Quaterniond& q) {

    // Normalize the quat
    Eigen::Quaterniond qn = q.normalized();

    double w = qn.w();
    double x = qn.x();
    double y = qn.y();
    double z = qn.z();

    // Yaw (Z-axis rotation)
    double yaw = std::atan2(2.0 * (w*z + x*y), 1.0 - 2.0 * (y*y + z*z));

    // Pitch (Y-axis rotation)
    double sinp = 2.0 * (w*y - z*x);
    double pitch;
    if (std::abs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2.0, sinp); // use 90 degrees if out of range
    else
        pitch = std::asin(sinp);

    // Roll (X-axis rotation)
    double roll = std::atan2(2.0 * (w*x + y*z), 1.0 - 2.0 * (x*x + y*y));

    return Eigen::Vector3d(roll, pitch, yaw); // ZYX: yaw, pitch, roll
}

bts_docking_controller::
    bts_docking_controller()
    : Node("bts_docking_controller") {
    // Initialize PIDs
    pid_x_ = pid(0.3, 0.0, 0.1, 0.0, 0.2, -0.2);
    pid_y_ = pid(0.3, 0.0, 0.1, 0.0, 0.2, -0.2);
    pid_z_ = pid(1.0, 0.0, 0.1, 0.0, 0.2, -0.2);
    pid_yaw_ = pid(0.7, 0.0, 0.1, 0.0, 0.2, -0.2);
    pid_yaw_.set_ssa(true);

    // Topics
    desired_state_topic_ = "blueye/desired_state";
    current_state_topic_ = "/blueye/pose_estimated_board_stamped";
    command_topic_ = "blueye/commands";
    enable_docking_service_name_ = "blueye/enable_docking";

    // Create subscribers
    current_state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        current_state_topic_, 10,
        std::bind(&bts_docking_controller::
                    current_state_callback,
                this, std::placeholders::_1));

  desired_state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      desired_state_topic_, 10,
      std::bind(&bts_docking_controller::
                    desired_state_callback,
                this, std::placeholders::_1));

  // Create service for enabling docking
  enable_docking_service_ = this->create_service<std_srvs::srv::SetBool>(
      enable_docking_service_name_,
      std::bind(&bts_docking_controller::
                    enable_docking_callback,
                this, std::placeholders::_1, std::placeholders::_2));

  // Create publisher for commands
  command_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
      command_topic_, 10);

  // Create a timer to periodically send commands
  command_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(
          &bts_docking_controller::command_callback,
          this));

  // Initialize state vectors
  current_state_ = Eigen::VectorXd::Zero(9);
  desired_state_ = Eigen::VectorXd::Zero(9);

  RCLCPP_INFO(get_logger(),
              "Blueye Current Compensated Docking Controller initialized");
}

void bts_docking_controller::current_state_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {

  // Process current state
  current_state_ = Eigen::VectorXd(9);

  // Position
  current_state_(0) = msg->pose.pose.position.x;
  current_state_(1) = msg->pose.pose.position.y;
  current_state_(2) = msg->pose.pose.position.z;

  // Velocity
  current_state_(3) = msg->twist.twist.linear.x;
  current_state_(4) = msg->twist.twist.linear.y;
  current_state_(5) = msg->twist.twist.linear.z;

  // Angles
  auto euler_angles = quaternion_to_euler_ned(Eigen::Quaterniond(
      msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y, msg->pose.pose.orientation.z));

  current_state_(6) = euler_angles(0);
  current_state_(7) = euler_angles(1);
  current_state_(8) = euler_angles(2);
}

void bts_docking_controller::desired_state_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {

  // Position
  desired_state_(0) = msg->pose.pose.position.x;
  desired_state_(1) = msg->pose.pose.position.y;
  desired_state_(2) = msg->pose.pose.position.z;

  // Velocity
  desired_state_(3) = msg->twist.twist.linear.x;
  desired_state_(4) = msg->twist.twist.linear.y;
  desired_state_(5) = msg->twist.twist.linear.z;

  // Angles
  auto euler_angles = quaternion_to_euler_ned(Eigen::Quaterniond(
      msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y, msg->pose.pose.orientation.z));

  desired_state_(6) = euler_angles(0);
  desired_state_(7) = euler_angles(1);
  desired_state_(8) = euler_angles(2);
}

void bts_docking_controller::enable_docking_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
  docking_enabled_ = request->data;
  if (docking_enabled_) {
    RCLCPP_INFO(this->get_logger(), "Docking enabled");
    pid_x_.reset();
    pid_y_.reset();
    pid_z_.reset();
    pid_yaw_.reset();
    last_timestamp_ = now().seconds() + now().nanoseconds() * 1e-9;
  } else {
    RCLCPP_INFO(this->get_logger(), "Docking disabled");
  }
  response->success = true;
  response->message = docking_enabled_ ? "Docking enabled" : "Docking disabled";
}

void bts_docking_controller::command_callback() {
  if (!docking_enabled_) {
    return; // Do not send commands if docking is not enabled
  }

  // Time
  double current_time = now().seconds() + now().nanoseconds() * 1e-9;
  double dt = current_time - last_timestamp_;

  // Get current offset in x and y
  auto offset_x = desired_state_(0) - current_state_(0);
  auto offset_y = desired_state_(1) - current_state_(1);

  // Take into consideration the current heading
  auto offset_body_x =
      offset_x * cos(current_state_(8)) + offset_y * sin(current_state_(8));
  auto offset_body_y =
      -offset_x * sin(current_state_(8)) + offset_y * cos(current_state_(8));

  // Get output to each PID controller
  double output_x = pid_x_.run(offset_body_x, 0.0, dt);
  double output_y = pid_y_.run(offset_body_y, 0.0, dt);
  double output_z = pid_z_.run(desired_state_(2), current_state_(2), dt);
  double output_yaw = pid_yaw_.run(desired_state_(8), current_state_(8), dt);

  // Create command message
  geometry_msgs::msg::WrenchStamped command_msg;
  command_msg.header.stamp = now();
  command_msg.header.frame_id = "blueye_link"; // Adjust frame_id as needed

  // Set wrench values based on PID outputs
  command_msg.wrench.force.x = output_x; // Force in x direction
  command_msg.wrench.force.y = output_y; // Force in y direction
  command_msg.wrench.force.z = -output_z; //#- 0.4; // Force in z direction
  command_msg.wrench.torque.x = 0.0;     // No torque in
  command_msg.wrench.torque.y = 0.0;     // No torque in y direction
  command_msg.wrench.torque.z =
      -output_yaw; // Torque in z direction for yaw control

  // Publish the command message
  command_pub_->publish(command_msg);

  // Set last timestamp
  last_timestamp_ = current_time;

  // Publish the command
  command_pub_->publish(command_msg);
  last_timestamp_ = current_time;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<bts_docking_controller>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}