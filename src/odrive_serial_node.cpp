#include <rclcpp/rclcpp.hpp>
#include <libserialport.h>
#include <string>
#include <vector>
#include <regex>
#include <sstream>
#include <cstring>

#include "odrive_interface/msg/odrive_command.hpp"
#include "odrive_interface/msg/odrive_feedback.hpp"

#define NUM_ODRIVES 1
#define BUFFER_SIZE 256

// ODrive IDs for steering and throttle
static const std::vector<int> steering_odrive_ids = {0};
static const std::vector<int> throttle_odrive_ids = {0};

// Serial port configuration
static const std::string ODRIVE_SERIAL_PORT_NAME = "/dev/ttyUSB0";
static const int ODRIVE_SERIAL_BAUD_RATE = 115200;

using odrive_interface::msg::OdriveCommand;
using odrive_interface::msg::OdriveFeedback;


class ODriveSerialNode : public rclcpp::Node {
public:
  ODriveSerialNode()
  : Node("odrive_serial_node"), port_(nullptr) {
    // Initialize the serial port
    sp_return error = sp_get_port_by_name(ODRIVE_SERIAL_PORT_NAME.c_str(), &port_);

    if (error != SP_OK) {
      RCLCPP_ERROR(this->get_logger(), "Failed to find serial port (%s): %s", ODRIVE_SERIAL_PORT_NAME.c_str(), sp_last_error_message());
      rclcpp::shutdown();
      return;
    }

    error = sp_open(port_, SP_MODE_READ_WRITE);
    if (error != SP_OK) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", sp_last_error_message());
      sp_free_port(port_);
      port_ = nullptr;
      rclcpp::shutdown();
      return;
    }

    // Configure serial port
    sp_set_baudrate(port_, ODRIVE_SERIAL_BAUD_RATE);
    sp_set_bits(port_, 8);
    sp_set_parity(port_, SP_PARITY_NONE);
    sp_set_stopbits(port_, 1);
    sp_set_flowcontrol(port_, SP_FLOWCONTROL_NONE);

    // Set up publishers/subscribers for each ODrive.
    for (int i = 0; i < NUM_ODRIVES; i++) {
      std::string topic_base = "/odrive_" + std::to_string(i);
      auto cmd_cb = [this, i](const OdriveCommand::SharedPtr msg) {
        this->sendCommand(i, msg->command_type, msg->value);
      };

      cmd_subs_.emplace_back(this->create_subscription<OdriveCommand>(
        topic_base + "/command", 10, cmd_cb
      ));

      auto pub = this->create_publisher<OdriveFeedback>(topic_base + "/feedback", 10);
      feedback_pubs_.push_back(pub);
    }

    // Steering command subscriber
    steering_cmd_sub_ = this->create_subscription<OdriveCommand>(
      "/steering", 10,
      [this](const OdriveCommand::SharedPtr msg) {
        this->sendGroupedCommand(steering_odrive_ids, msg->command_type, msg->value);
      });

    // Throttle command subscriber
    throttle_cmd_sub_ = this->create_subscription<OdriveCommand>(
      "/throttle", 10,
      [this](const OdriveCommand::SharedPtr msg) {
        this->sendGroupedCommand(throttle_odrive_ids, msg->command_type, msg->value);
      });

    // Timer to check for feedback from Arduino
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),
      [this]() {
        this->readSerial();
      }
    );

    RCLCPP_INFO(this->get_logger(), "ODrive Serial Node initialized.");
  }

  ~ODriveSerialNode() {
    if (port_) {
      sp_close(port_);
      sp_free_port(port_);
    }
  }

private:
  struct sp_port* port_;
  std::vector<rclcpp::Subscription<OdriveCommand>::SharedPtr> cmd_subs_;
  std::vector<rclcpp::Publisher<OdriveFeedback>::SharedPtr> feedback_pubs_;
  rclcpp::Subscription<OdriveCommand>::SharedPtr steering_cmd_sub_;
  rclcpp::Subscription<OdriveCommand>::SharedPtr throttle_cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string read_buffer_;

  void sendCommand(int index, const std::string& cmd_type, float value) {
    std::string cmd_char;
    if (cmd_type == "position") {
      cmd_char = "P";
    } else if (cmd_type == "velocity") {
      cmd_char = "V";
    } else {
      RCLCPP_WARN(this->get_logger(), "Unknown command_type: '%s'", cmd_type.c_str());
      return;
    }

    std::stringstream ss;
    ss << cmd_char << " " << index << " " << value << "\n";
    std::string cmd = ss.str();

    sp_nonblocking_write(port_, cmd.c_str(), cmd.length());
  }

  void sendGroupedCommand(const std::vector<int>& odrive_ids, const std::string& cmd_type, float value) {
    for (int id : odrive_ids) {
      if (id >= 0 && id < NUM_ODRIVES) {
        sendCommand(id, cmd_type, value);
      }
    }
  }

  void readSerial() {
    if (!port_) return;

    char buffer[BUFFER_SIZE];
    int bytes_read = sp_nonblocking_read(port_, buffer, BUFFER_SIZE - 1);

    if (bytes_read > 0) {
      buffer[bytes_read] = '\0';  // Null-terminate the string
      read_buffer_ += buffer;

      // Process any complete lines in the buffer
      size_t pos;
      while ((pos = read_buffer_.find('\n')) != std::string::npos) {
        std::string line = read_buffer_.substr(0, pos);
        read_buffer_ = read_buffer_.substr(pos + 1);

        if (!line.empty()) {
          parseFeedback(line);
        }
      }
    }
  }

  void parseFeedback(const std::string& line) {
    std::smatch match;
    std::regex regex(R"(OD(\d+): Pos=([-\d.]+) Vel=([-\d.]+))");

    if (std::regex_search(line, match, regex)) {
      int index = std::stoi(match[1]);
      if (index < 0 || index >= NUM_ODRIVES) return;

      OdriveFeedback msg;
      msg.position = std::stof(match[2]);
      msg.velocity = std::stof(match[3]);

      feedback_pubs_[index]->publish(msg);
    }
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ODriveSerialNode>());
  rclcpp::shutdown();
  return 0;
}
