#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "robotis_controller_msgs/msg/sync_write_item.hpp"
#include "robotis_controller_msgs/srv/set_module.hpp"
#include "op3_walking_module_msgs/msg/walking_param.hpp"

#include "op3_football_msgs/msg/joint_tick_array.hpp"
#include "op3_football_msgs/srv/joint_write.hpp"
#include "op3_football_msgs/srv/joint_read.hpp"
#include "op3_football_msgs/srv/joint_write_many.hpp"
#include "op3_football_msgs/srv/set_module.hpp"
#include "op3_football_msgs/srv/walking_command.hpp"
#include "op3_football_msgs/srv/set_walking_params.hpp"
#include "op3_football_msgs/srv/empty_trigger.hpp"
#include "op3_football_msgs/srv/set_led.hpp"
#include "op3_football_msgs/srv/set_torque.hpp"

#include "op3_football_l1/joint_map.hpp"

using op3_football_l1::clampTick;
using op3_football_l1::jointName;
using op3_football_l1::nameToId;
using op3_football_l1::radianToTick;
using op3_football_l1::tickToRadian;

class FootballBridge : public rclcpp::Node
{
public:
  FootballBridge()
  : Node("op3_football_bridge")
  {
    present_pub_ = create_publisher<op3_football_msgs::msg::JointTickArray>(
      "/op3_football/joint_ticks", 10);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/op3_football/imu", 10);
    button_pub_ = create_publisher<std_msgs::msg::String>("/op3_football/button", 10);

    joint_cmd_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      "/robotis/direct_control/set_joint_states", 10);
    enable_module_pub_ = create_publisher<std_msgs::msg::String>(
      "/robotis/enable_ctrl_module", 10);
    walking_cmd_pub_ = create_publisher<std_msgs::msg::String>(
      "/robotis/walking/command", 10);
    walking_param_pub_ = create_publisher<op3_walking_module_msgs::msg::WalkingParam>(
      "/robotis/walking/set_params", 10);
    ini_pose_pub_ = create_publisher<std_msgs::msg::String>(
      "/robotis/base/ini_pose", 10);
    torque_pub_ = create_publisher<std_msgs::msg::String>(
      "/robotis/dxl_torque", 10);
    sync_write_pub_ = create_publisher<robotis_controller_msgs::msg::SyncWriteItem>(
      "/robotis/sync_write_item", 10);
    head_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      "/robotis/head_control/set_joint_states", 10);

    set_module_client_ = create_client<robotis_controller_msgs::srv::SetModule>(
      "/robotis/set_present_ctrl_modules");

    present_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/robotis/present_joint_states", 10,
      std::bind(&FootballBridge::onPresentJoints, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/robotis/open_cr/imu", 10,
      std::bind(&FootballBridge::onImu, this, std::placeholders::_1));
    button_sub_ = create_subscription<std_msgs::msg::String>(
      "/robotis/open_cr/button", 10,
      std::bind(&FootballBridge::onButton, this, std::placeholders::_1));

    joint_write_srv_ = create_service<op3_football_msgs::srv::JointWrite>(
      "/op3_football/joint/write",
      std::bind(&FootballBridge::onJointWrite, this, std::placeholders::_1, std::placeholders::_2));
    joint_read_srv_ = create_service<op3_football_msgs::srv::JointRead>(
      "/op3_football/joint/read",
      std::bind(&FootballBridge::onJointRead, this, std::placeholders::_1, std::placeholders::_2));
    joint_write_many_srv_ = create_service<op3_football_msgs::srv::JointWriteMany>(
      "/op3_football/joint/write_many",
      std::bind(&FootballBridge::onJointWriteMany, this, std::placeholders::_1, std::placeholders::_2));
    set_module_srv_ = create_service<op3_football_msgs::srv::SetModule>(
      "/op3_football/module/set",
      std::bind(&FootballBridge::onSetModule, this, std::placeholders::_1, std::placeholders::_2));
    walking_cmd_srv_ = create_service<op3_football_msgs::srv::WalkingCommand>(
      "/op3_football/walking/command",
      std::bind(&FootballBridge::onWalkingCommand, this, std::placeholders::_1, std::placeholders::_2));
    walking_param_srv_ = create_service<op3_football_msgs::srv::SetWalkingParams>(
      "/op3_football/walking/set_params",
      std::bind(&FootballBridge::onSetWalkingParams, this, std::placeholders::_1, std::placeholders::_2));
    ini_pose_srv_ = create_service<op3_football_msgs::srv::EmptyTrigger>(
      "/op3_football/base/ini_pose",
      std::bind(&FootballBridge::onIniPose, this, std::placeholders::_1, std::placeholders::_2));
    estop_srv_ = create_service<op3_football_msgs::srv::EmptyTrigger>(
      "/op3_football/emergency_stop",
      std::bind(&FootballBridge::onEstop, this, std::placeholders::_1, std::placeholders::_2));
    led_srv_ = create_service<op3_football_msgs::srv::SetLed>(
      "/op3_football/led/set",
      std::bind(&FootballBridge::onSetLed, this, std::placeholders::_1, std::placeholders::_2));
    torque_srv_ = create_service<op3_football_msgs::srv::SetTorque>(
      "/op3_football/torque/set",
      std::bind(&FootballBridge::onSetTorque, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "op3_football L1 bridge ready");
  }

private:
  void onPresentJoints(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(joint_mutex_);
    present_rad_.clear();
    op3_football_msgs::msg::JointTickArray ticks;
    ticks.header = msg->header;

    for (size_t i = 0; i < msg->name.size(); ++i) {
      const auto & name = msg->name[i];
      const double rad = msg->position[i];
      present_rad_[name] = rad;

      auto it = nameToId().find(name);
      if (it == nameToId().end()) {
        continue;
      }
      ticks.ids.push_back(it->second);
      ticks.values.push_back(radianToTick(rad));
    }
    present_pub_->publish(ticks);
  }

  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    imu_pub_->publish(*msg);
  }

  void onButton(const std_msgs::msg::String::SharedPtr msg)
  {
    button_pub_->publish(*msg);
    // Hardware e-stop: mode button stops walking and cuts aggressive motion path
    if (msg->data == "mode" || msg->data == "user") {
      std_msgs::msg::String stop;
      stop.data = "stop";
      walking_cmd_pub_->publish(stop);
      RCLCPP_WARN(get_logger(), "Button '%s' treated as soft emergency stop (walk stop)",
                  msg->data.c_str());
    }
  }

  void publishJointGoals(
    const std::vector<int32_t> & ids, const std::vector<int32_t> & values)
  {
    sensor_msgs::msg::JointState js;
    js.header.stamp = now();
    bool has_body = false;
    bool has_head = false;
    sensor_msgs::msg::JointState head_js;
    head_js.header = js.header;

    for (size_t i = 0; i < ids.size(); ++i) {
      std::string name;
      if (!jointName(ids[i], name)) {
        continue;
      }
      const double rad = tickToRadian(clampTick(values[i]));
      if (name == "head_pan" || name == "head_tilt") {
        head_js.name.push_back(name);
        head_js.position.push_back(rad);
        has_head = true;
      } else {
        js.name.push_back(name);
        js.position.push_back(rad);
        has_body = true;
      }
    }

    if (has_body) {
      joint_cmd_pub_->publish(js);
    }
    if (has_head) {
      head_pub_->publish(head_js);
    }
  }

  void onJointWrite(
    const std::shared_ptr<op3_football_msgs::srv::JointWrite::Request> req,
    std::shared_ptr<op3_football_msgs::srv::JointWrite::Response> res)
  {
    std::string name;
    if (!jointName(req->id, name)) {
      res->success = false;
      res->message = "unknown joint id";
      return;
    }
    publishJointGoals({req->id}, {clampTick(req->value)});
    res->success = true;
    res->message = "ok";
  }

  void onJointRead(
    const std::shared_ptr<op3_football_msgs::srv::JointRead::Request> req,
    std::shared_ptr<op3_football_msgs::srv::JointRead::Response> res)
  {
    std::string name;
    if (!jointName(req->id, name)) {
      res->success = false;
      res->value = 0;
      res->message = "unknown joint id";
      return;
    }
    std::lock_guard<std::mutex> lock(joint_mutex_);
    auto it = present_rad_.find(name);
    if (it == present_rad_.end()) {
      res->success = false;
      res->value = 0;
      res->message = "joint state not received yet";
      return;
    }
    res->success = true;
    res->value = radianToTick(it->second);
    res->message = "ok";
  }

  void onJointWriteMany(
    const std::shared_ptr<op3_football_msgs::srv::JointWriteMany::Request> req,
    std::shared_ptr<op3_football_msgs::srv::JointWriteMany::Response> res)
  {
    if (req->ids.size() != req->values.size() || req->ids.empty()) {
      res->success = false;
      res->message = "ids/values size mismatch or empty";
      return;
    }
    publishJointGoals(req->ids, req->values);
    res->success = true;
    res->message = "ok";
  }

  void onSetModule(
    const std::shared_ptr<op3_football_msgs::srv::SetModule::Request> req,
    std::shared_ptr<op3_football_msgs::srv::SetModule::Response> res)
  {
    // Fast path used by demos
    std_msgs::msg::String msg;
    msg.data = req->module_name;
    enable_module_pub_->publish(msg);

    // Also try service for whole-body module switch
    if (set_module_client_->service_is_ready()) {
      auto request = std::make_shared<robotis_controller_msgs::srv::SetModule::Request>();
      request->module_name = req->module_name;
      set_module_client_->async_send_request(request);
    }

    res->success = true;
    res->message = "module request published: " + req->module_name;
  }

  void onWalkingCommand(
    const std::shared_ptr<op3_football_msgs::srv::WalkingCommand::Request> req,
    std::shared_ptr<op3_football_msgs::srv::WalkingCommand::Response> res)
  {
    if (estop_active_) {
      res->success = false;
      res->message = "emergency stop active";
      return;
    }
    std_msgs::msg::String msg;
    msg.data = req->command;
    walking_cmd_pub_->publish(msg);
    res->success = true;
    res->message = "ok";
  }

  void onSetWalkingParams(
    const std::shared_ptr<op3_football_msgs::srv::SetWalkingParams::Request> req,
    std::shared_ptr<op3_football_msgs::srv::SetWalkingParams::Response> res)
  {
    if (estop_active_) {
      res->success = false;
      res->message = "emergency stop active";
      return;
    }
    walking_param_pub_->publish(req->params);
    res->success = true;
    res->message = "ok";
  }

  void onIniPose(
    const std::shared_ptr<op3_football_msgs::srv::EmptyTrigger::Request> /*req*/,
    std::shared_ptr<op3_football_msgs::srv::EmptyTrigger::Response> res)
  {
    std_msgs::msg::String enable;
    enable.data = "base_module";
    enable_module_pub_->publish(enable);

    std_msgs::msg::String pose;
    pose.data = "ini_pose";
    ini_pose_pub_->publish(pose);
    res->success = true;
    res->message = "ini_pose requested";
  }

  void onEstop(
    const std::shared_ptr<op3_football_msgs::srv::EmptyTrigger::Request> /*req*/,
    std::shared_ptr<op3_football_msgs::srv::EmptyTrigger::Response> res)
  {
    estop_active_ = true;
    std_msgs::msg::String stop;
    stop.data = "stop";
    walking_cmd_pub_->publish(stop);
    RCLCPP_ERROR(get_logger(), "EMERGENCY STOP");
    res->success = true;
    res->message = "stopped walking; set estop_active";
  }

  void onSetLed(
    const std::shared_ptr<op3_football_msgs::srv::SetLed::Request> req,
    std::shared_ptr<op3_football_msgs::srv::SetLed::Response> res)
  {
    robotis_controller_msgs::msg::SyncWriteItem item;
    item.item_name = "LED";
    item.joint_name.push_back("open-cr");
    // Pack RGB into one value like demo (low bits)
    const int value = (req->blue & 0x1F) | ((req->green & 0x1F) << 5) | ((req->red & 0x1F) << 10);
    item.value.push_back(value);
    sync_write_pub_->publish(item);
    res->success = true;
    res->message = "ok";
  }

  void onSetTorque(
    const std::shared_ptr<op3_football_msgs::srv::SetTorque::Request> req,
    std::shared_ptr<op3_football_msgs::srv::SetTorque::Response> res)
  {
    std_msgs::msg::String msg;
    msg.data = req->command;
    torque_pub_->publish(msg);
    if (req->command == "on") {
      estop_active_ = false;
    }
    res->success = true;
    res->message = "ok";
  }

  std::mutex joint_mutex_;
  std::map<std::string, double> present_rad_;
  bool estop_active_{false};

  rclcpp::Publisher<op3_football_msgs::msg::JointTickArray>::SharedPtr present_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr button_pub_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr enable_module_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr walking_cmd_pub_;
  rclcpp::Publisher<op3_walking_module_msgs::msg::WalkingParam>::SharedPtr walking_param_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ini_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr torque_pub_;
  rclcpp::Publisher<robotis_controller_msgs::msg::SyncWriteItem>::SharedPtr sync_write_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr head_pub_;

  rclcpp::Client<robotis_controller_msgs::srv::SetModule>::SharedPtr set_module_client_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr present_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr button_sub_;

  rclcpp::Service<op3_football_msgs::srv::JointWrite>::SharedPtr joint_write_srv_;
  rclcpp::Service<op3_football_msgs::srv::JointRead>::SharedPtr joint_read_srv_;
  rclcpp::Service<op3_football_msgs::srv::JointWriteMany>::SharedPtr joint_write_many_srv_;
  rclcpp::Service<op3_football_msgs::srv::SetModule>::SharedPtr set_module_srv_;
  rclcpp::Service<op3_football_msgs::srv::WalkingCommand>::SharedPtr walking_cmd_srv_;
  rclcpp::Service<op3_football_msgs::srv::SetWalkingParams>::SharedPtr walking_param_srv_;
  rclcpp::Service<op3_football_msgs::srv::EmptyTrigger>::SharedPtr ini_pose_srv_;
  rclcpp::Service<op3_football_msgs::srv::EmptyTrigger>::SharedPtr estop_srv_;
  rclcpp::Service<op3_football_msgs::srv::SetLed>::SharedPtr led_srv_;
  rclcpp::Service<op3_football_msgs::srv::SetTorque>::SharedPtr torque_srv_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FootballBridge>());
  rclcpp::shutdown();
  return 0;
}
