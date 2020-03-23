#include <roboteq_device.h>

#include <iostream>
#include <iomanip>
#include <vector>
#include <cstdlib>
#include <cstring>
#include <sstream>

#include <chrono>
#include <thread>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/Temperature.h>

class RoboteqNode {
  public:
    RoboteqNode(ros::NodeHandle &_nh, ros::NodeHandle &_pnh);
    ~RoboteqNode();

    void onMotorCommand(const std_msgs::Int16MultiArray::ConstPtr& msg);
    void spinOnce();

  private:
    unsigned long int seq;
    RoboteqDevice* roboteq;

    ros::NodeHandle nh;
    ros::NodeHandle nh_priv;

    ros::Publisher pub_brushless_count;
    ros::Publisher pub_brushless_speed;
    ros::Publisher pub_closed_loop_error;
    ros::Publisher pub_current;
    ros::Publisher pub_flags_fault;
    ros::Publisher pub_flags_status;
    ros::Publisher pub_flags_runtime;
    ros::Publisher pub_rotor_angle;
    ros::Publisher pub_temperature;
    ros::Publisher pub_voltage;

    ros::Subscriber sub_command;

    std::string param_port;
    int param_baud;
    int param_num_channels;
};

RoboteqNode::RoboteqNode(ros::NodeHandle &_nh, ros::NodeHandle &_nh_priv) {
  nh = _nh;
  nh_priv = _nh_priv;
  seq = 0;

  nh_priv.param("port", param_port, (std::string)"/dev/roboteq0");
  nh_priv.param("baud", param_baud, 115200);
  nh_priv.param("num_channels", param_num_channels, 2);

  roboteq = new RoboteqDevice(param_port, param_baud, param_num_channels);

  pub_brushless_count = nh.advertise<std_msgs::Int32MultiArray>("brushless_count", 1);
  pub_brushless_speed = nh.advertise<std_msgs::Float32MultiArray>("brushless_speed", 1);
  pub_current = nh.advertise<std_msgs::Float32MultiArray>("current", 1);
  pub_closed_loop_error = nh.advertise<std_msgs::Int32MultiArray>("closed_loop_error", 1);
  pub_flags_fault = nh.advertise<std_msgs::Int16>("flags_fault", 1);
  pub_flags_runtime = nh.advertise<std_msgs::Int16MultiArray>("flags_runtime", 1);
  pub_flags_status = nh.advertise<std_msgs::Int16>("flags_status", 1);
  pub_rotor_angle = nh.advertise<std_msgs::Float32MultiArray>("rotor_angle", 1);
  pub_temperature = nh.advertise<sensor_msgs::Temperature>("temperature", 1);
  pub_voltage = nh.advertise<std_msgs::Float32>("voltage", 1);

  sub_command = nh.subscribe<std_msgs::Int16MultiArray>("command", 1, &RoboteqNode::onMotorCommand, this);
}

RoboteqNode::~RoboteqNode() {
  delete roboteq;
}

void RoboteqNode::onMotorCommand(const std_msgs::Int16MultiArray::ConstPtr& msg){
    if(msg->data.size() != param_num_channels) {
      ROS_ERROR_STREAM("Bad command: received "<< msg->data.size() << " values, expected num_channels = " << param_num_channels);
      return;
    }

    for(int i=0;i<param_num_channels;i++) {
      if(msg->data[i] < -1000 || msg->data[i] > 1000) {
        ROS_ERROR_STREAM("Bad command: command values cannot exceed +/- 1000");
        return;
      }
    }

    for(int i=0;i<param_num_channels;i++) {
      roboteq->commandGo(i+1, msg->data[i]);
    }
}

void RoboteqNode::spinOnce() {
  ros::spinOnce();
  ros::Time time = ros::Time::now();
  uint64_t t = 1000 * (uint64_t)time.sec + (uint64_t)time.nsec / 1e6;

  if(pub_brushless_count.getNumSubscribers() > 0) {
    auto brushless_count = roboteq->getBrushlessCount();
    std_msgs::Int32MultiArray msg_brushless_count;
    msg_brushless_count.data = brushless_count;
    msg_brushless_count.layout.dim = std::vector<std_msgs::MultiArrayDimension>(1);
    msg_brushless_count.layout.dim[0].label = "data";
    msg_brushless_count.layout.dim[0].size = brushless_count.size();
    msg_brushless_count.layout.dim[0].stride = msg_brushless_count.layout.dim[0].size;
    pub_brushless_count.publish(msg_brushless_count);
  }

  if(pub_brushless_speed.getNumSubscribers() > 0) {
    auto brushless_speed = roboteq->getBrushlessSpeed();
    std_msgs::Float32MultiArray msg_brushless_speed;
    msg_brushless_speed.data = brushless_speed;
    msg_brushless_speed.layout.dim = std::vector<std_msgs::MultiArrayDimension>(1);
    msg_brushless_speed.layout.dim[0].label = "data";
    msg_brushless_speed.layout.dim[0].size = brushless_speed.size();
    msg_brushless_speed.layout.dim[0].stride = msg_brushless_speed.layout.dim[0].size;
    pub_brushless_speed.publish(msg_brushless_speed);
  }

  if(pub_current.getNumSubscribers() > 0) {
    auto current = roboteq->getCurrent();
    std_msgs::Float32MultiArray msg_current;
    msg_current.data = current;
    msg_current.layout.dim = std::vector<std_msgs::MultiArrayDimension>(1);
    msg_current.layout.dim[0].label = "data";
    msg_current.layout.dim[0].size = current.size();
    msg_current.layout.dim[0].stride = msg_current.layout.dim[0].size;
    pub_current.publish(msg_current);
  }

  if(pub_closed_loop_error.getNumSubscribers() > 0) {
    auto closed_loop_error = roboteq->getClosedLoopError();
    std_msgs::Int32MultiArray msg_closed_loop_error;
    msg_closed_loop_error.data = closed_loop_error;
    msg_closed_loop_error.layout.dim = std::vector<std_msgs::MultiArrayDimension>(1);
    msg_closed_loop_error.layout.dim[0].label = "data";
    msg_closed_loop_error.layout.dim[0].size = closed_loop_error.size();
    msg_closed_loop_error.layout.dim[0].stride = msg_closed_loop_error.layout.dim[0].size;
    pub_closed_loop_error.publish(msg_closed_loop_error);
  }

  if(pub_rotor_angle.getNumSubscribers() > 0) {
    auto rotor_angle = roboteq->getRotorAngle();
    std_msgs::Float32MultiArray msg_rotor_angle;
    msg_rotor_angle.data = rotor_angle;
    msg_rotor_angle.layout.dim = std::vector<std_msgs::MultiArrayDimension>(1);
    msg_rotor_angle.layout.dim[0].label = "data";
    msg_rotor_angle.layout.dim[0].size = rotor_angle.size();
    msg_rotor_angle.layout.dim[0].stride = msg_rotor_angle.layout.dim[0].size;
    pub_rotor_angle.publish(msg_rotor_angle);
  }

  if(seq % 5 == 0) {
    if(pub_voltage.getNumSubscribers() > 0) {
      auto voltage = roboteq->getVoltage();
      std_msgs::Float32 msg_voltage;
      msg_voltage.data = voltage[1];
      pub_voltage.publish(msg_voltage);
    }

    if(pub_flags_fault.getNumSubscribers() > 0) {
      auto flags_fault = roboteq->getFlagsFault();
      std_msgs::Int16 msg_flags_fault;
      msg_flags_fault.data = flags_fault;
      pub_flags_fault.publish(msg_flags_fault);
    }

    if(pub_flags_status.getNumSubscribers() > 0) {
      auto flags_status = roboteq->getFlagsStatus();
      std_msgs::Int16 msg_flags_status;
      msg_flags_status.data = flags_status;
      pub_flags_status.publish(msg_flags_status);
    }

    if(pub_flags_runtime.getNumSubscribers() > 0) {
      auto flags_runtime = roboteq->getFlagsRuntime();
      std_msgs::Int16MultiArray msg_flags_runtime;
      msg_flags_runtime.data = std::vector<short int>(flags_runtime.begin(), flags_runtime.end());
      msg_flags_runtime.layout.dim = std::vector<std_msgs::MultiArrayDimension>(1);
      msg_flags_runtime.layout.dim[0].label = "data";
      msg_flags_runtime.layout.dim[0].size = flags_runtime.size();
      msg_flags_runtime.layout.dim[0].stride = msg_flags_runtime.layout.dim[0].size;
      pub_flags_runtime.publish(msg_flags_runtime);
    }
  }

  if(seq % 30 == 30) {
    if(pub_temperature.getNumSubscribers() > 0) {
      auto temperature = roboteq->getTemperature();
      sensor_msgs::Temperature msg_temperature;
      msg_temperature.temperature = temperature[0];
      pub_temperature.publish(msg_temperature);
    }
  }

  seq++;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "roboteq_node");

    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    RoboteqNode roboteq_node(nh, nh_priv);

    ros::Rate(60);
    while(ros::ok()) {
      roboteq_node.spinOnce();
    }
}
