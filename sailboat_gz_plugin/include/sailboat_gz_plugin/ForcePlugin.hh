#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>

namespace sailboat_gz_plugin
{

class ForcePlugin
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPreUpdate
{
public:

    void Configure(
        const gz::sim::Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        gz::sim::EntityComponentManager &_ecm,
        gz::sim::EventManager &_eventMgr
    ) override;

    void PreUpdate(
        const gz::sim::UpdateInfo &_info,
        gz::sim::EntityComponentManager &_ecm
    ) override;

private:

    // =====================================================
    // MODEL
    // =====================================================

    gz::sim::Model model{
        gz::sim::kNullEntity
    };

    gz::sim::Entity linkEntity{
        gz::sim::kNullEntity
    };

    // =====================================================
    // ROS
    // =====================================================

    rclcpp::Node::SharedPtr rosNode;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr
        windSpeedSub;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr
        windDirectionSub;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr
        sailSub;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr
        rudderSub;

    // =====================================================
    // STATE
    // =====================================================

    double windSpeed = 0.0;

    double windDirection = 0.0;

    double sailAngle = 0.0;

    double rudderAngle = 0.0;

    double yawRate = 0.0;
};

}