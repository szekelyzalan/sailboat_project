#include "sailboat_gz_plugin/ForcePlugin.hh"

#include <gz/sim/Link.hh>

#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/LinearVelocity.hh>

#include <gz/math/Vector3.hh>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>

#include <iostream>
#include <cmath>
#include <algorithm>

using namespace sailboat_gz_plugin;

//////////////////////////////////////////////////
void ForcePlugin::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &
)
{
    // =====================================================
    // MODEL
    // =====================================================

    this->model = gz::sim::Model(_entity);

    if (!this->model.Valid(_ecm))
    {
        std::cerr << "Invalid model." << std::endl;
        return;
    }

    this->linkEntity =
        this->model.LinkByName(
            _ecm,
            "base_link"
        );

    if (this->linkEntity == gz::sim::kNullEntity)
    {
        std::cerr << "Could not find base_link." << std::endl;
        return;
    }

    // =====================================================
    // ROS
    // =====================================================

    if (!rclcpp::ok())
    {
        rclcpp::init(0, nullptr);
    }

    this->rosNode =
        rclcpp::Node::make_shared(
            "force_plugin_node"
        );

    // =====================================================
    // WIND SPEED
    // =====================================================

    this->windSpeedSub =
        this->rosNode->create_subscription
        <std_msgs::msg::Float32>(

            "/vrx/debug/wind/speed",

            10,

            [this](
                const std_msgs::msg::Float32::SharedPtr msg
            )
            {
                this->windSpeed = msg->data;
            }
        );

    // =====================================================
    // WIND DIRECTION
    // =====================================================

    this->windDirectionSub =
        this->rosNode->create_subscription
        <std_msgs::msg::Float32>(

            "/vrx/debug/wind/direction",

            10,

            [this](
                const std_msgs::msg::Float32::SharedPtr msg
            )
            {
                this->windDirection = msg->data;
            }
        );

    // =====================================================
    // SAIL
    // =====================================================

    this->sailSub =
        this->rosNode->create_subscription
        <std_msgs::msg::Float64>(

            "/baum_pos",

            10,

            [this](
                const std_msgs::msg::Float64::SharedPtr msg
            )
            {
                this->sailAngle = msg->data;
            }
        );

    // =====================================================
    // RUDDER
    // =====================================================

    this->rudderSub =
        this->rosNode->create_subscription
        <std_msgs::msg::Float64>(

            "/rudder_pos",

            10,

            [this](
                const std_msgs::msg::Float64::SharedPtr msg
            )
            {
                this->rudderAngle = msg->data;
            }
        );

    std::cout
        << "Minimal Sailboat Plugin Loaded"
        << std::endl;
}

//////////////////////////////////////////////////
void ForcePlugin::PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm
)
{
    if (_info.paused)
    {
        return;
    }

    // =====================================================
    // ROS
    // =====================================================

    rclcpp::spin_some(this->rosNode);

    // =====================================================
    // LINK
    // =====================================================

    auto link =
        gz::sim::Link(
            this->linkEntity
        );

    // =====================================================
    // POSE
    // =====================================================

    auto poseComp =
        _ecm.Component<
            gz::sim::components::Pose
        >(this->linkEntity);

    if (!poseComp)
    {
        return;
    }

    auto pose = poseComp->Data();

    double yaw =
        pose.Rot().Yaw();

    // =====================================================
    // VELOCITY
    // =====================================================

    auto velComp =
        _ecm.Component<
            gz::sim::components::LinearVelocity
        >(this->linkEntity);

    if (!velComp)
    {
        return;
    }

    auto velocity =
        velComp->Data();

    // =====================================================
    // BOAT AXES
    // =====================================================

    gz::math::Vector3d forwardDir(

        std::cos(yaw),
        std::sin(yaw),
        0.0
    );

    gz::math::Vector3d rightDir(

        -std::sin(yaw),
        std::cos(yaw),
        0.0
    );

    // =====================================================
    // BOAT SPEEDS
    // =====================================================

    double forwardSpeed =

        velocity.Dot(
            forwardDir
        );

    double sideSpeed =

        velocity.Dot(
            rightDir
        );

    double boatSpeed =
        velocity.Length();

    // =====================================================
    // TRUE WIND
    // =====================================================

    double windRad =

        this->windDirection *
        M_PI / 180.0;

    gz::math::Vector3d trueWind(

        this->windSpeed *
        std::cos(windRad),

        this->windSpeed *
        std::sin(windRad),

        0.0
    );

    // =====================================================
    // APPARENT WIND
    // =====================================================

    gz::math::Vector3d apparentWind =

        trueWind - velocity;

    double apparentWindSpeed =
        apparentWind.Length();

    double apparentWindDirection =

        std::atan2(
            apparentWind.Y(),
            apparentWind.X()
        );

    // =====================================================
    // RELATIVE WIND
    // =====================================================

    double relativeWind =

        apparentWindDirection -
        yaw;

    relativeWind =

        std::atan2(
            std::sin(relativeWind),
            std::cos(relativeWind)
        );

    // =====================================================
    // SAIL
    // =====================================================

    // sheet limit

    double sailLimit =
        std::abs(this->sailAngle);

    // sail freely aligns to wind
    // but limited by sheet

    double effectiveSailAngle =

        std::clamp(

            relativeWind,

            -sailLimit,

             sailLimit
        );

    // =====================================================
    // SAIL FORCE
    // =====================================================

    // sail normal direction

    double sailNormalAngle =

        yaw +
        effectiveSailAngle +
        M_PI / 2.0;

    gz::math::Vector3d sailForceDir(

        std::cos(sailNormalAngle),
        std::sin(sailNormalAngle),
        0.0
    );

    // how well wind hits sail

    double sailPower =

        std::sin(
            relativeWind -
            effectiveSailAngle
        );

    sailPower =
        std::abs(sailPower);

    // stable force model

    double sailForceMagnitude =

        apparentWindSpeed *
        apparentWindSpeed *
        sailPower *
        0.8;

    gz::math::Vector3d sailForce =

        sailForceDir *
        sailForceMagnitude;

    // apply at center

    link.AddWorldForce(

        _ecm,

        sailForce
    );

    // =====================================================
    // KEEL
    // =====================================================

    // simple lateral damping

    gz::math::Vector3d keelForce =

        -rightDir *
        sideSpeed *
        4.0;

    link.AddWorldForce(

        _ecm,

        keelForce
    );

    // =====================================================
    // WATER DRAG
    // =====================================================

    gz::math::Vector3d forwardDrag =

        -forwardDir *
        forwardSpeed *
        1.5;

    gz::math::Vector3d sideDrag =

        -rightDir *
        sideSpeed *
        3.0;

    link.AddWorldForce(

        _ecm,

        forwardDrag +
        sideDrag
    );

    // =====================================================
    // RUDDER
    // =====================================================

    double rudderStrength =

        forwardSpeed *
        3.0;

    gz::math::Vector3d rudderForce =

        rightDir *
        this->rudderAngle *
        rudderStrength;

    // apply behind boat

    gz::math::Vector3d rudderOffset(

        -0.45,
        0.0,
        0.0
    );

    gz::math::Vector3d worldRudderOffset(

        rudderOffset.X() * std::cos(yaw) -
        rudderOffset.Y() * std::sin(yaw),

        rudderOffset.X() * std::sin(yaw) +
        rudderOffset.Y() * std::cos(yaw),

        0.0
    );

    link.AddWorldForce(

        _ecm,

        rudderForce,

        pose.Pos() + worldRudderOffset
    );

    // =====================================================
    // DEBUG
    // =====================================================

    static int counter = 0;

    counter++;

    if (counter > 200)
    {
        counter = 0;

        std::cout

            << "\n========================"

            << "\nHEADING      = "
            << yaw * 180.0 / M_PI

            << "\n"

            << "\nTRUE WIND"

            << "\ndir           = "
            << this->windDirection

            << "\nspeed         = "
            << this->windSpeed

            << "\n"

            << "\nBOAT"

            << "\nforward speed = "
            << forwardSpeed

            << "\nside speed    = "
            << sideSpeed

            << "\nboat speed    = "
            << boatSpeed

            << "\n"

            << "\nAPPARENT WIND"

            << "\ndir           = "
            << apparentWindDirection * 180.0 / M_PI

            << "\nrelative      = "
            << relativeWind * 180.0 / M_PI

            << "\nspeed         = "
            << apparentWindSpeed

            << "\n"

            << "\nSAIL"

            << "\ncmd           = "
            << this->sailAngle * 180.0 / M_PI

            << "\neffective     = "
            << effectiveSailAngle * 180.0 / M_PI

            << "\npower         = "
            << sailPower

            << "\nforce         = "
            << sailForceMagnitude

            << "\n"

            << "\nRUDDER"

            << "\nangle         = "
            << this->rudderAngle

            << "\nforce         = "
            << rudderForce.Length()

            << "\n"

            << "========================"

            << std::endl;
    }
}