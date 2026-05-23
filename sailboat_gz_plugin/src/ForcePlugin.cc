#include "sailboat_gz_plugin/ForcePlugin.hh"

#include <gz/sim/Link.hh>

#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/AngularVelocityCmd.hh>

#include <gz/math/Vector3.hh>

#include <iostream>
#include <cmath>

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
        std::cerr
            << "Invalid model."
            << std::endl;

        return;
    }

    this->linkEntity =
        this->model.LinkByName(
            _ecm,
            "base_link"
        );

    if (this->linkEntity ==
        gz::sim::kNullEntity)
    {
        std::cerr
            << "Could not find base_link."
            << std::endl;

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

    // WIND SPEED

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

    // WIND DIRECTION

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

    // SAIL

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

    // RUDDER

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
        << "ForcePlugin loaded."
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
    // ROS SPIN
    // =====================================================

    rclcpp::spin_some(
        this->rosNode
    );

    // =====================================================
    // LINK
    // =====================================================

    auto link =
        gz::sim::Link(
            this->linkEntity
        );

    // =====================================================
    // WORLD POSE
    // =====================================================

    auto poseComp =
        _ecm.Component<
            gz::sim::components::Pose
        >(this->linkEntity);

    if (!poseComp)
    {
        return;
    }

    auto pose =
        poseComp->Data();

    double yaw =
        pose.Rot().Yaw();

    // =====================================================
    // WORLD VELOCITY
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
    // ANGLE OF ATTACK
    // =====================================================

    double angleOfAttack =

        apparentWindDirection -
        yaw -
        this->sailAngle;

    angleOfAttack =
        std::atan2(
            std::sin(angleOfAttack),
            std::cos(angleOfAttack)
        );

// =====================================================
// PHYSICAL CONSTANTS
// =====================================================

double airDensity = 1.225;

double sailArea = 0.376;

// =====================================================
// LIFT / DRAG COEFFICIENTS
// =====================================================

// simple symmetric sail model

double liftCoefficient =
    std::sin(
        2.0 * angleOfAttack
    );

// always positive drag

double dragCoefficient =
    1.0 -
    std::cos(angleOfAttack);

// =====================================================
// DYNAMIC PRESSURE
// =====================================================

double dynamicPressure =

    0.5 *
    airDensity *
    apparentWindSpeed *
    apparentWindSpeed;

// =====================================================
// FORCE MAGNITUDES
// =====================================================

double liftForce =

    dynamicPressure *
    sailArea *
    liftCoefficient;

double dragForce =

    dynamicPressure *
    sailArea *
    dragCoefficient;

    // =====================================================
    // DRAG DIRECTION
    // =====================================================

    gz::math::Vector3d dragDirection(

        std::cos(apparentWindDirection),

        std::sin(apparentWindDirection),

        0.0
    );

    // =====================================================
    // LIFT DIRECTION
    // =====================================================

    double liftSign = 1.0;

    if (angleOfAttack < 0.0)
    {
        liftSign = -1.0;
    }

    double liftDirectionAngle =

        apparentWindDirection +
        liftSign * M_PI / 2.0;

    gz::math::Vector3d liftDirection(

        std::cos(liftDirectionAngle),

        std::sin(liftDirectionAngle),

        0.0
    );

    ///////////////////////////////////////////////////////////////////
    // // =====================================================
    // // TOTAL FORCE
    // // =====================================================

    // gz::math::Vector3d sailForce =

    //     liftDirection * liftForce +
    //     dragDirection * dragForce;
    ///////////////////////////////////////////////////////////////////

    // =====================================================
    // RAW SAIL FORCE
    // =====================================================

    gz::math::Vector3d rawSailForce =

        liftDirection * liftForce +
        dragDirection * dragForce;

    // =====================================================
    // BOAT DIRECTIONS
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
    // FORCE DECOMPOSITION
    // =====================================================

    double forwardComponent =
        rawSailForce.Dot(forwardDir);

    double sideComponent =
        rawSailForce.Dot(rightDir);

    // reduce sideways sail instability

    sideComponent *= 0.35;

    // reconstructed force

    gz::math::Vector3d sailForce =

        forwardDir * forwardComponent +
        rightDir * sideComponent;

    // =====================================================
    // APPLY FORCE
    // =====================================================

    link.AddWorldForce(
        _ecm,
        sailForce
    );

    // =====================================================
    // KEEL LATERAL DAMPING
    // =====================================================

    // boat right vector

    // gz::math::Vector3d rightDir(

    //     -std::sin(yaw),

    //     std::cos(yaw),

    //     0.0
    // );

    // lateral velocity

    double lateralSpeed =

        velocity.Dot(
            rightDir
        );

    // damping force

    double forwardSpeed =

        velocity.Dot(
            gz::math::Vector3d(
                std::cos(yaw),
                std::sin(yaw),
                0.0
            )
        );

    double keelEffectiveness =

        std::min(
            std::abs(forwardSpeed) / 2.0,
            1.0
        );

    double lateralResistance =

        std::tanh(
            std::abs(lateralSpeed) * 2.0
        );

    double keelStrength = 4.0;

    gz::math::Vector3d keelForce =

        -rightDir *

        lateralSpeed *

        keelStrength *

        lateralResistance *

        keelEffectiveness;

    // apply

    link.AddWorldForce(
        _ecm,
        keelForce
    );

    // =====================================================
    // RUDDER YAW DYNAMICS
    // =====================================================

    // boat forward speed

    double rudderEffectiveness =

        std::min(
            std::abs(forwardSpeed),
            3.0
        );

    // target yaw rate

    double targetYawRate =

        this->rudderAngle *
        rudderEffectiveness *
        2.5;

    // smooth yaw response

    double yawResponse = 2.0;

    this->yawRate +=

        (
            targetYawRate -
            this->yawRate
        ) *

        yawResponse *

        _info.dt.count() *
        1e-9;

    // set angular velocity command

    auto angVelCmd =

        _ecm.Component<
            gz::sim::components::AngularVelocityCmd
        >(this->linkEntity);

    if (!angVelCmd)
    {
        _ecm.CreateComponent(

            this->linkEntity,

            gz::sim::components::AngularVelocityCmd(

                gz::math::Vector3d(
                    0.0,
                    0.0,
                    this->yawRate
                )
            )
        );
    }
    else
    {
        angVelCmd->Data() =

            gz::math::Vector3d(
                0.0,
                0.0,
                this->yawRate
            );
    }

    // =====================================================
    // DEBUG
    // =====================================================

    static int counter = 0;

    counter++;

    if (counter > 200)
    {
        counter = 0;

        std::cout

            << "aws="
            << apparentWindSpeed

            << " aoa="
            << angleOfAttack

            << " lift="
            << liftForce

            << " drag="
            << dragForce

            << std::endl;
    }
}