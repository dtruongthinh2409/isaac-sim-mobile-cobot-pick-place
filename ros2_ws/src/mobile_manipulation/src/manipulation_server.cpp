/**
 * Manipulation Server — ROS2 Service Server for Franka Panda arm control via MoveIt2
 *
 * Services:
 *   /manipulation/go_to_named_target  (GoToNamedTarget)  — move arm to named pose
 *   /manipulation/go_to_pose          (GoToPose)          — move arm to Cartesian pose
 *   /manipulation/open_gripper        (Trigger)           — open gripper
 *   /manipulation/close_gripper       (Trigger)           — close gripper
 *
 * Usage:
 *   ros2 launch mobile_manipulation manipulation_server.launch.py use_sim_time:=true
 *
 * Requires: move_group node running (from isaac_moveit.launch.py)
 */

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_srvs/srv/trigger.hpp>
#include "mobile_manipulation/srv/go_to_named_target.hpp"
#include "mobile_manipulation/srv/go_to_pose.hpp"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using Trigger = std_srvs::srv::Trigger;
using GoToNamedTarget = mobile_manipulation::srv::GoToNamedTarget;
using GoToPose = mobile_manipulation::srv::GoToPose;
using namespace std::placeholders;


class ManipulationServer
{
public:
    ManipulationServer(std::shared_ptr<rclcpp::Node> node) : node_(node)
    {
        RCLCPP_INFO(node_->get_logger(), "Initializing MoveGroupInterface for panda_arm...");
        arm_ = std::make_shared<MoveGroupInterface>(node_, "panda_arm");
        arm_->setMaxVelocityScalingFactor(0.3);
        arm_->setMaxAccelerationScalingFactor(0.3);
        arm_->setPlanningTime(10.0);

        RCLCPP_INFO(node_->get_logger(), "Initializing MoveGroupInterface for hand...");
        gripper_ = std::make_shared<MoveGroupInterface>(node_, "hand");

        // Create services
        named_target_srv_ = node_->create_service<GoToNamedTarget>(
            "manipulation/go_to_named_target",
            std::bind(&ManipulationServer::namedTargetCallback, this, _1, _2));

        pose_srv_ = node_->create_service<GoToPose>(
            "manipulation/go_to_pose",
            std::bind(&ManipulationServer::poseCallback, this, _1, _2));

        open_gripper_srv_ = node_->create_service<Trigger>(
            "manipulation/open_gripper",
            std::bind(&ManipulationServer::openGripperCallback, this, _1, _2));

        close_gripper_srv_ = node_->create_service<Trigger>(
            "manipulation/close_gripper",
            std::bind(&ManipulationServer::closeGripperCallback, this, _1, _2));

        get_pose_srv_ = node_->create_service<Trigger>(
            "manipulation/get_current_pose",
            std::bind(&ManipulationServer::getCurrentPoseCallback, this, _1, _2));

        RCLCPP_INFO(node_->get_logger(), "All manipulation services created.");
    }

private:
    bool planAndExecute(std::shared_ptr<MoveGroupInterface> &interface)
    {
        MoveGroupInterface::Plan plan;
        bool plan_ok = (interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (plan_ok) {
            auto exec_result = interface->execute(plan);
            return (exec_result == moveit::core::MoveItErrorCode::SUCCESS);
        }
        return false;
    }

    void namedTargetCallback(
        const std::shared_ptr<GoToNamedTarget::Request> request,
        std::shared_ptr<GoToNamedTarget::Response> response)
    {
        RCLCPP_INFO(node_->get_logger(), "Service call: go_to_named_target('%s')",
                    request->target_name.c_str());

        arm_->setStartStateToCurrentState();

        if (!arm_->setNamedTarget(request->target_name)) {
            response->success = false;
            response->message = "Unknown target name: " + request->target_name;
            RCLCPP_ERROR(node_->get_logger(), "%s", response->message.c_str());
            return;
        }

        bool ok = planAndExecute(arm_);
        response->success = ok;
        response->message = ok
            ? "Reached: " + request->target_name
            : "Failed to reach: " + request->target_name;
        RCLCPP_INFO(node_->get_logger(), "%s", response->message.c_str());
    }

    void poseCallback(
        const std::shared_ptr<GoToPose::Request> request,
        std::shared_ptr<GoToPose::Response> response)
    {
        auto &pose = request->target_pose;
        RCLCPP_INFO(node_->get_logger(),
                    "Service call: go_to_pose([%.3f, %.3f, %.3f], frame=%s, cartesian=%s)",
                    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
                    pose.header.frame_id.c_str(),
                    request->cartesian_path ? "true" : "false");

        arm_->setStartStateToCurrentState();

        if (request->cartesian_path) {
            // Cartesian path planning (straight line to target)
            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.push_back(pose.pose);
            moveit_msgs::msg::RobotTrajectory trajectory;

            double fraction = arm_->computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

            if (fraction >= 0.95) {
                auto exec_result = arm_->execute(trajectory);
                response->success = (exec_result == moveit::core::MoveItErrorCode::SUCCESS);
            } else {
                response->success = false;
                response->message = "Cartesian path planning failed (fraction=" +
                                    std::to_string(fraction) + ")";
                RCLCPP_WARN(node_->get_logger(), "%s", response->message.c_str());
                return;
            }
        } else {
            // Free-space motion planning
            arm_->setPoseTarget(pose);
            response->success = planAndExecute(arm_);
        }

        response->message = response->success ? "Reached pose" : "Failed to reach pose";
        RCLCPP_INFO(node_->get_logger(), "%s", response->message.c_str());
    }

    void openGripperCallback(
        const std::shared_ptr<Trigger::Request> /*request*/,
        std::shared_ptr<Trigger::Response> response)
    {
        RCLCPP_INFO(node_->get_logger(), "Service call: open_gripper");

        gripper_->setStartStateToCurrentState();
        gripper_->setNamedTarget("open");

        response->success = planAndExecute(gripper_);
        response->message = response->success ? "Gripper opened" : "Failed to open gripper";
        RCLCPP_INFO(node_->get_logger(), "%s", response->message.c_str());
    }

    void closeGripperCallback(
        const std::shared_ptr<Trigger::Request> /*request*/,
        std::shared_ptr<Trigger::Response> response)
    {
        RCLCPP_INFO(node_->get_logger(), "Service call: close_gripper");

        gripper_->setStartStateToCurrentState();
        gripper_->setNamedTarget("close");

        response->success = planAndExecute(gripper_);
        response->message = response->success ? "Gripper closed" : "Failed to close gripper";
        RCLCPP_INFO(node_->get_logger(), "%s", response->message.c_str());
    }

    void getCurrentPoseCallback(
        const std::shared_ptr<Trigger::Request> /*request*/,
        std::shared_ptr<Trigger::Response> response)
    {
        RCLCPP_INFO(node_->get_logger(), "Service call: get_current_pose");

        auto current_pose = arm_->getCurrentPose();
        auto &p = current_pose.pose;

        // Also get the end effector link name and planning frame
        std::string ee_link = arm_->getEndEffectorLink();
        std::string plan_frame = arm_->getPlanningFrame();

        std::string pose_str =
            "ee_link=" + ee_link + ", planning_frame=" + plan_frame +
            ", position=[" +
            std::to_string(p.position.x) + ", " +
            std::to_string(p.position.y) + ", " +
            std::to_string(p.position.z) + "], " +
            "orientation=[x=" +
            std::to_string(p.orientation.x) + ", y=" +
            std::to_string(p.orientation.y) + ", z=" +
            std::to_string(p.orientation.z) + ", w=" +
            std::to_string(p.orientation.w) + "]";

        response->success = true;
        response->message = pose_str;
        RCLCPP_INFO(node_->get_logger(), "Current pose: %s", pose_str.c_str());
    }

    // Members
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<MoveGroupInterface> arm_;
    std::shared_ptr<MoveGroupInterface> gripper_;

    rclcpp::Service<GoToNamedTarget>::SharedPtr named_target_srv_;
    rclcpp::Service<GoToPose>::SharedPtr pose_srv_;
    rclcpp::Service<Trigger>::SharedPtr open_gripper_srv_;
    rclcpp::Service<Trigger>::SharedPtr close_gripper_srv_;
    rclcpp::Service<Trigger>::SharedPtr get_pose_srv_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>(
        "manipulation_server",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    // MultiThreadedExecutor: allows service callbacks to run
    // while MoveGroupInterface processes internal callbacks
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    // Create server AFTER spinning starts (MoveGroupInterface needs spinning to get robot state)
    auto server = std::make_shared<ManipulationServer>(node);

    RCLCPP_INFO(node->get_logger(),
                "========================================");
    RCLCPP_INFO(node->get_logger(),
                "Manipulation server ready!");
    RCLCPP_INFO(node->get_logger(),
                "Services: go_to_named_target, go_to_pose, open_gripper, close_gripper");
    RCLCPP_INFO(node->get_logger(),
                "========================================");

    spinner.join();
    rclcpp::shutdown();
    return 0;
}
