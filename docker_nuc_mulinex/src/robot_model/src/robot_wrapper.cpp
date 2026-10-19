#include "robot_model/robot_wrapper.hpp"

#include <array>
#include <cstdio>
#include <stdexcept>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/parsers/urdf.hpp"

#include <yaml-cpp/yaml.h>


namespace robot_model {

/* ============================== RobotWrapper ============================= */

RobotWrapper::RobotWrapper(const std::string& robot_name)
{
    // Location of the file containing some info on the robots
    const std::string file_path = ament_index_cpp::get_package_share_directory("robot_model") +
                                  std::string{"/robots/all_robots.yaml"};

    // Parse the yaml file
    YAML::Node config = YAML::LoadFile(file_path);
    YAML::Node robot = config[robot_name];

    if (!robot) {
        throw std::runtime_error(
            "RobotWrapper: robot '" + robot_name + "' not found in " + file_path
        );
    }

    std::string pkg_name = robot["pkg_name"].as<std::string>();
    const std::string package_share_directory =
        ament_index_cpp::get_package_share_directory(pkg_name);

    // Populate the urdf_path attribute
    this->urdf_path_ = package_share_directory + robot["urdf_path"].as<std::string>();

    this->base_name_ = robot["base_name"].as<std::string>();

    // Read n_joints_per_leg (default to 3 if not specified)
    if (robot["n_joints_per_leg"]) {
        this->n_joints_per_leg_ = robot["n_joints_per_leg"].as<int>();
    }

    // Populate the feet_names attribute (dynamically sized)
    const YAML::Node feet_node = robot["feet_names"];
    this->feet_names_.resize(feet_node.size());
    for (size_t i = 0; i < feet_node.size(); i++) {
        this->feet_names_[i] = feet_node[i].as<std::string>();
    }

    // Populate the joint_names attribute (dynamically sized)
    const YAML::Node joints_node = robot["ordered_joint_names"];
    this->joint_names_.resize(joints_node.size());
    for (size_t i = 0; i < joints_node.size(); i++) {
        this->joint_names_[i] = joints_node[i].as<std::string>();
    }

    // Load the urdf model
    const pinocchio::JointModelFreeFlyer root_joint;

    if (urdf_path_.find(".xacro") != std::string::npos) {
        // Build xacro command with urdf_args from YAML
        std::string cmd = "xacro " + urdf_path_;
        if (robot["urdf_args"]) {
            for (auto it = robot["urdf_args"].begin(); it != robot["urdf_args"].end(); ++it) {
                cmd += " " + it->first.as<std::string>() + ":=" + it->second.as<std::string>();
            }
        }

        // Execute xacro and capture the URDF XML output
        std::array<char, 4096> buffer;
        std::string urdf_xml;
        FILE* pipe = popen(cmd.c_str(), "r");
        if (!pipe) {
            throw std::runtime_error("RobotWrapper: failed to run xacro command: " + cmd);
        }
        while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe) != nullptr) {
            urdf_xml += buffer.data();
        }
        int ret = pclose(pipe);
        if (ret != 0) {
            throw std::runtime_error(
                "RobotWrapper: xacro command failed with exit code " + std::to_string(ret) + ": " +
                cmd
            );
        }

        pinocchio::urdf::buildModelFromXML(urdf_xml, root_joint, model_);
    } else {
        pinocchio::urdf::buildModel(urdf_path_, root_joint, model_);
    }

    // Create the data required by the algorithms
    this->data_ = pinocchio::Data(model_);
}

/* =============================== compute_EOM ============================== */

void RobotWrapper::compute_EOM(const Eigen::VectorXd& q, const Eigen::VectorXd& v)
{
    // Update the joint placements
    pinocchio::forwardKinematics(model_, data_, q);

    // Computes the full model Jacobian
    pinocchio::computeJointJacobians(model_, data_, q);

    // Update the frame placements
    pinocchio::updateFramePlacements(model_, data_);

    // Compute the upper part of the joint space inertia matrix
    pinocchio::crba(model_, data_, q);
    data_.M.triangularView<Eigen::StrictlyLower>() =
        data_.M.transpose().triangularView<Eigen::StrictlyLower>();

    // Compute the nonlinear effects vector (Coriolis, centrifugal and gravitational effects)
    pinocchio::nonLinearEffects(model_, data_, q, v);
}

/* ========================= compute_second_order_FK ======================== */

void RobotWrapper::compute_second_order_FK(const Eigen::VectorXd& q, const Eigen::VectorXd& v)
{
    // Update the joint accelerations
    pinocchio::forwardKinematics(model_, data_, q, v, Eigen::VectorXd::Zero(model_.nv));

    // Computes the full model Jacobian variations with respect to time
    pinocchio::computeJointJacobiansTimeVariation(model_, data_, q, v);
}

/* =============================== compute_Jc =============================== */

void RobotWrapper::get_Jc(Eigen::MatrixXd& Jc)
{
    // Initialize a temp Jacobian that must be used to store the contact jacobian of a contact foot.
    // Jc is the stack of J_temp of all the contact feet.
    Eigen::MatrixXd J_temp(6, model_.nv);

    // Compute the stack of the contact Jacobians
    for (size_t i = 0; i < contact_feet_names_.size(); i++) {
        pinocchio::FrameIndex frame_id = model_.getFrameId(contact_feet_names_[i]);
        J_temp.setZero();

        pinocchio::getFrameJacobian(
            model_, data_, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J_temp
        );

        Jc.block(3 * i, 0, 3, model_.nv) = J_temp.topRows(3);
    }
}

/* =============================== compute_Jb =============================== */

void RobotWrapper::get_Jb(Eigen::MatrixXd& Jb)
{
    Jb.setZero();

    pinocchio::FrameIndex base_id = 1;

    pinocchio::getFrameJacobian(model_, data_, base_id, pinocchio::LOCAL_WORLD_ALIGNED, Jb);
}

/* =============================== compute_Js =============================== */

void RobotWrapper::get_Js(Eigen::MatrixXd& Js)
{
    // Initialize a temp Jacobian that must be used to store the swing feet jacobian.
    // Js is the stack of J_temp of all the swing feet.
    Eigen::MatrixXd J_temp(6, model_.nv);

    // Compute the stack of the swing jacobians.
    for (size_t i = 0; i < swing_feet_names_.size(); i++) {
        pinocchio::FrameIndex frame_id = model_.getFrameId(swing_feet_names_[i]);

        J_temp.setZero();

        pinocchio::getFrameJacobian(
            model_, data_, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J_temp
        );

        Js.block(3 * i, 0, 3, model_.nv) = J_temp.topRows(3);
    }
}

/* ========================= compute_Jc_dot_times_v ========================= */

void RobotWrapper::get_Jc_dot_times_v(Eigen::VectorXd& Jc_dot_times_v)
{
    for (size_t i = 0; i < contact_feet_names_.size(); i++) {
        pinocchio::FrameIndex frame_id = model_.getFrameId(contact_feet_names_[i]);

        Jc_dot_times_v.segment(0 + 3 * i, 3) =
            pinocchio::getFrameClassicalAcceleration(
                model_, data_, frame_id, pinocchio::LOCAL_WORLD_ALIGNED
            )
                .linear();
    }
}

/* ========================= compute_Jb_dot_times_v ========================= */

void RobotWrapper::get_Jb_dot_times_v(Eigen::VectorXd& Jb_dot_times_v)
{
    pinocchio::FrameIndex base_id = 1;

    Jb_dot_times_v =
        pinocchio::getClassicalAcceleration(model_, data_, base_id, pinocchio::LOCAL_WORLD_ALIGNED)
            .toVector();
}

/* ========================= compute_Js_dot_times_v ========================= */

void RobotWrapper::get_Js_dot_times_v(Eigen::VectorXd& Js_dot_times_v)
{
    for (size_t i = 0; i < swing_feet_names_.size(); i++) {
        pinocchio::FrameIndex frame_id = model_.getFrameId(swing_feet_names_[i]);

        Js_dot_times_v.segment(0 + 3 * i, 3) =
            pinocchio::getFrameClassicalAcceleration(
                model_, data_, frame_id, pinocchio::LOCAL_WORLD_ALIGNED
            )
                .linear();
    }
}

/* =============================== Compute_oRb ============================== */

void RobotWrapper::get_oRb(Eigen::Matrix3d& oRb) const
{
    pinocchio::FrameIndex base_id = 1;

    oRb = data_.oMi[base_id].rotation();
}

/* ================================= get_r_s ================================ */

void RobotWrapper::get_r_s(Eigen::VectorXd& r_s) const
{
    for (size_t i = 0; i < swing_feet_names_.size(); i++) {
        pinocchio::FrameIndex frame_id = model_.getFrameId(swing_feet_names_[i]);

        r_s.segment(3 * i, 3) = data_.oMf[frame_id].translation();
    }
}

/* ============================== get_feet_pos ============================= */

Eigen::VectorXd RobotWrapper::get_feet_pos(const std::vector<std::string>& gen_feet_names) const
{
    std::vector<std::string> specific_feet_names = generic_to_specific_feet_names(gen_feet_names);

    Eigen::VectorXd feet_position(3 * specific_feet_names.size());

    for (size_t i = 0; i < specific_feet_names.size(); i++) {
        pinocchio::FrameIndex frame_id = model_.getFrameId(specific_feet_names[i]);

        feet_position.segment(3 * i, 3) = data_.oMf[frame_id].translation();
    }

    return feet_position;
}

/* ============================== get_feet_vel ============================= */

Eigen::VectorXd
RobotWrapper::get_feet_vel(const Eigen::VectorXd& v, const std::vector<std::string>& gen_feet_names)
{
    std::vector<std::string> specific_feet_names = generic_to_specific_feet_names(gen_feet_names);

    Eigen::VectorXd feet_velocities(3 * specific_feet_names.size());
    Eigen::MatrixXd J_temp(6, model_.nv);

    for (size_t i = 0; i < specific_feet_names.size(); i++) {
        pinocchio::FrameIndex frame_id = model_.getFrameId(specific_feet_names[i]);

        J_temp.setZero();

        pinocchio::getFrameJacobian(
            model_, data_, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J_temp
        );

        feet_velocities.segment(0 + 3 * i, 3) = J_temp.topRows(3) * v;
    }

    return feet_velocities;
}

/* =========================== get_feet_jacobians ========================== */

Eigen::MatrixXd RobotWrapper::get_feet_jacobians(const std::vector<std::string>& gen_feet_names)
{
    std::vector<std::string> specific_feet_names = generic_to_specific_feet_names(gen_feet_names);

    Eigen::MatrixXd J_feet(3 * specific_feet_names.size(), model_.nv);
    Eigen::MatrixXd J_temp(6, model_.nv);

    for (size_t i = 0; i < specific_feet_names.size(); i++) {
        pinocchio::FrameIndex frame_id = model_.getFrameId(specific_feet_names[i]);

        J_temp.setZero();

        pinocchio::getFrameJacobian(
            model_, data_, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J_temp
        );

        J_feet.block(0 + 3 * i, 0, 3, model_.nv) = J_temp.topRows(3);
    }

    return J_feet;
}

/* ============================= Set_feet_names ============================= */

void RobotWrapper::set_feet_names(const std::vector<std::string>& generic_contact_feet_names)
{
    this->contact_feet_names_ = generic_to_specific_feet_names(generic_contact_feet_names);

    swing_feet_names_ = {};

    for (size_t i = 0; i < feet_names_.size(); i++) {
        if (std::find(
                this->contact_feet_names_.begin(), this->contact_feet_names_.end(), feet_names_[i]
            ) == this->contact_feet_names_.end()) {
            // The foot name is NOT a member of the contact_feet_names vector, hence it is a swing
            // foot.
            swing_feet_names_.push_back(feet_names_[i]);
        }
    }
}

/* ===================== Generic_to_specific_feet_names ===================== */

std::vector<std::string>
RobotWrapper::generic_to_specific_feet_names(std::vector<std::string> generic_names) const
{
    for (auto& foot_name : generic_names) {
        auto it = std::find(
            this->generic_feet_names_.begin(), this->generic_feet_names_.end(), foot_name
        );

        if (it == this->generic_feet_names_.end()) {
            throw std::runtime_error(
                "RobotWrapper::generic_to_specific_feet_names: unknown foot name '" + foot_name +
                "'"
            );
        }

        int index = std::distance(this->generic_feet_names_.begin(), it);

        foot_name = this->feet_names_[index];
    }

    return generic_names;
}

} // namespace robot_model
