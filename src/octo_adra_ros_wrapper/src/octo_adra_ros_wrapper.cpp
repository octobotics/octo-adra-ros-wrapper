/**-------------------------------------------- COPYRIGHT NOTICE ---------------------------------------------------
Copyright (C) 2022 Octobotics Tech Pvt. Ltd. All Rights Reserved.
Do not remove this copyright notice.
Do not use, reuse, copy, merge, publish, sub-license, sell, distribute or modify this code - except without explicit,
written permission from Octobotics Tech Pvt. Ltd.
Contact connect@octobotics.tech for full license information.
Author: Yogesh Phalak
-------------------------------------------- COPYRIGHT NOTICE ---------------------------------------------------*/

/**
 * @file octo_adra_ros_wrapper.cpp
 * @brief source file for octo_adra_ros_wrapper
 * @author Yogesh Phalak
 */


#include "adra/adra_api_serial.h"
#include "octo_adra_ros_wrapper/octo_adra_ros_wrapper.h"


AdraRosWrapper::AdraRosWrapper(const ros::NodeHandle &nh, bool debug) {
    /**
     * @brief Constructor for AdraRosWrapper class
     * @param nh - ros::NodeHandle object
     * @param debug - bool value to enable/disable debug mode
     */

    /// initialize class variables
    nh_ = nh;
    debug_ = debug;
    log_header_ = "[OctoAdraRos]: ";
    mode_ = NONE;
    baud_rate_ = 926600;
    actuators_enabled_ = false;
    brakes_enabled_ = false;

    get_params();
    init_subs_pubs_srvs();
    if (debug_) {
        ROS_INFO("%s Debug mode enabled", log_header_.c_str());
    }

    /// initialize serial port via adra_api_ object
    adra_api_ = new AdraApiSerial(com_port_.c_str(), baud_rate_);
}

AdraRosWrapper::~AdraRosWrapper() {
    /**
     * @brief Destructor for AdraRosWrapper class
     * @param None
     * @return None
     */

    delete adra_api_;
}

void AdraRosWrapper::get_params() {
    /**
     * @brief Get parameters from ros parameter server
     * @param None
     * @return None
     */

    std::string ids_str;

    nh_.param<bool>("/octo_adra_ros/debug", debug_, false);
    nh_.param<std::string>("/octo_adra_ros/com_port", com_port_, "/dev/ttyUSB0");
    nh_.param<int>("/octo_adra_ros/baud_rate", baud_rate_, 921600);
    nh_.param<std::string>("/octo_adra_ros/ids", ids_str, "1,2,3,4,5");

    if (!nh_.getParam("/octo_adra_ros/ids", ids_str)) {
        ids_str = "1,2,3,4,5";
        ROS_WARN("%s Parameter ids not found, using default value %s", log_header_.c_str(), ids_str.c_str());
    }
    if (!nh_.getParam("/octo_adra_ros/debug", debug_)) {
        debug_ = false;
        ROS_WARN("%s Parameter debug not found, using default value %d", log_header_.c_str(), debug_);
    }
    if (!nh_.getParam("/octo_adra_ros/com_port", com_port_)) {
        com_port_ = "/dev/ttyUSB0";
        ROS_WARN("%s Parameter com_port not found, using default value %s", log_header_.c_str(), com_port_.c_str());
    }
    if (!nh_.getParam("/octo_adra_ros/baud_rate", baud_rate_)) {
        baud_rate_ = 921600;
        ROS_WARN("%s Parameter baud_rate not found, using default value %d", log_header_.c_str(), baud_rate_);
    }

    /// convert ids_str (std::string) to ids_ (std::vector<int>)
    std::stringstream ss(ids_str);
    std::string id_str;

    while (std::getline(ss, id_str, ',')) {
        ids_.push_back(std::stoi(id_str));
    }
}

void AdraRosWrapper::init_subs_pubs_srvs() {
    /**
     * @brief Initialize subscribers, publishers and services
     * @param None
     * @return None
     */

    /// subscribers
    sub_cmd_vel_ = nh_.subscribe("/cmd_vel", 1, &AdraRosWrapper::cmd_vel_callback_, this);
    sub_cmd_pose_ = nh_.subscribe("/cmd_pose", 1, &AdraRosWrapper::cmd_pose_callback_, this);

    /// services
    set_command_mode_srv_ = nh_.advertiseService("/set_mode", &AdraRosWrapper::set_command_mode_srv_callback_, this);
    enable_actuators_srv_ = nh_.advertiseService("/enable_actuators", &AdraRosWrapper::enable_actuators_srv_callback_,
                                                 this);
    disable_actuators_srv_ = nh_.advertiseService("/disable_actuators",
                                                  &AdraRosWrapper::disable_actuators_srv_callback_, this);
    enable_brakes_srv_ = nh_.advertiseService("/enable_brakes", &AdraRosWrapper::enable_brakes_srv_callback_, this);
    disable_brakes_srv_ = nh_.advertiseService("/disable_brakes", &AdraRosWrapper::disable_brakes_srv_callback_, this);
}

bool AdraRosWrapper::enable_actuators_srv_callback_(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    /**
     * @brief Enable actuators service callback
     * @param req - std_srvs::Trigger::Request object
     * @param res - std_srvs::Trigger::Response object
     * @return bool value - true if service was executed successfully, false otherwise
     */

    if (mode_ == NONE) {
        ROS_ERROR("%s Cannot enable actuators, no mode selected", log_header_.c_str());
        res.success = false;
        res.message = "No mode selected";
        return false;
    }

    ROS_INFO("%s Enable actuators", log_header_.c_str());
    for (int id: ids_) {
        int resp = adra_api_->into_motion_enable(id);
        if (resp == 0) {
            ROS_INFO("%s Actuator %d enabled", log_header_.c_str(), id);
        } else {
            ROS_ERROR("%s Actuator %d enable failed", log_header_.c_str(), id);
        }
    }
    actuators_enabled_ = true;
    res.success = true;
    res.message = "Actuators enabled";
    return true;
}

bool
AdraRosWrapper::disable_actuators_srv_callback_(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    /**
     * @brief Disable actuators service callback
     * @param req - std_srvs::Trigger::Request object
     * @param res - std_srvs::Trigger::Response object
     * @return bool value - true if service was executed successfully, false otherwise
     */

    if (!actuators_enabled_) {
        ROS_ERROR("%s Cannot disable actuators, actuators are not enabled", log_header_.c_str());
        res.success = false;
        res.message = "Actuators are not enabled";
        return false;
    }

    ROS_INFO("%s Disable actuators", log_header_.c_str());
    for (int id: ids_) {
        int resp = adra_api_->into_motion_disable(id);
        if (resp == 0) {
            ROS_INFO("%s Actuator %d disabled", log_header_.c_str(), id);
        } else {
            ROS_ERROR("%s Actuator %d disable failed", log_header_.c_str(), id);
        }
    }
    res.success = true;
    res.message = "Actuators disabled";
    return true;
}

bool AdraRosWrapper::enable_brakes_srv_callback_(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    /**
     * @brief Enable brakes service callback
     * @param req - std_srvs::Trigger::Request object
     * @param res - std_srvs::Trigger::Response object
     * @return bool value - true if service was executed successfully, false otherwise
     */


    ROS_INFO("%s Enable brakes", log_header_.c_str());
    for (int id: ids_) {
        int resp = adra_api_->into_brake_enable(id);
        if (resp == 0) {
            ROS_INFO("%s Brakes %d enabled", log_header_.c_str(), id);
        } else {
            ROS_ERROR("%s Brakes %d enable failed", log_header_.c_str(), id);
        }
    }
    brakes_enabled_ = true;
    res.success = true;
    res.message = "Brakes enabled";
    return true;
}

bool AdraRosWrapper::disable_brakes_srv_callback_(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    /**
     * @brief Disable brakes service callback
     * @param req - std_srvs::Trigger::Request object
     * @param res - std_srvs::Trigger::Response object
     * @return bool value - true if service was executed successfully, false otherwise
     */

    if (!brakes_enabled_) {
        ROS_ERROR("%s Cannot disable brakes, brakes are not enabled", log_header_.c_str());
        res.success = false;
        res.message = "Brakes are not enabled";
        return false;
    }

    ROS_INFO("%s Disable brakes", log_header_.c_str());
    for (int id: ids_) {
        int resp = adra_api_->into_brake_disable(id);
        if (resp == 0) {
            ROS_INFO("%s Brakes %d disabled", log_header_.c_str(), id);
        } else {
            ROS_ERROR("%s Brakes %d disable failed", log_header_.c_str(), id);
        }
    }
    res.success = true;
    res.message = "Brakes disabled";
    return true;
}

bool AdraRosWrapper::set_command_mode_srv_callback_(octo_adra_ros_wrapper::SetMode::Request &req,
                                                    octo_adra_ros_wrapper::SetMode::Response &res) {
    /**
     * @brief Set command mode service callback
     * @param req - octo_adra_ros_wrapper::SetMode::Request object
     * @param res - octo_adra_ros_wrapper::SetMode::Response object
     * @return bool value - true if service was executed successfully, false otherwise
     */

    if (req.mode_id == ActuatorControlMode::VELOCITY) {
        ROS_INFO("%s Set command mode to VELOCITY", log_header_.c_str());
        mode_ = ActuatorControlMode::VELOCITY;
        for (int id: ids_) {
            int resp = adra_api_->into_motion_mode_vel(id);
            if (resp == 0) {
                ROS_INFO("%s Actuator %d set to VELOCITY", log_header_.c_str(), id);
            } else {
                ROS_ERROR("%s Actuator %d set to VELOCITY failed", log_header_.c_str(), id);
            }
        }
        res.success = true;
        res.message = "Command mode set to VELOCITY";
    } else if (req.mode_id == ActuatorControlMode::POSITION) {
        ROS_INFO("%s Set command mode to POSITION", log_header_.c_str());
        mode_ = ActuatorControlMode::POSITION;
        for (int id: ids_) {
            int resp = adra_api_->into_motion_mode_pos(id);
            if (resp == 0) {
                ROS_INFO("%s Actuator %d set to POSITION", log_header_.c_str(), id);
            } else {
                ROS_ERROR("%s Actuator %d set to POSITION failed", log_header_.c_str(), id);
            }
        }
        res.success = true;
        res.message = "Command mode set to POSITION";
    } else {
        ROS_ERROR("%s Invalid command mode", log_header_.c_str());
        res.success = false;
        res.message = "Invalid command mode";
        return false;
    }

    return true;
}

void AdraRosWrapper::cmd_vel_callback_(const octo_adra_ros_wrapper::TargetValue::ConstPtr &msg) {
    /**
     * @brief Command velocity callback
     * @param msg - octo_adra_ros_wrapper::TargetValue object
     * @return void
     */

    if (mode_ == ActuatorControlMode::VELOCITY) {
        int id = msg->id;
        double value = msg->value;
        int ret = adra_api_->set_vel_target(id, (float)value);
        if (ret != 0) {
            if (debug_) {
                ROS_ERROR("%s Actuator %d set target failed", log_header_.c_str(), id);
            }
        }
    }
}

void AdraRosWrapper::cmd_pose_callback_(const octo_adra_ros_wrapper::TargetValue::ConstPtr &msg) {
    /**
     * @brief Command pose callback
     * @param msg - octo_adra_ros_wrapper::TargetValue object
     * @return void
     */

    if (mode_ == ActuatorControlMode::POSITION) {
        int id = msg->id;
        double value = msg->value;
        int ret = adra_api_->set_pos_target(id, (float)value);
        if (ret != 0) {
            if (debug_) {
                ROS_ERROR("%s Actuator %d set target failed", log_header_.c_str(), id);
            }
        }
    }
}


int main(int argc, char *argv[]) {
    /**
     * @brief Main function
     * @param argc - number of arguments
     * @param argv - array of arguments
     * @return int value - 0 if main function was executed successfully, -1 otherwise
     */

    ros::init(argc, argv, "octo_adra_ros");
    ros::NodeHandle nh;
    ROS_INFO("[OctoAdraRos]: Initializing node");

    bool debug = false;
    AdraRosWrapper adra_ros_wrapper(nh, debug);

    try {
        ros::spin();
    } catch (...) {
        ROS_ERROR("[OctoAdraRos]: Exception caught, shutting down node");
        return -1;
    }

    return 0;
}