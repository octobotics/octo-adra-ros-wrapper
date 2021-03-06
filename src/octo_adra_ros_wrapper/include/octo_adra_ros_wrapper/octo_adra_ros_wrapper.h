/**-------------------------------------------- COPYRIGHT NOTICE ---------------------------------------------------
Copyright (C) 2022 Octobotics Tech Pvt. Ltd. All Rights Reserved.
Do not remove this copyright notice.
Do not use, reuse, copy, merge, publish, sub-license, sell, distribute or modify this code - except without explicit,
written permission from Octobotics Tech Pvt. Ltd.
Contact connect@octobotics.tech for full license information.
Author: Yogesh Phalak
-------------------------------------------- COPYRIGHT NOTICE ---------------------------------------------------*/

/**
 * @file octo_adra_ros_wrapper.h
 * @brief ROS Wrapper for umbratek actuators
 * @author Yogesh Phalak
 */

#include "ros/ros.h"
#include "adra/adra_api_serial.h"
#include "std_srvs/Trigger.h"
#include "octo_adra_ros_wrapper/SetMode.h"
#include "octo_adra_ros_wrapper/TargetValue.h"

#ifndef OCTOBOTICS_WS_OCTO_ADRA_ROS_WRAPPER_H
#define OCTOBOTICS_WS_OCTO_ADRA_ROS_WRAPPER_H

enum ActuatorControlMode {
    NONE = 0,
    POSITION = 1,
    VELOCITY = 2,
    TORQUE = 3
};

class AdraRosWrapper {
public:
    /**
     * @brief Constructor
     * @param nh
     * @param debug
     * @return none
     */
    AdraRosWrapper(const ros::NodeHandle &nh, bool debug);

    /**
     * @brief Destructor
     * @param none
     * @return none
     */
    ~AdraRosWrapper();

    /**
     * @brief get ros parameters from launch file
     * @param none
     * @return none
     */
    void get_params();

    /**
     * @brief initialize subscribers, publishers and services
     * @param none
     * @return none
     */
    void init_subs_pubs_srvs();

private:
    /// node specific properties
    std::string log_header_;
    bool debug_;
    bool actuators_enabled_;
    bool brakes_enabled_;
    ros::NodeHandle nh_;

    /// parameters for adra apis
    std::string com_port_;
    int baud_rate_;
    std::vector<int> ids_;

    /// object of adra api
    AdraApiSerial *adra_api_;

    /// subscribers
    ros::Subscriber sub_cmd_vel_;
    ros::Subscriber sub_cmd_pose_;

    /// subscribed topics
    std::string cmd_vel_topic_;
    std::string cmd_pose_topic_;

    /// services
    ros::ServiceServer enable_actuators_srv_;
    ros::ServiceServer disable_actuators_srv_;
    ros::ServiceServer enable_brakes_srv_;
    ros::ServiceServer disable_brakes_srv_;
    ros::ServiceServer set_command_mode_srv_;

    /// control mode state
    ActuatorControlMode mode_;

    /// subscriber callbacks

    /**
     * @brief callback for cmd_vel topic
     * @param msg - octo_adra_ros_wrapper::TargetValue
     * @return none
     */
    void cmd_vel_callback_(const octo_adra_ros_wrapper::TargetValue::ConstPtr &msg);

    /**
     * @brief callback for cmd_pose topic
     * @param msg - octo_adra_ros_wrapper::TargetValue
     * @return none
     */
    void cmd_pose_callback_(const octo_adra_ros_wrapper::TargetValue::ConstPtr &msg);

    /// services callbacks

    /**
     * @brief callback for enable actuators service
     * @param req - std_srvs::Trigger::Request
     * @param res - std_srvs::Trigger::Response
     * @return bool - true if success
     */
    bool enable_actuators_srv_callback_(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    /**
     * @brief callback for disable actuators service
     * @param req - std_srvs::Trigger::Request
     * @param res - std_srvs::Trigger::Response
     * @return bool - true if success
     */
    bool disable_actuators_srv_callback_(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    /**
     * @brief callback for enable brakes service
     * @param req - std_srvs::Trigger::Request
     * @param res - std_srvs::Trigger::Response
     * @return bool - true if success
     */
    bool enable_brakes_srv_callback_(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    /**
     * @brief callback for disable brakes service
     * @param req - std_srvs::Trigger::Request
     * @param res - std_srvs::Trigger::Response
     * @return bool - true if success
     */
    bool disable_brakes_srv_callback_(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    /**
     * @brief callback for set command mode service
     * @param req - octo_adra_ros_wrapper::SetMode::Request
     * @param res - octo_adra_ros_wrapper::SetMode::Response
     * @return bool - true if success
     */
    bool set_command_mode_srv_callback_(octo_adra_ros_wrapper::SetMode::Request &req,
                                        octo_adra_ros_wrapper::SetMode::Response &res);
};


#endif //OCTOBOTICS_WS_OCTO_ADRA_ROS_WRAPPER_H
