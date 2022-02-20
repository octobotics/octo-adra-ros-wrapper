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
	publish_rate_ = 10.0;

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
    nh_.param<double>("/octo_adra_ros/publish_rate", publish_rate_, 10.0);

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

	if (!nh_.getParam("/octo_adra_ros/publish_rate", publish_rate_)) {
        publish_rate_ = 10.0;
        ROS_WARN("%s Parameter publish_rate not found, using default value %f", log_header_.c_str(), publish_rate_);
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

	///publishers
	pub_actuator_status_ = nh_.advertise<octo_adra_ros_wrapper::ActuatorStatus>("/octo_adra_ros/actuator_status",10);

    /// subscribers
    sub_cmd_vel_ = nh_.subscribe("/octo_adra_ros/cmd_vel", 1, &AdraRosWrapper::cmd_vel_callback_, this);
    sub_cmd_pose_ = nh_.subscribe("/octo_adra_ros/cmd_pose", 1, &AdraRosWrapper::cmd_pose_callback_, this);

    /// services
    set_command_mode_srv_ = nh_.advertiseService("/octo_adra_ros/set_mode",
                                                 &AdraRosWrapper::set_command_mode_srv_callback_, this);
    enable_actuators_srv_ = nh_.advertiseService("/octo_adra_ros/enable_actuators",
                                                 &AdraRosWrapper::enable_actuators_srv_callback_,this);
    disable_actuators_srv_ = nh_.advertiseService("/octo_adra_ros/disable_actuators",
                                                  &AdraRosWrapper::disable_actuators_srv_callback_, this);
    enable_brakes_srv_ = nh_.advertiseService("/octo_adra_ros/enable_brakes",
                                              &AdraRosWrapper::enable_brakes_srv_callback_, this);
    disable_brakes_srv_ = nh_.advertiseService("/octo_adra_ros/disable_brakes",
                                               &AdraRosWrapper::disable_brakes_srv_callback_, this);
	set_actuator_limits_srv_ = nh_.advertiseService("/octo_adra_ros/set_actuator_limits",
                                               &AdraRosWrapper::set_actuator_limits_srv_callback_, this);

	///ROS Timer
	update_timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_), &AdraRosWrapper::pub_actuator_status, this);
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
    mode_ = NONE;
    actuators_enabled_ = false;
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

bool AdraRosWrapper::set_actuator_limits_srv_callback_(octo_adra_ros_wrapper::SetLimits::Request &req,
                                        octo_adra_ros_wrapper::SetLimits::Response &res){
	/**
     * @brief Service call to set pos, vel and tau limits of the actuator
     * @param req - octo_adra_ros_wrapper::SetMode::Request
     * @param res - octo_adra_ros_wrapper::SetMode::Response
     * @return bool - true if success
     */

	// Position Limits (min, max, diff)
	res.response.push_back(adra_api_->set_pos_limit_min(req.id, req.pos_limit_min));
	res.response.push_back(adra_api_->set_pos_limit_max(req.id, req.pos_limit_max));
	res.response.push_back(adra_api_->set_pos_limit_diff(req.id, req.pos_limit_diff));

	// Velocity Limits (min, max, diff)
	res.response.push_back(adra_api_->set_vel_limit_min(req.id, req.vel_limit_min));
	res.response.push_back(adra_api_->set_vel_limit_max(req.id, req.vel_limit_max));
	res.response.push_back(adra_api_->set_vel_limit_diff(req.id, req.vel_limit_diff));

	// Effort Limits (min, max, diff)
	res.response.push_back(adra_api_->set_tau_limit_min(req.id, req.tau_limit_min));
	res.response.push_back(adra_api_->set_tau_limit_max(req.id, req.tau_limit_diff));
	res.response.push_back(adra_api_->set_tau_limit_diff(req.id, req.tau_limit_diff));

	for (int val : res.response){
		if (val != 0){
			res.success = false;
			ROS_ERROR("Setting the limits failed");
			return false;
		}
	}
	ROS_INFO("All the limits have been set successfully");

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
        int ret = adra_api_->set_vel_target(id, (float) value);
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
        int ret = adra_api_->set_pos_target(id, (float) value);
        if (ret != 0) {
            if (debug_) {
                ROS_ERROR("%s Actuator %d set target failed", log_header_.c_str(), id);
            }
        }
    }
}

// TODO: Think of a better structure for the actuator status publisher function
void AdraRosWrapper::pub_actuator_status(const ros::TimerEvent &){
	/**
	 * @brief Publish the status of all the ids of the actuator
	 * @param ros::TimerEvent
	 * @return none
	 *
	 */
	octo_adra_ros_wrapper::ActuatorStatus status;
	std::string value_str;
	char value_char[24];
	uint8_t value_uint;
	int8_t value_int8[2];
	uint8_t value_uint8[2];
	float value_float;
	int ret;

	for (int id : ids_){

		// Get UUID, sw_version, hw_version, multi_version
		ret = adra_api_->get_uuid(id, value_char);
		value_str = value_char;
		status.uuid.push_back(value_str);

		ret = adra_api_->get_sw_version(id, value_char);
		value_str = value_char;
		status.sw_version.push_back(value_str);

		ret = adra_api_->get_hw_version(id, value_char);
		value_str = value_char;
		status.hw_version.push_back(value_str);

		ret = adra_api_->get_multi_version(id, value_char);
		value_str = value_char;
		status.multi_version.push_back(value_str);

		// Get mech_ratio, elec_ration
		ret = adra_api_->get_mech_ratio(id, &value_float);
		status.mech_ratio.push_back(value_float);

		ret = adra_api_->get_elec_ratio(id, &value_float);
		status.elec_ratio.push_back(value_float);

		// Get motion_dir
		ret = adra_api_->get_motion_dir(id, &value_uint);
		status.motion_dir.push_back(value_uint);

		// Get temp_limit, volt_limit, curr_limit
		ret = adra_api_->get_temp_limit(id, &value_int8[0], &value_int8[1]);
		status.temp_limit_min.push_back(value_int8[0]);
		status.temp_limit_max.push_back(value_int8[1]);

		ret = adra_api_->get_volt_limit(id, &value_uint8[0], &value_uint8[1]);
		status.volt_limit_min.push_back(value_uint8[0]);
		status.volt_limit_max.push_back(value_uint8[1]);

		ret = adra_api_->get_curr_limit(id, &value_float);
		status.curr_limit.push_back(value_float);

		// Get motion_mode, motion_enable, brake_enable
		ret = adra_api_->get_motion_mode(id, &value_uint);
		status.motion_mode.push_back(value_uint);

		ret = adra_api_->get_motion_enable(id, &value_uint);
		status.motion_enable.push_back(value_uint);

		ret = adra_api_->get_brake_enable(id, &value_uint);
		status.brake_enable.push_back(value_uint);

		// Get temp_driver, temp_motor, bus_volt, bus_curr, multi_volt
		ret = adra_api_->get_temp_driver(id, &value_float);
		status.temp_driver.push_back(value_float);

		ret = adra_api_->get_temp_motor(id, &value_float);
		status.temp_motor.push_back(value_float);

		ret = adra_api_->get_bus_volt(id, &value_float);
		status.bus_volt.push_back(value_float);

		ret = adra_api_->get_bus_curr(id, &value_float);
		status.bus_curr.push_back(value_float);

		ret = adra_api_->get_multi_volt(id, &value_float);
		status.multi_volt.push_back(value_float);

		// Get error_code
		ret = adra_api_->get_error_code(id, &value_uint);
		status.error_code.push_back(value_uint);

		// Get pos, vel and tau limit min, max and diff
		ret = adra_api_->get_pos_limit_min(id, &value_float);
		status.pos_limit_min.push_back(value_float);

		ret = adra_api_->get_pos_limit_max(id, &value_float);
		status.pos_limit_max.push_back(value_float);

		ret = adra_api_->get_pos_limit_diff(id, &value_float);
		status.pos_limit_diff.push_back(value_float);

		ret = adra_api_->get_vel_limit_min(id, &value_float);
		status.vel_limit_min.push_back(value_float);

		ret = adra_api_->get_vel_limit_max(id, &value_float);
		status.vel_limit_max.push_back(value_float);

		ret = adra_api_->get_vel_limit_diff(id, &value_float);
		status.vel_limit_diff.push_back(value_float);

		ret = adra_api_->get_tau_limit_min(id, &value_float);
		status.tau_limit_min.push_back(value_float);

		ret = adra_api_->get_tau_limit_max(id, &value_float);
		status.tau_limit_max.push_back(value_float);

		ret = adra_api_->get_tau_limit_diff(id, &value_float);
		status.tau_limit_diff.push_back(value_float);

		// Get pos, vel and tau target and current value
		ret = adra_api_->get_pos_target(id, &value_float);
		status.pos_target.push_back(value_float);

		ret = adra_api_->get_pos_current(id, &value_float);
		status.pos_current.push_back(value_float);

		ret = adra_api_->get_vel_target(id, &value_float);
		status.vel_target.push_back(value_float);

		ret = adra_api_->get_vel_current(id, &value_float);
		status.vel_current.push_back(value_float);

		ret = adra_api_->get_tau_target(id, &value_float);
		status.tau_target.push_back(value_float);

		ret = adra_api_->get_tau_current(id, &value_float);
		status.tau_current.push_back(value_float);

		// Get pos, vel and tau pid values
		ret = adra_api_->get_pos_pidp(id, &value_float);
		status.pos_pidp.push_back(value_float);

		ret = adra_api_->get_vel_pidp(id, &value_float);
		status.vel_pidp.push_back(value_float);

		ret = adra_api_->get_vel_pidi(id, &value_float);
		status.vel_pidi.push_back(value_float);

		ret = adra_api_->get_tau_pidp(id, &value_float);
		status.tau_pidp.push_back(value_float);

		ret = adra_api_->get_tau_pidi(id, &value_float);
		status.tau_pidi.push_back(value_float);

		// Get pos, vel and tau smooth cyc
		ret = adra_api_->get_pos_smooth_cyc(id, &value_uint);
		status.pos_smooth_cyc.push_back(value_uint);

		ret = adra_api_->get_vel_smooth_cyc(id, &value_uint);
		status.vel_smooth_cyc.push_back(value_uint);

		ret = adra_api_->get_tau_smooth_cyc(id, &value_uint);
		status.tau_smooth_cyc.push_back(value_uint);
	}

	pub_actuator_status_.publish(status);

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