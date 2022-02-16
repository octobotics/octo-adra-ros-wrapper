//
// Created by octobotics on 16/02/22.
//

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
    // constructor and destructor
public:
    AdraRosWrapper(const ros::NodeHandle &nh, bool debug);

    ~AdraRosWrapper();

    void get_params();

    void init_subs_pubs_srvs();

private:
    std::string log_header_;
    bool debug_;
    ros::NodeHandle nh_;
    std::string com_port_;
    int baud_rate_{};
    std::vector<int> ids_;
    AdraApiSerial *adra_api_;

    // subscribed topics
    ros::Subscriber sub_cmd_vel_;
    ros::Subscriber sub_cmd_pose_;

    std::string cmd_vel_topic_;
    std::string cmd_pose_topic_;

    // services
    ros::ServiceServer enable_actuators_srv_;
    ros::ServiceServer disable_actuators_srv_;
    ros::ServiceServer enable_brakes_srv_;
    ros::ServiceServer disable_brakes_srv_;
    ros::ServiceServer set_command_mode_srv_;

    ActuatorControlMode mode_;

    // subscriber callbacks
    // take double as input message
    void cmd_vel_callback_(const octo_adra_ros_wrapper::TargetValue::ConstPtr &msg);

    void cmd_pose_callback_(const octo_adra_ros_wrapper::TargetValue::ConstPtr &msg);

    // services callbacks
    bool enable_actuators_srv_callback_(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    bool disable_actuators_srv_callback_(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    bool enable_brakes_srv_callback_(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    bool disable_brakes_srv_callback_(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    bool set_command_mode_srv_callback_(octo_adra_ros_wrapper::SetMode::Request &req,
                                        octo_adra_ros_wrapper::SetMode::Response &res);
};


#endif //OCTOBOTICS_WS_OCTO_ADRA_ROS_WRAPPER_H
