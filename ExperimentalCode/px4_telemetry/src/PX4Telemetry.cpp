#include "PX4Telemetry.hpp"

#define MIN_VOLTAGE 19.2
#define MAX_VOLTAGE 25.2

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

PX4Telemetry::PX4Telemetry() : Node("px4_telemetry_node"), landing_requested_(false), alt_init_(false), lpos_init_(false), gpos_init_(false) {
    RCLCPP_INFO(this->get_logger(), "Initializing PX4 Telemetry Node");
    
    //Get my namespace (remove the slash with substr)
    px4_id_ = std::string(this->get_namespace()).substr(1);

    //Temporary string storage for UTM band
    std::string utm_band_str;

    //Get park geodesy parameters
    this->declare_parameter("origin_x", 0.0);
    this->declare_parameter("origin_y", 0.0);
    this->declare_parameter("origin_r", 0.0);
    this->declare_parameter("utm_zone", 0);
    this->declare_parameter("utm_band", "R");
    if (
        this->get_parameter("origin_x", origin_x_) && 
        this->get_parameter("origin_y", origin_y_) && 
        this->get_parameter("origin_r", origin_r_) && 
        this->get_parameter("utm_zone", utm_zone_) &&
        this->get_parameter("utm_band", utm_band_str)
    ) {
        utm_band_ = utm_band_str[0];
        RCLCPP_INFO(this->get_logger(), "Park origin set to (%.4f, %.4f), %.4f rad, Zone %d, Band %c", origin_x_, origin_y_, origin_r_, utm_zone_, utm_band_);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Park geodesy parameters not provided.");
        rclcpp::shutdown();
    }

    //Joy button config
    this->declare_parameter("arm_button", -1);
    this->declare_parameter("disarm_button", -1);
    this->declare_parameter("control_button", -1);
    this->declare_parameter("follow_setpoint_button", -1);
    this->get_parameter("arm_button", buttons_.arm.button);
    this->get_parameter("disarm_button", buttons_.disarm.button);
    this->get_parameter("control_button", buttons_.control.button);
    this->get_parameter("follow_setpoint_button", buttons_.follow.button);

    RCLCPP_INFO(this->get_logger(), "Loaded joy parameters:\nArm: %d, Disarm: %d, Control: %d", buttons_.arm.button, buttons_.disarm.button, buttons_.control.button);

    //Get simulation mode parameter
    this->declare_parameter("sim_mode", false);
    this->get_parameter("sim_mode", sim_mode_);
    if (sim_mode_) {
        RCLCPP_WARN(this->get_logger(), "Simulation mode enabled.");
    }
    else RCLCPP_WARN(this->get_logger(), "Using local altitude for physical drone. Sim Mode Disabled.");

    //Convert park transform to quaternion
    q_utm_to_apark_.setRPY(0, 0, origin_r_);

    //Convert park transform to quaternion
    q_apark_to_utm_.setRPY(0, 0, -origin_r_);

    //Initialize egm96 (WGS-84) ellipsoid 
    egm96_5_ = std::make_shared<GeographicLib::Geoid>("egm96-5", "", true, true);

    //Autonomy park tf broadcaster
    apark_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    apark_tf_.header.frame_id = "autonomy_park";
    apark_tf_.child_frame_id = px4_id_;

    //Autonomy park pose publisher
    apark_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("autonomy_park/pose", 1);
    apark_pose_.header.frame_id = "autonomy_park";

    //Global position origin publisher
    gp_origin_publisher_ = this->create_publisher<geographic_msgs::msg::GeoPointStamped>("global_position/set_gp_origin", 1);
    
    set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("set_mode");
    arm_client_ = this->create_client<mavros_msgs::srv::CommandBool>("cmd/arming");
    takeoff_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("cmd/takeoff");
    land_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("cmd/land");

    // Wait for set mode service
    while (!set_mode_client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for mode service. Exiting.");
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Mode service not available, waiting again...");
    }

    //Wait for arm service
    while (!arm_client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for arming service. Exiting.");
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Arming service not available, waiting again...");
    }

    //Wait for TOL service
    while (!takeoff_client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for TOL service. Exiting.");
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "TOL service not available, waiting again...");
    }

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&PX4Telemetry::joy_callback, this, _1));

    //Set mavros QOS to keep last
    auto sub_qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_default);
    sub_qos.best_effort();
    sub_qos.durability_volatile();

    //Mavros subscribers
    state_sub_ = this->create_subscription<mavros_msgs::msg::State>("state", sub_qos, std::bind(&PX4Telemetry::state_callback, this, _1));
    ext_state_sub_ = this->create_subscription<mavros_msgs::msg::ExtendedState>("extended_state", sub_qos, std::bind(&PX4Telemetry::ext_state_callback, this, _1));
    battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>("battery", sub_qos, std::bind(&PX4Telemetry::battery_callback, this, _1));

    altitude_sub_ = this->create_subscription<mavros_msgs::msg::Altitude>("altitude", sub_qos, std::bind(&PX4Telemetry::altitude_callback, this, _1));
    global_lpos_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("global_position/local", sub_qos, std::bind(&PX4Telemetry::global_lpos_callback, this, _1));
    global_gpos_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("global_position/global", sub_qos, std::bind(&PX4Telemetry::global_gpos_callback, this, _1));

    loiter_str_ = std::string("AUTO.LOITER");
    offboard_str_ = std::string("OFFBOARD");

    RCLCPP_INFO(this->get_logger(), "Astro Telemetry Initialized.");
}

void PX4Telemetry::joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {
    //Prevent operation until telemetry is initialized
    if (!(alt_init_ && lpos_init_ && gpos_init_)) {
        RCLCPP_ERROR(this->get_logger(), "Button presses ignored until telemetry is initialized.");
    }

    //Check for arm button press
    int arm_button_state = get_button(joy_msg, buttons_.arm);
    if (arm_button_state != button_state_.arm.state) {
        if (arm_button_state == 1) {
            RCLCPP_INFO(this->get_logger(), "Arm/takeoff button pressed");

            //Arm or takeoff, depending on the state
            if (!current_state_.armed) {
                //Arm
                send_arming_request(true);
            } else {
                send_tol_request(true);
            }
        }

        button_state_.arm.state = arm_button_state;
    }

    // int control_button_state = get_button(joy_msg, buttons_.control);
    // if (control_button_state != button_state_.control.state) {
    //     if (control_button_state == 1) {
    //         if(control_mode_ == "position") {
    //             RCLCPP_WARN(this->get_logger(), "Mode set to velocity control.");
    //             control_mode_ = "velocity";
    //         }
    //         else { 
    //         control_mode_ = "position";
    //         RCLCPP_WARN(this->get_logger(), "Mode set to position control.");
    //         }
    //     }
    // }

    //Check for disarm button press
    int disarm_button_state = get_button(joy_msg, buttons_.disarm);
    if (disarm_button_state != button_state_.disarm.state) {
        if (disarm_button_state == 1) {
            RCLCPP_INFO(this->get_logger(), "Disarm/land button pressed");

            if (current_state_.armed) {
                if (landed_state_ == on_ground) {
                    //Send disarm request
                    send_arming_request(false);
                } else {
                    if (current_state_.mode == loiter_str_) {
                        //If in auto loiter, go ahead and land
                        send_tol_request(false);
                    } else {
                        //Otherwise, set landing request flag and request auto loiter mode 
                        landing_requested_ = true;

                        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
                        request->custom_mode = loiter_str_;
                        auto set_mode_result = set_mode_client_->async_send_request(request, std::bind(&PX4Telemetry::loiter_mode_response_callback, this, _1));
                    }
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "Not armed - disarm/land request ignored.");
            }
        }
        button_state_.disarm.state = disarm_button_state;
    }

    //Check for arm button press
    int offboard_button_state = get_button(joy_msg, buttons_.offboard);
    if (offboard_button_state != button_state_.offboard.state) {
        if (offboard_button_state == 1) {
            RCLCPP_INFO(this->get_logger(), "Offboard button pressed.");

            //Toggle between offboard and auto loiter
            if (current_state_.mode == offboard_str_) {
                auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
                request->custom_mode = loiter_str_;
                auto set_mode_result = set_mode_client_->async_send_request(request, std::bind(&PX4Telemetry::loiter_mode_response_callback, this, _1));
            } else {
                auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
                request->custom_mode = offboard_str_;
                auto set_mode_result = set_mode_client_->async_send_request(request, std::bind(&PX4Telemetry::offboard_mode_response_callback, this, _1));
            }
        }
        button_state_.offboard.state = offboard_button_state;
    }

    // check for velocity setpoint follow button press
    // int follow_button_state = get_button(joy_msg, buttons_.follow);
    // if(follow_button_state != button_state_.follow.state) {
    //     if(follow_button_state == 1) {
    //         setpoint_timer_ = this->create_wall_timer(100ms, std::bind(&AstroTeleop::follow_setpoint, this));
    //     }
    //     button_state_.follow.state = follow_button_state;
    // }
}

void PX4Telemetry::state_callback(const mavros_msgs::msg::State::SharedPtr state_msg) {

    if (state_msg->armed && !current_state_.armed) {
        RCLCPP_WARN(this->get_logger(), "Armed");
    } else if (!state_msg->armed && current_state_.armed) {
        RCLCPP_WARN(this->get_logger(), "Disarmed");
    }

    // if (state_msg->mode == offboard_str_ && current_state_.mode != offboard_str_) {
    //     RCLCPP_WARN(this->get_logger(), "Offboard mode enabled.");
    //     //If on the ground when offboard is enabled, disable it
    //     if (landed_state_ == on_ground) {
    //         auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    //         request->custom_mode = "AUTO.LOITER";
    //         auto set_mode_result = set_mode_client_->async_send_request(request, std::bind(&AstroTeleop::position_mode_response_callback, this, _1));
    //     }
    // } else 

    if (state_msg->mode == loiter_str_ && current_state_.mode != loiter_str_) {
        RCLCPP_WARN(this->get_logger(), "Loiter mode enabled.");
        
        if (landing_requested_) {
            //Send landing request
            send_tol_request(false);
            landing_requested_ = false;
        }
    } else if (state_msg->mode == offboard_str_ && current_state_.mode != offboard_str_) {
        RCLCPP_WARN(this->get_logger(), "Offboard mode enabled.");
    }

    current_state_ = *state_msg;
}

void PX4Telemetry::ext_state_callback(const mavros_msgs::msg::ExtendedState::SharedPtr ext_state_msg) {

    if ((LandedState)ext_state_msg->landed_state != landed_state_) {
        switch (ext_state_msg->landed_state) {
            case undefined: {
                RCLCPP_ERROR(this->get_logger(), "Undefined landed state!");
                break;
            } case on_ground: {
                RCLCPP_WARN(this->get_logger(), "Entered on ground state.");
                break;
            } case in_air: {
                RCLCPP_WARN(this->get_logger(), "Entered in air state.");
                break;
            } case takeoff: {
                RCLCPP_WARN(this->get_logger(), "Entered takeoff state.");
                break;
            } case landing: {
                RCLCPP_WARN(this->get_logger(), "Entered landing state.");
                break;
            }
        }

        landed_state_ = (LandedState)ext_state_msg->landed_state;
    }
}

void PX4Telemetry::battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg) {
    //Todo: Fix this so it matches readout on Astro (scale via usable battery life)
    // RCLCPP_WARN(this->get_logger(), "Battery = %.2f%%", (msg->voltage-MIN_VOLTAGE)/(MAX_VOLTAGE-MIN_VOLTAGE)*100.0);
    battery_voltage_ = msg->voltage;
}

//Get local altitude from altitude topic
void PX4Telemetry::altitude_callback(const mavros_msgs::msg::Altitude::SharedPtr msg) {
    //Gazebo sim uses monotonic altitude, physical drone uses local tied to bottom_clearance via lidar
    if (sim_mode_) {
        apark_pose_.pose.position.z = msg->monotonic;
    } else {
        apark_pose_.pose.position.z = msg->local;
    }
    
    altitude_amsl_ = msg->amsl;

    //Set initialization flag
    if (!alt_init_) alt_init_ = true;
}

//Get orientation from UTM pose
void PX4Telemetry::global_lpos_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    //Get UTM orientation
    tf2::Quaternion q_utm, q_apark;
    tf2::fromMsg(msg->pose.pose.orientation, q_utm);
    q_apark = q_utm_to_apark_*q_utm;
    apark_pose_.pose.orientation = tf2::toMsg(q_apark);

    //Set initialization flag
    if (!lpos_init_) lpos_init_ = true;
}

//Get LL from GPS pos
void PX4Telemetry::global_gpos_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    //Record the time the message came in
    rclcpp::Time now = this->get_clock()->now();

    //Convert GPS coords to DP frame and publish seperately
    auto geo_msg = geographic_msgs::msg::GeoPoint();
    geo_msg.latitude = msg->latitude;
    geo_msg.longitude = msg->longitude;
    geo_msg.altitude = msg->altitude; //Ellipsoidal altitude

    //Convert LLA to UTM
    geodesy::UTMPoint utm_pos;
    geodesy::fromMsg(geo_msg, utm_pos);
    
    //Convert ellipsoidal height to AMSL
    // double geoid_height = GeographicLib::Geoid::GEOIDTOELLIPSOID * (*egm96_5_)(msg->latitude, msg->longitude);
    // double altitude_amsl = msg->altitude - geoid_height;
    // RCLCPP_WARN(this->get_logger(), "AMSL = %.4f meters", altitude_amsl_);

    double dx = utm_pos.easting - origin_x_;
    double dy = utm_pos.northing - origin_y_;
    apark_pose_.pose.position.x = cos(origin_r_)*dx - sin(origin_r_)*dy;
    apark_pose_.pose.position.y = sin(origin_r_)*dx + cos(origin_r_)*dy;

    // apark_pose_.pose.position.z = altitude_amsl - origin_z_;

    apark_pose_.header.stamp = now;

    // RCLCPP_WARN(this->get_logger(), "Apark Z = %.4f meters", apark_pose_.pose.position.z);

    //Publish pose
    this->apark_pose_publisher_->publish(apark_pose_);

    //Broadcast TF
    apark_tf_.header.stamp = now;
    apark_tf_.transform.translation.x = apark_pose_.pose.position.x;
    apark_tf_.transform.translation.y = apark_pose_.pose.position.y;
    apark_tf_.transform.translation.z = apark_pose_.pose.position.z;
    apark_tf_.transform.rotation = apark_pose_.pose.orientation;

    apark_tf_broadcaster_->sendTransform(apark_tf_);

    //Set initialization flag
    if (!gpos_init_) gpos_init_ = true;
}

int PX4Telemetry::get_button(const sensor_msgs::msg::Joy::SharedPtr &joy_msg, const Button &button) {
    if (button.button < 0 || button.button > (int)joy_msg->buttons.size()-1) {
        RCLCPP_ERROR(this->get_logger(), "Button %d out of range, joy has %d buttons", button.button, (int)joy_msg->buttons.size());
        return -1;
    }

    return joy_msg->buttons[button.button];
}

void PX4Telemetry::send_arming_request(bool arm) {
    bool valid_request = false;
    if (arm) {
        if (!current_state_.armed) {
            RCLCPP_WARN(this->get_logger(), "Sending arm request.");
            valid_request = true;
        } else {
            RCLCPP_WARN(this->get_logger(), "Device already armed - arm request ignored.");
            return;
        }
    } else {
        if (current_state_.armed) {
            RCLCPP_WARN(this->get_logger(), "Sending disarm request.");
            valid_request = true;
        } else {
            RCLCPP_WARN(this->get_logger(), "Device already disarmed - disarm request ignored.");
            return;
        }
    }

    if (valid_request) {
        auto arm_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        arm_request->value = arm;
        auto arm_result = arm_client_->async_send_request(arm_request, std::bind(&PX4Telemetry::arm_response_callback, this, _1));
    }
}

void PX4Telemetry::send_tol_request(bool takeoff) {
    auto tol_request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();

    if (takeoff) {
        RCLCPP_WARN(this->get_logger(), "Sending takeoff request.");
        
        //Set takeoff position to current @ 1 meter altitude
        geometry_msgs::msg::Pose takeoff_pose = apark_pose_.pose;

        geographic_msgs::msg::GeoPose global_pose = apark_to_global(takeoff_pose);
        tol_request->yaw = quat_to_yaw(global_pose.orientation);
        tol_request->latitude = global_pose.position.latitude;
        tol_request->longitude = global_pose.position.longitude;

        //Take off to a height of 2 meters above the current AMSL
        // tol_request->altitude = altitude_amsl_ - apark_pose_.pose.position.z + 1.5;
        tol_request->altitude = altitude_amsl_ + 2.0;

        RCLCPP_WARN(this->get_logger(), "Sending takeoff request.");

        RCLCPP_WARN(this->get_logger(), "Taking off to %.2f meters AMSL", tol_request->altitude);

        auto tol_result = takeoff_client_->async_send_request(tol_request, std::bind(&PX4Telemetry::tol_response_callback, this, _1));
    } else {
        RCLCPP_WARN(this->get_logger(), "Sending landing request.");

        //Set landing pos to current @ 0 meter altitude
        geometry_msgs::msg::Pose landing_pose = apark_pose_.pose;

        geographic_msgs::msg::GeoPose global_pose = apark_to_global(landing_pose);
        tol_request->yaw = quat_to_yaw(global_pose.orientation);
        tol_request->latitude = global_pose.position.latitude;
        tol_request->longitude = global_pose.position.longitude;
        tol_request->altitude = 0.0;

        auto tol_result = land_client_->async_send_request(tol_request, std::bind(&PX4Telemetry::tol_response_callback, this, _1));
    }
}

void PX4Telemetry::loiter_mode_response_callback(rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
    auto response = future.get();

    if (response->mode_sent) {
        RCLCPP_INFO(this->get_logger(), "Loiter mode request succeeded.");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Loiter mode request failed!");
    }
}

void PX4Telemetry::offboard_mode_response_callback(rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
    auto response = future.get();

    if (response->mode_sent) {
        RCLCPP_INFO(this->get_logger(), "Offboard mode request succeeded.");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Offboard mode request failed!");
    }
}

void PX4Telemetry::arm_response_callback(rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
    auto response = future.get();

    if (response->success) {
        RCLCPP_INFO(this->get_logger(), "Arm/disarm request succeeded. Result=%d", response->result);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Arm/disarm request failed!");
    }
}

void PX4Telemetry::tol_response_callback(rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture future) {
    auto response = future.get();

    if (response->success) {
        RCLCPP_INFO(this->get_logger(), "TOL request succeeded. Result=%d", response->result);
    } else {
        RCLCPP_ERROR(this->get_logger(), "TOL request failed!");
    }
}

double PX4Telemetry::quat_to_yaw(geometry_msgs::msg::Quaternion quat) {

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y);
    double cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);
    return yaw;
}

//Converts a pose in the autonomy park frame to LLA
geographic_msgs::msg::GeoPose PX4Telemetry::apark_to_global(const geometry_msgs::msg::Pose &apark_pose) {
    //Autonomy park setpoint coordinates
    double sp_x = apark_pose.position.x;
    double sp_y = apark_pose.position.y;
    
    //Un-rotate setpoint coordinates
    double dx = cos(origin_r_)*sp_x + sin(origin_r_)*sp_y;
    double dy = -sin(origin_r_)*sp_x + cos(origin_r_)*sp_y;

    //Compute AMSL altitude using park elevation offset
    // double altitude_amsl = apark_pose.position.z + origin_z_;

    //Convert park coordinates to UTM
    geodesy::UTMPoint utm_pos;
    utm_pos.zone = utm_zone_;
    utm_pos.band = utm_band_;
    utm_pos.easting = dx + origin_x_;
    utm_pos.northing = dy + origin_y_;

    //Convert UTM easting/northing to lat/long
    geographic_msgs::msg::GeoPoint global_pos = geodesy::toMsg(utm_pos);

    //RCLCPP_WARN(this->get_logger(), "UTM Easting=%.8f, Northing=%.8f", utm_pos.easting, utm_pos.northing);
    //RCLCPP_WARN(this->get_logger(), "GPS Lat=%.8f, Long=%.8f\n", global_pos.latitude, global_pos.longitude);

    //Convert AMSL altitude to ellipsoidal
    // double geoid_height = GeographicLib::Geoid::GEOIDTOELLIPSOID * (*egm96_5_)(global_pos.latitude, global_pos.longitude);

    //IMPORTANT: Command altitude is AMSL! (feedback is WGS-84 ellipsoid)
    global_pos.altitude = altitude_amsl_;

    //Finally, compute global orientation
    tf2::Quaternion q_utm, q_apark;
    tf2::fromMsg(apark_pose.orientation, q_apark);
    q_utm = q_apark_to_utm_*q_apark;
    
    geographic_msgs::msg::GeoPose global_pose;
    global_pose.position = global_pos;
    global_pose.orientation = tf2::toMsg(q_utm);

    return global_pose;
}