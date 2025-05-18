#include <chrono>
#include <cstdint>
#include <memory>
#include <future>
#include <thread>
#include <iostream>
#include <sstream>
#include <functional> // For std::bind with thread
#include <mutex>      // For std::mutex and std::lock_guard
#include <vector>
#include <cmath>      // For M_PI, sin, cos (C++20 이상이면 <numbers>의 std::numbers::pi 사용 고려)
#include <string>     // For std::to_string

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp" // For bonus mission

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
// #include <mavsdk/plugins/offboard/offboard.h>

using namespace mavsdk;
using std::chrono::seconds;
using std::chrono::milliseconds;
using std::this_thread::sleep_for;

// Helper to convert MAVSDK Action::Result to string for logging
std::string action_result_str(Action::Result result) {
    switch (result) {
        case Action::Result::Success: return "Success";
        case Action::Result::NoSystem: return "NoSystem";
        case Action::Result::ConnectionError: return "ConnectionError";
        case Action::Result::Busy: return "Busy";
        case Action::Result::CommandDenied: return "CommandDenied";
        case Action::Result::CommandDeniedLandedStateUnknown: return "CommandDeniedLandedStateUnknown";
        case Action::Result::CommandDeniedNotLanded: return "CommandDeniedNotLanded";
        case Action::Result::Timeout: return "Timeout";
        case Action::Result::VtolTransitionSupportUnknown: return "VtolTransitionSupportUnknown";
        case Action::Result::NoVtolTransitionSupport: return "NoVtolTransitionSupport";
        case Action::Result::ParameterError: return "ParameterError";
        case Action::Result::Unknown: return "Unknown";
        default: return "Unknown";
    }
}

// Helper to convert MAVSDK Telemetry::Result to string
std::string telemetry_result_str(Telemetry::Result result) {
    switch (result) {
        case Telemetry::Result::Success: return "Success";
        case Telemetry::Result::NoSystem: return "NoSystem";
        case Telemetry::Result::ConnectionError: return "ConnectionError";
        case Telemetry::Result::Busy: return "Busy";
        case Telemetry::Result::CommandDenied: return "CommandDenied";
        case Telemetry::Result::Timeout: return "Timeout";
        case Telemetry::Result::Unsupported: return "Unsupported";
        default: return "Unknown";
    }
}

std::string connection_result_str(ConnectionResult conn_result) {
    switch (conn_result) {
        case ConnectionResult::Success: return "Success";
        case ConnectionResult::Timeout: return "Timeout";
        case ConnectionResult::SocketError: return "SocketError";
        case ConnectionResult::SocketConnectionError: return "SocketConnectionError";
        case ConnectionResult::ConnectionError: return "ConnectionError";
        case ConnectionResult::NotImplemented: return "NotImplemented";
        case ConnectionResult::SystemNotConnected: return "SystemNotConnected";
        case ConnectionResult::SystemBusy: return "SystemBusy";
        case ConnectionResult::CommandDenied: return "CommandDenied";
        case ConnectionResult::DestinationIpUnknown: return "DestinationIpUnknown";
        case ConnectionResult::ConnectionsExhausted: return "ConnectionsExhausted";
        case ConnectionResult::ConnectionUrlInvalid: return "ConnectionUrlInvalid";
        default: return "Unknown";
    }
}

// Mission Waypoint Structure
struct MissionWaypoint {
    double latitude_deg;
    double longitude_deg;
    float relative_altitude_m;
    float yaw_deg;
    std::string description;

    MissionWaypoint(double lat, double lon, float alt, float yaw, std::string desc = "") :
        latitude_deg(lat), longitude_deg(lon), relative_altitude_m(alt), yaw_deg(yaw), description(std::move(desc)) {}
};

namespace FlightConstants { // Constants grouped in a namespace
    const double EARTH_RADIUS_M = 6371000.0;
    const size_t WAYPOINTS_PER_SQUARE_LAYER = 4;
}

class AutonomousFlightNode : public rclcpp::Node
{
public:
    AutonomousFlightNode() : Node("autonomous_flight_node"),
                             current_mission_state_(MissionState::IDLE),
                             current_waypoint_index_(0),
                             mission_paused_(false)
    {
        this->declare_parameter<std::string>("connection_url", "udpin://0.0.0.0:14540");
        this->get_parameter("connection_url", connection_url_);
        this->declare_parameter<double>("takeoff_altitude", 2.0);
        this->get_parameter("takeoff_altitude", takeoff_altitude_m_);
        this->declare_parameter<double>("square_side_length_m", 10.0);
        this->get_parameter("square_side_length_m", square_side_length_m_);
        this->declare_parameter<double>("altitude_layer1_m", 2.0);
        this->get_parameter("altitude_layer1_m", altitude_layer1_m_);
        this->declare_parameter<double>("altitude_layer2_m", 4.0);
        this->get_parameter("altitude_layer2_m", altitude_layer2_m_);

        RCLCPP_INFO(this->get_logger(), "Connecting to MAVSDK with URL: %s", connection_url_.c_str());
        mavsdk_ = std::make_unique<Mavsdk>(Mavsdk::Configuration{ComponentType::GroundStation});
        ConnectionResult connection_result = mavsdk_->add_any_connection(connection_url_);

        if (connection_result != ConnectionResult::Success) {
            RCLCPP_ERROR(this->get_logger(), "MAVSDK connection failed: %s", connection_result_str(connection_result).c_str());
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Waiting for system to connect...");
        auto prom = std::promise<std::shared_ptr<System>>{};
        auto fut = prom.get_future();
        mavsdk_->subscribe_on_new_system([this, &prom]() {
            auto system = mavsdk_->systems().front();
            if (system && system->is_connected()) {
                RCLCPP_INFO(this->get_logger(), "System discovered (ID: %u).", static_cast<unsigned int>(system->get_system_id()));
                prom.set_value(system);
            } else if (system) {
                 RCLCPP_WARN(this->get_logger(), "System found but not connected yet.");
            } else {
                 RCLCPP_WARN(this->get_logger(), "No system found yet.");
            }
        });

        if (fut.wait_for(seconds(15)) == std::future_status::timeout) { // Increased timeout
            RCLCPP_ERROR(this->get_logger(), "Timed out waiting for system to connect.");
            rclcpp::shutdown();
            return;
        }
        system_ = fut.get();

        if (!system_ || !system_->is_connected()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get a connected system.");
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "System connected.");

        telemetry_ = std::make_shared<Telemetry>(system_);
        action_ = std::make_shared<Action>(system_);

        const auto set_pos_rate_result = telemetry_->set_rate_position(5.0);
        if (set_pos_rate_result != Telemetry::Result::Success) {
            RCLCPP_WARN(this->get_logger(), "Setting position rate failed: %s", telemetry_result_str(set_pos_rate_result).c_str());
        }

        altitude_publisher_ = this->create_publisher<std_msgs::msg::Float32>("altitude", 10);
        
        telemetry_->subscribe_position([this](Telemetry::Position position) {
            auto msg = std_msgs::msg::Float32();
            msg.data = position.relative_altitude_m;
            altitude_publisher_->publish(msg);

            std::lock_guard<std::mutex> lock(position_mutex_);
            last_known_position_ = position;
            if (!has_home_position_ && std::abs(position.latitude_deg) > 1e-6 && std::abs(position.longitude_deg) > 1e-6) { // Check for non-zero lat/lon
                home_latitude_deg_ = position.latitude_deg;
                home_longitude_deg_ = position.longitude_deg;
                home_absolute_altitude_m_ = position.absolute_altitude_m;
                has_home_position_ = true;
                RCLCPP_INFO(this->get_logger(), "Home position set: Lat=%.6f, Lon=%.6f, AltAMSL=%.2f m",
                            home_latitude_deg_, home_longitude_deg_, home_absolute_altitude_m_);
            }
            position_valid_ = true;
        });

        target_waypoint_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "set_target_waypoint", 10, std::bind(&AutonomousFlightNode::targetWaypointCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Node initialized. Starting autonomous mission thread.");
        mission_thread_ = std::thread(&AutonomousFlightNode::run_mission_fsm, this);
    }

    ~AutonomousFlightNode() {
        // Signal threads to stop
        currently_navigating_to_waypoint_ = false;
        
        // Join all threads
        if (mission_thread_.joinable()) {
            mission_thread_.join();
        }
        
        if (position_monitor_thread_.joinable()) {
            position_monitor_thread_.join();
        }
    }

private:
    enum class MissionState {
        IDLE,
        INITIALIZING,
        TAKING_OFF,
        FLYING_WAYPOINT,
        WAYPOINT_REACHED,
        LANDING,
        MISSION_COMPLETE,
        ERROR
    };

    void offset_lat_lon(double lat_in_deg, double lon_in_deg, double north_m, double east_m,
                        double& lat_out_deg, double& lon_out_deg) const {
        double lat_in_rad = lat_in_deg * M_PI / 180.0;
        lat_out_deg = lat_in_deg  + (north_m / FlightConstants::EARTH_RADIUS_M) * (180.0 / M_PI);
        lon_out_deg = lon_in_deg + (east_m / FlightConstants::EARTH_RADIUS_M) * (180.0 / M_PI) / cos(lat_in_rad);
    }

    void prepare_mission_waypoints() {
        waypoints_.clear();
        current_waypoint_index_.store(0); // Reset atomic index

        if (!has_home_position_) {
            RCLCPP_ERROR(this->get_logger(), "Home position not yet available. Cannot prepare waypoints.");
            current_mission_state_.store(MissionState::ERROR);
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Preparing waypoints relative to home: Lat=%.6f, Lon=%.6f",
                    home_latitude_deg_, home_longitude_deg_);

        float side = static_cast<float>(square_side_length_m_);
        float alt1 = static_cast<float>(altitude_layer1_m_);
        float alt2 = static_cast<float>(altitude_layer2_m_);
        double lat_wp, lon_wp;

        // Layer 1 (e.g., 2m) - Clockwise square
        offset_lat_lon(home_latitude_deg_, home_longitude_deg_, side / 2.0, side / 2.0, lat_wp, lon_wp);
        waypoints_.emplace_back(lat_wp, lon_wp, alt1, 45.0f, "L1_C1_NE");
        offset_lat_lon(home_latitude_deg_, home_longitude_deg_, -side / 2.0, side / 2.0, lat_wp, lon_wp);
        waypoints_.emplace_back(lat_wp, lon_wp, alt1, 135.0f, "L1_C2_SE");
        offset_lat_lon(home_latitude_deg_, home_longitude_deg_, -side / 2.0, -side / 2.0, lat_wp, lon_wp);
        waypoints_.emplace_back(lat_wp, lon_wp, alt1, 225.0f, "L1_C3_SW");
        offset_lat_lon(home_latitude_deg_, home_longitude_deg_, side / 2.0, -side / 2.0, lat_wp, lon_wp);
        waypoints_.emplace_back(lat_wp, lon_wp, alt1, 315.0f, "L1_C4_NW");

        // Layer 2 (e.g., 4m) - Clockwise square (or different pattern)
        offset_lat_lon(home_latitude_deg_, home_longitude_deg_, side / 2.0, side / 2.0, lat_wp, lon_wp);
        waypoints_.emplace_back(lat_wp, lon_wp, alt2, 45.0f, "L2_C1_NE");
        offset_lat_lon(home_latitude_deg_, home_longitude_deg_, -side / 2.0, side / 2.0, lat_wp, lon_wp);
        waypoints_.emplace_back(lat_wp, lon_wp, alt2, 135.0f, "L2_C2_SE");
        offset_lat_lon(home_latitude_deg_, home_longitude_deg_, -side / 2.0, -side / 2.0, lat_wp, lon_wp);
        waypoints_.emplace_back(lat_wp, lon_wp, alt2, 225.0f, "L2_C3_SW");
        offset_lat_lon(home_latitude_deg_, home_longitude_deg_, side / 2.0, -side / 2.0, lat_wp, lon_wp);
        waypoints_.emplace_back(lat_wp, lon_wp, alt2, 315.0f, "L2_C4_NW");

        RCLCPP_INFO(this->get_logger(), "Prepared %zu waypoints.", waypoints_.size());
        for(size_t i=0; i < waypoints_.size(); ++i) {
            RCLCPP_DEBUG(this->get_logger(), "WP%zu: Lat=%.6f, Lon=%.6f, AltRel=%.1fm, Yaw=%.1fdeg (%s)",
                        i, waypoints_[i].latitude_deg, waypoints_[i].longitude_deg,
                        waypoints_[i].relative_altitude_m, waypoints_[i].yaw_deg, waypoints_[i].description.c_str());
        }
    }

    void run_mission_fsm() {
        current_mission_state_.store(MissionState::INITIALIZING);

        while (rclcpp::ok()) {
            if (mission_paused_.load()) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Mission paused. Waiting to resume or for new command.");
                sleep_for(milliseconds(100));
                continue;
            }

            MissionState current_state_loaded = current_mission_state_.load(); // Load state once per iteration

            switch (current_state_loaded) {
                case MissionState::IDLE:
                    current_mission_state_.store(MissionState::INITIALIZING);
                    break;

                case MissionState::INITIALIZING:
                    RCLCPP_INFO(this->get_logger(), "State: INITIALIZING");
                    if (!wait_for_health_ok()) {
                        RCLCPP_ERROR(this->get_logger(), "Vehicle health checks failed.");
                        current_mission_state_.store(MissionState::ERROR);
                        break;
                    }
                    if (!wait_for_home_position()) {
                        RCLCPP_ERROR(this->get_logger(), "Failed to get home position.");
                        current_mission_state_.store(MissionState::ERROR);
                        break;
                    }
                    prepare_mission_waypoints();
                    if (waypoints_.empty()) {
                        RCLCPP_ERROR(this->get_logger(), "No waypoints prepared. Aborting.");
                        current_mission_state_.store(MissionState::ERROR);
                        break;
                    }
                    RCLCPP_INFO(this->get_logger(), "Setting takeoff altitude to %.1f m (AMSL: %.1f m)",
                                altitude_layer1_m_, home_absolute_altitude_m_ + altitude_layer1_m_);
                    {
                        Action::Result set_alt_res = action_->set_takeoff_altitude(static_cast<float>(altitude_layer1_m_)); // Use desired first layer alt
                        if (set_alt_res != Action::Result::Success) {
                            RCLCPP_ERROR(this->get_logger(), "Failed to set takeoff altitude: %s.", action_result_str(set_alt_res).c_str());
                            current_mission_state_.store(MissionState::ERROR);
                            break;
                        }
                    }
                    RCLCPP_INFO(this->get_logger(), "Arming vehicle...");
                    {
                        const Action::Result arm_result = action_->arm();
                        if (arm_result != Action::Result::Success) {
                            RCLCPP_ERROR(this->get_logger(), "Arming failed: %s.", action_result_str(arm_result).c_str());
                            current_mission_state_.store(MissionState::ERROR);
                            break;
                        }
                    }
                    current_mission_state_.store(MissionState::TAKING_OFF);
                    break;

                case MissionState::TAKING_OFF:
                    RCLCPP_INFO(this->get_logger(), "State: TAKING_OFF");
                    RCLCPP_INFO(this->get_logger(), "Taking off...");
                    {
                        const Action::Result takeoff_result = action_->takeoff();
                        if (takeoff_result != Action::Result::Success) {
                            RCLCPP_ERROR(this->get_logger(), "Takeoff failed: %s.", action_result_str(takeoff_result).c_str());
                            action_->disarm();
                            current_mission_state_.store(MissionState::ERROR);
                            break;
                        }
                    }
                    while (rclcpp::ok()) {
                        Telemetry::Position current_pos;
                        bool pos_ok = false;
                        {
                            std::lock_guard<std::mutex> lock(position_mutex_);
                            if(position_valid_) {
                                current_pos = last_known_position_;
                                pos_ok = true;
                            }
                        }
                        if (pos_ok && current_pos.relative_altitude_m >= altitude_layer1_m_ * 0.90) {
                            RCLCPP_INFO(this->get_logger(), "Reached target takeoff altitude of approx %.1f m.", current_pos.relative_altitude_m);
                            current_mission_state_.store(MissionState::FLYING_WAYPOINT);
                            goto_next_waypoint();
                            break;
                        }
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting to reach altitude... Current rel: %.1f m", pos_ok ? current_pos.relative_altitude_m : -1.0f);
                        sleep_for(milliseconds(500));
                        if (current_mission_state_.load() == MissionState::ERROR) break;
                    }
                    break;

                case MissionState::FLYING_WAYPOINT:
                    {
                        size_t flying_idx = current_waypoint_index_.load();
                        if (flying_idx < waypoints_.size()) {
                            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "State: FLYING_WAYPOINT to %zu (%s)",
                                                 flying_idx, waypoints_[flying_idx].description.c_str());
                        } else {
                            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                                 "State: FLYING_WAYPOINT - Invalid current_waypoint_index %zu", flying_idx);
                        }
                        sleep_for(milliseconds(100));
                    }
                    break;

                case MissionState::WAYPOINT_REACHED:
                    {
                        size_t completed_wp_idx_plus_one = current_waypoint_index_.load();
                        if (completed_wp_idx_plus_one > 0 && (completed_wp_idx_plus_one - 1) < waypoints_.size()) {
                             RCLCPP_INFO(this->get_logger(), "State: WAYPOINT_REACHED (Completed Waypoint %zu: %s)",
                                        completed_wp_idx_plus_one - 1, waypoints_[completed_wp_idx_plus_one - 1].description.c_str());
                        } else {
                            RCLCPP_WARN(this->get_logger(), "State: WAYPOINT_REACHED - Log index issue (current_waypoint_index: %zu).", completed_wp_idx_plus_one);
                        }

                        if (completed_wp_idx_plus_one >= waypoints_.size()) {
                            RCLCPP_INFO(this->get_logger(), "All waypoints completed.");
                            current_mission_state_.store(MissionState::LANDING);
                        } else {
                            if (completed_wp_idx_plus_one == FlightConstants::WAYPOINTS_PER_SQUARE_LAYER && waypoints_.size() > FlightConstants::WAYPOINTS_PER_SQUARE_LAYER) {
                                 RCLCPP_INFO(this->get_logger(), "First layer completed. Moving to second layer altitude.");
                            }
                            goto_next_waypoint();
                        }
                    }
                    break;

                case MissionState::LANDING:
                    RCLCPP_INFO(this->get_logger(), "State: LANDING");
                    RCLCPP_INFO(this->get_logger(), "Landing...");
                    {
                        const Action::Result land_result = action_->land();
                        if (land_result != Action::Result::Success) {
                            RCLCPP_ERROR(this->get_logger(), "Landing failed: %s.", action_result_str(land_result).c_str());
                            current_mission_state_.store(MissionState::ERROR);
                        } else {
                             RCLCPP_INFO(this->get_logger(), "Landing initiated. Monitoring until landed.");
                        }
                    }
                    while (rclcpp::ok() && telemetry_->in_air()) {
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Vehicle is landing...");
                        sleep_for(seconds(1));
                        if (current_mission_state_.load() == MissionState::ERROR) break;
                    }
                    if (!rclcpp::ok()) break;
                    if (!telemetry_->in_air()){
                         RCLCPP_INFO(this->get_logger(), "Vehicle has landed.");
                    } else {
                         RCLCPP_WARN(this->get_logger(), "Vehicle still reporting in_air after land sequence.");
                    }
                    current_mission_state_.store(MissionState::MISSION_COMPLETE);
                    break;

                case MissionState::MISSION_COMPLETE:
                    RCLCPP_INFO(this->get_logger(), "State: MISSION_COMPLETE");
                    RCLCPP_INFO(this->get_logger(), "Disarming...");
                    {
                        const Action::Result disarm_result = action_->disarm();
                        if (disarm_result != Action::Result::Success) {
                            RCLCPP_ERROR(this->get_logger(), "Disarming failed: %s.", action_result_str(disarm_result).c_str());
                        } else {
                            RCLCPP_INFO(this->get_logger(), "Vehicle disarmed.");
                        }
                    }
                    RCLCPP_INFO(this->get_logger(), "Autonomous mission finished. Shutting down node.");
                    rclcpp::shutdown();
                    return;

                case MissionState::ERROR:
                    RCLCPP_ERROR(this->get_logger(), "State: ERROR. Mission aborted.");
                    if (system_ && system_->is_connected() && telemetry_ && telemetry_->armed()) {
                        RCLCPP_WARN(this->get_logger(), "Attempting to disarm due to error.");
                        action_->disarm();
                    }
                    rclcpp::shutdown();
                    return;
                
                default:
                    RCLCPP_WARN(this->get_logger(), "Unhandled mission state: %d!", static_cast<int>(current_state_loaded));
                    sleep_for(seconds(1));
                    break;
            }
        }
    }

    void goto_next_waypoint() {
        size_t wp_idx_to_process = current_waypoint_index_.load();

        if (wp_idx_to_process >= waypoints_.size()) {
            RCLCPP_ERROR(this->get_logger(), "Waypoint index %zu out of bounds for goto_next_waypoint!", wp_idx_to_process);
            current_mission_state_.store(MissionState::ERROR);
            return;
        }

        const auto& wp = waypoints_[wp_idx_to_process];
        RCLCPP_INFO(this->get_logger(), "Moving to waypoint %zu: %s (Lat: %.6f, Lon: %.6f, AltRel: %.1fm, AltAMSL: %.1fm, Yaw: %.1f deg)",
                    wp_idx_to_process,
                    wp.description.c_str(),
                    wp.latitude_deg,
                    wp.longitude_deg,
                    wp.relative_altitude_m,
                    home_absolute_altitude_m_ + wp.relative_altitude_m,
                    wp.yaw_deg);

        float target_altitude_amsl = home_absolute_altitude_m_ + wp.relative_altitude_m;

        // Set flag to indicate we're actively monitoring waypoint progress
        currently_navigating_to_waypoint_ = true;
        target_waypoint_ = wp;

        action_->goto_location_async(
            wp.latitude_deg,
            wp.longitude_deg,
            target_altitude_amsl,
            wp.yaw_deg,
            [this, captured_idx = wp_idx_to_process](Action::Result result) {
                if (mission_paused_.load()) return;

                if (result == Action::Result::Success) {
                    RCLCPP_INFO(this->get_logger(), "Command to waypoint %zu: %s accepted",
                                captured_idx, waypoints_[captured_idx].description.c_str());
                    // We don't mark waypoint as reached here - we'll do that in position monitoring
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to issue command to waypoint %zu (%s): %s.",
                                captured_idx, waypoints_[captured_idx].description.c_str(), action_result_str(result).c_str());
                    current_mission_state_.store(MissionState::ERROR);
                    currently_navigating_to_waypoint_ = false;
                }
            });

        // Start the waypoint position monitoring
        if (!position_monitor_thread_.joinable()) {
            position_monitor_thread_ = std::thread(&AutonomousFlightNode::monitor_waypoint_progress, this);
        }
    }

    // New function to monitor waypoint progress
    void monitor_waypoint_progress() {
        const double POSITION_THRESHOLD_M = 1.0; // Distance threshold to consider waypoint reached (meters)
        const double ALTITUDE_THRESHOLD_M = 0.5; // Altitude threshold (meters)
        const double YAW_THRESHOLD_DEG = 15.0;   // Yaw threshold (degrees)
        const int MAX_ATTEMPTS = 60;             // Max number of attempts (60 * 1s = 60 seconds timeout)
        
        int attempts = 0;
        
        while (rclcpp::ok() && currently_navigating_to_waypoint_) {
            Telemetry::Position current_pos;
            bool pos_ok = false;
            
            {
                std::lock_guard<std::mutex> lock(position_mutex_);
                if(position_valid_) {
                    current_pos = last_known_position_;
                    pos_ok = true;
                }
            }
            
            if (pos_ok) {
                // Calculate 2D distance to waypoint (using Haversine formula for small distances)
                double lat1_rad = current_pos.latitude_deg * M_PI / 180.0;
                double lon1_rad = current_pos.longitude_deg * M_PI / 180.0;
                double lat2_rad = target_waypoint_.latitude_deg * M_PI / 180.0;
                double lon2_rad = target_waypoint_.longitude_deg * M_PI / 180.0;
                
                double dlon = lon2_rad - lon1_rad;
                double dlat = lat2_rad - lat1_rad;
                double a = sin(dlat/2) * sin(dlat/2) + cos(lat1_rad) * cos(lat2_rad) * sin(dlon/2) * sin(dlon/2);
                double c = 2 * atan2(sqrt(a), sqrt(1-a));
                double distance_m = FlightConstants::EARTH_RADIUS_M * c;
                
                // Calculate altitude difference
                double alt_diff_m = std::abs(current_pos.relative_altitude_m - target_waypoint_.relative_altitude_m);
                
                // Get current yaw (simplified - consider adding yaw tracking to telemetry callback)
                float current_yaw_deg = 0.0; // In a full implementation, you'd track this from telemetry
                double yaw_diff_deg = std::abs(current_yaw_deg - target_waypoint_.yaw_deg);
                if (yaw_diff_deg > 180.0) yaw_diff_deg = 360.0 - yaw_diff_deg; // Handle yaw wraparound
                
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                                "WP Progress: Dist=%.2fm (thresh=%.1f), Alt diff=%.2fm (thresh=%.1f)",
                                distance_m, POSITION_THRESHOLD_M, alt_diff_m, ALTITUDE_THRESHOLD_M);
                
                // Check if waypoint reached
                if (distance_m < POSITION_THRESHOLD_M && alt_diff_m < ALTITUDE_THRESHOLD_M) {
                    RCLCPP_INFO(this->get_logger(), "Successfully reached waypoint %zu: %s (Distance: %.2fm, Alt diff: %.2fm)",
                            current_waypoint_index_.load(), target_waypoint_.description.c_str(), distance_m, alt_diff_m);
                    
                    currently_navigating_to_waypoint_ = false;
                    current_waypoint_index_++;
                    current_mission_state_.store(MissionState::WAYPOINT_REACHED);
                    return;
                }
            }
            
            attempts++;
            if (attempts >= MAX_ATTEMPTS) {
                RCLCPP_WARN(this->get_logger(), "Timeout reaching waypoint %zu: %s. Moving to next waypoint.",
                        current_waypoint_index_.load(), target_waypoint_.description.c_str());
                currently_navigating_to_waypoint_ = false;
                current_waypoint_index_++;
                current_mission_state_.store(MissionState::WAYPOINT_REACHED);
                return;
            }
            
            sleep_for(seconds(1));
        }
    }

    bool wait_for_health_ok() {
        RCLCPP_INFO(this->get_logger(), "Waiting for vehicle to be ready to arm (health checks)...");
        rclcpp::Rate r(1.0); // 1 Hz
        while (rclcpp::ok()) {
            if (telemetry_ && telemetry_->health_all_ok()) {
                RCLCPP_INFO(this->get_logger(), "Vehicle health is OK.");
                return true;
            }
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Vehicle not healthy yet...");
            r.sleep();
        }
        return false;
    }

    bool wait_for_home_position() {
        RCLCPP_INFO(this->get_logger(), "Waiting for home position to be set (valid GPS fix)...");
        rclcpp::Rate r(1.0); // 1 Hz
        while(rclcpp::ok() && !has_home_position_) { // has_home_position_ is not atomic, but only written by telemetry callback, read by mission thread
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Home position not available yet...");
            r.sleep();
        }
        return has_home_position_;
    }

    void targetWaypointCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received new target waypoint request: Pos(%.2f, %.2f, %.2f) from frame '%s'",
                    msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, msg->header.frame_id.c_str());

        MissionState current_state = current_mission_state_.load();
        if (current_state != MissionState::FLYING_WAYPOINT && current_state != MissionState::WAYPOINT_REACHED && current_state != MissionState::TAKING_OFF) {
             RCLCPP_WARN(this->get_logger(), "Not in an appropriate state (%d) to accept new dynamic waypoints. Ignoring.", static_cast<int>(current_state));
             return;
        }
        if (!has_home_position_) {
            RCLCPP_WARN(this->get_logger(), "Home position not set yet. Cannot process dynamic waypoint relative to home. Ignoring.");
            return;
        }

        double new_lat, new_lon;
        float new_rel_alt = static_cast<float>(msg->pose.position.z);
        // Assuming msg->pose.position.x is North offset, y is East offset from home
        offset_lat_lon(home_latitude_deg_, home_longitude_deg_, msg->pose.position.x, msg->pose.position.y, new_lat, new_lon);

        double qx = msg->pose.orientation.x;
        double qy = msg->pose.orientation.y;
        double qz = msg->pose.orientation.z;
        double qw = msg->pose.orientation.w;
        double siny_cosp = 2 * (qw * qz + qx * qy);
        double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
        float new_yaw_deg = static_cast<float>(atan2(siny_cosp, cosy_cosp) * 180.0 / M_PI);

        RCLCPP_INFO(this->get_logger(), "Calculated new dynamic target (LLA): Lat=%.6f, Lon=%.6f, AltRel=%.1fm, Yaw=%.1fdeg",
                    new_lat, new_lon, new_rel_alt, new_yaw_deg);

        size_t wp_idx_to_update = current_waypoint_index_.load();
        if (wp_idx_to_update < waypoints_.size()) {
            // This overwrites the *next* scheduled waypoint.
            // For immediate redirection, MAVSDK's action cancel/override behavior needs to be understood.
            // A robust solution might involve a more complex state to handle interruption.
            RCLCPP_INFO(this->get_logger(), "Updating waypoint %zu with new dynamically received target.", wp_idx_to_update);
            waypoints_[wp_idx_to_update] = MissionWaypoint(new_lat, new_lon, new_rel_alt, new_yaw_deg, "DYN_WP_" + std::to_string(wp_idx_to_update));
            RCLCPP_INFO(this->get_logger(), "Waypoint %zu updated. It will be targeted after the current segment or on next FSM decision.", wp_idx_to_update);
            // If you want to attempt to go there NOW (and current_waypoint_index_ points to it):
            // if (current_mission_state_.load() == MissionState::WAYPOINT_REACHED || current_mission_state_.load() == MissionState::TAKING_OFF) {
            //    current_mission_state_.store(MissionState::FLYING_WAYPOINT);
            //    goto_next_waypoint(); // Will use the just-updated waypoint
            // } // This needs careful thought about race conditions with the async callback.
        } else {
            RCLCPP_WARN(this->get_logger(), "Cannot update waypoint: index %zu is out of bounds (waypoints size: %zu). Ignoring new target.",
                        wp_idx_to_update, waypoints_.size());
        }
    }

    // Member Variables
    std::string connection_url_;
    double takeoff_altitude_m_; // Used for initial takeoff altitude setting if different from first layer
    double square_side_length_m_;
    double altitude_layer1_m_;
    double altitude_layer2_m_;

    std::unique_ptr<Mavsdk> mavsdk_;
    std::shared_ptr<System> system_;
    std::shared_ptr<Telemetry> telemetry_;
    std::shared_ptr<Action> action_;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr altitude_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_waypoint_subscriber_;
    
    std::thread mission_thread_;

    std::mutex position_mutex_;
    Telemetry::Position last_known_position_;
    bool position_valid_ = false;
    double home_latitude_deg_ = 0.0;
    double home_longitude_deg_ = 0.0;
    float home_absolute_altitude_m_ = 0.0f;
    std::atomic<bool> has_home_position_{false}; // Made atomic for clearer multi-thread access intent

    std::atomic<MissionState> current_mission_state_;
    std::vector<MissionWaypoint> waypoints_; // Waypoints are prepared once, then potentially modified by callback
    std::atomic<size_t> current_waypoint_index_;
    std::atomic<bool> mission_paused_;


    std::atomic<bool> currently_navigating_to_waypoint_{false};
    MissionWaypoint target_waypoint_{0, 0, 0, 0, ""};  // Current target waypoint
    std::thread position_monitor_thread_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutonomousFlightNode>();
    if (rclcpp::ok()) { 
        rclcpp::spin(node);
    }
    // rclcpp::shutdown(); // Called from within run_mission_fsm
    return 0;
}

//holytorch