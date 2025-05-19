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
#include <atomic>     // For std::atomic

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp" // For bonus mission

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
// #include <mavsdk/plugins/offboard/offboard.h> // 현재 사용 안함

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
        default: return "Unknown result"; // Added default for safety
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
        case Telemetry::Result::Unknown: return "Unknown"; // Added case
        default: return "Unknown telemetry result"; // Added default
    }
}

std::string connection_result_str(ConnectionResult conn_result) {
    switch (conn_result) {
        case ConnectionResult::Success: return "Success";
        case ConnectionResult::Timeout: return "Timeout";
        case ConnectionResult::SocketError: return "SocketError";
        case ConnectionResult::SocketConnectionError: return "SocketConnectionError";
        case ConnectionResult::ConnectionError: return "ConnectionError"; // MAVSDK v0.x와 v1.x에서 이름이 다를 수 있음, 여기서는 mavsdk::ConnectionResult::ConnectionError
        case ConnectionResult::NotImplemented: return "NotImplemented";
        case ConnectionResult::SystemNotConnected: return "SystemNotConnected";
        case ConnectionResult::SystemBusy: return "SystemBusy";
        case ConnectionResult::CommandDenied: return "CommandDenied";
        case ConnectionResult::DestinationIpUnknown: return "DestinationIpUnknown";
        case ConnectionResult::ConnectionsExhausted: return "ConnectionsExhausted";
        case ConnectionResult::ConnectionUrlInvalid: return "ConnectionUrlInvalid";
        // case ConnectionResult::BaudrateUnknown: return "BaudrateUnknown"; // MAVSDK 버전에 따라 없을 수 있음
        // case ConnectionResult::InvalidUrl: return "InvalidUrl"; // MAVSDK 버전에 따라 없을 수 있음
        // case ConnectionResult::PluginUnsupported: return "PluginUnsupported"; // MAVSDK 버전에 따라 없을 수 있음
        default: return "Unknown connection result";
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
                             mission_paused_(false),
                             target_waypoint_(0,0,0,0,"") // Initialize target_waypoint_
    {
        this->declare_parameter<std::string>("connection_url", "udpin://0.0.0.0:14540");
        this->get_parameter("connection_url", connection_url_);
        // REMOVED: this->declare_parameter<double>("takeoff_altitude", 2.0);
        // REMOVED: this->get_parameter("takeoff_altitude", takeoff_altitude_m_);
        this->declare_parameter<double>("square_side_length_m", 10.0);
        this->get_parameter("square_side_length_m", square_side_length_m_);
        this->declare_parameter<double>("altitude_layer1_m", 2.0);
        this->get_parameter("altitude_layer1_m", altitude_layer1_m_);
        this->declare_parameter<double>("altitude_layer2_m", 4.0);
        this->get_parameter("altitude_layer2_m", altitude_layer2_m_);

        this->declare_parameter<double>("waypoint_threshold_m", 1.0);
        this->get_parameter("waypoint_threshold_m", waypoint_threshold_m_);

        RCLCPP_INFO(this->get_logger(), "Connecting to MAVSDK with URL: %s", connection_url_.c_str());
        
        // Use COMPONENT_AUTOPILOT or COMPONENT_ONBOARD_COMPUTER if this node is running on the drone.
        // Use COMPONENT_GROUND_STATION if this node is external control.
        mavsdk_ = std::make_unique<Mavsdk>(Mavsdk::Configuration{ComponentType::GroundStation});
        ConnectionResult connection_result = mavsdk_->add_any_connection(connection_url_);

        if (connection_result != ConnectionResult::Success) {
            RCLCPP_ERROR(this->get_logger(), "MAVSDK connection failed: %s", connection_result_str(connection_result).c_str());
            // rclcpp::shutdown(); // Shutdown should be handled more gracefully or let the main handle it
            throw std::runtime_error("MAVSDK connection failed"); // Propagate error
        }

        RCLCPP_INFO(this->get_logger(), "Waiting for system to connect...");
        auto prom = std::promise<std::shared_ptr<System>>{};
        auto fut = prom.get_future();
        
        // This lambda might be called after this constructor finishes if connection is slow.
        // Ensure `this` is valid. A std::weak_ptr could be used for safety if needed.
        mavsdk_->subscribe_on_new_system([this, &prom]() {
            if (!mavsdk_) { // mavsdk_ might have been reset if node is shutting down
                 RCLCPP_WARN(this->get_logger(), "subscribe_on_new_system callback: MAVSDK object no longer valid.");
                return;
            }
            auto system = mavsdk_->systems().front(); // Get the first system
            if (system && system->is_connected()) {
                RCLCPP_INFO(this->get_logger(), "System discovered (ID: %u).", static_cast<unsigned int>(system->get_system_id()));
                prom.set_value(system); // Fulfill the promise
            } else if (system) {
                 RCLCPP_WARN(this->get_logger(), "System found but not connected yet.");
            } else {
                 RCLCPP_WARN(this->get_logger(), "No system found yet in subscribe_on_new_system callback.");
            }
        });

        if (fut.wait_for(seconds(15)) == std::future_status::timeout) {
            RCLCPP_ERROR(this->get_logger(), "Timed out waiting for system to connect.");
            // rclcpp::shutdown();
            throw std::runtime_error("Timed out waiting for system");
        }
        system_ = fut.get(); // This can throw if promise was not fulfilled

        if (!system_ || !system_->is_connected()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get a connected system.");
            // rclcpp::shutdown();
            throw std::runtime_error("Failed to get connected system");
        }
        RCLCPP_INFO(this->get_logger(), "System connected.");

        telemetry_ = std::make_shared<Telemetry>(system_);
        action_ = std::make_shared<Action>(system_);

        const auto set_pos_rate_result = telemetry_->set_rate_position(5.0); // Hz
        if (set_pos_rate_result != Telemetry::Result::Success) {
            RCLCPP_WARN(this->get_logger(), "Setting position rate failed: %s", telemetry_result_str(set_pos_rate_result).c_str());
        }
        // Consider subscribing to telemetry_->subscribe_attitude_euler() or quaternion for yaw if needed

        altitude_publisher_ = this->create_publisher<std_msgs::msg::Float32>("altitude", 10);
        
        telemetry_->subscribe_position([this](Telemetry::Position position) {
            if (!rclcpp::ok()) return; // Node might be shutting down
            auto msg = std_msgs::msg::Float32();
            msg.data = position.relative_altitude_m;
            altitude_publisher_->publish(msg);

            std::lock_guard<std::mutex> lock(position_mutex_);
            last_known_position_ = position;
            if (!has_home_position_.load() && std::abs(position.latitude_deg) > 1e-6 && std::abs(position.longitude_deg) > 1e-6) {
                home_latitude_deg_ = position.latitude_deg;
                home_longitude_deg_ = position.longitude_deg;
                home_absolute_altitude_m_ = position.absolute_altitude_m;
                has_home_position_.store(true);
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
        RCLCPP_INFO(this->get_logger(), "AutonomousFlightNode destructor: Starting cleanup...");

        // Signal all threads to stop their work and exit loops
        // Set mission_state to a terminal state if it's used by threads to loop
        current_mission_state_.store(MissionState::ERROR); // Or a dedicated SHUTTING_DOWN state
        mission_paused_.store(true); // If mission_thread checks this
        currently_navigating_to_waypoint_.store(false); // For position_monitor_thread

        // Join threads. They should exit cleanly now.
        if (mission_thread_.joinable()) {
            RCLCPP_INFO(this->get_logger(), "Joining mission_thread_...");
            mission_thread_.join();
            RCLCPP_INFO(this->get_logger(), "mission_thread_ joined.");
        }
        
        if (position_monitor_thread_.joinable()) {
            RCLCPP_INFO(this->get_logger(), "Joining position_monitor_thread_...");
            position_monitor_thread_.join();
            RCLCPP_INFO(this->get_logger(), "position_monitor_thread_ joined.");
        }
        
        RCLCPP_INFO(this->get_logger(), "All threads joined. Proceeding with MAVSDK object cleanup.");

        // MAVSDK objects cleanup. Order can be important.
        // Plugins should be reset before the system_ object.
        if (action_) {
            RCLCPP_INFO(this->get_logger(), "Resetting MAVSDK Action plugin...");
            action_.reset();
        }
        if (telemetry_) {
            RCLCPP_INFO(this->get_logger(), "Resetting MAVSDK Telemetry plugin...");
            // MAVSDK typically handles unsubscribing internally when the Telemetry object is destroyed/reset.
            // If explicit unsubscribe methods were available for specific subscriptions, they could be called here.
            telemetry_.reset();
        }
        if (system_) {
            RCLCPP_INFO(this->get_logger(), "Resetting MAVSDK System object...");
            system_.reset();
        }
        if (mavsdk_) {
             RCLCPP_INFO(this->get_logger(), "Resetting MAVSDK core object (unique_ptr will do this)...");
             mavsdk_.reset(); // Explicitly reset unique_ptr, or let it go out of scope.
        }
        
        RCLCPP_INFO(this->get_logger(), "AutonomousFlightNode destructor: Cleanup finished.");
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
        // Consider adding SHUTTING_DOWN state
    };

    void offset_lat_lon(double lat_in_deg, double lon_in_deg, double north_m, double east_m,
                        double& lat_out_deg, double& lon_out_deg) const {
        if (std::abs(lat_in_deg) < 1e-7 && std::abs(lon_in_deg) < 1e-7) {
            RCLCPP_WARN_ONCE(this->get_logger(), "offset_lat_lon called with near-zero initial lat/lon. Results might be inaccurate if this is not intended (e.g. before home set).");
        }
        double lat_in_rad = lat_in_deg * M_PI / 180.0;
        lat_out_deg = lat_in_deg  + (north_m / FlightConstants::EARTH_RADIUS_M) * (180.0 / M_PI);
        lon_out_deg = lon_in_deg + (east_m / (FlightConstants::EARTH_RADIUS_M * cos(lat_in_rad))) * (180.0 / M_PI); // Corrected cos application
    }

    void prepare_mission_waypoints() {
        waypoints_.clear();
        current_waypoint_index_.store(0);

        if (!has_home_position_.load()) {
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

        // Layer 2 (e.g., 4m) - Clockwise square
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

        while (rclcpp::ok() && current_mission_state_.load() != MissionState::MISSION_COMPLETE && current_mission_state_.load() != MissionState::ERROR) {
            if (mission_paused_.load()) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Mission paused. Waiting to resume or for new command.");
                sleep_for(milliseconds(100));
                continue;
            }

            MissionState current_state_loaded = current_mission_state_.load(); 

            switch (current_state_loaded) {
                case MissionState::IDLE: // Should transition from IDLE quickly
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
                        // Use the altitude of the first layer as the takeoff altitude for set_takeoff_altitude
                        // This makes sense if the first waypoint is at this altitude.
                        Action::Result set_alt_res = action_->set_takeoff_altitude(static_cast<float>(altitude_layer1_m_));
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
                            // Attempt to disarm if takeoff fails and vehicle was armed.
                            if (telemetry_ && telemetry_->armed()) {
                                action_->disarm();
                            }
                            current_mission_state_.store(MissionState::ERROR);
                            break;
                        }
                    }
                    // Wait to reach takeoff altitude
                    while (rclcpp::ok() && current_mission_state_.load() == MissionState::TAKING_OFF) {
                        Telemetry::Position current_pos;
                        bool pos_ok = false;
                        {
                            std::lock_guard<std::mutex> lock(position_mutex_);
                            if(position_valid_) {
                                current_pos = last_known_position_;
                                pos_ok = true;
                            }
                        }
                        // Using altitude_layer1_m_ as the target takeoff altitude check
                        if (pos_ok && current_pos.relative_altitude_m >= altitude_layer1_m_ * 0.90) {
                            RCLCPP_INFO(this->get_logger(), "Reached target takeoff altitude of approx %.1f m.", current_pos.relative_altitude_m);
                            current_mission_state_.store(MissionState::FLYING_WAYPOINT);
                            goto_next_waypoint(); // This will also start position_monitor_thread
                            break; 
                        }
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting to reach altitude... Current rel: %.1f m", pos_ok ? current_pos.relative_altitude_m : -1.0f);
                        sleep_for(milliseconds(500)); 
                    }
                    break;

                case MissionState::FLYING_WAYPOINT:
                    {
                        size_t flying_idx = current_waypoint_index_.load();
                        if (flying_idx < waypoints_.size()) {
                            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "State: FLYING_WAYPOINT to %zu (%s)",
                                                 flying_idx, waypoints_[flying_idx].description.c_str());
                        } else {
                             // This case should ideally not be reached if logic is correct,
                             // as WAYPOINT_REACHED should transition to LANDING if all waypoints done.
                            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                                 "State: FLYING_WAYPOINT - Invalid current_waypoint_index %zu (Waypoints size: %zu). Potentially all waypoints done.", 
                                                 flying_idx, waypoints_.size());
                            if (flying_idx >= waypoints_.size() && !waypoints_.empty()) {
                                RCLCPP_INFO(this->get_logger(), "All waypoints appear to be processed. Transitioning to LANDING from FLYING_WAYPOINT.");
                                current_mission_state_.store(MissionState::LANDING);
                            } else {
                                // If waypoints_ is empty, this indicates an earlier error in prepare_mission_waypoints
                                current_mission_state_.store(MissionState::ERROR);
                            }
                        }
                        sleep_for(milliseconds(100)); // Reduce CPU usage while waiting for monitor thread
                    }
                    break;

                case MissionState::WAYPOINT_REACHED:
                    {
                        size_t completed_wp_idx_plus_one = current_waypoint_index_.load(); // current_waypoint_index_ is already incremented by monitor
                        size_t completed_wp_idx = completed_wp_idx_plus_one -1;

                        if (completed_wp_idx < waypoints_.size()) { // Check if the index is valid for logging
                             RCLCPP_INFO(this->get_logger(), "State: WAYPOINT_REACHED (Completed Waypoint %zu: %s)",
                                        completed_wp_idx, waypoints_[completed_wp_idx].description.c_str());
                        } else {
                            RCLCPP_WARN(this->get_logger(), "State: WAYPOINT_REACHED - Log index issue (current_waypoint_index: %zu).", completed_wp_idx_plus_one);
                        }

                        if (current_waypoint_index_.load() >= waypoints_.size()) {
                            RCLCPP_INFO(this->get_logger(), "All waypoints completed.");
                            current_mission_state_.store(MissionState::LANDING);
                        } else {
                            if (current_waypoint_index_.load() == FlightConstants::WAYPOINTS_PER_SQUARE_LAYER && waypoints_.size() > FlightConstants::WAYPOINTS_PER_SQUARE_LAYER) {
                                 RCLCPP_INFO(this->get_logger(), "First layer completed. Moving to second layer altitude.");
                            }
                            // Transition back to FLYING_WAYPOINT to process the next one
                            current_mission_state_.store(MissionState::FLYING_WAYPOINT);
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
                            break; // Break from switch, FSM loop will handle ERROR state
                        } else {
                             RCLCPP_INFO(this->get_logger(), "Landing initiated. Monitoring until landed.");
                        }
                    }
                    // Wait until landed
                    while (rclcpp::ok() && current_mission_state_.load() == MissionState::LANDING) {
                        if (!telemetry_->in_air()) {
                            RCLCPP_INFO(this->get_logger(), "Vehicle has landed.");
                            current_mission_state_.store(MissionState::MISSION_COMPLETE);
                            break; 
                        }
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Vehicle is landing...");
                        sleep_for(seconds(1));
                    }
                    // If loop exited due to !rclcpp::ok() or state change by other means before landing
                    if (current_mission_state_.load() == MissionState::LANDING && telemetry_->in_air()) {
                        RCLCPP_WARN(this->get_logger(), "Exited landing wait loop but vehicle still in air. Shutting down or error occurred.");
                        // If rclcpp::ok() is false, it will exit the main FSM loop.
                        // If state changed to ERROR, FSM loop will handle.
                    }
                    break;

                case MissionState::MISSION_COMPLETE:
                    RCLCPP_INFO(this->get_logger(), "State: MISSION_COMPLETE");
                    RCLCPP_INFO(this->get_logger(), "Disarming...");
                    {
                        // action_ 포인터 유효성 검사 추가
                        if (action_) {
                            const Action::Result disarm_result = action_->disarm();
                            RCLCPP_INFO(this->get_logger(), "Disarm result: %s.", action_result_str(disarm_result).c_str());
                        } else {
                            RCLCPP_WARN(this->get_logger(), "Action plugin not available for disarming.");
                        }
                    }
                    RCLCPP_INFO(this->get_logger(), "Autonomous mission finished. Requesting node shutdown (reason: Mission Complete).");
                    if(rclcpp::ok()){
                        rclcpp::shutdown(this->get_node_base_interface()->get_context()); // 컨텍스트 전달
                        // 또는 간단히 rclcpp::shutdown();
                    }
                    return;

                case MissionState::ERROR:
                    RCLCPP_ERROR(this->get_logger(), "State: ERROR. Mission aborted.");
                    if (system_ && system_->is_connected() && telemetry_ && telemetry_->armed()) {
                        RCLCPP_WARN(this->get_logger(), "Attempting to disarm due to error.");
                         // action_ 포인터 유효성 검사 추가
                        if (action_) {
                            action_->disarm();
                        } else {
                            RCLCPP_WARN(this->get_logger(), "Action plugin not available for disarming in error state.");
                        }
                    }
                    RCLCPP_INFO(this->get_logger(), "Error state reached. Requesting node shutdown (reason: Mission Error).");
                     if(rclcpp::ok()){
                        rclcpp::shutdown(this->get_node_base_interface()->get_context()); // 컨텍스트 전달
                        // 또는 간단히 rclcpp::shutdown();
                     }
                    return;
                
                default:
                    RCLCPP_WARN(this->get_logger(), "Unhandled mission state: %d! Transitioning to ERROR.", static_cast<int>(current_state_loaded));
                    current_mission_state_.store(MissionState::ERROR);
                    sleep_for(seconds(1));
                    break;
            }
        }
        RCLCPP_INFO(this->get_logger(), "Exiting run_mission_fsm loop. rclcpp::ok() is %s. Final state: %d",
            rclcpp::ok() ? "true" : "false", static_cast<int>(current_mission_state_.load()));
    }

    void goto_next_waypoint() {
        size_t wp_idx_to_process = current_waypoint_index_.load();

        if (wp_idx_to_process >= waypoints_.size()) {
            RCLCPP_ERROR(this->get_logger(), "Waypoint index %zu out of bounds for goto_next_waypoint! (Size: %zu)", wp_idx_to_process, waypoints_.size());
            // This could happen if current_waypoint_index_ was incremented past the end
            // and WAYPOINT_REACHED didn't catch it to transition to LANDING.
            // Or if waypoints_ is empty.
            if (!waypoints_.empty()){ // If there were waypoints, assume all done.
                 RCLCPP_INFO(this->get_logger(), "Assuming all waypoints completed. Transitioning to LANDING from goto_next_waypoint.");
                 current_mission_state_.store(MissionState::LANDING);
            } else {
                 current_mission_state_.store(MissionState::ERROR);
            }
            return;
        }

        const auto& wp = waypoints_[wp_idx_to_process];
        RCLCPP_INFO(this->get_logger(), "Moving to waypoint %zu: %s (Lat: %.6f, Lon: %.6f, AltRel: %.1fm, AltAMSL: %.1fm, Yaw: %.1f deg)",
                    wp_idx_to_process,
                    wp.description.c_str(),
                    wp.latitude_deg,
                    wp.longitude_deg,
                    wp.relative_altitude_m,
                    home_absolute_altitude_m_ + wp.relative_altitude_m, // Calculate AMSL for goto_location
                    wp.yaw_deg);

        float target_altitude_amsl = home_absolute_altitude_m_ + wp.relative_altitude_m;

        // Set flag to indicate we're actively monitoring waypoint progress
        currently_navigating_to_waypoint_.store(true); // Atomic store
        target_waypoint_ = wp; // Update the target waypoint for the monitor thread

        action_->goto_location_async(
            wp.latitude_deg,
            wp.longitude_deg,
            target_altitude_amsl, // MAVSDK goto_location expects AMSL
            wp.yaw_deg,
            [this, captured_idx = wp_idx_to_process](Action::Result result) {
                if (mission_paused_.load() || !currently_navigating_to_waypoint_.load()) {
                    RCLCPP_INFO(this->get_logger(), "goto_location_async callback for WP %zu: Mission paused or navigation cancelled. Ignoring result.", captured_idx);
                    return;
                }

                if (result == Action::Result::Success) {
                    RCLCPP_INFO(this->get_logger(), "Command to waypoint %zu: %s accepted by vehicle.",
                                captured_idx, waypoints_[captured_idx].description.c_str());
                    // Acceptance doesn't mean reached. monitor_waypoint_progress handles reaching.
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to issue command to waypoint %zu (%s): %s.",
                                captured_idx, waypoints_[captured_idx].description.c_str(), action_result_str(result).c_str());
                    current_mission_state_.store(MissionState::ERROR);
                    currently_navigating_to_waypoint_.store(false); // Stop monitor
                }
            });

        // Start/Restart the waypoint position monitoring thread
        if (position_monitor_thread_.joinable()) {
            position_monitor_thread_.join(); 
        }
        position_monitor_thread_ = std::thread(&AutonomousFlightNode::monitor_waypoint_progress, this);
    }

    void monitor_waypoint_progress() {
        const double ALTITUDE_THRESHOLD_M = 0.5; 
        // const double YAW_THRESHOLD_DEG = 15.0; // Yaw check currently not implemented with actual yaw
        const int MAX_ATTEMPTS = 120; // Max attempts (e.g., 120 * 0.5s = 60 seconds timeout)
        const int LOOP_SLEEP_MS = 500; // Sleep duration per loop
        
        int attempts = 0;
        size_t target_idx_at_start = current_waypoint_index_.load(); // The waypoint index we are monitoring

        RCLCPP_INFO(this->get_logger(), "Monitor thread started for waypoint %zu: %s", target_idx_at_start, target_waypoint_.description.c_str());
        
        while (rclcpp::ok() && currently_navigating_to_waypoint_.load()) {
            if (current_waypoint_index_.load() != target_idx_at_start) {
                RCLCPP_WARN(this->get_logger(), "Monitor WP %zu: Target index changed externally to %zu. Exiting monitor for old WP.",
                            target_idx_at_start, current_waypoint_index_.load());
                currently_navigating_to_waypoint_.store(false); // Ensure it's false if we exit due to this
                return;
            }

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
                double lat1_rad = current_pos.latitude_deg * M_PI / 180.0;
                double lon1_rad = current_pos.longitude_deg * M_PI / 180.0;
                double lat2_rad = target_waypoint_.latitude_deg * M_PI / 180.0;
                double lon2_rad = target_waypoint_.longitude_deg * M_PI / 180.0;
                
                double dlon = lon2_rad - lon1_rad;
                double dlat = lat2_rad - lat1_rad;
                double a = sin(dlat/2.0) * sin(dlat/2.0) + cos(lat1_rad) * cos(lat2_rad) * sin(dlon/2.0) * sin(dlon/2.0);
                double c = 2.0 * atan2(sqrt(a), sqrt(1.0-a));
                double distance_m = FlightConstants::EARTH_RADIUS_M * c;
                
                double alt_diff_m = std::abs(current_pos.relative_altitude_m - target_waypoint_.relative_altitude_m);
                
                // Yaw check: current_yaw_deg_ is not being updated from telemetry in this example.
                // float current_yaw_actual_deg = current_yaw_deg_.load(); // Placeholder
                // double yaw_diff_deg = std::abs(current_yaw_actual_deg - target_waypoint_.yaw_deg);
                // if (yaw_diff_deg > 180.0) yaw_diff_deg = 360.0 - yaw_diff_deg;
                
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                                "WP %zu Progress: Dist=%.2fm (thresh=%.1f), Alt diff=%.2fm (thresh=%.1f)",
                                target_idx_at_start, distance_m, waypoint_threshold_m_, alt_diff_m, ALTITUDE_THRESHOLD_M);
                
                if (distance_m < waypoint_threshold_m_ && alt_diff_m < ALTITUDE_THRESHOLD_M) {
                    RCLCPP_INFO(this->get_logger(), "Successfully reached waypoint %zu: %s (Distance: %.2fm, Alt diff: %.2fm)",
                            target_idx_at_start, target_waypoint_.description.c_str(), distance_m, alt_diff_m);
                    
                    currently_navigating_to_waypoint_.store(false);
                    current_waypoint_index_++; // Increment to next waypoint index
                    current_mission_state_.store(MissionState::WAYPOINT_REACHED);
                    return; // Exit monitor thread
                }
            } else {
                 RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Monitor WP %zu: Position data not valid yet.", target_idx_at_start);
            }
            
            attempts++;
            if (attempts >= MAX_ATTEMPTS) {
                RCLCPP_WARN(this->get_logger(), "Timeout reaching waypoint %zu: %s after %d attempts. Assuming reached or stuck.",
                        target_idx_at_start, target_waypoint_.description.c_str(), attempts);
                currently_navigating_to_waypoint_.store(false);
                current_waypoint_index_++; // Move to next waypoint index to avoid getting stuck
                current_mission_state_.store(MissionState::WAYPOINT_REACHED); // Treat as reached to proceed
                return; // Exit monitor thread
            }
            
            sleep_for(milliseconds(LOOP_SLEEP_MS));
        }
        RCLCPP_INFO(this->get_logger(), "Monitor thread for WP %zu exiting. rclcpp::ok(): %s, navigating: %s",
                    target_idx_at_start,
                    rclcpp::ok() ? "true" : "false",
                    currently_navigating_to_waypoint_.load() ? "true" : "false");
        currently_navigating_to_waypoint_.store(false); // Ensure it's false on exit
    }

    bool wait_for_health_ok() {
        RCLCPP_INFO(this->get_logger(), "Waiting for vehicle to be ready to arm (health checks)...");
        rclcpp::Rate r(1.0); 
        while (rclcpp::ok() && current_mission_state_.load() != MissionState::ERROR) { // Added state check
            if (telemetry_ && telemetry_->health_all_ok()) {
                RCLCPP_INFO(this->get_logger(), "Vehicle health is OK.");
                return true;
            }
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Vehicle not healthy yet...");
            r.sleep();
        }
        RCLCPP_WARN(this->get_logger(), "Exited wait_for_health_ok loop. rclcpp::ok(): %s, state: %d", 
            rclcpp::ok() ? "true":"false", static_cast<int>(current_mission_state_.load()));
        return false;
    }

    bool wait_for_home_position() {
        RCLCPP_INFO(this->get_logger(), "Waiting for home position to be set (valid GPS fix)...");
        rclcpp::Rate r(1.0); 
        while(rclcpp::ok() && current_mission_state_.load() != MissionState::ERROR && !has_home_position_.load()) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Home position not available yet...");
            r.sleep();
        }
        if (has_home_position_.load()) {
            RCLCPP_INFO(this->get_logger(), "Home position acquired.");
            return true;
        }
        RCLCPP_WARN(this->get_logger(), "Exited wait_for_home_position loop. rclcpp::ok(): %s, state: %d, home_set: %s", 
            rclcpp::ok() ? "true":"false", static_cast<int>(current_mission_state_.load()), has_home_position_.load() ? "true":"false");
        return false;
    }

    void targetWaypointCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received new target waypoint request: Pos(%.2f, %.2f, %.2f) from frame '%s'",
                    msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, msg->header.frame_id.c_str());

        MissionState current_state = current_mission_state_.load();
        // Allow dynamic waypoints if flying, reached, or even just after takeoff before first goto
        if (current_state != MissionState::FLYING_WAYPOINT &&
            current_state != MissionState::WAYPOINT_REACHED &&
            current_state != MissionState::TAKING_OFF) { // TAKING_OFF might be too early if first WP not sent
             RCLCPP_WARN(this->get_logger(), "Not in an appropriate state (%d) to accept new dynamic waypoints. Ignoring.", static_cast<int>(current_state));
             return;
        }
        if (!has_home_position_.load()) {
            RCLCPP_WARN(this->get_logger(), "Home position not set yet. Cannot process dynamic waypoint. Ignoring.");
            return;
        }

        double new_lat, new_lon;
        float new_rel_alt = static_cast<float>(msg->pose.position.z);
        // Assuming msg->pose.position.x is North offset, y is East offset from home
        offset_lat_lon(home_latitude_deg_, home_longitude_deg_, msg->pose.position.x, msg->pose.position.y, new_lat, new_lon);

        // Convert quaternion to yaw
        double qx = msg->pose.orientation.x;
        double qy = msg->pose.orientation.y;
        double qz = msg->pose.orientation.z;
        double qw = msg->pose.orientation.w;
        // yaw (z-axis rotation)
        double siny_cosp = 2.0 * (qw * qz + qx * qy);
        double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
        float new_yaw_deg = static_cast<float>(atan2(siny_cosp, cosy_cosp) * 180.0 / M_PI);

        RCLCPP_INFO(this->get_logger(), "Calculated new dynamic target (LLA): Lat=%.6f, Lon=%.6f, AltRel=%.1fm, Yaw=%.1fdeg",
                    new_lat, new_lon, new_rel_alt, new_yaw_deg);

        // Logic to decide how to use this new waypoint:
        // Option 1: Overwrite the *next* scheduled waypoint.
        // Option 2: Insert it as the immediate next, shifting others.
        // Option 3: Clear existing plan and go here, then perhaps land or await new plan. (More complex)

        // Current implementation (Option 1 - Overwrite):
        size_t wp_idx_to_update = current_waypoint_index_.load();
        if (wp_idx_to_update < waypoints_.size()) {
            RCLCPP_INFO(this->get_logger(), "Updating waypoint %zu with new dynamically received target.", wp_idx_to_update);
            waypoints_[wp_idx_to_update] = MissionWaypoint(new_lat, new_lon, new_rel_alt, new_yaw_deg, "DYN_WP_" + std::to_string(wp_idx_to_update));
            
            // If currently flying to this waypoint, or just reached it and about to decide next,
            // we might want to re-issue the goto command.
            if ( (current_state == MissionState::FLYING_WAYPOINT && currently_navigating_to_waypoint_.load()) ||
                 current_state == MissionState::WAYPOINT_REACHED || current_state == MissionState::TAKING_OFF) {
                RCLCPP_INFO(this->get_logger(), "Re-issuing goto command for updated waypoint %zu.", wp_idx_to_update);
                // Stop current navigation monitor if active, then restart with new target.
                currently_navigating_to_waypoint_.store(false); // Signal monitor to stop
                if(position_monitor_thread_.joinable()){
                    position_monitor_thread_.join(); // Wait for it to stop
                }
                // current_waypoint_index_ is already pointing to the one we updated.
                current_mission_state_.store(MissionState::FLYING_WAYPOINT); // Ensure correct state
                goto_next_waypoint(); // This will use the updated waypoint_ and restart monitor
            } else {
                 RCLCPP_INFO(this->get_logger(), "Waypoint %zu updated. It will be targeted on next FSM decision.", wp_idx_to_update);
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Cannot update waypoint: index %zu is out of bounds (waypoints size: %zu). Consider adding as new WP.",
                        wp_idx_to_update, waypoints_.size());
            // Alternative: Add as a new waypoint at the end or insert
            // waypoints_.insert(waypoints_.begin() + wp_idx_to_update, MissionWaypoint(...));
        }
    }

    // Member Variables
    std::string connection_url_;
    // REMOVED: double takeoff_altitude_m_; 
    double square_side_length_m_;
    double altitude_layer1_m_;
    double altitude_layer2_m_;
    double waypoint_threshold_m_;

    std::unique_ptr<Mavsdk> mavsdk_;
    std::shared_ptr<System> system_;
    std::shared_ptr<Telemetry> telemetry_;
    std::shared_ptr<Action> action_;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr altitude_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_waypoint_subscriber_;
    
    std::thread mission_thread_;

    std::mutex position_mutex_; // Protects last_known_position_ and position_valid_
    Telemetry::Position last_known_position_;
    bool position_valid_ = false; // Is last_known_position_ up-to-date?
    
    double home_latitude_deg_ = 0.0; // Set once on first valid GPS
    double home_longitude_deg_ = 0.0;
    float home_absolute_altitude_m_ = 0.0f;
    std::atomic<bool> has_home_position_{false};

    std::atomic<MissionState> current_mission_state_;
    std::vector<MissionWaypoint> waypoints_;
    std::atomic<size_t> current_waypoint_index_;
    std::atomic<bool> mission_paused_;

    // For waypoint navigation monitoring
    std::atomic<bool> currently_navigating_to_waypoint_{false};
    MissionWaypoint target_waypoint_; // Stores the current waypoint being flown to by monitor_waypoint_progress
    std::thread position_monitor_thread_;
    // std::atomic<float> current_yaw_deg_{0.0f}; // If actual yaw tracking from telemetry is implemented
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<AutonomousFlightNode> node = nullptr;
    try {
        node = std::make_shared<AutonomousFlightNode>();
        if (rclcpp::ok()) { 
            RCLCPP_INFO(node->get_logger(), "Spinning AutonomousFlightNode...");
            rclcpp::spin(node); // Blocks until node is shutdown
        }
    } catch (const std::exception & e) {
        // 로거를 안전하게 가져오기
        rclcpp::Logger logger = node ? node->get_logger() : rclcpp::get_logger("autonomous_flight_main_exception");
        RCLCPP_FATAL(logger, "Exception in main: %s. Shutting down.", e.what());

        if (rclcpp::ok()) {
            // 노드가 존재하고, 해당 노드의 컨텍스트를 가져올 수 있다면 그 컨텍스트를 종료
            // 그렇지 않다면 기본 컨텍스트를 종료
            if (node && node->get_node_base_interface()) {
                rclcpp::shutdown(node->get_node_base_interface()->get_context());
            } else {
                rclcpp::shutdown(); // 기본 컨텍스트 종료
            }
        }
        return 1;
    }
    
    // rclcpp::shutdown() is called from within run_mission_fsm or due to spin exit.
    // If node was created, its destructor will handle cleanup.
    RCLCPP_INFO(rclcpp::get_logger("main"), "ROS 2 spin completed or shutdown called. Exiting main.");
    return 0;
}