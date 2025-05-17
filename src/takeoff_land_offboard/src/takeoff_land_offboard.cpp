#include <chrono>
#include <cstdint>
#include <memory>
#include <future>
#include <thread>
#include <iostream>
#include <sstream>
#include <functional> // For std::bind with thread
#include <mutex>      // For std::mutex and std::lock_guard

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
// #include <mavsdk/plugins/offboard/offboard.h> // Potentially for more complex offboard

using namespace mavsdk;
using std::chrono::seconds;
using std::chrono::milliseconds;
using std::this_thread::sleep_for;

// ... (action_result_str, telemetry_result_str, connection_result_str 함수들은 그대로 둡니다) ...
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


class AutonomousFlightNode : public rclcpp::Node
{
public:
    AutonomousFlightNode() : Node("autonomous_flight_node")
    {
        // ... (파라미터 선언 및 MAVSDK 초기화 부분은 동일) ...
        this->declare_parameter<std::string>("connection_url", "udpin://0.0.0.0:14540");
        this->get_parameter("connection_url", connection_url_);
        this->declare_parameter<double>("takeoff_altitude", 5.0); // Default 5 meters
        this->get_parameter("takeoff_altitude", takeoff_altitude_m_);
        this->declare_parameter<int>("mission_hold_duration_s", 10); // Default 10 seconds hold
        this->get_parameter("mission_hold_duration_s", mission_hold_duration_s_);


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
                // 포맷 문자열 경고 수정
                RCLCPP_INFO(this->get_logger(), "System discovered (ID: %u).", static_cast<unsigned int>(system->get_system_id()));
                prom.set_value(system);
            } else if (system) {
                 RCLCPP_WARN(this->get_logger(), "System found but not connected yet.");
            } else {
                 RCLCPP_WARN(this->get_logger(), "No system found yet.");
            }
        });

        if (fut.wait_for(seconds(10)) == std::future_status::timeout) { // Increased timeout
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

        const auto set_rate_result = telemetry_->set_rate_position(1.0); // 1 Hz
        if (set_rate_result != Telemetry::Result::Success) {
            RCLCPP_WARN(this->get_logger(), "Setting position rate failed: %s", telemetry_result_str(set_rate_result).c_str());
        }

        altitude_publisher_ = this->create_publisher<std_msgs::msg::Float32>("altitude", 10);
        
        // subscribe_position 콜백에서 last_known_position_ 업데이트
        telemetry_->subscribe_position([this](Telemetry::Position position) {
            auto msg = std_msgs::msg::Float32();
            msg.data = position.relative_altitude_m;
            altitude_publisher_->publish(msg);

            { // Scope for lock_guard
                std::lock_guard<std::mutex> lock(position_mutex_);
                last_known_position_ = position;
                position_valid_ = true;
            }
        });

        RCLCPP_INFO(this->get_logger(), "Node initialized. Starting autonomous mission thread.");
        mission_thread_ = std::thread(&AutonomousFlightNode::run_mission, this);
    }

    ~AutonomousFlightNode() {
        if (mission_thread_.joinable()) {
            mission_thread_.join();
        }
    }

private:
    void run_mission() {
        if (!wait_for_health_ok()) {
            RCLCPP_ERROR(this->get_logger(), "Vehicle health checks failed. Aborting mission.");
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Setting takeoff altitude to %.1f m...", takeoff_altitude_m_);
        Action::Result set_alt_res = action_->set_takeoff_altitude(static_cast<float>(takeoff_altitude_m_));
        if (set_alt_res != Action::Result::Success) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set takeoff altitude: %s. Aborting.", action_result_str(set_alt_res).c_str());
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Takeoff altitude set to %.1f m.", takeoff_altitude_m_);


        RCLCPP_INFO(this->get_logger(), "Arming vehicle...");
        const Action::Result arm_result = action_->arm();
        if (arm_result != Action::Result::Success) {
            RCLCPP_ERROR(this->get_logger(), "Arming failed: %s. Aborting.", action_result_str(arm_result).c_str());
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Vehicle armed.");

        RCLCPP_INFO(this->get_logger(), "Taking off...");
        const Action::Result takeoff_result = action_->takeoff();
        if (takeoff_result != Action::Result::Success) {
            RCLCPP_ERROR(this->get_logger(), "Takeoff failed: %s. Aborting.", action_result_str(takeoff_result).c_str());
            action_->disarm();
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Takeoff initiated. Waiting to reach altitude...");

        // 고도 도달 확인 루프 (멤버 변수 사용)
        while (rclcpp::ok()) {
            Telemetry::Position current_pos;
            bool got_pos_from_member = false;
            { // Scope for lock_guard
                std::lock_guard<std::mutex> lock(position_mutex_);
                if (position_valid_) {
                    current_pos = last_known_position_;
                    got_pos_from_member = true;
                }
            }

            if (got_pos_from_member && current_pos.relative_altitude_m >= takeoff_altitude_m_ * 0.90) { // 90% of target alt
                RCLCPP_INFO(this->get_logger(), "Reached target altitude of approx %.1f m.", current_pos.relative_altitude_m);
                break;
            }
            if (!telemetry_->in_air()) { 
                 RCLCPP_WARN(this->get_logger(), "Vehicle not in air during takeoff sequence. Checking status.");
            }
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting to reach altitude... Current: %.1f m", got_pos_from_member ? current_pos.relative_altitude_m : -1.0f);
            sleep_for(milliseconds(500)); // Check periodically
        }
        if (!rclcpp::ok()) return;


        RCLCPP_INFO(this->get_logger(), "Holding position for %d seconds...", mission_hold_duration_s_);
        sleep_for(seconds(mission_hold_duration_s_));
        if (!rclcpp::ok()) return;

        RCLCPP_INFO(this->get_logger(), "Landing...");
        const Action::Result land_result = action_->land();
        if (land_result != Action::Result::Success) {
            RCLCPP_ERROR(this->get_logger(), "Landing failed: %s.", action_result_str(land_result).c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "Landing initiated. Monitoring until landed.");
        }

        while (rclcpp::ok() && telemetry_->in_air()) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Vehicle is landing...");
            sleep_for(seconds(1));
        }
        if (!rclcpp::ok()) return;

        if (!telemetry_->in_air()){
             RCLCPP_INFO(this->get_logger(), "Vehicle has landed.");
        } else {
             RCLCPP_WARN(this->get_logger(), "Vehicle still reporting in_air after land sequence.");
        }

        RCLCPP_INFO(this->get_logger(), "Mission complete. Disarming...");
        const Action::Result disarm_result = action_->disarm();
         if (disarm_result != Action::Result::Success) {
            RCLCPP_ERROR(this->get_logger(), "Disarming failed: %s.", action_result_str(disarm_result).c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "Vehicle disarmed.");
        }

        RCLCPP_INFO(this->get_logger(), "Autonomous mission finished. Shutting down node.");
        rclcpp::shutdown(); 
    }

    bool wait_for_health_ok() {
        RCLCPP_INFO(this->get_logger(), "Waiting for vehicle to be ready to arm (health checks)...");
        while (rclcpp::ok()) {
            // health_all_ok()는 동기 함수이므로 직접 호출
            if (telemetry_ && telemetry_->health_all_ok()) { // telemetry_ null 체크 추가
                RCLCPP_INFO(this->get_logger(), "Vehicle health is OK.");
                return true;
            }
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Vehicle not healthy yet...");
            sleep_for(seconds(1));
        }
        return false;
    }
    

    std::string connection_url_;
    double takeoff_altitude_m_;
    int mission_hold_duration_s_;

    std::unique_ptr<Mavsdk> mavsdk_;
    std::shared_ptr<System> system_;
    std::shared_ptr<Telemetry> telemetry_;
    std::shared_ptr<Action> action_;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr altitude_publisher_;
    
    std::thread mission_thread_;

    // 멤버 변수: 최신 위치 정보 저장용
    std::mutex position_mutex_;
    Telemetry::Position last_known_position_;
    bool position_valid_ = false;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutonomousFlightNode>();
    if (rclcpp::ok()) { 
        rclcpp::spin(node);
    }
    // rclcpp::shutdown(); // 이미 run_mission 끝에서 호출됨
    return 0;
}

//holytorch