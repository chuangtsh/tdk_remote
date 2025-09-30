#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

class MissionNode : public rclcpp::Node {
public:
    MissionNode() : Node("mission2") {
        // Create stage_step subscriber
        stage_step_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/current_stage", 10, 
            std::bind(&MissionNode::stage_step_callback, this, _1));
        
        // Create subscribers for status topics (reverse of original publishers)
        sub_gripper_status_ = this->create_subscription<std_msgs::msg::Bool>(
            "/gripper", 10,
            std::bind(&MissionNode::callback_gripper_status, this, _1));
        sub_basket_status_ = this->create_subscription<std_msgs::msg::Bool>(
            "/basket", 10,
            std::bind(&MissionNode::callback_basket_status, this, _1));
        sub_touch_ = this->create_subscription<std_msgs::msg::Bool>(
            "/touch", 10,
            std::bind(&MissionNode::callback_touch, this, _1));
        sub_current_y_ = this->create_subscription<std_msgs::msg::Float64>(
            "/current_y", 10,
            std::bind(&MissionNode::callback_current_y, this, _1));
        sub_current_theta_ = this->create_subscription<std_msgs::msg::Float64>(
            "/current_theta", 10,
            std::bind(&MissionNode::callback_current_theta, this, _1));
        // sub_elevator_status_ = this->create_subscription<std_msgs::msg::Float64>(
        //     "/elevator", 10,
        //     std::bind(&MissionNode::callback_elevator_status, this, _1));
        
        // Create publishers for command topics (reverse of original subscribers)
        pub_cmd_gripper_ = this->create_publisher<std_msgs::msg::Bool>("/cmd_gripperOpen", 10);
        pub_cmd_elevator_ = this->create_publisher<std_msgs::msg::Int32>("/cmd_elevator", 10);
        pub_cmd_basket_door_ = this->create_publisher<std_msgs::msg::Bool>("/cmd_basketDoor", 10);
        pub_cmd_mission_finish_ = this->create_publisher<std_msgs::msg::Bool>("/cmd_missionFinish", 10);
        pub_cmd_servoturn_ = this->create_publisher<std_msgs::msg::Int32>("/cmd_servoturn", 10);
        pub_cmd_forward_ = this->create_publisher<std_msgs::msg::Int32>("/cmd_forward", 10);
        
        // Create timer callback (executes every 10ms)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&MissionNode::timer_callback, this));
     
        RCLCPP_INFO(this->get_logger(), "Mission2 node started");
    }

private:
    void stage_step_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        // RCLCPP_INFO(this->get_logger(), "Received stage_step: %d", msg->data); 
        this->stage_step = msg->data;
    }

    void callback_gripper_status(const std_msgs::msg::Bool::SharedPtr msg) {
        // RCLCPP_INFO(this->get_logger(), "Gripper status received: %s", msg->data ? "finished" : "in progress");
        gripper_status = msg->data;
    }

    void callback_touch(const std_msgs::msg::Bool::SharedPtr msg) {
        // RCLCPP_INFO(this->get_logger(), "Touch status received: %s", msg->data ? "finished" : "in progress");
        touch = msg->data;
    }

    void callback_current_y(const std_msgs::msg::Float64::SharedPtr msg) {
        // RCLCPP_INFO(this->get_logger(), "Current Y received: %f", msg->data);
        current_y = (int)msg->data;  // Convert float to int
    }

    void callback_current_theta(const std_msgs::msg::Float64::SharedPtr msg) {
        // RCLCPP_INFO(this->get_logger(), "Current Theta received: %f", msg->data);
        current_theta = (int)msg->data;  // Convert float to int
    }

    void callback_basket_status(const std_msgs::msg::Bool::SharedPtr msg) {
        // RCLCPP_INFO(this->get_logger(), "Basket status received: %s", msg->data ? "finished" : "in progress");
        basket_status = msg->data;
    }

    void callback_elevator_status(const std_msgs::msg::Float64::SharedPtr msg) {
        // RCLCPP_INFO(this->get_logger(), "Elevator height status received: %f", msg->data);
        elevator_status = msg->data;
    }

    void timer_callback() {
        int elevator_ctrl = 0; // 0 stop, 1 up, -1 down, 2: 650
        int servoturn_ctrl = 0; // servo turn command 1 is forward, 2 is backward
        mission_complete = false;
        // gripper_ctrl: 0 is back, 1 is forward


        if ( this->stage_step == 21 ) {
            // cascade down till get to the table
            gripper_ctrl = 0;
            if ( touched_for_menu == 0 ) {
                elevator_ctrl = -1;
            }
            else {
                elevator_ctrl = 0;
            }
            if ( touch ) {
                touched_for_menu = 1;
                // Start 1-second one-shot timer when touched_for_menu becomes 1
                if (!timer_1sec_started) {
                    timer_1sec_ = this->create_wall_timer(
                        std::chrono::seconds(1),
                        std::bind(&MissionNode::timer_1sec_callback, this));
                    timer_1sec_started = true;
                    // RCLCPP_INFO(this->get_logger(), "1-second one-shot timer started after touch");
                }
            }

        }
        else if ( this->stage_step == 22 ) {
            // after go to the correct cup
            gripper_ctrl = 1;
            // if ( gripper_status == 1 ) {
                // Start time measurement when gripper_status becomes 1
                if (!gripper_wait_started) {
                    gripper_wait_start_time = std::chrono::steady_clock::now();
                    gripper_wait_started = true;
                }
                
                // Check if 1 second has elapsed since gripper_status became 1
                auto current_time = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - gripper_wait_start_time);
                
                if (elapsed.count() >= 1) {
                    // 1 second has passed, now move elevator
                    elevator_ctrl = 2;
                }
                if ( elapsed.count() >= 4 ) {
                    mission_complete = true;
                }
            // }
        }
        else if ( this->stage_step == 23 ) {
            // to see the sticker, make cascade upper
            elevator_ctrl = 2;
            
            // Start time measurement when stage 23 begins
            if (!stage23_timer_started) {
                stage23_start_time = std::chrono::steady_clock::now();
                stage23_timer_started = true;
            }
            
            // Check if 5 seconds have elapsed
            auto current_time = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - stage23_start_time);
            
            if (elapsed.count() >= 5) {
                // 5 seconds have passed, mission complete
                mission_complete = true;
            }

        }
        else if ( this->stage_step == 24 ) {
            // after go to the modified pose of the little table

            forward_ctrl = 2;

            // Start stage 24 timer once when we first enter this stage
            if (!stage24_timer_started) {
                stage24_start_time = std::chrono::steady_clock::now();
                stage24_timer_started = true;
            }

            auto current_time = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - stage24_start_time);

            if ( elapsed.count() > 8 ) {

                if ( touched_for_sticker == 0 ) {
                    elevator_ctrl = -1;
                }
                else {
                    elevator_ctrl = 0;
                    gripper_ctrl = 0;
                }
                if ( touch == 1 ) {
                    touched_for_sticker = 1;
                }
                if ( gripper_status == 0 ) {
                    mission_complete = true;
                }
            }

        }
        else if ( this->stage_step == 31 ) {
            basket_ctrl = 0;
        }
        else if ( this->stage_step == 32 ) {
            basket_ctrl = 1;
        }
        
        // Publish the commands
        auto gripper_msg = std_msgs::msg::Bool();
        gripper_msg.data = gripper_ctrl;
        pub_cmd_gripper_->publish(gripper_msg);
        
        auto elevator_msg = std_msgs::msg::Int32();
        elevator_msg.data = elevator_ctrl;
        pub_cmd_elevator_->publish(elevator_msg);
        
        auto basket_msg = std_msgs::msg::Bool();
        basket_msg.data = basket_ctrl;
        pub_cmd_basket_door_->publish(basket_msg);

        
        auto mission_finish_msg = std_msgs::msg::Bool();
        mission_finish_msg.data = mission_complete; 
        pub_cmd_mission_finish_->publish(mission_finish_msg);
        
        auto servoturn_msg = std_msgs::msg::Int32();
        servoturn_msg.data = servoturn_ctrl;
        pub_cmd_servoturn_->publish(servoturn_msg);
        
        auto forward_msg = std_msgs::msg::Int32();
        forward_msg.data = forward_ctrl;
        pub_cmd_forward_->publish(forward_msg);
        
        // RCLCPP_INFO(this->get_logger(), "Published commands - Gripper: %s, Elevator: %d, Basket: %s, Mission Complete: %s", 
        //             gripper_ctrl ? "open" : "closed", elevator_ctrl, basket_ctrl ? "open" : "closed", mission_complete ? "true" : "false");
    }

    void timer_1sec_callback() {
        // Your one-time code here that executes after the timer duration
        // RCLCPP_INFO(this->get_logger(), "one-shot timer executed!");
        
        // Add your specific logic that should run once, 1 second after touch
        mission_complete = true; 
        timer_1sec_started = false;
        
        auto mission_finish_msg = std_msgs::msg::Bool();
        mission_finish_msg.data = mission_complete; 
        pub_cmd_mission_finish_->publish(mission_finish_msg);

        // Cancel the timer after execution to prevent it from running again
        if (timer_1sec_) {
            timer_1sec_->cancel();
            timer_1sec_.reset();
        }
    }
    
    // Subscribers (receiving status from hardware)
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr stage_step_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_gripper_status_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_basket_status_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_touch_; // whether the cascade has arrived to the pose
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_current_y_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_current_theta_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_elevator_status_;
    
    // Publishers (sending commands to hardware)
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_cmd_gripper_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_cmd_elevator_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_cmd_basket_door_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_cmd_mission_finish_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_cmd_servoturn_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_cmd_forward_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_1sec_;
    bool timer_1sec_started = false;

    // Time tracking for stage 24
    std::chrono::steady_clock::time_point forward_start_time;
    bool forward_command_sent = false;

    // Time tracking for stage 22 - wait 1 sec after gripper_status == 1
    std::chrono::steady_clock::time_point gripper_wait_start_time;
    bool gripper_wait_started = false;

    // Time tracking for stage 23 - wait 5 sec
    std::chrono::steady_clock::time_point stage23_start_time;
    bool stage23_timer_started = false;

    // Time tracking for stage 24 - fallback timeout
    std::chrono::steady_clock::time_point stage24_start_time;
    bool stage24_timer_started = false;


    int stage_step = 0;

    bool gripper_ctrl = 0;
    bool mission_complete = false;
    int forward_ctrl = 1; // big table 2, small table 3, the most forward is 1

    bool gripper_status = 0;
    bool basket_ctrl = 1; // closed is 0?
    bool basket_status = 0;
    bool touch = 0;
    int current_y = 0;
    int current_theta = 0;
    bool touched_for_menu = 0;
    bool touched_for_sticker = 0;
    float elevator_status = 0;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MissionNode>());
    rclcpp::shutdown();
    return 0;
}
