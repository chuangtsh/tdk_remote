#include <memory>
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
        // sub_elevator_status_ = this->create_subscription<std_msgs::msg::Float64>(
        //     "/elevator", 10,
        //     std::bind(&MissionNode::callback_elevator_status, this, _1));
        
        // Create publishers for command topics (reverse of original subscribers)
        pub_cmd_gripper_ = this->create_publisher<std_msgs::msg::Bool>("/cmd_gripperOpen", 10);
        pub_cmd_elevator_ = this->create_publisher<std_msgs::msg::Int32>("/cmd_elevator", 10);
        pub_cmd_basket_door_ = this->create_publisher<std_msgs::msg::Bool>("/cmd_basketDoor", 10);
        pub_cmd_mission_finish_ = this->create_publisher<std_msgs::msg::Bool>("/cmd_missionFinish", 10);
        pub_cmd_servoturn_ = this->create_publisher<std_msgs::msg::Int32>("/cmd_servoturn", 10);
        
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

    void callback_basket_status(const std_msgs::msg::Bool::SharedPtr msg) {
        // RCLCPP_INFO(this->get_logger(), "Basket status received: %s", msg->data ? "finished" : "in progress");
        basket_status = msg->data;
    }

    void callback_elevator_status(const std_msgs::msg::Float64::SharedPtr msg) {
        // RCLCPP_INFO(this->get_logger(), "Elevator height status received: %f", msg->data);
        elevator_status = msg->data;
    }

    void timer_callback() {
        int elevator_ctrl = 0; // reset is 0?
        bool basket_ctrl = 0; // closed is 0?
        bool mission_complete = 0;
        int servoturn_ctrl = 900; // servo turn command


        if ( this->stage_step == 21 ) {
            // cascade down till get to the table
            // gripper_ctrl = 1;
            // elevator_ctrl = -1;
            // if ( touch == 1 || touched_for_menu == 1) {
            //     touched_for_menu = 1;
            //     elevator_ctrl = 0;
            //     mission_complete = 1;
            // }
            gripper_ctrl = 1;
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
                    RCLCPP_INFO(this->get_logger(), "1-second one-shot timer started after touch");
                }
            }

        }
        else if ( this->stage_step == 22 ) {
            // after go to the correct cup
            if ( gripper_status == 1 ) {
                gripper_ctrl = 0;
            }
            if ( gripper_status == 0 ) {
                mission_complete = 1;
            }
        }
        else if ( this->stage_step == 23 ) {
            // to see the sticker
            mission_complete = 1;
        }
        else if ( this->stage_step == 24 ) {
            // after go to the modified pose of the little table
            elevator_ctrl = -1;
            if ( touch == 1 && touched_for_sticker == 0) {
                touched_for_sticker = 1;
                elevator_ctrl = 0;
                gripper_ctrl = 1;
            }
            if ( touched_for_sticker == 1 && gripper_status == 1 ) {
                mission_complete = 1;
            }
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
        mission_finish_msg.data = mission_complete; // Use the same logic as mission_complete
        pub_cmd_mission_finish_->publish(mission_finish_msg);
        
        auto servoturn_msg = std_msgs::msg::Int32();
        servoturn_msg.data = servoturn_ctrl;
        pub_cmd_servoturn_->publish(servoturn_msg);
        
        // RCLCPP_INFO(this->get_logger(), "Published commands - Gripper: %s, Elevator: %d, Basket: %s, Mission Complete: %s", 
        //             gripper_ctrl ? "open" : "closed", elevator_ctrl, basket_ctrl ? "open" : "closed", mission_complete ? "true" : "false");
    }

    void timer_1sec_callback() {
        // Your one-time code here that executes 1 second after touch
        RCLCPP_INFO(this->get_logger(), "1-second one-shot timer executed!");
        
        // Add your specific logic that should run once, 1 second after touch
        mission_complete = 1; 
        
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
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_elevator_status_;
    
    // Publishers (sending commands to hardware)
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_cmd_gripper_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_cmd_elevator_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_cmd_basket_door_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_cmd_mission_finish_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_cmd_servoturn_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_1sec_;
    bool timer_1sec_started = false;

    int stage_step = 0;

    bool gripper_ctrl = 1;

    bool gripper_status = 0;
    bool basket_status = 0;
    bool touch = 0;
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
