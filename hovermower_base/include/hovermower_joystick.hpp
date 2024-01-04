#ifndef _HOVERMOWER_JOYSTICK_H
#define _HOVERMOWER_JOYSTICK_H

#include <rclcpp/rclcpp.hpp>
#include <string.h>
#include <stdio.h>
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "rosmower_msgs/srv/press_switch.hpp"
#include "rosmower_msgs/srv/set_switch.hpp"
#include "rosmower_msgs/srv/set_mow_motor.hpp"
#include "rosmower_msgs/msg/switches.hpp"
#include "rosmower_msgs/msg/mow_motor.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class HoverMower_joystick : public rclcpp::Node
{
public:
    HoverMower_joystick(std::string name);
    ~HoverMower_joystick();

    void joyCallback(const std::shared_ptr<sensor_msgs::msg::Joy> msg);
    void mowCallback(const std::shared_ptr<rosmower_msgs::msg::MowMotor> msg);
    void switchesCallback(const std::shared_ptr<rosmower_msgs::msg::Switches> msg);

/* valid values for DS4 joy message
    enum buttons : int
    {
        square = 0,
        triangle = 1,
        star = 2,
        cross = 3,
        LB = 4,
        LT = 5,
        RB = 6,
        RT = 7,
        share = 8,
        options = 9,
        ten = 10,
        touch = 11,
        left_click = 12,
        right_click = 13,
        left = 14,
        up = 15,
        right = 16,
        down = 17
    }; */

    enum buttons : int
    {
        square = 2,
        triangle = 3,
        star = 1,
        cross = 0,
        LB = 9,
        RB = 10,
        share = 4,
        options = 6,
        PS = 5,
        touch = 15,
        left_click = 7,
        right_click = 8,
        left = 13,
        up = 11,
        right = 14,
        down = 12
    };

private:
    // Publishers
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_eStop;

    // Service Clients
    rclcpp::Client<rosmower_msgs::srv::PressSwitch>::SharedPtr _srv_pressSwitch;// = this->create_client<rosmower_msgs::srv::PressSwitch>("hovermower/pressSwitch");
    rclcpp::Client<rosmower_msgs::srv::SetSwitch>::SharedPtr _srv_setSwitch;// = this->create_client<rosmower_msgs::srv::SetSwitch>("hovermower/setSwitch");
    rclcpp::Client<rosmower_msgs::srv::SetMowMotor>::SharedPtr _srv_mow;// = this->create_client<rosmower_msgs::srv::SetMowMotor>("hovermower/setMowMotorSpeed");

    // Subscriber
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _sub_joy;
    rclcpp::Subscription<rosmower_msgs::msg::MowMotor>::SharedPtr _sub_mow;
    rclcpp::Subscription<rosmower_msgs::msg::Switches>::SharedPtr _sub_switches;

    uint8_t _switch1;
    uint8_t _switch2;
    int _mow_speed;

    // keep last button state to not toggle too often (debounce)
    int last_button_options = 0;
    int last_button_share = 0;
    int last_button_ps = 0;
    int last_button_circle = 0;
    int last_button_square = 0;
    int last_button_triangle = 0;
    int last_button_cross = 0;

    bool _e_stop;
    bool _cross_pressed;
};

#endif
