#include "hovermower_joystick.hpp"

HoverMower_joystick::HoverMower_joystick(std::string name) : Node(name)
{

    // Register  publisher
    pub_eStop = create_publisher<std_msgs::msg::Bool>("/e_stop", 3);

    // Register subscriber
    _sub_mow = create_subscription<rosmower_msgs::msg::MowMotor>("hovermower/sensors/MowMotor", 1000, std::bind(&HoverMower_joystick::mowCallback, this, std::placeholders::_1));
    _sub_switches = create_subscription<rosmower_msgs::msg::Switches>("hovermower/switches", 10, std::bind(&HoverMower_joystick::switchesCallback, this, std::placeholders::_1));
    _sub_joy = create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&HoverMower_joystick::joyCallback, this, std::placeholders::_1));

    // register service clients
    _srv_pressSwitch = create_client<rosmower_msgs::srv::PressSwitch>("hovermower/pressSwitch");
    _srv_setSwitch = this->create_client<rosmower_msgs::srv::SetSwitch>("hovermower/setSwitch");
    _srv_mow = this->create_client<rosmower_msgs::srv::SetMowMotor>("hovermower/setMowMotorSpeed");

    _e_stop = false;

}

HoverMower_joystick::~HoverMower_joystick()
{
}

void HoverMower_joystick::joyCallback(const std::shared_ptr<sensor_msgs::msg::Joy> msg)
{
    // e_stop
    // Ensures not to toggle for each incoming message, when button keeps pressed
    if (msg->buttons[buttons::PS] > 0 && msg->buttons[buttons::PS] != last_button_ps)
    {
        _e_stop = !_e_stop;
        last_button_ps = msg->buttons[buttons::PS];

        std_msgs::msg::Bool msg_estop;
        msg_estop.data = _e_stop;
        pub_eStop->publish(msg_estop);
        // ros::spinOnce();
    }
    // reset if button has been released
    if (msg->buttons[buttons::PS] == 0)
    {
        last_button_ps = 0;
    }

    // enable Hoverboard PCB
    if (msg->buttons[buttons::options] > 0 && msg->buttons[buttons::options] != last_button_options)
    {
        last_button_options = msg->buttons[buttons::options];
        auto srv = std::make_shared<rosmower_msgs::srv::PressSwitch::Request>();
        srv->switch_id = 3;
        _srv_pressSwitch->async_send_request(srv);
    }
    // reset if button has been released
    if (msg->buttons[buttons::options] == 0)
    {
        last_button_options = 0;
    }

    // mow motor speed
    if (msg->buttons[buttons::share] > 0 && msg->buttons[buttons::share] != last_button_share)
    {
        last_button_share = msg->buttons[buttons::share];
        auto srv = std::make_shared<rosmower_msgs::srv::SetMowMotor::Request>();
        if (_mow_speed == 0)
        {
            srv->speed = 1500;
        }
        else
        {
            srv->speed = 0;
        }
        _srv_mow->async_send_request(srv);
    }
    // reset if button has been released
    if (msg->buttons[buttons::share] == 0)
    {
        last_button_share = 0;
    }
}

void HoverMower_joystick::mowCallback(const std::shared_ptr<rosmower_msgs::msg::MowMotor> msg)
{
    _mow_speed = msg->speed;
}

void HoverMower_joystick::switchesCallback(const std::shared_ptr<rosmower_msgs::msg::Switches> msg)
{
    _switch1 = msg->switch1;
    _switch2 = msg->switch2;
}

