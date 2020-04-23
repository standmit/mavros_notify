#include <ros/ros.h>
#include <mavros_notify/BlinkSequence.h>
#include <mavros_notify/utils.h>

constexpr double dt = 0.1;

mavros_notify::BlinkSequence msg;
ros::Publisher pub;

void blink() {
    pub.publish(msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "blinking");
    ros::NodeHandle nh("~");
    pub = nh.advertise<mavros_notify::BlinkSequence>("blink", 1, false);

    double duration = 0.0;
    msg.sequence.emplace_back(mavros_notify::utils::make_led_state(255, 0,   0,   dt, duration, mavros_notify::LedState::INSTANCE_ALL));
    msg.sequence.emplace_back(mavros_notify::utils::make_led_state(0,   0,   0,   dt, duration, mavros_notify::LedState::INSTANCE_ALL));
    msg.sequence.emplace_back(mavros_notify::utils::make_led_state(255, 0,   0,   dt, duration, mavros_notify::LedState::INSTANCE_ALL));
    msg.sequence.emplace_back(mavros_notify::utils::make_led_state(0,   0,   0,   dt, duration, mavros_notify::LedState::INSTANCE_ALL));
    msg.sequence.emplace_back(mavros_notify::utils::make_led_state(0,   255, 0,   dt, duration, mavros_notify::LedState::INSTANCE_ALL));
    msg.sequence.emplace_back(mavros_notify::utils::make_led_state(0,   0,   0,   dt, duration, mavros_notify::LedState::INSTANCE_ALL));
    msg.sequence.emplace_back(mavros_notify::utils::make_led_state(0,   255, 0,   dt, duration, mavros_notify::LedState::INSTANCE_ALL));
    msg.sequence.emplace_back(mavros_notify::utils::make_led_state(0,   0,   0,   dt, duration, mavros_notify::LedState::INSTANCE_ALL));
    msg.sequence.emplace_back(mavros_notify::utils::make_led_state(0,   0,   255, dt, duration, mavros_notify::LedState::INSTANCE_ALL));
    msg.sequence.emplace_back(mavros_notify::utils::make_led_state(0,   0,   0,   dt, duration, mavros_notify::LedState::INSTANCE_ALL));
    msg.sequence.emplace_back(mavros_notify::utils::make_led_state(0,   0,   255, dt, duration, mavros_notify::LedState::INSTANCE_ALL));
    msg.sequence.emplace_back(mavros_notify::utils::make_led_state(0,   0,   0,   dt, duration, mavros_notify::LedState::INSTANCE_ALL));

    ros::Timer timer = nh.createTimer(ros::Duration(duration), boost::bind(blink), false, true);

    ros::spin();

    return 0;
}

