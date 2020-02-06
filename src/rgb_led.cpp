/**
 * \file
 * \brief   Plugin for MAVROS to control RGB LED
 * \author  Andrey Stepanov
 * \version 0.2.0
 * \copyright
 * MIT License \n
 * Copyright (c) 2020 Andrey Stepanov \n
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions: \n
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software. \n
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <mavros/mavros_plugin.h>
#include <mavros_rgbled/LedControlRaw.h>
#include <mavros_rgbled/LedControl.h>
#include <mavros_rgbled/BlinkSequence.h>
#include <condition_variable>

namespace mavros {

using Subscriptions = plugin::PluginBase::Subscriptions;

namespace rgbled_plugin {

/**
 * \brief   RGB Led plugin
 * \details Send command to RGB LED via LED_CONTROL message
 */
class RGBLEDPlugin : public plugin::PluginBase {
    public:
        RGBLEDPlugin();

        void initialize(UAS& uas_);

        Subscriptions get_subscriptions();

    private:
        ros::NodeHandle plugin_nh;

        ros::Subscriber led_control_raw_sub;

        void led_control_raw_cb(const mavros_rgbled::LedControlRawConstPtr& msg);

        ros::Subscriber led_control_sub;

        void led_control_cb(const mavros_rgbled::LedControlConstPtr& msg);

        ros::Subscriber blink_sequence_sub;

        void blink_sequence_cb(const mavros_rgbled::BlinkSequenceConstPtr& msg);

        void set_rgb(const mavros_rgbled::LedState& led_state);

        void handle_param_value(const mavlink::mavlink_message_t* const msg, mavlink::common::msg::PARAM_VALUE& pmsg);
        static const std::string led_override_param_id;
        static const uint8_t led_override_param_type;
        bool set_led_override_param(const int value);
        int led_override_value;
        std::mutex led_override_value_mutex;
        std::condition_variable led_override_waiter;
        bool get_led_override_param();
};

const std::string RGBLEDPlugin::led_override_param_id("NTF_LED_OVERRIDE");
const uint8_t RGBLEDPlugin::led_override_param_type = utils::enum_value(mavlink::common::MAV_PARAM_TYPE::INT32);
#define PARAM_TIMEOUT_S 5

RGBLEDPlugin::RGBLEDPlugin():
        PluginBase(),
        plugin_nh("~rgb_led")
{}

void RGBLEDPlugin::initialize(UAS& uas_) {
    PluginBase::initialize(uas_);

    led_control_raw_sub = plugin_nh.subscribe("led_control_raw", 10, &RGBLEDPlugin::led_control_raw_cb, this);
    led_control_sub = plugin_nh.subscribe("led_control", 10, &RGBLEDPlugin::led_control_cb, this);
    blink_sequence_sub = plugin_nh.subscribe("blink_sequence", 1, &RGBLEDPlugin::blink_sequence_cb, this);
}

Subscriptions RGBLEDPlugin::get_subscriptions() {
    return {
        make_handler(&RGBLEDPlugin::handle_param_value),
    };
}

void RGBLEDPlugin::led_control_raw_cb(const mavros_rgbled::LedControlRawConstPtr& msg) {
    mavros_rgbled::LedControlRaw::_custom_bytes_type::size_type custom_len = msg->custom_bytes.size();
    if ((custom_len < 3) or (custom_len > 4)) {
        ROS_ERROR("Wrong LED pattern size (%lu)", custom_len);
        return;
    }

    mavlink::ardupilotmega::msg::LED_CONTROL lc_msg;
    lc_msg.target_system = msg->target_system;
    lc_msg.target_component = msg->target_component;
    lc_msg.instance = msg->instance;
    lc_msg.pattern = msg->pattern;
    lc_msg.custom_len = msg->custom_bytes.size();
    std::copy(
        msg->custom_bytes.begin(),
        msg->custom_bytes.end(),
        lc_msg.custom_bytes.begin()
    );
    UAS_FCU(m_uas)->send_message_ignore_drop(lc_msg);
}

void RGBLEDPlugin::led_control_cb(const mavros_rgbled::LedControlConstPtr& msg) {
    mavlink::ardupilotmega::msg::LED_CONTROL lc_msg;
    lc_msg.target_system = m_uas->get_tgt_system();
    lc_msg.target_component = m_uas->get_tgt_component();
    lc_msg.instance = msg->instance;
    lc_msg.pattern = utils::enum_value(mavlink::ardupilotmega::LED_CONTROL_PATTERN::CUSTOM);
    lc_msg.custom_len = 4;
    lc_msg.custom_bytes[0] = msg->red;
    lc_msg.custom_bytes[1] = msg->green;
    lc_msg.custom_bytes[2] = msg->blue;
    lc_msg.custom_bytes[3] = msg->rate_hz;
    UAS_FCU(m_uas)->send_message_ignore_drop(lc_msg);
}

void RGBLEDPlugin::set_rgb(const mavros_rgbled::LedState& led_state) {
    mavlink::ardupilotmega::msg::LED_CONTROL lc_msg;
    lc_msg.target_system = m_uas->get_tgt_system();
    lc_msg.target_component = m_uas->get_tgt_component();
    lc_msg.instance = led_state.instance;
    lc_msg.pattern = utils::enum_value(mavlink::ardupilotmega::LED_CONTROL_PATTERN::CUSTOM);
    lc_msg.custom_len = 3;
    lc_msg.custom_bytes[0] = led_state.red;
    lc_msg.custom_bytes[1] = led_state.green;
    lc_msg.custom_bytes[2] = led_state.blue;
    UAS_FCU(m_uas)->send_message_ignore_drop(lc_msg);
    led_state.duration.sleep();
}

void RGBLEDPlugin::blink_sequence_cb(const mavros_rgbled::BlinkSequenceConstPtr& msg) {
    bool success = get_led_override_param();
    if (not success) {
        ROS_ERROR("Can't blink sequence! Can't get previos value of %s parameter", led_override_param_id.c_str());
        return;
    }
    std::unique_lock<std::mutex> lock(led_override_value_mutex);
    int previous_led_override_value = led_override_value;
    lock.unlock();

    const int mavlink_led = 1;
    if (previous_led_override_value != mavlink_led) {
        success = set_led_override_param(mavlink_led);
        if (not success) {
            ROS_ERROR("Can't blink sequence! Can't set new value of %s parameter", led_override_param_id.c_str());
            return;
        }
    }

    ros::spinOnce();

    std::for_each(
        msg->sequence.begin(),
        msg->sequence.end(),
        boost::bind(&RGBLEDPlugin::set_rgb, this, _1)
    );

    if (previous_led_override_value != mavlink_led) {
        success = set_led_override_param(previous_led_override_value);
        if (not success) {
            ROS_FATAL("Error while blinking! Can't restore previous value of %s parameter", led_override_param_id.c_str());
        }
    }
}

void RGBLEDPlugin::handle_param_value(const mavlink::mavlink_message_t* const msg, mavlink::common::msg::PARAM_VALUE& pmsg) {
    const std::string param_id = mavlink::to_string(pmsg.param_id);
    if (param_id != led_override_param_id)
        return;

    {
        std::lock_guard<std::mutex> lock(led_override_value_mutex);
        led_override_value = static_cast<int>(pmsg.param_value);
    }

    led_override_waiter.notify_all();
}

bool RGBLEDPlugin::set_led_override_param(const int value) {
    mavlink::common::msg::PARAM_SET msg;
    mavlink::set_string(msg.param_id, led_override_param_id);
    msg.param_type = led_override_param_type;
    if (m_uas->is_ardupilotmega()) {
        msg.param_value = static_cast<int32_t>(value);
    } else {
        mavlink::mavlink_param_union_t uv;
        uv.param_int32 = static_cast<int32_t>(value);
        msg.param_value = uv.param_float;
    }
    m_uas->msg_set_target(msg);

    UAS_FCU(m_uas)->send_message_ignore_drop(msg);

    std::unique_lock<std::mutex> lock(led_override_value_mutex);

    return (
        (led_override_waiter.wait_for(lock, std::chrono::seconds(PARAM_TIMEOUT_S)) == std::cv_status::no_timeout)
        and
        (std::abs(led_override_value - value) < 0.1)
    );
}

bool RGBLEDPlugin::get_led_override_param() {
    mavlink::common::msg::PARAM_REQUEST_READ msg;
    mavlink::set_string(msg.param_id, led_override_param_id);
    msg.param_index = -1;
    m_uas->msg_set_target(msg);
    UAS_FCU(m_uas)->send_message_ignore_drop(msg);

    std::unique_lock<std::mutex> lock(led_override_value_mutex);
    return (led_override_waiter.wait_for(lock, std::chrono::seconds(PARAM_TIMEOUT_S)) == std::cv_status::no_timeout);
}

} // namespace rgbled_plugin

} // namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::rgbled_plugin::RGBLEDPlugin, mavros::plugin::PluginBase)
