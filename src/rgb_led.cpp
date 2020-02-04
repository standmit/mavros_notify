/**
 * \file
 * \brief   Plugin for MAVROS to control RGB LED
 * \author  Andrey Stepanov
 * \version 0.1.0
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
};

RGBLEDPlugin::RGBLEDPlugin():
        PluginBase(),
        plugin_nh("~rgb_led")
{}

void RGBLEDPlugin::initialize(UAS& uas_) {
    PluginBase::initialize(uas_);

    led_control_raw_sub = plugin_nh.subscribe("led_control_raw", 10, &RGBLEDPlugin::led_control_raw_cb, this);
    led_control_sub = plugin_nh.subscribe("led_control", 10, &RGBLEDPlugin::led_control_cb, this);
}

Subscriptions RGBLEDPlugin::get_subscriptions() {
    return {};
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

} // namespace rgbled_plugin

} // namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::rgbled_plugin::RGBLEDPlugin, mavros::plugin::PluginBase)
