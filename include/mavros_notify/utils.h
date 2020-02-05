/**
 * \file
 * \brief   Utilites for MAVROS Notify plugins
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

#ifndef MAVROS_RGBLED_UTILS_H_
#define MAVROS_RGBLED_UTILS_H_

#include <mavros_notify/LedState.h>

namespace mavros_notify {

namespace utils {

LedState make_led_state(
        const uint8_t red,
        const uint8_t green,
        const uint8_t blue,
        const double duration,
        const LedState::_instance_type instance = LedState::INSTANCE_ALL
);

}

}

#endif
