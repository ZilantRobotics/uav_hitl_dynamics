/* 
 * Copyright (c) 2020-2022 RaccoonLab.
 * 
 * This program is free software: you can redistribute it and/or modify  
 * it under the terms of the GNU General Public License as published by  
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 * 
 * Author: Dmitry Ponomarev <ponomarevda96@gmail.com>
 */

#ifndef SRC_SENSORS_ICE_HPP
#define SRC_SENSORS_ICE_HPP

#include "sensor_base.hpp"

class IceStatusSensor : public BaseSensor{
    public:
        IceStatusSensor(ros::NodeHandle* nh, const char* topic, double period);
        bool publish(double rpm);
        void start_stall_emulation();
        void stop_stall_emulation();
    private:
        void estimate_state(double rpm);
        void emulate_normal_mode(double rpm);
        void emulate_stall_mode();

        ros::Publisher _status_publisher;

        double _rpm{0};
        uint8_t _state{0};
        double _stallTsMs = 0;
        double _startTsSec = 0;
};

#endif  // SRC_SENSORS_ICE_HPP
