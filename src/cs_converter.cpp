/*
 * Copyright (c) 2020-2023 RaccoonLab.
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


#include "cs_converter.hpp"
#include <Eigen/Geometry>

namespace Converter
{

/**
 * @brief Quaternion for rotation between ENU and NED frames
 *
 * NED to ENU: +PI/2 rotation about Z (Down) followed by a +PI rotation around X (old North/new East)
 * ENU to NED: +PI/2 rotation about Z (Up) followed by a +PI rotation about X (old East/new North)
 * @note Example:
 * NED_px4 = Q_ENU_TO_NED * ENU_ros
 * NED_ros = Q_ENU_TO_NED.inverse() * ENU_px4
 */
const auto Q_ENU_TO_NED = Eigen::Quaterniond(0, 0.70711, 0.70711, 0);

/**
 * @brief Quaternion for rotation between body FLU and body FRD frames
 *
 * +PI rotation around X (Forward) axis rotates from Forward, Right, Down (aircraft)
 * to Forward, Left, Up (base_link) frames and vice-versa.
 * @note Example:
 * FRD_px4 = Q_FRD_FLU * FLU_ros
 * FRD_ros = Q_FRD_FLU * FLU_px4
 */
const auto Q_FRD_FLU = Eigen::Quaterniond(0, 1, 0, 0);



Eigen::Vector3d nedToEnu(Eigen::Vector3d ned){
    return Q_ENU_TO_NED.inverse() * ned;
}
Eigen::Vector3d enuToNed(Eigen::Vector3d enu){
    return Q_ENU_TO_NED * enu;
}

Eigen::Vector3d frdToFlu(Eigen::Vector3d frd){
    return Q_FRD_FLU * frd;
}
Eigen::Vector3d fluToFrd(Eigen::Vector3d flu){
    return Q_FRD_FLU * flu;
}


Eigen::Quaterniond frdNedTofluEnu(const Eigen::Quaterniond& q_frd_to_ned){
    return Q_ENU_TO_NED * q_frd_to_ned * Q_FRD_FLU;
}

Eigen::Quaterniond fluEnuToFrdNed(const Eigen::Quaterniond& q_flu_to_enu){
    return Q_ENU_TO_NED.inverse() * q_flu_to_enu * Q_FRD_FLU;
}

}  // namespace Converter
