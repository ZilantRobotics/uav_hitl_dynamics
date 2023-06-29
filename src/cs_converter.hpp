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

#ifndef SC_CONVERTER_HPP
#define SC_CONVERTER_HPP

#include <Eigen/Geometry>

namespace Converter {

Eigen::Vector3d nedToEnu(Eigen::Vector3d ned);
Eigen::Vector3d enuToNed(Eigen::Vector3d enu);

Eigen::Vector3d frdToFlu(Eigen::Vector3d frd);
Eigen::Vector3d fluToFrd(Eigen::Vector3d flu);

Eigen::Quaterniond frdNedTofluEnu(const Eigen::Quaterniond& q_frd_to_ned);
Eigen::Quaterniond fluEnuToFrdNed(const Eigen::Quaterniond& q_flu_to_enu);

}  // namespace Converter

#endif  // SC_CONVERTER_HPP
