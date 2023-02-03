/**
 * @file cs_converter.hpp
 * @author ponomarevda96@gmail.com
 * @brief Set of convertion method from NED to ENU, from FRD to FLU and vice versa
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
