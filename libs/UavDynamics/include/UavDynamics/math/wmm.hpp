/*
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

#ifndef SRC_MATH_WMM_HPP
#define SRC_MATH_WMM_HPP

/**
 * @param[in] lat - degree withing [-90; +90]
 * @param[in] lon - degree withing [-180; +180]
 * @param[in] alt - meters (actually it is unused)
 * @param[out] mag_east_gauss,mag_north_gauss,mag_up_gauss - Gauses
 * @note The magnetic field strength is withing [0.2226, 0.6691] Gauss
 */
void calculateMagneticFieldStrengthGauss(const double lat, const double lon, const double alt,
                                         double& mag_east_gauss, double& mag_north_gauss, double& mag_up_gauss);

#endif  // SRC_MATH_WMM_HPP
